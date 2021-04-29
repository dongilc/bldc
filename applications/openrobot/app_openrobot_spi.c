/*
 * app_openrobot_spi.c
 *
 *  Created on: 20 March, 2021
 *      Author: cdi
 */

#include "commands.h"

#define SPI_BUFFER_SIZE					1024
#define PACKET_HANDLER					2		// 0:USB, 1:UART, 2:UART_P (We can use this for spi)
#define SPI_FIXED_DATA_BYTE				128
#define VESCUINO_STACK_NUM_MAX			10
#define SPI_ERROR_CNT_FOR_REBOOT		5

#ifdef USE_VESCUINO_PCB
/*
 * SPI Slave Pins
 */
// RESET = GPIOE0
#define HW_SPI_PORT_RESET		GPIOE
#define HW_SPI_PIN_RESET		0//5
// READY = GPIOD1
#define HW_SPI_PORT_SRXRD		GPIOD
#define HW_SPI_PIN_SRXRD		1
// DEBUG = GPIOD0
#define HW_SPI_PORT_DEBUG		GPIOD
#define HW_SPI_PIN_DEBUG		0

#define SPI_Rx_RDY_SET_LOW()		palClearPad(HW_SPI_PORT_SRXRD, HW_SPI_PIN_SRXRD)
#define SPI_Rx_RDY_SET_HIGH()		palSetPad(HW_SPI_PORT_SRXRD, HW_SPI_PIN_SRXRD)
#define SPI_DEBUG_LED_ON()			palClearPad(HW_SPI_PORT_DEBUG, HW_SPI_PIN_DEBUG)
#define SPI_DEBUG_LED_OFF()			palSetPad(HW_SPI_PORT_DEBUG, HW_SPI_PIN_DEBUG)
#define SPI_DEBUG_LED_TOGGLE()		palTogglePad(HW_SPI_PORT_DEBUG, HW_SPI_PIN_DEBUG)
#define SPI_ARDUINO_nRESET_LOW()	palClearPad(HW_SPI_PORT_RESET, HW_SPI_PIN_RESET)
#define SPI_ARDUINO_nRESET_HIGH()	palSetPad(HW_SPI_PORT_RESET, HW_SPI_PIN_RESET)
#endif

static unsigned char spi_tx_bytes[SPI_FIXED_DATA_BYTE];
static uint8_t spi_debug_print = 0;

#ifndef USE_CUSTOM_ABI_ENCODER_AT_SPI
static systime_t time_sys;
static thread_t *spi_read_tp = NULL;
static int spi_read_dt_us;

static THD_FUNCTION(spi_slave_read_thread, arg);
static THD_WORKING_AREA(spi_slave_read_thread_wa, 4096);
#endif

/*
 * SPI1 Baudrate Setting
 */
#define SPI_SPEED_SETTING_REG	0		//Speed 42MHz. Doesn't matter at spi_slave

/*
 * SPI1 configuration structure.
 * common setting : CPHA=0, CPOL=0, 8bits frames, MSb transmitted first.
 */
static const SPIConfig spicfg = {SPI_SLAVE, NULL, HW_SPI_PORT_NSS, HW_SPI_PIN_NSS, SPI_SPEED_SETTING_REG};

/*
 * SPI1 Peripheral Setting
 */
void spi1_peripheral_setting_slave(void)
{
	/*
	 * SPI1 I/O pins setup.
	 */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1, ENABLE);

	// SPI pin
	palSetPadMode(HW_SPI_PORT_SCK, HW_SPI_PIN_SCK, PAL_MODE_ALTERNATE(HW_SPI_GPIO_AF));     /* SCK.     */
	palSetPadMode(HW_SPI_PORT_MISO, HW_SPI_PIN_MISO, PAL_MODE_ALTERNATE(HW_SPI_GPIO_AF));// | PAL_STM32_OSPEED_HIGHEST);   /* MISO.    */
	palSetPadMode(HW_SPI_PORT_MOSI, HW_SPI_PIN_MOSI, PAL_MODE_ALTERNATE(HW_SPI_GPIO_AF));   /* MOSI.    */
	palSetPadMode(HW_SPI_PORT_NSS, HW_SPI_PIN_NSS, PAL_MODE_INPUT_PULLUP);

#ifdef USE_VESCUINO_PCB
	// Slave RX Ready - Output
	palSetPadMode(HW_SPI_PORT_SRXRD, HW_SPI_PIN_SRXRD, PAL_MODE_OUTPUT_PUSHPULL);// | PAL_STM32_OSPEED_HIGHEST); //PAL_STM32_OTYPE_OPENDRAIN
	palClearPad(HW_SPI_PORT_SRXRD, HW_SPI_PIN_SRXRD);

	// Arduino Reset Pin - Output, Default High
	palSetPadMode(HW_SPI_PORT_RESET, HW_SPI_PIN_RESET, PAL_MODE_OUTPUT_PUSHPULL);
	palSetPad(HW_SPI_PORT_RESET, HW_SPI_PIN_RESET);
#endif

#ifdef USE_DEBUG_LED_TOGGLE
	// Debug Pin - Output
	palSetPadMode(HW_SPI_PORT_DEBUG, HW_SPI_PIN_DEBUG, PAL_MODE_OUTPUT_PUSHPULL);// | PAL_STM32_OSPEED_HIGHEST); //PAL_STM32_OTYPE_OPENDRAIN
	palClearPad(HW_SPI_PORT_DEBUG, HW_SPI_PIN_DEBUG);
#endif
}

/*
 * Send COMM* commands reply - slave
 */
void comm_reply_slave_spi(unsigned char *packet_data, unsigned int packet_len)
{
	for(unsigned int i=0; i<SPI_FIXED_DATA_BYTE; i++) {
		if(i<packet_len)		spi_tx_bytes[i] = packet_data[i];
		else					spi_tx_bytes[i] = 0;
	}
}

static void send_packet_spi_slave(unsigned char *data, unsigned int len)
{
	if(spi_debug_print==1) {
		commands_printf("ready to reply comm\r\n - len:%d, \r\n - data:");
		for(unsigned int i=0; i<len; i++)
		{
			if(i%10==0)	commands_printf("\r\n");
			commands_printf("%d ", data[i]);
		}
		commands_printf("\r\n");
	}

	comm_reply_slave_spi(data, len);
}

static void send_packet_wrapper_spi_slave(unsigned char *data, unsigned int len) {
	packet_send_packet(data, len, PACKET_HANDLER);
}

static void process_packet_spi_slave(unsigned char *data, unsigned int len) {
	// Packet Receiving Part
	//commands_set_send_func(send_packet_wrapper_spi_slave);
	commands_process_packet(data, len, send_packet_wrapper_spi_slave);
}

#ifndef USE_CUSTOM_ABI_ENCODER_AT_SPI
// SPI RX Thread
static THD_FUNCTION(spi_slave_read_thread, arg) {
	(void)arg;

	chRegSetThreadName("spi1 slave rx");

	time_sys = chVTGetSystemTime();
	spi_read_tp = chThdGetSelfX();

	chThdSleepMilliseconds(1000);

/*
	// can devs encoder reset
	for(int i=0; i<VESCUINO_STACK_NUM_MAX; i++) {
		comm_can_set_encoder_reset(i);
	}
*/

	for(;;) {
		systime_t time_elapsed;
		time_elapsed = chVTTimeElapsedSinceX(time_sys);
		time_sys = chVTGetSystemTime();
		spi_read_dt_us = ST2US(time_elapsed);

		// SPI data exchange
#ifdef USE_VESCUINO_PCB
		SPI_Rx_RDY_SET_LOW();
#endif

/*
		while(spi_start_flag==0 && SPI_CS_READ()) {
			chThdSleepMilliseconds(10);
		}
		spi_start_flag = 1;
		spiExchange(&HW_SPI_DEV, SPI_FIXED_DATA_BYTE, spi_tx_bytes, spi_rx_bytes);

		if(spi_debug_print==1)	{
			debug_printf("--------------------------------------------------------\r\n");
			debug_printf("exchange done, time=%d, dt_us=%d\r\n rx_data: ", time_sys, spi_read_dt_us);
			for(int i=0; i<SPI_FIXED_DATA_BYTE; i++) debug_printf("%d ", spi_rx_bytes[i]);
			debug_printf("\r\n");
		}
		if(spi_process_flag==1) {
			if(spi_debug_print==1) debug_printf("RX Packet Error\r\n");
			spi_process_error_cnt++;

			// restart for host reconnection when error occur
			if(spi_process_error_cnt>=SPI_ERROR_CNT_FOR_REBOOT) {
				// reboot
				sys_reboot();
			}
		}
		else {
			// No error
#ifdef USE_VESCUINO_PCB
			SPI_Rx_RDY_SET_HIGH();
#endif
			spi_process_error_cnt = 0;
		}

		spi_process_flag = 1;
		spi_communication_cnt++;
		saveDataToBuffer(spi_rx_bytes, SPI_FIXED_DATA_BYTE);

		if(spi_control_flag)
		{
			loadDataProcessPacket();
		}
		else
		{
			if(spi_brake_cnt<=20)
			{
				// current brake for 2 sec
				float curr = 2.0;

				mc_interface_set_brake_current(curr);
				timeout_reset();	// needed after every 'mc_interface_set_current()'
				// can devs
				for(int i=0; i<get_number_of_can_status(); i++)	comm_can_set_current_brake(i+1, curr);

				spi_brake_cnt++;
			}

			spi_process_flag = 0;
			spi_process_error_cnt = 0;

			chThdSleepMilliseconds(10);
		}
	*/

	}
}
#endif