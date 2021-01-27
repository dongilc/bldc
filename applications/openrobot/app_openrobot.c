/*
	Copyright 2021 Dongil Choi	drclab2018@gmail.com

	This file is part of the OpenRobot Custom App.

	The OpenRobot Custom App is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    The OpenRobot Custom App is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
    */

#include "app.h"
#include "ch.h"
#include "hal.h"

// Some useful includes
#include "mc_interface.h"
#include "mcpwm_foc.h"	//openrobot
#include "utils.h"
#include "encoder.h"
#include "terminal.h"
#include "comm_can.h"
#include "hw.h"
#include "commands.h"
#include "timeout.h"
#include "timer.h"
#include "buffer.h"

#include <math.h>
#include <string.h>
#include <stdio.h>

// 
#define USE_DPS_DT_PRINT			false
#define USE_POSITION_DEBUG_PRINT	true
#define USE_COMM_SET_DEBUG_PRINT	false
#define VESC_NUM_MAX				10
#define SPI_FIXED_DATA_BYTE			255
#define DPS_DT						0.0001		// 10khz
#define DPS_Vmax					25000.0		// default:25000.0, Maximum Value Cal.: max 58000 erpm / 12 polepair = 4800 rpm * 6 = 29000 dps
#define DPS_Amax					100000.0	// default:100000.0
#define DPS_CONTINUOUS_TIMEOUT		0.5			// dps control disabled when there is no continuous data for 0.5sec
#define GOTO_KP_DEFAULT				7.0

// Threads
static THD_FUNCTION(openrobot_thread, arg);
static THD_WORKING_AREA(openrobot_thread_wa, 2048);
static THD_FUNCTION(dps_control_thread, arg);
static THD_WORKING_AREA(dps_control_thread_wa, 1024);

// Private functions
void app_openrobot_set_dps(float d, float s, int c_mode);
void app_openrobot_set_dps_vmax(float Vmax);
void app_openrobot_set_dps_amax(float Amax);
void app_openrobot_set_goto(float g_t, int c_mode);
float app_openrobot_goto_controller(void);
void app_openrobot_control_enable(void);

static void terminal_show_eeprom_conf(int argc, const char **argv);
static void terminal_show_openrobot_conf(int argc, const char **argv);
static void terminal_show_position_now(int argc, const char **argv);
static void terminal_cmd_custom_app_mode_select(int argc, const char **argv);
static void terminal_cmd_custom_can_terminal_resistor(int argc, const char **argv);
static void terminal_cmd_custom_dps_control(int argc, const char **argv);
static void terminal_cmd_custom_goto_control(int argc, const char **argv);
static void terminal_cmd_custom_goto_zero_pos(int argc, const char **argv);
static void terminal_cmd_custom_set_zero_pos_now(int argc, const char **argv);
static void terminal_cmd_custom_motor_release(int argc, const char **argv);
static void terminal_cmd_custom_set_param(int argc, const char **argv);
static void terminal_cmd_reboot(int argc, const char **argv);

// Private variables
static volatile bool stop_now = true;
static volatile bool is_running = false;
static volatile uint8_t app_mode;
static volatile bool can_term_res;
static volatile float dt_rt = DPS_DT;
static volatile float Vel_maximum = DPS_Vmax;
static volatile float Acc_maximum = DPS_Amax;
static volatile float Goto_Kp = GOTO_KP_DEFAULT;
static volatile float Zero_Pos;
static volatile float v_prof;
static volatile float s_prof;
static volatile float deg_ref;
static volatile float dps_target;
static volatile float dps_now;
static volatile float dps_duration_sec;
static volatile float goto_target;
static volatile int control_mode = 0;
static volatile int control_set = 0;
static volatile uint32_t dps_cnt = 0;

typedef enum {
	NONE = 0,
	DPS_CONTROL_CONTINUOUS,
	DPS_CONTROL_DURATION,
	GOTO_CONTROL,
	RELEASE
} CONTROL_MODE;

typedef enum {
	EEPROM_ADDR_CUSTOM_VAR1 = 0,
	EEPROM_ADDR_CUSTOM_VAR2,
	EEPROM_ADDR_CUSTOM_VAR3
} EEPROM_ADDR_CUSTOM_VAR;

//
static uint8_t openrobot_host_model = 0;
static uint8_t openrobot_dps_pos_debug_print = USE_POSITION_DEBUG_PRINT;
static uint8_t openrobot_comm_set_debug_print = USE_COMM_SET_DEBUG_PRINT;

// Received data
#define TARGET_VESC_ID	255
static uint8_t active_can_devs[10];
static uint8_t vesc_id[VESC_NUM_MAX] = {0,};
static uint8_t comm_set[VESC_NUM_MAX] = {0,};
static float value_set[VESC_NUM_MAX] = {0.,};
can_status_msg *can_st_msg;
can_status_msg_2 *can_st_msg_2;
can_status_msg_3 *can_st_msg_3;
can_status_msg_4 *can_st_msg_4;
can_status_msg_5 *can_st_msg_5;

typedef enum  {
	UNKNOWN = 0,
	ARDUINO_MEGA,
	ARDUINO_DUE,
	ARDUINO_TEENSY_32,
	ARDUINO_TEENSY_36,
	USB
} OPENROBOT_HOST_TYPE;

typedef enum  {
	CAN_MASTER_REPLY_MODE = 80,
	CAN_STATUS_REPLY_MODE
} OPENROBOT_CUSTOM_REPLY_MODE;

typedef enum  {
	COMM_SET_RELEASE = 100,
	COMM_SET_DPS,
	COMM_SET_DPS_VMAX,
	COMM_SET_DPS_AMAX,
	COMM_SET_GOTO
} COMM_PACKET_ID_OPENROBOT;

// MODE_FLAGs can be specified up to 8(4bit*8=32bit).
typedef enum {
	U32_APP_SELECT = 0,
	U32_CAN_TERMINAL_RESISTOR_MODE_SELECT
} CUSTOM_VAR1_U32;

// 16(4bit) selection
typedef enum {
	APP_VESCular = 0,
	APP_VESCuino
} APP_SELECT;

// 16(4bit) selection
typedef enum {
	CAN_TERMINAL_RESISTOR_OFF = 0,
	CAN_TERMINAL_RESISTOR_ON
} CAN_TERMINAL_RESISTOR_MODE;

eeprom_var eeprom_custom_var1;
eeprom_var eeprom_custom_var2;
eeprom_var eeprom_custom_var3;

// return flag value(4bit) from eeprom_custom_var1.as_u32
uint8_t app_custom_get_eeprom_custom_var1_u32(uint8_t flag_num) {
	return (eeprom_custom_var1.as_u32 >> (flag_num*4)) & 0x0000000F;
}
// set flag value(4bit) to eeprom_custom_var1.as_u32
void app_custom_set_eeprom_custom_var1_u32(uint8_t flag_num, uint8_t value) {
	uint32_t temp;
	temp = (uint32_t)((value&0x0F) << (flag_num*4));
	eeprom_custom_var1.as_u32 = eeprom_custom_var1.as_u32 | temp;
	conf_general_store_eeprom_var_hw(&eeprom_custom_var1, EEPROM_ADDR_CUSTOM_VAR1);
}

// return float value from eeprom_custom_var1.float
float app_custom_get_eeprom_custom_var1_float(void) {
	return eeprom_custom_var1.as_float;
}
// set float value to eeprom_custom_var1.float
void app_custom_set_eeprom_custom_var1_float(float value) {
	eeprom_custom_var1.as_float = value;
	conf_general_store_eeprom_var_hw(&eeprom_custom_var1, EEPROM_ADDR_CUSTOM_VAR1);
}

// return float value from eeprom_custom_var2.float
float app_custom_get_eeprom_custom_var2_float(void) {
	return eeprom_custom_var2.as_float;
}
// set float value to eeprom_custom_var2.float
void app_custom_set_eeprom_custom_var2_float(float value) {
	eeprom_custom_var2.as_float = value;
	conf_general_store_eeprom_var_hw(&eeprom_custom_var2, EEPROM_ADDR_CUSTOM_VAR2);
}

// return float value from eeprom_custom_var3.float
float app_custom_get_eeprom_custom_var3_float(void) {
	return eeprom_custom_var3.as_float;
}
// set float value to eeprom_custom_var3.float
void app_custom_set_eeprom_custom_var3_float(float value) {
	eeprom_custom_var3.as_float = value;
	conf_general_store_eeprom_var_hw(&eeprom_custom_var3, EEPROM_ADDR_CUSTOM_VAR3);
}

// set can terminal resistor
void app_custom_can_terminal_resistor_set(bool flag) {
	// can terminal resister setting
	palSetPadMode(GPIOC, 13, PAL_MODE_OUTPUT_PUSHPULL | PAL_STM32_OSPEED_HIGHEST);
	if(flag == true) palSetPad(GPIOC, 13);
	else 			 palClearPad(GPIOC, 13);
}

/*
// CAN Forward Command
void app_custom_comm_can_set_dps(uint8_t controller_id, float dps) {
	int32_t send_index = 0;
	uint8_t buffer[4];
	buffer_append_int32(buffer, (int32_t)(dps * 1000.0), &send_index);
	comm_can_transmit_eid(controller_id |
			((uint32_t)CAN_PACKET_SET_DPS << 8), buffer, send_index);
}*/

// Called when the custom application is started. Start our
// threads here and set up callbacks.
void app_custom_start(void) {
	// OpenRobot Thread start
	stop_now = false;
	chThdCreateStatic(openrobot_thread_wa, sizeof(openrobot_thread_wa),
					NORMALPRIO, openrobot_thread, NULL);
	commands_printf("app_openrobot started");	

	// Terminal commands for the VESC Tool terminal can be registered.
	terminal_register_command_callback(
			"or_eep",
			"Show EEPROM stored configuration.",
			"", terminal_show_eeprom_conf);
	
	terminal_register_command_callback(
			"or_conf",
			"Show OpenRobot App current configuration.",
			"", terminal_show_openrobot_conf);

	terminal_register_command_callback(
			"or_pos",
			"Show the encoder position value now",
			"", terminal_show_position_now);

	terminal_register_command_callback(
			"or_app",
			"Print the number d, 0=VESCular, 1=VESCuino",
			"[d]", terminal_cmd_custom_app_mode_select);

	terminal_register_command_callback(
			"or_can",
			"Print the number d, 0=off, 1=on",
			"[d]", terminal_cmd_custom_can_terminal_resistor);

	terminal_register_command_callback(
			"or_dps",
			"Print the dps[degree/sec] command value",
			"[dps] [sec]", terminal_cmd_custom_dps_control);
	
	terminal_register_command_callback(
			"or_goto",
			"Print the degree command value",
			"[deg]", terminal_cmd_custom_goto_control);

	terminal_register_command_callback(
			"or_gz",
			"Goto Zero Position",
			"", terminal_cmd_custom_goto_zero_pos);

	terminal_register_command_callback(
			"or_sz",
			"Set current Position as Zero Position",
			"", terminal_cmd_custom_set_zero_pos_now);

	terminal_register_command_callback(
			"or_re",
			"Release motor position control",
			"", terminal_cmd_custom_motor_release);

	terminal_register_command_callback(
			"or_par",
			"Print the dps control Vmax and Amax value",
			"[Vmax] [Amax]", terminal_cmd_custom_set_param);

	terminal_register_command_callback(
			"or_rb",
			"vesc will be rebooted",
			"", terminal_cmd_reboot);
	
}

// Called when the custom application is stopped. Stop our threads
// and release callbacks.
void app_custom_stop(void) {
	mc_interface_set_pwm_callback(0);

	terminal_unregister_callback(terminal_show_eeprom_conf);
	terminal_unregister_callback(terminal_show_openrobot_conf);	
	terminal_unregister_callback(terminal_show_position_now);	
	terminal_unregister_callback(terminal_cmd_custom_app_mode_select);
	terminal_unregister_callback(terminal_cmd_custom_can_terminal_resistor);
	terminal_unregister_callback(terminal_cmd_custom_dps_control);
	terminal_unregister_callback(terminal_cmd_custom_goto_control);
	terminal_unregister_callback(terminal_cmd_custom_goto_zero_pos);
	terminal_unregister_callback(terminal_cmd_custom_set_zero_pos_now);	
	terminal_unregister_callback(terminal_cmd_custom_motor_release);
	terminal_unregister_callback(terminal_cmd_custom_set_param);
	terminal_unregister_callback(terminal_cmd_reboot);

	stop_now = true;
	while (is_running) {
		chThdSleepMilliseconds(1);
	}
	commands_printf("app_openrobot stopped");
}

void app_custom_configure(app_configuration *conf) {
	(void)conf;
}

// Callback function for the terminal command with arguments.
// Terminal command to show configuration
static void terminal_show_eeprom_conf(int argc, const char **argv)
{
	(void)argc;
	(void)argv;

	// print eeprom stored configuration
	commands_printf("OpenRobot App, EEPROM Stored Configuration Values:");
	commands_printf("  openrobot app mode: %d", 
		app_custom_get_eeprom_custom_var1_u32(U32_APP_SELECT));
	commands_printf("  can terminal resister on: %d", 
		app_custom_get_eeprom_custom_var1_u32(U32_CAN_TERMINAL_RESISTOR_MODE_SELECT));
	commands_printf("  dps control Vmax: %.1f", 
		(double)app_custom_get_eeprom_custom_var1_float());
	commands_printf("  dps control Amax: %.1f", 
		(double)app_custom_get_eeprom_custom_var2_float());	
	commands_printf("  zero position: %.3fdeg", 
		(double)app_custom_get_eeprom_custom_var3_float());
	commands_printf("");
}

static void terminal_show_openrobot_conf(int argc, const char **argv)
{
	(void)argc;
	(void)argv;

	// print eeprom stored configuration
	commands_printf("OpenRobot App, current Configuration Values:");
	commands_printf("  openrobot app mode: %d", 
		app_mode);
	commands_printf("  can terminal resister on: %d", 
		(uint8_t)can_term_res);
	commands_printf("  motor control mode: %d (0:NONE, DPS_CONTROL_CONTINUOUS, DPS_CONTROL_DURATION, GOTO_CONTROL, RELEASE), control set: %d", 
		(uint8_t)control_mode, (uint8_t)control_set);
	commands_printf("  dps control Vmax: %.1f", 
		(double)Vel_maximum);
	commands_printf("  dps control Amax: %.1f", 
		(double)Acc_maximum);
	commands_printf("  zero position: %.3fdeg", 
		(double)Zero_Pos);
	commands_printf("");
}

static void terminal_show_position_now(int argc, const char **argv)
{
	(void)argc;
	(void)argv;

	// print position data
	commands_printf("Position Data:");
	commands_printf("  encoder is configured: %d", encoder_is_configured());
	commands_printf("  accum. deg now: %.2f", (double)mcpwm_foc_get_pid_pos_now());
	commands_printf("  accum. deg target: %.2f", (double)deg_ref);
	commands_printf("");
}

static void terminal_cmd_custom_app_mode_select(int argc, const char **argv) {
	if (argc == 2) {
		int d = -1;
		sscanf(argv[1], "%d", &d);

		if(d>=0 && d<=15) {
			if(d==APP_VESCular) commands_printf("APP Mode: VESCular\n");
			else if(d==APP_VESCuino) commands_printf("APP Mode: VESCuino\n");
			else commands_printf("Select 0 ~ 15 to select App Mode.\n");
			// storing current setting to eeprom
			app_custom_set_eeprom_custom_var1_u32(U32_APP_SELECT, d);
		}
	} else commands_printf("This command requires one argument.\n");
}

static void terminal_cmd_custom_can_terminal_resistor(int argc, const char **argv) {
	if (argc == 2) {
		int d = -1;
		sscanf(argv[1], "%d", &d);

		if(d == 0 || d == 1) {
			app_custom_can_terminal_resistor_set(d);
			if(d==1) commands_printf("Can Terminal Resistor On\n");
			else     commands_printf("Can Terminal Resistor Off\n");
			// storing current setting to eeprom
			app_custom_set_eeprom_custom_var1_u32(U32_CAN_TERMINAL_RESISTOR_MODE_SELECT, d);
		}
		else commands_printf("Select 0 or 1 to control CAN terminal resistor.\n");	
	} else commands_printf("This command requires one argument.\n");
}

static void terminal_cmd_custom_dps_control(int argc, const char **argv) {
	if (argc == 3) {
		float dps = -1;
		float sec = -1;
		sscanf(argv[1], "%f", &dps);
		sscanf(argv[2], "%f", &sec);

		if(encoder_is_configured()) {
			if(fabs(dps) <= (double)DPS_Vmax) {
				commands_printf("[dps control run] %.2fdps, duration:%.2fsec", (double)dps, (double)sec);
				app_openrobot_set_dps(dps, sec, DPS_CONTROL_DURATION);
			} else commands_printf("Invalid DPS value\n");
		} else commands_printf("Encoder is not configured yet\n");	
	} else commands_printf("This command requires two argument.\n");
}

static void terminal_cmd_custom_goto_control(int argc, const char **argv) {
	if (argc == 2) {
		float deg = -1;
		sscanf(argv[1], "%f", &deg);

		if(encoder_is_configured()) {
			commands_printf("[goto control run] target:%.2fdeg, now:%.2f", (double)deg, (double)mcpwm_foc_get_pid_pos_now());
			app_openrobot_set_goto(deg, GOTO_CONTROL);
		} else commands_printf("Encoder is not configured yet\n");	
	} else commands_printf("This command requires one argument.\n");
}

static void terminal_cmd_custom_goto_zero_pos(int argc, const char **argv) {
	(void)argc;
	(void)argv;

	float enc_deg_now = app_custom_get_eeprom_custom_var3_float();
	if(encoder_is_configured()) {
		commands_printf("[goto zero position] target:%.2fdeg, now:%.2f", (double)enc_deg_now, (double)mcpwm_foc_get_pid_pos_now());
		goto_target = enc_deg_now;
		control_mode = GOTO_CONTROL;
	} else commands_printf("Encoder is not configured yet\n");
}

static void terminal_cmd_custom_set_zero_pos_now(int argc, const char **argv) {
	(void)argc;
	(void)argv;

	// Caution! in openrobot app, we use mcpwm_foc_get_pid_pos_now() instead of mc_interface_get_pid_pos_now() to get encoder position.
	float enc_deg_now = mcpwm_foc_get_pid_pos_now();
	// storing current setting to eeprom
	app_custom_set_eeprom_custom_var3_float(enc_deg_now);
	commands_printf("Set current Actuator Position %.3f as Zero Position\n", (double)enc_deg_now);
}

static void terminal_cmd_custom_motor_release(int argc, const char **argv) {
	(void)argc;
	(void)argv;

	control_mode = RELEASE;
	commands_printf("[release motor]");
	//mcpwm_foc_print_pos_accum_stored();
}

static void terminal_cmd_custom_set_param(int argc, const char **argv) {
	if (argc == 3) {
		float Vmax = -1;
		float Amax = -1;
		sscanf(argv[1], "%f", &Vmax);
		sscanf(argv[2], "%f", &Amax);

		app_openrobot_set_dps_vmax(Vmax);
		app_openrobot_set_dps_amax(Amax);	
	} else commands_printf("This command requires two argument.\n");
}

static void terminal_cmd_reboot(int argc, const char **argv)
{
	(void)argc;
	(void)argv;

	// print eeprom stored configuration
	commands_printf("rebooting...");
	chThdSleepMilliseconds(500);	// sleep 0.5sec

	// reboot
	__disable_irq();
	for(;;){};
}

uint8_t get_number_of_can_status(void)
{
	uint8_t id_num = 0;
	for (int i = 0; i<CAN_STATUS_MSGS_TO_STORE; i++)
	{
		can_status_msg *msg = comm_can_get_status_msg_index(i);
		if (msg->id >= 0 && UTILS_AGE_S(msg->rx_time) < 1.0) {
			id_num++;
			active_can_devs[i] = msg->id;
		}
	}

	return id_num;
}

static void send_openrobot_app_data(unsigned char *data, unsigned int len) {
	(void)len;

	int32_t ind = 0;
	uint8_t num_of_vesc = 0;

	if(openrobot_comm_set_debug_print==1) {
		commands_printf("custom rx done. len=%d\r\n", len);
		commands_printf("data:");
		for(unsigned int i=0; i<len; i++) commands_printf("%d ",data[i]);
	}

	// Rx Part
	openrobot_host_model = data[ind++];
	if(openrobot_host_model!=UNKNOWN)
	{
		//
		num_of_vesc = data[ind++];
		for(int i=0; i<num_of_vesc; i++)
		{
			vesc_id[i] = data[ind++];
			comm_set[i] = data[ind++];

			if(vesc_id[i]==TARGET_VESC_ID)	// Local VESC ID assumes to be 255
			{
				// Local
				switch(comm_set[i]) {
					case COMM_SET_DUTY:
						value_set[i] = (float)buffer_get_int32(data, &ind) / 100000.0;
						mc_interface_set_duty(value_set[i]);
						timeout_reset();
						break;
					case COMM_SET_CURRENT:
						value_set[i] = (float)buffer_get_int32(data, &ind) / 1000.0;
						mc_interface_set_current(value_set[i]);
						timeout_reset();
						break;
					case COMM_SET_CURRENT_BRAKE:
						value_set[i] = (float)buffer_get_int32(data, &ind) / 1000.0;
						mc_interface_set_brake_current(value_set[i]);
						timeout_reset();
						break;
					case COMM_SET_RPM:
						value_set[i] = (float)buffer_get_int32(data, &ind);
						mc_interface_set_pid_speed(value_set[i]);
						timeout_reset();
						break;
					case COMM_SET_POS:
						value_set[i] = (float)buffer_get_int32(data, &ind) / 1000000.0;
						mc_interface_set_pid_pos(value_set[i]);
						timeout_reset();
						break;
					case COMM_SET_DPS:
						value_set[i] = (float)buffer_get_int32(data, &ind) / 1000.0;
						app_openrobot_set_dps(value_set[i], 0, DPS_CONTROL_CONTINUOUS);
						break;
					case COMM_SET_DPS_VMAX:
						value_set[i] = (float)buffer_get_int32(data, &ind) / 1000.0;
						app_openrobot_set_dps_vmax(value_set[i]);
						break;
					case COMM_SET_DPS_AMAX:
						value_set[i] = (float)buffer_get_int32(data, &ind) / 1000.0;
						app_openrobot_set_dps_amax(value_set[i]);
						break;
					case COMM_SET_GOTO:
						value_set[i] = (float)buffer_get_int32(data, &ind) / 1000.0 ;
						app_openrobot_set_goto(value_set[i], GOTO_CONTROL);
						break;
					case COMM_SET_RELEASE:
						ind += 4;
						control_mode = RELEASE;
						break;
					default:
					    ind += 4;
						//spi_comm_set_index[i] = -1;	//error
						break;
				}
			}
			else
			{
				// CAN DEVs Messages are repacked as CUSTOM_APP_DATA and send it to each can devs again.
				int32_t k = 0;
				uint8_t send_buffer[10];
				send_buffer[k++] = COMM_CUSTOM_APP_DATA;	// CAN forward message is repacked as CUSTOM_APP_DATA again
				send_buffer[k++] = openrobot_host_model;
				send_buffer[k++] = 1;				// obiously 1
				send_buffer[k++] = TARGET_VESC_ID;	// forwarded CAN dev is set as TARGET_VESC_ID
				send_buffer[k++] = comm_set[i];
				for(int n=0; n<4; n++) send_buffer[k++] = data[ind++];
				comm_can_send_buffer(vesc_id[i], send_buffer, k, 0);	// set send as 0 : packet goes to the commands_process_packet
			}
		}
	}

	// Reply Part - custom msg return values
	ind = 0;
	uint8_t send_buffer[SPI_FIXED_DATA_BYTE] = {0,};
	send_buffer[ind++] = COMM_CUSTOM_APP_DATA;	// +1

	// can dev numbers
	uint8_t can_devs_num = 0;
	can_devs_num = get_number_of_can_status();
	send_buffer[ind++] = can_devs_num;	// +1

	// common return value
	send_buffer[ind++] = app_get_configuration()->controller_id;	// +1
	buffer_append_float16(send_buffer, GET_INPUT_VOLTAGE(), 1e1, &ind); // +2
	buffer_append_float16(send_buffer, mc_interface_temp_fet_filtered(), 1e1, &ind); // +2
	buffer_append_float16(send_buffer, mc_interface_temp_motor_filtered(), 1e1, &ind); // +2
	buffer_append_float16(send_buffer, mc_interface_read_reset_avg_motor_current(), 1e2, &ind); // +2
	buffer_append_float16(send_buffer, mc_interface_read_reset_avg_input_current(), 1e2, &ind); // +2
	buffer_append_float16(send_buffer, mc_interface_get_duty_cycle_now(), 1e3, &ind); // +2
	buffer_append_float32(send_buffer, mc_interface_get_watt_hours(false), 1e4, &ind); // +4
	buffer_append_float32(send_buffer, mc_interface_get_watt_hours_charged(false), 1e4, &ind); // +4
	buffer_append_float32(send_buffer, mcpwm_foc_get_pid_pos_now(), 1e2, &ind); // +4
	buffer_append_float16(send_buffer, mcpwm_foc_get_rpm(), 1e2, &ind); // +2

	// from can_status_msgs
	for(int i=0; i<can_devs_num; i++)
	{
		uint8_t can_id = active_can_devs[i];
		can_st_msg = comm_can_get_status_msg_id(can_id);		// +1
		can_st_msg_2 = comm_can_get_status_msg_2_id(can_id);	// +1
		can_st_msg_3 = comm_can_get_status_msg_3_id(can_id);	// +1
		can_st_msg_4= comm_can_get_status_msg_4_id(can_id);		// +1
		can_st_msg_5 = comm_can_get_status_msg_5_id(can_id);	// +1

		if(can_st_msg!=0) // +27 byte
		{
			send_buffer[ind++] = can_id;
			buffer_append_float16(send_buffer, can_st_msg_5->v_in, 1e1, &ind); // +2
			buffer_append_float16(send_buffer, can_st_msg_4->temp_fet, 1e1, &ind); // +2
			buffer_append_float16(send_buffer, can_st_msg_4->temp_motor, 1e1, &ind); // +2
			buffer_append_float16(send_buffer, can_st_msg->current, 1e2, &ind); // +2
			buffer_append_float16(send_buffer, can_st_msg_4->current_in, 1e2, &ind); // +2
			buffer_append_float16(send_buffer, can_st_msg->duty, 1e3, &ind); // +2
			buffer_append_float32(send_buffer, can_st_msg_3->watt_hours, 1e4, &ind); // +4	
			buffer_append_float32(send_buffer, can_st_msg_3->watt_hours_charged, 1e4, &ind); // +4	
			buffer_append_float32(send_buffer, can_st_msg_4->pid_pos_now, 1e2, &ind); // +4
			buffer_append_float16(send_buffer, can_st_msg->rpm, 1e2, &ind); // +2
		}
	}
	commands_send_packet(send_buffer, ind);
}

// OpenRobot Thread
static THD_FUNCTION(openrobot_thread, arg) {
	(void)arg;

	chRegSetThreadName("App OpenRobot");

	is_running = true;

	// when the firmware is flashed, eeprom stored values are initialized as all zero.
	// VESC-Tool's ConfBackup works only when you use backup data after the firmware is flashed.
	// read stored setting
	conf_general_read_eeprom_var_hw(&eeprom_custom_var1, EEPROM_ADDR_CUSTOM_VAR1);
	conf_general_read_eeprom_var_hw(&eeprom_custom_var2, EEPROM_ADDR_CUSTOM_VAR2);
	conf_general_read_eeprom_var_hw(&eeprom_custom_var3, EEPROM_ADDR_CUSTOM_VAR3);

	// set using stored configuration
	app_mode = app_custom_get_eeprom_custom_var1_u32(U32_APP_SELECT);
	can_term_res = app_custom_get_eeprom_custom_var1_u32(U32_CAN_TERMINAL_RESISTOR_MODE_SELECT);
	if(app_custom_get_eeprom_custom_var1_float()!=0) Vel_maximum = app_custom_get_eeprom_custom_var1_float();
	if(app_custom_get_eeprom_custom_var2_float()!=0) Acc_maximum = app_custom_get_eeprom_custom_var2_float();
	Zero_Pos = app_custom_get_eeprom_custom_var3_float();
	
	//
	if(app_mode == APP_VESCular) {
		// Start uart communication: use UART:VESC-Tool, USB:ROS
		app_uartcomm_start();	

		// set custom app as openrobot app, To set the RX function.
		commands_set_app_data_handler(send_openrobot_app_data);

		chThdCreateStatic(dps_control_thread_wa, sizeof(dps_control_thread_wa), 
					NORMALPRIO, dps_control_thread, NULL);
	}
	app_custom_can_terminal_resistor_set((bool)can_term_res);

	for(;;) {
		// Check if it is time to stop.
		if (stop_now) {
			is_running = false;
			return;
		}

		//timeout_reset(); // Reset timeout if everything is OK.

		// Run your logic here. A lot of functionality is available in mc_interface.h.
		
		chThdSleepMilliseconds(10);
	}
}

//
float app_openrobot_genProfile(float v_ref, float Amax, float dt)
{
	float da = 0;
	float dv = 0;
	float ds = 0;

	// Trapezoidal Velocity Profile (incremental inplementation)
	if(v_ref == v_prof) {
		dv = 0;
	}
	else {
		da = (v_ref - v_prof)/dt;
		if(fabs(da) >= (double)Amax) {
			if(da>0) da = Amax;
			else 	 da = -Amax;
		}
	}
	dv = da*dt;
	ds = v_prof*dt + 0.5*dv*dt;

	v_prof += dv;
	s_prof += ds;

	return s_prof;
}

void app_openrobot_set_dps(float d, float s, int c_mode)
{
	dps_target = d;
	dps_duration_sec = s;
	control_mode = c_mode;

	if(control_mode==DPS_CONTROL_CONTINUOUS) dps_cnt = 0;
}

void app_openrobot_set_dps_vmax(float Vmax)
{
	if(Vmax>0 && Vmax<=DPS_Vmax) {
		app_custom_set_eeprom_custom_var1_float(Vmax);
		Vel_maximum = Vmax;
		commands_printf("Set Vmax: %.2f", (double)Vel_maximum);
	} else commands_printf("Invalid Vmax Value (input Vmax from 0 ~ %.1f)\n", (double)DPS_Vmax);
}

void app_openrobot_set_dps_amax(float Amax)
{
	if(Amax>0 && Amax<=DPS_Amax) {
		app_custom_set_eeprom_custom_var2_float(Amax);
		Acc_maximum = Amax;
		commands_printf("Set Amax: %.2f", (double)Acc_maximum);
	} else commands_printf("Invalid Vmax Value (input Amax from 0 ~ %.1f)\n", (double)DPS_Amax);
}

void app_openrobot_set_goto(float g_t, int c_mode)
{
	goto_target = g_t;
	control_mode = c_mode;
}

void app_openrobot_control_enable(void)
{
	// run this every first connection of dps control
	if(control_set==0) {
		s_prof = deg_ref = mcpwm_foc_get_pid_pos_now();
	}
	control_set = 1;
}

float app_openrobot_goto_controller(void)
{
	float dps_goto;
	float dps_goto_err;
	
	dps_goto_err = goto_target - mcpwm_foc_get_pid_pos_now();

	dps_goto = Goto_Kp*dps_goto_err; // 7:optimal, 8:overshoot little, 6.:no overshoot
	if(fabs(dps_goto) >= (double)Vel_maximum) {
		if(dps_goto>0)	dps_goto = Vel_maximum;
		else 	 		dps_goto = -Vel_maximum;
	}

	return dps_goto;
}

// DPS Control Thread
static THD_FUNCTION(dps_control_thread, arg) {
	(void)arg;

	chRegSetThreadName("dps_control_thread");	// default 10kHz

	// time variables
	static systime_t time_start;
	static systime_t time_prev;
	static systime_t time_duration;
	uint32_t duration;
	uint32_t time_cnt = 0;
	uint32_t debug_cnt = 0;

	for(;;) {
		// Timer implementation
		time_prev = time_start;
		time_start = chVTGetSystemTime();
		time_duration = time_start - time_prev;
		duration = ST2US(time_duration);	// usec
	 	dt_rt = (float)(duration/1000000.);	// sec, realtime calcuation

		switch(control_mode) {
		case RELEASE: {
			mc_interface_release_motor();
			v_prof = 0.;
			dps_cnt = 0;
			dps_target = 0.;
			dps_duration_sec = 0;
			control_set = 0;
			control_mode = NONE;
		} break;

		// This is for ROS
		case DPS_CONTROL_CONTINUOUS: {
			if(dps_cnt/10000. <= DPS_CONTINUOUS_TIMEOUT) {
				app_openrobot_control_enable();
			}
			else {			
				control_mode = RELEASE;
			}
			
			deg_ref = app_openrobot_genProfile(dps_target, (float)Acc_maximum, dt_rt);
			dps_cnt++;
		} break;

		// This is for VESC-Tool
		case DPS_CONTROL_DURATION: {
			if(dps_cnt/10000. <= dps_duration_sec) {
				app_openrobot_control_enable();
			} else {
				control_mode = RELEASE;
			}
			deg_ref = app_openrobot_genProfile(dps_target, (float)Acc_maximum, dt_rt);
			dps_cnt++;
		} break;

		case GOTO_CONTROL: {
			dps_target = app_openrobot_goto_controller();
			app_openrobot_control_enable();
			// USE maximum Acceleration value at GOTO_Control, Speed regulated by Vmax at GOTO_Contrl
			deg_ref = app_openrobot_genProfile(dps_target, (float)DPS_Amax, dt_rt);	
		} break;

		default:
			break;
		}

		// run position control
		if(control_set==1) 	{
			mcpwm_foc_set_pos_accum(deg_ref);	
			timeout_reset();
		}

		// print dt for debugging, every 1.0sec
		if(debug_cnt>=10000 && control_set==1) {
			debug_cnt = 0;
			if(USE_DPS_DT_PRINT) 	commands_printf("\r\n> dps_control, dt:%d (usec)", duration);
			if(openrobot_dps_pos_debug_print)	{
				commands_printf("  id:%d, t:%.2fsec, t_to:%.2fsec, ang now:%.2fdeg, ang target:%.2fdeg", 
					app_get_configuration()->controller_id, (double)(time_cnt/10000.), (double)(dps_duration_sec - dps_cnt/10000.), (double)mcpwm_foc_get_pid_pos_now(), (double)deg_ref);
			}
		}

		// Loop Time Manage
		debug_cnt++;
		time_cnt++;
		chThdSleepMicroseconds(100);	// 10khz
	}
}