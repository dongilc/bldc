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
#include "comm_can.h"
#include "comm_usb_serial.h"
//openrobot
#include "packet.h"
#include "app_openrobot_util.c"
#include "app_openrobot_spi.c"

#include <math.h>
#include <string.h>
#include <stdio.h>

// 
#define USE_DPS_DT_PRINT			false
#define USE_POSITION_DEBUG_PRINT	false //true
#define USE_COMM_SET_DEBUG_PRINT	false //true
#define USE_CAN_TERM_RESISTOR_ON 	true
#define VESC_NUM_MAX				10
#define RAD2DEG						180./M_PI
#define DEG2RAD						M_PI/180.
#define RPM2RPS						2.*M_PI/60.
#define RPM2DPS						6.
#define DPS_DT						0.0001		// 10khz
#define DPS_VMAX_DEFAULT			10000.0		// default:25000.0, Maximum Value Cal.: max 58000 erpm / 12 polepair = 4800 rpm * 6 = 29000 dps
#define DPS_AMAX_DEFAULT			100000.0	// default:100000.0
#define DPS_CONTINUOUS_TIMEOUT		0.5			// dps control disabled when there is no continuous data for 0.5sec
#define SERVO_KP_DEFAULT				20.0
#define SERVO_KI_DEFAULT				0.

// Threads
static THD_FUNCTION(openrobot_thread, arg);
static THD_WORKING_AREA(openrobot_thread_wa, 2048);
static THD_FUNCTION(dps_control_thread, arg);
static THD_WORKING_AREA(dps_control_thread_wa, 1024);

// Private functions
void app_openrobot_set_dps(float d, float s, int c_mode);
void app_openrobot_set_dps_vmax(float Vmax, bool flash);
void app_openrobot_set_dps_amax(float Amax, bool flash);
void app_openrobot_control_enable(void);
void app_openrobot_set_servo(float g_t, int c_mode);
void app_openrobot_set_servo_gain(float kp, float ki);
float app_openrobot_servo_controller(void);
void app_openrobot_set_traj(float g_t, int c_mode);
float app_openrobot_traj_controller(void);
float app_openrobot_Trapezoidal_Traj_Gen_Given_Vmax_and_Amax(float start, float goal, float vmax, float amax, float dt);
//int app_openrobot_Trapezoidal_Traj_Gen_Given_Vmax_and_T(float vmax, float T, float dt);
//int app_openrobot_Trapezoidal_Traj_Gen_Given_Amax_and_T(float amax, float T, float dt);
//void app_openrobot_Trapezoidal_Path_Gen(float start, float goal, int total_i);
//static float traj[3000] = {0.};

static void terminal_cmd_custom_show_eeprom_conf(int argc, const char **argv);
static void terminal_cmd_custom_show_openrobot_conf(int argc, const char **argv);
static void terminal_cmd_custom_show_position_now(int argc, const char **argv);
static void terminal_cmd_custom_show_eeprom_value(int argc, const char **argv);
static void terminal_cmd_custom_show_can_status_msg(int argc, const char **argv);
static void terminal_cmd_custom_show_debug_position_status(int argc, const char **argv);
static void terminal_cmd_custom_show_comm_set_data(int argc, const char **argv);
static void terminal_cmd_custom_app_mode_select(int argc, const char **argv);
static void terminal_cmd_custom_can_terminal_resistor(int argc, const char **argv);
static void terminal_cmd_custom_dps_control(int argc, const char **argv);
static void terminal_cmd_custom_servo_control(int argc, const char **argv);
static void terminal_cmd_custom_servo_zero_pos(int argc, const char **argv);
static void terminal_cmd_custom_traj_control(int argc, const char **argv);
static void terminal_cmd_custom_set_zero_pos_now(int argc, const char **argv);
static void terminal_cmd_custom_find_torque_constant(int argc, const char **argv);
static void terminal_cmd_custom_servo_control_exam(int argc, const char **argv);
static void terminal_cmd_custom_motor_release(int argc, const char **argv);
static void terminal_cmd_custom_realtime_plot(int argc, const char **argv);
static void terminal_cmd_custom_set_param(int argc, const char **argv);
static void terminal_cmd_custom_set_servo_gain(int argc, const char **argv);
static void terminal_cmd_reboot(int argc, const char **argv);

// Private variables
static volatile bool stop_now = true;
static volatile bool is_running = false;
static volatile bool is_find_kt_running = false;
static volatile bool is_servo_exam_running = false;
static volatile bool is_exp_plot_running = false;
static volatile uint8_t app_mode;
static volatile bool can_term_res;
static volatile float dt_rt = DPS_DT;
static volatile float Vel_maximum = DPS_VMAX_DEFAULT;
static volatile float Acc_maximum = DPS_AMAX_DEFAULT;
static volatile float servo_Kp = SERVO_KP_DEFAULT;
static volatile float servo_Ki = SERVO_KI_DEFAULT;
static volatile float Zero_Pos;
static volatile float v_prof;
static volatile float s_prof;
static volatile float deg_ref;
static volatile float dps_target;
static volatile float dps_now;
static volatile float dps_duration_sec;
static volatile float dps_error_int = 0.;
static volatile float servo_target;
static volatile float traj_time;
static volatile float traj_start;
static volatile float traj_target;
static volatile int control_mode = 0;
static volatile int control_set = 0;
static uint8_t controller_id = 0;	
static int polepair_number = 0;
static volatile uint32_t dps_cnt = 0;

typedef enum {
	NONE = 0,
	DPS_CONTROL_TIMEOUT,
	DPS_CONTROL_DURATION,
	SERVO_CONTROL,
	TRAJ_CONTROL,
	RELEASE
} CONTROL_MODE;

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
	USB,
	CAN
} OPENROBOT_HOST_TYPE;

typedef enum  {
	COMM_SET_RELEASE = 100,
	COMM_SET_DPS,
	COMM_SET_DPS_VMAX,
	COMM_SET_DPS_AMAX,
	COMM_SET_SERVO,
	COMM_SET_TRAJ
} COMM_PACKET_ID_OPENROBOT;

//////////////// EEPROM DATA ////////////////
// eeprom_var memory index. range:0~63
typedef enum {
	EEP_APP_SELECT = 0,
	EEP_CAN_TERMINAL_RESISTOR_MODE,
	EEP_VMAX,
	EEP_AMAX,
	EEP_ENC_ZERO_POS
} EEPROM_VAR_LIST;

// value range : 0~15 (4bit)
typedef enum {
	APP_VESCular = 0,
	APP_VESCuino
} APP_SELECT;

// value range : 0~15 (4bit)
typedef enum {
	CAN_TERMINAL_RESISTOR_OFF = 0,
	CAN_TERMINAL_RESISTOR_ON
} CAN_TERMINAL_RESISTOR_MODE;
//////////////// EEPROM DATA ////////////////

void app_custom_show_eeprom_var(uint8_t var_index)
{
	eeprom_var eeprom_custom_var_temp;

	// load stored value first	
	if(conf_general_read_eeprom_var_custom(&eeprom_custom_var_temp, var_index)) {
		commands_printf("var_index:%d, uint32_value:%x, int32_value:%d, float_value:%f", 
					var_index, eeprom_custom_var_temp.as_u32, eeprom_custom_var_temp.as_i32, (double)eeprom_custom_var_temp.as_float);	
	}
	else {
		commands_printf("variable index %d is not yet stored on eeprom", var_index);
	}
}

// gpio setup CAN terminal resistor
void app_custom_can_terminal_resistor_set(bool flag) {
	// can terminal resister setting
	palSetPadMode(GPIOC, 13, PAL_MODE_OUTPUT_PUSHPULL | PAL_STM32_OSPEED_HIGHEST);
	if(flag == true) palSetPad(GPIOC, 13);
	else 			 palClearPad(GPIOC, 13);

	can_term_res = flag;
}

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
			"", terminal_cmd_custom_show_eeprom_conf);
	
	terminal_register_command_callback(
			"or_conf",
			"Show OpenRobot App current configuration.",
			"", terminal_cmd_custom_show_openrobot_conf);

	terminal_register_command_callback(
			"or_pos",
			"Show the encoder position value now",
			"", terminal_cmd_custom_show_position_now);

	terminal_register_command_callback(
			"or_sev",
			"Show eeprom variable value",
			"", terminal_cmd_custom_show_eeprom_value);

	terminal_register_command_callback(
			"or_scs",
			"Show can status messages",
			"[id] [msg_type]", terminal_cmd_custom_show_can_status_msg);

	terminal_register_command_callback(
			"or_dp",
			"Show debug position control status",
			"", terminal_cmd_custom_show_debug_position_status);

	terminal_register_command_callback(
			"or_dc",
			"Show debug comm set data",
			"", terminal_cmd_custom_show_comm_set_data);

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
			"or_servo",
			"Print the degree command value",
			"[deg]", terminal_cmd_custom_servo_control);
	
	terminal_register_command_callback(
			"or_gz",
			"Go to Zero Position",
			"", terminal_cmd_custom_servo_zero_pos);

	terminal_register_command_callback(
			"or_traj",
			"Print the degree command value",
			"[deg]", terminal_cmd_custom_traj_control);

	terminal_register_command_callback(
			"or_sz",
			"Set current Position as Zero Position",
			"", terminal_cmd_custom_set_zero_pos_now);

	terminal_register_command_callback(
			"or_kt",
			"Find Motor torque constant [Nm/A]",
			"", terminal_cmd_custom_find_torque_constant);

	terminal_register_command_callback(
			"or_se",
			"Servo Control Exam",
			"", terminal_cmd_custom_servo_control_exam);

	terminal_register_command_callback(
			"or_re",
			"Release motor position control",
			"", terminal_cmd_custom_motor_release);

	terminal_register_command_callback(
			"or_rp",
			"Realtime Plot On/Off",
			"", terminal_cmd_custom_realtime_plot);

	terminal_register_command_callback(
			"or_par",
			"Print the dps control Vmax and Amax value",
			"[Vmax] [Amax]", terminal_cmd_custom_set_param);

	terminal_register_command_callback(
			"or_sg",
			"Print the servo control Kp and Ki value",
			"[Kp] [Ki]", terminal_cmd_custom_set_servo_gain);

	terminal_register_command_callback(
			"or_rb",
			"vesc will be rebooted",
			"", terminal_cmd_reboot);
	
}

// Called when the custom application is stopped. Stop our threads
// and release callbacks.
void app_custom_stop(void) {
	mc_interface_set_pwm_callback(0);

	terminal_unregister_callback(terminal_cmd_custom_show_eeprom_conf);
	terminal_unregister_callback(terminal_cmd_custom_show_openrobot_conf);	
	terminal_unregister_callback(terminal_cmd_custom_show_position_now);	
	terminal_unregister_callback(terminal_cmd_custom_show_eeprom_value);	
	terminal_unregister_callback(terminal_cmd_custom_show_can_status_msg);
	terminal_unregister_callback(terminal_cmd_custom_app_mode_select);
	terminal_unregister_callback(terminal_cmd_custom_show_debug_position_status);
	terminal_unregister_callback(terminal_cmd_custom_show_comm_set_data);
	terminal_unregister_callback(terminal_cmd_custom_can_terminal_resistor);
	terminal_unregister_callback(terminal_cmd_custom_dps_control);
	terminal_unregister_callback(terminal_cmd_custom_servo_control);
	terminal_unregister_callback(terminal_cmd_custom_servo_zero_pos);
	terminal_unregister_callback(terminal_cmd_custom_traj_control);
	terminal_unregister_callback(terminal_cmd_custom_set_zero_pos_now);	
	terminal_unregister_callback(terminal_cmd_custom_find_torque_constant);		
	terminal_unregister_callback(terminal_cmd_custom_servo_control_exam);		
	terminal_unregister_callback(terminal_cmd_custom_motor_release);
	terminal_unregister_callback(terminal_cmd_custom_realtime_plot);
	terminal_unregister_callback(terminal_cmd_custom_set_param);
	terminal_unregister_callback(terminal_cmd_custom_set_servo_gain);
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
static void terminal_cmd_custom_show_eeprom_conf(int argc, const char **argv)
{
	(void)argc;
	(void)argv;

	eeprom_var eeprom_custom_var_temp;

	// print eeprom stored configuration
	commands_printf("OpenRobot App, EEPROM Stored Configuration Values:");

	//
	if(conf_general_read_eeprom_var_custom(&eeprom_custom_var_temp, EEP_APP_SELECT)) {
		commands_printf("  openrobot app mode: %d", (uint32_t)eeprom_custom_var_temp.as_float);
	}
	else commands_printf("  openrobot app mode is not yet stored on eeprom");

	//
	if(conf_general_read_eeprom_var_custom(&eeprom_custom_var_temp, EEP_CAN_TERMINAL_RESISTOR_MODE)) {
		commands_printf("  can terminal resister on: %d", (uint32_t)eeprom_custom_var_temp.as_float);
	}
	else commands_printf("  can terminal resister is not yet stored on eeprom");

	//
	if(conf_general_read_eeprom_var_custom(&eeprom_custom_var_temp, EEP_VMAX)) {
		commands_printf("  dps control Vmax: %.1f", (double)eeprom_custom_var_temp.as_float);
	}
	else commands_printf("  dps control Vmax is not yet stored on eeprom");
	
	//
	if(conf_general_read_eeprom_var_custom(&eeprom_custom_var_temp, EEP_AMAX)) {
		commands_printf("  dps control Amax: %.1f", (double)eeprom_custom_var_temp.as_float);
	}
	else commands_printf("  dps control Amax is not yet stored on eeprom");

	//
	if(conf_general_read_eeprom_var_custom(&eeprom_custom_var_temp, EEP_ENC_ZERO_POS)) {
		commands_printf("  zero position: %.3fdeg", (double)eeprom_custom_var_temp.as_float);
	}
	else commands_printf("  zero position is not yet stored on eeprom");
	
	commands_printf("");
}

static void terminal_cmd_custom_show_openrobot_conf(int argc, const char **argv)
{
	(void)argc;
	(void)argv;

	// print eeprom stored configuration
	commands_printf("OpenRobot App, current Configuration Values:");
	commands_printf("  openrobot app mode: %d (0:VESCular, VESCuino)", 
		(uint8_t)app_mode);
	commands_printf("  can terminal resister on: %d", 
		(uint8_t)can_term_res);
	commands_printf("  motor control mode: %d (0:NONE, DPS_CONTROL_TIMEOUT, DPS_CONTROL_DURATION, SERVO_CONTROL, RELEASE), control set: %d", 
		(uint8_t)control_mode, (uint8_t)control_set);
	commands_printf("  dps control Vmax: %.1f", 
		(double)Vel_maximum);
	commands_printf("  dps control Amax: %.1f", 
		(double)Acc_maximum);
	commands_printf("  zero position: %.3fdeg", 
		(double)Zero_Pos);
	commands_printf("");
}

static void terminal_cmd_custom_show_position_now(int argc, const char **argv)
{
	(void)argc;
	(void)argv;

	const volatile mc_configuration *conf = mc_interface_get_configuration();
	const volatile int motor_dir = conf->m_invert_direction ? -1 : 1;
	const volatile int encoder_dir = conf->foc_encoder_inverted ? -1 : 1;

	// print position data
	commands_printf("Position Data:");
	commands_printf("  encoder is configured: %d", encoder_is_configured());
	commands_printf("  encoder invert direction: %d", encoder_dir);
	commands_printf("  motor invert direction: %d", motor_dir);
	commands_printf("  accum. deg now: %.2f", (double)mcpwm_foc_get_pos_accum());
	commands_printf("  accum. deg target: %.2f", (double)deg_ref);
	commands_printf("");
}

static void terminal_cmd_custom_show_eeprom_value(int argc, const char **argv)
{
	if (argc == 2) {
		int d = -1;
		sscanf(argv[1], "%d", &d);

		app_custom_show_eeprom_var(d);
	} else commands_printf("This command requires one argument\n");
}

static void terminal_cmd_custom_show_can_status_msg(int argc, const char **argv)
{
	if (argc == 3) {
		int id = -1;
		int msg_type = -1;
		sscanf(argv[1], "%d", &id);
		sscanf(argv[2], "%d", &msg_type);

		can_status_msg *stat_msg = comm_can_get_status_msg_id(id);
		if(stat_msg!=0) {
			if(msg_type==1) {
			
			commands_printf("pos = %.3f rad, vel = %.4f rad/sec, curr = %.2f A", 
							(double)stat_msg->duty, (double)stat_msg->rpm, (double)stat_msg->current);
			}
		} else {
			commands_printf("Id:%d is not sending Status Msg %d", id, msg_type);
		}
		 
	} else commands_printf("This command requires two argument, or_cs [id] [msg_type], ex) or_cs 28 1\n");
}

static void terminal_cmd_custom_show_debug_position_status(int argc, const char **argv)
{
	(void)argc;
	(void)argv;

	if(openrobot_dps_pos_debug_print == true) {
		openrobot_dps_pos_debug_print = false;
		commands_printf("[Position Control Debug Print Disabled]");
	}
	else {
		openrobot_dps_pos_debug_print = true;
		commands_printf("[Position Control Debug Print Enabled]");
	}
}

static void terminal_cmd_custom_show_comm_set_data(int argc, const char **argv)
{
	(void)argc;
	(void)argv;

	if(openrobot_comm_set_debug_print == true) {
		openrobot_comm_set_debug_print = false;
		commands_printf("[COMM_SET Data Debug Print Disabled]");
	}
	else {
		openrobot_comm_set_debug_print = true;
		commands_printf("[COMM_SET Data Debug Print Enabled]");
	}
}

static void terminal_cmd_custom_app_mode_select(int argc, const char **argv) {
	if (argc == 2) {
		int d = -1;
		sscanf(argv[1], "%d", &d);

		if(d>=0 && d<=15) {
			if(d==APP_VESCular) {
				commands_printf("APP Mode: VESCular\n");
				app_mode = d;
			}
			else if(d==APP_VESCuino) {
				commands_printf("APP Mode: VESCuino\n");
				app_mode = d;
			}
			else commands_printf("Select 0 ~ 15 to select App Mode.\n");

			// storing current setting to eeprom
			eeprom_var eeprom_custom_var_temp;
			eeprom_custom_var_temp.as_float = (float)d;
			conf_general_store_eeprom_var_custom(&eeprom_custom_var_temp, EEP_APP_SELECT);
		}
	} else commands_printf("This command requires one argument.\n");
}

static void terminal_cmd_custom_can_terminal_resistor(int argc, const char **argv) {
	if (argc == 2) {
		int d = -1;
		sscanf(argv[1], "%d", &d);

		if(d == 0 || d == 1) {
			app_custom_can_terminal_resistor_set(d);
			if(d==1) {
				commands_printf("Can Terminal Resistor On\n");
			}
			else {
				commands_printf("Can Terminal Resistor Off\n");
			}

			// storing current setting to eeprom
			eeprom_var eeprom_custom_var_temp;
			eeprom_custom_var_temp.as_float = (float)d;
			conf_general_store_eeprom_var_custom(&eeprom_custom_var_temp, EEP_CAN_TERMINAL_RESISTOR_MODE);
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
			if(fabs(dps) <= (double)DPS_VMAX_DEFAULT) {
				commands_printf("[dps control run] %.2fdps, duration:%.2fsec", (double)dps, (double)sec);
				app_openrobot_set_dps(dps, sec, DPS_CONTROL_DURATION);
			} else commands_printf("Invalid DPS value\n");
		} else commands_printf("Encoder is not configured yet\n");	
	} else commands_printf("This command requires two argument.\n");
}

static void terminal_cmd_custom_servo_control(int argc, const char **argv) {
	if (argc == 2) {
		float deg = -1;
		sscanf(argv[1], "%f", &deg);

		if(encoder_is_configured()) {
			commands_printf("[servo control run] target:%.2fdeg, now:%.2f", (double)deg, (double)mcpwm_foc_get_pos_accum());
			app_openrobot_set_servo(deg, SERVO_CONTROL);
		} else commands_printf("Encoder is not configured yet\n");	
	} else commands_printf("This command requires one argument.\n");
}

static void terminal_cmd_custom_servo_zero_pos(int argc, const char **argv) {
	(void)argc;
	(void)argv;

	float enc_deg_now = 0.;
	eeprom_var eeprom_custom_var_temp;
	if(conf_general_read_eeprom_var_custom(&eeprom_custom_var_temp, EEP_ENC_ZERO_POS)){
		enc_deg_now = eeprom_custom_var_temp.as_float;
	}
	else {
		commands_printf("EEPROM Zero position is not yet stored on eeprom, zero position is set at 0\n");
	}

	if(encoder_is_configured()) {
		commands_printf("[go to zero position] target:%.2fdeg, now:%.2f", (double)enc_deg_now, (double)mcpwm_foc_get_pos_accum());
		servo_target = enc_deg_now;
		control_mode = SERVO_CONTROL;
	} else commands_printf("Encoder is not configured yet\n");
}

static void terminal_cmd_custom_traj_control(int argc, const char **argv) {
	if (argc == 2) {
		float deg = -1;
		sscanf(argv[1], "%f", &deg);

		if(encoder_is_configured()) {
			commands_printf("[traj control run] target:%.2fdeg, now:%.2f, Vmax:%.2f", 
									(double)deg, (double)mcpwm_foc_get_pos_accum(), (double)Vel_maximum);
			if(control_mode!=TRAJ_CONTROL) {
				traj_target = deg;
				traj_start = mcpwm_foc_get_pos_accum();
				control_mode = TRAJ_CONTROL;
			}
			else commands_printf("Trajectory Controller is running...\n");
		} else commands_printf("Encoder is not configured yet\n");
	} else commands_printf("This command requires one argument.\n");
}

static void terminal_cmd_custom_set_zero_pos_now(int argc, const char **argv) {
	(void)argc;
	(void)argv;

	if (argc == 1) {
		// Caution! in openrobot app, we use mcpwm_foc_get_pos_accum() to get encoder position.
		float enc_deg_now = mcpwm_foc_get_pos_accum();
		Zero_Pos = enc_deg_now;

		// storing current setting to eeprom
		eeprom_var eeprom_custom_var_temp;
		eeprom_custom_var_temp.as_float = enc_deg_now;
		conf_general_store_eeprom_var_custom(&eeprom_custom_var_temp, EEP_ENC_ZERO_POS);

		commands_printf("Set current Actuator Position %.3f as Zero Position\n", (double)enc_deg_now);
	}
	else commands_printf("This command requires no argument.\n");
}

static void terminal_cmd_custom_find_torque_constant(int argc, const char **argv) {
	(void)argc;
	(void)argv;

	// Caution! motor will rotate for 5 sec
	is_find_kt_running = true;
}

static void terminal_cmd_custom_servo_control_exam(int argc, const char **argv) {
	(void)argc;
	(void)argv;

	// Caution! motor will rotate for 5 sec
	is_servo_exam_running = true;
}

static void terminal_cmd_custom_motor_release(int argc, const char **argv) {
	(void)argc;
	(void)argv;

	control_mode = RELEASE;
	commands_printf("[release motor]");
	//commands_printf("Debugging print initializing values of 'pos_accum'");
	//mcpwm_foc_print_pos_accum_stored();
}

static void terminal_cmd_custom_realtime_plot(int argc, const char **argv) {
	(void)argc;
	(void)argv;

	if(is_exp_plot_running == true) {
		is_exp_plot_running = false;
		commands_printf("[Realtime Plot Disabled]");
	}
	else {
		is_exp_plot_running = true;
		commands_printf("[Realtime Plot Enabled]");
	}
}

static void terminal_cmd_custom_set_param(int argc, const char **argv) {
	if (argc == 3) {
		float Vmax = -1;
		float Amax = -1;
		sscanf(argv[1], "%f", &Vmax);
		sscanf(argv[2], "%f", &Amax);

		app_openrobot_set_dps_vmax(Vmax, true);
		app_openrobot_set_dps_amax(Amax, true);	
	} else commands_printf("This command requires two argument. Current servo Gain: Vmax:%.2f, Amax:%.2f\n", (double)Vel_maximum, (double)Acc_maximum);
}

static void terminal_cmd_custom_set_servo_gain(int argc, const char **argv) {
	if (argc == 3) {
		float kp = 0;
		float ki = 0;
		sscanf(argv[1], "%f", &kp);
		sscanf(argv[2], "%f", &ki);

		app_openrobot_set_servo_gain(kp, ki);
	} else commands_printf("This command requires two argument. Current servo Gain: Kp:%.2f, Ki:%.2f\n", (double)servo_Kp, (double)servo_Ki);
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
		for(unsigned int i=0; i<len; i++) commands_printf("(0x%x)%d ", data[i], data[i]);
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
						app_openrobot_set_dps(value_set[i], 0, DPS_CONTROL_TIMEOUT);
						break;
					case COMM_SET_DPS_VMAX:
						value_set[i] = (float)buffer_get_int32(data, &ind) / 1000.0;
						app_openrobot_set_dps_vmax(value_set[i], false);
						break;
					case COMM_SET_DPS_AMAX:
						value_set[i] = (float)buffer_get_int32(data, &ind) / 1000.0;
						app_openrobot_set_dps_amax(value_set[i], false);
						break;
					case COMM_SET_SERVO:
						value_set[i] = (float)buffer_get_int32(data, &ind) / 1000.0 ;
						app_openrobot_set_servo(value_set[i], SERVO_CONTROL);
						break;
					case COMM_SET_TRAJ:
						value_set[i] = (float)buffer_get_int32(data, &ind) / 1000.0 ;
						app_openrobot_set_traj(value_set[i], TRAJ_CONTROL);
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
				send_buffer[k++] = 1;				// target number of vesc using can msg, obiously 1, 
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
	buffer_append_float32(send_buffer, mc_interface_get_tachometer_value(false), 1, &ind); // +4
	buffer_append_float16(send_buffer, mcpwm_foc_get_rps(), 1e2, &ind); // +2

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
			buffer_append_float32(send_buffer, can_st_msg_5->tacho_value, 1, &ind); // +4
			buffer_append_float16(send_buffer, can_st_msg->rpm, 1e2, &ind); // +2 - currently this value is rps ()
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
	// VESC-Tool's ConfBackup cannot restore eeprom values. You need to make xml file backup.
	// set using stored configuration
	eeprom_var eeprom_custom_var_temp;
	if(conf_general_read_eeprom_var_custom(&eeprom_custom_var_temp, EEP_APP_SELECT)) app_mode = (uint32_t)eeprom_custom_var_temp.as_float;
	if(conf_general_read_eeprom_var_custom(&eeprom_custom_var_temp, EEP_CAN_TERMINAL_RESISTOR_MODE)) can_term_res = (uint32_t)eeprom_custom_var_temp.as_float;
	if(conf_general_read_eeprom_var_custom(&eeprom_custom_var_temp, EEP_VMAX)) Vel_maximum = eeprom_custom_var_temp.as_float;
	if(conf_general_read_eeprom_var_custom(&eeprom_custom_var_temp, EEP_AMAX)) Acc_maximum = eeprom_custom_var_temp.as_float;
	if(conf_general_read_eeprom_var_custom(&eeprom_custom_var_temp, EEP_ENC_ZERO_POS)) Zero_Pos = eeprom_custom_var_temp.as_float;
	
#ifdef HW60_IS_VESCUINO
	if(app_mode != APP_VESCuino) {
		app_mode = APP_VESCuino;
		// storing current setting to eeprom
		eeprom_custom_var_temp.as_float = (float)app_mode;
		conf_general_store_eeprom_var_custom(&eeprom_custom_var_temp, EEP_APP_SELECT);
	}
#elif defined HW60_IS_VESCULAR
	if(app_mode != APP_VESCular) {
		app_mode = APP_VESCular;
		// storing current setting to eeprom
		eeprom_custom_var_temp.as_float = (float)app_mode;
		conf_general_store_eeprom_var_custom(&eeprom_custom_var_temp, EEP_APP_SELECT);
	}
#endif

	//
	if(app_mode == APP_VESCular) {
		// Start uart communication: recommended usage UART:VESC-Tool, USB:ROS
		app_uartcomm_start();	

		// set custom app as openrobot app, To set the RX function.
		commands_set_app_data_handler(send_openrobot_app_data);

		chThdCreateStatic(dps_control_thread_wa, sizeof(dps_control_thread_wa), 
					NORMALPRIO, dps_control_thread, NULL);

		app_custom_can_terminal_resistor_set((bool)can_term_res);
	}
	else if(app_mode == APP_VESCuino) {
		// Initialize packet handler
		packet_init(send_packet_spi_slave, process_packet_spi_slave, PACKET_HANDLER);

		// set custom app as openrobot app, To set the RX function.
		commands_set_app_data_handler(send_openrobot_app_data);

#ifndef USE_CUSTOM_ABI_ENCODER_AT_SPI
		// SPI interface
		spi1_peripheral_setting_slave();
		// SPI Start
		spiStart(&HW_SPI_DEV, &spicfg);
		// SPI Start Slave Threads
		chThdCreateStatic(spi_slave_read_thread_wa, sizeof(spi_slave_read_thread_wa), NORMALPRIO, spi_slave_read_thread, NULL);
		commands_printf("\r\nVESCuino SPI RX Thread Start");
#else
		commands_printf("\r\nUse CUSTOM ABI Encoder at SPI Port");
#endif
	}
	

	int find_step = 0;
	float input_duty = 0;
	float input_volt = 0;
	float erpm_sum = 0;
	float erpm_avg = 0;
	float rps_sum = 0;
	float rps_avg = 0;
	float motor_Kv = 0; // rpm/volt
	float motor_Ke = 0;	// volt/(rad/s)
	float motor_Kt = 0; // Nm/A. Ke and Kt have the same value when the units are: Ke[volt/rad/s], Kv[Nm/A]

	// using the experiment plot
	chThdSleepMilliseconds(8000);
	commands_init_plot("Sample", "Data");
	commands_plot_add_graph("rps [get_rpm]");
	commands_plot_add_graph("rps [get_rpm_fast]");
	commands_plot_add_graph("rps [get_rpm_faster]");
	commands_plot_add_graph("rps [get_rps]");
	commands_plot_add_graph("rad [pos accum]");
	float samp = 0.0;

	// get motor setting and app setting handler
	const volatile mc_configuration *mc_conf = mc_interface_get_configuration();
	const volatile app_configuration *app_conf = app_get_configuration();

	for(;;) {
		// Check if it is time to stop.
		if (stop_now) {
			is_running = false;
			return;
		}

		//timeout_reset(); // Reset timeout if everything is OK.

		// Run your logic here. A lot of functionality is available in mc_interface.h.
		if (is_find_kt_running)
		{	
			if(encoder_is_configured()) {
				find_step++;
				if(find_step==1) {
						commands_printf("Finding motor constants, Controller Id:%d", controller_id);
						chThdSleepMilliseconds(500);
						commands_printf("Caution! The motor will start rotating in 5 seconds.");
						commands_printf("5...");
						chThdSleepMilliseconds(1000);
						commands_printf("4...");
						chThdSleepMilliseconds(1000);
						commands_printf("3...");
						chThdSleepMilliseconds(1000);
						commands_printf("2...");
						chThdSleepMilliseconds(1000);
						commands_printf("1...");
						chThdSleepMilliseconds(1000);
				}
				
				input_duty = find_step*0.2;
				commands_printf("Input Duty : %.1f", (double)input_duty);
				mc_interface_set_duty(input_duty);

				for (int i=0; i<400; i++) {
					timeout_reset();
					chThdSleepMilliseconds(10);
					if(i>=200 && i<300) {
						erpm_sum+=mc_interface_get_rpm();
						rps_sum+=mcpwm_foc_get_rps();
					}
					//commands_printf("motor speed : %.3ferpm, %.3frps", (double)mc_interface_get_rpm(), (double)mcpwm_foc_get_rps());

					commands_plot_set_graph(0);
					commands_send_plot_points(samp, mcpwm_foc_get_rpm()*RPM2RPS/polepair_number);
					commands_plot_set_graph(1);
					commands_send_plot_points(samp, mcpwm_foc_get_rpm_fast()*RPM2RPS/polepair_number);
					commands_plot_set_graph(2);
					commands_send_plot_points(samp, mcpwm_foc_get_rpm_faster()*RPM2RPS/polepair_number);		
					commands_plot_set_graph(3);
					commands_send_plot_points(samp, mcpwm_foc_get_rps());
					commands_plot_set_graph(4);
					commands_send_plot_points(samp, mcpwm_foc_get_pos_accum()*DEG2RAD);
					samp++;
				}

				erpm_avg = erpm_sum/100.;
				rps_avg = rps_sum/100.;
				input_volt = GET_INPUT_VOLTAGE()*input_duty;
				motor_Kv = erpm_avg/polepair_number/input_volt;
				motor_Ke = input_volt/rps_avg;
				motor_Kt = motor_Ke;
			
				commands_printf("Motor rotation speed average is %.3f[rad/s](%.3f[rpm]) at %.3f[volt]", (double)rps_avg, (double)(erpm_avg/polepair_number), (double)input_volt); 	
				commands_printf("Motor Polepair is %d, Kv:%.2f[rpm/volt], Ke:%.4f[volt/rps], Kt:%.4f[Nm/A]", polepair_number, (double)motor_Kv, (double)motor_Ke, (double)motor_Kt);
				commands_printf("Motor Max. Torque is %.2fNm at %.2fA (Short Period)", (double)(motor_Kt*mc_conf->l_current_max), (double)mc_conf->l_current_max);
				commands_printf("----------------------------------------------------------");

				mc_interface_set_current(0);
				erpm_sum = rps_sum = 0;
				if(find_step>=4) {
					is_find_kt_running = false;
					find_step = 0;
					samp = 0.;
				}
			}
			else {
				commands_printf("Encoder is not configured yet, Abort process");
				is_find_kt_running = false;
				find_step = 0;
				samp = 0.;
			}
		}

		// Run your logic here. A lot of functionality is available in mc_interface.h.
		if (is_servo_exam_running)
		{	
			if(encoder_is_configured()) {
				find_step++;
				if(find_step==1) {
					commands_printf("Servo Control Example");
					chThdSleepMilliseconds(500);
					commands_printf("Caution! The motor will start rotating in 5 seconds.");
					commands_printf("5...");
					chThdSleepMilliseconds(1000);
					commands_printf("4...");
					chThdSleepMilliseconds(1000);
					commands_printf("3...");
					chThdSleepMilliseconds(1000);
					commands_printf("2...");
					chThdSleepMilliseconds(1000);
					commands_printf("1...");
					chThdSleepMilliseconds(1000);
				}

				for (int i=0; i<500; i++) {	
					if(find_step==1) app_openrobot_set_servo(100, SERVO_CONTROL);
					if(find_step==2) app_openrobot_set_servo(2160, SERVO_CONTROL);
					if(find_step==3) app_openrobot_set_servo(-10000, SERVO_CONTROL);
					if(find_step==4) app_openrobot_set_dps(4000, 0, DPS_CONTROL_TIMEOUT);
					if(find_step==5) app_openrobot_set_servo(0, SERVO_CONTROL);

					commands_plot_set_graph(0);
					commands_send_plot_points(samp, mcpwm_foc_get_rpm()*RPM2RPS/polepair_number);
					commands_plot_set_graph(1);
					commands_send_plot_points(samp, mcpwm_foc_get_rpm_fast()*RPM2RPS/polepair_number);
					commands_plot_set_graph(2);
					commands_send_plot_points(samp, mcpwm_foc_get_rpm_faster()*RPM2RPS/polepair_number);		
					commands_plot_set_graph(3);
					commands_send_plot_points(samp, mcpwm_foc_get_rps());
					commands_plot_set_graph(4);
					commands_send_plot_points(samp, mcpwm_foc_get_pos_accum()*DEG2RAD);
					samp++;
					chThdSleepMilliseconds(10);
				}

				mc_interface_set_current(0);
				if(find_step>=6) {
					is_servo_exam_running = false;
					find_step = 0;
					samp = 0.;
				}
			}
			else {
				commands_printf("Encoder is not configured yet, Abort process");
				is_servo_exam_running = false;
				find_step = 0;
				samp = 0.;
			}
		}

		if (is_exp_plot_running)
		{	
			commands_plot_set_graph(0);
			commands_send_plot_points(samp, mcpwm_foc_get_rpm()*RPM2RPS/polepair_number);
			commands_plot_set_graph(1);
			commands_send_plot_points(samp, mcpwm_foc_get_rpm_fast()*RPM2RPS/polepair_number);
			commands_plot_set_graph(2);
			commands_send_plot_points(samp, mcpwm_foc_get_rpm_faster()*RPM2RPS/polepair_number);		
			commands_plot_set_graph(3);
			commands_send_plot_points(samp, mcpwm_foc_get_rps());
			commands_plot_set_graph(4);
			commands_send_plot_points(samp, mcpwm_foc_get_pos_accum()*DEG2RAD);
			samp++;
		}

		////////////////////////////////////////////////////////////////////////////
		// openrobot thread loop task
		// 1. update get can_id
		controller_id = app_conf->controller_id;

		// 2. update polepair_number
		polepair_number = mc_conf->foc_encoder_ratio;

		// 3. if can_terminal_resistor is not set on eeprom and usb connection is used, then set can_term_resistor on.
		if(conf_general_read_eeprom_var_custom(&eeprom_custom_var_temp, EEP_CAN_TERMINAL_RESISTOR_MODE)==0) {
			if(comm_usb_serial_configured_cnt()>=1 && can_term_res==0)	{
				// CAN Terminal Resistor EEPROM based On
				//const char *argv[6] = {"or_can", "1"};
				//terminal_cmd_custom_can_terminal_resistor(2, argv);

				// CAN Terminal Resistor Temporally On
				if(app_mode == APP_VESCular) {
					app_custom_can_terminal_resistor_set(1);
					commands_printf("Can Terminal Resistor Temporally On, At the USB connected VESCular.\n");
				}
			}
		}

		chThdSleepMilliseconds(10);	// 100Hz
		////////////////////////////////////////////////////////////////////////////
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

	if(control_mode==DPS_CONTROL_TIMEOUT) dps_cnt = 0;
}

void app_openrobot_set_dps_vmax(float Vmax, bool flash)
{
	if(Vmax>0 && Vmax<=DPS_VMAX_DEFAULT) {
		if(flash) {
			eeprom_var eeprom_custom_var_temp;
			eeprom_custom_var_temp.as_float = Vmax;
			conf_general_store_eeprom_var_custom(&eeprom_custom_var_temp, EEP_VMAX);
			commands_printf("Set Vmax: %.2f, eeprom stored", (double)Vmax);
		}
		Vel_maximum = Vmax;
	} else commands_printf("Invalid Vmax Value (input Vmax from 0 ~ %.1f)\n", (double)DPS_VMAX_DEFAULT);
}

void app_openrobot_set_dps_amax(float Amax, bool flash)
{
	if(Amax>0 && Amax<=DPS_AMAX_DEFAULT) {
		if(flash) {
			eeprom_var eeprom_custom_var_temp;
			eeprom_custom_var_temp.as_float = Amax;
			conf_general_store_eeprom_var_custom(&eeprom_custom_var_temp, EEP_AMAX);
			commands_printf("Set Amax: %.2f, eeprom stored", (double)Amax);
		}
		Acc_maximum = Amax;
	} else commands_printf("Invalid Vmax Value (input Amax from 0 ~ %.1f)\n", (double)DPS_AMAX_DEFAULT);
}

void app_openrobot_set_servo(float g_t, int c_mode)
{
	servo_target = g_t;
	control_mode = c_mode;
}

void app_openrobot_set_servo_gain(float kp, float ki)
{
	servo_Kp = kp;
	servo_Ki = ki;
	commands_printf("Set Servo Controller Gain, Kp:%.2f, Ki:%.2f", (double)kp, (double)ki);
}

void app_openrobot_set_traj(float g_t, int c_mode)
{
	if(control_mode != TRAJ_CONTROL) {
		if(traj_target != g_t) {
			traj_target = g_t;
			traj_start = mcpwm_foc_get_pos_accum();
			control_mode = c_mode;
		}
		else {
			app_openrobot_set_dps(0, 0, DPS_CONTROL_TIMEOUT);	
		}
	}
}

void app_openrobot_control_enable(void)
{
	// run this every first connection of dps control
	if(control_set==0) {
		s_prof = deg_ref = mcpwm_foc_get_pos_accum();
	}
	control_set = 1;
}

float app_openrobot_servo_controller(void)
{
	float dps_servo;
	float dps_servo_err;

	if (control_set == 0) {
		dps_error_int = 0;
	}
	
	dps_servo_err = servo_target - mcpwm_foc_get_pos_accum();
	dps_error_int += dps_servo_err * dt_rt;

	dps_servo = servo_Kp*dps_servo_err + servo_Ki*dps_error_int; // 7:optimal, 8:overshoot little, 6.:no overshoot
	if(fabs(dps_servo) >= (double)Vel_maximum) {
		if(dps_servo>0)	dps_servo = Vel_maximum;
		else 	 		dps_servo = -Vel_maximum;
	}

	return dps_servo;
}

float app_openrobot_traj_controller(void)
{
	static float start = 0;
	static float goal = 0;

	static float v = 0;
	static float a = 0;
	static float s = 0;
	static float T = 0;
	static float v_sq = 0;
	static float path = 0;

	start = traj_start;
	goal = traj_target;

	if((goal!=start) && T==0) {
		v = Vel_maximum/fabsf(goal - start);
		v_sq = powf(v,2);
		a = v_sq*1.5;//2.;

		if(v_sq/a > 1) {
			commands_printf("error - amax should be larger then %.3f", pow(v,2));
			control_mode = NONE;
		}
		else {
			traj_time = 0;
			T = (a + v_sq)/(v*a);
		}
	}

	if(T >= traj_time)
	{
		if(traj_time >= 0 && traj_time <= (v/a)) {
			//sddot = a;
			//sdot = a*time;
			s = 0.5*a*powf(traj_time,2);
		}
			
		if(traj_time > (v/a) && traj_time <= (T-v/a)) {
			//sddot = 0;
			//sdot = v;
			s = v*traj_time - 0.5*powf(v,2)/a;
		}
			
		if(traj_time > (T-v/a) && traj_time <= T) {
			//sddot = -a;
			//sdot = a*(T-time);
			s = (2*a*v*T - 2*powf(v,2) - powf(a,2)*powf((traj_time-T),2))/(2*a);
		}

		path = start + s*(goal-start);
		//commands_printf("time:%.3f, path:%.3f", (double)time, (double)path);

		traj_time += dt_rt;
	}
	else
	{
		control_mode = NONE;
		traj_time = 0;
		T = 0;	
	}

	return path;
}
/*
float app_openrobot_Trapezoidal_Traj_Gen_Given_Vmax_and_Amax(float start, float goal, float vmax, float amax, float dt)
{
    float v = vmax;
    float a = amax;
    
    if(powf(v,2)/a > 1) {
		commands_printf("amax should be larger then %.3f", pow(v,2));
		return false;
	}

    float T = (a + powf(v,2))/(v*a);
	commands_printf("time to go:%.3fsec", (double)T);
    
    float time = 0;
	float sddot = 0;
	float sdot = 0;
	float s = 0;
	int i = 0;
	int debug_print = 0;

	// time variables
	uint32_t ts = timer_time_now();
	float td = 0;
    while(T >= time)
	{
        if(time >= 0 && time <= (v/a)) {
            sddot = a;
            sdot = a*time;
            s = 0.5*a*powf(time,2);
		}
            
        if(time > (v/a) && time <= (T-v/a)) {
            sddot = 0;
            sdot = v;
            s = v*time - 0.5*powf(v,2)/a;
		}
            
        if(time > (T-v/a) && time <= T) {
            sddot = -a;
            sdot = a*(T-time);
            s = (2*a*v*T - 2*powf(v,2) - powf(a,2)*powf((time-T),2))/(2*a);
		}

		// set position reference value
		deg_ref = start + s*(goal-start);
		td = timer_seconds_elapsed_since(ts);	// end time check

		// position control on
		control_set=1;	 
		i++;
		//chThdSleepMilliseconds(1);	// 1000Hz

		if(debug_print==1) {
			commands_printf("time:%.3f / s:%.3f, sdot:%.3f, ssdot:%.3f", 
							(double)(time), (double)(s), (double)(sdot), (double)(sddot));
		}

        time += dt;
	}
	return td;
}
*/
/*
int app_openrobot_Trapezoidal_Traj_Gen_Given_Vmax_and_T(float vmax, float T, float dt) 
{
    float v = vmax;
	float a = 0;
    
    if(v*T > 1 && v*T <= 2) {
		a = powf(v,2)/(v*T-1);
	}
	else {
		commands_printf("vmax should be less then %.3f and larger than %.3f", (double)(2/T), (double)(1/T));
		return false;
	}        
    
    float time = 0;
	float sddot = 0;
	float sdot = 0;
	float s = 0;
	int i = 0;
    
    while(T >= time)
	{
        if(time >= 0 && time <= (v/a)) {
            sddot = a;
            sdot = a*time;
            s = 0.5*a*powf(time,2);
		}
            
        if(time > (v/a) && time <= (T-v/a)) {
            sddot = 0;
            sdot = v;
            s = v*time - 0.5*powf(v,2)/a;
		}

        if(time > (T-v/a) && time <= T) {
            sddot = -a;
            sdot = a*(T-time);
            s = (2*a*v*T - 2*powf(v,2) - powf(a,2)*powf((time-T),2))/(2*a);
		}

		traj[i++] = s;
		commands_printf("time:%.3f / s:%.3f, sdot:%.3f, ssdot:%.3f", 
						(double)(time), (double)(s), (double)(sdot), (double)(sddot));

        time += dt;
	}

    return i;
}

int app_openrobot_Trapezoidal_Traj_Gen_Given_Amax_and_T(float amax, float T, float dt) 
{
    float a = amax;
	float v = 0;
    
    if(a*powf(T,2) >= 4) {     
        v = 0.5*(a*T - powf(a,0.5)*powf((a*powf(T,2)-4),0.5));
	} 
	else {
		commands_printf("amax should be larger then %.3f", (double)(4./powf(T,2)));
		return false;         
	}

    float time = 0;
	float sddot = 0;
	float sdot = 0;
	float s = 0;
	int i = 0;

    while(T >= time)
	{
        if(time >= 0 && time <= (v/a)) {
            sddot = a;
            sdot = a*time;
            s = 0.5*a*powf(time,2);
		}
            
        if(time > (v/a) && time <= (T-v/a)) {
            sddot = 0;
            sdot = v;
            s = v*time - 0.5*powf(v,2)/a;
		}
            
        if(time > (T-v/a) && time <= T) {
            sddot = -a;
            sdot = a*(T-time);
            s = (2*a*v*T - 2*powf(v,2) - powf(a,2)*powf((time-T),2))/(2*a);
		}

		traj[i++] = s;
		//commands_printf("time:%.3f / s:%.3f, sdot:%.3f, ssdot:%.3f", 
		//				(double)(time), (double)(s), (double)(sdot), (double)(sddot));
        
        time += dt;
	}

    return i;
}

void app_openrobot_Trapezoidal_Path_Gen(float start, float goal, int total_i) {
	float pos = 0;

	int i = 0;
	while(total_i > i) {
		deg_ref = start + traj[i]*(goal-start);
		// position control on
		control_set=1;	 
		i++;
		chThdSleepMicroseconds(10000);	// 100Hz
	}
}
*/
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
			traj_time = 0;
			control_mode = NONE;
		} break;

		// This is for ROS. Assumption: ROS Command sent periodically.
		case DPS_CONTROL_TIMEOUT: {
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

		case SERVO_CONTROL: {
			dps_target = app_openrobot_servo_controller();
			app_openrobot_control_enable();
			// USE maximum Acceleration value at SERVO_Control, Speed regulated by Vmax at SERVO_Control
			// The Servo Control Kp. Ki gain is dependent on the Vmax, Amax
			// For hard servo control, it is good to use Acceleration as Maximum
			deg_ref = app_openrobot_genProfile(dps_target, (float)DPS_AMAX_DEFAULT, dt_rt);	
		} break;

		case TRAJ_CONTROL: {
			deg_ref = app_openrobot_traj_controller();
			app_openrobot_control_enable();
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
				commands_printf("  id:%d,t:%.2fs,t_to:%.2fs/dps_now:%.2f,dps_goal:%.2f/ang now:%.2fdeg,ang goal:%.2fdeg", 
					app_get_configuration()->controller_id, (double)(time_cnt/10000.), (double)(dps_duration_sec - dps_cnt/10000.), 
					(double)(mcpwm_foc_get_rps()*RAD2DEG), (double)dps_target, (double)mcpwm_foc_get_pos_accum(), (double)deg_ref);
			}
		}

		// Loop Time Manage
		debug_cnt++;
		time_cnt++;
		chThdSleepMicroseconds(100);	// 10khz
	}
}