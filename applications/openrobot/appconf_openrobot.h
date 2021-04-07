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

#ifndef APPCONF_APPCONF_EXAMPLE_PPM_H_
#define APPCONF_APPCONF_EXAMPLE_PPM_H_

/* overide default parameters in here */

// Use custom user application
#define APPCONF_APP_TO_USE				APP_CUSTOM

// can
#define APPCONF_SEND_CAN_STATUS				CAN_STATUS_1
#define APPCONF_CAN_MODE					CAN_MODE_VESC
#define APPCONF_SEND_CAN_STATUS_RATE_HZ		1000
#define APPCONF_CAN_BAUD_RATE				CAN_BAUD_1M

// Position PID parameters
#define MCCONF_P_PID_KP					0.02	// Proportional gain
#define MCCONF_P_PID_KI					0.0		// Integral gain
#define MCCONF_P_PID_KD					0.0002	// Derivative gain
#define MCCONF_P_PID_KD_FILTER			0.05	// Derivative filter
#define MCCONF_P_PID_ANG_DIV			1.0		// Divide angle by this value

#endif
