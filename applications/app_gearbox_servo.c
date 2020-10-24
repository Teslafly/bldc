/*
	Copyright 2016 - 2019 Benjamin Vedder	benjamin@vedder.se

	This file is part of the VESC firmware.

	The VESC firmware is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    The VESC firmware is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
    */

#include "app.h"

#include "ch.h"
#include "hal.h"
#include "stm32f4xx_conf.h"
#include "servo_dec.h"
#include "mc_interface.h"
#include "timeout.h"
#include "utils.h"
#include "comm_can.h"
#include "hw.h"
#include <math.h>
#include "commands.h"

// Only available if servo output is not active
#if !SERVO_OUT_ENABLE

typedef enum {
	PPM_CTRL_TYPE_NONE_SERVO = 0,
	PPM_CTRL_TYPE_CURRENT_NOREV_SERVO,
	PPM_CTRL_TYPE_ABS_SERVO,
} servo_control_modes;

// Settings
#define MAX_CAN_AGE						0.1
#define MIN_PULSES_WITHOUT_POWER		50


// limit switch pin
// palSetPadMode(SERVO_LIMIT_SWITCH_IO_PORT, 
// 			  SERVO_LIMIT_SWITCH_IO_PIN, 
// 		      PAL_MODE_INPUT_PULLUP);

//palSetPadMode(GPIOA, 5, PAL_MODE_INPUT_ANALOG); // adc1 input
//palSetPadMode(GPIOA, 6, PAL_MODE_INPUT_ANALOG); // adc2 input

// adc2 input
#define SERVO_LIMIT_SWITCH_IO_PORT GPIOA
#define SERVO_LIMIT_SWITCH_IO_PIN  6

// Threads
static THD_FUNCTION(servo_thread, arg);
static THD_WORKING_AREA(servo_thread_wa, 1536);
static thread_t *ppm_tp;
static volatile bool ppm_rx = false;

// Private functions
static void servodec_func(void);

// Private variables
static volatile bool is_running = false;
static volatile bool stop_now = true;
static volatile ppm_config config;
static volatile int pulses_without_power = 0;
static float input_val = 0.0;
static volatile float direction_hyst = 0;
static volatile servo_control_modes control_mode = PPM_CTRL_TYPE_NONE_SERVO;

static volatile bool servo_homed = false; // for testing with no home switch

static volatile float servo_per = 50.5;

// Private functions
#endif

void app_custom_configure(app_configuration *conf ) {
	
#if !SERVO_OUT_ENABLE
	//config = *conf; //(ppm_config *conf)
	//appconf = *conf;
	//config = appconf.app_ppm_conf;
	config = (*conf).app_ppm_conf;  // grab ppm app config.

	pulses_without_power = 0;

	if (is_running) {
		servodec_set_pulse_options(config.pulse_start, config.pulse_end, config.median_filter);
	}

	direction_hyst = config.max_erpm_for_dir * 0.20;
#else
	(void)conf;
#endif
}
/**
 * @brief 
 * 
 */
void app_custom_start(void) {
#if !SERVO_OUT_ENABLE
	stop_now = false;
	servo_homed = false;
	chThdCreateStatic(servo_thread_wa, sizeof(servo_thread_wa), NORMALPRIO, servo_thread, NULL);
#endif
}
 /**
  * @brief 
  * 
  */
void app_custom_stop(void) {
#if !SERVO_OUT_ENABLE
	stop_now = true;

	if (is_running) {
		chEvtSignalI(ppm_tp, (eventmask_t) 1);
		servodec_stop();
	}

	while(is_running) {
		chThdSleepMilliseconds(1);
	}
#endif
}

float app_ppm_servo_get_decoded_level(void) {
#if !SERVO_OUT_ENABLE
	return input_val;
#else
	return 0.0;
#endif
}

#if !SERVO_OUT_ENABLE
static void servodec_func(void) {
	ppm_rx = true;
	chSysLockFromISR();
	chEvtSignalI(ppm_tp, (eventmask_t) 1);
	chSysUnlockFromISR();
}

/**
 * @brief Construct a new thd function object
 * 	 servo app main thread.
 * 
 */

static THD_FUNCTION(servo_thread, arg) {
	(void)arg;

	chRegSetThreadName("APP_PPM_SERVO");
	ppm_tp = chThdGetSelfX();

	servodec_set_pulse_options(config.pulse_start, config.pulse_end, config.median_filter);
	servodec_init(servodec_func);
	is_running = true;

	//bool servo_homed = false; // not homed on boot
	bool servo_homed = true; // for testing with no home switch
							// goes directly into position control mode

	// limit switch pin
	palSetPadMode(SERVO_LIMIT_SWITCH_IO_PORT, 
				SERVO_LIMIT_SWITCH_IO_PIN, 
				PAL_MODE_INPUT_PULLUP);


	mc_interface_get_tachometer_value(true); // resets tachometer to 0, mostly helps with debug here.

	// Example of using the experiment plot
	//	chThdSleepMilliseconds(8000);
	//	commands_init_plot("Sample", "Value");
	//	commands_plot_add_graph("Temp Fet");
	//	commands_plot_add_graph("Input Voltage");
	//	float samp = 0.0;
	//
	//	for(;;) {
	//		commands_plot_set_graph(0);
	//		commands_send_plot_points(samp, mc_interface_temp_fet_filtered());
	//		commands_plot_set_graph(1);
	//		commands_send_plot_points(samp, GET_INPUT_VOLTAGE());
	//		samp++;
	//		chThdSleepMilliseconds(10);
	//	}

		// testing printout
		// chThdSleepMilliseconds(3000);
		// commands_init_plot("Sample", "Value");
		// commands_plot_add_graph("Servo percent");
		int samp = 0.0;
		int decimation = 0;


	for(;;) {
		chEvtWaitAnyTimeout((eventmask_t)1, MS2ST(2));

		if (stop_now) {
			is_running = false;
			return;
		}

		if (ppm_rx) {
			ppm_rx = false;
			timeout_reset();
		}

		const volatile mc_configuration *mcconf = mc_interface_get_configuration();
		const float rpm_now = mc_interface_get_rpm();
		float servo_val = servodec_get_servo(0);
		float servo_ms = utils_map(servo_val, -1.0, 1.0, config.pulse_start, config.pulse_end);

		// Truncate the read voltage
		utils_truncate_number(&servo_val, -1.0, 1.0);

		// // STICK MAPPING
		// // full stick, no middle
		// input_val = servo_val;
		// servo_val += 1.0;
		// servo_val /= 2.0;

		// Apply deadband
		utils_deadband(&servo_val, config.hyst, 1.0);

		//switch (control_mode) {
		// absolute input. 
		// takes servo value of -1 to 1 and maps from 0 to 1.
		// Mapping with respect to center pulsewidth
		if (servo_ms < config.pulse_center) {  // neg values
			servo_val = utils_map(servo_ms, config.pulse_start,
					config.pulse_center, 1.0, 0.0); 
		} else {  // pos values
			servo_val = utils_map(servo_ms, config.pulse_center,
					config.pulse_end, 0.0, 1.0);
		}

		input_val = servo_val;
		

		if (decimation  == 100){
			//commands_plot_set_graph(0);
			//commands_send_plot_points((float)samp, servo_percent);
			samp += 1;
			decimation = 0;
			servo_per =  servo_val * 100.0;
			//commands_printf("servo cmd: %.2f ", (double)(servo_per));
			//commands_printf("test: %.2f ", (double)(3.14159));
		}
		decimation++;


		// reset no signal counter
		if (timeout_has_timeout() || servodec_get_time_since_update() > timeout_get_timeout_msec() ||
				mc_interface_get_fault() != FAULT_CODE_NONE) {
			pulses_without_power = 0;
			continue;
		}

		// // Apply deadband
		// utils_deadband(&servo_val, config.hyst, 1.0);

		// Apply throttle curve
		//servo_val = utils_throttle_curve(servo_val, config.throttle_exp, config.throttle_exp_brake, config.throttle_exp_mode);

		// // Apply ramping
		// static systime_t last_time = 0;
		// static float servo_val_ramp = 0.0;
		// float ramp_time = fabsf(servo_val) > fabsf(servo_val_ramp) ? config.ramp_time_pos : config.ramp_time_neg;

		// const float dt = (float)ST2MS(chVTTimeElapsedSinceX(last_time)) / 1000.0;
		// last_time = chVTGetSystemTimeX();

		// if (ramp_time > 0.01) {
		// 	const float ramp_step = dt / ramp_time;
		// 	utils_step_towards(&servo_val_ramp, servo_val, ramp_step);
		// 	servo_val = servo_val_ramp;
		// }
		// // end ramping

		// at this point, servo_val has been mapped, deadbanded, and scaled.
		// it is a value from 0 to 1.0 mapped from pulselength start to pulselength end.

		float current = 0;

		// custom code
		//rotations = mc_interface_get_tachometer_value(false) / (??);
		//motor pole number = encoder_ratio *2
		//revolution = motor pole number * 3

		//read limit switch
		//bool limit_switch_input = false;
		bool limit_switch_input = !palReadPad(
								SERVO_LIMIT_SWITCH_IO_PORT, 
								SERVO_LIMIT_SWITCH_IO_PIN);

		// check if we have homed and limit switch is pressed.
		if  (servo_homed == false && limit_switch_input == true){
				servo_homed = true;
				mc_interface_get_tachometer_value(true); // resets tachometer to 0
				// or
				//mcpwm_foc_set_tachometer_value(3*encoder_ratio*2*limit_switch_zero_revolutions); 
		}

		// if not homed, go into current mode control
		// if homed, go into absolute servo control.
		if (servo_homed == false){
			//config.ctrl_type = PPM_CTRL_TYPE_CURRENT_NOREV;
			control_mode = PPM_CTRL_TYPE_CURRENT_NOREV_SERVO;
		} else {
			//config.ctrl_type = PPM_CTRL_TYPE_TACH_SERVO;
			// remap to min_revolutions / max_revolutions
			control_mode = PPM_CTRL_TYPE_ABS_SERVO;
		}
		
		// All pins and buttons are still decoded for debugging, even
		// when output is disabled.
		if (app_is_output_disabled()) {
			continue;
		}

		// if signal, increment signal present counter
		if (fabsf(servo_val) < config.hyst) {  // boot input no power interlock 
			pulses_without_power++;
		}

		// check no signal counter, brake motor if no signal.
		if (pulses_without_power < MIN_PULSES_WITHOUT_POWER && config.safe_start) {
			static int pulses_without_power_before = 0;
			if (pulses_without_power
			 == pulses_without_power_before) {
				pulses_without_power = 0;
			}
			pulses_without_power_before = pulses_without_power;
			mc_interface_set_brake_current(timeout_get_brake_current());
			continue;
		}

		// control motor
		switch (control_mode) {
			case PPM_CTRL_TYPE_ABS_SERVO: // make this an actual type.
				//current_mode = false; 
				mc_interface_set_pid_pos(servo_val * 360);
				break;

			case PPM_CTRL_TYPE_CURRENT_NOREV_SERVO:
				//current_mode = true;

				if ((servo_val >= 0.0 && rpm_now > 0.0) || (servo_val < 0.0 && rpm_now < 0.0)) {
					current = servo_val * mcconf->lo_current_motor_max_now;
				} else {
					current = servo_val * fabsf(mcconf->lo_current_motor_min_now);
				}
				mc_interface_set_current(current);
				break;

			default:
				continue;
		}
		// end of mode switch-case

		// // send current control command to motor
		// if (current_mode) {
		// 	// if (current_mode_brake) {
		// 	// 	mc_interface_set_brake_current(current);
		// 	// } else {
		// 		float current_out = current;
		// 		bool is_reverse = false;
		// 		if (current_out < 0.0) {
		// 			is_reverse = true;
		// 			current_out = -current_out;
		// 			//current = -current;
		// 			//rpm_local = -rpm_local;
		// 			//rpm_lowest = -rpm_lowest;
		// 		}

		// 		if (is_reverse) {
		// 			mc_interface_set_current(current_out);
		// 		} else {
		// 			mc_interface_set_current(-current_out);
		// 		}
		// 	//}
		// }

	}
}
#endif
