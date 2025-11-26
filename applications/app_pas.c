/*
	Copyright 2016 Benjamin Vedder	benjamin@vedder.se
	Copyright 2020 Marcos Chaparro	mchaparro@powerdesigns.ca

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

#pragma GCC optimize ("Os")

#include "app.h"

#include "ch.h"
#include "hal.h"
#include "stm32f4xx_conf.h"
#include "mc_interface.h"
#include "timeout.h"
#include "utils_math.h"
#include "comm_can.h"
#include "hw.h"
#include "timer.h"
#include <math.h>

// Settings
#define PEDAL_INPUT_TIMEOUT				0.2
#define MAX_MS_WITHOUT_CADENCE_OR_TORQUE	5000
#define MAX_MS_WITHOUT_CADENCE			1000
#define MIN_MS_WITHOUT_POWER			500
#define FILTER_SAMPLES					5
#define RPM_FILTER_SAMPLES				8

// Threads
static THD_FUNCTION(pas_thread, arg);
__attribute__((section(".ram4"))) static THD_WORKING_AREA(pas_thread_wa, 512);
static void pas_sensor_update(float loop_dt);
static void pas_sensor_update_quadrature(float loop_dt);
static void pas_sensor_update_single(float loop_dt);
static void pas_single_exti_enable(bool enable);

// Private variables
static volatile pas_config config;
static volatile float sub_scaling = 1.0;
static volatile float output_current_rel = 0.0;
static volatile float ms_without_power = 0.0;
static volatile float max_pulse_period = 0.0;
static volatile float min_pedal_period = 0.0;
static volatile float direction_conf = 0.0;
static volatile float pedal_rpm = 0;
static volatile bool primary_output = false;
static volatile bool stop_now = true;
static volatile bool is_running = false;
static volatile float torque_ratio = 0.0;
static volatile float pas_rpm_end_effective = 0.0f;
#if defined(HW_PAS_PPM_EXTI_LINE)
typedef struct {
	volatile uint32_t last_edge_time;
	volatile float revolution_period;
	volatile bool period_valid;
	volatile bool exti_enabled;
} pas_ppm_single_state_t;

static volatile pas_ppm_single_state_t ppm_pas_state = {
	.last_edge_time = 0,
	.revolution_period = 0.0f,
	.period_valid = false,
	.exti_enabled = false
};
#endif

static volatile float pas_target_erpm = 0.0f;

static float pas_default_cadence_rpm_end(void) {
#ifdef APPCONF_PAS_PEDAL_RPM_END
	return APPCONF_PAS_PEDAL_RPM_END;
#else
	return 180.0f;
#endif
}

static void pas_refresh_effective_rpm_end(void) {
	float rpm_end = config.pedal_rpm_end;
	if (config.sensor_type == PAS_SENSOR_TYPE_SINGLE_PIN_PPM) {
		rpm_end = pas_default_cadence_rpm_end();
	}

	if (!isfinite(rpm_end) || rpm_end < (config.pedal_rpm_start + 1.0f)) {
		rpm_end = config.pedal_rpm_start + 1.0f;
	}

	pas_rpm_end_effective = rpm_end;
}

static float pas_get_assist_ceiling(void) {
	if (config.sensor_type == PAS_SENSOR_TYPE_SINGLE_PIN_PPM) {
		return 1.0f;
	}
	return config.current_scaling;
}

#if defined(HW_PAS_PPM_EXTI_LINE)
static float pas_get_drive_reduction(void) {
	float ratio = config.current_scaling;
	if (config.sensor_type == PAS_SENSOR_TYPE_SINGLE_PIN_PPM) {
		ratio = config.pedal_rpm_end;
		if (!isfinite(ratio) || ratio < 1.0f) {
			ratio = 1.0f;
		}
	} else {
		if (!isfinite(ratio) || ratio <= 0.0f) {
			ratio = 1.0f;
		}
	}
	return ratio;
}

static float pas_get_motor_pole_pairs(void) {
	const volatile mc_configuration *mcconf = mc_interface_get_configuration();
	int poles = 0;
	if (mcconf) {
		poles = mcconf->si_motor_poles;
	}
	if (poles <= 0) {
		poles = 8;
	}
	return ((float)poles) * 0.5f;
}

static void pas_set_target_erpm(float pedal_rpm_target) {
	float target = pedal_rpm_target * pas_get_drive_reduction() * pas_get_motor_pole_pairs();
	pas_target_erpm = target;
}
#else
static void pas_set_target_erpm(float pedal_rpm_target) {
	(void)pedal_rpm_target;
	pas_target_erpm = 0.0f;
}
#endif

static void pas_single_exti_enable(bool enable) {
#if defined(HW_PAS_PPM_EXTI_LINE)
	if (enable) {
		if (ppm_pas_state.exti_enabled) {
			return;
		}

		RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);
		palSetPadMode(HW_ICU_GPIO, HW_ICU_PIN, PAL_MODE_INPUT_PULLUP);
		SYSCFG_EXTILineConfig(HW_PAS_PPM_EXTI_PORTSRC, HW_PAS_PPM_EXTI_PINSRC);

		EXTI_InitTypeDef EXTI_InitStructure;
		EXTI_StructInit(&EXTI_InitStructure);
		EXTI_InitStructure.EXTI_Line = HW_PAS_PPM_EXTI_LINE;
		EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
		EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;
		EXTI_InitStructure.EXTI_LineCmd = ENABLE;
		EXTI_Init(&EXTI_InitStructure);
		EXTI_ClearITPendingBit(HW_PAS_PPM_EXTI_LINE);

		nvicEnableVector(HW_PAS_PPM_EXTI_CH, 7);
		ppm_pas_state.last_edge_time = timer_time_now();
		ppm_pas_state.revolution_period = 0.0f;
		ppm_pas_state.period_valid = false;
		ppm_pas_state.exti_enabled = true;
	} else {
		if (!ppm_pas_state.exti_enabled) {
			return;
		}

		EXTI_InitTypeDef EXTI_InitStructure;
		EXTI_StructInit(&EXTI_InitStructure);
		EXTI_InitStructure.EXTI_Line = HW_PAS_PPM_EXTI_LINE;
		EXTI_InitStructure.EXTI_LineCmd = DISABLE;
		EXTI_Init(&EXTI_InitStructure);
		ppm_pas_state.exti_enabled = false;
		ppm_pas_state.period_valid = false;
	}
#else
	(void)enable;
#endif
}

void app_pas_pas_irq_handler(void) {
#if defined(HW_PAS_PPM_EXTI_LINE)
	if (!ppm_pas_state.exti_enabled) {
		return;
	}

	if (config.sensor_type != PAS_SENSOR_TYPE_SINGLE_PIN_PPM) {
		return;
	}

	uint32_t now = timer_time_now();
	uint32_t prev = ppm_pas_state.last_edge_time;
	ppm_pas_state.last_edge_time = now;

	if (prev != 0) {
		float dt = timer_seconds_elapsed_since(prev);
		if (dt <= 0.0f) {
			return;
		}

		float pedal_period = dt * (float)config.magnets;
		if (pedal_period < (min_pedal_period * 0.3f)) {
			return;
		}

		if (!ppm_pas_state.period_valid || !config.use_filter) {
			ppm_pas_state.revolution_period = pedal_period;
		} else {
			UTILS_LP_FAST(ppm_pas_state.revolution_period, pedal_period, 0.2f);
		}

		ppm_pas_state.period_valid = true;
	}
#else
	(void)config;
#endif
}

/**
 * Configure and initialize PAS application
 *
 * @param conf
 * App config
 */
void app_pas_configure(pas_config *conf) {
	config = *conf;
	ms_without_power = 0.0;
	output_current_rel = 0.0;
	pas_refresh_effective_rpm_end();

	// a period longer than this should immediately reduce power to zero
	max_pulse_period = 1.0 / ((config.pedal_rpm_start / 60.0) * config.magnets) * 1.2;

	// if pedal spins at x3 the end rpm, assume its beyond limits
	min_pedal_period = 1.0 / ((pas_rpm_end_effective * 3.0 / 60.0));

	(config.invert_pedal_direction) ? (direction_conf = -1.0) : (direction_conf = 1.0);

#if defined(HW_PAS_PPM_EXTI_LINE)
	ppm_pas_state.period_valid = false;
	ppm_pas_state.revolution_period = 0.0f;
	ppm_pas_state.last_edge_time = timer_time_now();
#endif
}

/**
 * Start PAS thread
 *
 * @param is_primary_output
 * True when PAS app takes direct control of the current target,
 * false when PAS app shares control with the ADC app for current command
 */
void app_pas_start(bool is_primary_output) {
	stop_now = false;
	chThdCreateStatic(pas_thread_wa, sizeof(pas_thread_wa), NORMALPRIO, pas_thread, NULL);

	if (config.sensor_type == PAS_SENSOR_TYPE_SINGLE_PIN_PPM) {
		pas_single_exti_enable(true);
	}

	primary_output = is_primary_output;
}

bool app_pas_is_running(void) {
	return is_running;
}

void app_pas_stop(void) {
	stop_now = true;
	while (is_running) {
		chThdSleepMilliseconds(1);
	}

	if (config.sensor_type == PAS_SENSOR_TYPE_SINGLE_PIN_PPM) {
		pas_single_exti_enable(false);
	}

	if (primary_output == true) {
		mc_interface_set_current_rel(0.0);
	}
	else {
		output_current_rel = 0.0;
	}
}

void app_pas_set_current_sub_scaling(float current_sub_scaling) {
	sub_scaling = current_sub_scaling;
}

float app_pas_get_current_target_rel(void) {
	return output_current_rel;
}

float app_pas_get_pedal_rpm(void) {
	return pedal_rpm;
}

float app_pas_get_target_erpm(void) {
	return pas_target_erpm;
}

static void pas_sensor_update_quadrature(float loop_dt) {
#ifdef HW_PAS1_PORT
	const int8_t QEM[] = {0,-1,1,2,1,0,2,-1,-1,2,0,1,2,1,-1,0}; // Quadrature Encoder Matrix
	int8_t direction_qem;
	uint8_t new_state;
	static uint8_t old_state = 0;
	static float old_timestamp = 0;
	static float inactivity_time = 0;
	static float period_filtered = 0;
	static int32_t correct_direction_counter = 0;

	uint8_t PAS1_level = palReadPad(HW_PAS1_PORT, HW_PAS1_PIN);
	uint8_t PAS2_level = palReadPad(HW_PAS2_PORT, HW_PAS2_PIN);

	new_state = PAS2_level * 2 + PAS1_level;
	direction_qem = (float) QEM[old_state * 4 + new_state];
	old_state = new_state;

	// Require several quadrature events in the right direction to prevent vibrations from
	// engging PAS
	int8_t direction = (direction_conf * direction_qem);
	
	switch(direction) {
		case 1: correct_direction_counter++; break;
		case -1:correct_direction_counter = 0; break;
	}

	const float timestamp = (float)chVTGetSystemTimeX() / (float)CH_CFG_ST_FREQUENCY;

	// sensors are poorly placed, so use only one rising edge as reference
	if( (new_state == 3) && (correct_direction_counter >= 4) ) {
		float period = (timestamp - old_timestamp) * (float)config.magnets;
		old_timestamp = timestamp;

		UTILS_LP_FAST(period_filtered, period, 1.0);

		if(period_filtered < min_pedal_period) { //can't be that short, abort
			return;
		}
		pedal_rpm = 60.0 / period_filtered;
		pedal_rpm *= (direction_conf * (float)direction_qem);
		inactivity_time = 0.0;
		correct_direction_counter = 0;
	}
	else {
		inactivity_time += loop_dt;

		//if no pedal activity, set RPM as zero
		if(inactivity_time > max_pulse_period) {
			pedal_rpm = 0.0;
		}
	}
#endif
	(void)loop_dt;
}

static void pas_sensor_update_single(float loop_dt) {
	(void)loop_dt;
#if defined(HW_PAS_PPM_EXTI_LINE)
	if (!ppm_pas_state.exti_enabled) {
		pedal_rpm = 0.0f;
		pas_set_target_erpm(0.0f);
		return;
	}

	float idle_time = timer_seconds_elapsed_since(ppm_pas_state.last_edge_time);
	if (idle_time > max_pulse_period) {
		pedal_rpm = 0.0f;
		ppm_pas_state.period_valid = false;
		pas_set_target_erpm(0.0f);
		return;
	}

	if (!ppm_pas_state.period_valid) {
		return;
	}

	float revolution_period = ppm_pas_state.revolution_period;
	if (revolution_period < min_pedal_period) {
		revolution_period = min_pedal_period;
	}

	float rpm = 0.0f;
	if (revolution_period > 1e-4f) {
		rpm = 60.0f / revolution_period;
	}
	rpm *= direction_conf;

	if (config.use_filter) {
		static float rpm_filtered = 0.0f;
		UTILS_LP_FAST(rpm_filtered, rpm, 0.2f);
		pedal_rpm = rpm_filtered;
	} else {
		pedal_rpm = rpm;
	}

	pas_set_target_erpm(pedal_rpm);
#else
	(void)loop_dt;
	pedal_rpm = 0.0f;
	pas_set_target_erpm(0.0f);
#endif
}

static void pas_sensor_update(float loop_dt) {
	switch (config.sensor_type) {
	case PAS_SENSOR_TYPE_SINGLE_PIN_PPM:
		pas_sensor_update_single(loop_dt);
		break;
	case PAS_SENSOR_TYPE_QUADRATURE:
	default:
		pas_sensor_update_quadrature(loop_dt);
		break;
	}
}

static THD_FUNCTION(pas_thread, arg) {
	(void)arg;

	float output = 0;
	chRegSetThreadName("APP_PAS");

#ifdef HW_PAS1_PORT
	palSetPadMode(HW_PAS1_PORT, HW_PAS1_PIN, PAL_MODE_INPUT_PULLUP);
	palSetPadMode(HW_PAS2_PORT, HW_PAS2_PIN, PAL_MODE_INPUT_PULLUP);
#endif

	is_running = true;

	for(;;) {
		// Sleep for a time according to the specified rate
		systime_t sleep_time = CH_CFG_ST_FREQUENCY / config.update_rate_hz;

		// At least one tick should be slept to not block the other threads
		if (sleep_time == 0) {
			sleep_time = 1;
		}
		chThdSleep(sleep_time);

		if (stop_now) {
			is_running = false;
			return;
		}

		const float loop_dt = (float)sleep_time / (float)CH_CFG_ST_FREQUENCY;
		pas_sensor_update(loop_dt);
		float assist_ceiling = pas_get_assist_ceiling();

		// For safe start when fault codes occur
		if (mc_interface_get_fault() != FAULT_CODE_NONE) {
			ms_without_power = 0;
		}

		if (app_is_output_disabled()) {
			continue;
		}

		switch (config.ctrl_type) {
			case PAS_CTRL_TYPE_NONE:
				output = 0.0;
				break;
			case PAS_CTRL_TYPE_CADENCE:
			{
				// Map pedal rpm to assist level

				// NOTE: If the limits are the same a numerical instability is approached, so in that case
				// just use on/off control (which is what setting the limits to the same value essentially means).
				float cadence_rpm_end = pas_rpm_end_effective;
				if (cadence_rpm_end > (config.pedal_rpm_start + 1.0)) {
					output = utils_map(pedal_rpm, config.pedal_rpm_start, cadence_rpm_end, 0.0, assist_ceiling * sub_scaling);
					utils_truncate_number(&output, 0.0, assist_ceiling * sub_scaling);
				} else {
					if (pedal_rpm > cadence_rpm_end) {
						output = assist_ceiling * sub_scaling;
					} else {
						output = 0.0;
					}
				}
				break;
			}

#ifdef HW_HAS_PAS_TORQUE_SENSOR
			case PAS_CTRL_TYPE_TORQUE:
			{
				torque_ratio = hw_get_PAS_torque();
				output = torque_ratio * assist_ceiling * sub_scaling;
				utils_truncate_number(&output, 0.0, assist_ceiling * sub_scaling);
			}
			/* fall through */
			case PAS_CTRL_TYPE_TORQUE_WITH_CADENCE_TIMEOUT:
			{
				// disable assistance if torque has been sensed for >5sec without any pedal movement. Prevents
				// motor overtemps when the rider is just resting on the pedals
				static float ms_without_cadence_or_torque = 0.0;
				if(output == 0.0 || pedal_rpm > 0) {
					ms_without_cadence_or_torque = 0.0;
				} else {
					ms_without_cadence_or_torque += (1000.0 * (float)sleep_time) / (float)CH_CFG_ST_FREQUENCY;
					if(ms_without_cadence_or_torque > MAX_MS_WITHOUT_CADENCE_OR_TORQUE) {
						output = 0.0;
					}
				}
				// if cranks are not moving, there should not be any output. This covers the case of a torque sensor
				// stuck with a non-zero signal.
				static float ms_without_cadence = 0.0;
				if(pedal_rpm < 0.01) {
					ms_without_cadence += (1000.0 * (float)sleep_time) / (float)CH_CFG_ST_FREQUENCY;
					if(ms_without_cadence > MAX_MS_WITHOUT_CADENCE) {
						output = 0.0;
					}
				} else {
					ms_without_cadence = 0.0;
				}
			}
#endif
			default:
				break;
		}

		// Apply ramping
		static systime_t last_time = 0;
		static float output_ramp = 0.0;
		float ramp_time = fabsf(output) > fabsf(output_ramp) ? config.ramp_time_pos : config.ramp_time_neg;

		if (ramp_time > 0.01) {
			const float ramp_step = (float)ST2MS(chVTTimeElapsedSinceX(last_time)) / (ramp_time * 1000.0);
			utils_step_towards(&output_ramp, output, ramp_step);
			utils_truncate_number(&output_ramp, 0.0, assist_ceiling * sub_scaling);

			last_time = chVTGetSystemTimeX();
			output = output_ramp;
		}

		if (output < 0.001) {
			ms_without_power += (1000.0 * (float)sleep_time) / (float)CH_CFG_ST_FREQUENCY;
		}

		// Safe start is enabled if the output has not been zero for long enough
		if (ms_without_power < MIN_MS_WITHOUT_POWER) {
			static int pulses_without_power_before = 0;
			if (ms_without_power == pulses_without_power_before) {
				ms_without_power = 0;
			}
			pulses_without_power_before = ms_without_power;
			output_current_rel = 0.0;
			continue;
		}

		// Reset timeout
		timeout_reset();

		if (primary_output == true) {
			mc_interface_set_current_rel(output);
		}
		else {
			output_current_rel = output;
		}
	}
}
