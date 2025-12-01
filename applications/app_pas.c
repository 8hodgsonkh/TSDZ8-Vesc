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
#include <stdint.h>
#include <stdbool.h>

// Settings
#define PEDAL_INPUT_TIMEOUT				0.2
#define MAX_MS_WITHOUT_CADENCE_OR_TORQUE	5000
#define MAX_MS_WITHOUT_CADENCE			1000
#define MIN_MS_WITHOUT_POWER			500
#define FILTER_SAMPLES					5
#define RPM_FILTER_SAMPLES				8
#define PAS_BOOST_CADENCE_TC		0.1f
#define PAS_BOOST_INTENT_THRESHOLD	1.0f
#define PAS_BOOST_INTENT_STRONG	40.0f
#define PAS_BOOST_RISE_TC		0.08f
#define PAS_BOOST_FALL_TC		0.25f
#define PAS_BOOST_MAX_FRACTION	0.6f
#define PAS_BOOST_MIN_CADENCE_RPM	20.0f

#define PAS_REAL_STEP_WINDOW		0.2f
#define PAS_REAL_STEP_WINDOW_MAX	1.6f
#define PAS_STARTUP_DETECT_RPM_DEFAULT	20.0f
#define PAS_STOP_TIMEOUT		0.5f

// Local-only PAS -> ERPM tracking knobs (not exposed via appconf)
#define PAS_ERPM_PER_RPM		100.0f
#define PAS_ERPM_BASE_MAX		9000.0f
#define PAS_ERPM_DELTA_THRESHOLD	2.0f
#define PAS_ERPM_BOOST_GAIN		50.0f
#define PAS_ERPM_BOOST_MAX		2000.0f
#define PAS_ERPM_BOOST_DECAY		0.95f
#define PAS_ERPM_STEP_LIMIT_RISE_LOW	3000.0f
#define PAS_ERPM_STEP_LIMIT_RISE_HIGH	25000.0f
#define PAS_ERPM_STEP_LIMIT_FALL	40000.0f
#define PAS_ERPM_STEP_THRESHOLD	6000.0f
#define PAS_CADENCE_MIN_ACTIVE		0.5f
#define PAS_STATIC_CADENCE_RPM_END	400.0f
#define PAS_STARTUP_ASSIST_WINDOW	0.8f
#define PAS_STARTUP_ASSIST_RPM_MIN	6.0f
#define PAS_STARTUP_ASSIST_RPM_MAX	35.0f
#define PAS_STARTUP_SEED_RPM_MIN	3.0f
#define PAS_STARTUP_SEED_RPM_SCALE	0.4f
#define PAS_ERPM_OVERBOOST_RATIO_CAP	1.35f
#define PAS_ERPM_OVERBOOST_ABS_MAX	1200.0f
#define PAS_ERPM_CADENCE_CAP_MARGIN	800.0f
// Threads
static THD_FUNCTION(pas_thread, arg);
__attribute__((section(".ram4"))) static THD_WORKING_AREA(pas_thread_wa, 512);
static void pas_sensor_update(float loop_dt);
static void pas_sensor_update_quadrature(float loop_dt);
static void pas_sensor_update_single(float loop_dt);
static void pas_single_exti_enable(bool enable);
static float pas_get_idle_timeout_limit(void);
static void pas_speed_state_reset(bool instant_zero);
static void pas_speed_target_update(float loop_dt, bool force_idle_now);
static void pas_startup_assist_trigger(void);
static float pas_get_startup_assist_rpm(void);
static float pas_get_seeded_start_rpm(void);
static float pas_get_reported_cadence(float measured);

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
static volatile float pas_base_erpm = 0.0f;
static volatile float pas_erpm_boost_offset = 0.0f;
static volatile float pas_erpm_target_filtered = 0.0f;
static volatile float pas_erpm_target_raw = 0.0f;
static float pas_cadence_rpm_prev = 0.0f;
static float pas_startup_assist_timer = 0.0f;
static float pas_start_rpm_config = PAS_STARTUP_DETECT_RPM_DEFAULT;
#if defined(HW_PAS1_PORT) && defined(HW_PAS1_PIN)
static volatile int pas_dbg_a_level = 0;
#else
static volatile int pas_dbg_a_level = -1;
#endif
#if defined(HW_PAS2_PORT) && defined(HW_PAS2_PIN)
static volatile int pas_dbg_b_level = 0;
#else
static volatile int pas_dbg_b_level = -1;
#endif
static float pas_boost_cadence_smooth = 0.0f;
static float pas_boost_state = 0.0f;
static bool pas_boost_initialized = false;
static volatile bool pas_force_idle = true;
static volatile uint32_t pas_step_count = 0;
static volatile float pas_time_last_real_step = 0.0f;
static volatile float pas_time_since_last_real_step = PAS_STOP_TIMEOUT;
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
static float pas_boost_compute_scale(float cadence_rpm, float loop_dt, bool enable);

static volatile float pas_target_erpm = 0.0f;
static float pas_step_window_config = PAS_REAL_STEP_WINDOW;

typedef struct {
	uint8_t last_state;
	uint8_t ref_state;
	float last_ref_timestamp;
	float event_period;
	float idle_time;
	bool has_period;
	bool seeded_start;
} pas_quadrature_state_t;

static pas_quadrature_state_t pas_quad_state = {
	.last_state = 0xFF,
	.ref_state = 0xFF,
	.last_ref_timestamp = 0.0f,
	.event_period = 0.0f,
	.idle_time = 0.0f,
	.has_period = false,
	.seeded_start = false
};

static void pas_quadrature_reset_state(void) {
	pas_quad_state.last_state = 0xFF;
	pas_quad_state.ref_state = 0xFF;
	pas_quad_state.last_ref_timestamp = 0.0f;
	pas_quad_state.event_period = 0.0f;
	pas_quad_state.idle_time = 0.0f;
	pas_quad_state.has_period = false;
	pas_quad_state.seeded_start = false;
}

static float pas_get_idle_timeout_limit(void) {
	float limit = PAS_STOP_TIMEOUT;
	if (pas_force_idle) {
		float extended = fmaxf(limit, pas_step_window_config);
		utils_truncate_number(&extended, PAS_STOP_TIMEOUT, PAS_REAL_STEP_WINDOW_MAX);
		limit = extended;
	}
	return limit;
}

static void pas_refresh_effective_rpm_end(void) {
	float rpm_end = PAS_STATIC_CADENCE_RPM_END;
	if (!isfinite(rpm_end) || rpm_end <= 0.0f) {
		rpm_end = PAS_STATIC_CADENCE_RPM_END;
	}
	float guard = pas_start_rpm_config + 1.0f;
	if (rpm_end < guard) {
		rpm_end = guard;
	}
	pas_rpm_end_effective = rpm_end;
}

static float pas_get_assist_ceiling(void) {
	if (config.sensor_type == PAS_SENSOR_TYPE_SINGLE_PIN_PPM) {
		return 1.0f;
	}
	return config.current_scaling;
}

static float pas_get_drive_reduction(void) {
	float ratio = config.pedal_rpm_end;
	if (!isfinite(ratio) || ratio <= 0.0f) {
		ratio = 1.0f;
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

static float pas_get_max_pas_erpm(void) {
	float max_erpm = mc_get_active_erpm_limit();
	if (!isfinite(max_erpm) || max_erpm <= 0.0f) {
		max_erpm = PAS_ERPM_BASE_MAX;
	}
	if (!isfinite(max_erpm) || max_erpm <= 0.0f) {
		max_erpm = PAS_ERPM_BASE_MAX;
	}
	return max_erpm;
}

static void pas_startup_assist_trigger(void) {
	pas_startup_assist_timer = PAS_STARTUP_ASSIST_WINDOW;
}

static float pas_get_startup_assist_rpm(void) {
	float rpm = pas_start_rpm_config;
	if (!isfinite(rpm) || rpm <= 0.0f) {
		rpm = PAS_STARTUP_DETECT_RPM_DEFAULT;
	}
	utils_truncate_number(&rpm, PAS_STARTUP_ASSIST_RPM_MIN, PAS_STARTUP_ASSIST_RPM_MAX);
	return rpm;
}

static float pas_get_seeded_start_rpm(void) {
	float rpm = pas_start_rpm_config * PAS_STARTUP_SEED_RPM_SCALE;
	if (!isfinite(rpm) || rpm <= 0.0f) {
		rpm = PAS_STARTUP_ASSIST_RPM_MIN;
	}
	float max_seed = pas_get_startup_assist_rpm();
	utils_truncate_number(&rpm, PAS_STARTUP_SEED_RPM_MIN, max_seed);
	return rpm;
}

static float pas_get_reported_cadence(float measured) {
	if (pas_startup_assist_timer > 0.0f) {
		float assist_rpm = pas_get_startup_assist_rpm();
		if (assist_rpm > measured) {
			return assist_rpm;
		}
	}
	return measured;
}

static void pas_speed_state_reset(bool instant_zero) {
	pas_base_erpm = 0.0f;
	pas_erpm_boost_offset = 0.0f;
	pas_erpm_target_raw = 0.0f;
	if (instant_zero) {
		pas_erpm_target_filtered = 0.0f;
	}
	pas_target_erpm = pas_erpm_target_filtered;
	pas_cadence_rpm_prev = pedal_rpm;
}

static void pas_speed_target_update(float loop_dt, bool force_idle_now) {
	float safe_dt = fmaxf(loop_dt, 1e-4f);
	float erpm_ceiling = pas_get_max_pas_erpm();

	if (pas_startup_assist_timer > 0.0f) {
		pas_startup_assist_timer = fmaxf(0.0f, pas_startup_assist_timer - safe_dt);
	}

	if (force_idle_now) {
		pas_startup_assist_timer = 0.0f;
		pas_speed_state_reset(true);
		return;
	}

	float cadence_measured = pedal_rpm;
	if (!isfinite(cadence_measured) || cadence_measured < PAS_CADENCE_MIN_ACTIVE) {
		cadence_measured = 0.0f;
	}
	if (pas_startup_assist_timer > 0.0f && cadence_measured >= pas_start_rpm_config) {
		pas_startup_assist_timer = 0.0f;
	}
	float cadence_for_drive = cadence_measured;
	if (pas_startup_assist_timer > 0.0f) {
		float assist_rpm = pas_get_startup_assist_rpm();
		if (cadence_for_drive < assist_rpm) {
			cadence_for_drive = assist_rpm;
		}
	}

	if (cadence_for_drive < PAS_CADENCE_MIN_ACTIVE) {
		pas_base_erpm = 0.0f;
		pas_erpm_boost_offset *= PAS_ERPM_BOOST_DECAY;
		pas_erpm_target_raw = 0.0f;
	} else {
		float gear_ratio = pas_get_drive_reduction();
		float pole_pairs = pas_get_motor_pole_pairs();
		float base_target = cadence_for_drive * gear_ratio * pole_pairs;
		utils_truncate_number(&base_target, 0.0f, erpm_ceiling);
		pas_base_erpm = base_target;

		float cadence_delta = cadence_measured - pas_cadence_rpm_prev;
		float erpm_delta = cadence_delta * PAS_ERPM_PER_RPM;
		float boost = pas_erpm_boost_offset * PAS_ERPM_BOOST_DECAY;
		if (erpm_delta > PAS_ERPM_DELTA_THRESHOLD) {
			boost += erpm_delta * PAS_ERPM_BOOST_GAIN;
		}
		utils_truncate_number(&boost, 0.0f, PAS_ERPM_BOOST_MAX);
		pas_erpm_boost_offset = boost;
		pas_cadence_rpm_prev = cadence_measured;

		pas_erpm_target_raw = base_target + boost;
		if (base_target > 1e-3f) {
			float ratio_cap = base_target * PAS_ERPM_OVERBOOST_RATIO_CAP;
			float abs_cap = base_target + PAS_ERPM_OVERBOOST_ABS_MAX;
			float margin_cap = base_target + PAS_ERPM_CADENCE_CAP_MARGIN;
			float target_limit = fminf(ratio_cap, abs_cap);
			target_limit = fminf(target_limit, margin_cap);
			if (pas_erpm_target_raw > target_limit) {
				pas_erpm_target_raw = target_limit;
			}
			float new_boost = pas_erpm_target_raw - base_target;
			utils_truncate_number(&new_boost, 0.0f, PAS_ERPM_BOOST_MAX);
			pas_erpm_boost_offset = new_boost;
		}
	}
	float target_clamped = pas_erpm_target_raw;
	utils_truncate_number(&target_clamped, 0.0f, erpm_ceiling);
	pas_erpm_target_raw = target_clamped;

	float target_delta = pas_erpm_target_raw - pas_erpm_target_filtered;
	float erpm_slew_state = fmaxf(pas_erpm_target_raw, pas_erpm_target_filtered);
	bool allow_high_slew = erpm_slew_state >= PAS_ERPM_STEP_THRESHOLD;
	float step_per_sec;
	if (target_delta >= 0.0f) {
		step_per_sec = allow_high_slew ? PAS_ERPM_STEP_LIMIT_RISE_HIGH : PAS_ERPM_STEP_LIMIT_RISE_LOW;
	} else {
		step_per_sec = PAS_ERPM_STEP_LIMIT_FALL;
	}
	float step_limit = step_per_sec * safe_dt;
	float filtered = pas_erpm_target_filtered;
	utils_step_towards(&filtered, pas_erpm_target_raw, step_limit);
	pas_erpm_target_filtered = filtered;
	pas_target_erpm = pas_erpm_target_filtered;
}

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

		ppm_pas_state.revolution_period = pedal_period;

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
	config.use_filter = false;
	ms_without_power = 0.0;
	output_current_rel = 0.0;
	pas_refresh_effective_rpm_end();

	float pulses_per_rev = fmaxf(1.0f, (float)config.magnets);
	// Use the existing pedal_rpm_start slider as the user-facing knob for
	// startup detection sensitivity so we stay compatible with older VESC Tool builds.
	float start_rpm = config.pedal_rpm_start;
	if (!isfinite(start_rpm) || start_rpm <= 0.1f) {
		start_rpm = PAS_STARTUP_DETECT_RPM_DEFAULT;
	}
	start_rpm = fmaxf(start_rpm, 1.0f);
	float detect_rpm = start_rpm;
	if (!isfinite(detect_rpm) || detect_rpm <= 0.1f) {
		detect_rpm = PAS_STARTUP_DETECT_RPM_DEFAULT;
	}
	utils_truncate_number(&detect_rpm, 1.0f, pas_rpm_end_effective);
	pas_start_rpm_config = detect_rpm;
	pas_refresh_effective_rpm_end();
	float detect_rps = detect_rpm / 60.0f;
	float edge_rate = detect_rps * pulses_per_rev;
	float est_event_period = (edge_rate > 1e-4f) ? (1.0f / edge_rate) : PAS_REAL_STEP_WINDOW_MAX;
	if (!isfinite(est_event_period) || est_event_period <= 0.0f) {
		est_event_period = PAS_REAL_STEP_WINDOW_MAX;
	}

	float step_window = est_event_period * 3.0f;
	if (!isfinite(step_window) || step_window <= 0.0f) {
		step_window = PAS_REAL_STEP_WINDOW_MAX;
	}
	utils_truncate_number(&step_window, PAS_REAL_STEP_WINDOW, PAS_REAL_STEP_WINDOW_MAX);
	pas_step_window_config = step_window;

	// a period longer than this should immediately reduce power to zero
	max_pulse_period = est_event_period * 1.2f;
	if (!isfinite(max_pulse_period) || max_pulse_period <= 0.0f) {
		max_pulse_period = PAS_REAL_STEP_WINDOW_MAX;
	}

	// if pedal spins at x3 the end rpm, assume its beyond limits
	float rpm_end_safe = fmaxf(pas_rpm_end_effective, start_rpm + 1.0f);
	float min_period_denom = (rpm_end_safe * 3.0f) / 60.0f;
	if (min_period_denom <= 1e-4f) {
		min_period_denom = 1e-4f;
	}
	min_pedal_period = 1.0f / min_period_denom;

	(config.invert_pedal_direction) ? (direction_conf = -1.0) : (direction_conf = 1.0);

#if defined(HW_PAS_PPM_EXTI_LINE)
	ppm_pas_state.period_valid = false;
	ppm_pas_state.revolution_period = 0.0f;
	ppm_pas_state.last_edge_time = timer_time_now();
#endif
	pas_step_count = 0;
	pas_time_last_real_step = 0.0f;
	pas_time_since_last_real_step = PAS_STOP_TIMEOUT;
	pas_quadrature_reset_state();
	pas_speed_state_reset(true);
	pas_force_idle = true;
	pas_boost_state = 0.0f;
	pas_boost_initialized = false;
	pas_startup_assist_timer = 0.0f;
	pedal_rpm = 0.0f;
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
	pas_quadrature_reset_state();
	pedal_rpm = 0.0f;
	pas_speed_state_reset(true);
	pas_force_idle = true;
	pas_boost_state = 0.0f;
	pas_boost_initialized = false;
	pas_startup_assist_timer = 0.0f;
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

float app_pas_get_target_erpm_base(void) {
	return pas_base_erpm;
}

float app_pas_get_target_erpm_boost(void) {
	return pas_erpm_boost_offset;
}

int app_pas_get_pas2_level(void) {
	return pas_dbg_b_level;
}

int app_pas_get_pas1_level(void) {
	return pas_dbg_a_level;
}

uint32_t app_pas_get_step_count(void) {
	return pas_step_count;
}

float app_pas_get_time_since_real_step(void) {
	return pas_time_since_last_real_step;
}

bool app_pas_is_forced_idle(void) {
	return pas_force_idle;
}

static void pas_sensor_update_quadrature(float loop_dt) {
#if defined(HW_PAS1_PORT) && defined(HW_PAS2_PORT)
	const int8_t QEM[] = {0,-1,1,2,1,0,2,-1,-1,2,0,1,2,1,-1,0};
	const float startup_rpm = 7.0f;
	const float stop_factor = 1.5f;
	static float rpm_filtered = 0.0f;

	pas_quad_state.idle_time += loop_dt;
	pas_time_since_last_real_step = pas_quad_state.idle_time;

	uint8_t pas1 = palReadPad(HW_PAS1_PORT, HW_PAS1_PIN) ? 1 : 0;
	uint8_t pas2 = palReadPad(HW_PAS2_PORT, HW_PAS2_PIN) ? 1 : 0;
	pas_dbg_a_level = pas1;
	pas_dbg_b_level = pas2;
	uint8_t new_state = (pas2 << 1) | pas1;

	if (pas_quad_state.last_state == 0xFF && new_state <= 3) {
		pas_quad_state.last_state = new_state;
	}

	if (new_state <= 3 && pas_quad_state.last_state <= 3 && new_state != pas_quad_state.last_state) {
		int idx = (pas_quad_state.last_state * 4) + new_state;
		int8_t step = QEM[idx];
		pas_quad_state.last_state = new_state;

		if (step == 2 || step == -2) {
			return;
		}

		int8_t direction = (int8_t)(direction_conf * (float)step);
		if (direction < 0) {
			pas_quadrature_reset_state();
			pedal_rpm = 0.0f;
			pas_force_idle = true;
			pas_boost_state = 0.0f;
			pas_boost_initialized = false;
			pas_startup_assist_timer = 0.0f;
			pas_time_last_real_step = 0.0f;
			pas_time_since_last_real_step = PAS_STOP_TIMEOUT;
			pas_speed_state_reset(true);
			return;
		} else if (direction == 0) {
			return;
		}

		pas_quad_state.idle_time = 0.0f;
		float now = (float)chVTGetSystemTimeX() / (float)CH_CFG_ST_FREQUENCY;
		if (pas_quad_state.ref_state == 0xFF) {
			pas_quad_state.ref_state = new_state;
			pas_quad_state.last_ref_timestamp = now;
			pas_quad_state.seeded_start = true;
		} else if (new_state == pas_quad_state.ref_state) {
			float event_period = now - pas_quad_state.last_ref_timestamp;
			pas_quad_state.last_ref_timestamp = now;

			float magnets = fmaxf(1.0f, (float)config.magnets);
			float min_event_period = min_pedal_period;
			if (magnets > 1.0f && min_event_period > 0.0f) {
				min_event_period /= magnets;
			}
			if (min_event_period < 1e-4f) {
				min_event_period = 1e-4f;
			}

			if (event_period >= min_event_period) {
				if (!pas_quad_state.has_period || !config.use_filter) {
					pas_quad_state.event_period = event_period;
				} else {
					UTILS_LP_FAST(pas_quad_state.event_period, event_period, 0.3f);
				}
				pas_quad_state.has_period = true;
				pas_quad_state.seeded_start = false;

				pas_time_last_real_step = now;
				pas_time_since_last_real_step = 0.0f;
				pas_step_count++;
				bool was_idle = pas_force_idle;
				pas_force_idle = false;
				if (was_idle) {
					pas_startup_assist_trigger();
				}
			}
		}
	}

	float rpm = pedal_rpm;
	if (pas_quad_state.has_period && pas_quad_state.event_period > 1e-4f) {
		float magnets = fmaxf(1.0f, (float)config.magnets);
		float rev_per_s = 1.0f / (pas_quad_state.event_period * magnets);
		rpm = rev_per_s * 60.0f;
	} else if (pas_quad_state.seeded_start) {
		rpm = fmaxf(rpm, startup_rpm);
	}

	if (config.use_filter) {
		UTILS_LP_FAST(rpm_filtered, rpm, 0.2f);
		rpm = rpm_filtered;
	}

	float timeout = max_pulse_period;
	if (pas_quad_state.has_period) {
		float magnets = fmaxf(1.0f, (float)config.magnets);
		float extended = pas_quad_state.event_period * magnets * stop_factor;
		timeout = fmaxf(timeout, extended);
	}

	if (pas_quad_state.idle_time > timeout) {
		pas_quadrature_reset_state();
		rpm = 0.0f;
		pas_force_idle = true;
		pas_boost_state = 0.0f;
		pas_boost_initialized = false;
		pas_startup_assist_timer = 0.0f;
		pas_time_last_real_step = 0.0f;
		pas_time_since_last_real_step = PAS_STOP_TIMEOUT;
		pas_speed_state_reset(true);
	}

	if (rpm < 0.001f) {
		rpm = 0.0f;
	}

	pedal_rpm = pas_get_reported_cadence(rpm);
#else
	(void)loop_dt;
	pedal_rpm = 0.0f;
	pas_speed_state_reset(true);
#endif
}

static void pas_sensor_update_single(float loop_dt) {
	(void)loop_dt;
#if defined(HW_PAS_PPM_EXTI_LINE)
	if (!ppm_pas_state.exti_enabled) {
		pedal_rpm = 0.0f;
		pas_speed_state_reset(true);
		return;
	}

	float idle_time = timer_seconds_elapsed_since(ppm_pas_state.last_edge_time);
	pas_time_since_last_real_step = idle_time;
	float idle_timeout = pas_get_idle_timeout_limit();
	if (idle_time > idle_timeout) {
		pas_force_idle = true;
		pas_boost_state = 0.0f;
		pas_boost_initialized = false;
		pas_startup_assist_timer = 0.0f;
	}
	if (idle_time > max_pulse_period) {
		pedal_rpm = 0.0f;
		ppm_pas_state.period_valid = false;
		pas_speed_state_reset(true);
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

	pedal_rpm = pas_get_reported_cadence(rpm);

	pas_time_last_real_step = (float)chVTGetSystemTimeX() / (float)CH_CFG_ST_FREQUENCY;
	pas_time_since_last_real_step = 0.0f;
	bool was_idle = pas_force_idle;
	pas_force_idle = false;
	if (was_idle) {
		pas_startup_assist_trigger();
	}
#else
	(void)loop_dt;
	pedal_rpm = 0.0f;
	pas_speed_state_reset(true);
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

static float pas_boost_compute_scale(float cadence_rpm, float loop_dt, bool enable) {
	if (!pas_boost_initialized) {
		pas_boost_cadence_smooth = cadence_rpm;
		pas_boost_initialized = true;
	}

	float safe_dt = fmaxf(loop_dt, 1e-4f);
	float alpha = safe_dt / fmaxf(PAS_BOOST_CADENCE_TC, 1e-3f);
	utils_truncate_number(&alpha, 0.0f, 1.0f);
	float prev_smooth = pas_boost_cadence_smooth;
	pas_boost_cadence_smooth += (cadence_rpm - pas_boost_cadence_smooth) * alpha;

	float cadence_delta = (pas_boost_cadence_smooth - prev_smooth) / safe_dt;
	float intent = 0.0f;
	if (enable && pas_boost_cadence_smooth >= PAS_BOOST_MIN_CADENCE_RPM) {
		float delta_above = cadence_delta - PAS_BOOST_INTENT_THRESHOLD;
		if (delta_above > 0.0f) {
			intent = delta_above / fmaxf(PAS_BOOST_INTENT_STRONG - PAS_BOOST_INTENT_THRESHOLD, 1.0f);
			utils_truncate_number(&intent, 0.0f, 1.0f);
		}
	}

	if (!enable) {
		intent = 0.0f;
	}

	float tau = (intent > pas_boost_state) ? PAS_BOOST_RISE_TC : PAS_BOOST_FALL_TC;
	float beta = safe_dt / fmaxf(tau, 1e-3f);
	utils_truncate_number(&beta, 0.0f, 1.0f);
	pas_boost_state += (intent - pas_boost_state) * beta;
	utils_truncate_number(&pas_boost_state, 0.0f, 1.0f);

	return 1.0f + (PAS_BOOST_MAX_FRACTION * pas_boost_state);
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
		float time_since_step = pas_time_since_last_real_step;
		bool timeout_idle = time_since_step > PAS_STOP_TIMEOUT;
		bool force_idle_now = pas_force_idle || timeout_idle;
		bool boost_enabled = (config.ctrl_type == PAS_CTRL_TYPE_CADENCE) && !force_idle_now;
		float boost_scale = pas_boost_compute_scale(pedal_rpm, loop_dt, boost_enabled);
		pas_speed_target_update(loop_dt, force_idle_now);
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
				float start_rpm = pas_start_rpm_config;
				float cadence_for_output = pedal_rpm;
				if (pas_startup_assist_timer > 0.0f) {
					float assist_rpm = pas_get_startup_assist_rpm();
					if (assist_rpm < start_rpm) {
						start_rpm = assist_rpm;
					}
					if (cadence_for_output < assist_rpm) {
						cadence_for_output = assist_rpm;
					}
				}
				if (cadence_rpm_end > (start_rpm + 1.0f)) {
					output = utils_map(cadence_for_output, start_rpm, cadence_rpm_end, 0.0, assist_ceiling * sub_scaling);
					utils_truncate_number(&output, 0.0, assist_ceiling * sub_scaling);
				} else {
					if (cadence_for_output > cadence_rpm_end) {
						output = assist_ceiling * sub_scaling;
					} else {
						output = 0.0;
					}
				}
				output *= boost_scale;
				utils_truncate_number(&output, 0.0, assist_ceiling * sub_scaling);
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

			if (force_idle_now) {
				output = 0.0f;
			}

		// Apply ramping
		static systime_t last_time = 0;
		static float output_ramp = 0.0;
		float ramp_time = fabsf(output) > fabsf(output_ramp) ? config.ramp_time_pos : config.ramp_time_neg;

			if (force_idle_now) {
				output_ramp = 0.0f;
				last_time = chVTGetSystemTimeX();
				output = 0.0f;
			} else if (ramp_time > 0.01) {
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
