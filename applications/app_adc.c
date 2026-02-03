/*
	Copyright 2016 - 2017 Benjamin Vedder	benjamin@vedder.se

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
#include "utils_sys.h"
#include "comm_can.h"
#include "hw.h"
#include "app_gear_detect.h"
#include <math.h>

// Settings
#define MAX_CAN_AGE						0.1
#define MIN_MS_WITHOUT_POWER			500
#define FILTER_SAMPLES					5
#define RPM_FILTER_SAMPLES				8
#define TC_DIFF_MAX_PASS				60  // TODO: move to app_conf
#define PAS_THROTTLE_IDLE_GATE		0.05f
#define PAS_FOLLOW_START_ROTATIONS_DEFAULT	0.15f
#define PAS_FOLLOW_IDLE_TIMEOUT_S_DEFAULT	0.6f
#define PAS_FOLLOW_BASE_CURRENT_FRAC_DEFAULT	0.2f
#define PAS_FOLLOW_BASE_RPM_FULL_DEFAULT	40.0f
#define PAS_FOLLOW_KP_A_PER_ERPM_DEFAULT	0.01f
#define PAS_FOLLOW_DEADBAND_ERPM_DEFAULT	30.0f
#define PAS_FOLLOW_TARGET_LEAD_DEFAULT	1.08f
#define PAS_FOLLOW_RAMP_UP_BASE_A_PER_S_DEFAULT	5.0f
#define PAS_FOLLOW_RAMP_UP_FULL_A_PER_S_DEFAULT	25.0f
#define PAS_FOLLOW_RAMP_UP_RISE_TIME_S_DEFAULT	2.0f
#define PAS_FOLLOW_RAMP_DOWN_A_PER_S_DEFAULT	60.0f
#define HAZ_THR_RELEASE_EPS_DEFAULT	0.01f
#define HAZ_THR_DUTY_GATE_SPAN_DEFAULT	0.08f
#define HAZ_THR_DUTY_GATE_MIN_SCALE_DEFAULT	0.60f
#define HAZ_THR_LAUNCH_BOOST_REL_DEFAULT	0.15f
#define HAZ_THR_LAUNCH_BOOST_THROTTLE_DEFAULT	0.15f
#define HAZ_THR_LAUNCH_BOOST_RELEASE_DUTY_DEFAULT	0.12f
#define HAZ_THR_LAUNCH_BOOST_RELEASE_ERPM_DEFAULT	250.0f
#define HAZ_THR_RAMP_UP_MIN_A_DEFAULT	10.0f
#define HAZ_THR_RAMP_UP_MID_A_DEFAULT	25.0f
#define HAZ_THR_RAMP_UP_MID_THROTTLE_DEFAULT	0.5f
#define HAZ_THR_RAMP_UP_MAX_A_DEFAULT	40.0f
#define HAZ_THR_RAMP_UP_LIMITED_A_DEFAULT	12.0f
#define HAZ_THR_RAMP_DOWN_A_DEFAULT	40.0f
#define HAZ_THR_FILTER_HZ_DEFAULT	12.0f

#define CTRL_USES_BUTTON(ctrl_type)(\
			ctrl_type == ADC_CTRL_TYPE_CURRENT_REV_BUTTON || \
			ctrl_type == ADC_CTRL_TYPE_CURRENT_REV_BUTTON_BRAKE_ADC || \
			ctrl_type == ADC_CTRL_TYPE_CURRENT_REV_BUTTON_BRAKE_CENTER || \
			ctrl_type == ADC_CTRL_TYPE_CURRENT_NOREV_BRAKE_BUTTON || \
			ctrl_type == ADC_CTRL_TYPE_DUTY_REV_BUTTON || \
			ctrl_type == ADC_CTRL_TYPE_PID_REV_BUTTON)

	static float haz_conf_clamp(float value, float fallback, float min_value, float max_value) {
		float out = isfinite(value) ? value : fallback;
		if (!isfinite(out)) {
			out = fallback;
		}
		if (max_value > min_value) {
			utils_truncate_number(&out, min_value, max_value);
		} else if (out < min_value) {
			out = min_value;
		}
		return out;
	}

	static float haz_pas_conf_clamp(float value, float fallback, float min_value, float max_value) {
		return haz_conf_clamp(value, fallback, min_value, max_value);
	}

// Threads
static THD_FUNCTION(adc_thread, arg);
__attribute__((section(".ram4"))) static THD_WORKING_AREA(adc_thread_wa, 512);

// Private variables
static volatile adc_config config;
static volatile float ms_without_power = 0.0;
static volatile float decoded_level = 0.0;
static volatile float read_voltage = 0.0;
static volatile float decoded_level2 = 0.0;
static volatile float read_voltage2 = 0.0;
static volatile float adc1_override = 0.0;
static volatile float adc2_override = 0.0;
static volatile bool use_rx_tx_as_buttons = false;
static volatile bool stop_now = true;
static volatile bool is_running = false;
static volatile int adc_detached = 0;
static volatile bool buttons_detached = false;
static volatile bool rev_override = false;
static volatile bool cc_override = false;
static volatile bool range_ok = true;

// =============================================================================
// ASSIST LEVEL SYSTEM
// Level 1-5: Power multiplier (20%, 40%, 60%, 80%, 100%)
// In street mode: Also apply 250W motor power limit
// In offroad mode: Level 5 = full power (VESC limits), 1-4 = multiplied
// =============================================================================
static volatile uint8_t assist_level = 5;  // Default to full power

// Get current assist level (1-5)
uint8_t app_adc_get_assist_level(void) {
	return assist_level;
}

// Set assist level from display (1-5)
void app_adc_set_assist_level(uint8_t level) {
	if (level >= 1 && level <= 5) {
		assist_level = level;
	}
}

// Get power multiplier for current assist level (0.2 to 1.0)
static float get_assist_power_multiplier(void) {
	// Level 1=20%, 2=40%, 3=60%, 4=80%, 5=100%
	return (float)assist_level * 0.2f;
}

// Hazza throttle state (current control)
typedef struct {
	float current_rel_cmd;
	// Street mode ERPM-follow state
	float target_erpm;
	float erpm_integral;
} haz_throttle_ctx_t;

static haz_throttle_ctx_t haz_throttle_ctx = {
	.current_rel_cmd = 0.0f,
	.target_erpm = 0.0f,
	.erpm_integral = 0.0f
};

static float haz_throttle_filtered = 0.0f;

typedef struct {
	float current_rel_cmd;
	float rotation_progress;
	float idle_time;
	bool engaged;
	float ramp_elapsed;
} haz_pas_follow_ctx_t;

static haz_pas_follow_ctx_t haz_pas_follow_ctx = {
	.current_rel_cmd = 0.0f,
	.rotation_progress = 0.0f,
	.idle_time = 0.0f,
	.engaged = false,
	.ramp_elapsed = 0.0f
};

// =============================================================================
// PAS DUTY MODE - Simple, Smooth Duty-Based PAS
// =============================================================================
// Concept: Match motor speed to pedals using duty commands (not current/PID)
// Much simpler than original - just smooth ramping to a cadence-based target
//
// target_duty = (pedal_rpm / max_rpm) * max_duty * power_scale
// Ramp smoothly to target. That's it.

typedef struct {
	float duty_cmd;              // Current duty output (smoothed)
	float target_duty;           // Target duty we're ramping toward
	float cadence_smooth;        // Smoothed pedal RPM
	float idle_time;             // Time since last pedal activity
	float startup_progress;      // Track startup rotations
	bool engaged;                // PAS currently active
	uint32_t last_step_count;    // For activity detection
} haz_pas_duty_ctx_t;

static haz_pas_duty_ctx_t haz_pas_duty_ctx = {
	.duty_cmd = 0.0f,
	.target_duty = 0.0f,
	.cadence_smooth = 0.0f,
	.idle_time = 0.0f,
	.startup_progress = 0.0f,
	.engaged = false,
	.last_step_count = 0
};

// Simple PAS duty - cadence-based duty output with smooth ramping
static float haz_pas_duty_process(volatile adc_config *conf, float dt_s) {
	if (!conf->haz_pas_duty_enabled) {
		haz_pas_duty_ctx.duty_cmd = 0.0f;
		return 0.0f;
	}

	// Get configs
	const volatile mc_configuration *mcconf = mc_interface_get_configuration();
	float max_duty = mcconf->l_max_duty;
	if (max_duty < 0.1f) max_duty = 0.95f;

	// Simple config
	const float ramp_up = fmaxf(conf->haz_pas_duty_ramp_up, 0.1f);
	const float ramp_down = fmaxf(conf->haz_pas_duty_ramp_down, 0.2f);
	const float power_scale = fmaxf(conf->haz_pas_duty_effort_gain, 0.1f);
	const float max_cadence = fmaxf(conf->haz_pas_duty_erpm_boost * 100.0f, 60.0f);
	const float idle_timeout = fmaxf(conf->haz_pas_duty_idle_timeout, 0.1f);

	// ========== SPEED LIMIT CHECK (STREET MODE ONLY) ==========
	// PAS cuts off at 15.5 mph in STREET MODE only
	// Offroad mode has no PAS speed limit
	float wheel_speed = fabsf(mc_interface_get_speed());
	float speed_power_scale = 1.0f;
	
	if (!mc_interface_is_offroad_mode()) {
		// Street mode - apply 15.5 mph limit
		const float pas_speed_limit_ms = 15.5f * 0.44704f;  // mph to m/s
		const float pas_speed_taper_start = pas_speed_limit_ms * 0.90f;  // Start tapering at 90%
		
		if (wheel_speed >= pas_speed_limit_ms) {
			// At or above limit - zero power
			speed_power_scale = 0.0f;
		} else if (wheel_speed > pas_speed_taper_start) {
			// In taper zone - smooth reduction
			speed_power_scale = (pas_speed_limit_ms - wheel_speed) / (pas_speed_limit_ms - pas_speed_taper_start);
		}
		
		// If over speed limit, force ramp down to zero
		if (speed_power_scale <= 0.0f) {
			haz_pas_duty_ctx.duty_cmd -= ramp_down * dt_s;
			if (haz_pas_duty_ctx.duty_cmd < 0.0f) haz_pas_duty_ctx.duty_cmd = 0.0f;
			return 0.0f;
		}
	}

	// Track step count and calculate step rate (more robust than single-step detection)
	uint32_t step_count = app_pas_get_step_count();
	uint32_t steps_this_cycle = 0;
	if (step_count >= haz_pas_duty_ctx.last_step_count) {
		steps_this_cycle = step_count - haz_pas_duty_ctx.last_step_count;
	}
	haz_pas_duty_ctx.last_step_count = step_count;

	// Calculate step rate (steps per second) - smooth it
	float step_rate = (float)steps_this_cycle / dt_s;
	static float step_rate_smooth = 0.0f;
	UTILS_LP_FAST(step_rate_smooth, step_rate, 0.3f);

	// With 40 magnets: 60 RPM = 40 steps/sec, 5 RPM = ~3.3 steps/sec
	// Consider idle if step rate drops below ~2 steps/sec (about 3 RPM)
	const float min_step_rate = 2.0f;
	
	// Track time below threshold
	if (step_rate_smooth < min_step_rate) {
		haz_pas_duty_ctx.idle_time += dt_s;
	} else {
		haz_pas_duty_ctx.idle_time = 0.0f;
	}

	bool is_idle = (haz_pas_duty_ctx.idle_time > idle_timeout);

	// Get pedal RPM with MEDIAN FILTER (outlier rejection)
	// Tunable via VESC Tool settings:
	// - PAS Duty: LP Smoothing (variation_threshold): 1-10 (1=responsive, 10=smooth)
	// - PAS Duty: Median Buffer (ramp_rise_time): 3-11 samples
	
	// Buffer size: direct integer from setting (3-11 samples)
	int buf_size = (int)conf->haz_pas_duty_ramp_rise_time;
	if (buf_size < 3) buf_size = 3;
	if (buf_size > 11) buf_size = 11;
	
	// LP filter: setting 1-10 maps to factor 0.5-0.01 (inverted: 1=responsive, 10=smooth)
	int lp_setting = (int)conf->haz_pas_duty_variation_threshold;
	if (lp_setting < 1) lp_setting = 1;
	if (lp_setting > 10) lp_setting = 10;
	float lp_factor = 0.5f - (lp_setting - 1) * (0.49f / 9.0f);  // 1→0.5, 10→0.01
	
	// Fixed 11-element buffer, always use full buffer for consistency
	// (changing buf_size only affects how many samples we consider for median)
	static float rpm_buffer[11] = {0};
	static int rpm_idx = 0;
	static int buf_filled = 0;  // Track how many valid samples we have
	
	float pedal_rpm_raw = app_pas_get_pedal_rpm();
	rpm_buffer[rpm_idx] = pedal_rpm_raw;
	rpm_idx = (rpm_idx + 1) % 11;  // Always wrap at 11
	if (buf_filled < 11) buf_filled++;
	
	// Use minimum of buf_size and buf_filled for median calculation
	int samples_to_use = (buf_filled < buf_size) ? buf_filled : buf_size;
	if (samples_to_use < 3) samples_to_use = 3;
	
	// Copy most recent samples for sorting
	float sorted[11];
	for (int i = 0; i < samples_to_use; i++) {
		int idx = (rpm_idx - 1 - i + 11) % 11;  // Work backwards from most recent
		sorted[i] = rpm_buffer[idx];
	}
	
	// Sort (simple insertion sort)
	for (int i = 1; i < samples_to_use; i++) {
		float key = sorted[i];
		int j = i - 1;
		while (j >= 0 && sorted[j] > key) {
			sorted[j + 1] = sorted[j];
			j--;
		}
		sorted[j + 1] = key;
	}
	
	// Take median (middle value)
	float pedal_rpm = sorted[samples_to_use / 2];
	
	// LP filter on top (tunable)
	UTILS_LP_FAST(haz_pas_duty_ctx.cadence_smooth, pedal_rpm, lp_factor);

	// Apply assist level multiplier (1-5 = 20-100%)
	// In offroad mode: Level 5 = normal VESC limits, 1-4 = reduced power
	// In street mode: Level is ignored (already limited by speed)
	float assist_mult = 1.0f;
	if (mc_interface_is_offroad_mode()) {
		assist_mult = get_assist_power_multiplier();
	}

	// Calculate target - zero if idle, otherwise from cadence
	float target_duty = 0.0f;
	if (!is_idle && haz_pas_duty_ctx.cadence_smooth > 3.0f) {
		float cadence_ratio = haz_pas_duty_ctx.cadence_smooth / max_cadence;
		if (cadence_ratio > 1.0f) cadence_ratio = 1.0f;
		target_duty = cadence_ratio * max_duty * power_scale * speed_power_scale * assist_mult;
	}

	// Ramp to target
	if (target_duty > haz_pas_duty_ctx.duty_cmd) {
		haz_pas_duty_ctx.duty_cmd += ramp_up * dt_s;
		if (haz_pas_duty_ctx.duty_cmd > target_duty) {
			haz_pas_duty_ctx.duty_cmd = target_duty;
		}
	} else {
		haz_pas_duty_ctx.duty_cmd -= ramp_down * dt_s;
		if (haz_pas_duty_ctx.duty_cmd < target_duty) {
			haz_pas_duty_ctx.duty_cmd = target_duty;
		}
	}

	// Clamp
	if (haz_pas_duty_ctx.duty_cmd < 0.0f) haz_pas_duty_ctx.duty_cmd = 0.0f;
	if (haz_pas_duty_ctx.duty_cmd > max_duty) haz_pas_duty_ctx.duty_cmd = max_duty;

	return haz_pas_duty_ctx.duty_cmd;
}

static bool haz_throttle_is_current_ctrl(adc_control_type ctrl) {
	switch (ctrl) {
	case ADC_CTRL_TYPE_CURRENT:
	case ADC_CTRL_TYPE_CURRENT_REV_CENTER:
	case ADC_CTRL_TYPE_CURRENT_REV_BUTTON:
	case ADC_CTRL_TYPE_CURRENT_REV_BUTTON_BRAKE_CENTER:
	case ADC_CTRL_TYPE_CURRENT_NOREV_BRAKE_CENTER:
	case ADC_CTRL_TYPE_CURRENT_NOREV_BRAKE_BUTTON:
	case ADC_CTRL_TYPE_CURRENT_NOREV_BRAKE_ADC:
	case ADC_CTRL_TYPE_CURRENT_REV_BUTTON_BRAKE_ADC:
		// Note: HYBRID_DUTY removed - it's now pure duty ramping mode
		return true;
	default:
		return false;
	}
}

static float haz_throttle_step(float current, float target, float rate_per_s, float dt_s) {
	float step = rate_per_s * dt_s;
	utils_truncate_number(&step, 0.0f, 1.0f);
	if (target > current) {
		current += step;
		if (current > target) current = target;
	} else {
		current -= step;
		if (current < target) current = target;
	}
	return current;
}

static float haz_throttle_process(float pwr_in, float dt_s) {
	const app_configuration *appconf = app_get_configuration();
	const adc_config *adc_conf = &appconf->app_adc_conf;
	const float release_eps = haz_conf_clamp(
		adc_conf->haz_throttle_release_eps,
		HAZ_THR_RELEASE_EPS_DEFAULT,
		0.0f,
		0.2f);
	const float duty_gate_span = haz_conf_clamp(
		adc_conf->haz_throttle_duty_gate_span,
		HAZ_THR_DUTY_GATE_SPAN_DEFAULT,
		0.0f,
		0.5f);
	const float duty_gate_min_scale = haz_conf_clamp(
		adc_conf->haz_throttle_duty_gate_min_scale,
		HAZ_THR_DUTY_GATE_MIN_SCALE_DEFAULT,
		0.0f,
		1.0f);
	const float launch_boost_rel = haz_conf_clamp(
		adc_conf->haz_throttle_launch_boost_rel,
		HAZ_THR_LAUNCH_BOOST_REL_DEFAULT,
		0.0f,
		1.0f);
	const float launch_boost_throttle = haz_conf_clamp(
		adc_conf->haz_throttle_launch_boost_throttle,
		HAZ_THR_LAUNCH_BOOST_THROTTLE_DEFAULT,
		0.01f,
		1.0f);
	const float launch_boost_release_duty = haz_conf_clamp(
		adc_conf->haz_throttle_launch_boost_release_duty,
		HAZ_THR_LAUNCH_BOOST_RELEASE_DUTY_DEFAULT,
		0.0f,
		0.6f);
	const float launch_boost_release_erpm = haz_conf_clamp(
		adc_conf->haz_throttle_launch_boost_release_erpm,
		HAZ_THR_LAUNCH_BOOST_RELEASE_ERPM_DEFAULT,
		0.0f,
		2000.0f);
	float ramp_up_min_a = haz_conf_clamp(
		adc_conf->haz_throttle_ramp_up_min_a,
		HAZ_THR_RAMP_UP_MIN_A_DEFAULT,
		0.0f,
		200.0f);
	float ramp_up_mid_a = haz_conf_clamp(
		adc_conf->haz_throttle_ramp_up_mid_a,
		HAZ_THR_RAMP_UP_MID_A_DEFAULT,
		0.0f,
		400.0f);
	float ramp_up_mid_throttle = haz_conf_clamp(
		adc_conf->haz_throttle_ramp_up_mid_throttle,
		HAZ_THR_RAMP_UP_MID_THROTTLE_DEFAULT,
		0.05f,
		0.95f);
	float ramp_up_max_a = haz_conf_clamp(
		adc_conf->haz_throttle_ramp_up_max_a,
		HAZ_THR_RAMP_UP_MAX_A_DEFAULT,
		ramp_up_min_a,
		400.0f);
	const float ramp_up_limited_a = haz_conf_clamp(
		adc_conf->haz_throttle_ramp_up_limited_a,
		HAZ_THR_RAMP_UP_LIMITED_A_DEFAULT,
		0.0f,
		400.0f);
	const float ramp_down_a = haz_conf_clamp(
		adc_conf->haz_throttle_ramp_down_a,
		HAZ_THR_RAMP_DOWN_A_DEFAULT,
		0.0f,
		400.0f);
	const float throttle_filter_hz = haz_conf_clamp(
		adc_conf->haz_throttle_filter_hz,
		HAZ_THR_FILTER_HZ_DEFAULT,
		0.0f,
		200.0f);

	if (ramp_up_max_a < ramp_up_min_a) {
		ramp_up_max_a = ramp_up_min_a;
	}

	if (pwr_in < 0.0f) {
		// braking/regen path untouched, decay drive command quickly
		haz_throttle_ctx.current_rel_cmd = haz_throttle_step(
			haz_throttle_ctx.current_rel_cmd, 0.0f, 10.0f, dt_s);
		return pwr_in;
	}

	float mag = fabsf(pwr_in);
	if (mag > 1.0f) {
		mag = 1.0f;
	}
	float alpha = throttle_filter_hz * dt_s;
	utils_truncate_number(&alpha, 0.0f, 1.0f);
	haz_throttle_filtered += (mag - haz_throttle_filtered) * alpha;
	mag = haz_throttle_filtered;
	if (mag < release_eps) {
		haz_throttle_ctx.current_rel_cmd = haz_throttle_step(
			haz_throttle_ctx.current_rel_cmd, 0.0f, ramp_down_a, dt_s);
		return haz_throttle_ctx.current_rel_cmd;
	}

	const volatile mc_configuration *mcconf = mc_interface_get_configuration();
	float max_phase_a = fabsf(mcconf->l_current_max);
	float max_batt_a = fabsf(mcconf->l_in_current_max);

	if (max_phase_a < 1e-3f || max_batt_a < 1e-3f) {
		haz_throttle_ctx.current_rel_cmd = 0.0f;
		return 0.0f;
	}

	float target_batt_a = mag * max_batt_a;
	float duty_now = fabsf(mc_interface_get_duty_cycle_now());
	if (duty_gate_span > 1e-5f && duty_now < duty_gate_span) {
		float ratio = duty_now / duty_gate_span;
		utils_truncate_number(&ratio, 0.0f, 1.0f);
		float scale = duty_gate_min_scale + (1.0f - duty_gate_min_scale) * ratio;
		target_batt_a *= scale;
	}

	float target_phase_a = target_batt_a * (max_phase_a / max_batt_a);
	if (target_phase_a > max_phase_a) {
		target_phase_a = max_phase_a;
	}

	float rpm_now_abs = fabsf(mc_interface_get_rpm());
	bool launch_boost_window = (duty_now < launch_boost_release_duty) &&
		(rpm_now_abs < launch_boost_release_erpm);
	if (launch_boost_window && mag > release_eps && launch_boost_throttle > 1e-4f) {
		float boost_mix = 1.0f - (mag / launch_boost_throttle);
		utils_truncate_number(&boost_mix, 0.0f, 1.0f);
		float min_phase_a = launch_boost_rel * max_phase_a * boost_mix;
		if (target_phase_a < min_phase_a) {
			target_phase_a = min_phase_a;
		}
		if (target_phase_a > max_phase_a) {
			target_phase_a = max_phase_a;
		}
	}

	float target_rel = target_phase_a / max_phase_a;
	float phase_now = fabsf(mc_interface_get_tot_current_filtered());
	float batt_now = fabsf(mc_interface_get_tot_current_in_filtered());
	bool phase_under = phase_now < max_phase_a;
	bool batt_under = batt_now < max_batt_a;

	// =========================================================================
	// SAFETY: Wheel speed sensor bypass detection (anti-pig feature)
	// If motor running in street mode for 3+ seconds with no wheel pulse, CUT IT
	// =========================================================================
	if (mc_street_mode_sensor_bypass_detected()) {
		// Sensor bypass detected - kill throttle output
		haz_throttle_ctx.target_erpm = 0.0f;
		haz_throttle_ctx.erpm_integral = 0.0f;
		return 0.0f;  // No power for you, pig!
	}

	// =========================================================================
	// STREET MODE: ERPM-FOLLOW (like PAS but for throttle)
	// Instead of ramping current which overshoots the limit, we follow a target
	// ERPM smoothly using PI control - just like the PAS algorithm.
	// This is only active when NOT in offroad mode.
	// Uses STREET_MODE_THROTTLE_ERPM_LIMIT (2000 = ~4 mph) not the PAS limit.
	// =========================================================================
	float erpm_limit = mc_get_throttle_erpm_limit();  // Throttle-specific limit (4 mph)
	// rpm_now_abs already defined above for launch boost
	
	if (erpm_limit > 0.0f && !mc_interface_is_offroad_mode()) {
		// ERPM-FOLLOW MODE: Throttle position maps to target ERPM
		// This naturally respects the limit without any overshoot/pulsing
		
		// Map throttle to target ERPM (0% = 0, 100% = erpm_limit)
		float new_target_erpm = mag * erpm_limit;
		
		// Smooth the target ERPM change (prevents jerky response)
		float target_slew_rate = erpm_limit * 2.0f;  // Can reach full speed in 0.5s
		float target_step = target_slew_rate * dt_s;
		if (new_target_erpm > haz_throttle_ctx.target_erpm) {
			haz_throttle_ctx.target_erpm += target_step;
			if (haz_throttle_ctx.target_erpm > new_target_erpm) {
				haz_throttle_ctx.target_erpm = new_target_erpm;
			}
		} else {
			haz_throttle_ctx.target_erpm -= target_step * 2.0f;  // Faster decel
			if (haz_throttle_ctx.target_erpm < new_target_erpm) {
				haz_throttle_ctx.target_erpm = new_target_erpm;
			}
		}
		if (haz_throttle_ctx.target_erpm < 0.0f) {
			haz_throttle_ctx.target_erpm = 0.0f;
		}
		
		// PI controller to track target ERPM (like PAS speed matching)
		float erpm_error = haz_throttle_ctx.target_erpm - rpm_now_abs;
		
		// PI gains (tuned for smooth response)
		const float kp = 0.0015f;  // Proportional: current per ERPM error
		const float ki = 0.0003f;  // Integral: accumulates for steady-state
		const float integral_limit = 0.5f;  // Max integral contribution (prevents windup)
		
		// Update integral with anti-windup
		haz_throttle_ctx.erpm_integral += erpm_error * dt_s;
		utils_truncate_number(&haz_throttle_ctx.erpm_integral, -integral_limit / ki, integral_limit / ki);
		
		// Reset integral if throttle released
		if (mag < release_eps) {
			haz_throttle_ctx.erpm_integral = 0.0f;
			haz_throttle_ctx.target_erpm = 0.0f;
		}
		
		// Calculate current command from PI
		float pi_output = kp * erpm_error + ki * haz_throttle_ctx.erpm_integral;
		
		// Add feedforward based on throttle position (gives instant response)
		float feedforward = mag * 0.3f;  // 30% of max current as baseline
		
		target_rel = feedforward + pi_output;
		utils_truncate_number(&target_rel, 0.0f, 1.0f);
		
		// Smooth the output (prevents any remaining jerkiness)
		float smooth_rate = 3.0f;  // Relative units per second
		if (target_rel > haz_throttle_ctx.current_rel_cmd) {
			haz_throttle_ctx.current_rel_cmd += smooth_rate * dt_s;
			if (haz_throttle_ctx.current_rel_cmd > target_rel) {
				haz_throttle_ctx.current_rel_cmd = target_rel;
			}
		} else {
			haz_throttle_ctx.current_rel_cmd -= smooth_rate * 2.0f * dt_s;
			if (haz_throttle_ctx.current_rel_cmd < target_rel) {
				haz_throttle_ctx.current_rel_cmd = target_rel;
			}
		}
		
		if (haz_throttle_ctx.current_rel_cmd < 0.0f) {
			haz_throttle_ctx.current_rel_cmd = 0.0f;
		}
		if (haz_throttle_ctx.current_rel_cmd > 1.0f) {
			haz_throttle_ctx.current_rel_cmd = 1.0f;
		}
		
		return haz_throttle_ctx.current_rel_cmd;
	}
	
	// =========================================================================
	// OFFROAD MODE: Original current ramping behavior
	// =========================================================================
	// Reset ERPM-follow state when entering offroad
	haz_throttle_ctx.target_erpm = 0.0f;
	haz_throttle_ctx.erpm_integral = 0.0f;

	// Two-segment ramp curve: min→mid→max based on throttle position
	float ramp_up_base_a;
	if (mag <= ramp_up_mid_throttle) {
		// Lower segment: interpolate from min to mid
		float t = mag / ramp_up_mid_throttle;
		ramp_up_base_a = ramp_up_min_a + (ramp_up_mid_a - ramp_up_min_a) * t;
	} else {
		// Upper segment: interpolate from mid to max
		float t = (mag - ramp_up_mid_throttle) / (1.0f - ramp_up_mid_throttle);
		ramp_up_base_a = ramp_up_mid_a + (ramp_up_max_a - ramp_up_mid_a) * t;
	}
	if (ramp_up_base_a > ramp_up_max_a) {
		ramp_up_base_a = ramp_up_max_a;
	}
	if (ramp_up_base_a < ramp_up_min_a) {
		ramp_up_base_a = ramp_up_min_a;
	}
	
	float ramp_up_a = (phase_under && batt_under) ? ramp_up_base_a : fminf(ramp_up_base_a, ramp_up_limited_a);
	float ramp_rate = ramp_up_a / max_phase_a;
	float ramp_down_rate = ramp_down_a / max_phase_a;

	if (target_rel < haz_throttle_ctx.current_rel_cmd) {
		haz_throttle_ctx.current_rel_cmd = haz_throttle_step(
			haz_throttle_ctx.current_rel_cmd, target_rel, ramp_down_rate, dt_s);
	} else {
		haz_throttle_ctx.current_rel_cmd = haz_throttle_step(
			haz_throttle_ctx.current_rel_cmd, target_rel, ramp_rate, dt_s);
	}

	if (haz_throttle_ctx.current_rel_cmd > 1.0f) {
		haz_throttle_ctx.current_rel_cmd = 1.0f;
	}

	return haz_throttle_ctx.current_rel_cmd;
}

static void haz_pas_follow_reset(void) {
	haz_pas_follow_ctx.current_rel_cmd = 0.0f;
	haz_pas_follow_ctx.rotation_progress = 0.0f;
	haz_pas_follow_ctx.idle_time = 0.0f;
	haz_pas_follow_ctx.engaged = false;
	haz_pas_follow_ctx.ramp_elapsed = 0.0f;
}

static float haz_pas_follow_process(float dt_s) {
	const app_configuration *appconf = app_get_configuration();
	const pas_config *pas_conf = &appconf->app_pas_conf;
	if (pas_conf->ctrl_type == PAS_CTRL_TYPE_NONE) {
		haz_pas_follow_reset();
		return 0.0f;
	}

	// ========== SPEED LIMIT CHECK (STREET MODE ONLY) ==========
	// PAS current mode also respects 15.5 mph limit in street mode
	if (!mc_interface_is_offroad_mode()) {
		const float pas_speed_limit_ms = 15.5f * 0.44704f;  // mph to m/s
		float wheel_speed = fabsf(mc_interface_get_speed());
		if (wheel_speed >= pas_speed_limit_ms) {
			// At or above limit - return zero, force ramp down
			haz_pas_follow_ctx.current_rel_cmd *= 0.9f; // Quick decay
			if (haz_pas_follow_ctx.current_rel_cmd < 0.01f) {
				haz_pas_follow_ctx.current_rel_cmd = 0.0f;
			}
			return 0.0f;
		}
	}

	const float start_rotations = haz_pas_conf_clamp(
		pas_conf->pas_follow_start_rotations,
		PAS_FOLLOW_START_ROTATIONS_DEFAULT,
		0.01f,
		2.0f);
	const float idle_timeout_s = haz_pas_conf_clamp(
		pas_conf->pas_follow_idle_timeout_s,
		PAS_FOLLOW_IDLE_TIMEOUT_S_DEFAULT,
		0.05f,
		5.0f);
	const float base_current_frac = haz_pas_conf_clamp(
		pas_conf->pas_follow_base_current_frac,
		PAS_FOLLOW_BASE_CURRENT_FRAC_DEFAULT,
		0.0f,
		1.0f);
	const float base_rpm_full = haz_pas_conf_clamp(
		pas_conf->pas_follow_base_rpm_full,
		PAS_FOLLOW_BASE_RPM_FULL_DEFAULT,
		1.0f,
		200.0f);
	const float kp_a_per_erpm = haz_pas_conf_clamp(
		pas_conf->pas_follow_kp_a_per_erpm,
		PAS_FOLLOW_KP_A_PER_ERPM_DEFAULT,
		0.0f,
		2.0f);
	const float deadband_erpm = haz_pas_conf_clamp(
		pas_conf->pas_follow_deadband_erpm,
		PAS_FOLLOW_DEADBAND_ERPM_DEFAULT,
		0.0f,
		500.0f);
	const float target_lead = haz_pas_conf_clamp(
		pas_conf->pas_follow_target_lead,
		PAS_FOLLOW_TARGET_LEAD_DEFAULT,
		0.5f,
		2.0f);
	float ramp_up_base_a = haz_pas_conf_clamp(
		pas_conf->pas_follow_ramp_up_base_a_per_s,
		PAS_FOLLOW_RAMP_UP_BASE_A_PER_S_DEFAULT,
		0.0f,
		200.0f);
	float ramp_up_full_a = haz_pas_conf_clamp(
		pas_conf->pas_follow_ramp_up_full_a_per_s,
		PAS_FOLLOW_RAMP_UP_FULL_A_PER_S_DEFAULT,
		0.0f,
		400.0f);
	const float ramp_up_rise_time_s = haz_pas_conf_clamp(
		pas_conf->pas_follow_ramp_up_rise_time_s,
		PAS_FOLLOW_RAMP_UP_RISE_TIME_S_DEFAULT,
		0.01f,
		10.0f);
	const float ramp_down_a = haz_pas_conf_clamp(
		pas_conf->pas_follow_ramp_down_a_per_s,
		PAS_FOLLOW_RAMP_DOWN_A_PER_S_DEFAULT,
		0.0f,
		400.0f);
	if (ramp_up_full_a < ramp_up_base_a) {
		ramp_up_full_a = ramp_up_base_a;
	}

	float target_erpm = app_pas_get_target_erpm();
	float pedal_rpm = app_pas_get_pedal_rpm();
	if (target_erpm <= 0.0f || pedal_rpm <= 0.1f) {
		haz_pas_follow_ctx.idle_time += dt_s;
		if (haz_pas_follow_ctx.idle_time > idle_timeout_s) {
			haz_pas_follow_reset();
		}
		return 0.0f;
	}

	haz_pas_follow_ctx.idle_time = 0.0f;
	float rotations = (pedal_rpm / 60.0f) * dt_s;
	haz_pas_follow_ctx.rotation_progress = fminf(haz_pas_follow_ctx.rotation_progress + rotations, start_rotations);
	if (!haz_pas_follow_ctx.engaged) {
		if (haz_pas_follow_ctx.rotation_progress >= start_rotations) {
			haz_pas_follow_ctx.engaged = true;
		} else {
			return 0.0f;
		}
	}

	const volatile mc_configuration *mcconf = mc_interface_get_configuration();
	float max_phase_a = fabsf(mcconf->l_current_max);
	if (max_phase_a < 0.1f) {
		return 0.0f;
	}

	float pas_max_current = pas_conf->max_current;
	float max_current = max_phase_a;
	if (isfinite(pas_max_current) && pas_max_current > 0.0f) {
		if (pas_max_current <= 1.0f) {
			float ratio = pas_max_current;
			utils_truncate_number(&ratio, 0.0f, 1.0f);
			max_current = ratio * max_phase_a;
		} else {
			max_current = fminf(fabsf(pas_max_current), max_phase_a);
		}
	}
	if (max_current < 1e-3f) {
		max_current = max_phase_a;
	}

	float target_erpm_lead = target_erpm * target_lead;
	float erpm_now = mc_interface_get_rpm();
	float erpm_err = target_erpm_lead - erpm_now;
	if (fabsf(erpm_err) < deadband_erpm) {
		erpm_err = 0.0f;
	}

	float trim_current = erpm_err * kp_a_per_erpm;
	float base_current = max_current * base_current_frac;
	if (pedal_rpm < base_rpm_full) {
		float scale = base_rpm_full > 1e-3f ? (pedal_rpm / base_rpm_full) : 0.0f;
		utils_truncate_number(&scale, 0.0f, 1.0f);
		base_current *= scale;
	}

	float target_current = base_current + trim_current;
	utils_truncate_number(&target_current, 0.0f, max_current);
	float target_rel = target_current / max_phase_a;
	if (target_rel >= haz_pas_follow_ctx.current_rel_cmd) {
		haz_pas_follow_ctx.ramp_elapsed += dt_s;
		if (haz_pas_follow_ctx.ramp_elapsed > ramp_up_rise_time_s) {
			haz_pas_follow_ctx.ramp_elapsed = ramp_up_rise_time_s;
		}
	} else {
		haz_pas_follow_ctx.ramp_elapsed = 0.0f;
	}

	float ramp_rate = 0.0f;
	if (target_rel >= haz_pas_follow_ctx.current_rel_cmd) {
		float ramp_ratio = 1.0f;
		if (ramp_up_rise_time_s > 1e-3f) {
			ramp_ratio = haz_pas_follow_ctx.ramp_elapsed / ramp_up_rise_time_s;
		}
		utils_truncate_number(&ramp_ratio, 0.0f, 1.0f);
		float ramp_up_a = ramp_up_base_a + (ramp_up_full_a - ramp_up_base_a) * ramp_ratio;
		ramp_rate = ramp_up_a / max_phase_a;
	} else {
		ramp_rate = ramp_down_a / max_phase_a;
	}
	haz_pas_follow_ctx.current_rel_cmd = haz_throttle_step(
		haz_pas_follow_ctx.current_rel_cmd,
		target_rel,
		ramp_rate,
		dt_s);

	return haz_pas_follow_ctx.current_rel_cmd;
}

static bool haz_pas_try_takeover(float *pwr, float *current_rel, float loop_dt) {
	if (*pwr < 0.0f) {
		haz_pas_follow_reset();
		return false;
	}

	if (!app_pas_is_running()) {
		haz_pas_follow_reset();
		return false;
	}

	if (fabsf(*pwr) < PAS_THROTTLE_IDLE_GATE) {
		float pas_rel = haz_pas_follow_process(loop_dt);
		if (pas_rel > 0.0f) {
			*pwr = pas_rel;
			if (current_rel) {
				*current_rel = pas_rel;
			}
			return true;
		}
		return false;
	}

	haz_pas_follow_reset();
	return false;
}

void app_adc_configure(adc_config *conf) {
	if (!buttons_detached && (((conf->buttons >> 0) & 1) || CTRL_USES_BUTTON(conf->ctrl_type))) {
		if (use_rx_tx_as_buttons) {
			palSetPadMode(HW_UART_TX_PORT, HW_UART_TX_PIN, PAL_MODE_INPUT_PULLUP);
			palSetPadMode(HW_UART_RX_PORT, HW_UART_RX_PIN, PAL_MODE_INPUT_PULLUP);
		} else {
			palSetPadMode(HW_ICU_GPIO, HW_ICU_PIN, PAL_MODE_INPUT_PULLUP);
		}
	}

	config = *conf;
	ms_without_power = 0.0;
}

void app_adc_start(bool use_rx_tx) {
#ifdef HW_ADC_EXT_GPIO
	palSetPadMode(HW_ADC_EXT_GPIO, HW_ADC_EXT_PIN, PAL_MODE_INPUT_ANALOG);
#endif
#ifdef HW_ADC_EXT2_GPIO
	palSetPadMode(HW_ADC_EXT2_GPIO, HW_ADC_EXT2_PIN, PAL_MODE_INPUT_ANALOG);
#endif

	if (buttons_detached) {
		use_rx_tx_as_buttons = false;
	} else {
		use_rx_tx_as_buttons = use_rx_tx;
	}

	stop_now = false;
	chThdCreateStatic(adc_thread_wa, sizeof(adc_thread_wa), NORMALPRIO, adc_thread, NULL);
}

void app_adc_stop(void) {
	stop_now = true;
	while (is_running) {
		chThdSleepMilliseconds(1);
	}
}

bool app_adc_is_running(void) {
	return is_running;
}

float app_adc_get_decoded_level(void) {
	return decoded_level;
}

float app_adc_get_voltage(void) {
	return read_voltage;
}

float app_adc_get_decoded_level2(void) {
	return decoded_level2;
}

float app_adc_get_voltage2(void) {
	return read_voltage2;
}

void app_adc_detach_adc(int detach) {
	adc_detached = detach;
	timeout_reset();
}

void app_adc_adc1_override(float val) {
	utils_truncate_number(&val, 0, 3.3);
	adc1_override = val;
	timeout_reset();
}

void app_adc_adc2_override(float val) {
	utils_truncate_number(&val, 0, 3.3);
	adc2_override = val;
	timeout_reset();
}

void app_adc_detach_buttons(bool state) {
	buttons_detached = state;
	timeout_reset();
}

void app_adc_rev_override(bool state) {
	rev_override = state;
	timeout_reset();
}

void app_adc_cc_override(bool state) {
	cc_override = state;
	timeout_reset();
}

bool app_adc_range_ok(void) {
	return range_ok;
}

static THD_FUNCTION(adc_thread, arg) {
	(void)arg;

	chRegSetThreadName("APP_ADC");
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

		// Reset throttle-only ERPM cap; it will be re-enabled later in this loop
		// only if the throttle (not PAS) is requesting forward torque.
		mc_interface_set_throttle_limit_active(false);

		// For safe start when fault codes occur
		if (mc_interface_get_fault() != FAULT_CODE_NONE && config.safe_start != SAFE_START_NO_FAULT) {
			ms_without_power = 0;
		}

		// Read the external ADC pin voltage
		float pwr = ADC_VOLTS(ADC_IND_EXT);

		// Override pwr value, when used from LISP
		if (adc_detached == 1 || adc_detached == 2) {
			pwr = adc1_override;
		}

		// Read voltage and range check
		static float read_filter = 0.0;
		UTILS_LP_MOVING_AVG_APPROX(read_filter, pwr, FILTER_SAMPLES);

		if (config.use_filter) {
			read_voltage = read_filter;
		} else {
			read_voltage = pwr;
		}

		range_ok = read_voltage >= config.voltage_min && read_voltage <= config.voltage_max;

		// Map the read voltage
		switch (config.ctrl_type) {
		case ADC_CTRL_TYPE_CURRENT_REV_CENTER:
		case ADC_CTRL_TYPE_CURRENT_REV_BUTTON_BRAKE_CENTER:
		case ADC_CTRL_TYPE_CURRENT_NOREV_BRAKE_CENTER:
		case ADC_CTRL_TYPE_DUTY_REV_CENTER:
		case ADC_CTRL_TYPE_PID_REV_CENTER:
			// Mapping with respect to center voltage
			if (pwr < config.voltage_center) {
				pwr = utils_map(pwr, config.voltage_start,
						config.voltage_center, 0.0, 0.5);
			} else {
				pwr = utils_map(pwr, config.voltage_center,
						config.voltage_end, 0.5, 1.0);
			}
			break;

		default:
			// Linear mapping between the start and end voltage
			pwr = utils_map(pwr, config.voltage_start, config.voltage_end, 0.0, 1.0);
			break;
		}

		// Optionally apply a filter
		static float pwr_filter = 0.0;
		UTILS_LP_MOVING_AVG_APPROX(pwr_filter, pwr, FILTER_SAMPLES);

		if (config.use_filter) {
			pwr = pwr_filter;
		}

		// Truncate the read voltage
		utils_truncate_number(&pwr, 0.0, 1.0);

		// Optionally invert the read voltage
		if (config.voltage_inverted) {
			pwr = 1.0 - pwr;
		}

		decoded_level = pwr;

		// Read the external ADC pin and convert the value to a voltage.
#ifdef ADC_IND_EXT2
		float brake = ADC_VOLTS(ADC_IND_EXT2);
#else
		float brake = 0.0;
#endif

#ifdef HW_HAS_BRAKE_OVERRIDE
		hw_brake_override(&brake);
#endif

		// Override brake value, when used from LISP
		if (adc_detached == 1 || adc_detached == 3) {
			brake = adc2_override;
		}

		read_voltage2 = brake;

		// Optionally apply a filter
		static float filter_val_2 = 0.0;
		UTILS_LP_MOVING_AVG_APPROX(filter_val_2, brake, FILTER_SAMPLES);

		if (config.use_filter) {
			brake = filter_val_2;
		}

		// Map and truncate the read voltage
		brake = utils_map(brake, config.voltage2_start, config.voltage2_end, 0.0, 1.0);
		utils_truncate_number(&brake, 0.0, 1.0);

		// Optionally invert the read voltage
		if (config.voltage2_inverted) {
			brake = 1.0 - brake;
		}

		decoded_level2 = brake;

		// Read the button pins
		bool cc_button = false;
		bool rev_button = false;
		if (use_rx_tx_as_buttons) {
			cc_button = !palReadPad(HW_UART_TX_PORT, HW_UART_TX_PIN);
			if ((config.buttons >> 1) & 1) {
				cc_button = !cc_button;
			}
			rev_button = !palReadPad(HW_UART_RX_PORT, HW_UART_RX_PIN);
			if ((config.buttons >> 2) & 1) {
				rev_button = !rev_button;
			}
		} else {
			// When only one button input is available, use it differently depending on the control mode
			if (config.ctrl_type == ADC_CTRL_TYPE_CURRENT_REV_BUTTON ||
                    config.ctrl_type == ADC_CTRL_TYPE_CURRENT_REV_BUTTON_BRAKE_CENTER ||
					config.ctrl_type == ADC_CTRL_TYPE_CURRENT_NOREV_BRAKE_BUTTON ||
					config.ctrl_type == ADC_CTRL_TYPE_DUTY_REV_BUTTON ||
					config.ctrl_type == ADC_CTRL_TYPE_PID_REV_BUTTON) {
				rev_button = !palReadPad(HW_ICU_GPIO, HW_ICU_PIN);
				if ((config.buttons >> 2) & 1) {
					rev_button = !rev_button;
				}
			} else {
				cc_button = !palReadPad(HW_ICU_GPIO, HW_ICU_PIN);
				if ((config.buttons >> 1) & 1) {
					cc_button = !cc_button;
				}
			}
		}

		// Override button values, when used from LISP
		if (buttons_detached) {
			cc_button = cc_override;
			rev_button = rev_override;
			if ((config.buttons >> 1) & 1) {
				cc_button = !cc_button;
			}
			if ((config.buttons >> 2) & 1) {
				rev_button = !rev_button;
			}
		}

		if (!((config.buttons >> 0) & 1)) {
			cc_button = false;
		}

		// All pins and buttons are still decoded for debugging, even
		// when output is disabled.
		if (app_is_output_disabled()) {
			continue;
		}

		if (adc_detached && timeout_has_timeout()) {
			continue;
		}

		switch (config.ctrl_type) {
		case ADC_CTRL_TYPE_CURRENT_REV_CENTER:
		case ADC_CTRL_TYPE_CURRENT_REV_BUTTON_BRAKE_CENTER:
		case ADC_CTRL_TYPE_CURRENT_NOREV_BRAKE_CENTER:
		case ADC_CTRL_TYPE_DUTY_REV_CENTER:
		case ADC_CTRL_TYPE_PID_REV_CENTER:
			// Scale the voltage and set 0 at the center
			pwr *= 2.0;
			pwr -= 1.0;
			break;

		case ADC_CTRL_TYPE_CURRENT_NOREV_BRAKE_ADC:
		case ADC_CTRL_TYPE_CURRENT_REV_BUTTON_BRAKE_ADC:
			pwr -= brake;
			break;

		case ADC_CTRL_TYPE_CURRENT_REV_BUTTON:
		case ADC_CTRL_TYPE_CURRENT_NOREV_BRAKE_BUTTON:
		case ADC_CTRL_TYPE_DUTY_REV_BUTTON:
		case ADC_CTRL_TYPE_PID_REV_BUTTON:
			// Invert the voltage if the button is pressed
			if (rev_button) {
				pwr = -pwr;
			}
			break;

		default:
			break;
		}

		// Apply deadband
		utils_deadband(&pwr, config.hyst, 1.0);

		// Apply throttle curve
		pwr = utils_throttle_curve(pwr, config.throttle_exp, config.throttle_exp_brake, config.throttle_exp_mode);

		const float loop_dt = (float)sleep_time / (float)CH_CFG_ST_FREQUENCY;
		if (haz_throttle_is_current_ctrl(config.ctrl_type)) {
			pwr = haz_throttle_process(pwr, loop_dt);
		}

		float current_rel = 0.0;
		bool current_mode = false;
		bool current_mode_brake = false;
		const volatile mc_configuration *mcconf = mc_interface_get_configuration();
		const float rpm_now = mc_interface_get_rpm();
		bool send_duty = false;

		// Use the filtered and mapped voltage for control according to the configuration.
		switch (config.ctrl_type) {
		case ADC_CTRL_TYPE_CURRENT:
		case ADC_CTRL_TYPE_CURRENT_REV_CENTER:
		case ADC_CTRL_TYPE_CURRENT_REV_BUTTON:
			current_mode = true;
			if (pwr >= 0.0f) {
				const float throttle_cmd = pwr;
				if (!haz_pas_try_takeover(&pwr, &current_rel, loop_dt)) {
					current_rel = pwr;
					if (throttle_cmd > 0.0f) {
						mc_interface_set_throttle_limit_active(true);
					}
				}
			} else {
				haz_pas_follow_reset();
				current_rel = pwr;
			}

			if (fabsf(pwr) < 0.001) {
				ms_without_power += (1000.0 * (float)sleep_time) / (float)CH_CFG_ST_FREQUENCY;
			}
			break;

		case ADC_CTRL_TYPE_CURRENT_REV_BUTTON_BRAKE_CENTER:
		case ADC_CTRL_TYPE_CURRENT_NOREV_BRAKE_CENTER:
		case ADC_CTRL_TYPE_CURRENT_NOREV_BRAKE_BUTTON:
		case ADC_CTRL_TYPE_CURRENT_NOREV_BRAKE_ADC:
		case ADC_CTRL_TYPE_CURRENT_REV_BUTTON_BRAKE_ADC:
			current_mode = true;
			if (pwr >= 0.0f) {
				const float throttle_cmd = pwr;
				if (!haz_pas_try_takeover(&pwr, &current_rel, loop_dt)) {
					current_rel = pwr;
					if (throttle_cmd > 0.0f) {
						mc_interface_set_throttle_limit_active(true);
					}
				}
			} else {
				haz_pas_follow_reset();
				current_rel = fabsf(pwr);
				current_mode_brake = true;
			}

			if (pwr < 0.001) {
				ms_without_power += (1000.0 * (float)sleep_time) / (float)CH_CFG_ST_FREQUENCY;
			}

			if ((config.ctrl_type == ADC_CTRL_TYPE_CURRENT_REV_BUTTON_BRAKE_ADC ||
			    config.ctrl_type == ADC_CTRL_TYPE_CURRENT_REV_BUTTON_BRAKE_CENTER) && rev_button) {
				current_rel = -current_rel;
			}
			break;

		case ADC_CTRL_TYPE_DUTY:
		case ADC_CTRL_TYPE_DUTY_REV_CENTER:
		case ADC_CTRL_TYPE_DUTY_REV_BUTTON:
			if (fabsf(pwr) < 0.001) {
				ms_without_power += (1000.0 * (float)sleep_time) / (float)CH_CFG_ST_FREQUENCY;
			}

			if (!(ms_without_power < MIN_MS_WITHOUT_POWER && config.safe_start)) {
				if (pwr > 0.0f) {
					mc_interface_set_throttle_limit_active(true);
				}
				mc_interface_set_duty(utils_map(pwr, -1.0, 1.0, -mcconf->l_max_duty, mcconf->l_max_duty));
				send_duty = true;
			}
			break;

		case ADC_CTRL_TYPE_CURRENT_HYBRID_DUTY:
			// OSF-style duty ramping: Throttle position = duty TARGET
			// Ramp speed depends on gap between current and target duty
			// PAS takes over with current control ONLY when throttle is idle
			// NEVER applies both duty and current simultaneously
			// Freewheel catch: spin motor to match wheel speed before engaging drivetrain
			{
				static float osf_duty_ramped = 0.0f;
				static int freewheel_catch_state = 0;  // 0=normal, 1=spinning up, 2=engaged
				static float freewheel_catch_duty = 0.0f;
				static int freewheel_catch_gear = 0;
				static float throttle_slam_initial_gap = 0.0f;  // Track initial gap for slam detection
				static float last_duty_target = 0.0f;           // Track target changes
				const float dt_s = (float)sleep_time / (float)CH_CFG_ST_FREQUENCY;

				float throttle_mag = fabsf(pwr);
				if (throttle_mag > 1.0f) throttle_mag = 1.0f;

				// Throttle ALWAYS takes priority - PAS only when throttle is truly zero
				bool throttle_active = (throttle_mag > 0.001f);
				
				// Reset PAS if throttle is being used at all
				if (throttle_active) {
					haz_pas_follow_reset();
				}

				if (throttle_mag < 0.001f) {
					ms_without_power += (1000.0f * (float)sleep_time) / (float)CH_CFG_ST_FREQUENCY;
				}

				// Freewheel catch logic
				const bool fwc_enabled = config.haz_freewheel_catch_enabled;
				const float fwc_current_threshold = config.haz_freewheel_catch_current_threshold;
				const float fwc_erpm_offset = config.haz_freewheel_catch_erpm_offset;
				const float fwc_final_rate = config.haz_freewheel_catch_final_rate;
				const float fwc_modifier = config.haz_freewheel_catch_modifier;
				
				float current_erpm = fabsf(mc_interface_get_rpm());
				float motor_current = fabsf(mc_interface_get_tot_current_filtered());
				float wheel_speed_kph = mc_interface_get_speed() * 3.6f;  // m/s to km/h
				
				// Get or remember gear for freewheel catch
				int current_gear = app_gear_detect_get_last_gear();
				if (throttle_active && freewheel_catch_gear == 0 && current_gear > 0) {
					freewheel_catch_gear = current_gear;
				}
				if (!throttle_active) {
					// Not throttling - update gear from current detection if available
					int detected = app_gear_detect_get_current();
					if (detected > 0) {
						freewheel_catch_gear = detected;
					}
					freewheel_catch_state = 0;  // Reset state when throttle released
					freewheel_catch_duty = 0.0f;
				}

				if (!(ms_without_power < MIN_MS_WITHOUT_POWER && config.safe_start)) {
					if (throttle_active) {
						// Check if freewheel catch should activate
						float target_erpm = 0.0f;
						bool fwc_active = false;
						
						if (fwc_enabled && freewheel_catch_gear > 0 && wheel_speed_kph > 3.0f) {
							target_erpm = app_gear_detect_target_erpm(wheel_speed_kph, freewheel_catch_gear) * fwc_modifier;
							
							// Freewheel condition: motor ERPM much lower than target (chain slack)
							bool is_freewheel = (current_erpm < target_erpm * 0.5f) && (freewheel_catch_state < 2);
							
							if (is_freewheel && freewheel_catch_state == 0) {
								// Start catch sequence
								freewheel_catch_state = 1;
								freewheel_catch_duty = fabsf(mc_interface_get_duty_cycle_now());
							}
							
							if (freewheel_catch_state == 1) {
								// Spinning up to catch wheel
								fwc_active = true;
								
								// Calculate duty ramp rate based on ERPM gap
								float erpm_gap = target_erpm - current_erpm;
								float ramp_rate;
								
								if (erpm_gap > fwc_erpm_offset) {
									// Far from target - use fast ramp (normal hybrid ramp up fast)
									ramp_rate = config.haz_hybrid_ramp_up_fast;
								} else {
									// Close to target - use slow final approach
									ramp_rate = fwc_final_rate;
								}
								
								freewheel_catch_duty += ramp_rate * dt_s;
								if (freewheel_catch_duty > mcconf->l_max_duty * throttle_mag) {
									freewheel_catch_duty = mcconf->l_max_duty * throttle_mag;
								}
								
								// Check for engagement (current spike)
								if (motor_current > fwc_current_threshold && current_erpm > target_erpm * 0.8f) {
									// Chain engaged! Transition to normal control
									freewheel_catch_state = 2;
									osf_duty_ramped = freewheel_catch_duty;
								}
								
								// Apply catch duty
								mc_interface_set_throttle_limit_active(true);
								mc_interface_set_duty(pwr >= 0.0f ? freewheel_catch_duty : -freewheel_catch_duty);
								send_duty = true;
							}
						}
						
						if (!fwc_active) {
							// NORMAL THROTTLE MODE: pure duty ramping
							
							// ========== STREET MODE SPEED LIMITS (mph-based) ==========
							// Throttle: 3 mph limit in street mode
							// No pulsing - just smoothly limit the target duty
							const float throttle_speed_limit_mph = 3.0f;
							const float throttle_speed_limit_ms = throttle_speed_limit_mph * 0.44704f;
							const float throttle_taper_start = throttle_speed_limit_ms * 0.80f;
							
							float wheel_speed = fabsf(mc_interface_get_speed());
							float speed_scale = 1.0f;
							
							if (!mc_interface_is_offroad_mode()) {
								// Street mode - apply throttle speed limit
								if (wheel_speed >= throttle_speed_limit_ms) {
									// At or above limit - zero throttle
									speed_scale = 0.0f;
								} else if (wheel_speed > throttle_taper_start) {
									// Taper zone - smooth reduction
									speed_scale = (throttle_speed_limit_ms - wheel_speed) / (throttle_speed_limit_ms - throttle_taper_start);
								}
							}
							
							// Apply assist level multiplier (1-5 = 20-100%)
							// In offroad mode: Level 5 = normal VESC limits, 1-4 = reduced power
							// In street mode: Level is ignored (already limited by speed)
							float assist_mult = 1.0f;
							if (mc_interface_is_offroad_mode()) {
								assist_mult = get_assist_power_multiplier();
							}
							// Street mode: already speed-limited, no assist scaling needed
							
							float duty_target = throttle_mag * mcconf->l_max_duty * speed_scale * assist_mult;
							float duty_gap = fabsf(duty_target - osf_duty_ramped);


							// Configurable ramp rates (duty/s)
							// Street mode: VERY slow ramp (0.05/s) to avoid slingshot past speed limit
							// with low-resolution 1 pulse/rotation speed sensor
							const bool street_mode = !mc_interface_is_offroad_mode();
							const float ramp_up_slow = street_mode ? 0.05f : config.haz_hybrid_ramp_up_slow;
							const float ramp_up_fast = street_mode ? 0.05f : config.haz_hybrid_ramp_up_fast;
							const float ramp_down_slow = config.haz_hybrid_ramp_down_slow;
							const float ramp_down_fast = config.haz_hybrid_ramp_down_fast;

							// Scale ramp by how far we are from target
							float gap_ratio = duty_gap / mcconf->l_max_duty;
							if (gap_ratio > 1.0f) gap_ratio = 1.0f;
							
							// Slam detection: if throttle target jumped significantly, remember it
							const float slam_threshold = 0.25f;  // 25% duty change = slam
							float target_change = fabsf(duty_target - last_duty_target);
							if (target_change > slam_threshold * mcconf->l_max_duty) {
								// New slam detected - remember the initial gap
								throttle_slam_initial_gap = duty_gap;
							}
							last_duty_target = duty_target;
							
							// If this was a big slam (initial gap > 30%), stay fast all the way
							// Otherwise use squared blend for fine control
							const float slam_fast_threshold = 0.30f;  // 30% initial gap = full fast mode
							if (throttle_slam_initial_gap > slam_fast_threshold * mcconf->l_max_duty) {
								gap_ratio = 1.0f;  // Full fast ramp until target reached
								// Reset slam tracking once we reach target
								if (duty_gap < 0.02f) {
									throttle_slam_initial_gap = 0.0f;
								}
							} else {
								gap_ratio = gap_ratio * gap_ratio;  // Square it - smooth transition to slow
							}

							if (duty_target > osf_duty_ramped) {
								// Ramping UP
								float ramp_rate = ramp_up_slow + (ramp_up_fast - ramp_up_slow) * gap_ratio;
								osf_duty_ramped += ramp_rate * dt_s;
								if (osf_duty_ramped > duty_target) osf_duty_ramped = duty_target;
							} else {
								// Ramping DOWN
								float ramp_rate = ramp_down_slow + (ramp_down_fast - ramp_down_slow) * gap_ratio;
								osf_duty_ramped -= ramp_rate * dt_s;
								if (osf_duty_ramped < duty_target) osf_duty_ramped = duty_target;
							}

							// Clamp
							if (osf_duty_ramped < 0.0f) osf_duty_ramped = 0.0f;
							if (osf_duty_ramped > mcconf->l_max_duty) osf_duty_ramped = mcconf->l_max_duty;

							// ========== CRITICAL: HARD SPEED LIMIT ENFORCEMENT ==========
							// Even if ramped duty is high, NEVER allow motor assist above speed limit
							// This is a safety clamp - applies the speed_scale to the actual output
							float duty_to_send = osf_duty_ramped;
							if (!mc_interface_is_offroad_mode()) {
								// In street mode, hard-limit the duty based on current speed
								// This ensures motor can NEVER push past the speed limit
								duty_to_send *= speed_scale;
								
								// If we're at or above speed limit, force release motor
								if (speed_scale <= 0.0f) {
									duty_to_send = 0.0f;
								}
							}

							if (duty_to_send > 0.001f) {
								mc_interface_set_throttle_limit_active(true);
								mc_interface_set_duty(pwr >= 0.0f ? duty_to_send : -duty_to_send);
								send_duty = true;
							} else {
								mc_interface_release_motor();
							}
							// current_mode stays FALSE - no current applied
						}
					} else {
						// PAS MODE: throttle is zero, check for pedaling
						// If PAS duty mode is enabled, ONLY use that (no fallback to current PAS)
						if (config.haz_pas_duty_enabled) {
							float pas_duty = haz_pas_duty_process(&config, dt_s);
							if (pas_duty > 0.001f) {
								// PAS Duty active - use DUTY control (not current!)
								osf_duty_ramped = pas_duty;
								mc_interface_set_throttle_limit_active(true);
								mc_interface_set_duty(pas_duty);
								send_duty = true;
							} else {
								// PAS duty at zero - release motor, no fallback
								mc_interface_release_motor();
								osf_duty_ramped = 0.0f;
							}
						} else {
							// PAS duty disabled - use current-based PAS follow
							float pas_current_rel = haz_pas_follow_process(dt_s);
							if (pas_current_rel > 0.0f) {
								// PAS active - use current control
								osf_duty_ramped = fabsf(mc_interface_get_duty_cycle_now());
								current_mode = true;
								current_rel = pas_current_rel;
								mc_interface_set_throttle_limit_active(true);
							} else {
								// Nothing - release motor
								mc_interface_release_motor();
								osf_duty_ramped = 0.0f;
							}
						}
					}
				} else {
					osf_duty_ramped = 0.0f;
					freewheel_catch_state = 0;
					freewheel_catch_duty = 0.0f;
				}
			}
			break;

		case ADC_CTRL_TYPE_PID:
		case ADC_CTRL_TYPE_PID_REV_CENTER:
		case ADC_CTRL_TYPE_PID_REV_BUTTON:
			if ((pwr >= 0.0 && rpm_now > 0.0) || (pwr < 0.0 && rpm_now < 0.0)) {
				current_rel = pwr;
			} else {
				current_rel = pwr;
			}

			if (!(ms_without_power < MIN_MS_WITHOUT_POWER && config.safe_start)) {
				if (pwr > 0.0f) {
					mc_interface_set_throttle_limit_active(true);
				}
				float speed = 0.0f;
				float erpm_cap = mc_get_active_erpm_limit();
				if (erpm_cap <= 0.0f) {
					erpm_cap = fabsf(mcconf->l_max_erpm);
				}
				float neg_cap = erpm_cap;
				float min_erpm_abs = fabsf(mcconf->l_min_erpm);
				if (isfinite(min_erpm_abs) && min_erpm_abs > 0.0f) {
					neg_cap = fminf(neg_cap, min_erpm_abs);
				}
				if (pwr >= 0.0f) {
					speed = pwr * erpm_cap;
				} else {
					speed = pwr * neg_cap;
				}

				mc_interface_set_pid_speed(speed);
				send_duty = true;
			}

			if (fabsf(pwr) < 0.001) {
				ms_without_power += (1000.0 * (float)sleep_time) / (float)CH_CFG_ST_FREQUENCY;
			}
			break;

		default:
			continue;
		}

		// If safe start is enabled and the output has not been zero for long enough
		if ((ms_without_power < MIN_MS_WITHOUT_POWER && config.safe_start) || !range_ok) {
			static int pulses_without_power_before = 0;
			if (ms_without_power == pulses_without_power_before) {
				ms_without_power = 0;
			}
			pulses_without_power_before = ms_without_power;
			mc_interface_set_brake_current(timeout_get_brake_current());

			if (config.multi_esc) {
				for (int i = 0;i < CAN_STATUS_MSGS_TO_STORE;i++) {
					can_status_msg *msg = comm_can_get_status_msg_index(i);

					if (msg->id >= 0 && UTILS_AGE_S(msg->rx_time) < MAX_CAN_AGE) {
						comm_can_set_current_brake(msg->id, timeout_get_brake_current());
					}
				}
			}

			continue;
		}

		// Reset timeout only when the ADC-app is not detached
		if (!adc_detached) {
			timeout_reset();
		}

		// If c is pressed and no throttle is used, maintain the current speed with PID control
		static bool was_pid = false;

		// Filter RPM to avoid glitches
		static float rpm_filtered = 0.0;
		UTILS_LP_MOVING_AVG_APPROX(rpm_filtered, mc_interface_get_rpm(), RPM_FILTER_SAMPLES);

		if (current_mode && cc_button && fabsf(pwr) < 0.001) {
			static float pid_rpm = 0.0;

			if (!was_pid) {
				was_pid = true;
				pid_rpm = rpm_filtered;
			}

			mc_interface_set_pid_speed(pid_rpm);

			// Send the same duty cycle to the other controllers
			if (config.multi_esc) {
				float current = mc_interface_get_tot_current_directional_filtered();

				for (int i = 0;i < CAN_STATUS_MSGS_TO_STORE;i++) {
					can_status_msg *msg = comm_can_get_status_msg_index(i);

					if (msg->id >= 0 && UTILS_AGE_S(msg->rx_time) < MAX_CAN_AGE) {
						comm_can_set_current(msg->id, current);
					}
				}
			}

			continue;
		}

		was_pid = false;

		// Find lowest RPM (for traction control)
		float rpm_local = mc_interface_get_rpm();
		float rpm_lowest = rpm_local;
		if (config.multi_esc) {
			for (int i = 0;i < CAN_STATUS_MSGS_TO_STORE;i++) {
				can_status_msg *msg = comm_can_get_status_msg_index(i);

				if (msg->id >= 0 && UTILS_AGE_S(msg->rx_time) < MAX_CAN_AGE) {
					float rpm_tmp = msg->rpm;

					if (fabsf(rpm_tmp) < fabsf(rpm_lowest)) {
						rpm_lowest = rpm_tmp;
					}
				}
			}
		}

		// Optionally send the duty cycles to the other ESCs seen on the CAN-bus
		if (send_duty && config.multi_esc) {
			float duty = mc_interface_get_duty_cycle_now();

			for (int i = 0;i < CAN_STATUS_MSGS_TO_STORE;i++) {
				can_status_msg *msg = comm_can_get_status_msg_index(i);

				if (msg->id >= 0 && UTILS_AGE_S(msg->rx_time) < MAX_CAN_AGE) {
					comm_can_set_duty(msg->id, duty);
				}
			}
		}

		if (current_mode) {
			if (current_mode_brake) {
				mc_interface_set_brake_current_rel(current_rel);

				// Send brake command to all ESCs seen recently on the CAN bus
				if (config.multi_esc) {
					for (int i = 0;i < CAN_STATUS_MSGS_TO_STORE;i++) {
						can_status_msg *msg = comm_can_get_status_msg_index(i);

						if (msg->id >= 0 && UTILS_AGE_S(msg->rx_time) < MAX_CAN_AGE) {
							comm_can_set_current_brake_rel(msg->id, current_rel);
						}
					}
				}
			} else {
				float current_out = current_rel;
				bool is_reverse = false;
				if (current_out < 0.0) {
					is_reverse = true;
					current_out = -current_out;
					current_rel = -current_rel;
					rpm_local = -rpm_local;
					rpm_lowest = -rpm_lowest;
				}

				// Traction control
				if (config.multi_esc) {
					for (int i = 0;i < CAN_STATUS_MSGS_TO_STORE;i++) {
						can_status_msg *msg = comm_can_get_status_msg_index(i);

						if (msg->id >= 0 && UTILS_AGE_S(msg->rx_time) < MAX_CAN_AGE) {
							if (config.tc && config.tc_max_diff > 1.0) {
								float rpm_tmp = msg->rpm;
								if (is_reverse) {
									rpm_tmp = -rpm_tmp;
								}

								float diff = rpm_tmp - rpm_lowest;
                                if (diff < TC_DIFF_MAX_PASS) diff = 0;
                                if (diff > config.tc_max_diff) diff = config.tc_max_diff;
								current_out = utils_map(diff, 0.0, config.tc_max_diff, current_rel, 0.0);
							}

							if (is_reverse) {
								comm_can_set_current_rel(msg->id, -current_out);
							} else {
								comm_can_set_current_rel(msg->id, current_out);
							}
						}
					}

					if (config.tc) {
						float diff = rpm_local - rpm_lowest;
                        if (diff < TC_DIFF_MAX_PASS) diff = 0;
                        if (diff > config.tc_max_diff) diff = config.tc_max_diff;
						current_out = utils_map(diff, 0.0, config.tc_max_diff, current_rel, 0.0);
					}
				}

				if (is_reverse) {
					mc_interface_set_current_rel(-current_out);
				} else {
					mc_interface_set_current_rel(current_out);
				}
			}
		}
	}
}
