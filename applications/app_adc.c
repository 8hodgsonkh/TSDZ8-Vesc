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
#include "mcpwm_foc.h"
#include "timeout.h"
#include "utils_math.h"
#include "utils_sys.h"
#include "comm_can.h"
#include "hw.h"
#include "app_gear_detect.h"
#include "terminal.h"
#include "commands.h"
#include <math.h>

// Settings
#define MAX_CAN_AGE						0.1
#define MIN_MS_WITHOUT_POWER			500
#define FILTER_SAMPLES					5
#define RPM_FILTER_SAMPLES				8
#define TC_DIFF_MAX_PASS				60  // TODO: move to app_conf
#define PAS_THROTTLE_IDLE_GATE		0.05f
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

// Get power multiplier for current assist level (0.2 to 1.0)
static float get_assist_power_multiplier(void) {
	// Level 1=20%, 2=40%, 3=60%, 4=80%, 5=100%
	return (float)assist_level * 0.2f;
}

// Apply assist level to motor current limit (called when level changes or mode changes)
static void apply_assist_current_limit(void) {
	// Current scaling applied at limit enforcement level in mc_interface.c
	// This scales l_current_max for ALL motor modes (duty, current, PAS, etc.)
	mc_interface_set_assist_current_scale(get_assist_power_multiplier());
}

// Set assist level from display (1-5)
void app_adc_set_assist_level(uint8_t level) {
	if (level >= 1 && level <= 5) {
		assist_level = level;
		apply_assist_current_limit();  // Apply new current scale
	}
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
	float target_erpm_smooth;  // Final output ERPM for speed PID
	float target_erpm_filtered; // Interpolated cadence base ERPM (replaces LP)
	float rotation_progress;
	float idle_time;
	bool engaged;
	uint32_t last_step_count;  // For step-rate idle detection
	float step_rate_smooth;    // LP-filtered PAS step rate (steps/sec)
	// Linear interpolation state: ramp from prev→next over edge_period
	float interp_prev_erpm;    // ERPM at previous cadence edge
	float interp_next_erpm;    // ERPM at current cadence edge (ramp target)
	float interp_elapsed;      // Time elapsed since last cadence edge
	float interp_period;       // Expected time until next edge (from ISR avg)
	float interp_last_raw;     // Last raw ERPM seen (to detect new edge)
} haz_pas_follow_ctx_t;

static haz_pas_follow_ctx_t haz_pas_follow_ctx = {
	.target_erpm_smooth = 0.0f,
	.target_erpm_filtered = 0.0f,
	.rotation_progress = 0.0f,
	.idle_time = 0.0f,
	.engaged = false,
	.last_step_count = 0,
	.step_rate_smooth = 0.0f,
	.interp_prev_erpm = 0.0f,
	.interp_next_erpm = 0.0f,
	.interp_elapsed = 0.0f,
	.interp_period = 0.0f,
	.interp_last_raw = 0.0f,
};

// =============================================================================
// PAS DUTY MODE - Simple, Smooth Duty-Based PAS
// =============================================================================
// ERPM-tracking PAS duty: match motor speed to pedal cadence through drivetrain
//
// motor_erpm = cadence_rpm × gear_ratio × pole_pairs
// target_duty = (target_erpm / max_erpm) × max_duty
//
// The motor runs at (or slightly ahead of) the speed your legs dictate,
// regardless of battery voltage, switching frequency, or load.

typedef struct {
	float duty_cmd;              // Current duty output (smoothed)
	float target_duty;           // Target duty we're ramping toward
	float cadence_smooth;        // Smoothed pedal RPM
	float cadence_prev;          // Previous cadence for acceleration detection
	float idle_time;             // Time since last pedal activity
	bool engaged;                // PAS currently active
	uint32_t last_step_count;    // For activity detection
	// Linear interpolation: smooth cadence between PAS edges
	float interp_prev_rpm;       // RPM at previous edge
	float interp_next_rpm;       // RPM at current edge (ramp target)
	float interp_elapsed;        // Time since last edge
	float interp_period;         // Expected time until next edge
	float interp_last_raw;       // Last raw RPM (to detect new edge)
} haz_pas_duty_ctx_t;

static haz_pas_duty_ctx_t haz_pas_duty_ctx = {
	.duty_cmd = 0.0f,
	.target_duty = 0.0f,
	.cadence_smooth = 0.0f,
	.cadence_prev = 0.0f,
	.idle_time = 0.0f,
	.engaged = false,
	.last_step_count = 0,
	.interp_prev_rpm = 0.0f,
	.interp_next_rpm = 0.0f,
	.interp_elapsed = 0.0f,
	.interp_period = 0.0f,
	.interp_last_raw = 0.0f
};

// =============================================================================
// EXTERNAL TORQUE SENSOR
// =============================================================================
// Two modes:
//   1. BLE: ESP32 sends processed torque (0-160) via COMM_CUSTOM_APP_DATA "HT"
//   2. Direct UART: TSDZ8 torque sensor wired to HW_UART (USART3, PB10/PB11)
//      Protocol: [0xAA][0x04][0x01][DATA_HI][DATA_LO][XOR_CHECKSUM]
//      115200 baud, ~50Hz, 10-bit ADC (0-1023)
// =============================================================================

// Compile flag: HAZ_TORQUE_UART_DIRECT defined in hw_go_foc_s100_d100s.h
// Uses HW_UART_P (UART4/SD4, PC10/PC11) — the unpopulated NRF52 port.
// This keeps HW_UART (UART3/SD3, PB10/PB11) free for Flipsky BT module.

static volatile uint16_t s_ext_torque = 0;        // 0-160 normalized
static volatile systime_t s_ext_torque_ts = 0;    // Last receive timestamp
static float s_ext_torque_smooth = 0.0f;          // LP-filtered torque

void app_adc_set_ext_torque(uint16_t value) {
	if (value > 160) value = 160;
	s_ext_torque = value;
	s_ext_torque_ts = chVTGetSystemTimeX();
}

uint16_t app_adc_get_ext_torque(void) {
	return s_ext_torque;
}

// =============================================================================
// DIRECT UART TORQUE SENSOR (TSDZ8 → VESC USART3)
// =============================================================================
#if HAZ_TORQUE_UART_DIRECT

static const SerialConfig torque_uart_cfg = {
	115200, 0, 0, 0
};
static bool torque_uart_running = false;

// Auto-calibration: average first N readings as resting baseline
#define TORQUE_CAL_SAMPLES   50
static uint16_t torque_cal_buf[TORQUE_CAL_SAMPLES];
static int torque_cal_idx = 0;
static bool torque_cal_done = false;
static uint16_t torque_offset = 0;

// Max ADC range above offset — typical TSDZ8 range
// 280 ADC counts ≈ max pedal force. Can be overridden by ESP32 calibration.
static volatile uint16_t torque_adc_max_val = 280;

// Debug counters
static volatile uint32_t torque_dbg_bytes       = 0;  // Total bytes received
static volatile uint32_t torque_dbg_frames      = 0;  // Valid parsed frames
static volatile uint32_t torque_dbg_xor_err     = 0;  // Checksum failures
static volatile uint32_t torque_dbg_sync_loss   = 0;  // Unexpected bytes while syncing
static volatile uint16_t torque_dbg_last_raw    = 0;  // Last raw ADC before calibration

// TSDZ8 UART protocol state machine
typedef enum {
	TS_SYNC, TS_LEN, TS_CMD, TS_DATA_HI, TS_DATA_LO, TS_CHECKSUM
} tsdz8_parse_state_t;

static tsdz8_parse_state_t ts_state = TS_SYNC;
static uint8_t ts_len, ts_cmd, ts_data_hi, ts_data_lo, ts_xor;

static void torque_uart_process_byte(uint8_t b) {
	torque_dbg_bytes++;
	switch (ts_state) {
	case TS_SYNC:
		if (b == 0xAA) { ts_xor = b; ts_state = TS_LEN; }
		else { torque_dbg_sync_loss++; }
		break;
	case TS_LEN:
		ts_len = b; ts_xor ^= b;
		ts_state = (ts_len >= 3 && ts_len <= 8) ? TS_CMD : TS_SYNC;
		break;
	case TS_CMD:
		ts_cmd = b; ts_xor ^= b;
		ts_state = (ts_cmd == 0x01) ? TS_DATA_HI : TS_SYNC;
		break;
	case TS_DATA_HI:
		ts_data_hi = b; ts_xor ^= b;
		ts_state = TS_DATA_LO;
		break;
	case TS_DATA_LO:
		ts_data_lo = b; ts_xor ^= b;
		ts_state = TS_CHECKSUM;
		break;
	case TS_CHECKSUM:
		if (b == ts_xor) {
			uint16_t raw = ((uint16_t)ts_data_hi << 8) | ts_data_lo;
			torque_dbg_frames++;
			torque_dbg_last_raw = raw;

			// Auto-calibrate offset from first N readings (must be at rest!)
			if (!torque_cal_done) {
				torque_cal_buf[torque_cal_idx++] = raw;
				if (torque_cal_idx >= TORQUE_CAL_SAMPLES) {
					uint32_t sum = 0;
					for (int i = 0; i < TORQUE_CAL_SAMPLES; i++) sum += torque_cal_buf[i];
					torque_offset = (uint16_t)(sum / TORQUE_CAL_SAMPLES);
					torque_cal_done = true;
				}
			} else {
				// Offset-subtract and normalize to 0-160
				int16_t val = (int16_t)raw - (int16_t)torque_offset;
				if (val < 0) val = 0;
				uint16_t adc_max = torque_adc_max_val > 0 ? torque_adc_max_val : 280;
				int16_t norm = (val * 160) / adc_max;
				if (norm > 160) norm = 160;
				app_adc_set_ext_torque((uint16_t)norm);
			}
		} else {
			torque_dbg_xor_err++;
		}
		ts_state = TS_SYNC;
		break;
	}
}

// Terminal command: "torque_dbg" — dump torque sensor debug info
static void torque_dbg_cmd(int argc, const char **argv) {
	(void)argc; (void)argv;
	commands_printf("=== Torque UART Debug ===");
	commands_printf("UART running:    %s", torque_uart_running ? "YES" : "NO");
	commands_printf("Bytes received:  %u", (unsigned)torque_dbg_bytes);
	commands_printf("Valid frames:    %u", (unsigned)torque_dbg_frames);
	commands_printf("XOR errors:      %u", (unsigned)torque_dbg_xor_err);
	commands_printf("Sync losses:     %u", (unsigned)torque_dbg_sync_loss);
	commands_printf("Last raw ADC:    %u", (unsigned)torque_dbg_last_raw);
	commands_printf("Cal done:        %s  (offset=%u, idx=%d)",
		torque_cal_done ? "YES" : "NO", (unsigned)torque_offset, torque_cal_idx);
	commands_printf("Torque (norm):   %u / 160", (unsigned)s_ext_torque);
	commands_printf("Torque (smooth): %.2f", (double)s_ext_torque_smooth);
	
	systime_t age = chVTTimeElapsedSinceX(s_ext_torque_ts);
	float age_s = (float)age / (float)CH_CFG_ST_FREQUENCY;
	commands_printf("Last update:     %.3f s ago", (double)age_s);
	commands_printf(" ");
}

static void torque_uart_start(void) {
	if (torque_uart_running) return;

	// Disable permanent UART comm on SD4 if it was started — we're taking over
	app_uartcomm_stop(UART_PORT_BUILTIN);

	palSetPadMode(HW_UART_P_TX_PORT, HW_UART_P_TX_PIN,
		PAL_MODE_ALTERNATE(HW_UART_P_GPIO_AF) | PAL_STM32_OSPEED_HIGHEST | PAL_STM32_PUDR_PULLUP);
	palSetPadMode(HW_UART_P_RX_PORT, HW_UART_P_RX_PIN,
		PAL_MODE_ALTERNATE(HW_UART_P_GPIO_AF) | PAL_STM32_OSPEED_HIGHEST | PAL_STM32_PUDR_PULLUP);
	sdStart(&HW_UART_P_DEV, &torque_uart_cfg);
	torque_uart_running = true;

	// Register terminal debug command
	terminal_register_command_callback(
		"torque_dbg",
		"Print torque sensor UART debug info",
		0,
		torque_dbg_cmd);

	commands_printf("Torque UART started on SD4 (PC10/PC11) @ 115200\n");
}

static void torque_uart_stop(void) {
	if (!torque_uart_running) return;
	sdStop(&HW_UART_P_DEV);
	torque_uart_running = false;
}

// Drain available UART bytes — call from ADC thread each iteration
static void torque_uart_poll(void) {
	if (!torque_uart_running) return;
	msg_t b;
	while ((b = sdGetTimeout(&HW_UART_P_DEV, TIME_IMMEDIATE)) != MSG_TIMEOUT) {
		torque_uart_process_byte((uint8_t)b);
	}
}

uint16_t app_adc_get_torque_raw(void) {
	return torque_dbg_last_raw;
}

void app_adc_set_torque_cal(uint16_t offset, uint16_t adc_max) {
	torque_offset = offset;
	torque_cal_done = true;  // Skip auto-cal when ESP32 sets values
	torque_adc_max_val = adc_max > 0 ? adc_max : 280;
}

#else // !HAZ_TORQUE_UART_DIRECT

uint16_t app_adc_get_torque_raw(void) { return 0; }
void app_adc_set_torque_cal(uint16_t offset, uint16_t adc_max) { (void)offset; (void)adc_max; }

#endif // HAZ_TORQUE_UART_DIRECT

// ERPM-tracking PAS duty - motor matches cadence through drivetrain
static float haz_pas_duty_process(volatile adc_config *conf, float dt_s) {
	if (!conf->haz_pas_duty_enabled) {
		haz_pas_duty_ctx.duty_cmd = 0.0f;
		return 0.0f;
	}

	// Motor config
	const volatile mc_configuration *mcconf = mc_interface_get_configuration();
	float max_duty = mcconf->l_max_duty;
	if (max_duty < 0.1f) max_duty = 0.95f;

	int poles = mcconf->si_motor_poles;
	if (poles <= 0) poles = 8;
	float pole_pairs = (float)poles * 0.5f;

	float lambda = mcconf->foc_motor_flux_linkage;
	if (!isfinite(lambda) || lambda < 1e-6f) lambda = 0.01259f;

	float v_bus = mc_interface_get_input_voltage_filtered();
	if (!isfinite(v_bus) || v_bus < 10.0f) v_bus = 48.0f;

	// Drivetrain: internal gearbox ratio (e.g. 38:1 for TSDZ8)
	const app_configuration *appconf = app_get_configuration();
	float gear_ratio = appconf->gear_detect_conf.internal_ratio;
	if (!isfinite(gear_ratio) || gear_ratio <= 0.0f) gear_ratio = 38.0f;

	// Config params — dual-purpose fields depending on torque mode:
	//
	// TORQUE MODE ON (haz_torque_enabled):
	//   ramp_up      → Max duty ramp rate at full torque (duty/s)
	//   ramp_down    → Ramp down rate when torque released (duty/s)
	//   accel_gain   → (unused in torque mode)
	//   load_gain    → (unused in torque mode)
	//   lead_pct     → Cadence lead: 0=0.50×, 1.0=1.0×, 2.0=1.50× cadence
	//   Cadence base tracks instantly (no ramp). Torque controls how fast
	//   duty ramps above cadence base. strength scales the ramp rate.
	//
	// TORQUE MODE OFF (cadence inference):
	//   ramp_up/down → duty/s slew rate limits
	//   accel_gain   → Cadence accel boost strength
	//   load_gain    → Load/current boost strength
	//   lead_pct     → Lead %: 0-2 → 1.0-1.20 lead factor
	const float ramp_up = fmaxf(conf->haz_pas_duty_ramp_up, 0.05f);
	const float ramp_down = fmaxf(conf->haz_pas_duty_ramp_down, 0.05f);
	const float accel_gain = fmaxf(conf->haz_pas_duty_accel_gain, 0.0f);
	const float load_gain = fmaxf(conf->haz_pas_duty_load_gain, 0.0f);

	// Lead factor: different mapping depending on torque mode
	float lead_raw = conf->haz_pas_duty_lead_pct;
	if (lead_raw < 0.0f) lead_raw = 0.0f;
	if (lead_raw > 2.0f) lead_raw = 2.0f;
	float lead_factor;
	if (conf->haz_torque_enabled) {
		// Torque mode: 0→0.50, 1→1.00, 2→1.50
		// Full range: motor can lag at half cadence or lead at 150%
		lead_factor = 0.50f + lead_raw * 0.50f;
	} else {
		// Cadence mode: 0→1.00, 2→1.20 (original behavior)
		lead_factor = 1.0f + lead_raw * 0.10f;
	}
	const float idle_timeout = fmaxf(conf->haz_pas_duty_idle_timeout, 0.1f);

	// ERPM→duty conversion factor from motor physics:
	// duty = ERPM × lambda × sqrt(3) × 2π / (v_bus × 60)
	// This auto-adjusts for battery voltage — no max_erpm setting needed.
	const float erpm_to_duty = (lambda * 1.7320508f * 6.2831853f) / (v_bus * 60.0f);

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

	// Idle detection: time since last real PAS edge from ISR timestamps.
	// No LP filter lag — edge arrives = instant not-idle, no edge = instant idle.
	// Uses the config idle_timeout for the dead time threshold.
	bool is_idle = (app_pas_get_time_since_real_step() > idle_timeout);

	// Get pedal RPM with optional MEDIAN FILTER (outlier rejection)
	// Tunable via VESC Tool:
	// - PAS Duty: Smoothing (smoothing): 0.0=raw, 1.0=max smooth
	// - PAS Duty: Median Filter (median_filter): 0=off, 1=latest, 3-11=median
	
	// Median buffer size: direct integer from setting (0-11)
	int buf_size = (int)(conf->haz_pas_duty_median_filter + 0.5f);  // round
	if (buf_size < 0) buf_size = 0;
	if (buf_size > 11) buf_size = 11;
	
	// Target smoothing: 0.0 = instant tracking, 1.0 = maximum smooth
	// Repurposed: filters the cadence DUTY TARGET (floor for torque boost)
	// rather than the cadence RPM input. PAS already LP-filters RPM in app_pas.c.
	float smoothing = conf->haz_pas_duty_smoothing;
	if (smoothing < 0.0f) smoothing = 0.0f;
	if (smoothing > 1.0f) smoothing = 1.0f;
	float target_lp_factor = (smoothing < 0.01f) ? 1.0f : (1.0f - smoothing * 0.99f);
	
	// Fixed 11-element ring buffer
	static float rpm_buffer[11] = {0};
	static int rpm_idx = 0;
	static int buf_filled = 0;
	
	float pedal_rpm_raw = app_pas_get_pedal_rpm();
	rpm_buffer[rpm_idx] = pedal_rpm_raw;
	rpm_idx = (rpm_idx + 1) % 11;
	if (buf_filled < 11) buf_filled++;
	
	float pedal_rpm;
	if (buf_size <= 1) {
		// 0 or 1 = no median, use raw (or latest) value directly
		pedal_rpm = pedal_rpm_raw;
	} else {
		// Median filter with buf_size samples
		int samples_to_use = (buf_filled < buf_size) ? buf_filled : buf_size;
		if (samples_to_use < 3) samples_to_use = 3;  // median needs at least 3
		
		float sorted[11];
		for (int i = 0; i < samples_to_use; i++) {
			int idx = (rpm_idx - 1 - i + 11) % 11;
			sorted[i] = rpm_buffer[idx];
		}
		
		// Insertion sort
		for (int i = 1; i < samples_to_use; i++) {
			float key = sorted[i];
			int j = i - 1;
			while (j >= 0 && sorted[j] > key) {
				sorted[j + 1] = sorted[j];
				j--;
			}
			sorted[j + 1] = key;
		}
		
		pedal_rpm = sorted[samples_to_use / 2];
	}
	
	// Cadence: linear interpolation between PAS edges
	// Instead of holding stale RPM for 25+ cycles between edges,
	// smoothly ramp from previous cadence to current cadence over
	// the expected edge period. Same approach as haz_pas_follow_process().
	float edge_period = app_pas_get_step_period();

	// Detect new PAS edge: raw RPM changed
	if (fabsf(pedal_rpm - haz_pas_duty_ctx.interp_last_raw) > 0.5f) {
		haz_pas_duty_ctx.interp_prev_rpm = haz_pas_duty_ctx.cadence_smooth;
		haz_pas_duty_ctx.interp_next_rpm = pedal_rpm;
		haz_pas_duty_ctx.interp_elapsed = 0.0f;
		haz_pas_duty_ctx.interp_period = fmaxf(edge_period, 0.005f);
		haz_pas_duty_ctx.interp_last_raw = pedal_rpm;
	}

	// Advance interpolation
	haz_pas_duty_ctx.interp_elapsed += dt_s;
	float interp_t = haz_pas_duty_ctx.interp_elapsed / haz_pas_duty_ctx.interp_period;
	if (interp_t > 1.0f) interp_t = 1.0f;
	if (!isfinite(interp_t) || haz_pas_duty_ctx.interp_period < 0.001f) interp_t = 1.0f;

	haz_pas_duty_ctx.cadence_smooth =
		haz_pas_duty_ctx.interp_prev_rpm +
		(haz_pas_duty_ctx.interp_next_rpm - haz_pas_duty_ctx.interp_prev_rpm) * interp_t;

	// ========== ERPM-TRACKING + TORQUE-CONTROLLED RAMP ==========
	//
	// TORQUE MODE (haz_torque_enabled):
	//   Cadence sets base duty (instant tracking, no ramp).
	//   Torque controls ramp rate above cadence base:
	//     ramp_rate = shaped_torque × strength × ramp_up (duty/s)
	//   While stomping: duty ramps up at that rate.
	//   On release: duty ramps down at ramp_down back to cadence base.
	//   Never below cadence base, never above max_duty.
	//
	// CADENCE INFERENCE MODE (no torque sensor):
	//   Original behavior — accel/load signals modulate lead factor.
	//
	static float smooth_accel_mult = 0.0f;
	static float smooth_load_mult = 0.0f;
	static float prev_event_period = 0.0f;
	static uint32_t prev_step_count = 0;
	static float accel_rate_smooth = 0.0f;
	float target_duty = 0.0f;
	if (!is_idle && haz_pas_duty_ctx.cadence_smooth > 3.0f) {

		if (conf->haz_torque_enabled) {
			// ========== TORQUE SENSOR MODE ==========
			// Read torque sensor
			systime_t torque_age = chVTTimeElapsedSinceX(s_ext_torque_ts);
			float torque_age_s = (float)torque_age / (float)CH_CFG_ST_FREQUENCY;
			float torque_timeout = fmaxf(conf->haz_torque_idle_timeout, 0.1f);

			float torque_frac = 0.0f;  // 0-1 normalized torque
			if (torque_age_s < torque_timeout && s_ext_torque > 0) {
				torque_frac = (float)s_ext_torque / 160.0f;
			}

			float start_thresh = conf->haz_torque_start_threshold / 160.0f;
			if (torque_frac < start_thresh) torque_frac = 0.0f;

			// LP filter (haz_torque_smoothing)
			float t_smooth = fmaxf(conf->haz_torque_smoothing, 0.0f);
			if (t_smooth > 0.99f) t_smooth = 0.99f;
			float t_lp = (t_smooth < 0.01f) ? 1.0f : (1.0f - t_smooth * 0.99f);
			UTILS_LP_FAST(s_ext_torque_smooth, torque_frac, t_lp);

			// Expo shaping (responsiveness curve)
			float resp = fmaxf(conf->haz_torque_responsiveness, 0.0f);
			if (resp > 2.0f) resp = 2.0f;
			float x = fmaxf(s_ext_torque_smooth, 0.0f);
			if (x > 1.0f) x = 1.0f;
			float shaped;
			if (resp < 0.5f) {
				shaped = x * x;
			} else if (resp > 1.5f) {
				shaped = sqrtf(x);
			} else {
				float blend = (resp - 0.5f);
				shaped = (x * x) * (1.0f - blend)
				       + sqrtf(fmaxf(x, 0.0f)) * blend;
			}

			// Torque → ramp rate (duty/s)
			// Full stomp (shaped=1) at strength=1: ramp at ramp_up duty/s
			// Light touch (shaped=0.3) at strength=2: ramp at 0.6 × ramp_up duty/s
			float strength = fmaxf(conf->haz_torque_strength, 0.1f);
			if (strength > 5.0f) strength = 5.0f;
			float torque_ramp = shaped * strength * ramp_up;

			// Cadence base duty — compute raw, then LP-filter the TARGET
			// This smooths what the torque sensor boosts off of.
			// Smoothing setting controls this filter, not the cadence input.
			float base_erpm = haz_pas_duty_ctx.cadence_smooth * gear_ratio * pole_pairs * lead_factor;
			float cadence_duty_raw = base_erpm * erpm_to_duty;
			cadence_duty_raw *= speed_power_scale;
			if (cadence_duty_raw > max_duty) cadence_duty_raw = max_duty;
			if (cadence_duty_raw < 0.0f) cadence_duty_raw = 0.0f;
			static float cadence_duty_smooth = 0.0f;
			UTILS_LP_FAST(cadence_duty_smooth, cadence_duty_raw, target_lp_factor);
			float cadence_duty = cadence_duty_smooth;

			// Ramp: torque active → climb, released → fall back to cadence
			if (torque_ramp > 0.001f) {
				haz_pas_duty_ctx.duty_cmd += torque_ramp * dt_s;
			} else {
				haz_pas_duty_ctx.duty_cmd -= ramp_down * dt_s;
			}

			// Clamp: never below cadence base, never above max
			if (haz_pas_duty_ctx.duty_cmd < cadence_duty) {
				haz_pas_duty_ctx.duty_cmd = cadence_duty;
			}
			if (haz_pas_duty_ctx.duty_cmd > max_duty) {
				haz_pas_duty_ctx.duty_cmd = max_duty;
			}

			target_duty = haz_pas_duty_ctx.duty_cmd;

		} else {
			// ========== CADENCE INFERENCE MODE (no torque sensor) ==========
			float assist_extra = 0.0f;
			{

		// === CADENCE ACCEL SIGNAL (period-based, LP-filtered) ===
		float event_period = app_pas_get_event_period();
		uint32_t cur_step_count = app_pas_get_step_count();
		float raw_rpm_rate = 0.0f;

		if (cur_step_count != prev_step_count && prev_event_period > 1e-4f && event_period > 1e-4f) {
			float magnets = fmaxf(1.0f, (float)appconf->app_pas_conf.magnets);
			float delta_rpm = (60.0f / magnets) * (1.0f/event_period - 1.0f/prev_event_period);
			raw_rpm_rate = delta_rpm / event_period;  // RPM/s
			prev_event_period = event_period;
			prev_step_count = cur_step_count;
		} else if (cur_step_count != prev_step_count) {
			prev_event_period = event_period;
			prev_step_count = cur_step_count;
		}

		// Heavy LP filter on rpm_rate — τ ≈ 300ms at 1kHz (kills spikes)
		if (raw_rpm_rate > 0.0f) {
			UTILS_LP_FAST(accel_rate_smooth, raw_rpm_rate, 0.01f);
		} else {
			accel_rate_smooth *= 0.995f;
		}

		float accel_target = accel_rate_smooth * accel_gain * 0.01f;
		if (accel_target > 0.30f) accel_target = 0.30f;
		if (accel_target < 0.0f) accel_target = 0.0f;

		float accel_ramp = 0.5f * dt_s;
		if (accel_target > smooth_accel_mult) {
			smooth_accel_mult += fminf(accel_ramp, accel_target - smooth_accel_mult);
		} else {
			smooth_accel_mult += (accel_target - smooth_accel_mult) * 4.0f * dt_s;
		}

		// === LOAD SIGNAL (current-based effort proxy) ===
		float motor_current = fabsf(mc_interface_get_tot_current_filtered());
		float max_current = mcconf->l_current_max;
		if (max_current < 1.0f) max_current = 30.0f;
		float load_fraction = motor_current / max_current;
		if (load_fraction > 1.0f) load_fraction = 1.0f;

		float base_target_erpm = haz_pas_duty_ctx.cadence_smooth * gear_ratio * pole_pairs * lead_factor;

		float actual_erpm = fabsf(mc_interface_get_rpm());
		float overshoot_ratio = 0.0f;
		if (base_target_erpm > 100.0f) {
			overshoot_ratio = (actual_erpm - base_target_erpm) / base_target_erpm;
		}
		float overshoot_scale = 1.0f - (overshoot_ratio * 5.0f);
		if (overshoot_scale > 1.0f) overshoot_scale = 1.0f;
		if (overshoot_scale < 0.0f) overshoot_scale = 0.0f;

		float load_target = 0.0f;
		if (load_fraction > 0.10f && load_gain > 0.01f) {
			load_target = load_fraction * 0.20f * load_gain * overshoot_scale;
			if (load_target > 0.20f) load_target = 0.20f;
		}

		float load_ramp = 0.4f * dt_s;
		if (load_target > smooth_load_mult) {
			smooth_load_mult += fminf(load_ramp, load_target - smooth_load_mult);
		} else {
			smooth_load_mult += (load_target - smooth_load_mult) * 6.0f * dt_s;
		}
		if (smooth_load_mult < 0.0f) smooth_load_mult = 0.0f;

		assist_extra = smooth_accel_mult + smooth_load_mult;
		if (assist_extra > 0.40f) assist_extra = 0.40f;

		} // end cadence inference

		float effective_lead = lead_factor * (1.0f + assist_extra);
		float target_erpm = haz_pas_duty_ctx.cadence_smooth * gear_ratio * pole_pairs * effective_lead;

		// Convert ERPM to duty using physics (voltage-independent)
		target_duty = target_erpm * erpm_to_duty;

		// Apply speed limit (street mode)
		target_duty *= speed_power_scale;

		if (target_duty > max_duty) target_duty = max_duty;
		if (target_duty < 0.0f) target_duty = 0.0f;

		// Cadence inference mode: use standard ramp
		if (target_duty > haz_pas_duty_ctx.duty_cmd) {
			haz_pas_duty_ctx.duty_cmd += ramp_up * dt_s;
			if (haz_pas_duty_ctx.duty_cmd > target_duty)
				haz_pas_duty_ctx.duty_cmd = target_duty;
		} else {
			haz_pas_duty_ctx.duty_cmd -= ramp_down * dt_s;
			if (haz_pas_duty_ctx.duty_cmd < target_duty)
				haz_pas_duty_ctx.duty_cmd = target_duty;
		}
		target_duty = haz_pas_duty_ctx.duty_cmd;

		} // end cadence inference mode
	}

	// Final output
	haz_pas_duty_ctx.duty_cmd = target_duty;

	// Clamp
	if (haz_pas_duty_ctx.duty_cmd < 0.0f) haz_pas_duty_ctx.duty_cmd = 0.0f;
	if (haz_pas_duty_ctx.duty_cmd > max_duty) haz_pas_duty_ctx.duty_cmd = max_duty;

	// Update previous cadence for next cycle's acceleration detection
	haz_pas_duty_ctx.cadence_prev = haz_pas_duty_ctx.cadence_smooth;

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

	float mag_raw = fabsf(pwr_in);
	if (mag_raw > 1.0f) {
		mag_raw = 1.0f;
	}
	float alpha = throttle_filter_hz * dt_s;
	utils_truncate_number(&alpha, 0.0f, 1.0f);
	haz_throttle_filtered += (mag_raw - haz_throttle_filtered) * alpha;
	float mag = haz_throttle_filtered;  // Filtered for target calculation
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

	// Two-segment ramp curve: min→mid→max based on RAW throttle position
	// Using raw (unfiltered) so a fast wrist flick immediately gets max ramp rate
	// even though the filtered target is still climbing
	float ramp_up_base_a;
	if (mag_raw <= ramp_up_mid_throttle) {
		// Lower segment: interpolate from min to mid
		float t = mag_raw / ramp_up_mid_throttle;
		ramp_up_base_a = ramp_up_min_a + (ramp_up_mid_a - ramp_up_min_a) * t;
	} else {
		// Upper segment: interpolate from mid to max
		float t = (mag_raw - ramp_up_mid_throttle) / (1.0f - ramp_up_mid_throttle);
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
	haz_pas_follow_ctx.target_erpm_smooth = 0.0f;
	haz_pas_follow_ctx.target_erpm_filtered = 0.0f;
	haz_pas_follow_ctx.rotation_progress = 0.0f;
	haz_pas_follow_ctx.idle_time = 0.0f;
	haz_pas_follow_ctx.engaged = false;
	haz_pas_follow_ctx.step_rate_smooth = 0.0f;
	haz_pas_follow_ctx.interp_prev_erpm = 0.0f;
	haz_pas_follow_ctx.interp_next_erpm = 0.0f;
	haz_pas_follow_ctx.interp_elapsed = 0.0f;
	haz_pas_follow_ctx.interp_period = 0.0f;
	haz_pas_follow_ctx.interp_last_raw = 0.0f;
	// Note: don't reset last_step_count — cumulative
}

// Returns:
//   > 0  : PAS active, speed PID already set on motor — caller must NOT apply current/duty
//   0.0  : PAS idle, caller should release motor
static float haz_pas_follow_process(float dt_s) {
	const app_configuration *appconf = app_get_configuration();
	const pas_config *pc = &appconf->app_pas_conf;
	if (pc->ctrl_type == PAS_CTRL_TYPE_NONE) {
		haz_pas_follow_reset();
		return 0.0f;
	}

	// Read config — all values stored in proper units, no scaling hacks
	const float start_rotations  = fmaxf(pc->pas_follow_start_rotations, 0.01f);
	const float idle_timeout_s   = fmaxf(pc->pas_follow_idle_timeout_s, 0.05f);
	const float target_lead      = fmaxf(pc->pas_follow_target_lead, 0.5f);
	const float erpm_ramp_rate   = fmaxf(pc->pas_follow_erpm_ramp_rate, 100.0f);
	const float cadence_filter   = fminf(fmaxf(pc->pas_follow_cadence_filter, 0.0f), 1.0f);
	const float ramp_down_rate   = fmaxf(pc->pas_follow_ramp_down_mult, 500.0f); // ERPM/s

	// ========== SPEED LIMIT CHECK (STREET MODE ONLY) ==========
	// Hardcoded 25 km/h — smooth taper from 90%, instant cut at limit
	if (!mc_interface_is_offroad_mode()) {
		const float speed_limit_ms = 25.0f / 3.6f;  // 25 km/h
		const float taper_start_ms = speed_limit_ms * 0.90f;
		float wheel_speed = fabsf(mc_interface_get_speed());

		if (wheel_speed >= speed_limit_ms) {
			// At or above limit — instant cut
			haz_pas_follow_ctx.target_erpm_smooth = 0.0f;
			return 0.0f;
		} else if (wheel_speed > taper_start_ms) {
			// Approaching limit — scale target ERPM proportionally
			float scale = (speed_limit_ms - wheel_speed) / (speed_limit_ms - taper_start_ms);
			haz_pas_follow_ctx.target_erpm_smooth *= scale;
			mc_interface_set_pid_speed(haz_pas_follow_ctx.target_erpm_smooth);
			return 1.0f;
		}
	}

	// ========== STEP-COUNT IDLE DETECTION (robust, like duty hybrid) ==========
	// Uses actual PAS magnet edge count instead of pedal_rpm, which can hold
	// the last value for 1-2 seconds after stopping due to event_period timeout.
	uint32_t step_count = app_pas_get_step_count();
	uint32_t steps_this_cycle = 0;
	if (step_count >= haz_pas_follow_ctx.last_step_count) {
		steps_this_cycle = step_count - haz_pas_follow_ctx.last_step_count;
	}
	haz_pas_follow_ctx.last_step_count = step_count;

	float step_rate = (float)steps_this_cycle / dt_s;
	UTILS_LP_FAST(haz_pas_follow_ctx.step_rate_smooth, step_rate, 0.5f);

	// With 20 magnets (quadrature pairs): 60 RPM = 20 steps/sec, 5 RPM ≈ 1.7 steps/sec
	// Below 1.5 steps/sec = not pedalling
	const float min_step_rate = 1.5f;
	if (haz_pas_follow_ctx.step_rate_smooth < min_step_rate) {
		haz_pas_follow_ctx.idle_time += dt_s;
	} else {
		haz_pas_follow_ctx.idle_time = 0.0f;
	}
	bool is_idle = (haz_pas_follow_ctx.idle_time > idle_timeout_s);

	// ========== GET RAW CADENCE (no LP filter — proportional ramp handles smoothness) ==========
	float pedal_rpm = app_pas_get_pedal_rpm();

	// ========== IDLE: INSTANT RELEASE ==========
	// Once idle timeout fires (pedaling stopped), command PID to 0 immediately.
	// No gradual ramp — dual freewheels mean motor just spins down naturally,
	// no phase shorting like duty control. Safe and responsive.
	// Ramp-down while still pedaling (easing off torque) is handled by the
	// normal proportional tracker + ramp_down_rate in the active section below.
	if (is_idle && haz_pas_follow_ctx.engaged) {
		haz_pas_follow_ctx.target_erpm_smooth = 0.0f;
		haz_pas_follow_ctx.target_erpm_filtered = 0.0f;
		haz_pas_follow_reset();
		return 0.0f; // Caller will release_motor()
	}

	// ========== ACTIVELY PEDALLING ==========
	// Engagement: count steps (magnet edges) rather than rotation_progress.
	// pedal_rpm is 0 until first full quadrature cycle completes, but step_count
	// fires on the very first edge. With 20 magnets, start_rotations=0.05 means
	// 0.05 * 20 = 1 step needed. This engages on the first detected pedal movement.
	if (!haz_pas_follow_ctx.engaged) {
		float steps_needed = fmaxf(start_rotations * 20.0f, 1.0f); // 20 magnets/rev
		haz_pas_follow_ctx.rotation_progress += (float)steps_this_cycle;
		if (haz_pas_follow_ctx.rotation_progress >= steps_needed) {
			haz_pas_follow_ctx.engaged = true;
			haz_pas_follow_ctx.target_erpm_smooth = fabsf(mc_interface_get_rpm());
		} else {
			return 0.0f;
		}
	}

	const volatile mc_configuration *mcconf = mc_interface_get_configuration();

	int poles = mcconf->si_motor_poles;
	if (poles <= 0) poles = 8;
	float pole_pairs = (float)poles * 0.5f;

	float gear_ratio = appconf->gear_detect_conf.internal_ratio;
	if (!isfinite(gear_ratio) || gear_ratio <= 0.0f) gear_ratio = 38.0f;

	// ========== TORQUE SENSOR: controls ERPM ramp rate ==========
	// Cadence base tracks instantly via PID. Torque controls how fast
	// ERPM ramps above cadence base. More stomp = faster climb.
	// Release = ramp down at ramp_down_rate back to cadence.
	// Uses speed PID's own config fields, not duty tab fields.
	float torque_ramp_erpm = 0.0f;  // ERPM/s from torque
	if (appconf->app_adc_conf.haz_torque_enable_current_pas) {
		systime_t torque_age = chVTTimeElapsedSinceX(s_ext_torque_ts);
		float torque_age_s = (float)torque_age / (float)CH_CFG_ST_FREQUENCY;
		float torque_timeout = fmaxf(appconf->app_adc_conf.haz_torque_idle_timeout, 0.1f);

		float torque_frac = 0.0f;
		if (torque_age_s < torque_timeout && s_ext_torque > 0) {
			torque_frac = (float)s_ext_torque / 160.0f;
		}

		float start_thresh = appconf->app_adc_conf.haz_torque_start_threshold / 160.0f;
		if (torque_frac < start_thresh) torque_frac = 0.0f;

		// LP filter
		float t_smooth = fmaxf(appconf->app_adc_conf.haz_torque_smoothing, 0.0f);
		if (t_smooth > 0.99f) t_smooth = 0.99f;
		float t_lp = (t_smooth < 0.01f) ? 1.0f : (1.0f - t_smooth * 0.99f);
		UTILS_LP_FAST(s_ext_torque_smooth, torque_frac, t_lp);

		// Expo shaping
		float resp = fmaxf(appconf->app_adc_conf.haz_torque_responsiveness, 0.0f);
		if (resp > 2.0f) resp = 2.0f;
		float x = fmaxf(s_ext_torque_smooth, 0.0f);
		if (x > 1.0f) x = 1.0f;
		float shaped;
		if (resp < 0.5f) {
			shaped = x * x;
		} else if (resp > 1.5f) {
			shaped = sqrtf(x);
		} else {
			float blend = (resp - 0.5f);
			shaped = (x * x) * (1.0f - blend) + sqrtf(fmaxf(x, 0.0f)) * blend;
		}

		// Torque → ERPM ramp rate (ERPM/s)
		// Full stomp (shaped=1) at strength=1: ramp at erpm_ramp_rate
		float strength = fmaxf(appconf->app_adc_conf.haz_torque_strength, 0.1f);
		if (strength > 5.0f) strength = 5.0f;
		torque_ramp_erpm = shaped * strength * erpm_ramp_rate;
	}

	// ========== CADENCE BASE: LINEAR INTERPOLATION ==========
	// EXTI ISR gives staircase cadence RPM (steps on each magnet edge).
	// We linearly interpolate between the previous and current ERPM target
	// over the edge period. No prediction, no filtering — just connecting
	// two known points with a straight line.
	//
	// On accel: new edge arrives with higher ERPM → start new ramp from
	//           wherever we are, instant direction change.
	// On decel: new edge arrives with lower ERPM → ramp down smoothly.
	// Constant: ramp completes, holds at target until next edge.
	float base_erpm_raw = pedal_rpm * gear_ratio * pole_pairs * target_lead;

	// Get edge period from ISR (time between A-edges, seconds)
	float edge_period = app_pas_get_step_period();

	// Detect new cadence edge: raw ERPM changed
	if (fabsf(base_erpm_raw - haz_pas_follow_ctx.interp_last_raw) > 1.0f) {
		// New edge arrived — start interpolation from current position to new target
		haz_pas_follow_ctx.interp_prev_erpm = haz_pas_follow_ctx.target_erpm_filtered;
		haz_pas_follow_ctx.interp_next_erpm = base_erpm_raw;
		haz_pas_follow_ctx.interp_elapsed = 0.0f;
		haz_pas_follow_ctx.interp_period = fmaxf(edge_period, 0.005f); // min 5ms
		haz_pas_follow_ctx.interp_last_raw = base_erpm_raw;
	}

	// Advance interpolation
	haz_pas_follow_ctx.interp_elapsed += dt_s;
	float t = haz_pas_follow_ctx.interp_elapsed / haz_pas_follow_ctx.interp_period;
	if (t > 1.0f) t = 1.0f;
	if (!isfinite(t)) t = 1.0f;

	haz_pas_follow_ctx.target_erpm_filtered =
		haz_pas_follow_ctx.interp_prev_erpm +
		(haz_pas_follow_ctx.interp_next_erpm - haz_pas_follow_ctx.interp_prev_erpm) * t;

	float max_erpm = fabsf(mcconf->l_max_erpm);
	if (max_erpm < 100.0f) max_erpm = 100000.0f;

	// ========== TORQUE BOOST: ramp ERPM above cadence ==========
	// While torque applied: ERPM climbs at torque_ramp_erpm (ERPM/s)
	// When released: falls back at ramp_down_rate to cadence base
	static float torque_erpm_extra = 0.0f;
	if (torque_ramp_erpm > 0.01f) {
		torque_erpm_extra += torque_ramp_erpm * dt_s;
	} else {
		torque_erpm_extra -= ramp_down_rate * dt_s;
	}
	if (torque_erpm_extra < 0.0f) torque_erpm_extra = 0.0f;

	// Clamp: don't let boost exceed max headroom (1s of ramp)
	float max_boost = erpm_ramp_rate * 1.0f;
	if (torque_erpm_extra > max_boost) torque_erpm_extra = max_boost;

	// Final target: cadence base (filtered) + torque boost
	float target_erpm = haz_pas_follow_ctx.target_erpm_filtered + torque_erpm_extra;
	if (target_erpm > max_erpm) target_erpm = max_erpm;
	if (target_erpm < 0.0f) target_erpm = 0.0f;

	// Anti-overshoot: don't run too far ahead of actual motor RPM
	float actual_rpm = fabsf(mc_interface_get_rpm());
	if (target_erpm > actual_rpm + max_boost) {
		target_erpm = actual_rpm + max_boost;
	}

	haz_pas_follow_ctx.target_erpm_smooth = target_erpm;

	// ========== COMMAND SPEED PID ==========
	mc_interface_set_pid_speed(haz_pas_follow_ctx.target_erpm_smooth);

	return 1.0f;
}

// ===========================================================================
// DIRECT TORQUE CURRENT CONTROL
// ===========================================================================
// Torque sensor → 3-point curve → phase current, bypassing Speed PID.
// Cadence gates current at low RPM, speed fades at the limit.
// Assist level and gear detection scale the output.

static float s_direct_torque_current = 0.0f;    // Ramped output current (A)
static float s_direct_torque_smooth = 0.0f;     // LP-filtered torque (0-1)

static void haz_direct_torque_reset(void) {
	s_direct_torque_current = 0.0f;
	s_direct_torque_smooth = 0.0f;
}

// 3-point piecewise linear curve.  Below in_low = 0, above in_high = out_high.
static float haz_torque_curve_3pt(float x,
		float in_low, float out_low,
		float in_mid, float out_mid,
		float in_high, float out_high) {
	if (x <= in_low) return 0.0f;
	if (x <= in_mid) {
		float t = (x - in_low) / fmaxf(in_mid - in_low, 0.001f);
		return out_low + t * (out_mid - out_low);
	}
	if (x <= in_high) {
		float t = (x - in_mid) / fmaxf(in_high - in_mid, 0.001f);
		return out_mid + t * (out_high - out_mid);
	}
	return out_high;
}

// Returns > 0 if direct torque is active (motor already commanded).
// Returns <= 0 if inactive — caller should fall through to Speed PID or release.
static float haz_direct_torque_process(float dt_s) {
	const app_configuration *appconf = app_get_configuration();
	const adc_config *ac = &appconf->app_adc_conf;

	if (!ac->haz_torque_direct_enable) {
		haz_direct_torque_reset();
		return -1.0f;
	}

	// --- Read torque sensor ---
	systime_t torque_age = chVTTimeElapsedSinceX(s_ext_torque_ts);
	float torque_age_s = (float)torque_age / (float)CH_CFG_ST_FREQUENCY;
	float torque_timeout = fmaxf(ac->haz_torque_idle_timeout, 0.1f);

	float torque_raw = 0.0f;
	if (torque_age_s < torque_timeout && s_ext_torque > 0) {
		torque_raw = (float)s_ext_torque / 160.0f;
	}

	// Deadzone
	float start_thresh = ac->haz_torque_start_threshold / 160.0f;
	if (torque_raw < start_thresh) torque_raw = 0.0f;

	// LP filter (reuses shared smoothing config)
	float t_smooth = fmaxf(ac->haz_torque_smoothing, 0.0f);
	if (t_smooth > 0.99f) t_smooth = 0.99f;
	float t_lp = (t_smooth < 0.01f) ? 1.0f : (1.0f - t_smooth * 0.99f);
	UTILS_LP_FAST(s_direct_torque_smooth, torque_raw, t_lp);

	// --- 3-point torque → current ratio curve ---
	float current_ratio = haz_torque_curve_3pt(s_direct_torque_smooth,
			ac->haz_torque_direct_in_low,  ac->haz_torque_direct_out_low,
			ac->haz_torque_direct_in_mid,  ac->haz_torque_direct_out_mid,
			ac->haz_torque_direct_in_high, ac->haz_torque_direct_out_high);

	float target_current = current_ratio * ac->haz_torque_direct_max_current;

	// --- Cadence gating ---
	// Uses PAS step count for robust idle detection (same as Speed PID PAS)
	float cadence_rpm = fabsf(app_pas_get_pedal_rpm());
	float cad_start = ac->haz_torque_direct_cadence_start;
	float cad_full  = ac->haz_torque_direct_cadence_full;
	float cad_min   = ac->haz_torque_direct_cadence_min;

	float cadence_scale;
	if (cadence_rpm <= cad_start) {
		cadence_scale = cad_min;
	} else if (cadence_rpm >= cad_full) {
		cadence_scale = 1.0f;
	} else {
		cadence_scale = cad_min + (1.0f - cad_min) *
			(cadence_rpm - cad_start) / fmaxf(cad_full - cad_start, 1.0f);
	}
	target_current *= cadence_scale;

	// --- Speed fade ---
	// Smooth current reduction approaching speed limit (street/offroad)
	if (!mc_interface_is_offroad_mode()) {
		const float speed_limit_ms = 25.0f / 3.6f;  // 25 km/h street
		float speed_fade_frac = ac->haz_torque_direct_speed_fade;
		float fade_start_ms = speed_limit_ms * speed_fade_frac;
		float wheel_speed = fabsf(mc_interface_get_speed());

		if (wheel_speed >= speed_limit_ms) {
			target_current = 0.0f;
		} else if (wheel_speed > fade_start_ms) {
			float fade = 1.0f - (wheel_speed - fade_start_ms) /
				fmaxf(speed_limit_ms - fade_start_ms, 0.1f);
			target_current *= fade;
		}
	}

	// --- Assist level scaling ---
	float assist_scale = mc_interface_get_assist_current_scale();
	target_current *= assist_scale;

	// --- Gear scaling (optional) ---
	if (ac->haz_torque_direct_use_gears && appconf->gear_detect_conf.enabled) {
		int gear = app_gear_detect_get_current();
		int num_gears = appconf->gear_detect_conf.num_gears;
		if (gear > 0 && num_gears > 1) {
			// Linear from 50% in gear 1 → 100% in top gear
			float gear_scale = 0.5f + 0.5f * (float)(gear - 1) / (float)(num_gears - 1);
			target_current *= gear_scale;
		}
	}

	// --- Clamp to motor config limits ---
	const volatile mc_configuration *mcconf = mc_interface_get_configuration();
	if (target_current > mcconf->l_current_max) {
		target_current = mcconf->l_current_max;
	}

	// --- Current ramping ---
	float ramp_up = ac->haz_torque_direct_ramp_up;
	float ramp_down = ac->haz_torque_direct_ramp_down;

	float delta = target_current - s_direct_torque_current;
	if (delta > 0.0f) {
		float max_up = ramp_up * dt_s;
		if (delta > max_up) delta = max_up;
	} else {
		float max_dn = ramp_down * dt_s;
		if (delta < -max_dn) delta = -max_dn;
	}
	s_direct_torque_current += delta;
	if (s_direct_torque_current < 0.0f) s_direct_torque_current = 0.0f;

	// --- Command motor ---
	if (s_direct_torque_current > 0.01f) {
		mc_interface_set_current(s_direct_torque_current);
		return s_direct_torque_current;
	} else {
		s_direct_torque_current = 0.0f;
		return -1.0f;
	}
}

static bool haz_pas_try_takeover(float *pwr, bool *p_current_mode, float loop_dt) {
	if (*pwr < 0.0f) {
		haz_pas_follow_reset();
		haz_direct_torque_reset();
		return false;
	}

	if (!app_pas_is_running()) {
		haz_pas_follow_reset();
		haz_direct_torque_reset();
		return false;
	}

	if (fabsf(*pwr) < PAS_THROTTLE_IDLE_GATE) {
		// Direct Torque mode takes priority over Speed PID PAS
		if (config.haz_torque_direct_enable) {
			float dt_ret = haz_direct_torque_process(loop_dt);
			if (dt_ret > 0.0f) {
				// Direct current active — motor already commanded
				*p_current_mode = false;  // Prevent downstream current control
				*pwr = 0.001f;            // Non-zero to prevent timeout release
				haz_pas_follow_reset();   // Keep speed PID reset
				return true;
			}
			// Direct torque returned no current — ramp down naturally
			// Don't fall through to speed PID when direct mode is enabled
			return false;
		}

		// Speed PID PAS (when direct torque is off)
		float pas_ret = haz_pas_follow_process(loop_dt);
		if (pas_ret > 0.0f) {
			// Speed PID active — motor already commanded by mc_interface_set_pid_speed.
			// Prevent downstream code from overriding with current control.
			*p_current_mode = false;
			*pwr = 0.001f;  // Non-zero to prevent timeout release
			return true;
		}
		return false;
	}

	haz_pas_follow_reset();
	haz_direct_torque_reset();
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

#if HAZ_TORQUE_UART_DIRECT
	// UART torque sensor uses HW_UART pins — never use as buttons
	(void)use_rx_tx;
	use_rx_tx_as_buttons = false;
	torque_uart_start();
#else
	if (buttons_detached) {
		use_rx_tx_as_buttons = false;
	} else {
		use_rx_tx_as_buttons = use_rx_tx;
	}
#endif

#ifdef HW_HAS_LUNA_SERIAL_DISPLAY
	// Bafang display uses HW_UART TX/RX — never read them as buttons
	use_rx_tx_as_buttons = false;
#endif

	stop_now = false;
	chThdCreateStatic(adc_thread_wa, sizeof(adc_thread_wa), NORMALPRIO, adc_thread, NULL);
}

void app_adc_stop(void) {
	stop_now = true;
	while (is_running) {
		chThdSleepMilliseconds(1);
	}
#if HAZ_TORQUE_UART_DIRECT
	torque_uart_stop();
#endif
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
	
	// Apply initial assist current limit based on mode and level
	apply_assist_current_limit();
	static bool last_offroad_mode = false;

	for(;;) {
		// Sleep for a time according to the specified rate
		systime_t sleep_time = CH_CFG_ST_FREQUENCY / config.update_rate_hz;

		// At least one tick should be slept to not block the other threads
		if (sleep_time == 0) {
			sleep_time = 1;
		}
		chThdSleep(sleep_time);
		
		// Check if offroad mode changed - update assist current limit
		bool current_offroad = mc_interface_is_offroad_mode();
		if (current_offroad != last_offroad_mode) {
			last_offroad_mode = current_offroad;
			apply_assist_current_limit();
		}

		// Update fused wheel speed: blends magnet sensor with ERPM-derived speed
		// Must run BEFORE anything that calls mc_interface_get_speed()
		app_gear_detect_update_fused_speed();

#if HAZ_TORQUE_UART_DIRECT
		// Drain TSDZ8 UART bytes — non-blocking
		torque_uart_poll();
#endif
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
				if (!haz_pas_try_takeover(&pwr, &current_mode, loop_dt)) {
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
				if (!haz_pas_try_takeover(&pwr, &current_mode, loop_dt)) {
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
				const float dt_s = (float)sleep_time / (float)CH_CFG_ST_FREQUENCY;

				float throttle_mag = fabsf(pwr);
				if (throttle_mag > 1.0f) throttle_mag = 1.0f;
				
				// Pass throttle position to FOC for FW scaling
				// (90-100% throttle = FW boost zone)
				mcpwm_foc_set_throttle_pos(throttle_mag);

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
							
													// Normal throttle: 0-100% maps to 0-100% duty
													// Assist level limits current via mc_interface.c limit enforcement
													float duty_target = throttle_mag * mcconf->l_max_duty * speed_scale;
													float duty_gap = fabsf(duty_target - osf_duty_ramped);
											// Street mode: VERY slow ramp (0.05/s) to avoid slingshot past speed limit
											// with low-resolution 1 pulse/rotation speed sensor
											const bool street_mode = !mc_interface_is_offroad_mode();
											const float ramp_up_slow = street_mode ? 0.05f : config.haz_hybrid_ramp_up_slow;
											const float ramp_up_fast = street_mode ? 0.05f : config.haz_hybrid_ramp_up_fast;
											const float ramp_down_slow = config.haz_hybrid_ramp_down_slow;
											const float ramp_down_fast = config.haz_hybrid_ramp_down_fast;

											// =============================================================================
											// HAZZA: Throttle Velocity + Gap-Based Ramp Scaling
											// Fast throttle movements = snappy response (up to 3x ramp speed)
											// Slow/micro throttle movements = precise duty control
											// Gap-based: big gap = fast ramp, small gap = smooth landing
											// Velocity mult has MEMORY — decays slowly so a fast slam sustains
											// the boost while duty catches up, not just for one cycle.
											// =============================================================================
											static float throttle_prev = 0.0f;
											static float velocity_mult_held = 1.0f;
											float throttle_velocity = fabsf(throttle_mag - throttle_prev) / dt_s;
											throttle_prev = throttle_mag;
											
											// Compute instantaneous velocity multiplier: 1x at rest, up to 1.5x
											float velocity_mult_instant = 1.0f;
											if (throttle_velocity > 0.5f) {
												velocity_mult_instant = 1.0f + fminf((throttle_velocity - 0.5f) / 2.5f, 1.0f) * 0.5f;
											}
											
											// Hold the peak and decay: fast slam sustains boost while ramp catches up
											if (velocity_mult_instant > velocity_mult_held) {
												velocity_mult_held = velocity_mult_instant;  // Instant peak capture
											} else {
												// Decay toward 1.0 over ~0.5s (blend rate 4/s)
												velocity_mult_held += (1.0f - velocity_mult_held) * fminf(4.0f * dt_s, 1.0f);
												if (velocity_mult_held < 1.0f) velocity_mult_held = 1.0f;
											}
											float velocity_mult = velocity_mult_held;
											
											// Gap-based scaling: big gap = fast ramp, small gap = smooth landing
											const float fast_threshold = 0.15f;  // Above 15% gap = full fast
											const float slow_threshold = 0.02f;  // Below 2% gap = full slow
											float gap_fraction = duty_gap / mcconf->l_max_duty;
											float gap_ratio;
											
											if (gap_fraction >= fast_threshold) {
												gap_ratio = 1.0f;  // Full fast
											} else if (gap_fraction <= slow_threshold) {
												gap_ratio = 0.0f;  // Full slow (smooth landing)
											} else {
												// Linear blend between thresholds
												gap_ratio = (gap_fraction - slow_threshold) / (fast_threshold - slow_threshold);
											}

											if (duty_target > osf_duty_ramped) {
												// Ramping UP — velocity mult directly scales the FINAL rate
												float ramp_rate = ramp_up_slow + (ramp_up_fast - ramp_up_slow) * gap_ratio;
												ramp_rate *= velocity_mult;  // Flick speed multiplies everything
												osf_duty_ramped += ramp_rate * dt_s;
												if (osf_duty_ramped > duty_target) osf_duty_ramped = duty_target;
											} else {
												// Ramping DOWN - apply both gap ratio and velocity mult
												float ramp_rate = ramp_down_slow + (ramp_down_fast - ramp_down_slow) * gap_ratio;
												ramp_rate *= velocity_mult;  // Fast throttle release = quicker response
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

						// Direct Torque mode takes priority — bypasses duty/speed PID entirely
						if (config.haz_torque_direct_enable) {
							float dt_ret = haz_direct_torque_process(dt_s);
							if (dt_ret > 0.0f) {
								// Direct current active — motor already commanded by set_current
								osf_duty_ramped = fabsf(mc_interface_get_duty_cycle_now());
								mc_interface_set_throttle_limit_active(true);
								haz_pas_follow_reset();
								// Don't send_duty — direct torque uses current control
							} else {
								// No torque input — release motor
								mc_interface_release_motor();
								osf_duty_ramped = 0.0f;
							}
						} else if (config.haz_pas_duty_enabled) {
							float pas_duty = haz_pas_duty_process(&config, dt_s);
							// Assist scaling handled by current_max_for_duty in FOC loop
							// Do NOT scale duty here — that would change motor RPM vs cadence
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
							// PAS duty disabled - use speed PID PAS follow
							float pas_ret = haz_pas_follow_process(dt_s);
							if (pas_ret > 0.0f) {
								// Speed PID active — motor already commanded
								osf_duty_ramped = fabsf(mc_interface_get_duty_cycle_now());
								mc_interface_set_throttle_limit_active(true);
								// DON'T set current_mode — speed PID handles it
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
