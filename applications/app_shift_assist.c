/*
	Copyright 2026 Hazza

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
	along with this program. If not, see <http://www.gnu.org/licenses/>.
*/

#include "app_shift_assist.h"
#include "app.h"
#include "app_gear_detect.h"
#include "mc_interface.h"
#include "commands.h"
#include "datatypes.h"

#include <math.h>
#include <stdatomic.h>

/* ------------------------------------------------------------------------- */
/* State                                                                     */
/* ------------------------------------------------------------------------- */

typedef struct {
	/* Tunables (latched from app_configuration on configure()) */
	bool   enabled;
	float  min_throttle;
	float  slip_pct;
	float  fallback_drop_pct;
	float  drop_rate;          /* duty/s */
	float  unload_offset;      /* added to gear-table-derived target */
	float  hold_s;
	float  reload_current_a;
	float  wait_timeout_s;
	float  restore_rate;       /* duty/s */
	float  lockout_s;
	float  debounce_s;

	/* Atomic flag from ISR/external */
	atomic_int trigger_pending;

	/* FSM */
	shift_assist_state_t state;
	float  state_timer;        /* seconds in current state */

	/* Snapshots taken at SHIFT_DETECTED */
	float  ceiling;            /* current duty ceiling */
	float  target_unload_duty; /* where to drop to */
	float  rider_at_trigger;
	float  current_baseline;   /* filtered iq baseline at WAIT entry */

	/* Filtered motor current (LP) */
	float  iq_filt;
	bool   iq_filt_init;

	/* Debounce / lockout */
	float  since_last_trigger;
	float  lockout_remaining;
} shift_ctx_t;

static shift_ctx_t S = {
	.enabled = false,
	.state = SHIFT_ASSIST_IDLE,
};

/* ------------------------------------------------------------------------- */
/* Helpers                                                                   */
/* ------------------------------------------------------------------------- */

static inline float clampf(float v, float lo, float hi) {
	if (v < lo) return lo;
	if (v > hi) return hi;
	return v;
}

static bool s_last_active_pushed = false;

/* Push "HX" + active-byte to commands stream when active state flips.
 * Consumed by ESP32 to light a UI indicator. */
static void push_active_state(bool active) {
	if (active == s_last_active_pushed) return;
	s_last_active_pushed = active;
	uint8_t buf[3];
	buf[0] = 0x48;          /* 'H' */
	buf[1] = 0x58;          /* 'X' */
	buf[2] = active ? 1 : 0;
	commands_send_app_data(buf, sizeof(buf));
}

static void enter_state(shift_assist_state_t s) {
	S.state = s;
	S.state_timer = 0.0f;
}

static void update_iq_filter(float dt_s) {
	const float tau = 0.030f;     /* 30 ms */
	const float alpha = (tau > 0.0f) ? (dt_s / (tau + dt_s)) : 1.0f;
	const float iq_now = fabsf(mc_interface_get_tot_current_filtered());
	if (!S.iq_filt_init) {
		S.iq_filt = iq_now;
		S.iq_filt_init = true;
	} else {
		S.iq_filt += alpha * (iq_now - S.iq_filt);
	}
}

static float compute_unload_duty(float current_duty, float rider_duty) {
	int gear = app_gear_detect_get_current();
	if (gear <= 0) {
		gear = app_gear_detect_get_last_gear();
	}

	float target;
	const float slip = clampf(S.slip_pct, 0.0f, 0.95f);

	if (gear > 0) {
		/* Gear-table-derived: motor ERPM that would match wheel speed in this
		 * gear, scaled down by slip_pct to land slightly behind wheel. Then
		 * back-solve duty by ratio with current ERPM (duty ~ ERPM at fixed VBus). */
		float wheel_kph = fabsf(mc_interface_get_speed()) * 3.6f;
		float match_erpm = app_gear_detect_target_erpm(wheel_kph, gear);
		float current_erpm = fabsf(mc_interface_get_rpm());
		if (match_erpm > 1.0f && current_erpm > 1.0f && current_duty > 0.05f) {
			float target_erpm = match_erpm * (1.0f - slip);
			target = current_duty * (target_erpm / current_erpm);
		} else {
			target = current_duty * (1.0f - slip);
		}
	} else {
		float drop = clampf(S.fallback_drop_pct, 0.0f, 0.95f);
		target = current_duty * (1.0f - drop);
	}

	/* User offset nudge (applied last, can be +/-) */
	target += S.unload_offset;

	/* Never command above rider; never negative. */
	if (target < 0.0f) target = 0.0f;
	if (target > rider_duty) target = rider_duty;
	return target;
}

/* ------------------------------------------------------------------------- */
/* Public API                                                                */
/* ------------------------------------------------------------------------- */

void app_shift_assist_init(void) {
	S.state = SHIFT_ASSIST_IDLE;
	S.state_timer = 0.0f;
	S.ceiling = 1.0f;
	S.iq_filt = 0.0f;
	S.iq_filt_init = false;
	S.since_last_trigger = 1e6f;
	S.lockout_remaining = 0.0f;
	atomic_store(&S.trigger_pending, 0);
}

void app_shift_assist_configure(const adc_config *c) {
	if (c == NULL) {
		S.enabled = false;
		return;
	}
	S.enabled            = c->haz_shift_assist_enabled;
	S.min_throttle       = c->haz_shift_min_throttle;
	S.slip_pct           = c->haz_shift_slip_pct;
	S.fallback_drop_pct  = c->haz_shift_fallback_drop_pct;
	S.drop_rate          = c->haz_shift_drop_rate;
	S.unload_offset      = c->haz_shift_unload_offset;
	S.hold_s             = (float)c->haz_shift_hold_ms * 0.001f;
	S.reload_current_a   = c->haz_shift_reload_current_a;
	S.wait_timeout_s     = (float)c->haz_shift_wait_timeout_ms * 0.001f;
	S.restore_rate       = c->haz_shift_restore_rate;
	S.lockout_s          = (float)c->haz_shift_lockout_ms * 0.001f;
	S.debounce_s         = (float)c->haz_shift_debounce_ms * 0.001f;

	if (!S.enabled) {
		app_shift_assist_abort();
	}
}

void app_shift_assist_trigger(void) {
	atomic_store(&S.trigger_pending, 1);
}

void app_shift_assist_abort(void) {
	enter_state(SHIFT_ASSIST_IDLE);
	S.ceiling = 1.0f;
	atomic_store(&S.trigger_pending, 0);
	push_active_state(false);
}

shift_assist_state_t app_shift_assist_state(void) {
	return S.state;
}

float app_shift_assist_apply(float rider_duty_target, float current_duty, float dt_s) {
	/* Disabled or invalid dt -> return ceiling=1.0 (unconstrained). The caller
	 * does min(ceiling, throttle_ramped); returning rider_duty_target here
	 * would bypass the throttle ramp on every tick. */
	if (!S.enabled || dt_s <= 0.0f || dt_s > 0.5f) {
		return 1.0f;
	}

	/* Always advance internal timers */
	S.since_last_trigger += dt_s;
	if (S.lockout_remaining > 0.0f) {
		S.lockout_remaining -= dt_s;
		if (S.lockout_remaining < 0.0f) S.lockout_remaining = 0.0f;
	}

	update_iq_filter(dt_s);

	/* Consume trigger flag (atomic) */
	int pending = atomic_exchange(&S.trigger_pending, 0);

	/* Validate trigger:
	 *   - debounce since last accepted edge
	 *   - lockout still active
	 *   - rider must hold throttle above min
	 *   - must currently be IDLE (no nesting)
	 */
	if (pending) {
		bool accept = (S.state == SHIFT_ASSIST_IDLE)
		           && (S.lockout_remaining <= 0.0f)
		           && (S.since_last_trigger >= S.debounce_s)
		           && (rider_duty_target >= S.min_throttle);
		if (accept) {
			S.since_last_trigger = 0.0f;
			S.rider_at_trigger = rider_duty_target;
			S.target_unload_duty = compute_unload_duty(current_duty, rider_duty_target);
			S.ceiling = current_duty;   /* start ceiling at current */
			enter_state(SHIFT_ASSIST_SHIFT_DETECTED);
		}
		/* Rejected triggers are silently dropped. */
	}

	S.state_timer += dt_s;

	switch (S.state) {
	case SHIFT_ASSIST_IDLE:
		return 1.0f;

	case SHIFT_ASSIST_SHIFT_DETECTED:
		/* One-tick latch; recompute target in case duty moved a bit */
		S.target_unload_duty = compute_unload_duty(current_duty, S.rider_at_trigger);
		enter_state(SHIFT_ASSIST_DUTY_REDUCTION);
		break;

	case SHIFT_ASSIST_DUTY_REDUCTION: {
		float step = S.drop_rate * dt_s;
		if (S.ceiling > S.target_unload_duty) {
			S.ceiling -= step;
			if (S.ceiling < S.target_unload_duty) S.ceiling = S.target_unload_duty;
		}
		if (S.ceiling <= S.target_unload_duty + 1e-4f) {
			enter_state(SHIFT_ASSIST_HOLD_UNLOADED);
		}
		break;
	}

	case SHIFT_ASSIST_HOLD_UNLOADED:
		S.ceiling = S.target_unload_duty;
		if (S.state_timer >= S.hold_s) {
			S.current_baseline = S.iq_filt;
			enter_state(SHIFT_ASSIST_WAIT_RELOAD);
		}
		break;

	case SHIFT_ASSIST_WAIT_RELOAD: {
		S.ceiling = S.target_unload_duty;
		bool reload = (S.iq_filt - S.current_baseline) >= S.reload_current_a;
		bool timeout = S.state_timer >= S.wait_timeout_s;
		if (reload || timeout) {
			enter_state(SHIFT_ASSIST_RAMP_RESTORE);
		}
		break;
	}

	case SHIFT_ASSIST_RAMP_RESTORE: {
		float step = S.restore_rate * dt_s;
		S.ceiling += step;
		if (S.ceiling >= rider_duty_target) {
			S.ceiling = rider_duty_target;
			enter_state(SHIFT_ASSIST_RETURN_IDLE);
		}
		break;
	}

	case SHIFT_ASSIST_RETURN_IDLE:
		S.lockout_remaining = S.lockout_s;
		S.ceiling = 1.0f;
		enter_state(SHIFT_ASSIST_IDLE);
		push_active_state(false);
		return 1.0f;

	default:
		enter_state(SHIFT_ASSIST_IDLE);
		push_active_state(false);
		return 1.0f;
	}

	/* Active states: emit "active" indicator on first non-idle tick. */
	push_active_state(S.state != SHIFT_ASSIST_IDLE);

	/* Active states: return ceiling. Caller does min(ceiling, throttle_ramped),
	 * so we MUST NOT clamp to rider here -- doing so bypasses throttle ramp-down
	 * when rider releases throttle mid-shift. */
	float out = S.ceiling;
	if (out < 0.0f) out = 0.0f;
	if (out > 1.0f) out = 1.0f;
	return out;
}
