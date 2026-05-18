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

/*
 * Paddle-shift style throttle gear shift assist.
 *
 * Active ONLY in ADC_CTRL_TYPE_CURRENT_HYBRID_DUTY rider-throttle branch.
 * Returns a duty CEILING — caller does final = min(rider, ceiling).
 * Never increases duty above rider command.
 *
 * Trigger is RT-safe (ISR-callable) — sets an atomic flag consumed in the
 * app_adc thread.
 */

#ifndef APP_SHIFT_ASSIST_H_
#define APP_SHIFT_ASSIST_H_

#include "conf_general.h"
#include <stdint.h>
#include <stdbool.h>

typedef enum {
	SHIFT_ASSIST_IDLE = 0,
	SHIFT_ASSIST_SHIFT_DETECTED,
	SHIFT_ASSIST_DUTY_REDUCTION,
	SHIFT_ASSIST_HOLD_UNLOADED,
	SHIFT_ASSIST_WAIT_RELOAD,
	SHIFT_ASSIST_RAMP_RESTORE,
	SHIFT_ASSIST_RETURN_IDLE
} shift_assist_state_t;

void  app_shift_assist_init(void);
void  app_shift_assist_configure(const adc_config *conf);

/** ISR-safe: arms the FSM. Will be filtered by debounce/lockout. */
void  app_shift_assist_trigger(void);

/**
 * Per-cycle update. Must be called from app_adc thread once per tick when
 * inside the rider-throttle HYBRID_DUTY branch.
 *
 * @param rider_duty_target  rider's commanded duty (post speed_scale, abs)
 * @param current_duty       current osf_duty_ramped (abs)
 * @param dt_s               loop period in seconds
 * @return                   ceiling duty (>= 0). Caller clamps:
 *                             out = min(current, ceiling)
 *                           When IDLE, returns rider_duty_target.
 */
float app_shift_assist_apply(float rider_duty_target, float current_duty, float dt_s);

/** Force FSM to IDLE (call on brake, throttle release, fault, mode change). */
void  app_shift_assist_abort(void);

/** For diagnostics / telemetry. */
shift_assist_state_t app_shift_assist_state(void);

#endif /* APP_SHIFT_ASSIST_H_ */
