/*
	Copyright 2025 Hazza

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

#ifndef APP_GEAR_DETECT_H_
#define APP_GEAR_DETECT_H_

#include <stdint.h>

/**
 * @brief Reset gear detection state
 * 
 * Call when starting fresh, e.g. after config change
 */
void app_gear_detect_reset(void);

/**
 * @brief Get detected gear from provided values
 * 
 * @param speed_kph    Wheel speed in km/h
 * @param erpm         Motor electrical RPM
 * @param duty_cycle   Current duty cycle (0-100%)
 * @param motor_amps   Motor current in Amps
 * @return             Detected gear (1-N), 0 if unknown
 */
int app_gear_detect_get(float speed_kph, int32_t erpm, float duty_cycle, float motor_amps);

/**
 * @brief Get detected gear from current VESC state
 * 
 * Convenience function that pulls values from mc_interface automatically.
 * @return             Detected gear (1-N), 0 if unknown
 */
int app_gear_detect_get_current(void);

/**
 * @brief Calculate target motor ERPM from wheel speed and gear
 * 
 * Used by freewheel catch to calculate what ERPM the motor should be at
 * to match the wheel speed in the given gear before engaging drivetrain.
 * 
 * @param speed_kph    Wheel speed in km/h
 * @param gear         Gear number (1-indexed)
 * @return             Target ERPM, or 0 if invalid
 */
float app_gear_detect_target_erpm(float speed_kph, int gear);

/**
 * @brief Get the last detected gear
 * 
 * Used by freewheel catch to remember gear while coasting (no load)
 * @return             Last detected gear (1-N), 0 if unknown
 */
int app_gear_detect_get_last_gear(void);

#endif /* APP_GEAR_DETECT_H_ */
