/*
	Copyright 2025 Hazza  Based on ESP32 display algorithm

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

#include "app_gear_detect.h"
#include "mc_interface.h"
#include "app.h"
#include <math.h>

// Hysteresis state
static int s_last_gear = 0;
static int s_pending_gear = 0;
static int s_consecutive_count = 0;

// Helper to get cassette teeth by index (0-11)
static inline uint8_t get_cassette_teeth(const gear_detection_config *conf, int idx) {
    switch (idx) {
        case 0:  return conf->cassette_teeth_1;
        case 1:  return conf->cassette_teeth_2;
        case 2:  return conf->cassette_teeth_3;
        case 3:  return conf->cassette_teeth_4;
        case 4:  return conf->cassette_teeth_5;
        case 5:  return conf->cassette_teeth_6;
        case 6:  return conf->cassette_teeth_7;
        case 7:  return conf->cassette_teeth_8;
        case 8:  return conf->cassette_teeth_9;
        case 9:  return conf->cassette_teeth_10;
        case 10: return conf->cassette_teeth_11;
        case 11: return conf->cassette_teeth_12;
        default: return 0;
    }
}

/**
 * @brief Calculate current gear from motor ERPM and wheel speed
 * 
 * Uses the known drivetrain ratios to work backwards from observed
 * motor speed vs wheel speed to determine which cassette cog is engaged.
 * 
 * Drivetrain: Motor → Internal Gearbox → Chainring → Cassette → Wheel
 * 
 * @param speed_kph  Wheel speed in km/h (from wheel speed sensor or GPS)
 * @param erpm       Motor electrical RPM
 * @param conf       Gear detection configuration
 * @return           Detected gear (1-N), 0 if uncertain/stationary
 */
static int detect_gear_raw(float speed_kph, int32_t erpm, const gear_detection_config *conf) {
    // Need both speed and ERPM to detect gear
    if (speed_kph < conf->min_speed_kph || erpm < conf->min_erpm) {
        return 0;
    }
    
    // Calculate wheel RPM from speed
    // circumference = π * diameter, convert mm to m
    float wheel_circumference_m = (float)conf->wheel_diameter_mm * 3.14159f / 1000.0f;
    // speed_kph * 1000m/km / 60min/h = m/min, divide by circumference = RPM
    float wheel_rpm = (speed_kph * 1000.0f / 60.0f) / wheel_circumference_m;
    
    // Calculate motor mechanical RPM from ERPM
    // ERPM = mechanical_RPM * (pole_pairs), pole_pairs = poles/2
    float motor_rpm = (float)erpm / ((float)conf->motor_poles / 2.0f);
    
    // Drivetrain math:
    // motor_rpm / internal_ratio = crank_rpm
    // crank_rpm * (chainring / cassette) = wheel_rpm
    // 
    // Solving for cassette teeth:
    // cassette = chainring * motor_rpm / (internal_ratio * wheel_rpm)
    float observed_cog_teeth = (float)conf->chainring_teeth * motor_rpm / 
                               (conf->internal_ratio * wheel_rpm);
    
    // Find closest matching gear
    int best_gear = 0;
    float best_diff = 999.0f;
    
    for (int g = 0; g < conf->num_gears && g < GEAR_MAX_GEARS; g++) {
        uint8_t cog_teeth = get_cassette_teeth(conf, g);
        if (cog_teeth == 0) continue;
        
        float diff = fabsf(observed_cog_teeth - (float)cog_teeth) / 
                     (float)cog_teeth;
        if (diff < best_diff && diff < conf->detect_tolerance) {
            best_diff = diff;
            best_gear = g + 1;  // 1-indexed gears
        }
    }
    
    return best_gear;
}

void app_gear_detect_reset(void) {
    s_last_gear = 0;
    s_pending_gear = 0;
    s_consecutive_count = 0;
}

int app_gear_detect_get(float speed_kph, int32_t erpm, float duty_cycle, float motor_amps) {
    const app_configuration *appconf = app_get_configuration();
    
    if (!appconf->gear_detect_conf.enabled) {
        return 0;
    }
    
    const gear_detection_config *conf = &appconf->gear_detect_conf;
    
    // Only calculate gear when chain is tensioned (motor current > 10A)
    // Hardcoded to match freewheel catch chain tension detection threshold
    bool under_load = (motor_amps > 10.0f);
    
    int raw_gear = detect_gear_raw(speed_kph, erpm, conf);
    int display_gear = s_last_gear;
    
    if (under_load && raw_gear > 0) {
        // Under load with valid reading - apply hysteresis
        if (raw_gear == s_pending_gear) {
            s_consecutive_count++;
            if (s_consecutive_count >= 3) {
                // 3 consecutive same readings - update
                display_gear = raw_gear;
                s_last_gear = raw_gear;
            }
        } else {
            // New gear detected, start counting
            s_pending_gear = raw_gear;
            s_consecutive_count = 1;
        }
    } else if (!under_load && s_last_gear > 0) {
        // Coasting - hold last known gear
        display_gear = s_last_gear;
    } else if (speed_kph < 1.0f) {
        // Stopped - clear gear
        display_gear = 0;
        s_last_gear = 0;
        s_consecutive_count = 0;
    }
    
    return display_gear;
}

/**
 * @brief Get gear from current VESC state (convenience function)
 * 
 * Pulls speed, ERPM, duty and current from mc_interface automatically.
 * Call this from your telemetry/display code.
 */
int app_gear_detect_get_current(void) {
    float speed_kph = mc_interface_get_speed() * 3.6f;  // m/s to km/h
    int32_t erpm = (int32_t)mc_interface_get_rpm();
    float duty = fabsf(mc_interface_get_duty_cycle_now()) * 100.0f;  // 0-1 to %
    float motor_amps = fabsf(mc_interface_get_tot_current_filtered());
    
    return app_gear_detect_get(speed_kph, erpm, duty, motor_amps);
}

/**
 * @brief Calculate target motor ERPM from wheel speed and gear
 * 
 * Used by freewheel catch to know what ERPM the motor should be at
 * to match the wheel speed in the given gear.
 * 
 * @param speed_kph  Wheel speed in km/h
 * @param gear       Gear number (1-indexed)
 * @return           Target ERPM, or 0 if invalid
 */
float app_gear_detect_target_erpm(float speed_kph, int gear) {
    const app_configuration *appconf = app_get_configuration();
    const gear_detection_config *conf = &appconf->gear_detect_conf;
    
    if (!conf->enabled || gear < 1 || gear > conf->num_gears || gear > GEAR_MAX_GEARS) {
        return 0.0f;
    }
    
    uint8_t cassette_teeth = get_cassette_teeth(conf, gear - 1);  // 0-indexed
    if (cassette_teeth == 0) {
        return 0.0f;
    }
    
    // Calculate wheel RPM from speed
    float wheel_circumference_m = (float)conf->wheel_diameter_mm * 3.14159f / 1000.0f;
    float wheel_rpm = (speed_kph * 1000.0f / 60.0f) / wheel_circumference_m;
    
    // Reverse the drivetrain math:
    // wheel_rpm = crank_rpm * (chainring / cassette)
    // crank_rpm = motor_rpm / internal_ratio
    // motor_rpm = wheel_rpm * internal_ratio * cassette / chainring
    // ERPM = motor_rpm * pole_pairs
    float motor_rpm = wheel_rpm * conf->internal_ratio * (float)cassette_teeth / (float)conf->chainring_teeth;
    float pole_pairs = (float)conf->motor_poles / 2.0f;
    float target_erpm = motor_rpm * pole_pairs;
    
    return target_erpm;
}

/**
 * @brief Get the last detected gear (for freewheel catch to remember gear while coasting)
 */
int app_gear_detect_get_last_gear(void) {
    return s_last_gear;
}
