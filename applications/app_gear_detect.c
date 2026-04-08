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
#include <stdbool.h>

// Fused speed state
static float s_fused_speed_ms = 0.0f;     // Current fused wheel speed in m/s
static bool s_fused_active = false;        // True when ERPM-derived speed is being used
static float s_magnet_speed_ms = 0.0f;     // Last raw magnet sensor speed for validation

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
    
    // Only calculate gear when chain is tensioned (motor current > 5A)
    // Lowered from 10A to catch gear changes during brief load dips mid-shift
    bool under_load = (motor_amps > 5.0f);
    
    int raw_gear = detect_gear_raw(speed_kph, erpm, conf);
    int display_gear = s_last_gear;
    
    if (under_load && raw_gear > 0) {
        // Under load with valid reading - apply hysteresis
        if (raw_gear == s_pending_gear) {
            s_consecutive_count++;
            if (s_consecutive_count >= 2) {
                // 2 consecutive same readings - update (was 3, reduced for faster response)
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
 * Uses RAW magnet sensor speed (not fused) to avoid circular dependency.
 * Gear detection must always use the magnet as ground truth.
 */
int app_gear_detect_get_current(void) {
    // Use raw hardware speed, NOT mc_interface_get_speed() which returns fused value
#ifdef HW_HAS_WHEEL_SPEED_SENSOR
    float speed_ms = hw_get_speed();  // Raw magnet sensor m/s
#else
    float speed_ms = mc_interface_get_speed();  // Fallback: ERPM-based
#endif
    float speed_kph = speed_ms * 3.6f;
    int32_t erpm = (int32_t)mc_interface_get_rpm();
    float duty = fabsf(mc_interface_get_duty_cycle_now()) * 100.0f;
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

/**
 * @brief Calculate wheel speed from motor ERPM and known gear
 * 
 * Inverse of app_gear_detect_target_erpm: given ERPM and gear, compute wheel speed.
 * 
 * @param erpm     Motor electrical RPM
 * @param gear     Gear number (1-indexed)
 * @return         Wheel speed in m/s, or 0 if invalid
 */
static float erpm_to_wheel_speed_ms(float erpm, int gear) {
    const app_configuration *appconf = app_get_configuration();
    const gear_detection_config *conf = &appconf->gear_detect_conf;
    
    if (!conf->enabled || gear < 1 || gear > conf->num_gears || gear > GEAR_MAX_GEARS) {
        return 0.0f;
    }
    
    uint8_t cassette_teeth = get_cassette_teeth(conf, gear - 1);
    if (cassette_teeth == 0) return 0.0f;
    
    // Reverse the chain: ERPM → motor RPM → crank RPM → wheel RPM → m/s
    float pole_pairs = (float)conf->motor_poles / 2.0f;
    float motor_rpm = fabsf(erpm) / pole_pairs;
    // motor_rpm / internal_ratio = crank_rpm
    // crank_rpm * chainring / cassette = wheel_rpm
    float wheel_rpm = motor_rpm / conf->internal_ratio * (float)conf->chainring_teeth / (float)cassette_teeth;
    // wheel_rpm → m/s
    float wheel_circumference_m = (float)conf->wheel_diameter_mm * 3.14159f / 1000.0f;
    float speed_ms = (wheel_rpm / 60.0f) * wheel_circumference_m;
    
    return speed_ms;
}

/**
 * @brief Re-detect gear using magnet speed as ground truth
 * 
 * When we have a fresh magnet pulse and ERPM, we can directly solve for the
 * gear without needing load thresholds or hysteresis — the magnet is truth.
 * This is called when fused speed disagrees with the magnet sensor.
 * 
 * @param magnet_speed_kph  Wheel speed from magnet sensor in km/h
 * @param erpm              Motor electrical RPM (absolute)
 * @return                  Detected gear, or 0 if can't determine
 */
static int redetect_gear_from_magnet(float magnet_speed_kph, float erpm) {
    const app_configuration *appconf = app_get_configuration();
    const gear_detection_config *conf = &appconf->gear_detect_conf;
    
    // Use slightly looser tolerance for magnet-based correction
    // since magnet is 1 pulse/rev and may have some jitter
    return detect_gear_raw(magnet_speed_kph, (int32_t)erpm, conf);
}

/**
 * @brief Update fused wheel speed — call every app_adc cycle
 * 
 * Blends magnet sensor (low-res, always-accurate) with ERPM-derived speed
 * (high-res, only valid when chain is engaged). Like hall+observer fusion:
 *   - Magnet sensor = ground truth, anchors gear detection
 *   - ERPM + gear = high-frequency interpolation between magnet pulses
 *   - Current threshold determines chain engagement (trust ERPM)
 *   - On each magnet pulse, validate the ERPM-derived speed is sane
 *   - If speed disagrees, re-detect gear from magnet+ERPM immediately
 * 
 * Result is injected via mc_interface_override_wheel_speed() so all
 * downstream code (telemetry, speed limits, freewheel catch) benefits.
 */
void app_gear_detect_update_fused_speed(void) {
    const app_configuration *appconf = app_get_configuration();
    
    if (!appconf->gear_detect_conf.enabled) {
        // Gear detection disabled — don't override, use raw sensor
        if (s_fused_active) {
            mc_interface_override_wheel_speed(false, 0.0f);
            s_fused_active = false;
        }
        return;
    }
    
    // Get raw magnet sensor speed (always truth)
#if !defined(HW_HAS_WHEEL_SPEED_SENSOR) || defined(HW_WHEEL_SPEED_USE_GPIO)
    // No hardware sensor, or GPIO-based sensor — use hw_get_speed() directly,
    // no ERPM fusion needed (fusion is for ADC-based sensors with gear detect)
    return;
#else
    float magnet_speed_ms = hw_get_speed();  // m/s from magnet sensor
    uint32_t pulse_age_ms = hw_wheel_speed_get_pulse_age_ms();
    
    s_magnet_speed_ms = magnet_speed_ms;
    
    float motor_current = fabsf(mc_interface_get_tot_current_filtered());
    float erpm = fabsf(mc_interface_get_rpm());
    int gear = s_last_gear;  // Use internal state directly, no circular call
    
    // Thresholds for chain engagement detection
    const float current_engage_threshold = 3.0f;   // Above 3A = chain definitely engaged
    const float current_disengage_threshold = 1.5f; // Below 1.5A = chain may be slack (hysteresis)
    const float min_erpm_for_fusion = 300.0f;       // Need some ERPM for meaningful calculation
    const float max_pulse_age_for_trust = 1500;     // If magnet pulse >1.5s old, speed is stale
    
    // ========================================================================
    // MAGNET-ANCHORED GEAR CORRECTION
    // Every fresh magnet pulse is a chance to correct the gear if it's wrong.
    // This catches gear changes that the load-based detection misses (e.g.
    // shifting under power where current briefly dips during the shift).
    // ========================================================================
    if (magnet_speed_ms > 1.0f && pulse_age_ms < max_pulse_age_for_trust && 
        erpm > min_erpm_for_fusion && gear > 0) {
        float erpm_speed_ms = erpm_to_wheel_speed_ms(erpm, gear);
        float ratio = (erpm_speed_ms > 0.1f) ? (erpm_speed_ms / magnet_speed_ms) : 0.0f;
        
        if (ratio < 0.8f || ratio > 1.25f) {
            // >20% disagreement — current gear is wrong. Re-detect immediately
            // using magnet speed as ground truth (no load or hysteresis required)
            float magnet_kph = magnet_speed_ms * 3.6f;
            int corrected_gear = redetect_gear_from_magnet(magnet_kph, erpm);
            
            if (corrected_gear > 0 && corrected_gear != gear) {
                // Validate: does the corrected gear produce a speed that matches magnet?
                float corrected_speed = erpm_to_wheel_speed_ms(erpm, corrected_gear);
                float corrected_ratio = corrected_speed / magnet_speed_ms;
                
                if (corrected_ratio > 0.85f && corrected_ratio < 1.18f) {
                    // Good match — adopt immediately, skip hysteresis
                    s_last_gear = corrected_gear;
                    s_pending_gear = corrected_gear;
                    s_consecutive_count = 3;  // Pre-saturate so normal path doesn't fight us
                    gear = corrected_gear;    // Use corrected gear for fusion below
                }
            }
        }
    }
    
    // Determine if chain is engaged (with hysteresis)
    static bool chain_engaged = false;
    if (motor_current > current_engage_threshold && erpm > min_erpm_for_fusion) {
        chain_engaged = true;
    } else if (motor_current < current_disengage_threshold || erpm < min_erpm_for_fusion * 0.5f) {
        chain_engaged = false;
    }
    
    if (chain_engaged && gear > 0) {
        // Chain is engaged and we know the gear — use ERPM-derived speed
        float erpm_speed_ms = erpm_to_wheel_speed_ms(erpm, gear);
        
        if (erpm_speed_ms > 0.1f) {
            // Sanity check against magnet sensor when pulse is fresh
            bool erpm_sane = true;
            if (magnet_speed_ms > 0.5f && pulse_age_ms < max_pulse_age_for_trust) {
                float ratio = erpm_speed_ms / magnet_speed_ms;
                if (ratio < 0.75f || ratio > 1.35f) {
                    // Still disagrees even after gear correction — maybe mid-shift
                    erpm_sane = false;
                }
            }
            
            if (erpm_sane) {
                // Smooth transition: LP filter the fused speed toward ERPM-derived
                // Fast filter (0.3) so ERPM speed is responsive but not jumpy
                float alpha = 0.3f;
                if (!s_fused_active) {
                    // First engagement — snap to ERPM speed (avoid lag)
                    s_fused_speed_ms = erpm_speed_ms;
                } else {
                    s_fused_speed_ms += alpha * (erpm_speed_ms - s_fused_speed_ms);
                }
                s_fused_active = true;
                mc_interface_override_wheel_speed(true, s_fused_speed_ms);
                return;
            }
        }
    }
    
    // Not fusing — use raw magnet sensor (default path)
    // Smooth transition back: don't snap, blend toward magnet speed
    if (s_fused_active) {
        if (magnet_speed_ms > 0.1f) {
            // Blend back toward magnet speed over a few cycles
            float alpha = 0.15f;
            s_fused_speed_ms += alpha * (magnet_speed_ms - s_fused_speed_ms);
            
            // Close enough — release override
            if (fabsf(s_fused_speed_ms - magnet_speed_ms) < 0.3f) {
                mc_interface_override_wheel_speed(false, 0.0f);
                s_fused_active = false;
            } else {
                mc_interface_override_wheel_speed(true, s_fused_speed_ms);
            }
        } else {
            // Stopped — release immediately
            mc_interface_override_wheel_speed(false, 0.0f);
            s_fused_active = false;
            s_fused_speed_ms = 0.0f;
        }
    }
#endif /* HW_HAS_WHEEL_SPEED_SENSOR */
}

/**
 * @brief Check if fused speed is currently active (ERPM-derived)
 */
bool app_gear_detect_fused_speed_active(void) {
    return s_fused_active;
}
