/*
 * Generic wheel speed sensor implementation for VESC-based mid-drives
 * 
 * Include this file in your hw_xxx.c after defining:
 *   HW_HAS_WHEEL_SPEED_SENSOR
 *   HW_WHEEL_SPEED_ADC_CH     - ADC channel index (e.g., ADC_IND_TEMP_MOTOR)
 *   HW_WHEEL_SPEED_THRESH_HIGH - ADC value above which = no magnet
 *   HW_WHEEL_SPEED_THRESH_LOW  - ADC value below which = magnet present
 *   HW_WHEEL_SPEED_MAGNETS    - Number of magnets on wheel
 *
 * This uses the motor temp pin (or other ADC pin) with the existing 10K 
 * resistor network to detect a hall-type wheel speed sensor.
 * 
 * Compatible with TSDZ8 and similar mid-drive conversions.
 */

#ifndef HW_WHEEL_SPEED_GENERIC_H_
#define HW_WHEEL_SPEED_GENERIC_H_

#ifdef HW_HAS_WHEEL_SPEED_SENSOR

// State machine for edge detection with hysteresis
typedef enum {
	WHEEL_SPEED_STATE_UNKNOWN = 0,
	WHEEL_SPEED_STATE_HIGH,		// No magnet - ADC pulled high by 10K
	WHEEL_SPEED_STATE_LOW		// Magnet present - sensor pulls low
} wheel_speed_state_t;

static volatile uint32_t wheel_pulse_count = 0;
static volatile uint32_t wheel_last_pulse_time = 0;
static volatile uint32_t wheel_pulse_period_us = 0;
static volatile wheel_speed_state_t wheel_state = WHEEL_SPEED_STATE_UNKNOWN;
static volatile float wheel_speed_rps = 0.0f;

// Call this from mc_interface timer task (called every 1ms)
// ADC_Value[] is updated at FOC rate (~30kHz), so we get good resolution
void hw_update_speed_sensor(void) {
	uint16_t adc_val = ADC_Value[HW_WHEEL_SPEED_ADC_CH];
	uint32_t now_us = chVTGetSystemTimeX() * (1000000 / CH_CFG_ST_FREQUENCY);
	
	wheel_speed_state_t new_state = wheel_state;
	
	// Hysteresis state machine to avoid noise triggering false edges
	switch (wheel_state) {
	case WHEEL_SPEED_STATE_UNKNOWN:
		// Initialize based on current reading
		if (adc_val > HW_WHEEL_SPEED_THRESH_HIGH) {
			new_state = WHEEL_SPEED_STATE_HIGH;
		} else if (adc_val < HW_WHEEL_SPEED_THRESH_LOW) {
			new_state = WHEEL_SPEED_STATE_LOW;
		}
		break;
		
	case WHEEL_SPEED_STATE_HIGH:
		// Looking for falling edge (magnet arrives)
		if (adc_val < HW_WHEEL_SPEED_THRESH_LOW) {
			new_state = WHEEL_SPEED_STATE_LOW;
			
			// Count pulse on falling edge
			wheel_pulse_count++;
			
			// Calculate period
			if (wheel_last_pulse_time > 0) {
				uint32_t period = now_us - wheel_last_pulse_time;
				if (period > 1000 && period < 10000000) {  // 0.1 RPM to 60000 RPM sanity check
					wheel_pulse_period_us = period;
					// RPS = 1 / (period_sec * magnets)
					wheel_speed_rps = 1000000.0f / ((float)period * HW_WHEEL_SPEED_MAGNETS);
				}
			}
			wheel_last_pulse_time = now_us;
		}
		break;
		
	case WHEEL_SPEED_STATE_LOW:
		// Looking for rising edge (magnet leaves)
		if (adc_val > HW_WHEEL_SPEED_THRESH_HIGH) {
			new_state = WHEEL_SPEED_STATE_HIGH;
		}
		break;
	}
	
	wheel_state = new_state;
	
	// Timeout: if no pulse for 2 seconds, assume stopped
	if (wheel_last_pulse_time > 0 && (now_us - wheel_last_pulse_time) > 2000000) {
		wheel_speed_rps = 0.0f;
		wheel_pulse_period_us = 0;
	}
}

// Standard VESC interface - returns wheel revs per second
float hw_get_speed(void) {
	return wheel_speed_rps;
}

// Returns absolute distance in meters (requires wheel diameter from config)
// For now, returns pulse count * wheel circumference / magnets
// This is a placeholder - proper implementation needs wheel_diameter from mcconf
float hw_get_distance_abs(void) {
	// Assume 0.7m wheel diameter as default (700c bike wheel)
	// TODO: Get this from mc_interface_get_configuration()->si_wheel_diameter
	const float wheel_diameter = 0.7f;
	const float wheel_circumference = wheel_diameter * 3.14159f;
	return (float)wheel_pulse_count * wheel_circumference / (float)HW_WHEEL_SPEED_MAGNETS;
}

uint32_t hw_wheel_speed_get_pulses(void) {
	return wheel_pulse_count;
}

uint16_t hw_wheel_speed_get_adc_raw(void) {
	return ADC_Value[HW_WHEEL_SPEED_ADC_CH];
}

#endif // HW_HAS_WHEEL_SPEED_SENSOR

#endif // HW_WHEEL_SPEED_GENERIC_H_
