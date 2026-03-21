/*
	Flipsky 6.7 Pro / Mini MK5 — E-Bike configuration
	
	Kartman's build:
	  Motor:        Giant SyncDrive mid-drive
	  Display:      Bafang S-LCD (luna serial protocol, 1200 baud)
	  PAS sensor:   Bafang quadrature PAS (24 magnets)
	  Speed sensor: Wheel speed → PPM pin
	  Throttle:     Standard → ADC 1
	  Temp sensor:  Motor temp → TEMP (ADC ch 9)
	  Power:        Antispark switch — NO momentary power button

	Wiring:
	  PAS 1       → SW DIO  (GPIOB 12, shared with NRF5x SWDIO — not used at runtime)
	  PAS 2       → SW CLK  (GPIOA 4,  shared with NRF5x SWCLK — not used at runtime)
	  Speed sensor → PPM     (GPIOB 6,  ICU/servo input)
	  Temp sensor  → TEMP    (ADC ch 9, real motor temp — NOT repurposed)
	  Throttle     → ADC 1   (ADC ch 6, standard)
	  Bafang display → TX/RX (USART3, GPIOB 10/11)
*/

#ifndef HW_FLIPSKY_60_MK5_EBIKE_H_
#define HW_FLIPSKY_60_MK5_EBIKE_H_

#define HW60_IS_MK5

#include "hw_flipsky_60_core.h"

// ── No power button — antispark switch only ──
// Remove the entire shutdown-pin mechanism that MK5 core.h defines.
// Without a real momentary power button on GPIOC5, the shutdown sampling
// thread would read a floating pin and could randomly cut the gate driver.
// We keep only the current-filter init from HW_EARLY_INIT.
#undef HW_SHUTDOWN_GPIO
#undef HW_SHUTDOWN_PIN
#undef HW_SHUTDOWN_HOLD_ON
#undef HW_SHUTDOWN_HOLD_OFF
#undef HW_SAMPLE_SHUTDOWN
#undef HW_EARLY_INIT
#define HW_EARLY_INIT()		palSetPadMode(GPIOD, 2, \
							PAL_MODE_OUTPUT_PUSHPULL | \
							PAL_STM32_OSPEED_HIGHEST); \
							CURRENT_FILTER_ON()

// ── App defaults — preconfigured so Kartman only tunes motor params ──
// core.h sets APPCONF_APP_TO_USE to APP_NONE — override it
#undef APPCONF_APP_TO_USE
#define APPCONF_APP_TO_USE				APP_ADC		// Throttle + PAS sidecar (NOT APP_ADC_UART — display owns UART)
#define APPCONF_ADC_CTRL_TYPE			ADC_CTRL_TYPE_CURRENT_HYBRID_DUTY	// Hazza hybrid duty
#define APPCONF_ADC_SAFE_START			true		// Don't start if throttle is held on power-up
#define APPCONF_ADC_USE_FILTER			true		// Filter ADC readings
#define APPCONF_ADC_BUTTONS				0			// No buttons on UART pins (display uses them)
#define APPCONF_SHUTDOWN_MODE			SHUTDOWN_MODE_ALWAYS_ON	// No power button — stay on

// ── Hybrid duty throttle ramps (duty/s) — conservative for ebike ──
#define APPCONF_ADC_HAZ_HYBRID_RAMP_UP_SLOW		0.1f	// Fine control near target
#define APPCONF_ADC_HAZ_HYBRID_RAMP_UP_FAST		0.5f	// Launch response
#define APPCONF_ADC_HAZ_HYBRID_RAMP_DOWN_SLOW	0.2f	// Gentle coast
#define APPCONF_ADC_HAZ_HYBRID_RAMP_DOWN_FAST	1.0f	// Quick stop

// ── PAS Duty — smooth cadence-to-duty pedal assist ──
#define APPCONF_ADC_HAZ_PAS_DUTY_ENABLED		true
#define APPCONF_ADC_HAZ_PAS_DUTY_SMOOTHING		0.0f	// No smoothing — let him tune
#define APPCONF_ADC_HAZ_PAS_DUTY_MEDIAN_FILTER	0.0f	// No median — let him tune
#define APPCONF_ADC_HAZ_PAS_DUTY_IDLE_TIMEOUT	0.3f	// Quick stop on pedal stop
#define APPCONF_ADC_HAZ_PAS_DUTY_ACCEL_GAIN		1.2f	// Responsive to cadence changes
#define APPCONF_ADC_HAZ_PAS_DUTY_LOAD_GAIN		1.2f	// Responsive to load
#define APPCONF_ADC_HAZ_PAS_DUTY_LEAD_PCT		1.05f	// Slight lead over cadence
#define APPCONF_ADC_HAZ_PAS_DUTY_RAMP_UP		0.3f	// Smooth ramp up
#define APPCONF_ADC_HAZ_PAS_DUTY_RAMP_DOWN		0.3f	// Smooth ramp down

// ── PAS sensor (Bafang quadrature, 24 magnets) ──
#define APPCONF_PAS_SENSOR_TYPE			PAS_SENSOR_TYPE_QUADRATURE
#define APPCONF_PAS_MAGNETS				24			// Standard Bafang PAS ring
#define APPCONF_PAS_UPDATE_RATE_HZ		500			// Fast polling
#define APPCONF_PAS_USE_FILTER			true

// ── Bafang display on USART3 (HW_UART_DEV = SD3, GPIOB 10/11) ──
// Luna serial display protocol: 1200 baud, Bafang S-LCD compatible
// PAS levels 0-9 from display buttons, battery SOC, speed, current reported back
// U-D-U-D-U-D combo on display buttons = toggle offroad/street mode
// Boots in street mode — combo unlocks offroad (full power)
#define HW_HAS_LUNA_SERIAL_DISPLAY

// ── PAS sensor pins (quadrature, polled by app_pas.c) ──
// Reuses NRF5x SWD pads — NRF programmer not active at runtime
#define HW_PAS1_PORT		GPIOB
#define HW_PAS1_PIN			12		// SW DIO header
#define HW_PAS2_PORT		GPIOA
#define HW_PAS2_PIN			4		// SW CLK header

#endif /* HW_FLIPSKY_60_MK5_EBIKE_H_ */
