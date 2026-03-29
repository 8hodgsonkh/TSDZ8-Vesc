/*
	Flipsky 6.7 Pro / Mini MK5 — E-Bike configuration
	
	Kartman's wiring:
	  PAS 1       → SW DIO  (GPIOB 12, shared with NRF5x SWDIO — not used at runtime)
	  PAS 2       → SW CLK  (GPIOA 4,  shared with NRF5x SWCLK — not used at runtime)
	  Speed sensor → PPM     (GPIOB 6,  ICU/servo input — handled by Bafang display)
	  Temp sensor  → TEMP    (ADC ch 9, real motor temp — NOT repurposed)
	  Throttle     → ADC 1   (ADC ch 6, standard)
	  Bafang display → TX/RX (USART3, GPIOB 10/11)

	Based on hw_flipsky_60_mk5.h with PAS pin definitions added
	for quadrature PAS support in app_pas.c / app_adc.c.
*/

#ifndef HW_FLIPSKY_60_MK5_EBIKE_H_
#define HW_FLIPSKY_60_MK5_EBIKE_H_

#define HW60_IS_MK5

#include "hw_flipsky_60_core.h"

// ── Default to APP_ADC on fresh flash ──
#undef APPCONF_APP_TO_USE
#define APPCONF_APP_TO_USE		APP_ADC

// ── Power button: tap = street, hold 5s = offroad ──
// Same logic as GO-FOC S100 (HW_SAMPLE_SHUTDOWN defined in core.h).
// HW_EBIKE_DEFAULT_OFFROAD is fallback for boards without a power button.
#define HW_EBIKE_DEFAULT_OFFROAD

// ── Shutdown mode = toggle (same as GO-FOC S100) ──
// Without this, shutdown thread may auto-off the board unexpectedly.
#undef HAZZA_SHUTDOWN_TOGGLE_GUARD
#define HAZZA_SHUTDOWN_TOGGLE_GUARD	1
#ifndef APPCONF_SHUTDOWN_MODE
#define APPCONF_SHUTDOWN_MODE		SHUTDOWN_MODE_TOGGLE_BUTTON_ONLY
#endif

// ── Bafang display on USART3 (HW_UART_DEV = SD3, GPIOB 10/11) ──
// 1200 baud, Bafang S-LCD protocol. U-D-U-D-U-D combo toggles offroad/street.
#define HW_HAS_LUNA_SERIAL_DISPLAY

// ── PAS sensor pins (quadrature, polled by app_pas.c / app_adc.c) ──
// NRF52 SWD pads — unused at runtime, free for PAS
#define HW_PAS1_PORT		GPIOB
#define HW_PAS1_PIN			12		// NRF52 SWDIO pad
#define HW_PAS2_PORT		GPIOA
#define HW_PAS2_PIN			4		// NRF52 SWCLK pad

// ── Wheel speed sensor on ADC_EXT2 / PA6 (COMM header) ──
// Hall sensor with 10K pullup, reads via ADC with hysteresis
#define HW_HAS_WHEEL_SPEED_SENSOR
#define HW_WHEEL_SPEED_ADC_CH		7		// ADC_IND_EXT2 = 7 (IN6, PA6)
#define HW_WHEEL_SPEED_THRESH_HIGH	2000
#define HW_WHEEL_SPEED_THRESH_LOW	1000
#define HW_WHEEL_SPEED_MAGNETS		1

#endif /* HW_FLIPSKY_60_MK5_EBIKE_H_ */
