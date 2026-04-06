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

// ── Default to APP_ADC on fresh flash (core.h defaults to APP_NONE) ──
#undef APPCONF_APP_TO_USE
#define APPCONF_APP_TO_USE		APP_ADC

// ── No power button — boot straight into offroad (full power) mode ──
#define HW_EBIKE_DEFAULT_OFFROAD

// ── Bafang display on USART3 (HW_UART_DEV = SD3, GPIOB 10/11) ──
// Uses Luna serial display protocol: 1200 baud, Bafang S-LCD compatible
// PAS levels 0-9, battery SOC, speed, current all reported to display
// Triple-tap walk mode button = toggle offroad/street mode
#define HW_HAS_LUNA_SERIAL_DISPLAY

// ── PAS sensor pins (quadrature, polled by app_pas.c) ──
// Reuses NRF5x SWD pads — NRF programmer not active at runtime
#define HW_PAS1_PORT		GPIOB
#define HW_PAS1_PIN			12		// SW DIO header
#define HW_PAS2_PORT		GPIOA
#define HW_PAS2_PIN			4		// SW CLK header

// ── Wheel speed sensor on PPM pin (digital GPIO polling) ──
// Hall-type sensor output wired to PPM/ICU input (GPIOB 6, TIM4_CH1)
// PPM app not used, so this pin is free for GPIO polling
#define HW_HAS_WHEEL_SPEED_SENSOR
#define HW_WHEEL_SPEED_USE_GPIO
#define HW_WHEEL_SPEED_GPIO_PORT	GPIOB
#define HW_WHEEL_SPEED_GPIO_PIN		6
#define HW_WHEEL_SPEED_MAGNETS		1

#endif /* HW_FLIPSKY_60_MK5_EBIKE_H_ */
