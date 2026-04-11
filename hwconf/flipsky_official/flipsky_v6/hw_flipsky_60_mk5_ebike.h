/*
	Flipsky 6.x MK5 — E-Bike configuration (Kartman's board)
	
	Kartman's wiring:
	  PAS 1        → SIN/PPM  (GPIOB 6, TIM4_CH1 — EXTI interrupt for cadence)
	  PAS 2        → ADC2     (GPIOA 6, direction gating — needs ext. pull-up)
	  Wheel speed  → TMP      (GPIOC 4, has 10k pull-up on PCB)
	  Throttle     → ADC1     (GPIOA 5, standard analog)
	  Bafang display → TX/RX  (USART3, GPIOB 10/11)
	  Motor temp   → N/A      (TMP repurposed for wheel speed)

	This is the MK5 variant — DRV8301 SPI on PB3/PB4/PC10/PC9,
	NOT on PC11/PC12 like MK1. Using wrong variant = gate driver dead = no motor.

	IMPORTANT: #include "hw_flipsky_60_core.h" must come LAST because core.h
	has #ifdef guards (e.g. HW_HAS_WHEEL_SPEED_SENSOR) that gate function
	declarations. Defines must be visible before the include.
*/

#ifndef HW_FLIPSKY_60_MK5_EBIKE_H_
#define HW_FLIPSKY_60_MK5_EBIKE_H_

#define HW60_IS_MK5

// ── Default to APP_ADC on fresh flash (core.h defaults to APP_NONE) ──
#undef APPCONF_APP_TO_USE
#define APPCONF_APP_TO_USE		APP_ADC

// ── Motor & app defaults for Giant SyncDrive / Bafang DZ40 ──
#include "mcconf_flipsky_60_ebike.h"
#include "appconf_flipsky_60_ebike.h"

// ── Boot in street mode — unlock offroad via display combo ──
// Up-Down-Up-Down-Up-Down on PAS buttons toggles offroad/street

// ── Bafang display on USART3 (HW_UART_DEV = SD3, GPIOB 10/11) ──
// Uses Luna serial display protocol: 1200 baud, Bafang S-LCD compatible
// PAS levels 0-5 (20% per level), battery SOC, speed, current reported
// Up-Down×3 combo on display buttons = toggle offroad/street mode
#define HW_HAS_LUNA_SERIAL_DISPLAY

// ── PAS sensor pins ──
// PAS1 on SIN/PPM pin (PB6) — clean interrupt-driven cadence detection
// PAS2 on ADC2 pin (PA6) — direction gating (needs external ~4.7k pull-up to 3.3V)
#define HW_PAS1_PORT		GPIOB
#define HW_PAS1_PIN			6		// SIN/PPM connector
#define HW_PAS2_PORT		GPIOA
#define HW_PAS2_PIN			6		// ADC2 on COMM connector

// ── PAS EXTI on PB6 → EXTI Line6 → EXTI9_5_IRQHandler ──
// PB6 is on EXTI line 6, which routes to EXTI9_5 — SAME vector as the
// encoder (line 8). The shared handler in irq_handlers.c checks both lines.
// Do NOT define HW_PAS_SEPARATE_EXTI — they share the vector.
#define HW_PAS_PPM_EXTI_PORTSRC		EXTI_PortSourceGPIOB
#define HW_PAS_PPM_EXTI_PINSRC		EXTI_PinSource6
#define HW_PAS_PPM_EXTI_LINE		EXTI_Line6
#define HW_PAS_PPM_EXTI_CH			EXTI9_5_IRQn
#define HW_PAS_PPM_EXTI_ISR_VEC	EXTI9_5_IRQHandler

// ── Wheel speed sensor on TMP pin (PC4 — has 10k pull-up on PCB) ──
// Motor temp sensing disabled — TMP repurposed for wheel speed
// Hall-type speed sensor, 1 magnet, GPIO polling with debounce
#define HW_HAS_WHEEL_SPEED_SENSOR
#define HW_WHEEL_SPEED_USE_GPIO
#define HW_WHEEL_SPEED_GPIO_PORT	GPIOC
#define HW_WHEEL_SPEED_GPIO_PIN		4
#define HW_WHEEL_SPEED_MAGNETS		1
// TMP pin repurposed for wheel speed — disable motor temp sensing
#undef MCCONF_M_MOTOR_TEMP_SENS_TYPE
#define MCCONF_M_MOTOR_TEMP_SENS_TYPE	TEMP_SENSOR_DISABLED

// ── HW-specific function declarations ──
void hw_update_speed_sensor(void);
float hw_get_speed(void);
float hw_get_distance(void);
float hw_get_distance_abs(void);
void hw_brake_override(float *brake);
float hw_read_motor_temp(float beta);
bool hw_bbshd_has_fixed_throttle_level(void);

// ── Include core LAST so all defines above are visible to core.h ifdefs ──
#include "hw_flipsky_60_core.h"

#endif /* HW_FLIPSKY_60_MK5_EBIKE_H_ */
