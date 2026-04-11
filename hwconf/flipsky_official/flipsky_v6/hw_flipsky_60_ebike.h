/*
	Flipsky 6.x MK1 — E-Bike configuration (Kartman's board)
	
	Kartman's wiring:
	  PAS 1        → GPIOA 13 (STM32 SWDIO pin — released from debug at boot)
	  PAS 2        → GPIOA 14 (STM32 SWCLK pin — released from debug at boot)
	  Speed sensor → PPM      (GPIOB 6, ICU/servo input)
	  Temp sensor  → TEMP     (ADC ch 9, real motor temp — NOT repurposed)
	  Throttle     → ADC 1    (ADC ch 6, standard)
	  Bafang display → TX/RX  (USART3, GPIOB 10/11)

	This is the MK1 variant — DRV8301 SPI on PC9/PC10/PC11/PC12,
	NOT on PB3/PB4 like MK5. Using wrong variant = gate driver dead = no motor.

	IMPORTANT: #include "hw_flipsky_60_core.h" must come LAST because core.h
	has #ifdef guards (e.g. HW_HAS_WHEEL_SPEED_SENSOR) that gate function
	declarations. Defines must be visible before the include.
*/

#ifndef HW_FLIPSKY_60_EBIKE_H_
#define HW_FLIPSKY_60_EBIKE_H_

#define HW60_IS_MK1

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
// PAS levels 0-9, battery SOC, speed, current all reported to display
// Up-Down-Up-Down-Up-Down combo on PAS buttons = toggle offroad/street mode
#define HW_HAS_LUNA_SERIAL_DISPLAY

// ── PAS sensor pins (on STM32 SWD/JTAG pads) ──
// GPIOA 13/14 are the MCU debug pins — VESC releases them from JTAG at boot
#define HW_PAS1_PORT		GPIOA
#define HW_PAS1_PIN			13		// SWDIO pad
#define HW_PAS2_PORT		GPIOA
#define HW_PAS2_PIN			14		// SWCLK pad

// ── PAS EXTI on PA13 → EXTI Line13 → EXTI15_10_IRQHandler ──
// PA13 is on EXTI line 13, which routes to the EXTI15_10 vector.
// The encoder uses EXTI9_5 (line 8), so PAS needs its own ISR.
#define HW_PAS_PPM_EXTI_PORTSRC		EXTI_PortSourceGPIOA
#define HW_PAS_PPM_EXTI_PINSRC		EXTI_PinSource13
#define HW_PAS_PPM_EXTI_LINE		EXTI_Line13
#define HW_PAS_PPM_EXTI_CH			EXTI15_10_IRQn
#define HW_PAS_PPM_EXTI_ISR_VEC	EXTI15_10_IRQHandler
#define HW_PAS_SEPARATE_EXTI		// PAS EXTI on different vector than encoder

// ── Wheel speed sensor on PPM pin (digital GPIO polling) ──
// Hall-type sensor output wired to PPM/ICU input (GPIOB 6, TIM4_CH1)
// PPM app not used, so this pin is free for GPIO polling
#define HW_HAS_WHEEL_SPEED_SENSOR
#define HW_WHEEL_SPEED_USE_GPIO
#define HW_WHEEL_SPEED_GPIO_PORT	GPIOB
#define HW_WHEEL_SPEED_GPIO_PIN		6
#define HW_WHEEL_SPEED_MAGNETS		1

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

#endif /* HW_FLIPSKY_60_EBIKE_H_ */
