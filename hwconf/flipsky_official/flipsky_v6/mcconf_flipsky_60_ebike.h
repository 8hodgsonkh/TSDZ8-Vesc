// Motor config defaults for Kartman's Giant SyncDrive / Bafang DZ40 motor
// Exported from VESC Tool — these override mcconf_default.h via #ifndef guards

#ifndef MCCONF_FLIPSKY_60_EBIKE_H_
#define MCCONF_FLIPSKY_60_EBIKE_H_

// Motor Type: FOC
#define MCCONF_DEFAULT_MOTOR_TYPE		MOTOR_TYPE_FOC

// Sensor Mode: Hall
#define MCCONF_FOC_SENSOR_MODE			FOC_SENSOR_MODE_HALL

// ── Current Limits ──
#define MCCONF_L_CURRENT_MAX			50.0
#define MCCONF_L_CURRENT_MIN			0.0
#define MCCONF_L_IN_CURRENT_MAX			99.0
#define MCCONF_L_IN_CURRENT_MIN			0.0
#define MCCONF_L_MAX_ABS_CURRENT		150.0
#define MCCONF_L_CURRENT_MAX_SCALE		0.0
#define MCCONF_L_CURRENT_MIN_SCALE		1.0

// ── ERPM Limits ──
#define MCCONF_L_RPM_MIN				-100000.0
#define MCCONF_L_RPM_MAX				100000.0
#define MCCONF_L_RPM_START				0.8

// ── Voltage / Battery ──
#define MCCONF_L_MIN_VOLTAGE			8.0
#define MCCONF_L_MAX_VOLTAGE			57.0
#define MCCONF_L_BATTERY_CUT_START		34.0
#define MCCONF_L_BATTERY_CUT_END		30.0
#define MCCONF_SI_BATTERY_CELLS			10
#define MCCONF_SI_BATTERY_AH			12.0

// ── Duty Cycle ──
#define MCCONF_L_MIN_DUTY				0.005
#define MCCONF_L_MAX_DUTY				0.95
#define MCCONF_L_DUTY_START				1.0

// ── Wattage ──
#define MCCONF_L_WATT_MAX				1500000.0
#define MCCONF_L_WATT_MIN				-1500000.0

// ── Temperature ──
#define MCCONF_L_LIM_TEMP_FET_START		85.0
#define MCCONF_L_LIM_TEMP_FET_END		100.0
#define MCCONF_L_LIM_TEMP_MOTOR_START	85.0
#define MCCONF_L_LIM_TEMP_MOTOR_END		100.0
#define MCCONF_L_LIM_TEMP_ACCEL_DEC		0.15

// ── FOC Motor Parameters (from detection) ──
#define MCCONF_FOC_MOTOR_R				0.0356
#define MCCONF_FOC_MOTOR_L				8.969e-05
#define MCCONF_FOC_MOTOR_LD_LQ_DIFF		8.59e-06
#define MCCONF_FOC_MOTOR_FLUX_LINKAGE	0.005073

// ── FOC Current Controller ──
#define MCCONF_FOC_CURRENT_KP			0.0897
#define MCCONF_FOC_CURRENT_KI			35.6
#define MCCONF_FOC_F_ZV					30000.0
#define MCCONF_FOC_DT_US				0.12
#define MCCONF_FOC_CC_DECOUPLING		FOC_CC_DECOUPLING_DISABLED

// ── FOC Observer ──
#define MCCONF_FOC_OBSERVER_GAIN		3.885e+07
#define MCCONF_FOC_OBSERVER_GAIN_SLOW	0.05
#define MCCONF_FOC_OBSERVER_TYPE		3
#define MCCONF_FOC_SAT_COMP				0.0
#define MCCONF_FOC_TEMP_COMP			false
#define MCCONF_FOC_TEMP_COMP_BASE_TEMP	19.6

// ── FOC Hall Table (from detection) ──
#define MCCONF_FOC_HALL_TAB_0			255
#define MCCONF_FOC_HALL_TAB_1			45
#define MCCONF_FOC_HALL_TAB_2			111
#define MCCONF_FOC_HALL_TAB_3			78
#define MCCONF_FOC_HALL_TAB_4			179
#define MCCONF_FOC_HALL_TAB_5			12
#define MCCONF_FOC_HALL_TAB_6			145
#define MCCONF_FOC_HALL_TAB_7			255
#define MCCONF_FOC_HALL_INTERP_ERPM		500

// ── FOC Sensorless Transition ──
#define MCCONF_FOC_SL_ERPM_START		2500.0
#define MCCONF_FOC_SL_ERPM				4000.0
#define MCCONF_FOC_OPENLOOP_RPM			1400.0
#define MCCONF_FOC_OPENLOOP_RPM_LOW		0.0

// ── FOC PLL ──
#define MCCONF_FOC_PLL_KP				2000.0
#define MCCONF_FOC_PLL_KI				30000.0

// ── Phase Filters (MK1 doesn't have them) ──
#define MCCONF_FOC_PHASE_FILTER_ENABLE	false

// ── FOC Sampling ──
#define MCCONF_FOC_SAMPLE_V0_V7			false
#define MCCONF_FOC_CURRENT_FILTER_CONST	0.1

// ── Calibration ──
#define MCCONF_FOC_OFFSETS_CAL_MODE		1

// ── Motor Physical ──
#define MCCONF_SI_MOTOR_POLES			14
#define MCCONF_SI_GEAR_RATIO			1.0
#define MCCONF_SI_WHEEL_DIAMETER		0.65
#define MCCONF_M_NTC_MOTOR_BETA			3380.0
#define MCCONF_M_MOTOR_TEMP_SENS_TYPE	TEMP_SENSOR_NTC_10K_25C
#define MCCONF_M_HALL_EXTRA_SAMPLES		3

// ── Speed PID (for PAS follow mode) ──
#define MCCONF_S_PID_KP					0.004
#define MCCONF_S_PID_KI					0.004
#define MCCONF_S_PID_KD					0.0001
#define MCCONF_S_PID_KD_FILTER			0.2
#define MCCONF_S_PID_MIN_ERPM			900.0
#define MCCONF_S_PID_RAMP_ERPMS_S		25000.0

#endif // MCCONF_FLIPSKY_60_EBIKE_H_
