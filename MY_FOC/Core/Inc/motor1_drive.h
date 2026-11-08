#ifndef MOTOR1_DRIVE_H
#define MOTOR1_DRIVE_H

#include <stdint.h>

/* Enable the selected alignment diagnostic or candidate-offset speed trial.
 * Setting this to 0 always stops before either path. Flash is not erased. */
#ifndef FOC_ENABLE_POWER_ON_MOTION
#define FOC_ENABLE_POWER_ON_MOTION      1U
#endif

/* Explicit user-requested trial of the measured candidate, not a verified
 * Flash calibration. Disable to return to the alignment diagnostic below. */
#ifndef FOC_SPEED_TRIAL_ENABLE
#define FOC_SPEED_TRIAL_ENABLE          1U
#endif
#define FOC_SPEED_TRIAL_OFFSET_DEG      4.72575378f /* Mechanical; electrical offset = 33.08027 deg. */
#define FOC_SPEED_TRIAL_TARGET_RAD_S    50.0f      //目标速度
#define FOC_SPEED_TRIAL_IQ_LIMIT_A      3.0f
#define FOC_SPEED_TRIAL_VOLTAGE_LIMIT_V 2.0f     //电压矢量限幅

/* Alignment path only (FOC_SPEED_TRIAL_ENABLE=0): force a fresh RAM candidate
 * and keep it out of Flash when FOC_ALIGN_DIAGNOSTIC_ONLY=1. */
#ifndef FOC_ALIGN_DIAGNOSTIC_ONLY
#define FOC_ALIGN_DIAGNOSTIC_ONLY        1U
#endif
#define FOC_ALIGN_FORCE_RECALIBRATION    1U
#define FOC_ALIGN_CHECK_POINTS           7U
#define FOC_ALIGN_CHECK_SAMPLES          50U
#define FOC_ALIGN_CHECK_MAX_ERROR_DEG    5.0f
#define FOC_ALIGN_CHECK_MAX_SPREAD_DEG   2.0f

#define USER_MOTOR1_ALIGN_CURRENT_A     3.0f
#define USER_MOTOR1_ENCODER_DIR         1.0f
#define FOC_ALIGN_CURRENT_LIMIT_A       4.0f
#define FOC_ALIGN_VOLTAGE_LIMIT_V       0.8f
#define FOC_ALIGN_KP                    0.05f
#define FOC_ALIGN_KI                    100.0f
#define FOC_ALIGN_RAMP_MS               1000U
#define FOC_ALIGN_HOLD_MS               1000U
#define FOC_ALIGN_RELEASE_MS            300U
#define FOC_ALIGN_TIMEOUT_MS            270000U
#define FOC_ALIGN_ENCODER_STALE_MS      20U
#define FOC_ALIGN_CURRENT_STALE_MS      5U
/* New method/quality field: do not restore results from previous methods. */
#define FOC_ALIGN_CONFIG_ID             0x00040001UL
#define FOC_ALIGN_FLASH_BASE            0x080E0000UL
#define FOC_ALIGN_FLASH_END             0x08100000UL

/* Existing running-control API and fault values are preserved. */
typedef enum
{
    MOTOR_STOP_IDLE = 0,
    MOTOR_ALIGNMENT,
    MOTOR_CL_RUNNING,
    MOTOR_FAULT_STOP
} MOTOR_Status_e;

typedef enum
{
    FOC_ALIGN_ERROR_NONE = 0,
    FOC_ALIGN_ERROR_PARAMETER = 1,
    FOC_ALIGN_ERROR_ENCODER = 2,
    FOC_ALIGN_ERROR_CURRENT = 3,
    FOC_ALIGN_ERROR_TIMEOUT = 4,
    FOC_ALIGN_ERROR_DIRECTION = 5, /* Direction/ratio disagree with installation. */
    FOC_ALIGN_ERROR_FLASH = 7,
    FOC_ALIGN_ERROR_OVERCURRENT = 8,
    FOC_ALIGN_ERROR_OFFSET = 9
} MOTOR_AlignFault_e;

typedef struct { float value[2]; } MATH_Vec2;
typedef struct
{
    MOTOR_Status_e motorState;
    uint8_t flagEnableAlignment;
    float alignCurrent_A;
    uint32_t alignTimeDelay;
    uint32_t stateRunTimeCnt;
    float angleFOC_rad;
    MATH_Vec2 Idq_out_A;
    MATH_Vec2 Idq_in_A;
    MATH_Vec2 Vdq_out_V;
    MOTOR_AlignFault_e faultMtrNow;
} MOTOR_Vars_t;
extern volatile MOTOR_Vars_t motorVars_M1;

/* 0 idle, 1 sync/ratio, 2 offset sweep, 3 completed, 4 saved, 5 loaded, 6 failed,
 * 7 candidate-offset speed trial (valid stays 0; no precision verification).
 * resultant is circular concentration, not an independent accuracy estimate. */
typedef struct
{
    uint32_t state;
    uint32_t valid;
    uint32_t pointsCollected;
    uint32_t sweepDirection; /* 0 increasing, 1 decreasing forced angle. */
    uint32_t pointIndex;
    float encoderDirection;
    float encoderRatio;
    float offsetCandidate_deg; /* Mechanical zero for the existing FOC API. */
    float offsetElec_deg;      /* VESC: subtracted electrical offset. */
    float resultant;
} MOTOR_AlignCalibration_t;
extern volatile MOTOR_AlignCalibration_t motorAlignCalibration_M1;

/* Project diagnostics, not VESC API. All angle errors are electrical degrees.
 * error = encoder - command, wrapped to [-180,180]; positive means leading.
 * encoderTravel is unwrapped relative to the first settled point, so a lost
 * electrical revolution cannot disappear in a modulo-360 comparison. */
typedef struct
{
    float commandElec_deg;
    float encoderMechanical_deg;
    float encoderElec_deg;
    float error_deg;
    float encoderTravel_deg;
    float travelError_deg;
    float peakToPeak_deg;
    float Id_A;
    float Iq_A;
    float voltage_V;
} MOTOR_AlignCheckPoint_t;

typedef struct
{
    uint32_t state; /* 0 not started, 1 moving, 2 settling, 3 sampling, 4 done, 5 aborted. */
    uint32_t valid; /* All seven settled points passed the diagnostic limits. */
    uint32_t pointsCollected;
    uint32_t pointIndex;
    uint32_t samplesCollected;
    float commandElec_deg;
    float encoderMechanical_deg;
    float encoderElec_deg;
    float error_deg;
    float encoderTravel_deg;
    float travelError_deg;
    float maxError_deg;
    float maxTravelError_deg;
    float maxHysteresis_deg;
    float maxPeakToPeak_deg;
    MOTOR_AlignCheckPoint_t points[FOC_ALIGN_CHECK_POINTS];
} MOTOR_AlignCheck_t;
extern volatile MOTOR_AlignCheck_t motorAlignCheck_M1;

void runMotor1Control(void);
void motor1CtrlISR(float ia, float ib, float ic);
void stopMotorControl(MOTOR_AlignFault_e error);
uint8_t ENC_isCalibrationValid(void);
uint8_t ENC_isCalibrationReady(void);
void ENC_saveCalibration(void); /* Foreground only; TIM1 MOE must be off. */
void HAL_writePWMData(float ud, float uq, float theta);
void resetMotorControl(void);
#endif
