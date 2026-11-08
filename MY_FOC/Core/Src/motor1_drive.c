/*
 *  Copyright (C) 2024 Texas Instruments Incorporated
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *    Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 *    Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the
 *    distribution.
 *
 *    Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 *  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 *  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 *  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 *  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 *  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include "foc.h"
#include "main.h"
#include "encoder.h"
#include <math.h>
#include <string.h>

/* Encoder detector adapted from Benjamin Vedder's VESC firmware:
 * Copyright 2016-2022 Benjamin Vedder <benjamin@vedder.se>.
 * mcpwm_foc_encoder_detect(), commit 2392b4b8483adecc81c87ccabcaca968efc50d60.
 * This detector adaptation is GPL-3.0-or-later, without warranty, as its source.
 * See telemetry/vesc_encoder_reference_20261107/COPYING.txt.
 * ChibiOS sleeps are replaced by a nonblocking STM32 foreground state machine.
 * Running FOC, board current PI/limits and the Flash transaction are retained.
 */
#define ALIGN_PI 3.14159265358979323846f
#define ALIGN_TWO_PI (2.0f * ALIGN_PI)
#define ALIGN_VERIFIED 0x56455231UL
#define ALIGN_COMMIT 0x414C4947UL
#define ALIGN_WORDS 8U
#define ALIGN_RECORD_BYTES (ALIGN_WORDS * 4U)
#ifndef FOC_FLASH_READ
#define FOC_FLASH_READ(address) (*(volatile const uint32_t *)(uintptr_t)(address))
#endif

extern volatile uint32_t foc_bumb;
volatile MOTOR_AlignCalibration_t motorAlignCalibration_M1;
volatile MOTOR_AlignCheck_t motorAlignCheck_M1;
volatile MOTOR_Vars_t motorVars_M1 =
{
    MOTOR_STOP_IDLE, FOC_ALIGN_FORCE_RECALIBRATION, USER_MOTOR1_ALIGN_CURRENT_A, FOC_ALIGN_HOLD_MS,
    0U, 0.0f, {{0.0f, 0.0f}}, {{0.0f, 0.0f}}, {{0.0f, 0.0f}}, FOC_ALIGN_ERROR_NONE
};
typedef enum
{
    ENC_ALIGN_RAMP = 0,
    ENC_ALIGN_SYNC,
    ENC_ALIGN_RATIO_MOVE,
    ENC_ALIGN_RATIO_WAIT,
    ENC_ALIGN_SYNC_OFFSET,
    ENC_ALIGN_OFFSET_MOVE,
    ENC_ALIGN_OFFSET_WAIT,
    ENC_ALIGN_CHECK_MOVE,
    ENC_ALIGN_CHECK_WAIT,
    ENC_ALIGN_CHECK_SAMPLE,
    ENC_ALIGN_RELEASE,
    ENC_ALIGN_SAVE,
    ENC_ALIGN_DONE
} ENC_AlignState_e;
static volatile ENC_AlignState_e alignmentState;
static uint32_t start_ms, state_ms, step_ms, encoder_ms, encoder_count;
static volatile uint32_t current_count;
static uint32_t current_seen, current_ms, current_cycles;
static float calibrated_pp, calibrated_dir, calibrated_current, zero_position;
static float integral_d, integral_q;
static uint8_t current_started, calibration_verified;
/* VESC detector angle/sum roles; forced angles are radians. */
static float m_phase_now_override, phase_old, first, override, s_sum, c_sum;
static uint32_t i, it_ofs, sweepDirection;
static float candidate_offset, resultant;
static float check_last_mechanical, check_unwrapped_mechanical, check_first_mechanical;
static float check_mechanical_sum, check_sin_sum, check_cos_sum;
static float check_first_error, check_error_min, check_error_max;
static float check_id_sum, check_iq_sum, check_voltage_sum;

static void begin_angle_check(uint32_t now);
static void update_angle_check(void);
static void collect_angle_check(uint32_t now);

static float wrap_deg(float x)
{
    x = fmodf(x, 360.0f);
    return x < 0.0f ? x + 360.0f : x;
}

static float utils_angle_difference_rad(float a, float b)
{
    float diff = fmodf(a - b, ALIGN_TWO_PI);
    if (diff > ALIGN_PI) diff -= ALIGN_TWO_PI;
    else if (diff < -ALIGN_PI) diff += ALIGN_TWO_PI;
    return diff;
}

static uint8_t parameters_ok(void)
{
    return (USER_MOTOR1_ENCODER_DIR == 1.0f || USER_MOTOR1_ENCODER_DIR == -1.0f) &&
           foc_pp >= 1.0f && foc_pp <= 64.0f && foc_pp == floorf(foc_pp) &&
           motorVars_M1.alignCurrent_A > 0.0f && motorVars_M1.alignCurrent_A < FOC_ALIGN_CURRENT_LIMIT_A &&
           motorVars_M1.alignTimeDelay > 0U && motorVars_M1.alignTimeDelay < FOC_ALIGN_TIMEOUT_MS &&
           FOC_ALIGN_VOLTAGE_LIMIT_V > 0.0f && FOC_ALIGN_VOLTAGE_LIMIT_V <= 13.856f;
}

uint8_t ENC_isCalibrationValid(void)
{
    return foc_pp == calibrated_pp && foc_dir == calibrated_dir &&
           (!FOC_SPEED_TRIAL_ENABLE || enc_offset_deg == candidate_offset);
}

void stopMotorControl(MOTOR_AlignFault_e error)
{
    uint32_t mask = __get_PRIMASK();
    __disable_irq();
    if (motorVars_M1.motorState != MOTOR_FAULT_STOP) motorVars_M1.faultMtrNow = error;
    motorVars_M1.motorState = MOTOR_FAULT_STOP;
    foc_aligned = calibration_verified = 0U;
    motorAlignCalibration_M1.valid = 0U;
    motorAlignCalibration_M1.state = 6U;
    if (motorAlignCheck_M1.state > 0U && motorAlignCheck_M1.state < 4U)
        motorAlignCheck_M1.state = 5U;
    motorAlignCheck_M1.valid = 0U;
    motorVars_M1.Idq_out_A.value[0] = motorVars_M1.Idq_out_A.value[1] = 0.0f;
    Motor_SafetyStop(12U);
    __set_PRIMASK(mask);
}

static void enter_state(ENC_AlignState_e state, uint32_t now)
{
    uint32_t mask = __get_PRIMASK();
    __disable_irq();
    if (!foc_bumb && motorVars_M1.motorState == MOTOR_ALIGNMENT)
    {
        state_ms = step_ms = now;
        motorVars_M1.stateRunTimeCnt = 0U;
        alignmentState = state;
    }
    __set_PRIMASK(mask);
}

static void publish(float id)
{
    float theta = fmodf(m_phase_now_override, ALIGN_TWO_PI);
    uint32_t mask;
    if (theta < 0.0f) theta += ALIGN_TWO_PI;
    mask = __get_PRIMASK();
    __disable_irq();
    if (!foc_bumb && motorVars_M1.motorState == MOTOR_ALIGNMENT)
    {
        motorVars_M1.angleFOC_rad = theta;
        motorVars_M1.Idq_out_A.value[0] = id;
        motorVars_M1.Idq_out_A.value[1] = 0.0f;
    }
    __set_PRIMASK(mask);
}

static uint32_t float_word(float x)
{
    uint32_t w;
    memcpy(&w, &x, sizeof(w));
    return w;
}

static float word_float(uint32_t w)
{
    float x;
    memcpy(&x, &w, sizeof(x));
    return x;
}

static uint32_t record_crc(const uint32_t *words)
{
    uint32_t crc = 0xFFFFFFFFUL;
    for (uint32_t j = 0U; j < 6U; ++j)
    {
        uint32_t w = words[j];
        for (uint32_t bit = 0U; bit < 32U; ++bit)
        {
            uint32_t lsb = (crc ^ w) & 1U;
            crc >>= 1U;
            if (lsb) crc ^= 0xEDB88320UL;
            w >>= 1U;
        }
    }
    return ~crc;
}

static void read_record(uint32_t address, uint32_t *words)
{
    for (uint32_t j = 0U; j < ALIGN_WORDS; ++j) words[j] = FOC_FLASH_READ(address + 4U * j);
}

static uint8_t record_valid(const uint32_t *w)
{
    float pp = word_float(w[1]), dir = word_float(w[2]);
    float offset = word_float(w[3]), concentration = word_float(w[4]);
    return w[0] == FOC_ALIGN_CONFIG_ID && w[5] == ALIGN_VERIFIED && w[7] == ALIGN_COMMIT &&
           w[6] == record_crc(w) && pp >= 1.0f && pp <= 64.0f && pp == floorf(pp) &&
           dir == USER_MOTOR1_ENCODER_DIR && offset >= 0.0f && offset < 360.0f &&
           concentration > 0.000001f && concentration <= 1.0f;
}

static uint8_t load_record(void)
{
    uint32_t w[ALIGN_WORDS], latest[ALIGN_WORDS], mask;
    uint8_t found = 0U;
    for (uint32_t a = FOC_ALIGN_FLASH_BASE; a < FOC_ALIGN_FLASH_END; a += ALIGN_RECORD_BYTES)
    {
        read_record(a, w);
        if (record_valid(w)) { memcpy(latest, w, sizeof(latest)); found = 1U; }
    }
    if (!found || word_float(latest[1]) != foc_pp) return 0U;
    mask = __get_PRIMASK();
    __disable_irq();
    if (foc_bumb) { __set_PRIMASK(mask); return 0U; }
    calibrated_pp = foc_pp;
    calibrated_dir = foc_dir = word_float(latest[2]);
    enc_offset_deg = candidate_offset = word_float(latest[3]);
    resultant = word_float(latest[4]);
    calibration_verified = 1U;
    motorAlignCalibration_M1.encoderDirection = calibrated_dir;
    motorAlignCalibration_M1.encoderRatio = calibrated_pp;
    motorAlignCalibration_M1.offsetCandidate_deg = candidate_offset;
    motorAlignCalibration_M1.offsetElec_deg = wrap_deg(calibrated_dir * calibrated_pp * candidate_offset);
    motorAlignCalibration_M1.resultant = resultant;
    motorAlignCalibration_M1.valid = 1U;
    motorAlignCalibration_M1.state = 5U;
    __set_PRIMASK(mask);
    return 1U;
}

uint8_t ENC_isCalibrationReady(void)
{
    return !FOC_ALIGN_DIAGNOSTIC_ONLY && !foc_bumb && motorVars_M1.motorState == MOTOR_ALIGNMENT &&
           alignmentState == ENC_ALIGN_SAVE && calibration_verified && motorAlignCalibration_M1.valid &&
           motorAlignCheck_M1.state == 4U && motorAlignCheck_M1.valid;
}

void ENC_saveCalibration(void)
{
    uint32_t words[ALIGN_WORDS], verify[ALIGN_WORDS];
    uint32_t address = FOC_ALIGN_FLASH_BASE;
    HAL_StatusTypeDef status;
    if (!ENC_isCalibrationReady() || foc_bumb) return;
    if ((TIM1->BDTR & TIM_BDTR_MOE) || !ENC_isCalibrationValid() ||
        !calibration_verified || !(enc_offset_deg == candidate_offset))
    {
        stopMotorControl(FOC_ALIGN_ERROR_FLASH);
        return;
    }
    for (uint32_t a = FOC_ALIGN_FLASH_BASE; a < FOC_ALIGN_FLASH_END; a += ALIGN_RECORD_BYTES)
    {
        uint8_t empty = 1U;
        read_record(a, verify);
        for (uint32_t i = 0U; i < ALIGN_WORDS; ++i)
            if (verify[i] != 0xFFFFFFFFUL) empty = 0U;
        /* Always append after the last used slot, including torn records. */
        if (!empty) address = a + ALIGN_RECORD_BYTES;
    }
    words[0] = FOC_ALIGN_CONFIG_ID;
    words[1] = float_word(calibrated_pp);
    words[2] = float_word(calibrated_dir);
    words[3] = float_word(enc_offset_deg);
    words[4] = float_word(resultant);
    words[5] = ALIGN_VERIFIED;
    words[6] = record_crc(words);
    words[7] = ALIGN_COMMIT;
    if (!record_valid(words))
    {
        stopMotorControl(FOC_ALIGN_ERROR_FLASH);
        return;
    }
    status = HAL_FLASH_Unlock();
    if (status == HAL_OK)
    {
        __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_EOP | FLASH_FLAG_OPERR | FLASH_FLAG_WRPERR |
                              FLASH_FLAG_PGAERR | FLASH_FLAG_PGPERR | FLASH_FLAG_PGSERR);
        if (address == FOC_ALIGN_FLASH_END)
        {
            FLASH_EraseInitTypeDef erase = {0};
            uint32_t sector_error;
            erase.TypeErase = FLASH_TYPEERASE_SECTORS;
            erase.Sector = FLASH_SECTOR_11;
            erase.NbSectors = 1U;
            erase.VoltageRange = FLASH_VOLTAGE_RANGE_3;
            status = HAL_FLASHEx_Erase(&erase, &sector_error);
            if (status == HAL_OK && sector_error != 0xFFFFFFFFUL) status = HAL_ERROR;
            address = FOC_ALIGN_FLASH_BASE;
        }
        /* Commit word is programmed last; a torn write is never loaded. */
        for (uint32_t i = 0U; i < ALIGN_WORDS && status == HAL_OK; ++i)
        {
            if (foc_bumb) { status = HAL_ERROR; break; }
            status = HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, address + i * 4U, words[i]);
        }
    }
    if (HAL_FLASH_Lock() != HAL_OK) status = HAL_ERROR;
    /* The polling HAL program function does not invalidate the F4 flash cache. */
    FLASH_FlushCaches();
    if (status == HAL_OK)
    {
        read_record(address, verify);
        if (memcmp(words, verify, sizeof(words)) != 0 || !record_valid(verify)) status = HAL_ERROR;
    }
    if (status != HAL_OK || foc_bumb)
    {
        stopMotorControl(FOC_ALIGN_ERROR_FLASH);
        return;
    }
    {
        uint32_t mask = __get_PRIMASK();
        __disable_irq();
        if (!foc_bumb)
        {
            resetMotorControl();
            motorVars_M1.motorState = MOTOR_STOP_IDLE;
            motorAlignCalibration_M1.state = 4U;
            foc_aligned = 0U; /* Saving calibration never starts the speed loop. */
            Motor_SafetyStop(6U);
        }
        __set_PRIMASK(mask);
    }
}

/* Existing alignment PI, phase-current trip and vector anti-windup retained. */
void motor1CtrlISR(float ia, float ib, float ic)
{
    uint32_t now, cycles;
    float dt, theta, id_ref, id, iq, ed, eq, ud, uq, magnitude, scale;
    float s, c, beta, next_d, next_q;
    if (foc_bumb || foc_aligned || motorVars_M1.motorState != MOTOR_ALIGNMENT ||
        alignmentState == ENC_ALIGN_SAVE) return;
    if (!(fabsf(ia) <= FOC_ALIGN_CURRENT_LIMIT_A) || !(fabsf(ib) <= FOC_ALIGN_CURRENT_LIMIT_A) ||
        !(fabsf(ic) <= FOC_ALIGN_CURRENT_LIMIT_A))
    { stopMotorControl(FOC_ALIGN_ERROR_OVERCURRENT); return; }
    now = DWT->CYCCNT;
    cycles = now - current_cycles;
    current_cycles = now;
    if (!current_started) { current_started = 1U; cycles = 0U; }
    dt = (float)cycles / (float)SystemCoreClock;
    if (dt > 0.0002f) dt = 0.0002f;
    theta = motorVars_M1.angleFOC_rad;
    id_ref = motorVars_M1.Idq_out_A.value[0];
    if (!(id_ref >= 0.0f && id_ref < FOC_ALIGN_CURRENT_LIMIT_A) ||
        !(theta >= 0.0f && theta <= ALIGN_TWO_PI))
    { stopMotorControl(FOC_ALIGN_ERROR_PARAMETER); return; }
    s = sinf(theta);
    c = cosf(theta);
    beta = (ia + 2.0f * ib) / 1.7320508f;
    id = ia * c + beta * s;
    iq = -ia * s + beta * c;
    motorVars_M1.Idq_in_A.value[0] = id;
    motorVars_M1.Idq_in_A.value[1] = iq;
    ed = id_ref - id;
    eq = -iq;
    next_d = integral_d + FOC_ALIGN_KI * ed * dt;
    next_q = integral_q + FOC_ALIGN_KI * eq * dt;
    ud = FOC_ALIGN_KP * ed + next_d;
    uq = FOC_ALIGN_KP * eq + next_q;
    magnitude = sqrtf(ud * ud + uq * uq);
    if (magnitude > FOC_ALIGN_VOLTAGE_LIMIT_V)
    {
        scale = FOC_ALIGN_VOLTAGE_LIMIT_V / magnitude;
        ud *= scale;
        uq *= scale;
    }
    integral_d = ud - FOC_ALIGN_KP * ed;
    integral_q = uq - FOC_ALIGN_KP * eq;
    motorVars_M1.Vdq_out_V.value[0] = ud;
    motorVars_M1.Vdq_out_V.value[1] = uq;
    HAL_writePWMData(ud, uq, theta);
    ++current_count;
}

static uint8_t move_phase(uint32_t now, float target, float step, uint32_t interval_ms)
{
    float diff;
    if (now - step_ms < interval_ms) return 0U;
    step_ms = now;
    diff = target - m_phase_now_override;
    if (fabsf(diff) <= step) m_phase_now_override = target;
    else m_phase_now_override += diff > 0.0f ? step : -step;
    publish(calibrated_current);
    return m_phase_now_override == target;
}

static void begin_ratio_move(uint32_t now)
{
    phase_old = enc_deg * (ALIGN_PI / 180.0f);
    override = m_phase_now_override + (sweepDirection ? -1.0f : 1.0f) * (ALIGN_TWO_PI / 3.0f);
    motorAlignCalibration_M1.sweepDirection = sweepDirection;
    motorAlignCalibration_M1.pointIndex = i;
    enter_state(ENC_ALIGN_RATIO_MOVE, now);
}

static void complete_offset(uint32_t now)
{
    float offset, electrical_zero, mechanical_zero, concentration;
    uint32_t mask;
    if (motorAlignCalibration_M1.pointsCollected != 2U * it_ofs)
    { stopMotorControl(FOC_ALIGN_ERROR_OFFSET); return; }
    concentration = sqrtf(s_sum * s_sum + c_sum * c_sum) / (float)(2U * it_ofs);
    if (!(concentration > 0.000001f && concentration <= 1.00001f))
    { stopMotorControl(FOC_ALIGN_ERROR_OFFSET); return; }
    if (concentration > 1.0f) concentration = 1.0f;
    offset = atan2f(s_sum, c_sum);
    electrical_zero = wrap_deg(offset * (180.0f / ALIGN_PI));
    /* Keep the mechanical-offset API; pick an equivalent zero near startup. */
    mechanical_zero = wrap_deg(zero_position + utils_angle_difference_rad(offset,
        calibrated_dir * calibrated_pp * zero_position * (ALIGN_PI / 180.0f)) *
        (180.0f / ALIGN_PI) / (calibrated_dir * calibrated_pp));
    if (!(mechanical_zero >= 0.0f && mechanical_zero < 360.0f))
    { stopMotorControl(FOC_ALIGN_ERROR_OFFSET); return; }
    mask = __get_PRIMASK();
    __disable_irq();
    if (!foc_bumb && motorVars_M1.motorState == MOTOR_ALIGNMENT)
    {
        enc_offset_deg = candidate_offset = mechanical_zero;
        resultant = concentration;
        calibration_verified = 0U; /* A candidate still needs the separate angle check. */
        motorAlignCalibration_M1.offsetCandidate_deg = mechanical_zero;
        motorAlignCalibration_M1.offsetElec_deg = electrical_zero;
        motorAlignCalibration_M1.resultant = concentration;
        motorAlignCalibration_M1.valid = 1U;
        motorAlignCalibration_M1.state = 3U;
        begin_angle_check(now);
    }
    __set_PRIMASK(mask);
}

static float check_target_deg(uint32_t index)
{
    return (float)(index <= 3U ? index : 6U - index) * 120.0f;
}

static void begin_angle_check(uint32_t now)
{
    memset((void *)&motorAlignCheck_M1, 0, sizeof(motorAlignCheck_M1));
    check_last_mechanical = check_unwrapped_mechanical = enc_deg;
    check_first_mechanical = enc_deg;
    /* Avoid reversing multiple turns of the sweep's unwrapped accumulator. */
    m_phase_now_override = motorVars_M1.angleFOC_rad;
    motorAlignCheck_M1.state = 1U;
    enter_state(ENC_ALIGN_CHECK_MOVE, now);
    update_angle_check();
}

static void update_angle_check(void)
{
    float measured, command;
    if (motorAlignCheck_M1.state == 0U || motorAlignCheck_M1.state >= 4U) return;
    check_unwrapped_mechanical += utils_angle_difference_rad(enc_deg * (ALIGN_PI / 180.0f),
        check_last_mechanical * (ALIGN_PI / 180.0f)) * (180.0f / ALIGN_PI);
    check_last_mechanical = enc_deg;
    measured = wrap_deg(calibrated_dir * calibrated_pp * (enc_deg - candidate_offset));
    command = motorVars_M1.angleFOC_rad * (180.0f / ALIGN_PI);
    motorAlignCheck_M1.commandElec_deg = command;
    motorAlignCheck_M1.encoderMechanical_deg = enc_deg;
    motorAlignCheck_M1.encoderElec_deg = measured;
    motorAlignCheck_M1.error_deg = utils_angle_difference_rad(measured * (ALIGN_PI / 180.0f),
        command * (ALIGN_PI / 180.0f)) * (180.0f / ALIGN_PI);
    motorAlignCheck_M1.encoderTravel_deg = calibrated_dir * calibrated_pp *
        (check_unwrapped_mechanical - check_first_mechanical);
    motorAlignCheck_M1.travelError_deg = motorAlignCheck_M1.encoderTravel_deg -
        m_phase_now_override * (180.0f / ALIGN_PI);
}

static void collect_angle_check(uint32_t now)
{
    uint32_t index = motorAlignCheck_M1.pointIndex;
    uint32_t count = motorAlignCheck_M1.samplesCollected;
    uint32_t mask;
    float error = motorAlignCheck_M1.error_deg;
    float mean_error, excursion, target = check_target_deg(index);
    volatile MOTOR_AlignCheckPoint_t *point = &motorAlignCheck_M1.points[index];
    if (now - step_ms < 1U) return; /* One fresh encoder sample per millisecond. */
    step_ms = now;
    if (count == 0U)
    {
        check_mechanical_sum = check_sin_sum = check_cos_sum = 0.0f;
        check_id_sum = check_iq_sum = check_voltage_sum = 0.0f;
        check_first_error = error;
        check_error_min = check_error_max = 0.0f;
    }
    excursion = utils_angle_difference_rad(error * (ALIGN_PI / 180.0f),
        check_first_error * (ALIGN_PI / 180.0f)) * (180.0f / ALIGN_PI);
    check_error_min = fminf(check_error_min, excursion);
    check_error_max = fmaxf(check_error_max, excursion);
    check_mechanical_sum += check_unwrapped_mechanical;
    check_sin_sum += sinf(error * (ALIGN_PI / 180.0f));
    check_cos_sum += cosf(error * (ALIGN_PI / 180.0f));
    check_id_sum += motorVars_M1.Idq_in_A.value[0];
    check_iq_sum += motorVars_M1.Idq_in_A.value[1];
    check_voltage_sum += hypotf(motorVars_M1.Vdq_out_V.value[0], motorVars_M1.Vdq_out_V.value[1]);
    motorAlignCheck_M1.samplesCollected = ++count;
    if (count < FOC_ALIGN_CHECK_SAMPLES) return;
    mean_error = atan2f(check_sin_sum, check_cos_sum) * (180.0f / ALIGN_PI);
    mask = __get_PRIMASK();
    __disable_irq();
    if (foc_bumb || motorVars_M1.motorState != MOTOR_ALIGNMENT)
    {
        __set_PRIMASK(mask);
        return; /* An ISR stop must not be overwritten by a completed result. */
    }
    if (index == 0U) check_first_mechanical = check_mechanical_sum / (float)count;
    point->commandElec_deg = target; /* Retain 360 as a full-turn command. */
    point->encoderMechanical_deg = wrap_deg(check_mechanical_sum / (float)count);
    point->encoderElec_deg = wrap_deg(target + mean_error);
    point->error_deg = mean_error;
    point->encoderTravel_deg = calibrated_dir * calibrated_pp *
        (check_mechanical_sum / (float)count - check_first_mechanical);
    point->travelError_deg = point->encoderTravel_deg - target;
    point->peakToPeak_deg = check_error_max - check_error_min;
    point->Id_A = check_id_sum / (float)count;
    point->Iq_A = check_iq_sum / (float)count;
    point->voltage_V = check_voltage_sum / (float)count;
    motorAlignCheck_M1.maxError_deg = fmaxf(motorAlignCheck_M1.maxError_deg, fabsf(mean_error));
    motorAlignCheck_M1.maxTravelError_deg = fmaxf(motorAlignCheck_M1.maxTravelError_deg, fabsf(point->travelError_deg));
    motorAlignCheck_M1.maxPeakToPeak_deg = fmaxf(motorAlignCheck_M1.maxPeakToPeak_deg, point->peakToPeak_deg);
    if (index > 3U)
    {
        float opposite = motorAlignCheck_M1.points[6U - index].error_deg;
        float hysteresis = fabsf(utils_angle_difference_rad(mean_error * (ALIGN_PI / 180.0f),
            opposite * (ALIGN_PI / 180.0f)) * (180.0f / ALIGN_PI));
        motorAlignCheck_M1.maxHysteresis_deg = fmaxf(motorAlignCheck_M1.maxHysteresis_deg, hysteresis);
    }
    motorAlignCheck_M1.pointsCollected = index + 1U;
    if (index + 1U < FOC_ALIGN_CHECK_POINTS)
    {
        motorAlignCheck_M1.pointIndex = index + 1U;
        motorAlignCheck_M1.samplesCollected = 0U;
        motorAlignCheck_M1.state = 1U;
        enter_state(ENC_ALIGN_CHECK_MOVE, now);
    }
    else
    {
        motorAlignCheck_M1.valid =
            motorAlignCheck_M1.maxError_deg <= FOC_ALIGN_CHECK_MAX_ERROR_DEG &&
            motorAlignCheck_M1.maxTravelError_deg <= FOC_ALIGN_CHECK_MAX_ERROR_DEG &&
            motorAlignCheck_M1.maxHysteresis_deg <= FOC_ALIGN_CHECK_MAX_ERROR_DEG &&
            motorAlignCheck_M1.maxPeakToPeak_deg <= FOC_ALIGN_CHECK_MAX_SPREAD_DEG;
        motorAlignCheck_M1.state = 4U;
        calibration_verified = (uint8_t)motorAlignCheck_M1.valid;
        enter_state(ENC_ALIGN_RELEASE, now);
    }
    __set_PRIMASK(mask);
}

/* Nonblocking mcpwm_foc_encoder_detect adaptation, not its blocking SDK ABI.
 * No per-point error gate, current-vector correction, or run-out stage.
 * Direction/ratio follow the source and must match the installed configuration.
 */
static void mcpwm_foc_encoder_detect(uint32_t now)
{
    uint32_t elapsed = now - state_ms;
    float progress, diff, step, ratio, dir;
    switch (alignmentState)
    {
    case ENC_ALIGN_RAMP:
        progress = fminf((float)elapsed / (float)FOC_ALIGN_RAMP_MS, 1.0f);
        publish(calibrated_current * progress);
        if (elapsed >= FOC_ALIGN_RAMP_MS) enter_state(ENC_ALIGN_SYNC, now);
        break;
    case ENC_ALIGN_SYNC:
        if (move_phase(now, ALIGN_TWO_PI, ALIGN_TWO_PI / 500.0f, 1U))
            enter_state(ENC_ALIGN_RATIO_WAIT, now);
        break;
    case ENC_ALIGN_RATIO_WAIT:
        if (i == 0U && sweepDirection == 0U)
        {
            if (elapsed < motorVars_M1.alignTimeDelay) break;
            m_phase_now_override = fmodf(m_phase_now_override, ALIGN_TWO_PI);
            first = enc_deg * (ALIGN_PI / 180.0f);
            begin_ratio_move(now);
            break;
        }
        if (elapsed < 300U) break;
        diff = sweepDirection ? utils_angle_difference_rad(phase_old, enc_deg * (ALIGN_PI / 180.0f)) :
                                utils_angle_difference_rad(enc_deg * (ALIGN_PI / 180.0f), phase_old);
        s_sum += sinf(diff);
        c_sum += cosf(diff);
        if (i >= 30U || (i >= 5U && fabsf(utils_angle_difference_rad(
            enc_deg * (ALIGN_PI / 180.0f), first)) < fabsf(diff * 0.5f)))
        {
            if (!sweepDirection)
            {
                sweepDirection = 1U;
                i = 0U;
                first = enc_deg * (ALIGN_PI / 180.0f);
                begin_ratio_move(now);
            }
            else
            {
                diff = atan2f(s_sum, c_sum);
                dir = diff < 0.0f ? -1.0f : 1.0f;
                ratio = fabsf(diff) > 0.000001f ? roundf((ALIGN_TWO_PI / 3.0f) / fabsf(diff)) : 0.0f;
                motorAlignCalibration_M1.encoderDirection = dir;
                motorAlignCalibration_M1.encoderRatio = ratio;
                if (!(dir == calibrated_dir && ratio == calibrated_pp))
                { stopMotorControl(FOC_ALIGN_ERROR_DIRECTION); return; }
                s_sum = c_sum = 0.0f;
                it_ofs = (uint32_t)(calibrated_pp * 3.0f);
                i = sweepDirection = 0U;
                motorAlignCalibration_M1.state = 2U;
                enter_state(ENC_ALIGN_SYNC_OFFSET, now);
            }
        }
        else begin_ratio_move(now);
        break;
    case ENC_ALIGN_RATIO_MOVE:
        if (move_phase(now, override, ALIGN_TWO_PI / 500.0f, 1U))
        {
            m_phase_now_override = fmodf(m_phase_now_override, ALIGN_TWO_PI);
            if (m_phase_now_override < 0.0f) m_phase_now_override += ALIGN_TWO_PI;
            ++i;
            enter_state(ENC_ALIGN_RATIO_WAIT, now);
        }
        break;
    case ENC_ALIGN_SYNC_OFFSET:
        if (move_phase(now, ALIGN_TWO_PI, ALIGN_TWO_PI / 500.0f, 2U))
        {
            motorAlignCalibration_M1.sweepDirection = motorAlignCalibration_M1.pointIndex = 0U;
            enter_state(ENC_ALIGN_OFFSET_MOVE, now);
        }
        break;
    case ENC_ALIGN_OFFSET_MOVE:
        step = (ALIGN_TWO_PI * calibrated_pp) / (float)it_ofs;
        if (move_phase(now, (float)i * step, step / 100.0f, 4U))
            enter_state(ENC_ALIGN_OFFSET_WAIT, now);
        break;
    case ENC_ALIGN_OFFSET_WAIT:
        if (elapsed < 100U) break;
        diff = utils_angle_difference_rad(calibrated_dir * calibrated_pp * enc_deg * (ALIGN_PI / 180.0f),
                                          m_phase_now_override);
        s_sum += sinf(diff);
        c_sum += cosf(diff);
        ++motorAlignCalibration_M1.pointsCollected;
        if (!sweepDirection)
        {
            ++i;
            if (i == it_ofs) sweepDirection = 1U;
        }
        else if (--i == 0U) { complete_offset(now); break; }
        motorAlignCalibration_M1.sweepDirection = sweepDirection;
        motorAlignCalibration_M1.pointIndex = i;
        enter_state(ENC_ALIGN_OFFSET_MOVE, now);
        break;
    case ENC_ALIGN_CHECK_MOVE:
        if (move_phase(now, check_target_deg(motorAlignCheck_M1.pointIndex) * (ALIGN_PI / 180.0f),
                       (ALIGN_TWO_PI / 3.0f) / 100.0f, 4U))
        {
            motorAlignCheck_M1.state = 2U;
            enter_state(ENC_ALIGN_CHECK_WAIT, now);
        }
        break;
    case ENC_ALIGN_CHECK_WAIT:
        if (elapsed >= motorVars_M1.alignTimeDelay)
        {
            motorAlignCheck_M1.samplesCollected = 0U;
            motorAlignCheck_M1.state = 3U;
            enter_state(ENC_ALIGN_CHECK_SAMPLE, now);
        }
        break;
    case ENC_ALIGN_CHECK_SAMPLE:
        collect_angle_check(now);
        break;
    case ENC_ALIGN_RELEASE:
        progress = fminf((float)elapsed / (float)FOC_ALIGN_RELEASE_MS, 1.0f);
        publish(calibrated_current * (1.0f - progress));
        if (elapsed >= FOC_ALIGN_RELEASE_MS)
        {
            uint32_t mask = __get_PRIMASK();
            __disable_irq();
            if (!foc_bumb && motorVars_M1.motorState == MOTOR_ALIGNMENT)
            {
                HAL_writePWMData(0.0f, 0.0f, 0.0f);
                integral_d = integral_q = 0.0f;
                motorVars_M1.Vdq_out_V.value[0] = motorVars_M1.Vdq_out_V.value[1] = 0.0f;
                if (FOC_ALIGN_DIAGNOSTIC_ONLY || !motorAlignCheck_M1.valid)
                {
                    /* Retain all measured points in RAM. Never hand off to speed control. */
                    alignmentState = ENC_ALIGN_DONE;
                    motorVars_M1.motorState = MOTOR_STOP_IDLE;
                    foc_aligned = 0U;
                    Motor_SafetyStop(6U);
                }
                else alignmentState = ENC_ALIGN_SAVE;
            }
            __set_PRIMASK(mask);
        }
        break;
    default:
        stopMotorControl(FOC_ALIGN_ERROR_PARAMETER);
        break;
    }
}

void runMotor1Control(void)
{
    uint32_t now = HAL_GetTick(), mask;
    uint8_t force, fresh = 0U;
    if (foc_bumb || motorVars_M1.motorState == MOTOR_FAULT_STOP) return;
    if (!FOC_ENABLE_POWER_ON_MOTION)
    {
        /* Use the existing PWM/driver stop; never start a sweep or load-to-run.
         * Reason 6 is the existing requested-stop code, not an alignment fault. */
        Motor_SafetyStop(6U);
        return;
    }
    force = motorVars_M1.flagEnableAlignment;
    if (force || motorVars_M1.motorState == MOTOR_STOP_IDLE)
    {
        if (!parameters_ok()) { stopMotorControl(FOC_ALIGN_ERROR_PARAMETER); return; }
        if (!enc_valid || !(enc_deg >= 0.0f && enc_deg < 360.0f))
        { stopMotorControl(FOC_ALIGN_ERROR_ENCODER); return; }
        mask = __get_PRIMASK();
        __disable_irq();
        if (foc_bumb) { __set_PRIMASK(mask); return; }
        foc_aligned = calibration_verified = 0U;
        motorVars_M1.flagEnableAlignment = 0U;
        motorVars_M1.faultMtrNow = FOC_ALIGN_ERROR_NONE;
        resetMotorControl();
        HAL_writePWMData(0.0f, 0.0f, 0.0f);
        integral_d = integral_q = 0.0f;
        current_started = 0U;
        foc_dir = calibrated_dir = USER_MOTOR1_ENCODER_DIR;
        calibrated_pp = foc_pp;
        calibrated_current = motorVars_M1.alignCurrent_A;
        memset((void *)&motorAlignCalibration_M1, 0, sizeof(motorAlignCalibration_M1));
        memset((void *)&motorAlignCheck_M1, 0, sizeof(motorAlignCheck_M1));
        __set_PRIMASK(mask);
        if (FOC_SPEED_TRIAL_ENABLE)
        {
            /* This candidate was measured at 7 pole pairs and direction +1.
             * Never silently accept a changed installation or call it verified. */
            if (!(calibrated_pp == 7.0f && calibrated_dir == 1.0f &&
                  FOC_SPEED_TRIAL_OFFSET_DEG >= 0.0f && FOC_SPEED_TRIAL_OFFSET_DEG < 360.0f &&
                  FOC_SPEED_TRIAL_TARGET_RAD_S > 0.0f &&
                  FOC_SPEED_TRIAL_IQ_LIMIT_A > 0.0f && FOC_SPEED_TRIAL_IQ_LIMIT_A <= 3.0f &&
                  FOC_SPEED_TRIAL_VOLTAGE_LIMIT_V > 0.0f && FOC_SPEED_TRIAL_VOLTAGE_LIMIT_V <= 2.0f))
            { stopMotorControl(FOC_ALIGN_ERROR_PARAMETER); return; }
            mask = __get_PRIMASK();
            __disable_irq();
            if (!foc_bumb)
            {
                enc_offset_deg = candidate_offset = FOC_SPEED_TRIAL_OFFSET_DEG;



                foc_target_id = 0.0f;
                foc_target_iq = 0.0f; /* The speed PI supplies Iq after startup. */



                foc_target_speed_rad_s = FOC_SPEED_TRIAL_TARGET_RAD_S;
                motorAlignCalibration_M1.encoderDirection = calibrated_dir;
                motorAlignCalibration_M1.encoderRatio = calibrated_pp;
                motorAlignCalibration_M1.offsetCandidate_deg = candidate_offset;
                motorAlignCalibration_M1.offsetElec_deg = wrap_deg(calibrated_dir * calibrated_pp * candidate_offset);
                motorAlignCalibration_M1.state = 7U;
                alignmentState = ENC_ALIGN_DONE;
                motorVars_M1.motorState = MOTOR_CL_RUNNING;
                foc_aligned = 1U; /* Runtime angle selection, not a precision certificate. */
            }
            __set_PRIMASK(mask);
            return; /* No scan, Flash restore, save, or seven-point check in this trial. */
        }
        if (!force && load_record())
        {
            mask = __get_PRIMASK();
            __disable_irq();
            if (!foc_bumb)
            {
                motorVars_M1.motorState = MOTOR_STOP_IDLE;
                foc_aligned = 0U;
                Motor_SafetyStop(6U); /* A restored offset never requests motor motion. */
            }
            __set_PRIMASK(mask);
            return;
        }
        if (foc_bumb) return;
        m_phase_now_override = s_sum = c_sum = resultant = 0.0f;
        i = it_ofs = sweepDirection = 0U;
        zero_position = enc_deg;
        encoder_count = enc_valid_count;
        start_ms = encoder_ms = current_ms = now;
        current_seen = current_count;
        motorVars_M1.motorState = MOTOR_ALIGNMENT;
        motorAlignCalibration_M1.state = 1U;
        publish(0.0f);
        enter_state(ENC_ALIGN_RAMP, now);
    }
    if (foc_aligned || alignmentState == ENC_ALIGN_SAVE) return;
    if (!parameters_ok() || !ENC_isCalibrationValid() || motorVars_M1.alignCurrent_A != calibrated_current)
    { stopMotorControl(FOC_ALIGN_ERROR_PARAMETER); return; }
    if (now - start_ms >= FOC_ALIGN_TIMEOUT_MS) { stopMotorControl(FOC_ALIGN_ERROR_TIMEOUT); return; }
    if (current_seen != current_count) { current_seen = current_count; current_ms = now; }
    if (now - current_ms >= FOC_ALIGN_CURRENT_STALE_MS) { stopMotorControl(FOC_ALIGN_ERROR_CURRENT); return; }
    if (enc_valid && encoder_count != enc_valid_count)
    {
        if (!(enc_deg >= 0.0f && enc_deg < 360.0f)) { stopMotorControl(FOC_ALIGN_ERROR_ENCODER); return; }
        encoder_count = enc_valid_count;
        encoder_ms = now;
        fresh = 1U;
    }
    if (now - encoder_ms >= FOC_ALIGN_ENCODER_STALE_MS) { stopMotorControl(FOC_ALIGN_ERROR_ENCODER); return; }
    if (!fresh) return;
    update_angle_check();
    motorVars_M1.stateRunTimeCnt = now - state_ms;
    mcpwm_foc_encoder_detect(now);
    update_angle_check();
}
