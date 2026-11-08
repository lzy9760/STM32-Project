#include "foc.h"
#include "main.h"
#include "math.h"
#include "encoder.h"
#include "debug.h"


extern TIM_HandleTypeDef htim1;

#define foc_mutipos_turn 8.0f /* Signed mechanical turns from the starting position. */
#define FOC_POS_KP 20.0f       /* Position error (rad) -> speed reference (rad/s). */
#define FOC_POS_KD 0.4f       /* Damping from observed mechanical speed; no error difference. */

typedef struct
{
    float kp, ki;
    float integral;
    float limit;

} foc_nb;

foc_nb pi_id = {0.1f, 0.003f, 0.0f, 20.0f};
foc_nb pi_iq = {0.1f, 0.003f, 0.0f, 20.0f};

foc_nb pi_speed = {0.28f, 0.006f, 0.0f,
                  FOC_SPEED_TRIAL_ENABLE ? FOC_SPEED_TRIAL_IQ_LIMIT_A : 8.0f};

volatile float foc_target_id = 0.0f;
volatile float foc_target_iq = 0.0f;
volatile float foc_iq = 0.0f; 
volatile float foc_dir = 1.0f;
volatile float foc_speed_rad_s = 0.0f;//机械角速度
volatile float foc_speed_raw_rad_s = 0.0f; /* Angle difference / actual time. */
volatile float foc_speed_error_deg = 0.0f; /* Measured minus predicted angle. */
volatile float foc_speed_dt_s = 0.0f; /* Actual observer update interval. */
volatile float foc_target_speed_rad_s = 5.0f;//目标机械角速度
volatile float foc_pos_deg = 0.0f; /* Accumulated signed mechanical angle. */
volatile float foc_pos_turn = 0.0f;
volatile float foc_target_pos_turn = foc_mutipos_turn;
volatile float foc_pos_error_turn = 0.0f;


extern volatile float foc_ia;
extern volatile float foc_ib;
extern volatile float foc_ic;

static float foc_nb_pid(foc_nb *pi, float error);
static void  foc_clarke_park(float ia, float ib, float ic, float theta,
                             float *id_out, float *iq_out);
static void  foc_anti_clarke_park(float ud, float uq, float theta);
static void foc_pwm_write(float ua, float ub, float uc);
volatile float foc_pp = 7.0f;//极对数
volatile float enc_offset_deg = 0.0f;

volatile uint8_t foc_aligned = 0;
extern volatile uint32_t foc_bumb;
static float speed_last_deg, speed_angle, speed_value;
static uint8_t speed_first = 1U;
static uint32_t speed_last_cycles, speed_count;
static float pos_last_deg;
static double pos_total_deg; /* Keep small increments accurate over many turns. */
static uint32_t pos_count;
static uint8_t pos_first = 1U;

#define M_PI 3.14159265358979362646f
#define FOC_SPEED_HZ 60.0f /* Natural frequency; critical damping, no extra LPF. */
#define FOC_U_MAX (FOC_SPEED_TRIAL_ENABLE ? FOC_SPEED_TRIAL_VOLTAGE_LIMIT_V : 13.856f)


volatile float foc_uq = 0.85f;//推力电压
volatile float foc_ud = 0.0f;//推力电压差



void foc_pos_loop_nb(void)
{
    float now, speed_ref, limit;
    double diff;
    uint32_t count, mask;

    if (!foc_aligned || foc_bumb) return;
    if (enc_valid)
    {
        now = enc_deg;
        count = enc_valid_count;
        if (now >= 0.0f && now < 360.0f && (pos_first || count != pos_count))
        {
            if (pos_first)
            {
                pos_total_deg = 0.0;
                pos_first = 0U;
            }
            else
            {
                diff = (double)now - (double)pos_last_deg;
                if (diff > 180.0) diff -= 360.0;
                else if (diff < -180.0) diff += 360.0;
                pos_total_deg += (double)foc_dir * diff;
            }
            pos_last_deg = now;
            pos_count = count;
            foc_pos_deg = (float)pos_total_deg;
            foc_pos_turn = (float)(pos_total_deg / 360.0);
        }
    }
    
    if (pos_first)
    {
        foc_target_speed_rad_s = 0.0f;
        return;
    }

    
    foc_pos_error_turn = foc_target_pos_turn - foc_pos_turn;
    speed_ref = FOC_POS_KP * foc_pos_error_turn * (2.0f * M_PI)
              - FOC_POS_KD * foc_speed_rad_s;
    limit = FOC_SPEED_TRIAL_TARGET_RAD_S;
    if (speed_ref > limit) speed_ref = limit;
    else if (speed_ref < -limit) speed_ref = -limit;

    mask = __get_PRIMASK();
    __disable_irq();
    
    if (foc_aligned && !foc_bumb) foc_target_speed_rad_s = speed_ref;
    else foc_target_speed_rad_s = 0.0f;
    __set_PRIMASK(mask);
}








void foc_curr_loop_nb(void)
{
    if (!foc_aligned || foc_bumb) return;
    if (!ENC_isCalibrationValid())
    {
        stopMotorControl(FOC_ALIGN_ERROR_PARAMETER);
        return;
    }
    float theta = foc_get_elec_angle();//编码器电角度

    float id = 0.0f, iq = 0.0f;

    foc_clarke_park(foc_ia, foc_ib, foc_ic, theta, &id, &iq);
    foc_iq = iq;

    foc_ud  = foc_nb_pid(&pi_id, foc_target_id - id);
    foc_uq = foc_nb_pid(&pi_iq, foc_target_iq - iq);

    //限幅：相电压幅值不超过 Vdc/√3，防止SVPWM占空比越界
    float umag = sqrtf(foc_ud * foc_ud + foc_uq * foc_uq);
    if (umag > FOC_U_MAX)
    {
        float k = FOC_U_MAX / umag;
        foc_ud *= k;
        foc_uq *= k;
#if FOC_SPEED_TRIAL_ENABLE
        /* Track the trial vector limit so the existing per-axis integrators
         * cannot accumulate behind the deliberately reduced voltage ceiling. */
        pi_id.integral = foc_ud - pi_id.kp * (foc_target_id - id);
        pi_iq.integral = foc_uq - pi_iq.kp * (foc_target_iq - iq);
#endif
    }

    foc_anti_clarke_park(foc_ud, foc_uq, theta);//输出SVPWM
    Debug_Capture(id, iq);
    
}

void foc_speed_loop_nb(void)
{
    if (!foc_aligned || foc_bumb) return;
    float error = foc_target_speed_rad_s - foc_speed_rad_s;
    float iq_ref = foc_nb_pid(&pi_speed, error);
    uint32_t mask = __get_PRIMASK();
    __disable_irq();
    /* An ADC/driver stop during PI calculation must keep its zero reference. */
    if (foc_aligned && !foc_bumb) foc_target_iq = iq_ref;
    __set_PRIMASK(mask);
}



void HAL_writePWMData(float ud, float uq, float theta)
{
    foc_anti_clarke_park(ud, uq, theta);
}

float foc_get_elec_angle(void)
{
    float elec_deg = foc_dir * foc_pp * (enc_deg - enc_offset_deg);
    elec_deg = fmodf(elec_deg, 360.0f);
    if (elec_deg < 0.0f) elec_deg += 360.0f;
    return elec_deg * (M_PI / 180.0f);
}

void resetMotorControl(void)
{
    pi_id.integral = pi_iq.integral = pi_speed.integral = 0.0f;
    foc_target_id = foc_target_iq = 0.0f;
    foc_ud = foc_uq = 0.0f;
    foc_speed_rad_s = foc_speed_raw_rad_s = 0.0f;
    foc_speed_error_deg = foc_speed_dt_s = 0.0f;
    speed_last_deg = enc_deg;
    speed_angle = speed_value = 0.0f;
    speed_last_cycles = speed_count = 0U;
    speed_first = 1U;
    foc_pos_deg = foc_pos_turn = foc_pos_error_turn = 0.0f;
    foc_target_pos_turn = foc_mutipos_turn;
    pos_last_deg = 0.0f;
    pos_total_deg = 0.0;
    pos_count = 0U;
    pos_first = 1U;
}

static float foc_nb_pid(foc_nb *pi, float error)
{
    pi->integral += pi->ki * error;

    if (pi->integral >  pi->limit) pi->integral =  pi->limit; 
    if (pi->integral < -pi->limit) pi->integral = -pi->limit;
    
    float out = pi->kp * error + pi->integral;

    if (out >  pi->limit) out =  pi->limit; 
    if (out < -pi->limit) out = -pi->limit;

    return out;
}

static void foc_anti_clarke_park(float ud, float uq, float theta)
{
    float s = sinf(theta), c = cosf(theta);

    float u_alpha = ud * c - uq * s;
    float u_beta = uq * c + ud * s;

    float ua = u_alpha;
    float ub = -0.5f * ua + 0.8660254f * u_beta;
    float uc = -ua - ub;

    foc_pwm_write(ua, ub, uc);
}

static void foc_pwm_write(float ua, float ub, float uc)
{

    //SVPWM：注入零序分量（min-max 法），把三相整体搬移
    float vmax = ua;
    float vmin = ua;
    if (ub > vmax) vmax = ub;
    if (uc > vmax) vmax = uc;
    if (ub < vmin) vmin = ub;
    if (uc < vmin) vmin = uc;
    float v0 = -0.5f * (vmax + vmin);
    ua += v0;
    ub += v0;
    uc += v0;

    uint32_t pwm_a = (uint32_t)((0.5f + ua / 24.0f) * 4200.0f);
    uint32_t pwm_b = (uint32_t)((0.5f + ub / 24.0f) * 4200.0f);
    uint32_t pwm_c = (uint32_t)((0.5f + uc / 24.0f) * 4200.0f);

    if(pwm_a > 4199)
    {
        pwm_a = 4199;
    }
    if(pwm_b > 4199)
    {
        pwm_b = 4199;
    }
    if(pwm_c > 4199)
    {
        pwm_c = 4199;
    }


   __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, pwm_a);
   __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, pwm_b);
   __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, pwm_c);

}

static void foc_clarke_park(float ia, float ib, float ic, float theta, float *id_out, float *iq_out)
{
    float i_alpha = ia;
    float i_beta = (ia + 2.0f * ib) / 1.7320508f;

    float c = cosf(theta);
    float s = sinf(theta);

    *id_out = i_alpha * c + i_beta * s;
    *iq_out = -i_alpha * s + i_beta * c;
}


/* Microchip MCAF angle tracking loop, adapted to fresh SPI samples.
 * Discrete prediction/correction has two poles p = exp(-2*pi*f*dt).
 * Its small-dt limit is angle_dot = speed + 2*w*error,
 * speed_dot = w*w*error. The speed state feeds the existing speed PI.
 * Reference: https://microchiptech.github.io/mcaf-doc/6.0.2/algorithms/qei.html#tracking-loop
 */
void foc_update_speed(void)
{
    uint32_t count, cycles_now, cycles;
    float now, angle, dt, diff, predicted, error, p, gain;

    if (!enc_valid) return;
    count = enc_valid_count;
    if (!speed_first && count == speed_count) return;
    now = enc_deg;
    if (!(now >= 0.0f && now < 360.0f)) return;
    cycles_now = DWT->CYCCNT;
    cycles = cycles_now - speed_last_cycles;
    dt = (float)cycles / (float)SystemCoreClock;
    angle = now * (M_PI / 180.0f);

    /* Seed from the actual angle; restart after a debugger/long sample gap. */
    if (speed_first || dt > 0.020f)
    {
        speed_first = 0U;
        speed_last_deg = now;
        speed_angle = angle;
        speed_value = 0.0f;
        speed_last_cycles = cycles_now;
        speed_count = count;
        foc_speed_rad_s = foc_speed_raw_rad_s = 0.0f;
        foc_speed_error_deg = foc_speed_dt_s = 0.0f;
        return;
    }
    /* Run about every 1 ms in foreground; do not change the speed PI period. */
    if (cycles < SystemCoreClock / 1000U) return;

    diff = now - speed_last_deg;
    if (diff > 180.0f) diff -= 360.0f;
    else if (diff < -180.0f) diff += 360.0f;
    foc_speed_raw_rad_s = foc_dir * diff * (M_PI / 180.0f) / dt;

    predicted = speed_angle + speed_value * dt;
    error = fmodf(angle - predicted + M_PI, 2.0f * M_PI);
    if (error < 0.0f) error += 2.0f * M_PI;
    error -= M_PI;

    p = expf(-2.0f * M_PI * FOC_SPEED_HZ * dt);
    gain = 1.0f - p;
    speed_value += gain * gain * error / dt;
    speed_angle = fmodf(predicted + (1.0f - p * p) * error, 2.0f * M_PI);
    if (speed_angle < 0.0f) speed_angle += 2.0f * M_PI;

    foc_speed_rad_s = foc_dir * speed_value;
    foc_speed_error_deg = error * (180.0f / M_PI);
    foc_speed_dt_s = dt;
    speed_last_deg = now;
    speed_last_cycles = cycles_now;
    speed_count = count;
}
