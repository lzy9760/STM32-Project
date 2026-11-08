/* Actual foc.c with synthetic absolute encoder samples; no hardware access. */
#include "main.h"
#include "foc.h"
#include <math.h>
#include <stdio.h>

HostTIM host_tim;
HostDWT host_dwt;
TIM_HandleTypeDef htim1;
uint32_t SystemCoreClock = 168000000U;
volatile uint32_t foc_bumb, enc_valid_count;
volatile uint8_t enc_valid;
volatile float enc_deg, foc_ia, foc_ib, foc_ic;
volatile MOTOR_Vars_t motorVars_M1;
static uint32_t tick, trip_on_commit, pos_calls, speed_calls;
static int checks, failures;
static const double pi = 3.14159265358979323846;
#define CHECK(x) do { ++checks; if (!(x)) { if (failures < 20) \
    printf("FAIL %d: %s\n", __LINE__, #x); ++failures; } } while (0)
#define NEAR(a,b,t) CHECK(fabs((double)(a)-(double)(b)) <= (t))
uint32_t HAL_GetTick(void) { return tick; }
void host_disable_irq(void)
{
    if (trip_on_commit) {
        trip_on_commit = 0U;
        foc_bumb = 1U;
        foc_target_id = foc_target_iq = foc_target_speed_rad_s = 0.0f;
    }
}
uint8_t ENC_isCalibrationValid(void) { return 1U; }
void stopMotorControl(MOTOR_AlignFault_e reason) { (void)reason; foc_bumb = 1U; }
void Debug_Capture(float id, float iq) { (void)id; (void)iq; }
#ifdef M_PI
#undef M_PI
#endif
#define foc_pos_loop_nb actual_pos_loop
#define foc_speed_loop_nb actual_speed_loop
#include "../Core/Src/foc.c"
#undef foc_pos_loop_nb
#undef foc_speed_loop_nb
void foc_pos_loop_nb(void) { ++pos_calls; actual_pos_loop(); }
void foc_speed_loop_nb(void)
{
    CHECK(pos_calls == speed_calls+1U);
    ++speed_calls;
    actual_speed_loop();
}
#include "position_scheduler.h"

static float wrapped(double deg)
{
    double v = fmod(deg, 360.0);
    return (float)(v < 0.0 ? v+360.0 : v);
}
static void sample(double deg)
{
    enc_deg = wrapped(deg);
    enc_valid = 1U;
    ++enc_valid_count;
    actual_pos_loop();
}
static void begin(double deg, float direction, float target)
{
    foc_bumb = 0U;
    foc_aligned = 1U;
    foc_dir = direction;
    resetMotorControl();
    NEAR(foc_target_pos_turn, foc_mutipos_turn, 0.0);
    foc_target_pos_turn = target;
    sample(deg);
    NEAR(foc_pos_turn, 0.0, 0.0);
}

static void travel(double initial, double turns, float direction)
{
    begin(initial, direction, (float)(turns*direction));
    for (unsigned i = 1U; i <= 2000U; ++i) {
        double done = turns*(double)i/2000.0;
        sample(initial+done*360.0);
        NEAR(foc_pos_turn, done*direction, 0.000003);
        NEAR(foc_pos_error_turn, turns*direction-done*direction, 0.000004);
        CHECK(fabsf(foc_target_speed_rad_s) <= FOC_SPEED_TRIAL_TARGET_RAD_S);
    }
    NEAR(foc_target_speed_rad_s, 0.0, 0.00005);
}

static void guards(void)
{
    begin(359.99, 1.0f, 1.0f);
    sample(360.01); sample(359.99); sample(360.02);
    NEAR(foc_pos_turn, 0.03/360.0, 0.000001);
    { float held = foc_pos_turn;
      enc_deg = 180.0f; actual_pos_loop();
      NEAR(foc_pos_turn, held, 0.0); /* Same sample count cannot add turns. */
      ++enc_valid_count; enc_valid = 0U; actual_pos_loop();
      NEAR(foc_pos_turn, held, 0.0);
    }
    begin(123.0, 1.0f, 2.0f);
    NEAR(foc_pos_error_turn, 2.0f, 0.0); /* Do not wrap a two-turn target to zero. */
    NEAR(foc_target_speed_rad_s, FOC_SPEED_TRIAL_TARGET_RAD_S, 0.0);
    foc_target_pos_turn = -2.0f; actual_pos_loop();
    NEAR(foc_target_speed_rad_s, -FOC_SPEED_TRIAL_TARGET_RAD_S, 0.0);
    begin(123.0, 1.0f, 0.0f);
    NEAR(foc_target_speed_rad_s, 0.0, 0.0);
    foc_aligned = 0U; foc_target_speed_rad_s = 0.0f;
    foc_target_pos_turn = 3.0f; sample(124.0);
    NEAR(foc_target_speed_rad_s, 0.0, 0.0);
    foc_aligned = 1U; foc_bumb = 1U; sample(125.0);
    NEAR(foc_target_speed_rad_s, 0.0, 0.0);
    begin(123.0, 1.0f, 1.0f);
    trip_on_commit = 1U; actual_pos_loop();
    CHECK(foc_bumb == 1U);
    NEAR(foc_target_speed_rad_s, 0.0, 0.0);
    NEAR(foc_target_iq, 0.0, 0.0);
    begin(123.0, 1.0f, 1.0f);
    resetMotorControl(); enc_valid = 0U; foc_target_speed_rad_s = 20.0f;
    actual_pos_loop();
    NEAR(foc_target_speed_rad_s, 0.0, 0.0); /* No blind startup speed. */
    sample(240.0);
    NEAR(foc_pos_turn, 0.0, 0.0); /* Reset establishes a new relative zero. */
    NEAR(foc_target_pos_turn, foc_mutipos_turn, 0.0);
}

static void kinematic_move(float target, float direction)
{
    double position = 0.0, initial = 6.1;
    begin(initial*180.0/pi, direction, target);
    /* Ideal inner velocity tracking only: this is not a motor/bench model. */
    for (unsigned i = 0U; i < 7000U; ++i) {
        double command = foc_target_speed_rad_s;
        foc_speed_rad_s=(float)command;
        position += command*0.002;
        sample((initial+position*direction)*180.0/pi);
        CHECK(fabs(command) <= FOC_SPEED_TRIAL_TARGET_RAD_S);
    }
    NEAR(foc_pos_turn, target, 0.00001);
    NEAR(foc_target_speed_rad_s, 0.0, 0.001);
}

static void scheduler(void)
{
    begin(200.0, 1.0f, 1.0f);
    pos_calls = speed_calls = tick = 0U;
    motorVars_M1.flagEnableAlignment = 0U;
    foreground_position_service(); CHECK(pos_calls == 0U);
    tick = 1U; foreground_position_service(); CHECK(pos_calls == 0U);
    tick = 2U; foreground_position_service(); CHECK(pos_calls == 1U);
    { double expected=fmin(FOC_POS_KP*2.0*pi, FOC_SPEED_TRIAL_TARGET_RAD_S);
      NEAR(foc_target_speed_rad_s, expected, 0.00001);
      NEAR(foc_target_iq, fmin(expected*(pi_speed.kp+pi_speed.ki),pi_speed.limit), 0.00001); }
    foreground_position_service(); CHECK(pos_calls == 1U);
    tick = 100U; foreground_position_service(); CHECK(pos_calls == 2U);
    foreground_position_service(); CHECK(pos_calls == 2U); /* No catch-up burst. */
    foc_aligned = 0U; tick = 102U; foreground_position_service(); CHECK(pos_calls == 2U);
    foc_aligned = 1U; motorVars_M1.flagEnableAlignment = 1U;
    tick = 104U; foreground_position_service(); CHECK(pos_calls == 2U);
    motorVars_M1.flagEnableAlignment = 0U; foc_bumb = 1U;
    tick = 106U; foreground_position_service(); CHECK(pos_calls == 2U);
    foc_bumb = 0U; tick = 0xFFFFFFFEU; foreground_position_service(); CHECK(pos_calls == 3U);
    tick = 0U; foreground_position_service(); CHECK(pos_calls == 4U); /* Tick wrap. */
}


static void increments(void)
{
    begin(359.0,1.0f,2.0f);
    sample(361.0); NEAR(foc_pos_deg,2.0,0.00001);
    sample(359.0); NEAR(foc_pos_deg,0.0,0.00001);
    begin(1.0,1.0f,2.0f);
    sample(-1.0); NEAR(foc_pos_deg,-2.0,0.00001);
    begin(123.0,1.0f,2.0f);
    for(unsigned i=1;i<=720;++i) {
        sample(123.0+i);
        if(i==360) {
            NEAR(foc_pos_deg,360.0,0.00001);
            NEAR(foc_pos_turn,1.0,0.000001);
            NEAR(foc_pos_error_turn,1.0,0.000001);
            CHECK(foc_target_speed_rad_s>0.0f);
        }
    }
    NEAR(foc_pos_deg,720.0,0.00001);
    NEAR(foc_pos_turn,2.0,0.000001);
    NEAR(foc_target_speed_rad_s,0.0,0.00001);
    for(unsigned i=0;i<100;++i) actual_pos_loop();
    NEAR(foc_pos_deg,720.0,0.00001);
    enc_valid=0U; enc_deg=0.0f; ++enc_valid_count;actual_pos_loop();
    NEAR(foc_pos_deg,720.0,0.00001);
    resetMotorControl(); NEAR(foc_pos_deg,0.0,0.0);
    begin(123.0,-1.0f,1.0f);
    for(unsigned i=1;i<=360;++i) sample(123.0-i);
    NEAR(foc_pos_deg,360.0,0.00001);
    NEAR(foc_pos_turn,1.0,0.000001);
}

int main(void)
{
    increments();
    travel(350.0, 10.0, 1.0f);
    travel(12.0, -5.5, 1.0f);
    travel(350.0, -3.0, -1.0f);
    travel(123.0, 0.25, 1.0f);
    travel(80.0, -0.25, 1.0f);
    guards();
    kinematic_move(1.0f, 1.0f);
    kinematic_move(-2.5f, 1.0f);
    kinematic_move(0.25f, -1.0f);
    scheduler();
    printf("%s: %d position checks, %d failures; synthetic encoder/ideal inner tracking only.\n",
           failures ? "FAIL" : "PASS", checks, failures);
    return failures ? 1 : 0;
}
