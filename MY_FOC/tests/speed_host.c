/* Exercise the actual firmware estimator using synthetic encoder trajectories. */
#include "main.h"
#include "foc.h"
#include <math.h>
#include <stdio.h>

HostTIM host_tim;
HostDWT host_dwt;
TIM_HandleTypeDef htim1;
uint32_t SystemCoreClock = 168000000U;
volatile uint32_t foc_bumb;
volatile uint32_t enc_valid_count;
volatile uint8_t enc_valid;
volatile float enc_deg, foc_ia, foc_ib, foc_ic;
uint8_t ENC_isCalibrationValid(void) { return 1U; }
void stopMotorControl(MOTOR_AlignFault_e reason) { (void)reason; foc_bumb = 1U; }
void Debug_Capture(float id, float iq) { (void)id; (void)iq; }
#ifdef M_PI
#undef M_PI
#endif
#include "../Core/Src/foc.c"

static const double pi = 3.14159265358979323846;
static unsigned checks, failures;
#define CHECK(x) do { ++checks; if (!(x)) { ++failures; \
    printf("FAIL line %d: %s\n", __LINE__, #x); } } while (0)
#define NEAR(a,b,tol) CHECK(fabs((double)(a)-(double)(b)) <= (tol))

static void sample(double angle, double dt)
{
    double deg = fmod(angle * 180.0 / pi, 360.0);
    if (deg < 0.0) deg += 360.0;
    host_dwt.CYCCNT += (uint32_t)llround(dt * SystemCoreClock);
    enc_deg = (float)deg;
    enc_valid = 1U;
    ++enc_valid_count;
    foc_update_speed();
}

static void start(double angle, float direction)
{
    resetMotorControl();
    foc_dir = direction;
    sample(angle, 0.0);
    NEAR(foc_speed_rad_s, 0.0, 0.0);
    NEAR(foc_speed_raw_rad_s, 0.0, 0.0);
}

static void constant_speed(double velocity, float direction)
{
    double t = 0.0, angle = 6.27;
    unsigned i;
    start(angle, direction);
    for (i = 0U; i < 30000U; ++i) {
        double dt = (80.0 + (double)(i % 11U) * 11.0) * 1e-6;
        t += dt;
        sample(angle + velocity * t, dt);
        if (t > 0.1) {
            CHECK(isfinite(foc_speed_rad_s));
            NEAR(foc_speed_rad_s, velocity * direction, 0.015);
        }
    }
    CHECK(foc_speed_dt_s >= 0.001f && foc_speed_dt_s < 0.0013f);
}

static void sample_rules(void)
{
    float held;
    unsigned count;
    start(1.0, 1.0f);
    sample(1.005, 0.0005);
    CHECK(foc_speed_dt_s == 0.0f); /* No update before the 1 ms gate. */
    sample(1.010, 0.0005);
    CHECK(foc_speed_dt_s > 0.0f);
    held = foc_speed_rad_s;
    count = speed_count;
    host_dwt.CYCCNT += 168000U;
    foc_update_speed();
    NEAR(foc_speed_rad_s, held, 0.0);
    CHECK(speed_count == count); /* Do not integrate a reused SPI reading. */
    enc_valid = 0U;
    ++enc_valid_count;
    foc_update_speed();
    NEAR(foc_speed_rad_s, held, 0.0);
    enc_valid = 1U;
    enc_deg = NAN;
    foc_update_speed();
    NEAR(foc_speed_rad_s, held, 0.0);
    enc_deg = 360.0f;
    foc_update_speed();
    NEAR(foc_speed_rad_s, held, 0.0);
    enc_deg = -1.0f;
    foc_update_speed();
    NEAR(foc_speed_rad_s, held, 0.0);
    sample(2.0, 0.050);
    NEAR(foc_speed_rad_s, 0.0, 0.0); /* Long gap rebases without a spike. */
    sample(2.01, 0.001);
    CHECK(isfinite(foc_speed_rad_s));
    CHECK(fabsf(foc_speed_rad_s) < 10.0f);
    host_dwt.CYCCNT = 0xFFFF0000U;
    start(6.28, 1.0f);
    for (unsigned i = 1U; i <= 100U; ++i) sample(6.28 + 10.0 * i * 0.001, 0.001);
    NEAR(foc_speed_rad_s, 10.0, 0.01); /* Unsigned cycle-counter wrap. */
    resetMotorControl();
    NEAR(foc_speed_rad_s, 0.0, 0.0);
    NEAR(foc_speed_raw_rad_s, 0.0, 0.0);
    NEAR(foc_speed_error_deg, 0.0, 0.0);
    NEAR(foc_speed_dt_s, 0.0, 0.0);
    sample(4.0, 0.001);
    NEAR(foc_speed_rad_s, 0.0, 0.0); /* Re-start angle creates no impulse. */
}

static void ramp(void)
{
    double t = 0.0;
    start(1.0, 1.0f);
    for (unsigned i = 0U; i < 1000U; ++i) {
        t += 0.001;
        sample(1.0 + 10.0 * t + 25.0 * t * t, 0.001);
        if (i > 100U) NEAR(foc_speed_rad_s, 10.0 + 50.0 * t, 0.35);
    }
}

static void noise(void)
{
    double raw2 = 0.0, out2 = 0.0, mean = 0.0;
    unsigned rng = 71U, n = 0U;
    start(1.0, 1.0f);
    for (unsigned i = 1U; i <= 12000U; ++i) {
        double angle, error;
        rng = rng * 1664525U + 1013904223U;
        error = ((double)(rng >> 8U) / 16777215.0 * 2.0 - 1.0) * 0.2 * pi / 180.0;
        angle = 1.0 + 10.0 * i * 0.001 + error;
        angle = round(angle / (2.0 * pi / 16384.0)) * (2.0 * pi / 16384.0);
        sample(angle, 0.001);
        if (i > 100U) {
            raw2 += pow(foc_speed_raw_rad_s - 10.0, 2.0);
            out2 += pow(foc_speed_rad_s - 10.0, 2.0);
            mean += foc_speed_rad_s;
            ++n;
        }
    }
    raw2 = sqrt(raw2 / n);
    out2 = sqrt(out2 / n);
    printf("Noise: raw RMS %.5f, observer RMS %.5f rad/s\n", raw2, out2);
    CHECK(out2 < raw2 * 0.5);
    NEAR(mean / n, 10.0, 0.01);
}

static void response(double hz)
{
    double s = 0.0, c = 0.0, n = 0.0, lag, gain;
    start(1.0, 1.0f);
    for (unsigned i = 1U; i <= 6000U; ++i) {
        double t = i * 0.001, phase = 2.0 * pi * hz * t;
        double angle = 1.0 + 10.0 * t + 2.0 / (2.0 * pi * hz) * (1.0 - cos(phase));
        sample(angle, 0.001);
        if (i > 1000U) {
            s += (foc_speed_rad_s - 10.0) * sin(phase);
            c += (foc_speed_rad_s - 10.0) * cos(phase);
            n += 1.0;
        }
    }
    lag = -atan2(c, s) / (2.0 * pi * hz);
    gain = sqrt(s*s + c*c) / n;
    printf("Response %.0f Hz: lag %.4f ms, gain %.5f\n", hz, lag * 1000.0, gain);
    CHECK(lag >= 0.0 && lag < 0.006);
    CHECK(gain > (hz <= 10.0 ? 0.95 : 0.85) && gain < 1.03);
}

int main(void)
{
    constant_speed(10.0, 1.0f);
    constant_speed(-10.0, 1.0f);
    constant_speed(10.0, -1.0f);
    constant_speed(0.0, 1.0f);
    sample_rules();
    ramp();
    noise();
    response(5.0);
    response(10.0);
    response(20.0);
    printf("%s: %u checks, %u failures\n", failures ? "FAIL" : "PASS", checks, failures);
    return failures ? 1 : 0;
}
