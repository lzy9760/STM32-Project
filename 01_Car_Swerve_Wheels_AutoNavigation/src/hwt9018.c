#include "hwt9018.h"

#define HWT_FRAME_HEAD       0x55U
#define HWT_FRAME_GYRO       0x52U
#define HWT_FRAME_ANGLE      0x53U
#define HWT_FRAME_LENGTH     11U
#define HWT_YAW_SIGN         1.0f
#define HWT_GYRO_Z_SIGN      HWT_YAW_SIGN
#define HWT_GYRO_RANGE_DPS   2000.0f
#define HWT_GYRO_BIAS_LIMIT_DPS       1.0f
#define HWT_GYRO_BIAS_MIN_SAMPLES     20U
#define HWT_GYRO_BIAS_WINDOW_SAMPLES  32U
#define HWT_COMMAND_LENGTH             5U
#define HWT_CONFIG_STARTUP_DELAY_MS    500U
#define HWT_CONFIG_COMMAND_DELAY_MS    50U

extern volatile uint32_t control_time_ms;

static volatile uint8_t g_rx_frame[HWT_FRAME_LENGTH];
static volatile uint8_t g_rx_index;
static volatile float g_raw_yaw_deg;
static volatile float g_yaw_zero_deg;
static volatile float g_raw_gyro_z_dps;
static volatile float g_gyro_z_bias_dps;
static volatile float g_gyro_bias_sum_dps;
static volatile float g_gyro_bias_samples[HWT_GYRO_BIAS_WINDOW_SAMPLES];
static volatile uint32_t g_gyro_bias_sample_count;
static volatile uint8_t g_gyro_bias_sample_index;
static volatile uint8_t g_gyro_bias_locked;
static volatile uint8_t g_gyro_bias_lock_requested;
static volatile uint8_t g_yaw_valid;
static volatile uint8_t g_gyro_valid;
volatile uint32_t g_angle_frame_count;
volatile float g_gyro_z_dps;
volatile float g_hwt_yaw_deg;
volatile float g_hwt_gyro_bias_dps;
volatile uint32_t g_gyro_frame_count;
volatile uint32_t g_angle_frame_timestamp_ms;
volatile uint32_t g_gyro_frame_timestamp_ms;
volatile uint32_t g_hwt_checksum_error_count;
volatile uint8_t g_gyro_bias_valid;
volatile uint8_t g_hwt_output_config_sent;

static const uint8_t g_hwt_unlock_command[HWT_COMMAND_LENGTH] =
{
    0xFFU, 0xAAU, 0x69U, 0x88U, 0xB5U
};

static const uint8_t g_hwt_gyro_angle_command[HWT_COMMAND_LENGTH] =
{
    0xFFU, 0xAAU, 0x02U, 0x0CU, 0x00U
};

static void HWT_SendCommand(const uint8_t *command)
{
    uint8_t i;

    for(i = 0U; i < HWT_COMMAND_LENGTH; i++)
    {
        while(usart_flag_get(UART5, USART_TDBE_FLAG) == RESET)
        {
        }
        usart_data_transmit(UART5, command[i]);
    }
    while(usart_flag_get(UART5, USART_TDC_FLAG) == RESET)
    {
    }
}

static float HWT_WrapDeg(float angle)
{
    while(angle > 180.0f)
    {
        angle -= 360.0f;
    }
    while(angle < -180.0f)
    {
        angle += 360.0f;
    }
    return angle;
}

void HWT9018_Init(void)
{
    gpio_init_type gpio;

    g_rx_index = 0U;
    g_raw_yaw_deg = 0.0f;
    g_yaw_zero_deg = 0.0f;
    g_raw_gyro_z_dps = 0.0f;
    g_gyro_z_bias_dps = 0.0f;
    g_gyro_bias_sum_dps = 0.0f;
    g_gyro_bias_sample_count = 0U;
    g_gyro_bias_sample_index = 0U;
    g_gyro_bias_locked = 0U;
    g_gyro_bias_lock_requested = 0U;
    g_yaw_valid = 0U;
    g_gyro_valid = 0U;
    g_angle_frame_count = 0U;
    g_gyro_z_dps = 0.0f;
    g_hwt_yaw_deg = 0.0f;
    g_hwt_gyro_bias_dps = 0.0f;
    g_gyro_frame_count = 0U;
    g_angle_frame_timestamp_ms = 0U;
    g_gyro_frame_timestamp_ms = 0U;
    g_hwt_checksum_error_count = 0U;
    g_gyro_bias_valid = 0U;
    g_hwt_output_config_sent = 0U;

    crm_periph_clock_enable(CRM_UART5_PERIPH_CLOCK, TRUE);
    crm_periph_clock_enable(CRM_GPIOC_PERIPH_CLOCK, TRUE);
    crm_periph_clock_enable(CRM_GPIOD_PERIPH_CLOCK, TRUE);

    gpio_default_para_init(&gpio);
    gpio.gpio_pins = GPIO_PINS_12;
    gpio.gpio_mode = GPIO_MODE_MUX;
    gpio.gpio_out_type = GPIO_OUTPUT_PUSH_PULL;
    gpio.gpio_pull = GPIO_PULL_NONE;
    gpio_init(GPIOC, &gpio);

    gpio.gpio_pins = GPIO_PINS_2;
    gpio.gpio_mode = GPIO_MODE_INPUT;
    gpio.gpio_pull = GPIO_PULL_UP;
    gpio_init(GPIOD, &gpio);

    usart_init(UART5, 115200U, USART_DATA_8BITS, USART_STOP_1_BIT);
    usart_transmitter_enable(UART5, TRUE);
    usart_receiver_enable(UART5, TRUE);
    usart_interrupt_enable(UART5, USART_RDBF_INT, TRUE);
    nvic_irq_enable(UART5_IRQn, 2U, 1U);
    usart_enable(UART5, TRUE);

    delay_ms(HWT_CONFIG_STARTUP_DELAY_MS);
    HWT_SendCommand(g_hwt_unlock_command);
    delay_ms(HWT_CONFIG_COMMAND_DELAY_MS);
    HWT_SendCommand(g_hwt_gyro_angle_command);
    g_hwt_output_config_sent = 1U;
}

void HWT9018_ZeroYaw(void)
{
    uint32_t primask;

    primask = __get_PRIMASK();
    __disable_irq();
    g_yaw_zero_deg = g_raw_yaw_deg;
    g_hwt_yaw_deg = 0.0f;
    if(g_gyro_bias_sample_count >= HWT_GYRO_BIAS_MIN_SAMPLES)
    {
        g_gyro_z_bias_dps = g_gyro_bias_sum_dps /
                            (float)g_gyro_bias_sample_count;
        g_gyro_bias_valid = 1U;
        g_gyro_bias_locked = 1U;
        g_gyro_bias_lock_requested = 0U;
    }
    else
    {
        g_gyro_z_bias_dps = 0.0f;
        g_gyro_bias_valid = 0U;
        g_gyro_bias_locked = 0U;
        g_gyro_bias_lock_requested = 1U;
    }
    g_hwt_gyro_bias_dps = g_gyro_z_bias_dps * HWT_GYRO_Z_SIGN;
    g_gyro_z_dps = (g_raw_gyro_z_dps - g_gyro_z_bias_dps) *
                   HWT_GYRO_Z_SIGN;
    if(primask == 0U)
    {
        __enable_irq();
    }
}

float HWT9018_GetYawDeg(void)
{
    HWT9018Snapshot snapshot;

    HWT9018_GetSnapshot(&snapshot);
    return snapshot.yaw_deg;
}

float HWT9018_GetGyroZDegS(void)
{
    HWT9018Snapshot snapshot;

    HWT9018_GetSnapshot(&snapshot);
    return snapshot.gyro_z_deg_s;
}

uint8_t HWT9018_YawValid(void)
{
    return g_yaw_valid;
}

uint8_t HWT9018_GyroValid(void)
{
    return g_gyro_valid;
}

uint8_t HWT9018_GyroBiasValid(void)
{
    return g_gyro_bias_valid;
}

uint32_t HWT9018_GetAngleFrameCount(void)
{
    return g_angle_frame_count;
}

uint32_t HWT9018_GetGyroFrameCount(void)
{
    return g_gyro_frame_count;
}

void HWT9018_GetSnapshot(HWT9018Snapshot *snapshot)
{
    float raw_yaw;
    float zero_yaw;
    float raw_gyro_z;
    float gyro_bias;
    uint32_t primask;

    if(snapshot == 0)
    {
        return;
    }

    primask = __get_PRIMASK();
    __disable_irq();
    raw_yaw = g_raw_yaw_deg;
    zero_yaw = g_yaw_zero_deg;
    raw_gyro_z = g_raw_gyro_z_dps;
    gyro_bias = g_gyro_z_bias_dps;
    snapshot->angle_frame_count = g_angle_frame_count;
    snapshot->gyro_frame_count = g_gyro_frame_count;
    snapshot->angle_timestamp_ms = g_angle_frame_timestamp_ms;
    snapshot->gyro_timestamp_ms = g_gyro_frame_timestamp_ms;
    snapshot->yaw_valid = g_yaw_valid;
    snapshot->gyro_valid = g_gyro_valid;
    snapshot->gyro_bias_valid = g_gyro_bias_valid;
    if(primask == 0U)
    {
        __enable_irq();
    }

    snapshot->yaw_deg = HWT_WrapDeg((raw_yaw - zero_yaw) *
                                    HWT_YAW_SIGN);
    snapshot->gyro_z_deg_s = (raw_gyro_z - gyro_bias) *
                             HWT_GYRO_Z_SIGN;
}

void UART5_IRQHandler(void)
{
    uint8_t data;
    uint8_t checksum;
    uint8_t i;
    int16_t raw_yaw;
    int16_t raw_gyro_z;
    float gyro_z_dps;

    if(usart_flag_get(UART5, USART_RDBF_FLAG) == RESET)
    {
        return;
    }

    data = (uint8_t)usart_data_receive(UART5);
    if(g_rx_index == 0U)
    {
        if(data == HWT_FRAME_HEAD)
        {
            g_rx_frame[0] = data;
            g_rx_index = 1U;
        }
        return;
    }

    g_rx_frame[g_rx_index] = data;
    g_rx_index++;
    if(g_rx_index < HWT_FRAME_LENGTH)
    {
        return;
    }

    checksum = 0U;
    for(i = 0U; i < (HWT_FRAME_LENGTH - 1U); i++)
    {
        checksum = (uint8_t)(checksum + g_rx_frame[i]);
    }

    if(checksum == g_rx_frame[HWT_FRAME_LENGTH - 1U])
    {
        if(g_rx_frame[1] == HWT_FRAME_ANGLE)
        {
            raw_yaw = (int16_t)(((uint16_t)g_rx_frame[7] << 8) |
                                (uint16_t)g_rx_frame[6]);
            g_raw_yaw_deg = ((float)raw_yaw * 180.0f) / 32768.0f;
            g_hwt_yaw_deg = HWT_WrapDeg((g_raw_yaw_deg -
                                         g_yaw_zero_deg) *
                                        HWT_YAW_SIGN);
            g_yaw_valid = 1U;
            g_angle_frame_count++;
            g_angle_frame_timestamp_ms = control_time_ms;
        }
        else if(g_rx_frame[1] == HWT_FRAME_GYRO)
        {
            raw_gyro_z = (int16_t)(((uint16_t)g_rx_frame[7] << 8) |
                                   (uint16_t)g_rx_frame[6]);
            gyro_z_dps = ((float)raw_gyro_z * HWT_GYRO_RANGE_DPS) /
                         32768.0f;
            g_raw_gyro_z_dps = gyro_z_dps;
            if(g_gyro_bias_locked == 0U)
            {
                if((gyro_z_dps >= -HWT_GYRO_BIAS_LIMIT_DPS) &&
                   (gyro_z_dps <= HWT_GYRO_BIAS_LIMIT_DPS))
                {
                    if(g_gyro_bias_sample_count <
                       HWT_GYRO_BIAS_WINDOW_SAMPLES)
                    {
                        g_gyro_bias_samples[g_gyro_bias_sample_index] =
                            gyro_z_dps;
                        g_gyro_bias_sum_dps += gyro_z_dps;
                        g_gyro_bias_sample_count++;
                    }
                    else
                    {
                        g_gyro_bias_sum_dps -=
                            g_gyro_bias_samples[g_gyro_bias_sample_index];
                        g_gyro_bias_samples[g_gyro_bias_sample_index] =
                            gyro_z_dps;
                        g_gyro_bias_sum_dps += gyro_z_dps;
                    }
                    g_gyro_bias_sample_index++;
                    if(g_gyro_bias_sample_index >=
                       HWT_GYRO_BIAS_WINDOW_SAMPLES)
                    {
                        g_gyro_bias_sample_index = 0U;
                    }
                    if((g_gyro_bias_lock_requested != 0U) &&
                       (g_gyro_bias_sample_count >=
                        HWT_GYRO_BIAS_MIN_SAMPLES))
                    {
                        g_gyro_z_bias_dps = g_gyro_bias_sum_dps /
                                            (float)g_gyro_bias_sample_count;
                        g_hwt_gyro_bias_dps =
                            g_gyro_z_bias_dps * HWT_GYRO_Z_SIGN;
                        g_gyro_bias_valid = 1U;
                        g_gyro_bias_locked = 1U;
                        g_gyro_bias_lock_requested = 0U;
                    }
                }
                else
                {
                    g_gyro_bias_sum_dps = 0.0f;
                    g_gyro_bias_sample_count = 0U;
                    g_gyro_bias_sample_index = 0U;
                }
            }
            g_gyro_z_dps = (gyro_z_dps - g_gyro_z_bias_dps) *
                           HWT_GYRO_Z_SIGN;
            g_gyro_valid = 1U;
            g_gyro_frame_count++;
            g_gyro_frame_timestamp_ms = control_time_ms;
        }
    }
    else
    {
        g_hwt_checksum_error_count++;
    }

    g_rx_index = 0U;
}
