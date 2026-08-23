#include "hwt9018.h"

#define HWT_FRAME_HEAD       0x55U
#define HWT_FRAME_ANGLE      0x53U
#define HWT_FRAME_LENGTH     11U
#define HWT_YAW_SIGN         1.0f

static volatile uint8_t g_rx_frame[HWT_FRAME_LENGTH];
static volatile uint8_t g_rx_index;
static volatile float g_raw_yaw_deg;
static volatile float g_yaw_zero_deg;
static volatile uint8_t g_yaw_valid;
volatile uint32_t g_angle_frame_count;

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
    g_yaw_valid = 0U;
    g_angle_frame_count = 0U;

    crm_periph_clock_enable(CRM_UART5_PERIPH_CLOCK, TRUE);
    crm_periph_clock_enable(CRM_GPIOD_PERIPH_CLOCK, TRUE);

    gpio_default_para_init(&gpio);
    gpio.gpio_pins = GPIO_PINS_2;
    gpio.gpio_mode = GPIO_MODE_INPUT;
    gpio.gpio_pull = GPIO_PULL_UP;
    gpio_init(GPIOD, &gpio);

    usart_init(UART5, 115200U, USART_DATA_8BITS, USART_STOP_1_BIT);
    usart_receiver_enable(UART5, TRUE);
    usart_interrupt_enable(UART5, USART_RDBF_INT, TRUE);
    nvic_irq_enable(UART5_IRQn, 2U, 1U);
    usart_enable(UART5, TRUE);
}

void HWT9018_ZeroYaw(void)
{
    uint32_t primask;

    primask = __get_PRIMASK();
    __disable_irq();
    g_yaw_zero_deg = g_raw_yaw_deg;
    if(primask == 0U)
    {
        __enable_irq();
    }
}

float HWT9018_GetYawDeg(void)
{
    float raw_yaw;
    float zero_yaw;
    uint32_t primask;

    primask = __get_PRIMASK();
    __disable_irq();
    raw_yaw = g_raw_yaw_deg;
    zero_yaw = g_yaw_zero_deg;
    if(primask == 0U)
    {
        __enable_irq();
    }

    return HWT_WrapDeg((raw_yaw - zero_yaw) * HWT_YAW_SIGN);
}

uint8_t HWT9018_YawValid(void)
{
    return g_yaw_valid;
}

uint32_t HWT9018_GetAngleFrameCount(void)
{
    return g_angle_frame_count;
}

void UART5_IRQHandler(void)
{
    uint8_t data;
    uint8_t checksum;
    uint8_t i;
    int16_t raw_yaw;

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

    if((checksum == g_rx_frame[HWT_FRAME_LENGTH - 1U]) &&
       (g_rx_frame[1] == HWT_FRAME_ANGLE))
    {
        raw_yaw = (int16_t)(((uint16_t)g_rx_frame[7] << 8) |
                            (uint16_t)g_rx_frame[6]);
        g_raw_yaw_deg = ((float)raw_yaw * 180.0f) / 32768.0f;
        g_yaw_valid = 1U;
        g_angle_frame_count++;
    }

    g_rx_index = 0U;
}
