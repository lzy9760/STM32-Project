#include "remote.h"
#include <string.h>

/* AA + 4摇杆 + 旋钮 + 使能 + 开环 + 55，共9字节 */
#define REMOTE_FRAME_LEN          9U
#define REMOTE_FRAME_HEAD         0xAAU
#define REMOTE_FRAME_FOOT         0x55U

#define REMOTE_STICK_CENTER       100.0f
#define REMOTE_STICK_HALF         100.0f
#define REMOTE_KNOB_MAX           200.0f
#define REMOTE_STICK_DEADBAND     0.05f

#define REMOTE_MAX_VX             3.8f
#define REMOTE_MAX_VY             2.8f
#define REMOTE_MAX_VW             4.2f
#define REMOTE_TIMEOUT_MS         500U

/* 最近收到的9字节 */
static volatile uint8_t rx_window[REMOTE_FRAME_LEN];

/* 已确认完整的数据帧 */
static volatile uint8_t rx_frame[REMOTE_FRAME_LEN];

static volatile uint8_t rx_count = 0;
static volatile uint8_t rx_complete = 0;
static volatile uint8_t rx_seen = 0;
static volatile uint32_t rx_last_cycle = 0;

/* 调试变量 */
volatile uint32_t remote_rx_count = 0;
volatile uint32_t remote_frame_count = 0;

remote_t remote;


static float remote_clamp_axis(float value)
{
    if(value > 1.0f)
    {
        value = 1.0f;
    }
    else if(value < -1.0f)
    {
        value = -1.0f;
    }

    if((value > -REMOTE_STICK_DEADBAND) &&
       (value < REMOTE_STICK_DEADBAND))
    {
        value = 0.0f;
    }

    return value;
}


static void remote_stop(void)
{
    remote.fwd = 0.0f;
    remote.strafe = 0.0f;
    remote.rot = 0.0f;
    remote.vx = 0.0f;
    remote.vy = 0.0f;
    remote.vw = 0.0f;
}


/* =========================
   USART3初始化
   ========================= */
void remote_init(void)
{
    gpio_init_type gpio;

    crm_periph_clock_enable(CRM_USART3_PERIPH_CLOCK, TRUE);
    crm_periph_clock_enable(CRM_GPIOB_PERIPH_CLOCK, TRUE);

    gpio_default_para_init(&gpio);

    /* PB10 TX */
    gpio.gpio_pins      = GPIO_PINS_10;
    gpio.gpio_mode      = GPIO_MODE_MUX;
    gpio.gpio_out_type  = GPIO_OUTPUT_PUSH_PULL;
    gpio.gpio_pull      = GPIO_PULL_NONE;
    gpio_init(GPIOB, &gpio);

    /* PB11 RX */
    gpio.gpio_pins = GPIO_PINS_11;
    gpio.gpio_mode = GPIO_MODE_INPUT;
    gpio.gpio_pull = GPIO_PULL_UP;
    gpio_init(GPIOB, &gpio);

    /* 115200 8N1 */
    usart_init(
        USART3,
        115200,
        USART_DATA_8BITS,
        USART_STOP_1_BIT
    );

    usart_transmitter_enable(USART3, TRUE);
    usart_receiver_enable(USART3, TRUE);

    /*
     * 只开接收数据中断
     * 不再使用IDLE判断一帧
     */
    usart_interrupt_enable(
        USART3,
        USART_RDBF_INT,
        TRUE
    );

    nvic_irq_enable(USART3_IRQn, 2, 0);

    usart_enable(USART3, TRUE);

    memset(&remote, 0, sizeof(remote));
    remote.header = REMOTE_FRAME_HEAD;
    remote.stick_x1 = (uint8_t)REMOTE_STICK_CENTER;
    remote.stick_y1 = (uint8_t)REMOTE_STICK_CENTER;
    remote.stick_x2 = (uint8_t)REMOTE_STICK_CENTER;
    remote.stick_y2 = (uint8_t)REMOTE_STICK_CENTER;
    remote.knob_ratio = 0.25f;
    remote.footer = REMOTE_FRAME_FOOT;

    /* 用DWT周期计数实现与Chs一致的500ms断链停机，不占用其它定时器 */
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}


/* =========================
   USART3接收中断
   ========================= */
void USART3_IRQHandler(void)
{
    uint8_t data;
    uint8_t i;

    if(usart_flag_get(
           USART3,
           USART_RDBF_FLAG) != RESET)
    {
        data =
            (uint8_t)usart_data_receive(USART3);

        remote_rx_count++;

        /* 保存最近9个字节 */
        if(rx_count < REMOTE_FRAME_LEN)
        {
            rx_window[rx_count] = data;
            rx_count++;
        }
        else
        {
            for(i = 0; i < (REMOTE_FRAME_LEN - 1U); i++)
            {
                rx_window[i] =
                    rx_window[i + 1];
            }

            rx_window[REMOTE_FRAME_LEN - 1U] = data;
        }

        /* AA + 4摇杆 + 旋钮 + 使能 + 开环 + 55 */
        if(rx_count >= REMOTE_FRAME_LEN)
        {
            if((rx_window[0] == REMOTE_FRAME_HEAD) &&
               (rx_window[REMOTE_FRAME_LEN - 1U] == REMOTE_FRAME_FOOT))
            {
                for(i = 0; i < REMOTE_FRAME_LEN; i++)
                {
                    rx_frame[i] =
                        rx_window[i];
                }

                rx_complete = 1;
                rx_seen = 1;
                rx_last_cycle = DWT->CYCCNT;
                remote_frame_count++;
            }
        }
    }
}



void remote_parse(void)
{
    uint8_t frame[REMOTE_FRAME_LEN];
    uint8_t i;
    float knob;
    uint32_t timeout_cycles;

    timeout_cycles =
        (system_core_clock / 1000U) * REMOTE_TIMEOUT_MS;

    if(rx_complete == 0)
    {
        if((rx_seen == 0U) ||
           ((uint32_t)(DWT->CYCCNT - rx_last_cycle) > timeout_cycles))
        {
            remote_stop();
        }
        return;
    }

    
     /*防止复制的时候中断又修改rx_frame*/
    __disable_irq();

    for(i = 0; i < REMOTE_FRAME_LEN; i++)
    {
        frame[i] = rx_frame[i];
    }

    rx_complete = 0;

    __enable_irq();


    if((frame[0] != REMOTE_FRAME_HEAD) ||
       (frame[REMOTE_FRAME_LEN - 1U] != REMOTE_FRAME_FOOT))
    {
        return;
    }

    remote.header = frame[0];
    remote.stick_x1 = frame[1];
    remote.stick_y1 = frame[2];
    remote.stick_x2 = frame[3];
    remote.stick_y2 = frame[4];
    remote.knob = frame[5];
    remote.enable = frame[6];
    remote.open_loop = frame[7];
    remote.footer = frame[8];

    /* 实测映射：左Y前后、左X横移、右X旋转 */
    remote.fwd = remote_clamp_axis(
        ((float)remote.stick_y1 - REMOTE_STICK_CENTER) /
        REMOTE_STICK_HALF);
    remote.strafe = remote_clamp_axis(
        ((float)remote.stick_x1 - REMOTE_STICK_CENTER) /
        REMOTE_STICK_HALF);
    remote.rot = remote_clamp_axis(
        ((float)remote.stick_x2 - REMOTE_STICK_CENTER) /
        REMOTE_STICK_HALF);

    knob = (float)remote.knob / REMOTE_KNOB_MAX;
    if(knob > 1.0f)
    {
        knob = 1.0f;
    }
    remote.knob_ratio = 0.25f + 0.75f * knob;

    remote.vx = remote.fwd * REMOTE_MAX_VX * remote.knob_ratio;
    remote.vy = remote.strafe * REMOTE_MAX_VY * remote.knob_ratio;
    remote.vw = remote.rot * REMOTE_MAX_VW * remote.knob_ratio;
}
