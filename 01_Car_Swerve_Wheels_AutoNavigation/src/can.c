#include "can.h"
#include "motor.h"

volatile uint32_t id4_feedback_count = 0U;



void CAN_Init(void)
{
    can_base_type        can_base;
    can_baudrate_type    can_baud;
    can_filter_init_type can_filter;
    gpio_init_type       gpio;

    /* ---- 使能时钟 ---- */
    crm_periph_clock_enable(CRM_GPIOA_PERIPH_CLOCK, TRUE);
    crm_periph_clock_enable(CRM_GPIOB_PERIPH_CLOCK, TRUE);
    crm_periph_clock_enable(CRM_CAN1_PERIPH_CLOCK,  TRUE);
    crm_periph_clock_enable(CRM_CAN2_PERIPH_CLOCK,  TRUE);

    /* ---- CAN1 引脚: PA11=RX(上拉输入), PA12=TX(复用推挽) ---- */
    gpio_default_para_init(&gpio);

    gpio.gpio_pins  = GPIO_PINS_11;
    gpio.gpio_mode  = GPIO_MODE_INPUT;
    gpio.gpio_pull  = GPIO_PULL_UP;
    gpio_init(GPIOA, &gpio);

    gpio.gpio_pins      = GPIO_PINS_12;
    gpio.gpio_mode      = GPIO_MODE_MUX;
    gpio.gpio_out_type  = GPIO_OUTPUT_PUSH_PULL;
    gpio.gpio_pull      = GPIO_PULL_NONE;
    gpio_init(GPIOA, &gpio);

    /* ---- CAN2 引脚: PB12=RX(上拉输入), PB13=TX(复用推挽) ---- */
    gpio.gpio_pins  = GPIO_PINS_12;
    gpio.gpio_mode  = GPIO_MODE_INPUT;
    gpio.gpio_pull  = GPIO_PULL_UP;
    gpio_init(GPIOB, &gpio);

    gpio.gpio_pins      = GPIO_PINS_13;
    gpio.gpio_mode      = GPIO_MODE_MUX;
    gpio.gpio_out_type  = GPIO_OUTPUT_PUSH_PULL;
    gpio.gpio_pull      = GPIO_PULL_NONE;
    gpio_init(GPIOB, &gpio);

    /* ---- CAN 基础配置 ---- */
    can_default_para_init(&can_base);
    can_base.mode_selection    = CAN_MODE_COMMUNICATE;
    can_base.ttc_enable        = FALSE;
    can_base.aebo_enable       = TRUE;
    can_base.aed_enable        = TRUE;
    can_base.prsf_enable       = FALSE;
    can_base.mdrsel_selection  = CAN_DISCARDING_FIRST_RECEIVED;
    can_base.mmssr_selection   = CAN_SENDING_BY_ID;

    /* ---- CAN 波特率: 120MHz / 8 / 15 = 1MHz ---- */
    can_baud.baudrate_div = 8;
    can_baud.rsaw_size    = CAN_RSAW_3TQ;
    can_baud.bts1_size    = CAN_BTS1_11TQ;
    can_baud.bts2_size    = CAN_BTS2_3TQ;

    /* ---- CAN 过滤器: 接收所有帧 ---- */
    can_filter.filter_activate_enable = TRUE;
    can_filter.filter_mode            = CAN_FILTER_MODE_ID_MASK;
    can_filter.filter_fifo            = CAN_FILTER_FIFO0;
    can_filter.filter_number          = 0;
    can_filter.filter_bit             = CAN_FILTER_32BIT;
    can_filter.filter_id_high         = 0;
    can_filter.filter_id_low          = 0;
    can_filter.filter_mask_high       = 0;
    can_filter.filter_mask_low        = 0;

    /* ---- 应用到 CAN1 ---- */
    can_base_init(CAN1, &can_base);
    can_baudrate_set(CAN1, &can_baud);
    can_filter_init(CAN1, &can_filter);

    /* ---- 应用到 CAN2 ---- */
    can_base_init(CAN2, &can_base);
    can_baudrate_set(CAN2, &can_baud);
    can_filter_init(CAN2, &can_filter);
}


void CAN_Nb_Send(can_type *can, uint32_t id, int16_t m0, int16_t m1, int16_t m2, int16_t m3)
{
    can_tx_message_type tx_msg;

    tx_msg.standard_id = id;
    tx_msg.id_type     = CAN_ID_STANDARD;
    tx_msg.frame_type  = CAN_TFT_DATA;
    tx_msg.dlc         = 8;

    tx_msg.data[0] = (m0 >> 8) & 0xFF;
    tx_msg.data[1] = m0 & 0xFF;
    tx_msg.data[2] = (m1 >> 8) & 0xFF;
    tx_msg.data[3] = m1 & 0xFF;
    tx_msg.data[4] = (m2 >> 8) & 0xFF;
    tx_msg.data[5] = m2 & 0xFF;
    tx_msg.data[6] = (m3 >> 8) & 0xFF;
    tx_msg.data[7] = m3 & 0xFF;

    can_message_transmit(can, &tx_msg);
}


void CAN_Nb_Receive(uint8_t data[8], uint16_t *angle, int16_t *rpm, int16_t *current, uint8_t *temp)
{
    *angle   = ((uint16_t)data[0] << 8) | data[1];
    *rpm     = ((int16_t)data[2] << 8)  | data[3];
    *current = ((int16_t)data[4] << 8)  | data[5];
    *temp    = data[6];
}

void Motor_CAN_Nb_Receive(can_type *can_x,
                                  can_rx_message_type *rx_msg)
{
    Motor *target_motor = NULL;

    /* 可选：过滤掉非标准数据帧或长度异常帧 */
    if ((rx_msg->id_type != CAN_ID_STANDARD) ||
        (rx_msg->frame_type != CAN_TFT_DATA) ||
        (rx_msg->dlc != 8U))
    {
        return;
    }

    if (can_x == CAN1)
    {
        switch (rx_msg->standard_id)
        {
            /* CAN1：GM6020 ID1、ID3 */
            case 0x205U:
                target_motor = &motors[0];
                break;

            case 0x207U:
                target_motor = &motors[1];
                break;

            /* CAN1：M3508 ID1、ID3 */
            case 0x201U:
                target_motor = &motors[4];
                break;

            case 0x203U:
                target_motor = &motors[5];
                break;

            default:
                break;
        }
    }
    else if (can_x == CAN2)
    {
        switch (rx_msg->standard_id)
        {
            /* CAN2：GM6020 ID2、ID4 */
            case 0x206U:
                target_motor = &motors[2];
                break;

            case 0x208U:
                id4_feedback_count++;
                target_motor = &motors[3];
                break;

            /* CAN2：M3508 ID2、ID4 */
            case 0x202U:
                target_motor = &motors[6];
                break;

            case 0x204U:
                target_motor = &motors[7];
                break;

            default:
                break;
        }
    }

    if (target_motor != NULL)
    {
        Motor_Nb_Receive(target_motor, rx_msg->data);
    }
}
