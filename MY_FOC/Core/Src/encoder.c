
#include "encoder.h"

extern SPI_HandleTypeDef hspi1;

/* 编码器数据 */
volatile uint32_t enc_sb_count = 0;
volatile uint32_t enc_raw = 0;
volatile uint32_t enc_frame_raw = 0;
volatile float enc_deg = 0.0f;
volatile uint8_t enc_valid = 0U;
volatile uint32_t enc_valid_count = 0U;

/* 调试变量 */
volatile uint32_t enc_spi_fail = 0;
volatile uint32_t enc_parity_fail = 0;
volatile uint32_t enc_glitch_count = 0;
volatile uint16_t enc_last_frame = 0;
volatile uint32_t enc_zero_frame_count = 0U;
volatile uint32_t enc_zero_reject_count = 0U;
volatile uint16_t enc_last_rejected_frame = 0U;
volatile enc_reject_reason_t enc_last_reject_reason = ENC_REJECT_NONE;
volatile float enc_last_glitch_delta_deg = 0.0f;

volatile uint16_t enc_errfl = 0;
volatile uint16_t enc_errfl_first = 0xFFFF;
volatile uint8_t enc_errfl_valid = 0U;
volatile uint16_t enc_diag_boot = 0xFFFFU;
volatile uint8_t enc_diag_boot_valid = 0U;
volatile uint8_t enc_diag_boot_ready = 0U;

#define ENC_READ_ANGLECOM  0xFFFFU
#define ENC_READ_ERRFL     0x4001U
#define ENC_READ_DIAAGC    0xFFFCU
#define ENC_READ_NOP       0xC000U

/* 当前 SPI1 为 84 MHz / 256，半个 SCK 周期约 1.524 us。
 * 低电平建立/帧间高电平各留 1 us，末个时钟后留 3 us 再拉高 CS。
 * DWT 不清零，避免影响现有 USB 遥测计时。 */
static void enc_delay_us(uint32_t us)
{
    uint32_t cycles = ((SystemCoreClock + 999999U) / 1000000U) * us;
    uint32_t start = DWT->CYCCNT;
    while ((uint32_t)(DWT->CYCCNT - start) < cycles) __NOP();
}

static void enc_reject(enc_reject_reason_t reason, uint16_t frame)
{
    enc_last_rejected_frame = frame;
    enc_last_reject_reason = reason;
}

/* SPI 单帧传输 */
static HAL_StatusTypeDef enc_xfer(
    uint16_t tx,
    uint16_t *rx
)
{
    HAL_StatusTypeDef status;

    *rx = 0;

    /* CS 拉低 */
    HAL_GPIO_WritePin(
        GPIOA,
        GPIO_PIN_4,
        GPIO_PIN_RESET
    );
    enc_delay_us(1U);

    /* 注意：只传输一次！ */
    status = HAL_SPI_TransmitReceive(
        &hspi1,
        (uint8_t *)&tx,
        (uint8_t *)rx,
        1,
        10
    );

    /* 保证末个时钟到 CS 拉高的保持时间，不只依赖 HAL 返回开销。 */
    enc_delay_us(3U);
    /* CS 拉高 */
    HAL_GPIO_WritePin(
        GPIOA,
        GPIO_PIN_4,
        GPIO_PIN_SET
    );
    enc_delay_us(1U);

    return status;
}

/* 偶校验：16 位中 1 的数量必须为偶数 */
static uint8_t enc_parity_ok(uint16_t data)
{
    uint8_t parity = 0;

    while (data)
    {
        parity ^= (uint8_t)(data & 1U);
        data >>= 1;
    }

    return (parity == 0U);
}

/* 两帧流水读取；首帧响应属于上一个命令，第二帧才是本次寄存器。
 * 0xC000 是偶校验正确的读 NOP，不把 NOP 响应当角度。
 * 先验证 PARD，再由调用方解释 EF 和寄存器内容。 */
static uint8_t enc_read_register(uint16_t command, uint16_t *data)
{
    uint16_t dummy;
    *data = 0U;
    if (enc_xfer(command, &dummy) != HAL_OK || enc_xfer(ENC_READ_NOP, data) != HAL_OK)
    {
        ++enc_spi_fail;
        enc_reject(ENC_REJECT_SPI, *data);
        return 0U;
    }
    if (!enc_parity_ok(*data))
    {
        ++enc_parity_fail;
        enc_reject(ENC_REJECT_PARITY, *data);
        return 0U;
    }
    return 1U;
}

/* 更新编码器 */
void encoder_666_update(void)
{
    uint16_t data = 0U;

    enc_valid = 0U;

    if (!enc_read_register(ENC_READ_ANGLECOM, &data))
    {
        enc_last_frame = data;
        enc_frame_raw = data & 0x3FFFU;
        return;
    }

    /* 保存原始帧，方便调试 */
    enc_last_frame = data;
    enc_frame_raw = data & 0x3FFFU;
    if (data == 0x0000U) ++enc_zero_frame_count;

    /* 检查错误标志 EF */
    
    if (data & 0x4000U)
    {
        uint16_t err_data;
        ++enc_sb_count;
        enc_reject(ENC_REJECT_SENSOR_EF, data);
        enc_errfl_valid = 0U;
        /* 读 ERRFL 会清除传感器错误，返回帧也必须通过 PARD/EF 检查。 */
        if (enc_read_register(ENC_READ_ERRFL, &err_data) && !(err_data & 0x4000U))
        {
            enc_errfl = err_data & 0x0007U;
            enc_errfl_valid = 1U;
            if (enc_errfl_first == 0xFFFFU) enc_errfl_first = enc_errfl;
        }
        return;
    }

    /* 候选值暂存局部，全部通过后才同时发布 raw 和 deg。 */
    uint32_t new_raw = data & 0x3FFFU;

    /* 转换为机械角度 */
    float new_deg = (float)new_raw * 360.0f / 16384.0f;

    /* 角度跳变检测：偶校验挡不住 0x0000（0 个 1 也是偶），
     * 这种 SPI 坏读数会伪装成 0°，让电角度瞬间乱掉。
     * 转子有惯性，一次更新内机械角度不可能跳超过 30°，超过就当毛刺丢弃；
     * 正常的 0°/360° 过零只差 1° 以内，不会被误伤。 */
    static float last_valid_deg = 0.0f;
    static uint8_t have_valid = 0;
    if (have_valid)
    {
        float d = new_deg - last_valid_deg;
        if (d > 180.0f)      d -= 360.0f;
        else if (d < -180.0f) d += 360.0f;
        if (d > 30.0f || d < -30.0f)
        {
            enc_glitch_count++;
            if (new_raw == 0U) ++enc_zero_reject_count;
            enc_last_glitch_delta_deg = d;
            enc_reject(ENC_REJECT_GLITCH, data);
            return;   /* 丢弃这次坏读数，enc_deg 保留上次有效值 */
        }
    }

    enc_raw = new_raw;
    enc_deg = new_deg;
    last_valid_deg = new_deg;
    have_valid = 1;
    ++enc_valid_count;
    enc_valid = 1U;
}

void encoder_init(void)
{
    uint16_t diag;
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
    HAL_GPIO_WritePin(
        GPIOA,
        GPIO_PIN_4,
        GPIO_PIN_SET
    );

    HAL_Delay(10);
    /* 仅记录上电磁场/内部就绪状态，不写设置或 OTP，也不改变角度过滤。 */
    if (enc_read_register(ENC_READ_DIAAGC, &diag) && !(diag & 0x4000U))
    {
        enc_diag_boot = diag & 0x3FFFU;
        enc_diag_boot_valid = 1U;
        enc_diag_boot_ready = ((diag & 0x0100U) != 0U && (diag & 0x0200U) == 0U);
    }
}
