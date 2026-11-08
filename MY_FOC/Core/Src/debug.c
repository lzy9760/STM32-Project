#include "debug.h"
#include "usbd_core.h"
#include "usbd_cdc.h"
#include "usbd_desc.h"
#include "foc.h"
#include <math.h>

extern volatile float foc_ia, foc_ib, foc_ic;
extern volatile float enc_deg;

#define SCOPE_RING_SIZE 128U
#define SCOPE_BATCH_SIZE 12U
typedef struct {
    float channels[DEBUG_CHANNEL_COUNT];
    uint32_t tail;
} ScopeFrame;
typedef char ScopeFrame_must_be_44_bytes[(sizeof(ScopeFrame) == 44U) ? 1 : -1];
static ScopeFrame *Debug_BeginFrame(void);
static void Debug_EndFrame(ScopeFrame *frame);

/* ==================== VOFA+ 变量接口：只在这里选变量 ====================
 * JustFloat 通道从 CH0 开始；通道名称/单位在 VOFA+ 配置中设置。
 * 只读现有变量和 TIM1 比较值，不写电机控制参数或 PWM 寄存器。
 * PWM 为占空比包络 (%)，不是示波器上的高频开关脉冲。
 * 此入口仍由原电流闭环调用；对齐前/停机后没有新采样。
 */
void Debug_Capture(float id, float iq)
{
    ScopeFrame *frame = Debug_BeginFrame();
    (void)id;
    if (frame == NULL) return;

    frame->channels[0] = foc_ia;                  /* CH0: Ia (A) */
    frame->channels[1] = foc_ib;                  /* CH1: Ib (A) */
    frame->channels[2] = foc_ic;                  /* CH2: Ic (A) */
    frame->channels[3] = enc_deg;                 /* CH3: mechanical angle (deg) */
    frame->channels[4] = foc_speed_raw_rad_s;     /* CH4: raw speed (rad/s) */
    frame->channels[5] = sqrtf(foc_ud * foc_ud + foc_uq * foc_uq); /* CH5: voltage (V) */
    frame->channels[6] = foc_target_speed_rad_s;  /* CH6: Speed_ref (rad/s) */
    frame->channels[7] = foc_speed_rad_s;         /* CH7: Speed_fb (rad/s) */
    frame->channels[8] = foc_target_iq;           /* CH8: Iq_ref (A) */
    frame->channels[9] = iq;                      /* CH9: Iq_fb (A) */
    Debug_EndFrame(frame);
}
/* ==================== 以下为 USB 核心，无需日常修改 ==================== */

static USBD_HandleTypeDef device;
static ScopeFrame ring[SCOPE_RING_SIZE];
static ScopeFrame tx[SCOPE_BATCH_SIZE];
static uint32_t class_memory[256];
/* ADC IRQ produces frames; foreground consumes them. */
static volatile uint32_t read_index, write_index;
static uint32_t last_cycles, period_cycles;
static uint32_t last_task_ms, rate_cycles, rate_samples;
static uint8_t rx[CDC_DATA_FS_MAX_PACKET_SIZE];
static uint8_t coding[7] = {0x00, 0xC2, 0x01, 0x00, 0, 0, 8};
static volatile uint8_t stream_open, capture_active, discard_pending;
volatile uint32_t debug_dropped;
volatile uint32_t debug_sent;
volatile uint32_t debug_error;
volatile uint32_t debug_skipped;
volatile uint32_t debug_invalid;
volatile uint32_t debug_samples;
volatile float debug_sample_hz;

void *Debug_Alloc(uint32_t size)
{
    if (size > sizeof(class_memory)) return NULL;
    memset(class_memory, 0, sizeof(class_memory));
    return class_memory;
}
void Debug_Free(void *pointer) { (void)pointer; }

static int8_t cdc_init(void)
{
    stream_open = 0; capture_active = 0; discard_pending = 1;
    USBD_CDC_SetTxBuffer(&device, (uint8_t *)tx, 0);
    USBD_CDC_SetRxBuffer(&device, rx);
    return USBD_OK;
}
static int8_t cdc_deinit(void)
{
    stream_open = 0; capture_active = 0; discard_pending = 1;
    return USBD_OK;
}
static int8_t cdc_control(uint8_t command, uint8_t *buffer, uint16_t length)
{
    if (command == CDC_SET_LINE_CODING && length == 7U) memcpy(coding, buffer, 7);
    else if (command == CDC_GET_LINE_CODING) memcpy(buffer, coding, 7);
    else if (command == CDC_SET_CONTROL_LINE_STATE) {
        USBD_SetupReqTypedef *request = (USBD_SetupReqTypedef *)buffer;
        stream_open = (request->wValue & 1U) ? 1U : 0U;
        capture_active = 0; discard_pending = 1;
    }
    return USBD_OK;
}
static int8_t cdc_receive(uint8_t *buffer, uint32_t *length)
{
    /* Read-only telemetry: host bytes cannot change motor parameters. */
    (void)buffer; (void)length;
    USBD_CDC_SetRxBuffer(&device, rx);
    USBD_CDC_ReceivePacket(&device);
    return USBD_OK;
}
static USBD_CDC_ItfTypeDef interface = {cdc_init, cdc_deinit, cdc_control, cdc_receive, NULL};

void Debug_Init(void)
{
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk; /* Do not reset debugger's cycle counter. */
    last_cycles = DWT->CYCCNT;
    rate_cycles = last_cycles;
    period_cycles = SystemCoreClock / DEBUG_SAMPLE_RATE_HZ;
    if (period_cycles == 0U) period_cycles = 1U;
    if (USBD_Init(&device, &VCP_Desc, 0) != USBD_OK ||
        USBD_RegisterClass(&device, &USBD_CDC) != USBD_OK ||
        USBD_CDC_RegisterInterface(&device, &interface) != USBD_OK ||
        USBD_Start(&device) != USBD_OK) debug_error = 1;
}

static ScopeFrame *Debug_BeginFrame(void)
{
    uint32_t now, elapsed, slots, next;
    if (!stream_open || discard_pending) return NULL;
    now = DWT->CYCCNT;
    if (!capture_active) {
        last_cycles = now;
        capture_active = 1;
        return NULL;
    }
    elapsed = (uint32_t)(now - last_cycles);
    if (elapsed < period_cycles) return NULL;
    /* Keep a fixed time grid; select the first completed control call per slot.
     * Never wait or catch up by replaying samples after a delayed control call. */
    slots = elapsed / period_cycles;
    last_cycles += slots * period_cycles;
    debug_skipped += slots - 1U;
    next = (write_index + 1U) & (SCOPE_RING_SIZE - 1U);
    if (next == read_index) { ++debug_dropped; return NULL; }
    return &ring[write_index];
}

static void Debug_EndFrame(ScopeFrame *frame)
{
    uint32_t i, bits;
    /* JustFloat reserves +infinity as a delimiter. Reject invalid frames,
     * without changing the source variables or sending invented values. */
    for (i = 0; i < DEBUG_CHANNEL_COUNT; ++i) {
        memcpy(&bits, &frame->channels[i], sizeof(bits));
        if ((bits & 0x7F800000UL) == 0x7F800000UL) {
            ++debug_invalid;
            return;
        }
    }
    frame->tail = 0x7F800000UL; /* JustFloat tail: 00 00 80 7F, little-endian. */
    __DMB(); /* Publish only after the complete frame has been stored. */
    write_index = (write_index + 1U) & (SCOPE_RING_SIZE - 1U);
    ++debug_samples;
}

void Debug_Plot4(float ch1, float ch2, float ch3, float ch4,
                 const char *name1, const char *name2, const char *name3, const char *name4)
{
    ScopeFrame *frame = Debug_BeginFrame();
    uint32_t i;
    (void)name1; (void)name2; (void)name3; (void)name4;
    if (frame == NULL) return;
    frame->channels[0] = ch1; frame->channels[1] = ch2;
    frame->channels[2] = ch3; frame->channels[3] = ch4;
    for (i = 4; i < DEBUG_CHANNEL_COUNT; ++i) frame->channels[i] = 0.0f;
    Debug_EndFrame(frame);
}

void Debug_Task(void)
{
    uint32_t now = HAL_GetTick(), n = 0, cursor, available, cycles, elapsed;
    USBD_CDC_HandleTypeDef *cdc;
    if (now == last_task_ms) return;
    last_task_ms = now;
    cycles = DWT->CYCCNT;
    elapsed = (uint32_t)(cycles - rate_cycles);
    if (elapsed >= SystemCoreClock) {
        debug_sample_hz = (float)(debug_samples - rate_samples) *
                          ((float)SystemCoreClock / (float)elapsed);
        rate_samples = debug_samples;
        rate_cycles = cycles;
    }
    /* USB reset/class deinit can occur in its IRQ. Exclude only that IRQ,
     * leaving the existing ADC and all other interrupt priorities intact. */
    HAL_NVIC_DisableIRQ(OTG_FS_IRQn);
    cdc = (USBD_CDC_HandleTypeDef *)device.pClassData;
    if (discard_pending || !stream_open || device.dev_state != USBD_STATE_CONFIGURED || cdc == NULL) {
        read_index = write_index;
        discard_pending = 0;
    } else if (cdc->TxState == 0U) {
        cursor = read_index;
        available = write_index;
        __DMB(); /* Acquire the producer's fully written frame snapshot. */
        while (cursor != available && n < SCOPE_BATCH_SIZE) {
            tx[n++] = ring[cursor];
            cursor = (cursor + 1U) & (SCOPE_RING_SIZE - 1U);
        }
        if (n != 0U) {
            USBD_CDC_SetTxBuffer(&device, (uint8_t *)tx, n * sizeof(ScopeFrame));
            if (USBD_CDC_TransmitPacket(&device) == USBD_OK) {
                __DMB(); /* Release ring slots only after their data was copied. */
                read_index = cursor;
                debug_sent += n;
            }
        }
    }
    HAL_NVIC_EnableIRQ(OTG_FS_IRQn);
}
