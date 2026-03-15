#include "bsp_vision_link.h"

#include "stm32f1xx_hal.h"
#include <string.h>

#define BSP_VISION_SOF             0xA5U// 帧头
#define BSP_VISION_MAX_PAYLOAD     24U//最大有效载荷字节数
#define BSP_VISION_TX_BUFFER_SIZE  256U//发送缓冲区大小

#define BSP_VISION_TYPE_TARGET     0x01U//目标类型
#define BSP_VISION_TYPE_TELEMETRY  0x02U//遥测类型

typedef enum
{
  BSP_VISION_PARSE_WAIT_SOF = 0,//等待帧头，固定值0xA5
  BSP_VISION_PARSE_WAIT_TYPE,//等待类型，0x01为目标误差，0x02为姿态数据
  BSP_VISION_PARSE_WAIT_LEN,//等待长度
  BSP_VISION_PARSE_WAIT_PAYLOAD,//等待有效载荷
  BSP_VISION_PARSE_WAIT_CRC//等待校验和，前四个字节的异或值，用于检测数据错误
} BSP_VisionParseState_t;

static BSP_VisionTarget_t s_target;
static BSP_VisionParseState_t s_parse_state = BSP_VISION_PARSE_WAIT_SOF;
static uint8_t s_parse_type = 0U;
static uint8_t s_parse_len = 0U;
static uint8_t s_parse_payload[BSP_VISION_MAX_PAYLOAD];
static uint8_t s_parse_idx = 0U;
static uint8_t s_parse_crc = 0U;

static uint8_t s_tx_buffer[BSP_VISION_TX_BUFFER_SIZE];
static uint16_t s_tx_head = 0U;//发送缓冲区头指针
static uint16_t s_tx_tail = 0U;//发送缓冲区尾指针

static int16_t BSP_Vision_FloatToCentiDeg(float value)
{//将浮点数转换为100倍的摄氏度，用于在串口通信中传输角度值
  float scaled;

  if (value > 327.67f)
  {
    value = 327.67f;
  }
  if (value < -327.68f)
  {
    value = -327.68f;
  }

  scaled = value * 100.0f;
  if (scaled >= 0.0f)
  {
    return (int16_t)(scaled + 0.5f);
  }

  return (int16_t)(scaled - 0.5f);
}

static uint16_t BSP_Vision_PackFrame(uint8_t type,//帧类型
                                     const uint8_t *payload,//有效载荷指针
                                     uint8_t len,//有效载荷长度
                                     uint8_t *out,//输出缓冲区指针
                                     uint16_t out_len)//输出缓冲区长度
{//打包vision帧，包括帧头、类型、长度、有效载荷和校验和，用于在串口通信中传输
  uint8_t crc;
  uint16_t i;

  if (out == 0 || payload == 0)
  {
    return 0U;
  }

  if (out_len < (uint16_t)(len + 4U))
  {
    return 0U;
  }

  out[0] = BSP_VISION_SOF;
  out[1] = type;
  out[2] = len;
  memcpy(&out[3], payload, len);

  crc = 0U;
  for (i = 0U; i < (uint16_t)(len + 3U); i++)
  {
    crc ^= out[i];//计算校验和，对帧头、类型、长度、有效载荷进行异或操作
  }

  out[len + 3U] = crc;//将校验和写入输出缓冲区的最后一个字节

  return (uint16_t)(len + 4U);//返回打包后的帧长度，包括帧头、类型、长度、有效载荷、校验和
}

static void BSP_Vision_OnFrame(uint8_t type, const uint8_t *payload, uint8_t len)
{//处理接收到的完整帧，根据类型和有效载荷更新目标信息
  int16_t yaw_err_cdeg;
  int16_t pitch_err_cdeg;

  if (type != BSP_VISION_TYPE_TARGET)
  {
    return;
  }

  if (len != 5U)
  {
    return;
  }

  yaw_err_cdeg = (int16_t)((uint16_t)payload[1] | ((uint16_t)payload[2] << 8U));
  pitch_err_cdeg = (int16_t)((uint16_t)payload[3] | ((uint16_t)payload[4] << 8U));

  s_target.target_valid = payload[0] ? 1U : 0U;
  s_target.yaw_err_deg = (float)yaw_err_cdeg / 100.0f;
  s_target.pitch_err_deg = (float)pitch_err_cdeg / 100.0f;
  s_target.timestamp_ms = HAL_GetTick();
}

static void BSP_Vision_PushTx(const uint8_t *data, uint16_t len)
{//将数据压入发送缓冲区，用于在串口通信中发送，环形队列
  uint16_t next_head;
  uint16_t i;

  if (data == 0U || len == 0U)
  {
    return;
  }

  for (i = 0U; i < len; i++)
  {
    next_head = (uint16_t)((s_tx_head + 1U) % BSP_VISION_TX_BUFFER_SIZE);
    if (next_head == s_tx_tail)
    {
      break;
    }

    s_tx_buffer[s_tx_head] = data[i];
    s_tx_head = next_head;
  }
}

void BSP_Vision_Init(void)
{//初始化vision链接，包括目标信息、解析状态、类型、长度、有效载荷索引、校验和、发送缓冲区头、尾指针
  memset(&s_target, 0, sizeof(s_target));
  s_parse_state = BSP_VISION_PARSE_WAIT_SOF;
  s_parse_type = 0U;
  s_parse_len = 0U;
  s_parse_idx = 0U;
  s_parse_crc = 0U;
  s_tx_head = 0U;
  s_tx_tail = 0U;
}

void BSP_Vision_FeedRx(const uint8_t *data, uint16_t len)
{//处理接收数据，根据帧头、类型、长度、有效载荷和校验和解析数据（状态机实现）
  uint16_t i;
  uint8_t byte;

  if (data == 0 || len == 0U)
  {
    return;
  }

  for (i = 0U; i < len; i++)
  {
    byte = data[i];

    switch (s_parse_state)
    {
    case BSP_VISION_PARSE_WAIT_SOF://等待帧头
      if (byte == BSP_VISION_SOF)
      {
        s_parse_state = BSP_VISION_PARSE_WAIT_TYPE;
        s_parse_crc = byte;
      }
      break;

    case BSP_VISION_PARSE_WAIT_TYPE://等待类型
      s_parse_type = byte;
      s_parse_crc ^= byte;//更新校验和
      s_parse_state = BSP_VISION_PARSE_WAIT_LEN;
      break;

    case BSP_VISION_PARSE_WAIT_LEN://等待长度
      s_parse_len = byte;
      s_parse_crc ^= byte;//更新校验和，按位异或，相同为0，不同为1
      if (s_parse_len > BSP_VISION_MAX_PAYLOAD)
      {
        s_parse_state = BSP_VISION_PARSE_WAIT_SOF;
      }
      else if (s_parse_len == 0U)
      {
        s_parse_state = BSP_VISION_PARSE_WAIT_CRC;
      }
      else
      {
        s_parse_idx = 0U;
        s_parse_state = BSP_VISION_PARSE_WAIT_PAYLOAD;
      }
      break;

    case BSP_VISION_PARSE_WAIT_PAYLOAD://等待有效载荷
      s_parse_payload[s_parse_idx++] = byte;
      s_parse_crc ^= byte;
      if (s_parse_idx >= s_parse_len)
      {
        s_parse_state = BSP_VISION_PARSE_WAIT_CRC;
      }
      break;

    case BSP_VISION_PARSE_WAIT_CRC://等待校验和
      if (byte == s_parse_crc)
      {
        BSP_Vision_OnFrame(s_parse_type, s_parse_payload, s_parse_len);
      }
      s_parse_state = BSP_VISION_PARSE_WAIT_SOF;
      break;

    default://默认状态，等待帧头
      s_parse_state = BSP_VISION_PARSE_WAIT_SOF;
      break;
    }
  }
}

uint8_t BSP_Vision_GetLatestTarget(BSP_VisionTarget_t *target, uint32_t timeout_ms)
{//获取最新的目标信息，根据超时时间判断是否有效
  uint32_t now;

  if (target == 0)
  {
    return 0U;
  }

  now = HAL_GetTick();
  if ((now - s_target.timestamp_ms) > timeout_ms)
  {
    return 0U;
  }

  *target = s_target;
  return 1U;
}

void BSP_Vision_SendTelemetry(const BSP_VisionTelemetry_t *telemetry)
{//发送遥测数据，包括模式、角度、角速度，小端序发送
  uint8_t payload[11];//有效载荷，包括模式、角度、角速度
  uint8_t frame[16];//打包后的vision帧，包括帧头、类型、长度、有效载荷和校验和
  uint16_t frame_len;
  int16_t yaw_cdeg;
  int16_t pitch_cdeg;
  int16_t roll_cdeg;
  int16_t yaw_rate_cdeg;
  int16_t pitch_rate_cdeg;

  if (telemetry == 0)
  {
    return;
  }

  yaw_cdeg = BSP_Vision_FloatToCentiDeg(telemetry->yaw_deg);
  pitch_cdeg = BSP_Vision_FloatToCentiDeg(telemetry->pitch_deg);
  roll_cdeg = BSP_Vision_FloatToCentiDeg(telemetry->roll_deg);
  yaw_rate_cdeg = BSP_Vision_FloatToCentiDeg(telemetry->yaw_rate_dps);
  pitch_rate_cdeg = BSP_Vision_FloatToCentiDeg(telemetry->pitch_rate_dps);

  payload[0] = telemetry->mode;//模式
  payload[1] = (uint8_t)(yaw_cdeg & 0xFF);//偏航角，单位：0.01度
  payload[2] = (uint8_t)((uint16_t)yaw_cdeg >> 8U);//偏航角，单位：0.01度/秒
  payload[3] = (uint8_t)(pitch_cdeg & 0xFF);//俯仰角，单位：0.01度
  payload[4] = (uint8_t)((uint16_t)pitch_cdeg >> 8U);//俯仰角
  payload[5] = (uint8_t)(roll_cdeg & 0xFF);//横滚角，单位：0.01度
  payload[6] = (uint8_t)((uint16_t)roll_cdeg >> 8U);//横滚角
  payload[7] = (uint8_t)(yaw_rate_cdeg & 0xFF);//偏航角速度，单位：0.01度
  payload[8] = (uint8_t)((uint16_t)yaw_rate_cdeg >> 8U);//偏航角速度，单位：0.01度/秒
  payload[9] = (uint8_t)(pitch_rate_cdeg & 0xFF);//俯仰角速度，单位：0.01度
  payload[10] = (uint8_t)((uint16_t)pitch_rate_cdeg >> 8U);//俯仰角速度，单位：0.01度/秒

  frame_len = BSP_Vision_PackFrame(BSP_VISION_TYPE_TELEMETRY,
                                   payload,
                                   (uint8_t)sizeof(payload),
                                   frame,
                                   (uint16_t)sizeof(frame));
  BSP_Vision_PushTx(frame, frame_len);
}

uint16_t BSP_Vision_PopTx(uint8_t *out, uint16_t max_len)
{//从发送缓冲区弹出数据，用于在串口通信中发送
  uint16_t count = 0U;

  if (out == 0 || max_len == 0U)
  {
    return 0U;
  }

  while ((s_tx_tail != s_tx_head) && (count < max_len))
  {
    out[count++] = s_tx_buffer[s_tx_tail];
    s_tx_tail = (uint16_t)((s_tx_tail + 1U) % BSP_VISION_TX_BUFFER_SIZE);
  }

  return count;
}
