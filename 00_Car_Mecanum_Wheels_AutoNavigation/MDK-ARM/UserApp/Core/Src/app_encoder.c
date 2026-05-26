#include "app_encoder.h"

#include "main.h"

#define ENCODER_U1_DIR_SIGN -1  
#define ENCODER_U2_DIR_SIGN  1  
#define ENCODER_U3_DIR_SIGN -1  
#define ENCODER_U4_DIR_SIGN  1  

typedef struct
{
  TIM_HandleTypeDef *htim;
  int8_t direction_sign;
  uint16_t last_count;
} Encoder_Timer_t;

extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;
extern TIM_HandleTypeDef htim8;

static Encoder_Timer_t g_encoders[ENCODER_COUNT] =
{
  {&htim1,  ENCODER_U1_DIR_SIGN, 0U},
  {&htim4,  ENCODER_U2_DIR_SIGN, 0U},
  {&htim3,  ENCODER_U3_DIR_SIGN, 0U},
  {&htim8,  ENCODER_U4_DIR_SIGN, 0U}
};
static volatile int32_t g_encoder_count[ENCODER_COUNT];

static void Encoder_UpdateOne(uint8_t encoder_id)
{
  uint16_t now_count;
  int16_t count_delta;

  if (encoder_id >= ENCODER_COUNT)
  {
    return;
  }

  now_count = (uint16_t)__HAL_TIM_GET_COUNTER(g_encoders[encoder_id].htim);
  count_delta = (int16_t)(now_count - g_encoders[encoder_id].last_count);
  g_encoders[encoder_id].last_count = now_count;
  g_encoder_count[encoder_id] += (int32_t)count_delta * g_encoders[encoder_id].direction_sign;
}

void Encoder_Init(void)
{
  uint8_t encoder_id;

  for (encoder_id = 0U; encoder_id < ENCODER_COUNT; encoder_id++)
  {
    if (HAL_TIM_Encoder_Start(g_encoders[encoder_id].htim, TIM_CHANNEL_ALL) != HAL_OK)
    {
      Error_Handler();
    }
    __HAL_TIM_SET_COUNTER(g_encoders[encoder_id].htim, 0U);
    g_encoders[encoder_id].last_count = 0U;
    g_encoder_count[encoder_id] = 0;
  }
}

void Encoder_Scan(void)
{
  uint8_t encoder_id;
  uint32_t primask;

  primask = __get_PRIMASK();
  __disable_irq();
  for (encoder_id = 0U; encoder_id < ENCODER_COUNT; encoder_id++)
  {
    Encoder_UpdateOne(encoder_id);
  }
  if (primask == 0U)
  {
    __enable_irq();
  }
}

int32_t Encoder_GetCount(uint8_t encoder_id)
{
  int32_t count;
  uint32_t primask;

  if (encoder_id >= ENCODER_COUNT)
  {
    return 0;
  }

  primask = __get_PRIMASK();
  __disable_irq();
  Encoder_UpdateOne(encoder_id);
  count = g_encoder_count[encoder_id];
  if (primask == 0U)
  {
    __enable_irq();
  }

  return count;
}

void Encoder_ResetCount(uint8_t encoder_id)
{
  uint32_t primask;

  if (encoder_id >= ENCODER_COUNT)
  {
    return;
  }

  primask = __get_PRIMASK();
  __disable_irq();
  g_encoder_count[encoder_id] = 0;
  __HAL_TIM_SET_COUNTER(g_encoders[encoder_id].htim, 0U);
  g_encoders[encoder_id].last_count = 0U;
  if (primask == 0U)
  {
    __enable_irq();
  }
}

void Encoder_ResetAll(void)
{
  uint8_t encoder_id;
  uint32_t primask;

  primask = __get_PRIMASK();
  __disable_irq();
  for (encoder_id = 0U; encoder_id < ENCODER_COUNT; encoder_id++)
  {
    g_encoder_count[encoder_id] = 0;
    __HAL_TIM_SET_COUNTER(g_encoders[encoder_id].htim, 0U);
    g_encoders[encoder_id].last_count = 0U;
  }
  if (primask == 0U)
  {
    __enable_irq();
  }
}
