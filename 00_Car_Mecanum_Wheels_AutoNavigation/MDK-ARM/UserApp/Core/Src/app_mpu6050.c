#include "app_mpu6050.h"
#include "app_imu_ahrs.h"
#include "app_imu_filter.h"
#include "main.h"

#include <math.h>



#define MPU6050_I2C_ADDR          0x68U
#define MPU6050_REG_SMPLRT_DIV    0x19U
#define MPU6050_REG_CONFIG        0x1AU
#define MPU6050_REG_GYRO_CONFIG   0x1BU
#define MPU6050_REG_ACCEL_CONFIG  0x1CU
#define MPU6050_REG_ACCEL_XOUT_H  0x3BU
#define MPU6050_REG_PWR_MGMT_1    0x6BU
#define MPU6050_REG_WHO_AM_I      0x75U
#define MPU6050_DLPF_CFG          0x04U   
#define MPU6050_GYRO_FS_SEL       0x00U   
#define MPU6050_GYRO_SENSITIVITY  131.0f
#define MPU6050_ACCEL_FS_SEL      0x00U   
#define MPU6050_ACCEL_SENSITIVITY 16384.0f
#define MPU6050_FRAME_LEN         14U

#define CALIB_WARMUP_MS           100U    
#define CALIB_SAMPLES             1000U   
#define CALIB_DELAY_MS            2U                  

#define HEADING_KP                8.0f
#define HEADING_MAX_CORRECTION    45.0f
#define HEADING_DEADBAND_DEG      0.5f
#define HEADING_FILTER_ALPHA      0.45f
#define HEADING_CORRECTION_SIGN   -1

#define STATIONARY_RATE_DPS       1.2f    
#define STATIONARY_BIAS_ALPHA     0.002f  
#define STATIC_GYRO_NORM_DPS      2.0f
#define STATIC_GYRO_HYSTERESIS    0.5f
#define MANUAL_TURN_SETTLE_MS     300U    

#define DT_NOMINAL                0.005f  

static ImuFilter_GyroAxis_t g_gyro_x_filter;
static ImuFilter_GyroAxis_t g_gyro_y_filter;
static ImuFilter_GyroAxis_t g_gyro_z_filter;
static ImuAhrs_Mahony_t     g_ahrs;


static I2C_HandleTypeDef  g_mpu_i2c;
static float              g_current_yaw;        
static float              g_current_roll;
static float              g_current_pitch;
static float              g_accel_x_offset_g;
static float              g_accel_y_offset_g;
static float              g_x_bias_dps;
static float              g_y_bias_dps;
static float              g_z_bias_dps;        
static uint8_t            g_ready;               
static uint32_t           g_last_tick;          

static float              g_ref_yaw;            
static uint8_t            g_hold_active;         
static uint8_t            g_last_car_moving;
static uint8_t            g_car_moving;//閻劋绨幐鍥┿仛鏉烇箒绶犻弰顖氭儊濮濓絽婀粔璇插З          
static float              g_corr_filtered;//閻劋绨€涙ê鍋嶅銈嗗皾閸氬海娈戦懜顏勬倻娣囶喗顒滈崐?      
static uint32_t           g_manual_turn_until;    
static uint8_t            g_static_last = 1U;

static float AbsFloat(float value)
{
  return (value >= 0.0f) ? value : -value;
}

static int16_t MPU_ParseI16(const uint8_t *buf)
{
  return (int16_t)(((uint16_t)buf[0] << 8) | buf[1]);
}

static float WrapYawDeg(float yaw)
{
  while (yaw > 180.0f)
  {
    yaw -= 360.0f;
  }
  while (yaw < -180.0f)
  {
    yaw += 360.0f;
  }
  return yaw;
}

static uint8_t IsStaticByGyro(float gx_dps, float gy_dps, float gz_dps)
{
  float gyro_norm;
  float threshold;

  gyro_norm = sqrtf((gx_dps * gx_dps) + (gy_dps * gy_dps) + (gz_dps * gz_dps));
  threshold = (g_static_last != 0U) ?
              (STATIC_GYRO_NORM_DPS + STATIC_GYRO_HYSTERESIS) :
              (STATIC_GYRO_NORM_DPS - STATIC_GYRO_HYSTERESIS);

  g_static_last = (gyro_norm < threshold) ? 1U : 0U;
  return g_static_last;
}



void HAL_I2C_MspInit(I2C_HandleTypeDef *hi2c)
{
  GPIO_InitTypeDef gpio;

  if (hi2c->Instance == I2C2)
  {
    __HAL_RCC_I2C2_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();

    gpio.Pin       = GPIO_PIN_10 | GPIO_PIN_11;
    gpio.Mode      = GPIO_MODE_AF_OD;
    gpio.Pull      = GPIO_PULLUP;
    gpio.Speed     = GPIO_SPEED_FREQ_VERY_HIGH;
    gpio.Alternate = GPIO_AF4_I2C2;
    HAL_GPIO_Init(GPIOB, &gpio);
  }
}


static HAL_StatusTypeDef MPU_WriteReg(uint8_t reg, uint8_t val)
{
  return HAL_I2C_Mem_Write(&g_mpu_i2c,
                           (uint16_t)(MPU6050_I2C_ADDR << 1),
                           reg, I2C_MEMADD_SIZE_8BIT,
                           &val, 1U, 10U);
}

static HAL_StatusTypeDef MPU_ReadRegs(uint8_t reg, uint8_t *buf, uint16_t len)
{
  return HAL_I2C_Mem_Read(&g_mpu_i2c,
                          (uint16_t)(MPU6050_I2C_ADDR << 1),
                          reg, I2C_MEMADD_SIZE_8BIT,
                          buf, len, 10U);
}



void MPU6050_Init(void)
{
  uint32_t  i;
  int64_t   sum_gx_raw;
  int64_t   sum_gy_raw;
  int64_t   sum_gz_raw;
  int64_t   sum_sq_gx_raw;
  int64_t   sum_sq_gy_raw;
  int64_t   sum_sq_gz_raw;
  int64_t   sum_ax_raw;
  int64_t   sum_ay_raw;
  int64_t   sum_az_raw;
  int16_t   raw_ax;
  int16_t   raw_ay;
  int16_t   raw_az;
  int16_t   raw_gx;
  int16_t   raw_gy;
  int16_t   raw_gz;
  uint8_t   buf[MPU6050_FRAME_LEN];
  uint8_t   who;
  float     mean_gx_raw;
  float     mean_gy_raw;
  float     mean_gz_raw;
  float     var_gx_raw;
  float     var_gy_raw;
  float     var_gz_raw;
  float     measure_var_x;
  float     measure_var_y;
  float     measure_var_z;


  g_mpu_i2c.Instance             = I2C2;
  g_mpu_i2c.Init.ClockSpeed      = 400000U;
  g_mpu_i2c.Init.DutyCycle       = I2C_DUTYCYCLE_16_9;
  g_mpu_i2c.Init.OwnAddress1     = 0U;
  g_mpu_i2c.Init.AddressingMode  = I2C_ADDRESSINGMODE_7BIT;
  g_mpu_i2c.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  g_mpu_i2c.Init.OwnAddress2     = 0U;
  g_mpu_i2c.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  g_mpu_i2c.Init.NoStretchMode   = I2C_NOSTRETCH_DISABLE;

  if (HAL_I2C_Init(&g_mpu_i2c) != HAL_OK)
  {
    Error_Handler();
  }

  if (MPU_ReadRegs(MPU6050_REG_WHO_AM_I, &who, 1U) != HAL_OK
      || who != 0x68U)
  {
    Error_Handler();
  }

  if (MPU_WriteReg(MPU6050_REG_PWR_MGMT_1, 0x01U) != HAL_OK)
  {
    Error_Handler();
  }
  HAL_Delay(CALIB_WARMUP_MS);

  
  if (MPU_WriteReg(MPU6050_REG_GYRO_CONFIG, MPU6050_GYRO_FS_SEL) != HAL_OK)
  {
    Error_Handler();
  }


  if (MPU_WriteReg(MPU6050_REG_ACCEL_CONFIG, MPU6050_ACCEL_FS_SEL) != HAL_OK)
  {
    Error_Handler();
  }

  
  if (MPU_WriteReg(MPU6050_REG_CONFIG, MPU6050_DLPF_CFG) != HAL_OK)
  {
    Error_Handler();
  }

  
  if (MPU_WriteReg(MPU6050_REG_SMPLRT_DIV, 0x00U) != HAL_OK)
  {
    Error_Handler();
  }

 
  sum_gx_raw = 0;
  sum_gy_raw = 0;
  sum_gz_raw = 0;
  sum_sq_gx_raw = 0;
  sum_sq_gy_raw = 0;
  sum_sq_gz_raw = 0;
  sum_ax_raw = 0;
  sum_ay_raw = 0;
  sum_az_raw = 0;
  for (i = 0U; i < CALIB_SAMPLES; i++)
  {
    if (MPU_ReadRegs(MPU6050_REG_ACCEL_XOUT_H, buf, MPU6050_FRAME_LEN) == HAL_OK)
    {
      raw_ax = MPU_ParseI16(&buf[0]);
      raw_ay = MPU_ParseI16(&buf[2]);
      raw_az = MPU_ParseI16(&buf[4]);
      raw_gx = MPU_ParseI16(&buf[8]);
      raw_gy = MPU_ParseI16(&buf[10]);
      raw_gz = MPU_ParseI16(&buf[12]);

      sum_ax_raw += raw_ax;
      sum_ay_raw += raw_ay;
      sum_az_raw += raw_az;
      sum_gx_raw += raw_gx;
      sum_gy_raw += raw_gy;
      sum_gz_raw += raw_gz;
      sum_sq_gx_raw += (int64_t)raw_gx * (int64_t)raw_gx;
      sum_sq_gy_raw += (int64_t)raw_gy * (int64_t)raw_gy;
      sum_sq_gz_raw += (int64_t)raw_gz * (int64_t)raw_gz;
    }
    HAL_Delay(CALIB_DELAY_MS);
  }
  g_x_bias_dps = ((float)sum_gx_raw / (float)CALIB_SAMPLES) / MPU6050_GYRO_SENSITIVITY;
  g_y_bias_dps = ((float)sum_gy_raw / (float)CALIB_SAMPLES) / MPU6050_GYRO_SENSITIVITY;
  g_z_bias_dps = ((float)sum_gz_raw / (float)CALIB_SAMPLES) / MPU6050_GYRO_SENSITIVITY;
  g_accel_x_offset_g = ((float)sum_ax_raw / (float)CALIB_SAMPLES) / MPU6050_ACCEL_SENSITIVITY;
  g_accel_y_offset_g = ((float)sum_ay_raw / (float)CALIB_SAMPLES) / MPU6050_ACCEL_SENSITIVITY;

  mean_gx_raw = (float)sum_gx_raw / (float)CALIB_SAMPLES;
  mean_gy_raw = (float)sum_gy_raw / (float)CALIB_SAMPLES;
  mean_gz_raw = (float)sum_gz_raw / (float)CALIB_SAMPLES;
  var_gx_raw = ((float)sum_sq_gx_raw / (float)CALIB_SAMPLES) - (mean_gx_raw * mean_gx_raw);
  var_gy_raw = ((float)sum_sq_gy_raw / (float)CALIB_SAMPLES) - (mean_gy_raw * mean_gy_raw);
  var_gz_raw = ((float)sum_sq_gz_raw / (float)CALIB_SAMPLES) - (mean_gz_raw * mean_gz_raw);
  if (var_gx_raw < 0.0f)
  {
    var_gx_raw = 0.0f;
  }
  if (var_gy_raw < 0.0f)
  {
    var_gy_raw = 0.0f;
  }
  if (var_gz_raw < 0.0f)
  {
    var_gz_raw = 0.0f;
  }
  measure_var_x = var_gx_raw / (MPU6050_GYRO_SENSITIVITY * MPU6050_GYRO_SENSITIVITY);
  measure_var_y = var_gy_raw / (MPU6050_GYRO_SENSITIVITY * MPU6050_GYRO_SENSITIVITY);
  measure_var_z = var_gz_raw / (MPU6050_GYRO_SENSITIVITY * MPU6050_GYRO_SENSITIVITY);
  ImuFilter_GyroInit(&g_gyro_x_filter, g_x_bias_dps, measure_var_x);
  ImuFilter_GyroInit(&g_gyro_y_filter, g_y_bias_dps, measure_var_y);
  ImuFilter_GyroInit(&g_gyro_z_filter, g_z_bias_dps, measure_var_z);
  ImuAhrs_MahonyInitWithAccel(&g_ahrs,
                              g_accel_x_offset_g - g_accel_x_offset_g,
                              g_accel_y_offset_g - g_accel_y_offset_g,
                              ((float)sum_az_raw / (float)CALIB_SAMPLES) / MPU6050_ACCEL_SENSITIVITY);

  
  g_current_yaw  = 0.0f;
  g_current_roll = 0.0f;
  g_current_pitch = 0.0f;
  g_ref_yaw      = 0.0f;
  g_hold_active  = 0U;
  g_corr_filtered = 0.0f;
  g_car_moving   = 0U;
  g_last_car_moving = 0U;
  g_manual_turn_until = 0U;
  g_static_last = 1U;
  g_last_tick    = HAL_GetTick();
  g_ready        = 1U;
}



void MPU6050_Update(void)
{
  uint8_t  buf[MPU6050_FRAME_LEN];
  int16_t  raw_ax;
  int16_t  raw_ay;
  int16_t  raw_az;
  int16_t  raw_gx;
  int16_t  raw_gy;
  int16_t  raw_gz;
  float    ax_g;
  float    ay_g;
  float    az_g;
  float    gx_dps;
  float    gy_dps;
  float    gz_dps;
  float    dt;
  uint32_t now;
  uint8_t  allow_static_bias;
  uint8_t  is_static;

  if (g_ready == 0U)
  {
    return;
  }

  if (MPU_ReadRegs(MPU6050_REG_ACCEL_XOUT_H, buf, MPU6050_FRAME_LEN) != HAL_OK)
  {
    return;
  }
  raw_ax = MPU_ParseI16(&buf[0]);
  raw_ay = MPU_ParseI16(&buf[2]);
  raw_az = MPU_ParseI16(&buf[4]);
  raw_gx = MPU_ParseI16(&buf[8]);
  raw_gy = MPU_ParseI16(&buf[10]);
  raw_gz = MPU_ParseI16(&buf[12]);

  ax_g = ((float)raw_ax / MPU6050_ACCEL_SENSITIVITY) - g_accel_x_offset_g;
  ay_g = ((float)raw_ay / MPU6050_ACCEL_SENSITIVITY) - g_accel_y_offset_g;
  az_g = (float)raw_az / MPU6050_ACCEL_SENSITIVITY;
  gx_dps = (float)raw_gx / MPU6050_GYRO_SENSITIVITY;
  gy_dps = (float)raw_gy / MPU6050_GYRO_SENSITIVITY;
  gz_dps = (float)raw_gz / MPU6050_GYRO_SENSITIVITY;

  now = HAL_GetTick();
  dt  = (float)(now - g_last_tick) / 1000.0f;//鐠侊紕鐣婚弮鍫曟？濮濄儵鏆?
  g_last_tick = now;

  if (dt > 0.1f)   { dt = DT_NOMINAL; }//闂勬劕鍩楅弮鍫曟？濮濄儵鏆?
  if (dt < 0.001f) { dt = DT_NOMINAL; }

  allow_static_bias = ((g_car_moving == 0U) &&
                       ((int32_t)(now - g_manual_turn_until) >= 0)) ? 1U : 0U;

  gx_dps = ImuFilter_GyroUpdate(&g_gyro_x_filter, gx_dps, allow_static_bias);
  gy_dps = ImuFilter_GyroUpdate(&g_gyro_y_filter, gy_dps, allow_static_bias);
  gz_dps = ImuFilter_GyroUpdate(&g_gyro_z_filter, gz_dps, allow_static_bias);

  is_static = ((allow_static_bias != 0U) &&
               (IsStaticByGyro(gx_dps, gy_dps, gz_dps) != 0U)) ? 1U : 0U;

  if (is_static != 0U)
  {
    if (AbsFloat(gx_dps) < STATIONARY_RATE_DPS) { gx_dps = 0.0f; }
    if (AbsFloat(gy_dps) < STATIONARY_RATE_DPS) { gy_dps = 0.0f; }
    if (AbsFloat(gz_dps) < STATIONARY_RATE_DPS) { gz_dps = 0.0f; }
  }

  ImuAhrs_MahonyUpdate(&g_ahrs,
                       ax_g, ay_g, az_g,
                       gx_dps, gy_dps, gz_dps,
                       is_static,
                       dt);
  ImuAhrs_MahonyGetEulerDeg(&g_ahrs,
                            &g_current_roll,
                            &g_current_pitch,
                            &g_current_yaw);
  g_current_yaw = WrapYawDeg(g_current_yaw);
}

/* ------------------------------------------------------------------ */
/*  MPU6050_GetYaw                                                     */
/* ------------------------------------------------------------------ */

float MPU6050_GetYaw(void)
{
  return g_current_yaw;
}

/* ------------------------------------------------------------------ */
/*  MPU6050_NotifyMoving                                               */
/* ------------------------------------------------------------------ */

void MPU6050_NotifyMoving(uint8_t moving)
{
  if ((moving != 0U) && (g_last_car_moving == 0U))
  {
    g_ref_yaw = g_current_yaw;
    g_hold_active = 1U;
    g_corr_filtered = 0.0f;
  }
  else if (moving == 0U)
  {
    g_hold_active = 0U;
    g_corr_filtered = 0.0f;
    g_ref_yaw = g_current_yaw;
  }

  g_car_moving = moving;
  g_last_car_moving = moving;
}

/* ------------------------------------------------------------------ */
/*  MPU6050_GetHeadingCorrection  (P-only heading hold)                */
/* ------------------------------------------------------------------ */

int16_t MPU6050_GetHeadingCorrection(int16_t manual_yaw)
{
  float error;
  float corr;

  if (manual_yaw != 0)
  {
    g_manual_turn_until = HAL_GetTick() + MANUAL_TURN_SETTLE_MS;
    g_hold_active   = 0U;
    g_corr_filtered = 0.0f;
    g_ref_yaw       = g_current_yaw;
    g_last_car_moving = 0U;
    return manual_yaw;
  }

  if (g_car_moving == 0U)
  {
    return 0;
  }

  if (g_hold_active == 0U)
  {
    g_ref_yaw       = g_current_yaw;
    g_hold_active   = 1U;
    g_corr_filtered = 0.0f;
    return 0;
  }

  error = g_ref_yaw - g_current_yaw;
  if (error > 180.0f)  { error -= 360.0f; }
  if (error < -180.0f) { error += 360.0f; }

  if (error > -HEADING_DEADBAND_DEG && error < HEADING_DEADBAND_DEG)
  {
    error = 0.0f;
  }

  corr = error * HEADING_KP;

  if (corr > HEADING_MAX_CORRECTION)
  {
    corr = HEADING_MAX_CORRECTION;
  }
  else if (corr < -HEADING_MAX_CORRECTION)
  {
    corr = -HEADING_MAX_CORRECTION;
  }

  g_corr_filtered = (HEADING_FILTER_ALPHA * corr)
                  + ((1.0f - HEADING_FILTER_ALPHA) * g_corr_filtered);

  return (int16_t)(HEADING_CORRECTION_SIGN * g_corr_filtered);
}
