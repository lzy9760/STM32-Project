
#ifndef HOST_MAIN_H
#define HOST_MAIN_H
#include <stdint.h>
typedef struct { uint32_t ccr[3]; } TIM_HandleTypeDef;
typedef struct { volatile uint32_t BDTR; } HostTIM;
typedef struct { volatile uint32_t CYCCNT; } HostDWT;
typedef struct { void *Instance; } ADC_HandleTypeDef;
typedef struct { uint32_t TypeErase, Banks, Sector, NbSectors, VoltageRange; } FLASH_EraseInitTypeDef;
typedef enum { HAL_OK=0, HAL_ERROR=1 } HAL_StatusTypeDef;
extern HostTIM host_tim;
extern HostDWT host_dwt;
extern uint32_t SystemCoreClock;
#define TIM1 (&host_tim)
#define DWT (&host_dwt)
#define TIM_BDTR_MOE 1U
#define TIM_BDTR_OSSI 2U
#define TIM_CHANNEL_1 0U
#define TIM_CHANNEL_2 1U
#define TIM_CHANNEL_3 2U
#define __HAL_TIM_SET_COMPARE(t,c,v) ((t)->ccr[c]=(v))
#define GPIOB ((void *)1)
#define GPIO_PIN_7 128U
#define RESET 0U
#define ADC1 ((void *)3)
#define ADC_INJECTED_RANK_1 1U
#define ADC_INJECTED_RANK_2 2U
#define ADC_INJECTED_RANK_3 3U
#define FLASH_TYPEERASE_SECTORS 1U
#define FLASH_SECTOR_11 11U
#define FLASH_VOLTAGE_RANGE_3 3U
#define FLASH_TYPEPROGRAM_WORD 2U
#define FLASH_FLAG_EOP 1U
#define FLASH_FLAG_OPERR 2U
#define FLASH_FLAG_WRPERR 4U
#define FLASH_FLAG_PGAERR 8U
#define FLASH_FLAG_PGPERR 16U
#define FLASH_FLAG_PGSERR 32U
#define __HAL_FLASH_CLEAR_FLAG(x) ((void)(x))
static inline uint32_t __get_PRIMASK(void) { return 0U; }
void host_disable_irq(void);
static inline void __disable_irq(void) { host_disable_irq(); }
static inline void __set_PRIMASK(uint32_t x) { (void)x; }
static inline void __DSB(void) {}
uint32_t host_flash_read(uint32_t address);
#define FOC_FLASH_READ(a) host_flash_read(a)
uint32_t HAL_GetTick(void);
uint32_t host_exti(void);
#define __HAL_GPIO_EXTI_GET_IT(pin) host_exti()
int HAL_GPIO_ReadPin(void *port, uint16_t pin);
void Motor_SafetyStop(uint32_t reason);
HAL_StatusTypeDef HAL_FLASH_Unlock(void);
HAL_StatusTypeDef HAL_FLASH_Lock(void);
void FLASH_FlushCaches(void);
HAL_StatusTypeDef HAL_FLASHEx_Erase(FLASH_EraseInitTypeDef *erase, uint32_t *error);
HAL_StatusTypeDef HAL_FLASH_Program(uint32_t type, uint32_t address, uint64_t data);
uint32_t HAL_ADCEx_InjectedGetValue(ADC_HandleTypeDef *adc, uint32_t rank);
void HAL_ADCEx_InjectedConvCpltCallback(ADC_HandleTypeDef *adc);
void host_store_service(void);
#endif
