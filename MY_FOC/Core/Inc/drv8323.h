#include "main.h"

#ifndef __DRV8323_H__
#define __DRV8323_H__

typedef struct {
    uint16_t writeback_ojbk;
    uint16_t fault1; //0x00房
    uint16_t fault2; //0x01房
    uint16_t nfault; //PB7 报警电平，1 = 平安

    uint16_t reg02;
    uint16_t reg03;
    uint16_t reg04;
    uint16_t reg05;
    uint16_t reg06;
    uint16_t enable;
} drv_status_t;


extern drv_status_t drv_nb;

void drv8323_Init(void);

/* First nFAULT only; values remain in RAM after ENABLE is switched off. */
typedef enum {
    DRV_FAULT_SNAPSHOT_IDLE = 0,
    DRV_FAULT_SNAPSHOT_PENDING,
    DRV_FAULT_SNAPSHOT_READING,
    DRV_FAULT_SNAPSHOT_VALID,
    DRV_FAULT_SNAPSHOT_SPI_ERROR,
    DRV_FAULT_SNAPSHOT_TIMEOUT,
    DRV_FAULT_SNAPSHOT_ABORTED
} drv_fault_snapshot_state_t;

typedef struct {
    uint32_t state;          /* 0 idle, 1 pending, 2 reading, 3 valid, 4/5/6 failed */
    uint32_t time_ms;        /* First detection time, not the physical edge time. */
    uint32_t elapsed_ms;
    uint32_t startup_stage;  /* 0 before PWM, 1 starting, 2 started, 3 main loop */
    uint32_t valid_mask;     /* bit 0: fault1 read; bit 1: fault2 read */
    uint32_t spi_status;     /* HAL status; 0xFFFF means not attempted */
    uint16_t fault1;         /* DRV8323 address 0x00; 0xFFFF until read */
    uint16_t fault2;         /* DRV8323 address 0x01; 0xFFFF until read */
    uint32_t nfault;         /* PB7 level at detection */
    uint32_t enable;         /* PB5 output at detection */
    uint32_t tim1_cnt;
    uint32_t tim1_ccer;
    uint32_t tim1_ccr[4];
} drv_fault_snapshot_t;

extern volatile drv_fault_snapshot_t drv_fault_snapshot;
void drv8323_FaultSnapshotRequest(uint32_t startup_stage);
uint8_t drv8323_FaultSnapshotPending(void);
void drv8323_FaultSnapshotTask(void);
void drv8323_FaultSnapshotAbort(void);


#endif
