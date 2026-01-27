#include "stm32f10x.h"
#include "MyCAN.h"
#include "MPU6050.h"
#include "MyI2C.h"
#include "Usart.h"

int main(void)
{
    int16_t accel[3], gyro[3];
    float target_voltage = 5.0f;

    System_Init();
    USART1_Config();
    CAN_Config();
    MyI2C_Init();

    if (!MPU6050_Init())
    {
        while (1);
    }

    while (1)
    {
        MPU6050_ReadData(accel, gyro);

        for (int i = 0; i < 3; i++)
            USART_SendFloat((float)accel[i] / 16384.0f);

        for (int i = 0; i < 3; i++)
            USART_SendFloat((float)gyro[i] / 131.0f);

        USART_SendFloat(0.0f);
        USART_SendVOFA_Tail();

        CAN_SendVoltage(target_voltage);

        for (volatile int i = 0; i < 1000000; i++);
    }
}
