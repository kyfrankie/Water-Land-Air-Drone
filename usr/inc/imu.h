//
// Created by HUI Ka Yiu on 2018-12-27.
//

#ifndef PLANE_IMU_H
#define PLANE_IMU_H

#include "stm32h7xx.h"
#include "stm32h7xx_hal_uart.h"
#include "tft.h"
#include "imu_defines.h"

typedef struct {
    UART_HandleTypeDef* huart;
    uint8_t rxbuff[55];
    double angle_x, angle_y, angle_z, accel_x, accel_y, accel_z, angularV_x, angularV_y, angularV_z,
             magnet_x, magnet_y, magnet_z;
    int32_t altitude;
}UL_IMU_typedef;

void UL_IMU_Init(UL_IMU_typedef* IMU, UART_HandleTypeDef* huart);
void UL_IMU_SetUp(UL_IMU_typedef* IMU);
void UL_IMU_Read(UL_IMU_typedef* IMU);
void UL_IMU_Write(UL_IMU_typedef* IMU, uint8_t cmd, uint8_t option1, uint8_t option2);
void UL_IMU_Displace(UL_IMU_typedef* IMU);

#endif //PLANE_IMU_H
