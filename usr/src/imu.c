//
// Created by HUI Ka Yiu on 2018-12-27.
//

#include "imu.h"
#include "main.h"
#include "stdlib.h"

void UL_IMU_Init(UL_IMU_typedef* IMU, UART_HandleTypeDef* huart){
    IMU->huart = huart;
    HAL_UART_Receive_IT(IMU->huart, (uint8_t *)IMU->rxbuff, sizeof(IMU->rxbuff));
    __HAL_UART_DISABLE(IMU->huart);
}

void UL_IMU_SetUp(UL_IMU_typedef* IMU){
    //UL_IMU_Write(IMU, RSW, 0xDE, 0x00);         //feedback data package
    UL_IMU_Write(IMU, RRATE, 0x06, 0x0b);       //feedback rate
    //UL_IMU_Write(IMU, BAUD, 0x06, 0x00);        //BAUD rate
    UL_IMU_Write(IMU, SAVE, 0x00, 0x00);        //Save
}

void UL_IMU_Read(UL_IMU_typedef* IMU){
    for (uint8_t i = 0; i < 55; i++){
        if (IMU->rxbuff[i] == 0x55){
            switch (IMU->rxbuff[i+1]){
                case 0x51 :
                    IMU->accel_x = (short)(IMU->rxbuff[i+3] << 8 | IMU->rxbuff[i+2]) / 32768.0*16.0*9.8;
                    IMU->accel_y = (short)(IMU->rxbuff[i+5] << 8 | IMU->rxbuff[i+4]) / 32768.0*16.0*9.8;
                    IMU->accel_z = (short)(IMU->rxbuff[i+7] << 8 | IMU->rxbuff[i+6]) / 32768.0*16.0*9.8;
                    i += 10;
                    break;
                case 0x52 :
                    IMU->angularV_x = (short)(IMU->rxbuff[i+3] << 8 | IMU->rxbuff[i+2]) / 32768.0*2000.0;
                    IMU->angularV_y = (short)(IMU->rxbuff[i+5] << 8 | IMU->rxbuff[i+4]) / 32768.0*2000.0;
                    IMU->angularV_z = (short)(IMU->rxbuff[i+7] << 8 | IMU->rxbuff[i+6]) / 32768.0*2000.0;
                    i += 10;
                    break;
                case 0x53 :
                    IMU->pitch = (short)(IMU->rxbuff[i+3] << 8 | IMU->rxbuff[i+2]) / 32768.0*180.0 - IMU->offset_pitch;
                    IMU->roll = (short)(IMU->rxbuff[i+5] << 8 | IMU->rxbuff[i+4]) / 32768.0*180.0 - IMU->offset_roll;
                    IMU->yaw = (short)(IMU->rxbuff[i+7] << 8 | IMU->rxbuff[i+6]) / 32768.0*180.0;
                    if (IMU->yaw < 0.0)
                        IMU->yaw *= -1.0;
                    i += 10;
                    break;
                case 0x54 :
                    IMU->magnet_x = (short)(IMU->rxbuff[i+3] << 8 | IMU->rxbuff[i+2]);
                    IMU->magnet_y = (short)(IMU->rxbuff[i+5] << 8 | IMU->rxbuff[i+4]);
                    IMU->magnet_z = (short)(IMU->rxbuff[i+7] << 8 | IMU->rxbuff[i+6]) - IMU->offset_accel_z;
                    i += 10;
                    break;
                case 0x56 :
                    IMU->altitude = IMU->rxbuff[i+9] << 24 | IMU->rxbuff[i+8] << 16 | IMU->rxbuff[i+7] << 8 | IMU->rxbuff[i+6];
                    i += 10;
                    break;
                case 0x57:

                default:
                    break;
            }
        }
    }
}

void UL_IMU_CAL(UL_IMU_typedef* IMU){
    IMU->offset_pitch = IMU->pitch;
    IMU->offset_roll = IMU->roll;
    IMU->offset_accel_z = IMU->accel_z;
}

void UL_IMU_Write(UL_IMU_typedef* IMU, uint8_t cmd, uint8_t option1, uint8_t option2){
    uint8_t data[5] = {0xFF, 0xAA, cmd, option1, option2};
    HAL_UART_Transmit(IMU->huart, data, sizeof(data), HAL_MAX_DELAY);
    HAL_Delay(100);
}