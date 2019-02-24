//
// Created by HUI Ka Yiu on 2018-12-27.
//

#include "imu.h"

void UL_IMU_Init(UL_IMU_typedef* IMU, UART_HandleTypeDef* huart){
    IMU->huart = huart;
    //UL_IMU_Read(IMU);
}

void UL_IMU_SetUp(UL_IMU_typedef* IMU){
    UL_IMU_Write(IMU, RSW, 0x5E, 0x00);         //feedback data package
    UL_IMU_Write(IMU, RRATE, 0x06, 0x00);       //feedback rate
    UL_IMU_Write(IMU, BAUD, 0x06, 0x00);        //BAUD rate
    UL_IMU_Write(IMU, SAVE, 0x00, 0x00);        //Save
}

void UL_IMU_Read(UL_IMU_typedef* IMU){
    uint8_t rxbuff[55];
    HAL_UART_Receive(IMU->huart, rxbuff, 55, HAL_MAX_DELAY);
    for (uint8_t i = 0; i < 55; i++){
        if (rxbuff[i] == 0x55){
            switch (rxbuff[i+1]){
                case 0x51 :
                    IMU->accel_x = (short)(rxbuff[i+3] << 8 | rxbuff[i+2]) / 32768.0*16.0*9.8;
                    IMU->accel_y = (short)(rxbuff[i+5] << 8 | rxbuff[i+4]) / 32768.0*16.0*9.8;
                    IMU->accel_z = (short)(rxbuff[i+7] << 8 | rxbuff[i+6]) / 32768.0*16.0*9.8;
                    i += 10;
                    break;
                case 0x52 :
                    IMU->angularV_x = (short)(rxbuff[i+3] << 8 | rxbuff[i+2]) / 32768.0*2000.0;
                    IMU->angularV_y = (short)(rxbuff[i+5] << 8 | rxbuff[i+4]) / 32768.0*2000.0;
                    IMU->angularV_z = (short)(rxbuff[i+7] << 8 | rxbuff[i+6]) / 32768.0*2000.0;
                    i += 10;
                    break;
                case 0x53 :
                    IMU->angle_x = (short)(rxbuff[i+3] << 8 | rxbuff[i+2]) / 32768.0*180.0;
                    IMU->angle_y = (short)(rxbuff[i+5] << 8 | rxbuff[i+4]) / 32768.0*180.0;
                    IMU->angle_z = (short)(rxbuff[i+7] << 8 | rxbuff[i+6]) / 32768.0*180.0;
                    i += 10;
                    break;
                case 0x54 :
                    IMU->magnet_x = (short)(rxbuff[i+3] << 8 | rxbuff[i+2]);
                    IMU->magnet_y = (short)(rxbuff[i+5] << 8 | rxbuff[i+4]);
                    IMU->magnet_z = (short)(rxbuff[i+7] << 8 | rxbuff[i+6]);
                    i += 10;
                    break;
                case 0x56 :
                    IMU->altitude = rxbuff[i+9] << 24 | rxbuff[i+8] << 16 | rxbuff[i+7] << 8 | rxbuff[i+6];
                    i += 10;
                    break;
                default:
                    break;
            }
        }
    }
}

void UL_IMU_Write(UL_IMU_typedef* IMU, uint8_t cmd, uint8_t option1, uint8_t option2){
    uint8_t data[5] = {0xFF, 0xAA, cmd, option1, option2};
    HAL_UART_Transmit(IMU->huart, data, sizeof(data), HAL_MAX_DELAY);
    HAL_Delay(100);
}

void UL_IMU_Displace(UL_IMU_typedef* IMU){
    extern UL_TFT_typedef Tft;

    UL_TFT_ST7735_WriteString(&Tft, 1, 1, "Angle", Font_7x10, ST7735_WHITE, ST7735_BLACK);
    UL_TFT_ST7735_WriteNumber(&Tft, 40, 1, IMU->angle_x, Font_7x10, ST7735_BLUE, ST7735_BLACK);
    UL_TFT_ST7735_WriteNumber(&Tft, 70, 1, IMU->angle_y, Font_7x10, ST7735_BLUE, ST7735_BLACK);
    UL_TFT_ST7735_WriteNumber(&Tft, 100, 1, IMU->angle_z, Font_7x10, ST7735_BLUE, ST7735_BLACK);

    UL_TFT_ST7735_WriteString(&Tft, 1, 15, "Accel", Font_7x10, ST7735_WHITE, ST7735_BLACK);
    UL_TFT_ST7735_WriteNumber(&Tft, 40, 15, IMU->accel_x, Font_7x10, ST7735_BLUE, ST7735_BLACK);
    UL_TFT_ST7735_WriteNumber(&Tft, 70, 15, IMU->accel_y, Font_7x10, ST7735_BLUE, ST7735_BLACK);
    UL_TFT_ST7735_WriteNumber(&Tft, 100, 15, IMU->accel_z, Font_7x10, ST7735_BLUE, ST7735_BLACK);

    UL_TFT_ST7735_WriteString(&Tft, 1, 30, "AnguV", Font_7x10, ST7735_WHITE, ST7735_BLACK);
    UL_TFT_ST7735_WriteNumber(&Tft, 40, 30, IMU->angularV_x, Font_7x10, ST7735_BLUE, ST7735_BLACK);
    UL_TFT_ST7735_WriteNumber(&Tft, 70, 30, IMU->angularV_y, Font_7x10, ST7735_BLUE, ST7735_BLACK);
    UL_TFT_ST7735_WriteNumber(&Tft, 100, 30, IMU->angularV_z, Font_7x10, ST7735_BLUE, ST7735_BLACK);
}