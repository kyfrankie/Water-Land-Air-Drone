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
    UL_IMU_Write(IMU, RSW, 0x5E, 0x00);         //feedback data package
    UL_IMU_Write(IMU, RRATE, 0x06, 0x00);       //feedback rate
    UL_IMU_Write(IMU, BAUD, 0x06, 0x00);        //BAUD rate
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
                    IMU->roll = (short)(IMU->rxbuff[i+3] << 8 | IMU->rxbuff[i+2]) / 32768.0*180.0;
                    IMU->pitch = (short)(IMU->rxbuff[i+5] << 8 | IMU->rxbuff[i+4]) / 32768.0*180.0;
                    IMU->yaw = (short)(IMU->rxbuff[i+7] << 8 | IMU->rxbuff[i+6]) / 32768.0*180.0;
                    i += 10;
                    break;
                case 0x54 :
                    IMU->magnet_x = (short)(IMU->rxbuff[i+3] << 8 | IMU->rxbuff[i+2]);
                    IMU->magnet_y = (short)(IMU->rxbuff[i+5] << 8 | IMU->rxbuff[i+4]);
                    IMU->magnet_z = (short)(IMU->rxbuff[i+7] << 8 | IMU->rxbuff[i+6]);
                    i += 10;
                    break;
                case 0x56 :
                    IMU->altitude = IMU->rxbuff[i+9] << 24 | IMU->rxbuff[i+8] << 16 | IMU->rxbuff[i+7] << 8 | IMU->rxbuff[i+6];
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

    UL_TFT_ST7735_WriteString(&Tft, 1, 1, "Angle", FONT_7x10, ST7735_WHITE, ST7735_BLACK);
    UL_TFT_ST7735_WriteNumber(&Tft, 45, 1, abs((int)(IMU->roll)), FONT_7x10, ST7735_BLUE, ST7735_BLACK);
    UL_TFT_ST7735_WriteNumber(&Tft, 75, 1, abs((int)(IMU->pitch)), FONT_7x10, ST7735_BLUE, ST7735_BLACK);
    UL_TFT_ST7735_WriteNumber(&Tft, 105, 1, abs((int)(IMU->yaw)), FONT_7x10, ST7735_BLUE, ST7735_BLACK);

    UL_TFT_ST7735_WriteString(&Tft, 1, 15, "Accel", FONT_7x10, ST7735_WHITE, ST7735_BLACK);
    UL_TFT_ST7735_WriteNumber(&Tft, 45, 15, abs((int)(IMU->accel_x)), FONT_7x10, ST7735_BLUE, ST7735_BLACK);
    UL_TFT_ST7735_WriteNumber(&Tft, 75, 15, abs((int)(IMU->accel_y)), FONT_7x10, ST7735_BLUE, ST7735_BLACK);
    UL_TFT_ST7735_WriteNumber(&Tft, 105, 15, abs((int)(IMU->accel_z)), FONT_7x10, ST7735_BLUE, ST7735_BLACK);

    UL_TFT_ST7735_WriteString(&Tft, 1, 30, "AnguV", FONT_7x10, ST7735_WHITE, ST7735_BLACK);
    UL_TFT_ST7735_WriteNumber(&Tft, 45, 30, abs((int)(IMU->angularV_x)), FONT_7x10, ST7735_BLUE, ST7735_BLACK);
    UL_TFT_ST7735_WriteNumber(&Tft, 75, 30, abs((int)(IMU->angularV_y)), FONT_7x10, ST7735_BLUE, ST7735_BLACK);
    UL_TFT_ST7735_WriteNumber(&Tft, 105, 30, abs((int)(IMU->angularV_z)), FONT_7x10, ST7735_BLUE, ST7735_BLACK);
}