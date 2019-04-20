//
// Created by HUI Ka Yiu on 2018-12-27.
//

#ifndef PLANE_RC_H
#define PLANE_RC_H

#include "stm32h7xx.h"
#include "stm32h7xx_hal_tim.h"



typedef struct {
    TIM_HandleTypeDef* htim;
    uint32_t yaw, pitch, roll;
    uint32_t throttle;
    uint8_t CaptureIndex[4];
    uint16_t Capture1[4], Capture2[4], diffCapture[4];
}UL_RC_typedef;

void UL_RC_Init(UL_RC_typedef* RC, TIM_HandleTypeDef* htim);

void UL_RC_Get_us(UL_RC_typedef* RC);

#endif //PLANE_RC_H
