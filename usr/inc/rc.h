//
// Created by HUI Ka Yiu on 2018-12-27.
//

#ifndef PLANE_RC_H
#define PLANE_RC_H

#include "stm32h7xx.h"
#include "stm32h7xx_hal_tim.h"



typedef struct {
    TIM_HandleTypeDef* htim;
    uint8_t CaptureIndex[4];
    uint16_t Capture1[4], Capture2[4], diffCapture[4];
}UL_RC_typedef;

enum RC_Map{
    RC_THROTTLE = 3,
    RC_YAW = 1,
    RC_PITCH = 0,
    RC_ROLL = 2
};

void UL_RC_Init(UL_RC_typedef* RC, TIM_HandleTypeDef* htim);

#endif //PLANE_RC_H
