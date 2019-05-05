//
// Created by HUI Ka Yiu on 2018-12-27.
//

#ifndef PLANE_RC_H
#define PLANE_RC_H

#include "stm32h7xx.h"
#include "stm32h7xx_hal_tim.h"



volatile typedef struct {
    TIM_HandleTypeDef* htim1;
    TIM_HandleTypeDef* htim2;
    uint8_t CaptureIndex[6];
    uint16_t Capture1[6], diffCapture[6];
}UL_RC_typedef;

enum RC_Map{
    RC_THROTTLE = 3,
    RC_YAW = 1,
    RC_PITCH = 0,
    RC_ROLL = 2,
    RC_MODE = 4,
    RC_ONOFF = 5
};

void UL_RC_Init(UL_RC_typedef* RC, TIM_HandleTypeDef* htim1, TIM_HandleTypeDef* htim2);

#endif //PLANE_RC_H
