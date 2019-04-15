//
// Created by HUI Ka Yiu on 2018-12-27.
//

#ifndef PLANE_RC_H
#define PLANE_RC_H

#include "stm32h7xx.h"
#include "stm32h7xx_hal_tim.h"



typedef struct {
    TIM_HandleTypeDef* htim;
    double yaw, pitch, roll;
    uint32_t throttle;
}UL_RC_typedef;

void UL_RC_Init(UL_RC_typedef* RC, TIM_HandleTypeDef* htim);

void UL_RC_Get_us(UL_RC_typedef* RC);

#endif //PLANE_RC_H
