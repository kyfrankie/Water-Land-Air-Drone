//
// Created by HUI Ka Yiu on 2018-12-27.
//

#include "rc.h"

void UL_RC_Init(UL_RC_typedef* RC, TIM_HandleTypeDef* htim){
    RC->htim = htim;
    RC->yaw = 0;
    RC->pitch = 0;
    RC->roll = 0;
    RC->throttle = 0;

    HAL_TIM_IC_Start_IT(htim, TIM_CHANNEL_1);
    HAL_TIM_IC_Start_IT(htim, TIM_CHANNEL_2);
    HAL_TIM_IC_Start_IT(htim, TIM_CHANNEL_3);
    HAL_TIM_IC_Start_IT(htim, TIM_CHANNEL_4);
}

void UL_RC_Get_us(UL_RC_typedef* RC){
    RC->yaw = __HAL_TIM_GET_COMPARE(RC->htim, TIM_CHANNEL_4);
    RC->pitch = __HAL_TIM_GET_COMPARE(RC->htim, TIM_CHANNEL_1);
    RC->roll = __HAL_TIM_GET_COMPARE(RC->htim, TIM_CHANNEL_2);
    RC->throttle = __HAL_TIM_GET_COMPARE(RC->htim, TIM_CHANNEL_3);
}