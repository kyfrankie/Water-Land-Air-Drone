//
// Created by HUI Ka Yiu on 2018-12-27.
//

#include "rc.h"

void UL_RC_Init(UL_RC_typedef* RC, TIM_HandleTypeDef* htim){
    RC->htim = htim;

    HAL_TIM_IC_Start_IT(htim, TIM_CHANNEL_1);
    HAL_TIM_IC_Start_IT(htim, TIM_CHANNEL_2);
    HAL_TIM_IC_Start_IT(htim, TIM_CHANNEL_3);
    HAL_TIM_IC_Start_IT(htim, TIM_CHANNEL_4);
}