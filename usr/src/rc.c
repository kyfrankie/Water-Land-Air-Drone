//
// Created by HUI Ka Yiu on 2018-12-27.
//

#include "rc.h"

void UL_RC_Init(UL_RC_typedef* RC, TIM_HandleTypeDef* htim1, TIM_HandleTypeDef* htim2){
    RC->htim1 = htim1;
    RC->htim2 = htim2;

    /*
    RC->diffCapture[RC_PITCH] = 1500;
    RC->diffCapture[RC_ROLL] = 1500;
    RC->diffCapture[RC_YAW] = 1500;
    RC->diffCapture[RC_THROTTLE] = 1800;
    RC->diffCapture[RC_MODE] = 1500;
     */

    HAL_TIM_IC_Start_IT(RC->htim1, TIM_CHANNEL_1);
    HAL_TIM_IC_Start_IT(RC->htim1, TIM_CHANNEL_2);
    HAL_TIM_IC_Start_IT(RC->htim1, TIM_CHANNEL_3);
    HAL_TIM_IC_Start_IT(RC->htim1, TIM_CHANNEL_4);

    HAL_TIM_IC_Start_IT(RC->htim2, TIM_CHANNEL_1);
    //HAL_TIM_IC_Start_IT(RC->htim2, TIM_CHANNEL_2);
}