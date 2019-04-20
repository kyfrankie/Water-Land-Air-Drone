//
// Created by HUI Ka Yiu on 2018-12-27.
//

#include "pwm.h"

void UL_PWM_Init(UL_PWM_typedef* PWM, TIM_HandleTypeDef* htim, uint32_t channel, uint32_t period){
    PWM->htim = htim;
    PWM->channel = channel;
    PWM->period = period;
    HAL_TIM_PWM_Start(PWM->htim, PWM->channel);
}    //! Updatefrequency = TIM clk/((PSC+1)*(ARR+1))

void UL_PWM_SetUs(UL_PWM_typedef* PWM, uint32_t us){
    __HAL_TIM_SET_COMPARE(PWM->htim, PWM->channel, (uint32_t)((PWM->htim->Instance->ARR-1) * (double)(us) / (double)(PWM->period)));
}