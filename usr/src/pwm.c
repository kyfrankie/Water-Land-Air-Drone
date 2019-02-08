//
// Created by HUI Ka Yiu on 2018-12-27.
//

#include "pwm.h"

void UL_PWM_Init(UL_PWM_typedef* PWM, TIM_HandleTypeDef* htim, uint32_t channel, uint32_t min, uint32_t max){
    PWM->htim = htim;
    PWM->channel = channel;
    PWM->frequency = SystemCoreClock/((htim->Instance->ARR+1)*(htim->Instance->PSC));
    PWM->min = min;
    PWM->max = max;
    HAL_TIM_Base_Start(PWM->htim);
    HAL_TIM_PWM_Start(PWM->htim, PWM->channel);
    UL_PWM_SetMicros(PWM, 0.0015);
}    //! Updatefrequency = TIM clk/((PSC+1)*(ARR+1))

HAL_StatusTypeDef UL_PWM_SetMicros(UL_PWM_typedef* PWM, float micros){
    if (micros < 0 || micros > 1/PWM->frequency)
        return !HAL_OK;
    __HAL_TIM_SET_COMPARE(PWM->htim, PWM->channel, (uint32_t)(PWM->htim->Instance->ARR*micros/PWM->frequency));
}

HAL_StatusTypeDef UL_PWM_ServoSetDegree(UL_PWM_typedef* Servo, float degrees){
    if (degrees < 0 || degrees > 180)
        return !HAL_OK;
    __HAL_TIM_SET_COMPARE(Servo->htim, Servo->channel, (uint32_t)((Servo->max - Servo->min)*degrees/180 + Servo->min)); //! 0 <-> PWM Pulse = html->Instance->CCR1
    return HAL_OK;
}
