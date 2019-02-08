//
// Created by HUI Ka Yiu on 2018-12-27.
//

#ifndef PLANE_SERVO_H
#define PLANE_SERVO_H

#include "stm32h7xx.h"
#include "stm32h7xx_hal_dma.h"
#include "stm32h7xx_hal_tim.h"

typedef struct{
    TIM_HandleTypeDef* htim;
    uint32_t channel;
    uint32_t frequency;
    uint32_t min;
    uint32_t max;
}UL_PWM_typedef;

void UL_PWM_Init(UL_PWM_typedef *PWM, TIM_HandleTypeDef* htim, uint32_t channel, uint32_t min, uint32_t max);

HAL_StatusTypeDef UL_PWM_SetMicros(UL_PWM_typedef* PWM, float micros);

HAL_StatusTypeDef UL_PWM_ServoSetDegree(UL_PWM_typedef* Servo, float degrees);


/**
 * @param Prescalar                        PSC
 * @param Auto Reload Value                ARR
 * @param Output Compare Value             CRRx
 * @brief Servo 1.5 ms up time = neutral position
 * @brief Servo pwm frequency 40hz - 200hz, Neutral Position at 1.5ms , 1.0ms - 2.0ms
 * @fn Updatefrequency = TIM clk/((PSC+1)*(ARR+1)) https://eleccelerator.com/avr-timer-calculator/
 */

#endif //PLANE_SERVO_H
