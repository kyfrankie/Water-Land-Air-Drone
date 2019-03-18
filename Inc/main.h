/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2019 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32h7xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "tft.h"
#include "pwm.h"
#include "rc.h"
#include "imu.h"
#include "gps.h"
#include "camera.h"
#include "brushless.h"
//#include "flight_control.h"
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define USER_Btn_Pin GPIO_PIN_13
#define USER_Btn_GPIO_Port GPIOC
#define TFT_RST_Pin GPIO_PIN_0
#define TFT_RST_GPIO_Port GPIOF
#define TFT_CS_Pin GPIO_PIN_1
#define TFT_CS_GPIO_Port GPIOF
#define TFT_DC_Pin GPIO_PIN_2
#define TFT_DC_GPIO_Port GPIOF
#define SD_CS_Pin GPIO_PIN_3
#define SD_CS_GPIO_Port GPIOF
#define Camera_OE_Pin GPIO_PIN_4
#define Camera_OE_GPIO_Port GPIOF
#define Camera_PRST_Pin GPIO_PIN_5
#define Camera_PRST_GPIO_Port GPIOF
#define Bluetooth_RX_Pin GPIO_PIN_6
#define Bluetooth_RX_GPIO_Port GPIOF
#define Bluetooth_TX_Pin GPIO_PIN_7
#define Bluetooth_TX_GPIO_Port GPIOF
#define MCO_Pin GPIO_PIN_0
#define MCO_GPIO_Port GPIOH
#define Laser1_TX_Pin GPIO_PIN_0
#define Laser1_TX_GPIO_Port GPIOA
#define Servo1_Pin GPIO_PIN_1
#define Servo1_GPIO_Port GPIOA
#define Servo2_Pin GPIO_PIN_2
#define Servo2_GPIO_Port GPIOA
#define Servo3_Pin GPIO_PIN_3
#define Servo3_GPIO_Port GPIOA
#define Servo4_Pin GPIO_PIN_5
#define Servo4_GPIO_Port GPIOA
#define RC1_Pin GPIO_PIN_0
#define RC1_GPIO_Port GPIOB
#define RC2_Pin GPIO_PIN_1
#define RC2_GPIO_Port GPIOB
#define Brushless1_Pin GPIO_PIN_9
#define Brushless1_GPIO_Port GPIOE
#define Brushless2_Pin GPIO_PIN_11
#define Brushless2_GPIO_Port GPIOE
#define Brushless3_Pin GPIO_PIN_13
#define Brushless3_GPIO_Port GPIOE
#define Brushless4_Pin GPIO_PIN_14
#define Brushless4_GPIO_Port GPIOE
#define Laser2_RX_Pin GPIO_PIN_12
#define Laser2_RX_GPIO_Port GPIOB
#define Laser2_TX_Pin GPIO_PIN_13
#define Laser2_TX_GPIO_Port GPIOB
#define LD3_Pin GPIO_PIN_14
#define LD3_GPIO_Port GPIOB
#define IMU_RX_Pin GPIO_PIN_15
#define IMU_RX_GPIO_Port GPIOB
#define STLK_RX_Pin GPIO_PIN_8
#define STLK_RX_GPIO_Port GPIOD
#define STLK_TX_Pin GPIO_PIN_9
#define STLK_TX_GPIO_Port GPIOD
#define Servo5_Pin GPIO_PIN_12
#define Servo5_GPIO_Port GPIOD
#define PWM1_Pin GPIO_PIN_13
#define PWM1_GPIO_Port GPIOD
#define PWM2_Pin GPIO_PIN_14
#define PWM2_GPIO_Port GPIOD
#define PWM3_Pin GPIO_PIN_15
#define PWM3_GPIO_Port GPIOD
#define USB_PowerSwitchOn_Pin GPIO_PIN_6
#define USB_PowerSwitchOn_GPIO_Port GPIOG
#define USB_OverCurrent_Pin GPIO_PIN_7
#define USB_OverCurrent_GPIO_Port GPIOG
#define USB_SOF_Pin GPIO_PIN_8
#define USB_SOF_GPIO_Port GPIOA
#define USB_VBUS_Pin GPIO_PIN_9
#define USB_VBUS_GPIO_Port GPIOA
#define USB_DM_Pin GPIO_PIN_11
#define USB_DM_GPIO_Port GPIOA
#define USB_DP_Pin GPIO_PIN_12
#define USB_DP_GPIO_Port GPIOA
#define TMS_Pin GPIO_PIN_13
#define TMS_GPIO_Port GPIOA
#define TCK_Pin GPIO_PIN_14
#define TCK_GPIO_Port GPIOA
#define Laser1_RX_Pin GPIO_PIN_11
#define Laser1_RX_GPIO_Port GPIOC
#define RC3_Pin GPIO_PIN_4
#define RC3_GPIO_Port GPIOB
#define RC4_Pin GPIO_PIN_5
#define RC4_GPIO_Port GPIOB
#define IMU_TX_Pin GPIO_PIN_6
#define IMU_TX_GPIO_Port GPIOB
#define LD2_Pin GPIO_PIN_7
#define LD2_GPIO_Port GPIOB
#define Camera_SIO_C_Pin GPIO_PIN_8
#define Camera_SIO_C_GPIO_Port GPIOB
#define Camera_SIO_D_Pin GPIO_PIN_9
#define Camera_SIO_D_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
