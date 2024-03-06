/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
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
#include "stm32f4xx_hal.h"

#include "stm32f4xx_ll_adc.h"
#include "stm32f4xx_ll_i2c.h"
#include "stm32f4xx_ll_rcc.h"
#include "stm32f4xx_ll_bus.h"
#include "stm32f4xx_ll_system.h"
#include "stm32f4xx_ll_exti.h"
#include "stm32f4xx_ll_cortex.h"
#include "stm32f4xx_ll_utils.h"
#include "stm32f4xx_ll_pwr.h"
#include "stm32f4xx_ll_dma.h"
#include "stm32f4xx_ll_tim.h"
#include "stm32f4xx_ll_usart.h"
#include "stm32f4xx_ll_gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

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

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

//#define BLDC
//#define IBUS
//#define SBUS


/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define USER_BUTTON_Pin LL_GPIO_PIN_13
#define USER_BUTTON_GPIO_Port GPIOC
#define ENCODERR_A_Pin LL_GPIO_PIN_0
#define ENCODERR_A_GPIO_Port GPIOA
#define ENCODERR_B_Pin LL_GPIO_PIN_1
#define ENCODERR_B_GPIO_Port GPIOA
#define ENCODER3_PWM_Pin LL_GPIO_PIN_3
#define ENCODER3_PWM_GPIO_Port GPIOA
#define ADC1_I_MOT_3_Pin LL_GPIO_PIN_4
#define ADC1_I_MOT_3_GPIO_Port GPIOA
#define ADC1_I_MOT_R_Pin LL_GPIO_PIN_5
#define ADC1_I_MOT_R_GPIO_Port GPIOA
#define ADC1_I_MOT_L_Pin LL_GPIO_PIN_6
#define ADC1_I_MOT_L_GPIO_Port GPIOA
#define ADC1_V_POWER_Pin LL_GPIO_PIN_7
#define ADC1_V_POWER_GPIO_Port GPIOA
#define MOT_OUT1_R_Pin LL_GPIO_PIN_0
#define MOT_OUT1_R_GPIO_Port GPIOB
#define MOT_OUT2_R_Pin LL_GPIO_PIN_1
#define MOT_OUT2_R_GPIO_Port GPIOB
#define EXTI_PPM_Pin LL_GPIO_PIN_12
#define EXTI_PPM_GPIO_Port GPIOB
#define GPIO_PRZEKA_NIK_Pin LL_GPIO_PIN_13
#define GPIO_PRZEKA_NIK_GPIO_Port GPIOB
#define GPIO_KLUCZYK_Pin LL_GPIO_PIN_14
#define GPIO_KLUCZYK_GPIO_Port GPIOB
#define GPIO_GRZYB_IN_Pin LL_GPIO_PIN_15
#define GPIO_GRZYB_IN_GPIO_Port GPIOB
#define ENCODERL_A_Pin LL_GPIO_PIN_8
#define ENCODERL_A_GPIO_Port GPIOA
#define ENCODERL_B_Pin LL_GPIO_PIN_9
#define ENCODERL_B_GPIO_Port GPIOA
#define GPIO_LIDAR_Pin LL_GPIO_PIN_10
#define GPIO_LIDAR_GPIO_Port GPIOA
#define USB_OTG_FS_FM_Pin LL_GPIO_PIN_11
#define USB_OTG_FS_FM_GPIO_Port GPIOA
#define STATUS_LED_Pin LL_GPIO_PIN_15
#define STATUS_LED_GPIO_Port GPIOA
#define MOT_OUT1_L_Pin LL_GPIO_PIN_4
#define MOT_OUT1_L_GPIO_Port GPIOB
#define MOT_OUT2_L_Pin LL_GPIO_PIN_5
#define MOT_OUT2_L_GPIO_Port GPIOB
#define MOT_OUT1_3_Pin LL_GPIO_PIN_8
#define MOT_OUT1_3_GPIO_Port GPIOB
#define MOT_OUT2_3_Pin LL_GPIO_PIN_9
#define MOT_OUT2_3_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
