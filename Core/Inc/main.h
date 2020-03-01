/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
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
#include "stm32g4xx_hal.h"
#include "stm32g4xx_ll_iwdg.h"
#include "stm32g4xx_ll_bus.h"
#include "stm32g4xx_ll_cortex.h"
#include "stm32g4xx_ll_rcc.h"
#include "stm32g4xx_ll_system.h"
#include "stm32g4xx_ll_utils.h"
#include "stm32g4xx_ll_pwr.h"
#include "stm32g4xx_ll_gpio.h"
#include "stm32g4xx_ll_dma.h"

#include "stm32g4xx_ll_exti.h"

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

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define LEDBLUE_Pin GPIO_PIN_4
#define LEDBLUE_GPIO_Port GPIOE
#define SDA_Pin GPIO_PIN_9
#define SDA_GPIO_Port GPIOB
#define SCL_Pin GPIO_PIN_8
#define SCL_GPIO_Port GPIOB
#define TXMODE_Pin GPIO_PIN_6
#define TXMODE_GPIO_Port GPIOB
#define CANRX_Pin GPIO_PIN_3
#define CANRX_GPIO_Port GPIOB
#define PWM1_Pin GPIO_PIN_6
#define PWM1_GPIO_Port GPIOD
#define PWM3_Pin GPIO_PIN_4
#define PWM3_GPIO_Port GPIOD
#define PWM4_Pin GPIO_PIN_12
#define PWM4_GPIO_Port GPIOC
#define LEDGREEN_Pin GPIO_PIN_5
#define LEDGREEN_GPIO_Port GPIOE
#define LEDRED_Pin GPIO_PIN_3
#define LEDRED_GPIO_Port GPIOE
#define PWM2_Pin GPIO_PIN_7
#define PWM2_GPIO_Port GPIOD
#define PWM5_Pin GPIO_PIN_15
#define PWM5_GPIO_Port GPIOA
#define CANEN_Pin GPIO_PIN_6
#define CANEN_GPIO_Port GPIOE
#define CANTX_Pin GPIO_PIN_4
#define CANTX_GPIO_Port GPIOB
#define GPSRX_Pin GPIO_PIN_10
#define GPSRX_GPIO_Port GPIOA
#define GPSTX_Pin GPIO_PIN_9
#define GPSTX_GPIO_Port GPIOA
#define PWM6_Pin GPIO_PIN_9
#define PWM6_GPIO_Port GPIOF
#define TXORRX_Pin GPIO_PIN_9
#define TXORRX_GPIO_Port GPIOC
#define SYNC_Pin GPIO_PIN_8
#define SYNC_GPIO_Port GPIOA
#define PWMOE_Pin GPIO_PIN_7
#define PWMOE_GPIO_Port GPIOC
#define RF2RXEN_Pin GPIO_PIN_3
#define RF2RXEN_GPIO_Port GPIOC
#define PWM7_Pin GPIO_PIN_0
#define PWM7_GPIO_Port GPIOA
#define BMPINT_Pin GPIO_PIN_9
#define BMPINT_GPIO_Port GPIOD
#define BMPINT_EXTI_IRQn EXTI9_5_IRQn
#define RF1TXEN_Pin GPIO_PIN_15
#define RF1TXEN_GPIO_Port GPIOD
#define PWM8_Pin GPIO_PIN_2
#define PWM8_GPIO_Port GPIOA
#define RF2NSS_Pin GPIO_PIN_4
#define RF2NSS_GPIO_Port GPIOA
#define RFPOWEREN_Pin GPIO_PIN_0
#define RFPOWEREN_GPIO_Port GPIOB
#define BNORESET_Pin GPIO_PIN_15
#define BNORESET_GPIO_Port GPIOE
#define BNOINT_Pin GPIO_PIN_11
#define BNOINT_GPIO_Port GPIOB
#define BNOINT_EXTI_IRQn EXTI15_10_IRQn
#define RF1MISO_Pin GPIO_PIN_14
#define RF1MISO_GPIO_Port GPIOB
#define RF1NRESET_Pin GPIO_PIN_11
#define RF1NRESET_GPIO_Port GPIOD
#define RF2CLK_Pin GPIO_PIN_5
#define RF2CLK_GPIO_Port GPIOA
#define RF2MISO_Pin GPIO_PIN_6
#define RF2MISO_GPIO_Port GPIOA
#define RF2NRESET_Pin GPIO_PIN_5
#define RF2NRESET_GPIO_Port GPIOC
#define RF1CLK_Pin GPIO_PIN_13
#define RF1CLK_GPIO_Port GPIOB
#define RF1BUSY_Pin GPIO_PIN_12
#define RF1BUSY_GPIO_Port GPIOD
#define RF2MOSI_Pin GPIO_PIN_7
#define RF2MOSI_GPIO_Port GPIOA
#define RF2BUSY_Pin GPIO_PIN_4
#define RF2BUSY_GPIO_Port GPIOC
#define RF2TXEN_Pin GPIO_PIN_1
#define RF2TXEN_GPIO_Port GPIOB
#define RF2IRQ_Pin GPIO_PIN_10
#define RF2IRQ_GPIO_Port GPIOE
#define RF2IRQ_EXTI_IRQn EXTI15_10_IRQn
#define RF1RXEN_Pin GPIO_PIN_13
#define RF1RXEN_GPIO_Port GPIOE
#define RF1NSS_Pin GPIO_PIN_12
#define RF1NSS_GPIO_Port GPIOB
#define RF1MOSI_Pin GPIO_PIN_15
#define RF1MOSI_GPIO_Port GPIOB
#define RF1IRQ_Pin GPIO_PIN_8
#define RF1IRQ_GPIO_Port GPIOD
#define RF1IRQ_EXTI_IRQn EXTI9_5_IRQn
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
