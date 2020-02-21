/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "fdcan.h"
#include "i2c.h"
#include "iwdg.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include "rflink.h"
#include "visualstatus.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

bool transmitter { false };
RfLink *rfLink;
VisualStatus *visualStatus;

int16_t testData { 0 };
bool testDirectionUp { true };

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {

//	uint16_t c = rfLink.rf1Module->getFirmwareVersion();
//	uint16_t c2 = rfLink.rf2Module->getFirmwareVersion();
//	HAL_GPIO_WritePin(PWMOE_GPIO_Port, PWMOE_Pin, GPIO_PIN_SET);

	if (htim->Instance == TIM4) {
		testData += testDirectionUp ? 10 : -10;
		if (testData > 0xFFF) {
			testData = 0xFFF;
			testDirectionUp = false;
		} else if (testData < 0) {
			testData = 0;
			testDirectionUp = true;
		}
	}

	rfLink->processHeartBeat(htim);
	visualStatus->processHeartBeat(htim);
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
	rfLink->processIrqs(GPIO_Pin);
}


/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */
  

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_SPI1_Init();
  MX_TIM2_Init();
  MX_TIM5_Init();
  MX_SPI2_Init();
  MX_FDCAN3_Init();
  MX_I2C1_Init();
  MX_USART1_UART_Init();
  MX_TIM4_Init();
//  MX_IWDG_Init();
  /* USER CODE BEGIN 2 */

  // Check if this is a transmitter?
//  if (HAL_GPIO_ReadPin(TXMODE_GPIO_Port, TXMODE_Pin) == GPIO_PIN_RESET) {
//	  transmitter = false;
//  }


  Pin redLed = Pin(LEDRED_GPIO_Port, LEDRED_Pin);
  Pin greenLed = Pin(LEDGREEN_GPIO_Port, LEDGREEN_Pin);
  Pin blueLed = Pin(LEDBLUE_GPIO_Port, LEDBLUE_Pin);
  visualStatus = new VisualStatus(&htim4, redLed, greenLed, blueLed);

  rfLink = new RfLink(&htim4, transmitter);
  rfLink->init();
	rfLink->onTransmit = [](Packet &packet) {
		ServoData data {0};
		uint8_t lsb = (uint8_t)testData;
		uint8_t msb = (uint8_t)((testData >> 8) & 0x0F);
		data.ch1Lsb = lsb;
		data.ch12Msb.ch1Msb = msb;
		data.ch2Lsb = lsb;
		data.ch12Msb.ch2Msb = msb;
		data.ch3Lsb = lsb;
		data.ch34Msb.ch3Msb = msb;
		data.ch4Lsb = lsb;
		data.ch34Msb.ch4Msb = msb;

		std::copy(std::begin(data), std::end(data), std::begin(packet.payload));
	};
	rfLink->onReceive = [](Packet &packet) {
	  float resolution = 139999.0f / 4095.0f;
	  float pwm1 = 139999 + (packet.payload[0] + (((packet.payload[2] >> 4) & 0x0F) << 8)) * resolution;
	  float pwm2 = 139999 + (packet.payload[1] + ((packet.payload[2] & 0x0F) << 8)) * resolution;
	  float pwm3 = 139999 + (packet.payload[3] + (((packet.payload[5] >> 4) & 0x0F) << 8)) * resolution;
	  float pwm4 = 139999 + (packet.payload[4] + ((packet.payload[5] & 0x0F) << 8)) * resolution;

	  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, pwm1);
	  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, pwm2);
	  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, pwm3);
	  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, pwm4);
	  __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_1, pwm1);
	  __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_2, pwm2);
	  __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_3, pwm3);
	  __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_4, pwm4);
	};


	rfLink->onLinkStatusChange = [](bool tracking) {
		tracking ? visualStatus->setStatus(TRACKING) : visualStatus->setStatus(CONNECTION_LOST);
	};

//  HAL_GPIO_WritePin(LEDBLUE_GPIO_Port, LEDBLUE_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(RFPOWEREN_GPIO_Port, RFPOWEREN_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(PWMOE_GPIO_Port, PWMOE_Pin, GPIO_PIN_SET);

//  HAL_GPIO_WritePin(BNORESET_GPIO_Port, BNORESET_Pin, GPIO_PIN_SET);

  unsigned char buffer[5];
  memset(buffer, 0, 5);
//  unsigned short address = 0x0;

  /* USER CODE END 2 */
 
 

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4);
  HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_4);
  HAL_TIM_Base_Start_IT(&htim4);  // Start heartbeat timer
  while (1)
  {
	rfLink->runLoop();
	visualStatus->runLoop();
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

//	  if (HAL_I2C_Master_Transmit(&hi2c1, address, buffer, 1, 1000000) == HAL_OK) {
//		  HAL_GPIO_WritePin(LEDRED_GPIO_Port, LEDRED_Pin, GPIO_PIN_RESET);
//	  }
//	  HAL_Delay(20);
//	  if (HAL_I2C_Master_Receive(&hi2c1, 0x28, buffer, 1, 1000) == HAL_OK) {
//		  HAL_GPIO_WritePin(LEDGREEN_GPIO_Port, LEDGREEN_Pin, GPIO_PIN_RESET);
//	  }

//	  if (HAL_I2C_Mem_Read(&hi2c1, 0x50, 0, 1, buffer, 1, 1000) == HAL_OK) {
//		  HAL_GPIO_WritePin(LEDGREEN_GPIO_Port, LEDGREEN_Pin, GPIO_PIN_RESET);
//	  }
//	  HAL_Delay(100);
//	  address++;
//	  if (address > 0xFF) {
//		  address = 0;
//	  }
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Configure the main internal regulator output voltage 
  */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV4;
  RCC_OscInitStruct.PLL.PLLN = 70;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_6) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the peripherals clocks 
  */
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1|RCC_PERIPHCLK_I2C1
                              |RCC_PERIPHCLK_FDCAN;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK2;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_PCLK1;
  PeriphClkInit.FdcanClockSelection = RCC_FDCANCLKSOURCE_PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
