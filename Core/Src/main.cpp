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
#include "crc.h"
#include "dma.h"
#include "fdcan.h"
#include "i2c.h"
#include "iwdg.h"
#include "rng.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include "rflink.h"
#include "visualstatus.h"
#include "channel.h"
#include "telemetry.h"
#include "orientation/orientationsensor.h"
#include "vario/variosensor.h"
#include "radio/transmittersensor.h"
#include "radio/receiversensor.h"
#include "crsf.h"

extern DMA_HandleTypeDef hdma_usart3_rx;

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

bool transmitter { true };
RfLink *rfLink;
VisualStatus *visualStatus;
ChannelData *channelData;
Telemetry *telemetry;
Crossfire *crossfire;

uint8_t crsfBuffer[CRSF_FRAMELEN_MAX] { 0 };
volatile bool crsfPacketReceived { false };

// temp
bool rxTracking { false };

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {

	if (htim->Instance == TIM4) {
		if (!transmitter) {
			telemetry->processHeartBeat();
		}
	}

	rfLink->processHeartBeat(htim);
	visualStatus->processHeartBeat(htim);
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
	rfLink->processIrqs(GPIO_Pin);
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
	crossfire->processRxComplete(huart, &crsfPacketReceived);
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart) {
	crossfire->processSerialError(huart);
}

void setUpChannels() {
	channelData = new ChannelData(16);
}

void setUpStatusLeds() {
	Pin redLed = Pin(LEDRED_GPIO_Port, LEDRED_Pin);
	Pin greenLed = Pin(LEDGREEN_GPIO_Port, LEDGREEN_Pin);
	Pin blueLed = Pin(LEDBLUE_GPIO_Port, LEDBLUE_Pin);
	visualStatus = new VisualStatus(&htim4, redLed, greenLed, blueLed);
}

void setUpTelemetry() {
	if (transmitter) {
		TransmitterSensor *transmitterSensor = new TransmitterSensor();

		telemetry = new Telemetry({
			transmitterSensor
		});
	} else {
		ReceiverSensor *receiverSensor = new ReceiverSensor();
		OrientationSensor *orientationSensor = new OrientationSensor();
		VarioSensor *varioSensor = new VarioSensor(BMP3_I2C_ADDR_SEC);

		telemetry = new Telemetry({
			receiverSensor,
			orientationSensor,
			varioSensor
		});
	}
}

void setUpRfLink() {
	// Power up RF modules
	HAL_GPIO_WritePin(RFPOWEREN_GPIO_Port, RFPOWEREN_Pin, GPIO_PIN_SET);

	rfLink = new RfLink(&htim4, transmitter);
	rfLink->init();

	if (transmitter) {
		rfLink->onTransmit = [](Packet &packet) {
			channelData->fillRawChannelData(packet);
		};

		rfLink->onReceiveTelemetry = [](Packet &packet) {
			*telemetry = packet;
		};
	} else {
		rfLink->onPrepareTelemetryPacket = []() {
			uint8_t telemetryPacketSize = telemetry->prepareTelemetryPacket();
			return telemetryPacketSize + sizeof(Packet::status);
		};

		rfLink->onTransmitTelemetry = [](Packet &packet) {
			telemetry->sendTelemetryPacket(packet);
		};

		rfLink->onReceive = [](Packet &packet) {
			static const float pwm1msec = 139999.0f;

			*channelData = packet;
			float resolution = pwm1msec / (packet.status.packetType == NORMAL ? 4096.0f : 2048.0f);
			float pwm1 = pwm1msec + *(*channelData)[0] * resolution;
			float pwm2 = pwm1msec + *(*channelData)[1] * resolution;
			float pwm3 = pwm1msec + *(*channelData)[2] * resolution;
			float pwm4 = pwm1msec + *(*channelData)[3] * resolution;
			float pwm5 = pwm1msec + *(*channelData)[4] * resolution;
			float pwm6 = pwm1msec + *(*channelData)[5] * resolution;
			float pwm7 = pwm1msec + *(*channelData)[6] * resolution;
			float pwm8 = pwm1msec + *(*channelData)[7] * resolution;
			// !!!!!!!!! Make sure we will be in the range !!!!!!!!!!!!!!!

			__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, pwm1);
			__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, pwm2);
			__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, pwm3);
			__HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_2, pwm4);
			__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, pwm5);
			__HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_4, pwm6);
			__HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_1, pwm7);
			__HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_3, pwm8);
		};
	}

	rfLink->onLinkStatusChange = [](bool tracking) {
		rxTracking = tracking;
		tracking ? visualStatus->setStatus(TRACKING) : visualStatus->setStatus(CONNECTION_LOST);
	};
}

void setUpExtensionPort() {
	if (transmitter) {
		// setup crossfire
		crossfire = new Crossfire(&huart3, &hcrc);
		HAL_UART_Receive_DMA(&huart3, crsfBuffer, sizeof(crsfBuffer));
	}
}

void startPwmOutputForChannelData() {
	if (transmitter) { return; }

	HAL_GPIO_WritePin(PWMOE_GPIO_Port, PWMOE_Pin, GPIO_PIN_SET);
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4);
	HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_4);
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
  MX_DMA_Init();
  MX_SPI1_Init();
  MX_TIM2_Init();
  MX_TIM5_Init();
  MX_SPI2_Init();
  MX_FDCAN3_Init();
  MX_USART1_UART_Init();
  MX_TIM4_Init();
//  MX_IWDG_Init();
  MX_I2C3_Init();
  MX_RNG_Init();
  MX_USART3_UART_Init();
  MX_CRC_Init();
  /* USER CODE BEGIN 2 */

	// Check if this is a transmitter?
	transmitter = HAL_GPIO_ReadPin(TXMODE_GPIO_Port, TXMODE_Pin) == GPIO_PIN_RESET;

	setUpChannels();
	setUpStatusLeds();
	setUpTelemetry();
	setUpRfLink();
	setUpExtensionPort();

	startPwmOutputForChannelData();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  HAL_TIM_Base_Start_IT(&htim4);  // Start heartbeat timer
  while (1)
  {
	rfLink->runLoop();
	visualStatus->runLoop();
	rfLink->setTelemetry(telemetry);
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	if (transmitter) {
		crossfire->setTelemetry(rxTracking, telemetry);
		crossfire->decodePacket(crsfBuffer, CRSF_FRAMELEN_MAX, *channelData, &crsfPacketReceived);
	}

//	  HAL_Delay(100);
//	  int8_t temperature = orientationSensor.getTemperature();
//	  bno055_quaternion_t quaternion = orientationSensor.getQuaternion();
//	  bno055_euler_double_t vector = orientationSensor.getEulerVector();
//	  altitudeSensor.performReading();
//	  double temperature = altitudeSensor.temperature;
//	  double pressure = altitudeSensor.pressure; //readAltitude(1023);
//	  for (int i = 0; i < 100; i++) { };
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
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_HSI48
                              |RCC_OSCILLATORTYPE_LSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.HSI48State = RCC_HSI48_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV4;
  RCC_OscInitStruct.PLL.PLLN = 70;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV6;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the peripherals clocks
  */
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1|RCC_PERIPHCLK_USART3
                              |RCC_PERIPHCLK_I2C3|RCC_PERIPHCLK_RNG
                              |RCC_PERIPHCLK_FDCAN;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK2;
  PeriphClkInit.Usart3ClockSelection = RCC_USART3CLKSOURCE_HSI;
  PeriphClkInit.I2c3ClockSelection = RCC_I2C3CLKSOURCE_PCLK1;
  PeriphClkInit.FdcanClockSelection = RCC_FDCANCLKSOURCE_PCLK1;
  PeriphClkInit.RngClockSelection = RCC_RNGCLKSOURCE_HSI48;
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
