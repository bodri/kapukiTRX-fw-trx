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
#include "altitude/altitudesensor.h"

extern DMA_HandleTypeDef hdma_usart3_rx;

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define CRSF_MAX_CHANNELS   16U      // Maximum number of channels from crsf datastream
#define CRSF_FRAMELEN_MAX   64U      // maximum possible framelength

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

uint8_t crsfBuffer[26] { 0 };
volatile bool crsfPacketReceived { false };
volatile bool crsfFrameError { false };

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

struct Channels11Bit {
    // 176 bits of data (11 bits per channel * 16 channels) = 22 bytes.
#if __BYTE_ORDER != __LITTLE_ENDIAN
#error "Only supported on little-endian architectures"
#endif
    uint32_t ch0 : 11;
    uint32_t ch1 : 11;
    uint32_t ch2 : 11;
    uint32_t ch3 : 11;
    uint32_t ch4 : 11;
    uint32_t ch5 : 11;
    uint32_t ch6 : 11;
    uint32_t ch7 : 11;
    uint32_t ch8 : 11;
    uint32_t ch9 : 11;
    uint32_t ch10 : 11;
    uint32_t ch11 : 11;
    uint32_t ch12 : 11;
    uint32_t ch13 : 11;
    uint32_t ch14 : 11;
    uint32_t ch15 : 11;
};

void decode11BitChannels(const uint8_t* data, uint8_t nchannels, ChannelData &channelData, uint16_t mult, uint16_t div, uint16_t offset) {
#define CHANNEL_SCALE(x) ((int32_t(x) * mult) / div + offset)

    const Channels11Bit* channels = (const Channels11Bit *)data;
    channelData[0]->value = CHANNEL_SCALE(channels->ch0);
    channelData[1]->value = CHANNEL_SCALE(channels->ch1);
    channelData[2]->value = CHANNEL_SCALE(channels->ch2);
    channelData[3]->value = CHANNEL_SCALE(channels->ch3);
    channelData[4]->value = CHANNEL_SCALE(channels->ch4);
    channelData[5]->value = CHANNEL_SCALE(channels->ch5);
    channelData[6]->value = CHANNEL_SCALE(channels->ch6);
    channelData[7]->value = CHANNEL_SCALE(channels->ch7);
    channelData[8]->value = CHANNEL_SCALE(channels->ch8);
    channelData[9]->value = CHANNEL_SCALE(channels->ch9);
    channelData[10]->value = CHANNEL_SCALE(channels->ch10);
    channelData[11]->value = CHANNEL_SCALE(channels->ch11);
    channelData[12]->value = CHANNEL_SCALE(channels->ch12);
    channelData[13]->value = CHANNEL_SCALE(channels->ch13);
    channelData[14]->value = CHANNEL_SCALE(channels->ch14);
    channelData[15]->value = CHANNEL_SCALE(channels->ch15);
}

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
	if (huart->Instance == USART3) {
		crsfPacketReceived = true;
	}
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart) {
	__HAL_UART_CLEAR_OREFLAG(huart);
	__HAL_UART_CLEAR_PEFLAG(huart);
	__HAL_UART_CLEAR_NEFLAG(huart);
	__HAL_UART_CLEAR_FEFLAG(huart);
	__HAL_UART_DISABLE_IT(huart, UART_IT_ERR);
	crsfFrameError = true;
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
//  transmitter = HAL_GPIO_ReadPin(TXMODE_GPIO_Port, TXMODE_Pin) == GPIO_PIN_SET ? false : true;

  Pin redLed = Pin(LEDRED_GPIO_Port, LEDRED_Pin);
  Pin greenLed = Pin(LEDGREEN_GPIO_Port, LEDGREEN_Pin);
  Pin blueLed = Pin(LEDBLUE_GPIO_Port, LEDBLUE_Pin);
  visualStatus = new VisualStatus(&htim4, redLed, greenLed, blueLed);

  //  HAL_GPIO_WritePin(BNORESET_GPIO_Port, BNORESET_Pin, GPIO_PIN_SET);

	// Setup channels
	channelData = new ChannelData(26);

	// Setup telemetry
	if (transmitter) {
		telemetry = new Telemetry();
	} else {
		OrientationSensor *orientationSensor = new OrientationSensor();
		AltitudeSensor *altitudeSensor = new AltitudeSensor(BMP3_I2C_ADDR_SEC);

		telemetry = new Telemetry({
		  orientationSensor,
		  altitudeSensor
		});
	}

	rfLink = new RfLink(&htim4, transmitter);
	rfLink->init();
	rfLink->onTransmit = [](Packet &packet) {
//		for (unsigned i = 0; i < 8; i++) {
//			(*channelData)[i]->value = testData;
//		}
		channelData->fillRawChannelData(packet);
	};
	if (!transmitter) {
		rfLink->onPrepareTelemetryPacket = []() {
			uint8_t telemetryPacketSize = telemetry->prepareTelemetryPacket();
			return telemetryPacketSize + sizeof(Packet::status);
		};
		rfLink->onTransmitTelemetry = [](Packet &packet) {
			telemetry->sendTelemetryPacket(packet);
		};
	}
	rfLink->onReceive = [](Packet &packet) {
		*channelData = packet;
		float resolution = 139999.0f / 4095.0f;
		float pwm1 = 139999 + (*channelData)[0]->value * resolution;
		float pwm2 = 139999 + (*channelData)[1]->value * resolution;
		float pwm3 = 139999 + (*channelData)[2]->value * resolution;
		float pwm4 = 139999 + (*channelData)[3]->value * resolution;
		float pwm5 = 139999 + (*channelData)[4]->value * resolution;
		float pwm6 = 139999 + (*channelData)[5]->value * resolution;
		float pwm7 = 139999 + (*channelData)[6]->value * resolution;
		float pwm8 = 139999 + (*channelData)[7]->value * resolution;

		__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, pwm5);
		__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, pwm3);
		__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, pwm1);
		__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, pwm2);
		__HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_1, pwm7);
		__HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_2, pwm8);
		__HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_3, pwm4);
		__HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_4, pwm6);
	};
	rfLink->onReceiveTelemetry = [](Packet &packet) {
////		*telemetry = packet;
//		char buffer[127];
////		sprintf(buffer, "t:%d,y:%d,p:%d,r:%d", packet.payload[1], ((packet.payload[4] << 8) + packet.payload[3])/100, ((packet.payload[7] << 8) + packet.payload[6])/100, ((packet.payload[10] << 8) + packet.payload[9])/1000);
//		memcpy(buffer, packet.payload, packet.size - 2);
//		if (HAL_UART_Transmit(&huart3, (uint8_t *)buffer, packet.size - 2, 1000) != HAL_OK) {
//			Error_Handler();
//		}
	};

	rfLink->onLinkStatusChange = [](bool tracking) {
		tracking ? visualStatus->setStatus(TRACKING) : visualStatus->setStatus(CONNECTION_LOST);
	};

//  HAL_GPIO_WritePin(LEDBLUE_GPIO_Port, LEDBLUE_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(RFPOWEREN_GPIO_Port, RFPOWEREN_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(PWMOE_GPIO_Port, PWMOE_Pin, GPIO_PIN_SET);

  __HAL_UART_ENABLE_IT(&huart3, UART_IT_IDLE);
  HAL_UART_Receive_DMA(&huart3, crsfBuffer, sizeof(crsfBuffer));

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

	  if (crsfFrameError) {
		  HAL_UART_Receive_DMA(&huart3, crsfBuffer, sizeof(crsfBuffer));
		  crsfFrameError = false;
	  }

	  if (crsfPacketReceived) {
		  if (crsfBuffer[0] == 0xee) {
			  uint8_t crc = HAL_CRC_Calculate(&hcrc, (uint32_t *)&crsfBuffer[2], 23);
			  if (crsfBuffer[25] == crc) {
				  // valid packet
				  decode11BitChannels((const uint8_t *)(&crsfBuffer[3]), CRSF_MAX_CHANNELS, *channelData, 2U, 1U, 0U);

				  // send back dummy telemetry
				  uint8_t telemetry[] = "\x14\xA2\xA2\x55\x65\x02\x00\x05\x96\x44\x22";
				  static uint8_t buffer[sizeof(telemetry) + 2];
				  size_t len = sizeof(telemetry) - 1;
				  memcpy(&buffer[2], &telemetry[0], len);
				  buffer[0] = 0xEA;
				  buffer[1] = len + 1;
				  buffer[sizeof(buffer) - 1] = HAL_CRC_Calculate(&hcrc, (uint32_t *)&telemetry[0], len);
				  for (int i = 0; i < 1000; i++) { }
				  HAL_UART_Transmit_DMA(&huart3, buffer, sizeof(buffer));
			  } else {
				  for (int i = 0; i < 100; i++) { }
			  }
		  }

		  HAL_UART_Receive_DMA(&huart3, crsfBuffer, sizeof(crsfBuffer));
		  crsfPacketReceived = false;
	  }

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
