/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "math.h"
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

ADC_HandleTypeDef hadc;
DMA_HandleTypeDef hdma_adc;

I2C_HandleTypeDef hi2c2;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
//static const uint8_t TMP102_ADDR = 0x48 << 1;
//static const uint8_t REG_TEMP = 0x00;
static const uint8_t CHIPCAP2 = 0x28 << 1;

uint16_t adc_val1 = 0;
uint16_t adc_val2 = 0;
uint16_t adc_val3 = 0;
uint16_t adc_val4 = 0;
uint16_t adc_val5 = 0;
uint16_t adc_val6 = 0;
uint16_t adc_vbat = 0;

uint16_t raw_values[7];
char msgbuf[30];

uint8_t conv_completed = 0;


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC_Init(void);
static void MX_I2C2_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */



//void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
//{
//  conv_completed = 1;
//}

void XferCpltCallback(DMA_HandleTypeDef *hdma)
{
  __NOP(); //Line reached only if transfer was successful. Toggle a breakpoint here
}

float humidity_conversion(uint8_t rh_high, uint8_t rh_low) {
	return ((rh_high*256 + rh_low) / pow(2,14)) * 100;
}

float temp_conversion(uint8_t temp_high, uint8_t temp_low) {
	return ( ((temp_high*64 + temp_low/4) / pow(2,14)) * 165 ) - 40;
}

void read_temp_humid() {
	HAL_StatusTypeDef ret;
	uint8_t i2c_buf[30];
	ret = HAL_I2C_Master_Receive(&hi2c2, CHIPCAP2, i2c_buf, 4, HAL_MAX_DELAY);
	if (ret != HAL_OK) {
		strcpy((char*)i2c_buf, "Error Rx\r\n");
	} else {
		// Need 6 bits of the 8 bits of some parts of i2c_buf
		float hum = humidity_conversion((i2c_buf[0] & 0x3F), i2c_buf[1]);
		float temp = temp_conversion(i2c_buf[2], (i2c_buf[3] & 0x3F));
		sprintf((char*)i2c_buf,
			  "Hum: %u, Temp: %u \r\n",
			  ((unsigned int)hum),
			  ((unsigned int)temp)
			  );
	}
	HAL_UART_Transmit(&huart2, i2c_buf, strlen((char*)i2c_buf), HAL_MAX_DELAY);
}

void read_load_sensor_data() {
	uint16_t adc_val;
	uint8_t adc_buf[30];
	HAL_ADC_PollForConversion(&hadc, 20);
	adc_val = HAL_ADC_GetValue(&hadc);
	sprintf(adc_buf, "adc val: %u\r\n", adc_val);
	HAL_UART_Transmit(&huart2, adc_buf, strlen((char*)adc_buf), HAL_MAX_DELAY);
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

//  HAL_StatusTypeDef ret;
//  uint16_t val;
//  float temp_c;
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
  MX_ADC_Init();
  MX_I2C2_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
  hdma_adc.XferCpltCallback = &XferCpltCallback;
  while (HAL_ADCEx_Calibration_Start(&hadc) != HAL_OK);
//  uint32_t adc_cal;
//  adc_cal = HAL_ADCEx_Calibration_GetValue(&hadc);
//  sprintf(msgbuf, "adc cal: %hu \r\t\t", adc_cal);
//  HAL_UART_Transmit(&huart2, (uint8_t *) msgbuf, strlen(msgbuf), HAL_MAX_DELAY);

  HAL_ADC_Start_DMA(&hadc, (uint32_t*)raw_values, 7);
  /* USER CODE END 2 */

  /* Initialize leds */
  BSP_LED_Init(LED_BLUE);
  BSP_LED_Init(LED_GREEN);
  BSP_LED_Init(LED_RED);

  /* Initialize USER push-button, will be used to trigger an interrupt each time it's pressed.*/
  BSP_PB_Init(BUTTON_SW1, BUTTON_MODE_EXTI);
  BSP_PB_Init(BUTTON_SW2, BUTTON_MODE_EXTI);
  BSP_PB_Init(BUTTON_SW3, BUTTON_MODE_EXTI);

  /* Boot CPU2 */
  HAL_PWREx_ReleaseCore(PWR_CORE_CPU2);

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {



//	  buf[0] = REG_TEMP;
//	  	    ret = HAL_I2C_Master_Transmit(&hi2c2, TMP102_ADDR, buf, 1, 200);
//	  	    if ( ret != HAL_OK ) {
//	  	      strcpy((char*)buf, "Error Tx\r\n");
//	  	    } else {
//
//	  	      // Read 2 bytes from the temperature register
//	  	      ret = HAL_I2C_Master_Receive(&hi2c2, TMP102_ADDR, buf, 2, 200);
//	  	      if ( ret != HAL_OK ) {
//	  	        strcpy((char*)buf, "Error Rx\r\n");
//	  	      } else {
//
//	  	        //Combine the bytes
//	  	        val = ((int16_t)buf[0] << 4) | (buf[1] >> 4);
//
//	  	        // Convert to 2's complement, since temperature can be negative
//	  	        if ( val > 0x7FF ) {
//	  	          val |= 0xF000;
//	  	        }
//
//	  	        // Convert to float temperature value (Celsius)
//	  	        temp_c = val * 0.0625;
//
//	  	        // Convert temperature to decimal format
//	  	        temp_c *= 100;
//	  	        sprintf((char*)buf,
//	  	              "%u.%u C\r\n",
//	  	              ((unsigned int)temp_c / 100),
//	  	              ((unsigned int)temp_c % 100));
//	  	      }
//	  	    }
//
//	  	    // Send out buffer (temperature or error message)
//	  	    HAL_UART_Transmit(&huart2, buf, strlen((char*)buf), HAL_MAX_DELAY);
//
//	  	    // Wait
//	  	    HAL_Delay(500);

//	ret = HAL_I2C_Master_Transmit(&hi2c2, TEMPHUM_ADDR, buf, 1, HAL_MAX_DELAY);
//	if (ret != HAL_OK) {
//		strcpy((char*)buf, "Error Tx\r\n");
//	} else {
//		ret = HAL_I2C_Master_Receive(&hi2c2, TEMPHUM_ADDR, buf, 4, HAL_MAX_DELAY);
//		if (ret != HAL_OK) {
//			strcpy((char*)buf, "Error Rx\r\n");
//		} else {
//			sprintf((char*)buf, "Got something!\r\n");
//		}
//
//	}
//	while(!conv_completed);


	//HAL_ADC_Start(&hadc);
	//read_load_sensor_data();
//	HAL_ADC_Stop(&hadc);
//	read_temp_humid();
//	read_load_sensor_data();
	for(uint8_t i = 0; i < hadc.Init.NbrOfConversion; i++) {
		adc_val1 = (uint16_t) raw_values[0];
		adc_val2 = (uint16_t) raw_values[1];
		adc_val3 = (uint16_t) raw_values[2];
		adc_val4 = (uint16_t) raw_values[3];
		adc_val5 = (uint16_t) raw_values[4];
		adc_val6 = (uint16_t) raw_values[5];
		adc_vbat = (uint16_t) raw_values[6];
	}

//	sprintf(msgbuf, "vbat: %hu \r\t\t", adc_vbat);
//	HAL_UART_Transmit(&huart2, (uint8_t *) msgbuf, strlen(msgbuf), HAL_MAX_DELAY);
//
//	sprintf(msgbuf, "val1: %hu \r\t\t", adc_val1);
//	HAL_UART_Transmit(&huart2, (uint8_t *) msgbuf, strlen(msgbuf), HAL_MAX_DELAY);
//
//	sprintf(msgbuf, "val2: %hu \r\t\t", adc_val2);
//	HAL_UART_Transmit(&huart2, (uint8_t *) msgbuf, strlen(msgbuf), HAL_MAX_DELAY);
//
//	sprintf(msgbuf, "val3: %hu \r\t\t", adc_val3);
//	HAL_UART_Transmit(&huart2, (uint8_t *) msgbuf, strlen(msgbuf), HAL_MAX_DELAY);
//
//	sprintf(msgbuf, "val4: %hu \r\t\t", adc_val4);
//	HAL_UART_Transmit(&huart2, (uint8_t *) msgbuf, strlen(msgbuf), HAL_MAX_DELAY);
//
	sprintf(msgbuf, "val5: %hu \r\t\t\n", adc_val5);
	HAL_UART_Transmit(&huart2, (uint8_t *) msgbuf, strlen(msgbuf), HAL_MAX_DELAY);
//
//	sprintf(msgbuf, "val6: %hu \r\t\t", adc_val6);
//	HAL_UART_Transmit(&huart2, (uint8_t *) msgbuf, strlen(msgbuf), HAL_MAX_DELAY);

	HAL_Delay(1000);
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
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

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.MSICalibrationValue = RCC_MSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure the SYSCLKSource, HCLK, PCLK1 and PCLK2 clocks dividers
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK3|RCC_CLOCKTYPE_HCLK2
                              |RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_MSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.AHBCLK2Divider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLK3Divider = RCC_SYSCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC_Init(void)
{

  /* USER CODE BEGIN ADC_Init 0 */

  /* USER CODE END ADC_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC_Init 1 */

  /* USER CODE END ADC_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc.Instance = ADC;
  hadc.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV64;
  hadc.Init.Resolution = ADC_RESOLUTION_12B;
  hadc.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc.Init.LowPowerAutoWait = DISABLE;
  hadc.Init.LowPowerAutoPowerOff = DISABLE;
  hadc.Init.ContinuousConvMode = ENABLE;
  hadc.Init.NbrOfConversion = 7;
  hadc.Init.DiscontinuousConvMode = DISABLE;
  hadc.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc.Init.DMAContinuousRequests = ENABLE;
  hadc.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc.Init.SamplingTimeCommon1 = ADC_SAMPLETIME_79CYCLES_5;
  hadc.Init.SamplingTimeCommon2 = ADC_SAMPLETIME_79CYCLES_5;
  hadc.Init.OversamplingMode = DISABLE;
  hadc.Init.TriggerFrequencyMode = ADC_TRIGGER_FREQ_HIGH;
  if (HAL_ADC_Init(&hadc) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLINGTIME_COMMON_1;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_2;
  sConfig.Rank = ADC_REGULAR_RANK_2;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_3;
  sConfig.Rank = ADC_REGULAR_RANK_3;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_4;
  sConfig.Rank = ADC_REGULAR_RANK_4;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_5;
  sConfig.Rank = ADC_REGULAR_RANK_5;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_6;
  sConfig.Rank = ADC_REGULAR_RANK_6;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_VBAT;
  sConfig.Rank = ADC_REGULAR_RANK_7;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC_Init 2 */

  /* USER CODE END ADC_Init 2 */

}

/**
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.Timing = 0x00100D14;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c2, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c2, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart2, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart2, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMAMUX1_CLK_ENABLE();
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
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
  __disable_irq();
  while (1)
  {
  }
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
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
