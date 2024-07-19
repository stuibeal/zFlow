/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2022 STMicroelectronics.
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
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "communication.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
//__IO uint32_t Transfer_Direction = 0;
//__IO uint32_t Xfer_Complete = 0;
//__IO uint32_t errorCode = 0;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* Buffer used for transmission */
uint8_t aTxBuffer[2];

/* Buffer used for reception */
uint8_t aRxBuffer[3];

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;
DMA_HandleTypeDef hdma_i2c1_rx;
DMA_HandleTypeDef hdma_i2c1_tx;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim5;

/* USER CODE BEGIN PV */
__IO uint8_t interruptCounter = 0;
unsigned int userMillis = 500;
__IO unsigned int zapfMillis = 0;
unsigned int oldMillis = 0;
unsigned int countProMl;
char buffer[60];
uint8_t energieSparen = 0;
uint8_t ebnerModus = 0;
uint8_t ledDelayVar = 10;
uint8_t recieveComplete = 0;
unsigned int helligkeit = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM5_Init(void);
static void MX_NVIC_Init(void);
/* USER CODE BEGIN PFP */
void zapfStandLed(uint16_t anzeigeMillis);
void dimLed(unsigned int, unsigned int);
void ledDelay(void);
void warpLed(uint16_t wieOftAusfuehren);
void warpLed2(uint32_t laufZeit);
void warpLed3(uint32_t laufZeit);
void gauselMann(void);
void dimLedToWert(void);
void checkAfterMaster(void);
uint16_t getFromRxBuffer(void);

extern uint8_t CDC_Transmit_FS(char *Buf, uint16_t Len);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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
	aRxBuffer[0] = 0x00;
	aRxBuffer[1] = 0x00;
	aRxBuffer[2] = 0x00;
	aTxBuffer[0] = 0x00;
	aTxBuffer[1] = 0x00;
	countProMl = 1048; // bis der Master was anderes schickt
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_I2C1_Init();
  MX_USB_DEVICE_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM5_Init();

  /* Initialize interrupts */
  MX_NVIC_Init();
  /* USER CODE BEGIN 2 */
	/* Starte PWM  Timer */
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);
	HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_1);

	HAL_Delay(1);

	/*##-2- Put I2C peripheral in reception process ###########################*/
	if (HAL_I2C_Slave_Receive_DMA(&hi2c1, aRxBuffer, 3) != HAL_OK) {
		/* Transfer error in reception process */
		Error_Handler();
	}

	/* LED aus */
	HAL_GPIO_WritePin(userLed_GPIO_Port, userLed_Pin, 1);


	aRxBuffer[2]=10;
	warpLed3(250);
	aRxBuffer[2]=0;

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1) {

		if (recieveComplete == 1) {
			recieveComplete = 0;
			if (aRxBuffer[0] > 0x01)
				checkAfterMaster();
		}

		if (oldMillis != zapfMillis) {
			//HAL_GPIO_TogglePin(userLed_GPIO_Port, userLed_Pin);
			oldMillis = zapfMillis;
			if (zapfMillis < userMillis) {
				zapfStandLed(zapfMillis);
			}
			else {
				zapfStandLed(userMillis);
			}

//			sprintf(buffer, "zapf: %d ml aTx0: %x aTx1: %x  \n\r", zapfMillis,
//					aTxBuffer[0], aTxBuffer[1]);
//			CDC_Transmit_FS(buffer, sizeof(buffer));

//			if (zapfMillis >= userMillis) {
//				zapfMillis = 0;
//				for (unsigned int x = 65535; x > 128; x -= 127) {
//					dimLed(0, x);
//					HAL_Delay(1);
//				}
//				dimLed(0, 0);
//			}

		}


	}

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

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
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 25;
  RCC_OscInitStruct.PLL.PLLN = 192;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV4;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief NVIC Configuration.
  * @retval None
  */
static void MX_NVIC_Init(void)
{
  /* EXTI4_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(EXTI4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI4_IRQn);
  /* DMA1_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream0_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream0_IRQn);
  /* DMA1_Stream1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream1_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream1_IRQn);
  /* I2C1_EV_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(I2C1_EV_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(I2C1_EV_IRQn);
  /* I2C1_ER_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(I2C1_ER_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(I2C1_ER_IRQn);
  /* OTG_FS_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(OTG_FS_IRQn, 2, 0);
  HAL_NVIC_EnableIRQ(OTG_FS_IRQn);
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 400000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 36;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 65535;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * @brief TIM5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM5_Init(void)
{

  /* USER CODE BEGIN TIM5_Init 0 */

  /* USER CODE END TIM5_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM5_Init 1 */

  /* USER CODE END TIM5_Init 1 */
  htim5.Instance = TIM5;
  htim5.Init.Prescaler = 0;
  htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim5.Init.Period = 65535;
  htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim5.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim5) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim5, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim5, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM5_Init 2 */

  /* USER CODE END TIM5_Init 2 */
  HAL_TIM_MspPostInit(&htim5);

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(userLed_GPIO_Port, userLed_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : userLed_Pin */
  GPIO_InitStruct.Pin = userLed_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(userLed_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : flowIn_Pin */
  GPIO_InitStruct.Pin = flowIn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(flowIn_GPIO_Port, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
/* Interrupt Service Routine für den SM6020 Flowmeter von ifm
 *
 * Zählt nur vernünftig, wenn der Pin auf Rising/Falling eingestellt ist.
 * I DONT KNOW WHY. FUCK.
 *
 * Natürlich hat man dann die doppelte Anzahl an Pulsen. Wenn die ISR die volatile zapfMillis bei
 * jedem zweiten Puls hochzählt hat man aber absolut korrekte Ergebnisse,
 * egal was noch einen IRQ auslöst.
 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
	if (GPIO_Pin == flowIn_Pin) {
		interruptCounter++;
		if (interruptCounter == 2) {
			interruptCounter = 0;
			zapfMillis++;
		}
		/* Gleich mal die Ausgabewerte für den I2C befüllen */
		aTxBuffer[1] = zapfMillis & 0xFF;   // Lowbyte (die hinteren 8 Bit)
		aTxBuffer[0] = zapfMillis >> 8;	// Highbyte (die vorderen acht Bit)
	}
}

/**
 * @brief  Diese Funktion gibt den Wert der zwei I2C Datenbytes als INT zurück
 * @param  none
 * @retval INT von aRxBuffer[1] und aRxBuffer[2]
 */
uint16_t getFromRxBuffer(void) {
	return ((aRxBuffer[1] << 8) | (aRxBuffer[2] & 0xFF));
}

void checkAfterMaster(void) {
	HAL_GPIO_TogglePin(userLed_GPIO_Port, userLed_Pin);

	switch (aRxBuffer[0]) {

	case setUserMilliLitres:
		userMillis = getFromRxBuffer();
		countProMl = 524287 / userMillis;
		break;
	case beginZapf:
		zapfMillis = 0;
		interruptCounter = 0;
		dimLed(0, helligkeit);
		countProMl = 524287 / userMillis;
		aTxBuffer[1] = zapfMillis & 0xFF;   // Lowbyte (die hinteren 8 Bit)
		aTxBuffer[0] = zapfMillis >> 8;	// Highbyte (die vorderen acht Bit)
		break;
	case makeFunWithLeds1:
		/* Laufzeit kommt von aRxBuffer[1], Delay von aRxBuffer[2] */
		warpLed(aRxBuffer[1]);
		break;
	case makeFunWithLeds2:
		warpLed2(aRxBuffer[1] * 1000);
		break;
	case makeFunWithLeds3:
		warpLed3(aRxBuffer[1] * 1000);
		break;
	case makeFunWithLeds4:
		gauselMann();
		break;
	case DIM_LED_TO_WERT:
		dimLedToWert();
		break;
	case endZapf:
		zapfMillis = 0;
		interruptCounter = 0;
		aTxBuffer[1] = zapfMillis & 0xFF;   // Lowbyte (die hinteren 8 Bit)
		aTxBuffer[0] = zapfMillis >> 8;	// Highbyte (die vorderen acht Bit)
		break;
	case ebiMode:
		ebnerModus = 1;
		break;
	case lowEnergy:
		if (aRxBuffer[1] == 0x01) {
			energieSparen = 1;
			dimLed(0, 0);
		}
		if (aRxBuffer[1] == 0x00) {
			energieSparen = 0;
			dimLed(0,0xFFFF);
			HAL_Delay(250);
			dimLed(0,0);
		}

		break;

	}

}

/* Zeigt den aktuellen  Zapfstand an */
void zapfStandLed(uint16_t anzeigeMillis) {
	unsigned long countsAktuell = countProMl * anzeigeMillis;


	if (countsAktuell < 1 * 0xFFFF) {
		dimLed(1, countsAktuell);
		//TIM2->CCR1 = countsAktuell;
	}
	if ((countsAktuell < 2 * 0xFFFF) && (countsAktuell > 1 * 0xFFFF)) {
		dimLed(1, 0xFFFF);
		//TIM2->CCR1 = 0xFFFF;
		dimLed(2, countsAktuell - 0xFFFF);
		//TIM2->CCR2 = countsAktuell - 0xFFFF;
	}
	if ((countsAktuell < 3 * 0xFFFF) && (countsAktuell > 2 * 0xFFFF)) {
		dimLed(2, 0xFFFF);
		dimLed(3, countsAktuell - 2 * 0xFFFF);
		//TIM2->CCR2 = 0xFFFF;
		//TIM2->CCR3 = countsAktuell - 2 * 0xFFFF;
	}
	if ((countsAktuell < 4 * 0xFFFF) && (countsAktuell > 3 * 0xFFFF)) {
		dimLed(3, 0xFFFF);
		dimLed(4, countsAktuell - 3 * 0xFFFF);

		//TIM2->CCR3 = 0xFFFF;
		//TIM3->CCR1 = countsAktuell - 3 * 0xFFFF;
	}
	if ((countsAktuell < 5 * 0xFFFF) && (countsAktuell > 4 * 0xFFFF)) {
		dimLed(4, 0xFFFF);
		dimLed(5, countsAktuell - 4 * 0xFFFF);

		//TIM3->CCR1 = 0xFFFF;
		//TIM3->CCR2 = countsAktuell - 4 * 0xFFFF;
	}
	if ((countsAktuell < 6 * 0xFFFF) && (countsAktuell > 5 * 0xFFFF)) {
		dimLed(5, 0xFFFF);
		dimLed(6, countsAktuell - 5 * 0xFFFF);

		//TIM3->CCR3 = 0xFFFF;
		//TIM3->CCR3 = countsAktuell - 5 * 0xFFFF;
	}
	if ((countsAktuell < 7 * 0xFFFF) && (countsAktuell > 6 * 0xFFFF)) {
		dimLed(6, 0xFFFF);
		dimLed(7, countsAktuell - 6 * 0xFFFF);

		//TIM3->CCR3 = 0xFFFF;
		//TIM3->CCR4 = countsAktuell - 6 * 0xFFFF;
	}
	if ((countsAktuell < 8 * 0xFFFF) && (countsAktuell > 7 * 0xFFFF)) {
		dimLed(7, 0xFFFF);
		dimLed(8, countsAktuell - 7 * 0xFFFF);

		//TIM3->CCR4 = 0xFFFF;
		//TIM5->CCR1 = countsAktuell - 7 * 0xFFFF;
	}
	if (energieSparen == 1){
		dimLed(0,0);
		}

}


void ledDelay(void) {
	/* Das zweite Byte von makeFunWithLeds wird als Systick Delay genutzt */

	uint16_t x = 0;
	if (aRxBuffer[2] > 10) {
		ledDelayVar = aRxBuffer[2];
	}
	while (x <= ledDelayVar * 100) {
		x++;
	}
}

void warpLed(uint16_t wieOftAusfuehren) {
	//uint32_t beginnZeit = HAL_GetTick();
	uint16_t warp[8];
    uint16_t wieOftAusgefuehrt = 0;

    // alles leicht einschalten am Anfang
	for (int x = 0; x<8; x++){
		warp[x] = 1023;
	}

	uint16_t teile = 0;
	uint8_t standPunkt = 0;
	//while (HAL_GetTick() - beginnZeit < laufZeit) {
	while (wieOftAusgefuehrt < aRxBuffer[1]) {  //stoppt wenn man eine Anfrage schickt die den aRXbuffer auf 0 setzt
		teile += 31;
		if (teile > 1984) {
			teile = 0;
			standPunkt++;
		}

		if (standPunkt == 0) {
			warp[0] += teile;
		}

		if (standPunkt == 1) {
			warp[1] += teile;
			warp[0] -= teile;
		}
		if (standPunkt == 2) {
			warp[2] += teile;
			warp[1] -= teile;
		}
		if (standPunkt == 3) {
			warp[3] += teile;
			warp[2] -= teile;
		}
		if (standPunkt == 4) {
			warp[0] += teile;
			warp[3] -= teile;
		}
		if (standPunkt > 4) {
			wieOftAusgefuehrt++;
			standPunkt = 1;
		}

		warp[7] = warp[0];
		warp[6] = warp[1];
		warp[5] = warp[2];
		warp[4] = warp[3];

		//hier die ganzen Werte ausgeben auf den LEDs
		for (uint8_t x = 0; x < 9; x++) {
			dimLed(x + 1, warp[x]);
		}
		ledDelay();
	}


	/* Alle Leds runterdimmen */

	/*
	for (uint8_t x = 0; x < 9; x++) {

		for (uint16_t y = warp[x]; y < 1; y--) {
			dimLed(x + 1, y);
		}
	}
	*/

}

void dimLedToWert() {
	//hier die LEDS dimmen wenn feddich
	uint8_t anfang = aRxBuffer[1];
	uint8_t ende = aRxBuffer[2];
	aRxBuffer[2] = 100; //shorter! das ist fürs delay dann
	for (uint8_t x = anfang; x > ende ; x --) {
		dimLed(0, (x << 8| 0xFF)); //0= alle LEDS, x der Zustand
		ledDelay();
	}
	if (ende > 0) {
		helligkeit = ende << 8 | 0x00;
	} else {
		helligkeit = 0;
	}
	dimLed(0,helligkeit);

}

void warpLed2(uint32_t laufZeit) {
	uint32_t beginnZeit = HAL_GetTick();
	uint16_t warp[8];
	for (uint16_t y = 0; y < 1024; y++) {
		for (uint8_t x = 0; x < 8; x++) {
			warp[x] = y;
			dimLed(x + 1, warp[x]);
		}
	}

	uint16_t teile = 0;
	uint8_t standPunkt = 0;
	while (HAL_GetTick() - beginnZeit < laufZeit) {
		teile += 31;
		if (teile > 1984) {
			teile = 0;
			standPunkt++;
			if (standPunkt > 9) {
				standPunkt = 2;
			}

		}

		if (standPunkt == 0) {
			warp[0] += teile;
		}

		if (standPunkt == 1) {
			warp[1] += teile;
		}
		if (standPunkt == 2) {
			warp[0] -= teile;
			warp[2] += teile;
		}
		if (standPunkt == 3) {
			warp[1] -= teile;
			warp[3] += teile;
		}
		if (standPunkt == 4) {
			warp[2] -= teile;
			warp[4] += teile;
		}
		if (standPunkt == 5) {
			warp[3] -= teile;
			warp[5] += teile;
		}
		if (standPunkt == 6) {
			warp[4] -= teile;
			warp[6] += teile;
		}
		if (standPunkt == 7) {
			warp[5] -= teile;
			warp[7] += teile;
		}
		if (standPunkt == 8) {
			warp[6] -= teile;
			warp[0] += teile;
		}
		if (standPunkt == 9) {
			warp[7] -= teile;
			warp[1] += teile;
		}

		for (uint8_t x = 0; x < 8; x++) {
			dimLed(x + 1, warp[x]);
		}

		ledDelay();
	}

	/* Alle Leds runterdimmen */

	for (uint8_t x = 0; x < 8; x++) {
		while (warp[x] > 0) {
			warp[x]--;
			dimLed(x + 1, warp[x]);
		}
	}
}

void warpLed3(uint32_t laufZeit) {
	uint32_t beginnZeit = HAL_GetTick();
	uint16_t warp[8];
	for (uint16_t y = 0; y < 1024; y++) {
		for (uint8_t x = 0; x < 8; x++) {
			warp[x] = y;
			dimLed(x + 1, warp[x]);
		}
	}

	uint16_t teile = 0;
	uint8_t standPunkt = 0;

	while (HAL_GetTick() - beginnZeit < laufZeit) {
		teile += 31;
		if (teile > 1984) {
			teile = 0;
			standPunkt++;

			if (standPunkt > 16) {
				standPunkt = 1;
			}

		}

		if (standPunkt == 0) {
			warp[0] += teile;
		}

		if (standPunkt == 1) {
			warp[1] += teile;
		}
		if (standPunkt == 2) {
			warp[0] -= teile;
			warp[2] += teile;
		}
		if (standPunkt == 3) {
			warp[1] -= teile;
			warp[3] += teile;
		}
		if (standPunkt == 4) {
			warp[2] -= teile;
			warp[4] += teile;
		}
		if (standPunkt == 5) {
			warp[3] -= teile;
			warp[5] += teile;
			warp[6] = 1023;
		}
		if (standPunkt == 6) {
			warp[4] -= teile;
			warp[6] += teile;
		}
		if (standPunkt == 7) {
			warp[5] -= teile;
			warp[7] += teile;
		}
		if (standPunkt == 8) {
			warp[6] -= teile;
		}
		if (standPunkt == 9) {
			warp[6] += teile;
		}
		if (standPunkt == 10) {
			warp[7] -= teile;
			warp[5] += teile;
		}

		if (standPunkt == 11) {
			warp[6] -= teile;
			warp[4] += teile;
		}
		if (standPunkt == 12) {
			warp[5] -= teile;
			warp[3] += teile;
		}
		if (standPunkt == 13) {
			warp[4] -= teile;
			warp[2] += teile;
		}
		if (standPunkt == 14) {
			warp[3] -= teile;
			warp[1] += teile;
		}
		if (standPunkt == 15) {
			warp[2] -= teile;
			warp[0] += teile;
		}
		if (standPunkt == 16) {
			warp[1] -= teile;
		}

		for (uint8_t x = 0; x < 8; x++) {
			dimLed(x + 1, warp[x]);
		}

//HAL_Delay(pauseDelay);

		ledDelay();
	}
	/* Alle Leds runterdimmen */

	for (uint8_t x = 0; x < 8; x++) {
		while (warp[x] > 0) {
			warp[x]--;
			dimLed(x + 1, warp[x]);
		}
	}
}




/**
 * @brief  Einfache Routine um die LEDs einzeln oder mehrere
 * 		   durch das zweite übertragene I2C Byte zu steuern
 * @param  werden durch I2C, Befehl 0x25 (in aRxBuffer[0]) gesetzt:
 * 		   aRxBuffer[1]: gibt an welche LEDs an oder aus
 * 		   aRxBuffer[2]: gibt an wie hell
 * @note
 * @retval None
 */
void gauselMann(void) {

	unsigned int helligkeit = (aRxBuffer[2] << 8 | 0x00);
	for (uint8_t x = 0; x < 9; x++) {
		if (aRxBuffer[1] & (0x01 << x)) {
			dimLed(x + 1, helligkeit); //(aRxBuffer[1] & (0x01 << x))*helligkeit);
		} else if (!(aRxBuffer[1] & (0x01 << x))) {
			dimLed(x + 1, 0);
		}

	}

}

/**
 * @brief  Einfache Routine um die LEDs zu steuern durch den Timer
 * @param  nummer: 1-8 helligkeit 0x00 - 0xFF
 * @note
 * @retval None
 */
void dimLed(unsigned int nummer, unsigned int helligkeit) {
	if (energieSparen == 1) {
		helligkeit = 0;
	}

	switch (nummer) {
	case 0:
		TIM2->CCR1 = helligkeit;
		TIM2->CCR2 = helligkeit;
		TIM2->CCR3 = helligkeit;
		TIM3->CCR1 = helligkeit;
		TIM3->CCR2 = helligkeit;
		TIM3->CCR3 = helligkeit;
		TIM3->CCR4 = helligkeit;
		TIM5->CCR1 = helligkeit;
		break;
	case 1:
		TIM5->CCR1 = helligkeit;
		break;
	case 2:
		TIM3->CCR4 = helligkeit;
		break;
	case 3:
		TIM3->CCR3 = helligkeit;
		break;
	case 4:
		TIM3->CCR2 = helligkeit;
		break;
	case 5:
		TIM3->CCR1 = helligkeit;
		break;
	case 6:
		TIM2->CCR3 = helligkeit;
		break;
	case 7:
		TIM2->CCR2 = helligkeit;
		break;
	case 8:
		TIM2->CCR1 = helligkeit;
		break;
	}
}

/**
 * @brief  Tx Transfer completed callback.
 * @param  I2cHandle: I2C handle.
 * @note   Switcht vom Sender wieder zum Reciever
 * @retval None
 */

void HAL_I2C_SlaveTxCpltCallback(I2C_HandleTypeDef *I2cHandle) {
	if (HAL_I2C_Slave_Receive_DMA(&hi2c1, aRxBuffer, 3) != HAL_OK) {
		Error_Handler();
	}
}

/**
 * @brief  Rx Transfer completed callback.
 * @param  I2cHandle: I2C handle
 * @note   Schaltet nach Empfang wieder in Slave Mode und setzt Flag recieveComplete
 * @retval None
 */
void HAL_I2C_SlaveRxCpltCallback(I2C_HandleTypeDef *I2cHandle) {
	/* Jetzt verschick mal wieder Modus*/
	if (HAL_I2C_Slave_Transmit_DMA(&hi2c1, aTxBuffer, 2) != HAL_OK) {
		Error_Handler();
	}
	/* kucken bis er wirklich in Slave Mode is */
	while (HAL_I2C_GetMode(&hi2c1) != HAL_I2C_MODE_SLAVE) {
	}
	recieveComplete = 1;
}

/**
 * @brief  I2C error callbacks.
 * @param  I2cHandle: I2C handle
 * @note   This example shows a simple way to report transfer error, and you can
 *         add your own implementation.
 * @retval None
 */
void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *I2cHandle) {
	/** Error_Handler() function is called when error occurs.
	 * 1- When Slave doesn't acknowledge its address, Master restarts communication.
	 * 2- When Master doesn't acknowledge the last data transferred, Slave doesn't care in this example.
	 */
	if (HAL_I2C_GetError(I2cHandle) != HAL_I2C_ERROR_AF) {
		Error_Handler();
	}
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	HAL_GPIO_WritePin(userLed_GPIO_Port, userLed_Pin, 0);
	__disable_irq();
	while (1) {
		HAL_GPIO_TogglePin(userLed_GPIO_Port, userLed_Pin);
		HAL_Delay(200);
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
