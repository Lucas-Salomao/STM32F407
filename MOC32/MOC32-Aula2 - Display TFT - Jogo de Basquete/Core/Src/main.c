/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
#include "ili9341.h"
#include "common.h"
#include "fonts.h"
#include "lcd.h"
#include "math.h"
#include <stdbool.h>

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
ADC_HandleTypeDef hadc1;

UART_HandleTypeDef huart2;

SRAM_HandleTypeDef hsram1;

/* USER CODE BEGIN PV */
volatile char dataReceivedInTheRx[10];
volatile int ballsXAxis[1] = {288};
volatile int ballsYAxis[1] = {80};
volatile uint32_t flagReceivedDataInTheRx = 0;
extern const unsigned short gameImage[54720];
extern const unsigned short StrengthAimImage[19840];
extern const unsigned short gameOverImage[72000];
extern const unsigned short youWinImage[57600];
volatile int strengthBarAimValue;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_FSMC_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */
void initImageBall();
void initStrengthBar();
void animationHitThePitch();
void strongThrowAnimation();
void weakThrowAnimation();
void checkingRxData();
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
uint32_t flagEnableStrengthBar = 1;
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
  MX_ADC1_Init();
  MX_FSMC_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
  HAL_UART_Receive_IT(&huart2, dataReceivedInTheRx, 1);
  ili9341_Init();
  ili9341_FillScreen(COLOR_BLACK); // fundo com tela preta
  ili9341_DrawRGBImage(0, 0, 320, 171, gameImage); // carrega a imagem do game
  ili9341_DrawRGBImage(0, 180, 320, 62, StrengthAimImage);

  initImageBall(); // inicializa a função criar bola de basquete
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  initStrengthBar();
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
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 168;
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
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_8B;
  hadc1.Init.ScanConvMode = ENABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 2;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_480CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_11;
  sConfig.Rank = 2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

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
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin : BOTAO_Pin */
  GPIO_InitStruct.Pin = BOTAO_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(BOTAO_GPIO_Port, &GPIO_InitStruct);

}

/* FSMC initialization function */
static void MX_FSMC_Init(void)
{

  /* USER CODE BEGIN FSMC_Init 0 */

  /* USER CODE END FSMC_Init 0 */

  FSMC_NORSRAM_TimingTypeDef Timing = {0};

  /* USER CODE BEGIN FSMC_Init 1 */

  /* USER CODE END FSMC_Init 1 */

  /** Perform the SRAM1 memory initialization sequence
  */
  hsram1.Instance = FSMC_NORSRAM_DEVICE;
  hsram1.Extended = FSMC_NORSRAM_EXTENDED_DEVICE;
  /* hsram1.Init */
  hsram1.Init.NSBank = FSMC_NORSRAM_BANK1;
  hsram1.Init.DataAddressMux = FSMC_DATA_ADDRESS_MUX_DISABLE;
  hsram1.Init.MemoryType = FSMC_MEMORY_TYPE_SRAM;
  hsram1.Init.MemoryDataWidth = FSMC_NORSRAM_MEM_BUS_WIDTH_8;
  hsram1.Init.BurstAccessMode = FSMC_BURST_ACCESS_MODE_DISABLE;
  hsram1.Init.WaitSignalPolarity = FSMC_WAIT_SIGNAL_POLARITY_LOW;
  hsram1.Init.WrapMode = FSMC_WRAP_MODE_DISABLE;
  hsram1.Init.WaitSignalActive = FSMC_WAIT_TIMING_BEFORE_WS;
  hsram1.Init.WriteOperation = FSMC_WRITE_OPERATION_ENABLE;
  hsram1.Init.WaitSignal = FSMC_WAIT_SIGNAL_DISABLE;
  hsram1.Init.ExtendedMode = FSMC_EXTENDED_MODE_DISABLE;
  hsram1.Init.AsynchronousWait = FSMC_ASYNCHRONOUS_WAIT_DISABLE;
  hsram1.Init.WriteBurst = FSMC_WRITE_BURST_DISABLE;
  hsram1.Init.PageSize = FSMC_PAGE_SIZE_NONE;
  /* Timing */
  Timing.AddressSetupTime = 6;
  Timing.AddressHoldTime = 15;
  Timing.DataSetupTime = 6;
  Timing.BusTurnAroundDuration = 15;
  Timing.CLKDivision = 16;
  Timing.DataLatency = 17;
  Timing.AccessMode = FSMC_ACCESS_MODE_A;
  /* ExtTiming */

  if (HAL_SRAM_Init(&hsram1, &Timing, NULL) != HAL_OK)
  {
    Error_Handler( );
  }

  /* USER CODE BEGIN FSMC_Init 2 */

  /* USER CODE END FSMC_Init 2 */
}

/* USER CODE BEGIN 4 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	HAL_UART_Receive_IT(&huart2, dataReceivedInTheRx, 1);
	flagReceivedDataInTheRx = 1;
}

void checkingRxData()
{
	if (flagReceivedDataInTheRx) // verifica se chegou algo na porta serial
	{
		flagReceivedDataInTheRx = 0;
		if(strengthBarAimValue >= 130 && strengthBarAimValue <= 170)
		{
			animationHitThePitch();
			ili9341_FillScreen(COLOR_BLACK); // fundo com tela preta
			ili9341_DrawRGBImage(40, 0, 240, 240, youWinImage); // carrega a imagem do game
			HAL_Delay(2000);
			NVIC_SystemReset();
		}
		if(strengthBarAimValue < 130)
		{
			strongThrowAnimation();
			ili9341_FillScreen(COLOR_BLACK); // fundo com tela preta
			ili9341_DrawRGBImage(10, 0, 300, 240, gameOverImage); // carrega a imagem do game
			HAL_Delay(2000);
			NVIC_SystemReset();
		}
		if(strengthBarAimValue >170)
		{
			weakThrowAnimation();
			ili9341_FillScreen(COLOR_BLACK); // fundo com tela preta
			ili9341_DrawRGBImage(10, 0, 300, 240, gameOverImage); // carrega a imagem do game
			HAL_Delay(2000);
			NVIC_SystemReset();
		}
	}
}

void initImageBall(void) // função criar bola de basquete
{
	ili9341_DrawCircle(ballsXAxis[0], ballsYAxis[0], 1, COLOR_ORANGE);
	ili9341_DrawCircle(ballsXAxis[0], ballsYAxis[0], 2, COLOR_ORANGE);
	ili9341_DrawCircle(ballsXAxis[0], ballsYAxis[0], 3, COLOR_ORANGE);
	ili9341_DrawCircle(ballsXAxis[0], ballsYAxis[0], 4, COLOR_ORANGE);
}

void initStrengthBar() // função iniciar barra de força
{
	if (strengthBarAimValue < 290)
	{
		for (int i = 4; i <= 300; i = i + 10)
		{
			strengthBarAimValue = i;
			ili9341_DrawRGBImage(0, 180, 320, 62, StrengthAimImage);
			ili9341_FillRect(strengthBarAimValue, 178, 20, 62, COLOR_BLACK);
			checkingRxData();
			HAL_Delay(50);
		}
	} else {
		for (int i = 310; i >= 10; i = i - 10)
		{
			strengthBarAimValue = i;
			ili9341_DrawRGBImage(0, 180, 320, 62, StrengthAimImage);
			ili9341_FillRect(strengthBarAimValue, 178, 20, 62, COLOR_BLACK);
			checkingRxData();
			HAL_Delay(50);
		}
	}
}

void animationHitThePitch()
{
	int count = 1;
	while(count <= 11)
	{
		if(ballsXAxis[0] > 175)
		{
			ili9341_DrawRGBImage(0, 0, 320, 171, gameImage);
			ballsXAxis[0] = ballsXAxis[0] - 19;
			ballsYAxis[0] = ballsYAxis[0] - 11;
			initImageBall();
			HAL_Delay(300);
		}
		if(ballsXAxis[0] < 175 && ballsXAxis[0] > 69)
		{
			ili9341_DrawRGBImage(0, 0, 320, 171, gameImage);
			ballsXAxis[0] = ballsXAxis[0] - 21;
			ballsYAxis[0] = ballsYAxis[0] + 14;
			initImageBall();
			HAL_Delay(300);
		}
		count++;
	}
}

void strongThrowAnimation()
{
	int count = 1;
	while(count <= 14)
	{
		if(ballsXAxis[0] > 0)
		{
			ili9341_DrawRGBImage(0, 0, 320, 171, gameImage);
			ballsXAxis[0] = ballsXAxis[0] - 20;
			ballsYAxis[0] = ballsYAxis[0] - 4;
			initImageBall();
			HAL_Delay(100);
		}
		count++;
	}
}
void weakThrowAnimation()
{
	int count = 1;
	while(count <= 15)
	{
		if(ballsXAxis[0] > 200)
		{
			ili9341_DrawRGBImage(0, 0, 320, 171, gameImage);
			ballsXAxis[0] = ballsXAxis[0] - 19;
			ballsYAxis[0] = ballsYAxis[0] - 11;
			initImageBall();
			HAL_Delay(300);
		}
		if(ballsXAxis[0] < 200 && ballsXAxis[0] > 30)
		{
			ili9341_DrawRGBImage(0, 0, 320, 171, gameImage);
			ballsXAxis[0] = ballsXAxis[0] - 21;
			ballsYAxis[0] = ballsYAxis[0] + 14;
			initImageBall();
			HAL_Delay(300);
		}
		count++;
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
