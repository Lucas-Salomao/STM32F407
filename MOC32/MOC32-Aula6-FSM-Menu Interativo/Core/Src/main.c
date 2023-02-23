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
SRAM_HandleTypeDef hsram1;

/* USER CODE BEGIN PV */
uint16_t tftID = 0;
uint32_t fsmLastMenuState;
uint32_t buttonStatus = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_FSMC_Init(void);
/* USER CODE BEGIN PFP */
void tftMenuColors(int menuColorsValue);
void tftMenuWrite(int menuWriteValue);
void encoderInit();
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
struct State {
	  uint32_t out; //Saída para o estado
	  //uint32_t wait; //Tempo de espera do estado
	  uint32_t next[5]; //Vetor de próximos estados
};
typedef const struct State tipoS;
//Apelidos para referenciar os estados da FSM
#define MENUPRINCIPAL 0
#define MENU1 1
#define MENU2 2
#define MENU3 3
#define MENU4 4

//Estrutura de dados que corresponde ao diagrama de transição de estados da FSM
tipoS Fsm[5] = {
		[MENUPRINCIPAL] =      {0,  {MENU1, MENU1, MENU2, MENU3, MENUPRINCIPAL}},
		[MENU1] =              {1,  {MENU1, MENU1, MENU1, MENU1, MENUPRINCIPAL}},
		[MENU2] =              {2,  {MENU2, MENU2, MENU2, MENU2, MENUPRINCIPAL}},
		[MENU3] =              {3,  {MENU3, MENU3, MENU3, MENU4, MENUPRINCIPAL}},
		[MENU4] =              {4,  {MENU4, MENU4, MENU4, MENU4, MENU3}}
};

uint32_t fsmMenuState; //Estado atual é Par ou �?mpar

menuWriteValue = 1;
menuColorsValue = 0;
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
  MX_FSMC_Init();
  /* USER CODE BEGIN 2 */
  ili9341_Init();
  ili9341_FillScreen(COLOR_BLACK);

  fsmMenuState = MENUPRINCIPAL; //Estado inicial
  tftMenuWrite(Fsm[fsmMenuState].out);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  encoderInit();
	  //1. Saída baseada no estado atual
	  if (fsmMenuState != fsmLastMenuState)
	  {
		  tftMenuWrite(Fsm[fsmMenuState].out);
		  fsmLastMenuState = fsmMenuState;
	  }
	  //3. Lê a entrada
	  if(HAL_GPIO_ReadPin(BTN_OK_GPIO_Port, BTN_OK_Pin) == GPIO_PIN_RESET)
	  {
		  buttonStatus = menuColorsValue;
		  fsmMenuState = Fsm[fsmMenuState].next[buttonStatus];
		  tftMenuWrite(Fsm[fsmMenuState].out);
	  }
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
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pins : OUTA_Pin OUTB_Pin */
  GPIO_InitStruct.Pin = OUTA_Pin|OUTB_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : BTN_OK_Pin */
  GPIO_InitStruct.Pin = BTN_OK_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(BTN_OK_GPIO_Port, &GPIO_InitStruct);

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
void tftMenuColors(int menuColorsValue)
{
	switch (menuColorsValue)
	{
	case 1:
		ili9341_FillRect(20, 20, 200, 50, COLOR_GREEN);
		break;
	case 2:
		ili9341_FillRect(20, 90, 200, 50, COLOR_GREEN);
		break;
	case 3:
		ili9341_FillRect(20, 160, 200, 50, COLOR_GREEN);
		break;
	case 4:
		ili9341_FillRect(20, 230, 200, 50, COLOR_GREEN);
		break;
	}
}

void tftMenuWrite(int menuWriteValue)
{
	switch (menuWriteValue)
	{
	case 0:
		ili9341_FillRect(20, 20, 200, 50, COLOR_CYAN);
		ili9341_FillRect(20, 90, 200, 50, COLOR_CYAN);
		ili9341_FillRect(20, 160, 200, 50, COLOR_CYAN);
		ili9341_FillRect(20, 230, 200, 50, COLOR_CYAN);
		tftMenuColors(menuColorsValue);
		tftWriteText(80, 50, COLOR_BLACK, &mono12x7bold, 1, (uint8_t *)"MENU 1");
		tftWriteText(80, 120, COLOR_BLACK, &mono12x7bold, 1, (uint8_t *)"MENU 2");
		tftWriteText(80, 190, COLOR_BLACK, &mono12x7bold, 1, (uint8_t *)"MENU 3");
		tftWriteText(80, 255, COLOR_BLACK, &mono12x7bold, 1, (uint8_t *)"VOLTAR");
		break;
	case 1:
		ili9341_FillRect(20, 20, 200, 50, COLOR_YELLOW);
		ili9341_FillRect(20, 90, 200, 50, COLOR_YELLOW);
		ili9341_FillRect(20, 160, 200, 50, COLOR_YELLOW);
		ili9341_FillRect(20, 230, 200, 50, COLOR_YELLOW);
		tftMenuColors(menuColorsValue);
		tftWriteText(50, 50, COLOR_BLACK, &mono12x7bold, 1, (uint8_t *)"MENU 1 - A");
		tftWriteText(50, 120, COLOR_BLACK, &mono12x7bold, 1, (uint8_t *)"MENU 1 - B");
		tftWriteText(50, 190, COLOR_BLACK, &mono12x7bold, 1, (uint8_t *)"MENU 1 - C");
		tftWriteText(80, 255, COLOR_BLACK, &mono12x7bold, 1, (uint8_t *)"VOLTAR");
		break;
	case 2:
		ili9341_FillRect(20, 20, 200, 50, COLOR_BLUE);
		ili9341_FillRect(20, 90, 200, 50, COLOR_BLUE);
		ili9341_FillRect(20, 160, 200, 50, COLOR_BLUE);
		ili9341_FillRect(20, 230, 200, 50, COLOR_BLUE);
		tftMenuColors(menuColorsValue);
		tftWriteText(50, 50, COLOR_BLACK, &mono12x7bold, 1, (uint8_t *)"MENU 2 - A");
		tftWriteText(50, 120, COLOR_BLACK, &mono12x7bold, 1, (uint8_t *)"MENU 2 - B");
		tftWriteText(50, 190, COLOR_BLACK, &mono12x7bold, 1, (uint8_t *)"MENU 2 - C");
		tftWriteText(80, 255, COLOR_BLACK, &mono12x7bold, 1, (uint8_t *)"VOLTAR");
		break;
	case 3:
		ili9341_FillRect(20, 20, 200, 50, COLOR_RED);
		ili9341_FillRect(20, 90, 200, 50, COLOR_RED);
		ili9341_FillRect(20, 160, 200, 50, COLOR_RED);
		ili9341_FillRect(20, 230, 200, 50, COLOR_RED);
		tftMenuColors(menuColorsValue);
		tftWriteText(50, 50, COLOR_BLACK, &mono12x7bold, 1, (uint8_t *)"MENU 3 - A");
		tftWriteText(50, 120, COLOR_BLACK, &mono12x7bold, 1, (uint8_t *)"MENU 3 - B");
		tftWriteText(50, 190, COLOR_BLACK, &mono12x7bold, 1, (uint8_t *)"MENU 3 - C");
		tftWriteText(80, 255, COLOR_BLACK, &mono12x7bold, 1, (uint8_t *)"VOLTAR");
		break;
	case 4:
		ili9341_FillRect(20, 20, 200, 50, COLOR_RED);
		ili9341_FillRect(20, 90, 200, 50, COLOR_RED);
		ili9341_FillRect(20, 160, 200, 50, COLOR_RED);
		ili9341_FillRect(20, 230, 200, 50, COLOR_RED);
		tftMenuColors(menuColorsValue);
		tftWriteText(40, 50, COLOR_BLACK, &mono12x7bold, 1, (uint8_t *)"MENU 3 - CA");
		tftWriteText(40, 120, COLOR_BLACK, &mono12x7bold, 1, (uint8_t *)"MENU 3 - CB");
		tftWriteText(40, 190, COLOR_BLACK, &mono12x7bold, 1, (uint8_t *)"MENU 3 - CC");
		tftWriteText(80, 255, COLOR_BLACK, &mono12x7bold, 1, (uint8_t *)"VOLTAR");
		break;
	}
}

void encoderInit()
{
	if (HAL_GPIO_ReadPin(OUTA_GPIO_Port, OUTA_Pin) == GPIO_PIN_RESET)  // If the OUTA is RESET
	{
		if (HAL_GPIO_ReadPin(OUTA_GPIO_Port, OUTB_Pin) == GPIO_PIN_RESET)  // If OUTB is also reset... CCK
		{
			while (HAL_GPIO_ReadPin(OUTA_GPIO_Port, OUTB_Pin) == GPIO_PIN_RESET){};  // wait for the OUTB to go high
			menuColorsValue--;
			while (HAL_GPIO_ReadPin(OUTA_GPIO_Port, OUTA_Pin) == GPIO_PIN_RESET){};  // wait for the OUTA to go high
			if (menuColorsValue <= 1){menuColorsValue = 1;}
			if (menuColorsValue >= 4){menuColorsValue = 4;}
			tftMenuColors(menuColorsValue);
			tftMenuWrite(Fsm[fsmMenuState].out);
			HAL_Delay (10);  // wait for some more time
		}

		else if (HAL_GPIO_ReadPin(OUTA_GPIO_Port, OUTB_Pin) == GPIO_PIN_SET)  // If OUTB is also set
		{
			while (HAL_GPIO_ReadPin(OUTA_GPIO_Port, OUTB_Pin) == GPIO_PIN_SET){};  // wait for the OUTB to go LOW.. CK
			menuColorsValue++;
			while (HAL_GPIO_ReadPin(OUTA_GPIO_Port, OUTA_Pin) == GPIO_PIN_RESET){};  // wait for the OUTA to go high
			while (HAL_GPIO_ReadPin(OUTA_GPIO_Port, OUTB_Pin) == GPIO_PIN_RESET){};  // wait for the OUTB to go high
			if (menuColorsValue <= 1){menuColorsValue = 1;}
			if (menuColorsValue >= 4){menuColorsValue = 4;}
			tftMenuColors(menuColorsValue);
			tftMenuWrite(Fsm[fsmMenuState].out);
			HAL_Delay (10);  // wait for some more time
		}
		//uint8_t MSG[48] = {"\0"};
		//sprintf(MSG, "menuColorsValue: %d\r\n", menuColorsValue);
		//(&huart2, MSG, sizeof(MSG), 10);
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
