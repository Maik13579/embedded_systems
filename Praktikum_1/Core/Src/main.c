/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

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
TIM_HandleTypeDef htim16;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM16_Init(void);
/* USER CODE BEGIN PFP */

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

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART2_UART_Init();
  MX_TIM16_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_Base_Start_IT(&htim16);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  while (1)
  {
	// turn on LEDs
	if(counter >= 5)HAL_GPIO_WritePin(LED_red_GPIO_Port, LED_red_Pin, GPIO_PIN_SET);
	else HAL_GPIO_WritePin(LED_red_GPIO_Port, LED_red_Pin, GPIO_PIN_RESET);

	if(counter >= 10)HAL_GPIO_WritePin(LED_yellow_GPIO_Port, LED_yellow_Pin, GPIO_PIN_SET);
	else HAL_GPIO_WritePin(LED_yellow_GPIO_Port, LED_yellow_Pin, GPIO_PIN_RESET);

	if(counter >= 25)HAL_GPIO_WritePin(LED_green_GPIO_Port, LED_green_Pin, GPIO_PIN_SET);
	else HAL_GPIO_WritePin(LED_green_GPIO_Port, LED_green_Pin, GPIO_PIN_RESET);


	//print matrix in terminal
	uint8_t row1[35] = {'\0'};
	sprintf(row1, "%1c %1c %1c %1c \r\n", 'O', 'O', 'O', 'O');
	uint8_t row2[35] = {'\0'};
	sprintf(row2, "%1c %1c %1c %1c \r\n", 'O', 'O', 'O', 'O');
	uint8_t row3[35] = {'\0'};
	sprintf(row3, "%1c %1c %1c %1c \r\n", 'O', 'O', 'O', 'O');
	uint8_t row4[35] = {'\0'};
	sprintf(row4, "%1c %1c %1c %1c \r\n\n\n", 'O', 'O', 'O', 'O');

	HAL_UART_Transmit(&huart2, row1, sizeof(row1), 100);
	HAL_UART_Transmit(&huart2, row2, sizeof(row2), 100);
	HAL_UART_Transmit(&huart2, row3, sizeof(row3), 100);
	HAL_UART_Transmit(&huart2, row4, sizeof(row4), 100);

	HAL_Delay(2000);

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
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 10;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
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
}

/**
  * @brief TIM16 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM16_Init(void)
{

  /* USER CODE BEGIN TIM16_Init 0 */

  /* USER CODE END TIM16_Init 0 */

  /* USER CODE BEGIN TIM16_Init 1 */

//80MHz / 8000 = 10 kHz -> 1 tick every 0.0001 sec
//0.0001 sec * 100 = 0.01 sec -> timer interrupt every 10 ms

  /* USER CODE END TIM16_Init 1 */
  htim16.Instance = TIM16;
  htim16.Init.Prescaler = 8000-1;
  htim16.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim16.Init.Period = 100-1;
  htim16.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim16.Init.RepetitionCounter = 0;
  htim16.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim16) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM16_Init 2 */

  /* USER CODE END TIM16_Init 2 */

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
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, LED_green_Pin|LED_yellow_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LED_red_Pin|matrix_1_Pin|matrix_2_Pin|matrix_3_Pin
                          |matrix_4_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LED_green_Pin LED_yellow_Pin */
  GPIO_InitStruct.Pin = LED_green_Pin|LED_yellow_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : Button_down_Pin Button_up_Pin */
  GPIO_InitStruct.Pin = Button_down_Pin|Button_up_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LED_red_Pin matrix_1_Pin matrix_2_Pin matrix_3_Pin
                           matrix_4_Pin */
  GPIO_InitStruct.Pin = LED_red_Pin|matrix_1_Pin|matrix_2_Pin|matrix_3_Pin
                          |matrix_4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : matrix_5_Pin matrix_6_Pin matrix_7_Pin matrix_8_Pin */
  GPIO_InitStruct.Pin = matrix_5_Pin|matrix_6_Pin|matrix_7_Pin|matrix_8_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
/* USER CODE BEGIN 4 */

uint16_t read_matrix(void){
	uint16_t matrix = 0;
		// 1----5---9---12
		// |	|	|   |
		// 2----6---10--13
		// |	|	|   |
		// 3----7---11--14
		// |	|	|   |
		// 4----8---12--15
	//set 1 to High
	HAL_GPIO_WritePin(matrix_1_GPIO_Port, matrix_1_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(matrix_2_GPIO_Port, matrix_2_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(matrix_3_GPIO_Port, matrix_3_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(matrix_4_GPIO_Port, matrix_4_Pin, GPIO_PIN_RESET);

	//read pins 5 to 8
	if(HAL_GPIO_ReadPin(matrix_5_GPIO_Port, matrix_5_Pin)) matrix |= (1<<0);
	else matrix &= ~(1<<0);
	if(HAL_GPIO_ReadPin(matrix_6_GPIO_Port, matrix_6_Pin)) matrix |= (1<<1);
	else matrix &= ~(1<<1);
	if(HAL_GPIO_ReadPin(matrix_7_GPIO_Port, matrix_7_Pin)) matrix |= (1<<2);
	else matrix &= ~(1<<2);
	if(HAL_GPIO_ReadPin(matrix_8_GPIO_Port, matrix_8_Pin)) matrix |= (1<<3);
	else matrix &= ~(1<<3);

	//set 2 to High
	HAL_GPIO_WritePin(matrix_1_GPIO_Port, matrix_1_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(matrix_2_GPIO_Port, matrix_2_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(matrix_3_GPIO_Port, matrix_3_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(matrix_4_GPIO_Port, matrix_4_Pin, GPIO_PIN_RESET);

	//read pins 5 to 8
	if(HAL_GPIO_ReadPin(matrix_5_GPIO_Port, matrix_5_Pin)) matrix |= (1<<4);
	else matrix &= ~(1<<4);
	if(HAL_GPIO_ReadPin(matrix_6_GPIO_Port, matrix_6_Pin)) matrix |= (1<<5);
	else matrix &= ~(1<<5);
	if(HAL_GPIO_ReadPin(matrix_7_GPIO_Port, matrix_7_Pin)) matrix |= (1<<6);
	else matrix &= ~(1<<6);
	if(HAL_GPIO_ReadPin(matrix_8_GPIO_Port, matrix_8_Pin)) matrix |= (1<<7);
	else matrix &= ~(1<<7);

	//set 3 to High
	HAL_GPIO_WritePin(matrix_1_GPIO_Port, matrix_1_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(matrix_2_GPIO_Port, matrix_2_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(matrix_3_GPIO_Port, matrix_3_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(matrix_4_GPIO_Port, matrix_4_Pin, GPIO_PIN_RESET);

	//read pins 5 to 8
	if(HAL_GPIO_ReadPin(matrix_5_GPIO_Port, matrix_5_Pin)) matrix |= (1<<8);
	else matrix &= ~(1<<8);
	if(HAL_GPIO_ReadPin(matrix_6_GPIO_Port, matrix_6_Pin)) matrix |= (1<<9);
	else matrix &= ~(1<<9);
	if(HAL_GPIO_ReadPin(matrix_7_GPIO_Port, matrix_7_Pin)) matrix |= (1<<10);
	else matrix &= ~(1<<10);
	if(HAL_GPIO_ReadPin(matrix_8_GPIO_Port, matrix_8_Pin)) matrix |= (1<<11);
	else matrix &= ~(1<<11);

	//set 4 to High
	HAL_GPIO_WritePin(matrix_1_GPIO_Port, matrix_1_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(matrix_2_GPIO_Port, matrix_2_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(matrix_3_GPIO_Port, matrix_3_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(matrix_4_GPIO_Port, matrix_4_Pin, GPIO_PIN_SET);

	//read pins 5 to 8
	if(HAL_GPIO_ReadPin(matrix_5_GPIO_Port, matrix_5_Pin)) matrix |= (1<<12);
	else matrix &= ~(1<<12);
	if(HAL_GPIO_ReadPin(matrix_6_GPIO_Port, matrix_6_Pin)) matrix |= (1<<13);
	else matrix &= ~(1<<13);
	if(HAL_GPIO_ReadPin(matrix_7_GPIO_Port, matrix_7_Pin)) matrix |= (1<<14);
	else matrix &= ~(1<<14);
	if(HAL_GPIO_ReadPin(matrix_8_GPIO_Port, matrix_8_Pin)) matrix |= (1<<15);
	else matrix &= ~(1<<15);

	return matrix;
}


uint8_t button_state; //states for 8 buttons. bit = 1 -> button is pressed
uint8_t counter0 =0xFF, counter1  =0xFF;//8 * two bit counter
uint8_t button_pin; //pins for 8 buttons

// same like above only for 4x4 button matrix
uint16_t counter0_matrix =0xFF, counter1_matrix= 0xFF;
uint16_t matrix_state;
uint16_t matrix_pin;

//Callback: timer has reset
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){
	if(htim == &htim16){
		uint8_t button_changed;

		if(HAL_GPIO_ReadPin(Button_up_GPIO_Port, Button_up_Pin)) button_pin |= (1<<button_up);
		else button_pin &= ~(1<<button_up);
		if(HAL_GPIO_ReadPin(Button_down_GPIO_Port, Button_down_Pin)) button_pin |= (1<<button_down);
		else button_pin &= ~(1<<button_down);

		button_changed = button_state ^ button_pin; //bit = 1 -> button changed

		// count to 4 or reset if button_changed != 1	Round:	1	|	2	|	3	|	4	|
		counter0 = ~(counter0 & button_changed);		//0	|	1	|	0	|	1	|
		counter1 = counter0 ^ (counter1 & button_changed);	//1	|	0	|	0	|	1	|

		button_changed &= counter0 & counter1; //change button state only if timer rolls over!
		button_state ^= button_changed; //toggle state

		//count how long button up is pressed in 10 ms
		if(button_state & (1<<button_up)) counter_up++;

		//count how long button down is pressed in 10 ms
		if(button_state & (1<<button_down)) counter_down++;

		if(~button_state & button_changed & (1<<button_up)){// stopped pressing button up
			counter += (counter_up >= 100) ? 10 : 1; //increase counter with 10 if button is pressed longer than 1 sec, else increase 1
			counter_up = 0; //reset counter
		}

		if(~button_state & button_changed & (1<<button_down)){// stopped pressing button down
			counter -= (counter_down >= 100) ? 10 : 1; //decrease counter with 10 if button is pressed longer than 1 sec, else decrease 1
			counter_down = 0; //reset counter
		}


		/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
		//////////////4X4 Button Matrix//////////////////////////////////////////////////////////////////////////////////////////////////////////
		/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

		// 1	2	3	4
		// |	|	|	|
		// +----+---+---+-----5
		// |	|	|	|
		// +----+---+---+-----6
		// |	|	|	|
		// +----+---+---+-----7
		// |	|	|	|
		// +----+---+---+-----8
		uint16_t matrix_changed;

		//Todo: read matrix into matrix_pin
		matrix_pin = read_matrix();

		matrix_changed = matrix_state ^ matrix_pin;
		counter0_matrix = ~(counter0_matrix & matrix_changed);
		counter1_matrix = counter0_matrix ^ (counter1_matrix & matrix_changed);

		matrix_changed &= counter0_matrix & counter1_matrix;
		matrix_state ^= matrix_changed;

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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
