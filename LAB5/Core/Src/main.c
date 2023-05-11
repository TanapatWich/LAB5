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
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
uint8_t key[1];
uint8_t tx_buf[100];
float LEDspd = 50;
uint8_t state = 0;
uint8_t ledfreq = 10;
uint8_t startup = 0;
uint16_t led_buf[2];
uint8_t str[110];
uint8_t Ispress = 1;
uint8_t LEDIson = 1;
uint8_t ButtonPress = 0;
static uint32_t timestamp =0;
static uint32_t timestamp2 =0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */
void uart_poll();
void uart_it_conf();
void LEDTask();
void stkuy();

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
  /* USER CODE BEGIN 2 */
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  HAL_UART_Receive_IT(&huart2, key, 1);
	  LEDTask();
	  stkuy();
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
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
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
  huart2.Init.BaudRate = 57600;
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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){Ispress = 0;}

void LEDTask(){


	if(HAL_GetTick()>=timestamp && LEDIson == 1){
		timestamp = HAL_GetTick()+LEDspd;
		HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);

	}

	if(LEDIson == 0){
			HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, 0);
		}
}

void stkuy(){
	switch(state){
	case 0:
			if(startup == 0){
				HAL_UART_Transmit_IT(&huart2,"\n\rMenu\r\nPress 0 To Enter LED Control Mode\r\nPress 1 To Enter Button Status\r\n", 75 );
				startup = 1;
			}
			if(Ispress == 0 && key[0] == '0'){
				Ispress = 1;
				HAL_UART_Transmit_IT(&huart2,"\n\rMenu\r\nPress a To Speed Up\r\nPress s To Speed Down\nPress d To Turn On/Off LED\r\nPress x To Back To Menu\r\n", 104);
				state = 1;
			}
			else if(Ispress == 0 && key[0] == '1'){
				Ispress = 1;
				HAL_UART_Transmit_IT(&huart2,"\n\rMenu\r\nx : Back\r\n", 18);
				state = 2;
			}
			else if(Ispress == 0 && key[0] != '1' && key[0] != '0'){
				Ispress = 1;
				HAL_UART_Transmit_IT(&huart2,"Not in menu\r\n", 13);
			}
			break;
	case 1:// Speed Con
			//back
			if(Ispress == 0 && key[0] == 'x'){
				Ispress = 1;
				state = 0;
				HAL_UART_Transmit_IT(&huart2,"\n\rMenu\r\nPress 0 To Enter LED Control Mode\r\nPress 1 To Enter Button Status\r\n", 75);
			}
			//speed up
			else if(Ispress == 0 && key[0] == 'a'){
				Ispress = 1;
				LEDspd = (500*LEDspd)/(500+LEDspd);
				ledfreq = ceil(500/LEDspd);
				sprintf(str,"\r\nPress a To Speed Up \r\nPress s To Speed Down \r\nPress d To On/Off\r\nPress x To Back\r\nLED Frequency : %d", ledfreq);
				HAL_UART_Transmit_IT(&huart2, str, 102);

			}
			//speed down
			else if(Ispress == 0 && key[0] == 's'){
				Ispress = 1;
				LEDspd = (500*LEDspd)/(500-LEDspd);
				ledfreq = ceil(500/LEDspd);
				sprintf(str,"\r\nPress a To Speed Up \r\nPress s To Speed Down \r\nPress d To On/Off\r\nPress x To Back\r\nLED Frequency : %d", ledfreq);
				HAL_UART_Transmit_IT(&huart2,str, 102);
			}
			//On/Off
			else if(Ispress == 0 && key[0] == 'd'){
				Ispress = 1;

				if(LEDIson == 1){
					LEDIson = 0;
					HAL_UART_Transmit_IT(&huart2,"\n\rMenu\r\nPress a To Speed Up\r\nPress s To Speed Down\nPress d To Turn On/Off LED\r\nPress x To Back To Menu\r\nLED Turned Off\r", 119);
				}
				else if(LEDIson == 0){
					LEDIson = 1;
					HAL_UART_Transmit_IT(&huart2,"\n\rMenu\r\nPress a To Speed Up\r\nPress s To Speed Down\nPress d To Turn On/Off LED\r\nPress x To Back To Menu\r\nLED Turned On\r", 118);
				}
			}
			else if(Ispress == 0 && key[0] != 'x' && key[0] != 'a' && key[0] != 's' && key[0] != 'd'){
				Ispress = 1;
				HAL_UART_Transmit_IT(&huart2,"Not in menu\r\n", 13);
			}
			break;

	case 2://Show button
			ButtonPress = HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13);
			if(Ispress == 0 && key[0] == 'x'){
				Ispress = 1;
				state = 0;
				HAL_UART_Transmit_IT(&huart2,"\n\rMenu\r\nPress 0 To Enter LED Control Mode\r\nPress 1 To Enter Button Status\r\n", 75 );
			}
			else if(Ispress == 0 && key[0] != 'x'){
				Ispress = 1;
				HAL_UART_Transmit_IT(&huart2,"Not in menu\r\n", 13);
			}
			else if(HAL_GetTick()>=timestamp2){
				timestamp2 = HAL_GetTick()+250;
				if(ButtonPress == 0){
					HAL_UART_Transmit_IT(&huart2,"\n\rButton Pressed\r\n\rMenu\r\nPress x To Back To Menu\r\n\r", 51);
				}
				else if(ButtonPress == 1){
					HAL_UART_Transmit_IT(&huart2,"\n\rButton Not Pressed\r\n\rMenu\r\nPress x To Back To Menu\r\n\r", 55);
				}
			}
			break;



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
