/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include "I2C_LCD.h"
#include <string.h>
#include <stdio.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef enum{
   HOME_SCREEN,
   BPM_SCREEN,
   HUMIDITY_SCREEN,
   INC_TEMP_SCREEN,
   INF_TEMP_SCREEN,
} MenuState;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
	#define SHT31_ADDR 0x44 << 1 // SHT31 I2C address shifted left by 1 bit
	#define CMD_MEASURE_TEMP 0x2C06 // Command to measure temperature
	#define CMD_MEASURE_HUMIDITY 0x2C10 // Command to measure humidity

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;
I2C_LCD_HandleTypeDef lcd1;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_I2C1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

char buffer[100] = {0};
float temp = 0;
float humid = 0;



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
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE BEGIN 2 */
  lcd1.hi2c = &hi2c1;   // use global hi2c1 (initialized by MX_I2C1_Init)
  lcd1.address = (0x27 << 1);
  lcd_init(&lcd1);

  // Pulled-down input so default low.
  uint8_t lastButtonState1 = 0;
  uint8_t lastButtonState2 = 0;
  uint8_t lastButtonState3 = 0;
  uint8_t lastButtonState4 = 0;
  uint8_t lastButtonState5 = 0;

  MenuState currentScreen = HOME_SCREEN;
  MenuState lastScreen = INF_TEMP_SCREEN;

  // Random testing variables
  char buffer[24] = {0};
  int averageBPM = 120;
  int minBPM = 80;
  int maxBPM = 150;
  int averageInfTemp = 96;
  int minInfTemp = 93;
  int maxInfTemp = 98;
  int averageHumidity = 97;
  int averageIncTemp = 70;
  int maxHumidity = 98;
  int minHumidity = 54;

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
      // Reading the button states
      uint8_t currentState1 = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_10);
      uint8_t currentState2 = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_9);
      uint8_t currentState3 = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_5);
      uint8_t currentState4 = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_4);
      uint8_t currentState5 = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_10);

      // Button presses
      if (currentState1 == GPIO_PIN_RESET && lastButtonState1 == GPIO_PIN_SET)
          currentScreen = BPM_SCREEN;
      if (currentState2 == GPIO_PIN_RESET && lastButtonState2 == GPIO_PIN_SET)
          currentScreen = HUMIDITY_SCREEN;
      if (currentState3 == GPIO_PIN_RESET && lastButtonState3 == GPIO_PIN_SET)
          currentScreen = INC_TEMP_SCREEN;
      if (currentState4 == GPIO_PIN_RESET && lastButtonState4 == GPIO_PIN_SET)
          currentScreen = INF_TEMP_SCREEN;
      if (currentState5 == GPIO_PIN_RESET && lastButtonState5 == GPIO_PIN_SET)
          currentScreen = HOME_SCREEN;

      // Saving the button state
      lastButtonState1 = currentState1;
      lastButtonState2 = currentState2;
      lastButtonState3 = currentState3;
      lastButtonState4 = currentState4;
      lastButtonState5 = currentState5;

      // Only update screen on changes
	  if (currentScreen != lastScreen){
		  lcd_clear(&lcd1);

      // Home screen as default
      switch(currentScreen){
      case HOME_SCREEN:
          lcd_gotoxy(&lcd1, 0, 0);
          lcd_puts(&lcd1, "Home:");
          // BPM line
          lcd_gotoxy(&lcd1, 0, 1);
          sprintf(buffer, "BPM: %d", averageBPM);
          lcd_puts(&lcd1, buffer);
          // Humidity Line
          lcd_gotoxy(&lcd1, 11, 1);
          sprintf(buffer, "HUM: %d.4", averageHumidity);
          lcd_puts(&lcd1, buffer);
          // Infant Temp Line
          lcd_gotoxy(&lcd1, 0, 2);
          sprintf(buffer, "INC: %d.4", averageIncTemp);
          lcd_puts(&lcd1, buffer);
          // Incubator Temp Line
          lcd_gotoxy(&lcd1, 11, 2);
          sprintf(buffer, "INF: %d.4", averageInfTemp);
          lcd_puts(&lcd1, buffer);
          break;

      case BPM_SCREEN:
    	  lcd_gotoxy(&lcd1, 0, 0);
    	  lcd_puts(&lcd1, "BPM:");
    	  lcd_gotoxy(&lcd1, 17, 0);
    	  sprintf(buffer, "%d", averageBPM);
    	  // Print average BPM here.
    	  lcd_puts(&lcd1, buffer);
    	  // Printing the minimum BPM bound.
    	  lcd_gotoxy(&lcd1, 0, 1);
    	  lcd_puts(&lcd1, "Min. Bound:");
    	  lcd_gotoxy(&lcd1, 18, 1);
    	  sprintf(buffer, "%d", minBPM);
    	  lcd_puts(&lcd1, buffer);
    	  // Printing the maximum BPM bound.
    	  lcd_gotoxy(&lcd1, 0, 2);
    	  lcd_puts(&lcd1, "Max. Bound:");
    	  lcd_gotoxy(&lcd1, 17, 2);
    	  sprintf(buffer, "%d", maxBPM);
    	  lcd_puts(&lcd1, buffer);
    	  break;

      case HUMIDITY_SCREEN:
    	  lcd_gotoxy(&lcd1, 0, 0);
   	      lcd_puts(&lcd1, "Humidity Screen");
    	  lcd_gotoxy(&lcd1, 16, 0);
    	  sprintf(buffer, "%d.4", averageHumidity);
    	  // Print average humidity here.
    	  lcd_puts(&lcd1, buffer);
    	  // Printing the minimum humidity bound.
    	  lcd_gotoxy(&lcd1, 0, 1);
    	  lcd_puts(&lcd1, "Min. Bound:");
    	  lcd_gotoxy(&lcd1, 16, 1);
    	  sprintf(buffer, "%d.4", minHumidity);
    	  lcd_puts(&lcd1, buffer);
    	  // Printing the maximum humidity bound.
    	  lcd_gotoxy(&lcd1, 0, 2);
    	  lcd_puts(&lcd1, "Max. Bound:");
    	  lcd_gotoxy(&lcd1, 16, 2);
    	  sprintf(buffer, "%d.4", maxHumidity);
    	  lcd_puts(&lcd1, buffer);
    	  break;

      case INC_TEMP_SCREEN:
    	  lcd_gotoxy(&lcd1, 0, 0);
    	  lcd_puts(&lcd1, "Incubator Temp:");
    	  lcd_gotoxy(&lcd1, 16, 0);
    	  sprintf(buffer, "%d.4", averageInfTemp);
    	  // Print average infant temperature here.
    	  lcd_puts(&lcd1, buffer);
    	  // Printing the minimum temperature bound.
    	  lcd_gotoxy(&lcd1, 0, 1);
    	  lcd_puts(&lcd1, "Min. Bound:");
    	  lcd_gotoxy(&lcd1, 16, 1);
    	  sprintf(buffer, "%d.4", minInfTemp);
    	  lcd_puts(&lcd1, buffer);
    	  // Printing the maximum temperature bound.
    	  lcd_gotoxy(&lcd1, 0, 2);
    	  lcd_puts(&lcd1, "Max. Bound:");
    	  lcd_gotoxy(&lcd1, 16, 2);
    	  sprintf(buffer, "%d.4", maxInfTemp);
    	  lcd_puts(&lcd1, buffer);
    	  break;

      case INF_TEMP_SCREEN:
    	  lcd_gotoxy(&lcd1, 0, 0);
    	  lcd_puts(&lcd1, "Infant Temp:");
    	  lcd_gotoxy(&lcd1, 16, 0);
    	  sprintf(buffer, "%d.4", averageInfTemp);
    	  // Print average infant temperature here.
    	  lcd_puts(&lcd1, buffer);
    	  // Printing the minimum temperature bound.
    	  lcd_gotoxy(&lcd1, 0, 1);
    	  lcd_puts(&lcd1, "Min. Bound:");
    	  lcd_gotoxy(&lcd1, 16, 1);
    	  sprintf(buffer, "%d.4", minInfTemp);
    	  lcd_puts(&lcd1, buffer);
    	  // Printing the maximum temperature bound.
    	  lcd_gotoxy(&lcd1, 0, 2);
    	  lcd_puts(&lcd1, "Max. Bound:");
    	  lcd_gotoxy(&lcd1, 16, 2);
    	  sprintf(buffer, "%d.4", maxInfTemp);
    	  lcd_puts(&lcd1, buffer);
    	  break;
    	  }

      lastScreen = currentScreen;
	  }

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
      HAL_Delay(50);
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void){
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL6;
  RCC_OscInitStruct.PLL.PLLDIV = RCC_PLL_DIV3;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void){

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
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
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void){

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
static void MX_GPIO_Init(void){
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, LD2_Pin|GPIO_PIN_7, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PA1 */
  GPIO_InitStruct.Pin = GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : LD2_Pin PA7 */
  GPIO_InitStruct.Pin = LD2_Pin|GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB10 PB3 PB4 PB5 */
  GPIO_InitStruct.Pin = GPIO_PIN_10|GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PA8 PA9 */
  GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PA10 */
  GPIO_InitStruct.Pin = GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
}

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void){
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1);
  {
  };


  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
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
