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
ADC_HandleTypeDef hadc;
TIM_HandleTypeDef htim3;
I2C_HandleTypeDef hi2c1;
I2C_LCD_HandleTypeDef lcd1;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
const uint32_t SCREEN_REFRESH_MS = 1500;   // update every 500 ms
char buffer[256] = {0};
uint32_t lastUpdateTick = 0;
uint32_t now = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_I2C1_Init(void);
static void MX_ADC_Init(void);
static void MX_TIM3_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

// calculates CRC value for STS-35 Sensor
uint8_t STS35_CalcCRC(uint8_t *data)
{
    uint8_t crc = 0xFF;   // Start with initial CRC value

    // Process both temperature bytes
    for (int i = 0; i < 2; i++)
    {
        crc ^= data[i];   // XOR incoming byte with current CRC

        // Process each of the 8 bits in this byte
        for (int j = 0; j < 8; j++)
        {
            // If MSB is 1, shift and XOR with polynomial 0x31
            if (crc & 0x80)
                crc = (crc << 1) ^ 0x31;
            else
                crc = (crc << 1);   // Otherwise just shift left
        }
    }

    return crc;   // Final CRC result
}
//takes reading from STS35 sensor
float STS35_ReadTemperature(void)
{
   uint8_t cmd[2] = {0x24, 0x00};     // High repeatability, no clock stretch
   uint8_t rx[3];
   uint16_t raw;
   // Send command
   if (HAL_I2C_Master_Transmit(&hi2c1, (0x4B << 1), cmd, 2, HAL_MAX_DELAY) != HAL_OK)
       return -300.0f;
   HAL_Delay(20);
   // Read response: 2 bytes + CRC
   if (HAL_I2C_Master_Receive(&hi2c1, (0x4B << 1), rx, 3, HAL_MAX_DELAY) != HAL_OK)
       return -300.0f;
   // CRC check
   if (STS35_CalcCRC(rx) != rx[2])
       return -301.0f;
   // Convert raw temperature reading
   raw = (rx[0] << 8) | rx[1];
   return -45.0f + (175.0f * (float)raw / 65535.0f);
}

//Takes reading from SHT31 Sensor
void SHT31_ReadTempHumidity(float* temp, float* humidity)
{
    uint8_t cmd[2] = {0x2C, 0x06};
    uint8_t data[6];
    uint16_t temp_raw, humidity_raw;

    // Send measurement command
    HAL_I2C_Master_Transmit(&hi2c1, SHT31_ADDR, cmd, 2, HAL_MAX_DELAY);
    HAL_Delay(20);

    // Read 6 bytes (temp + humidity)
    HAL_I2C_Master_Receive(&hi2c1, SHT31_ADDR, data, 6, HAL_MAX_DELAY);

    // Parse raw values
    temp_raw = (data[0] << 8) | data[1];
    humidity_raw = (data[3] << 8) | data[4];

    // Convert to human-readable units
    *temp = -45.0f + 175.0f * ((float)temp_raw / 65535.0f);
    *humidity = 100.0f * ((float)humidity_raw / 65535.0f);
}

uint32_t Read_ADC(void)
{
    uint32_t adc_val;

    HAL_ADC_Start(&hadc);                               // Start ADC
    HAL_ADC_PollForConversion(&hadc, HAL_MAX_DELAY);    // Wait for conversion
    adc_val = HAL_ADC_GetValue(&hadc);                  // Read value
    HAL_ADC_Stop(&hadc);                                // Stop ADC

    return adc_val;
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
  MX_USART2_UART_Init();
  MX_I2C1_Init();
  MX_ADC_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);

  /* USER CODE BEGIN 2 */
  lcd1.hi2c1 = &hi2c1;   // use global hi2c1 (initialized by MX_I2C1_Init)
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
  float infTemp = 0;
  float humidity = 0;
  float incTemp = 0;
  int alarm = 0;
  int maxIncTemp = 50;
  int minIncTemp = 10;
  int bpm = 100;
  int minBpm = 80;
  int maxBpm = 150;
  int minInfTemp = 20;
  int maxInfTemp = 30;
  int maxHumidity = 98;
  int minHumidity = 2;



  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {


	  int16_t pos = (int16_t)__HAL_TIM_GET_COUNTER(&htim3);  // signed makes wrap-around easier to think about

	  sprintf(buffer, "ENC = %d\r\n", pos);
	  HAL_UART_Transmit(&huart2, (uint8_t*)buffer, strlen(buffer), HAL_MAX_DELAY);
	  HAL_Delay(50);


	  HAL_ADC_Start(&hadc);
	  HAL_ADC_PollForConversion(&hadc, HAL_MAX_DELAY);
	  uint32_t adc = HAL_ADC_GetValue(&hadc);
	  HAL_ADC_Stop(&hadc);


	  HAL_UART_Transmit(&huart2, (uint8_t*)buffer, strlen(buffer), HAL_MAX_DELAY);

	  HAL_Delay(500);


	  // update time stamp
      now = HAL_GetTick();

      // Reading the button states
      uint8_t currentState1 = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_10);
      uint8_t currentState2 = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_9);
      uint8_t currentState3 = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_5);
      uint8_t currentState4 = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_4);
      uint8_t currentState5 = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_10);

      // Button presses
      if (currentState1 == GPIO_PIN_RESET && lastButtonState1 == GPIO_PIN_SET)
          currentScreen = BPM_SCREEN;
      else if (currentState2 == GPIO_PIN_RESET && lastButtonState2 == GPIO_PIN_SET)
    	  currentScreen = HUMIDITY_SCREEN;
      else if (currentState3 == GPIO_PIN_RESET && lastButtonState3 == GPIO_PIN_SET)
          currentScreen = INC_TEMP_SCREEN;
      else if (currentState4 == GPIO_PIN_RESET && lastButtonState4 == GPIO_PIN_SET)
          currentScreen = INF_TEMP_SCREEN;
      else if (currentState5 == GPIO_PIN_RESET && lastButtonState5 == GPIO_PIN_SET)
          currentScreen = HOME_SCREEN;

      // Saving the button state
      lastButtonState1 = currentState1;
      lastButtonState2 = currentState2;
      lastButtonState3 = currentState3;
      lastButtonState4 = currentState4;
      lastButtonState5 = currentState5;

      // get sensor readings
      infTemp = STS35_ReadTemperature();
      SHT31_ReadTempHumidity(&incTemp, &humidity);


      //Alarms

      //Infant temperature alarm
      if (infTemp > maxInfTemp || infTemp < minInfTemp) alarm = 1;

      //Incubator temperature alarm
      else if (incTemp > maxIncTemp || incTemp < minIncTemp) alarm = 1;

      //Incubator humidity alarm
      else if (humidity > maxHumidity || infTemp < minHumidity) alarm = 1;

      //Incubator humidity alarm
      else if (bpm > maxBpm || bpm < minBpm) alarm = 1;
      else alarm = 0;

      //Write to alarm pin
      HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, alarm ? GPIO_PIN_SET : GPIO_PIN_RESET);

      // Only update screen on changes

      if (currentScreen != lastScreen || (now - lastUpdateTick) >= SCREEN_REFRESH_MS) {
          lastUpdateTick = now;   // record time of this update

          lcd_clear(&lcd1);

		  // Home screen as default
		  switch(currentScreen){
		  case HOME_SCREEN:
			  lcd_gotoxy(&lcd1, 0, 0);
			  lcd_puts(&lcd1, "Home:");

//			   BPM line
			  lcd_gotoxy(&lcd1, 0, 1);
			  sprintf(buffer, "BPM: %d", bpm);
			  lcd_puts(&lcd1, buffer);

			  // Humidity Line
			  lcd_gotoxy(&lcd1, 11, 1);
			  sprintf(buffer, "HUM: %4.1f", humidity);
			  lcd_puts(&lcd1, buffer);

			  // Incubator Temp Line
			  lcd_gotoxy(&lcd1, 0, 2);
			  sprintf(buffer, "INC: %4.1f", incTemp);
			  lcd_puts(&lcd1, buffer);

			  // Infant Temp Line
			  lcd_gotoxy(&lcd1, 11, 2);
			  sprintf(buffer, "INF: %4.1f", infTemp);
			  lcd_puts(&lcd1, buffer);
			  break;

		  case BPM_SCREEN:
			  lcd_gotoxy(&lcd1, 0, 0);
			  lcd_puts(&lcd1, "BPM:");

			  // Print average BPM here.
			  lcd_gotoxy(&lcd1, 17, 0);
			  sprintf(buffer, "%d", bpm);
			  lcd_puts(&lcd1, buffer);

			  // Printing the minimum BPM bound.
			  lcd_gotoxy(&lcd1, 0, 1);
			  lcd_puts(&lcd1, "Min. Bound:");
			  lcd_gotoxy(&lcd1, 18, 1);
			  sprintf(buffer, "%d", minBpm);
			  lcd_puts(&lcd1, buffer);

			  // Printing the maximum BPM bound.
			  lcd_gotoxy(&lcd1, 0, 2);
			  lcd_puts(&lcd1, "Max. Bound:");
			  lcd_gotoxy(&lcd1, 17, 2);
			  sprintf(buffer, "%d", maxBpm);
			  lcd_puts(&lcd1, buffer);
			  break;

		  case HUMIDITY_SCREEN:
			  lcd_gotoxy(&lcd1, 0, 0);
			  lcd_puts(&lcd1, "Humidity Screen");

			  // Print average humidity here.
			  lcd_gotoxy(&lcd1, 16, 0);
			  sprintf(buffer, "%4.1f", humidity);
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

			  // Print average infant temperature here.
			  lcd_gotoxy(&lcd1, 16, 0);
			  sprintf(buffer, "%4.1f", incTemp);
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

			  // Print average infant temperature here.
			  lcd_gotoxy(&lcd1, 16, 0);
			  sprintf(buffer, "%4.1f", infTemp);
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
  hadc.Instance = ADC1;
  hadc.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc.Init.Resolution = ADC_RESOLUTION_12B;
  hadc.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc.Init.EOCSelection = ADC_EOC_SEQ_CONV;
  hadc.Init.LowPowerAutoWait = ADC_AUTOWAIT_DISABLE;
  hadc.Init.LowPowerAutoPowerOff = ADC_AUTOPOWEROFF_DISABLE;
  hadc.Init.ChannelsBank = ADC_CHANNELS_BANK_A;
  hadc.Init.ContinuousConvMode = DISABLE;
  hadc.Init.NbrOfConversion = 1;
  hadc.Init.DiscontinuousConvMode = DISABLE;
  hadc.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc.Init.DMAContinuousRequests = DISABLE;
  if (HAL_ADC_Init(&hadc) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_4CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC_Init 2 */

  /* USER CODE END ADC_Init 2 */

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
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim3, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

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
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, LD2_Pin|GPIO_PIN_7, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_RESET);

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
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PA10 */
  GPIO_InitStruct.Pin = GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PB9 */
  GPIO_InitStruct.Pin = GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */
  // --- Rotary encoder pins: PA6 = TIM3_CH1, PC7 = TIM3_CH2 ---
  // IMPORTANT: Set GPIO_AFx_TIM3 to the correct AF number for your MCU.
  // CubeMX normally shows it; otherwise check the datasheet "Alternate function mapping".


  // Enable GPIOC clock already enabled above; GPIOA too (you already do this)

  // PA6 -> TIM3_CH1
  GPIO_InitStruct.Pin = GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;                 // because encoder common is GND
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF2_TIM3;          // <--- THIS AF NUMBER MAY DIFFER
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  // PC7 -> TIM3_CH2
  GPIO_InitStruct.Pin = GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;                 // pull-up for switch-to-ground encoder
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF2_TIM3;          // <--- THIS AF NUMBER MAY DIFFER
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

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
