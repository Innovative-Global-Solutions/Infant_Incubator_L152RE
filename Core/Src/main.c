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
#include <stdbool.h>
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

typedef struct {
    int *min;
    int *max;
    int absMin;
    int absMax;
} paramBounds;
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

TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
const uint32_t SCREEN_REFRESH_MS = 1500;   // update every 500 ms
char buffer[256] = {0};
uint32_t lastUpdateTick = 0;
uint32_t now = 0;
I2C_LCD_HandleTypeDef lcd1;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_I2C1_Init(void);
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

void drawParamScreen(I2C_LCD_HandleTypeDef *lcd, const char *title, float value, int minVal, int maxVal, uint8_t editMax, char *buffer)
{
    // Display the name of the current screen
    lcd_gotoxy(lcd, 0, 0);
    lcd_puts(lcd, title);

    // Display the live sensor value on the right side
    lcd_gotoxy(lcd, 15, 0);
    sprintf(buffer, "%4.1f", value);
    lcd_puts(lcd, buffer);

    // Show cursor if editing MIN (editMax == 0)
    lcd_gotoxy(lcd, 0, 1);
    lcd_puts(lcd, (!editMax) ? ">" : " ");

    // Label for minimum bound
    lcd_gotoxy(lcd, 1, 1);
    lcd_puts(lcd, "Min. Bound:");

    // Display minimum bound value
    lcd_gotoxy(lcd, 15, 1);
    sprintf(buffer, "%d", minVal);
    lcd_puts(lcd, buffer);

    // Show cursor if editing MAX (editMax == 1)
    lcd_gotoxy(lcd, 0, 2);
    lcd_puts(lcd, (editMax) ? ">" : " ");

    // Label for maximum bound
    lcd_gotoxy(lcd, 1, 2);
    lcd_puts(lcd, "Max. Bound:");

    // Display maximum bound value
    lcd_gotoxy(lcd, 15, 2);
    sprintf(buffer, "%d", maxVal);
    lcd_puts(lcd, buffer);
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
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */
 HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);
 /* USER CODE BEGIN 2 */
 lcd1.hi2c1 = &hi2c1;   // use global hi2c1 (initialized by MX_I2C1_Init)
 lcd1.address = (0x27 << 1);
 lcd_init(&lcd1);
 // Pulled-down input so default low.
 uint8_t bpmLastState = 0;
 uint8_t humidityLastState = 0;
 uint8_t incubatorTempLastState = 0;
 uint8_t infantTempLastState = 0;
 uint8_t homeScreenLastState = 0;
 MenuState currentScreen = HOME_SCREEN;
 MenuState lastScreen = INF_TEMP_SCREEN;
 bool editMax = false;   // false = edit min, true = edit max
 int16_t lastEnc = (int16_t)__HAL_TIM_GET_COUNTER(&htim3);
 // Random testing variables
 float infTemp = 0;
 float humidity = 0;
 float incTemp = 0;
 float bpm = 100;
 int alarm = 0;
 int maxIncTemp = 50;
 int minIncTemp = 10;
 int minBpm = 80;
 int maxBpm = 150;
 int minInfTemp = 20;
 int maxInfTemp = 30;
 int maxHumidity = 98;
 int minHumidity = 2;

 paramBounds params[] = {
     [BPM_SCREEN]       = {&minBpm, &maxBpm, 0, 300},
     [HUMIDITY_SCREEN]  = {&minHumidity, &maxHumidity, 0, 100},
     [INC_TEMP_SCREEN]  = {&minIncTemp, &maxIncTemp, -100, 100},
     [INF_TEMP_SCREEN]  = {&minInfTemp, &maxInfTemp, -100, 100}
 };
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
 while (1)
 {
	  // Read encoder value
	  int16_t encNow = (int16_t)__HAL_TIM_GET_COUNTER(&htim3);
	  int16_t encDelta = encNow - lastEnc;

	  // Encoder input handling.
	  // When the encoder is rotated enough to register, the system adjusts parameters for selected screen.
	  float step = 0;
	  if (encDelta >= 2)
	  {
	      step = 1;
	      lastEnc = encNow;
	  }
	  else if (encDelta <= -2)
	  {
	      step = -1;
	      lastEnc = encNow;
	  }
	  // update time stamp
     now = HAL_GetTick();
     // Reading the button states
     uint8_t bpmState = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_10);
     uint8_t humidityState = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_9);
     uint8_t incubatorTempState = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_5);
     uint8_t infantTempState = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_4);
     uint8_t homeScreenState = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_10);
     // Button press handling for screen navigation and parameter editing.

     // Button 1 : BPM screen
          if (bpmState == GPIO_PIN_RESET && bpmLastState == GPIO_PIN_SET)
          {
              if (currentScreen == BPM_SCREEN)
                  editMax = !editMax;
              else
              {
                  currentScreen = BPM_SCREEN;
                  editMax = false;
              }
          }
          // Button 2 : Humidity screen
          else if (humidityState == GPIO_PIN_RESET && humidityLastState == GPIO_PIN_SET)
          {
              if (currentScreen == HUMIDITY_SCREEN)
                  editMax = !editMax;
              else
              {
                  currentScreen = HUMIDITY_SCREEN;
                  editMax = false;
              }
          }
          // Button 3 : Incubator temperature screen
          else if (incubatorTempState == GPIO_PIN_RESET && incubatorTempLastState == GPIO_PIN_SET)
          {
              if (currentScreen == INC_TEMP_SCREEN)
                  editMax = !editMax;
              else
              {
                  currentScreen = INC_TEMP_SCREEN;
                  editMax = false;
              }
          }
          // Button 4 : Infant temperature screen
          else if (infantTempState == GPIO_PIN_RESET && infantTempLastState == GPIO_PIN_SET)
          {
              if (currentScreen == INF_TEMP_SCREEN)
                  editMax = !editMax;
              else
              {
                  currentScreen = INF_TEMP_SCREEN;
                  editMax = false;
              }
          }
          // Button 5 : Home screen
          else if (homeScreenState == GPIO_PIN_RESET && homeScreenLastState == GPIO_PIN_SET)
          {
              currentScreen = HOME_SCREEN;
          }

          // Saving the button state
          bpmLastState = bpmState;
          humidityLastState = humidityState;
          incubatorTempLastState = incubatorTempState;
          infantTempLastState = infantTempState;
          homeScreenLastState = homeScreenState;

          // get sensor readings
          infTemp = STS35_ReadTemperature();
          SHT31_ReadTempHumidity(&incTemp, &humidity);

          // Encoder step changes bounds on selected screen
          if (step != 0 && currentScreen != HOME_SCREEN)
          {
              paramBounds *p = &params[currentScreen];

              if (!editMax)
              {
                  *p->min += step;

                  if (*p->min < p->absMin) *p->min = p->absMin;
                  if (*p->min >= *p->max) *p->min = *p->max - 1;
              }
              else
              {
                  *p->max += step;

                  if (*p->max <= *p->min) *p->max = *p->min + 1;
                  if (*p->max > p->absMax) *p->max = p->absMax;
              }
          }

     //Alarms
     //Infant temperature alarm
     if (infTemp > maxInfTemp || infTemp < minInfTemp) alarm = 1;
     //Incubator temperature alarm
     else if (incTemp > maxIncTemp || incTemp < minIncTemp) alarm = 1;
     //Incubator humidity alarm
     else if (humidity > maxHumidity || humidity < minHumidity) alarm = 1;
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
         switch(currentScreen)
         {
             case HOME_SCREEN:
                 // Displays all current sensor values

                 // Print screen title
                 lcd_gotoxy(&lcd1, 0, 0);
                 lcd_puts(&lcd1, "Home:");

                 // Print BPM value
                 lcd_gotoxy(&lcd1, 0, 1);
                 sprintf(buffer, "BPM: %4.1f", bpm);
                 lcd_puts(&lcd1, buffer);

                 // Print humidity value
                 lcd_gotoxy(&lcd1, 11, 1);
                 sprintf(buffer, "HUM: %4.1f", humidity);
                 lcd_puts(&lcd1, buffer);

                 // Print incubator temperature
                 lcd_gotoxy(&lcd1, 0, 2);
                 sprintf(buffer, "INC: %4.1f", incTemp);
                 lcd_puts(&lcd1, buffer);

                 // Print infant temperature
                 lcd_gotoxy(&lcd1, 11, 2);
                 sprintf(buffer, "INF: %4.1f", infTemp);
                 lcd_puts(&lcd1, buffer);

                 break;

             case BPM_SCREEN:
                 // Allows editing of BPM min/max bounds using encoder

                 drawParamScreen(&lcd1, "BPM:",
                                 bpm, minBpm, maxBpm,
                                 editMax, buffer);
                 break;

             case HUMIDITY_SCREEN:
                 // Allows editing of humidity min/max bounds

                 drawParamScreen(&lcd1, "Humidity",
                                 humidity, minHumidity, maxHumidity,
                                 editMax, buffer);
                 break;

             case INC_TEMP_SCREEN:
                 // Allows editing of incubator temperature limits

                 drawParamScreen(&lcd1, "Incubator Temp:",
                                 incTemp, minIncTemp, maxIncTemp,
                                 editMax, buffer);
                 break;

             case INF_TEMP_SCREEN:
                 // Allows editing of infant temperature limits

                 drawParamScreen(&lcd1, "Infant Temp:",
                                 infTemp, minInfTemp, maxInfTemp,
                                 editMax, buffer);
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

  /*Configure GPIO pin : PA8 */
  GPIO_InitStruct.Pin = GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PA9 PA10 */
  GPIO_InitStruct.Pin = GPIO_PIN_9|GPIO_PIN_10;
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
