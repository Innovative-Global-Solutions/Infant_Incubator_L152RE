/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "I2C_LCD.h"
#include <stdio.h>
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
char buffer[100];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_I2C1_Init(void);

/* USER CODE BEGIN 0 */

/**
 * Redirect printf → UART2
 */
int __io_putchar(int ch)
{
    HAL_UART_Transmit(&huart2, (uint8_t *)&ch, 1, HAL_MAX_DELAY);
    return ch;
}

/**
 * STS35 CRC calculator (same polynomial as SHT3x)
 */
uint8_t STS35_CalcCRC(uint8_t *data)
{
    uint8_t crc = 0xFF;

    for (int i = 0; i < 2; i++)
    {
        crc ^= data[i];
        for (int j = 0; j < 8; j++)
            crc = (crc & 0x80) ? ((crc << 1) ^ 0x31) : (crc << 1);
    }
    return crc;
}

/**
 * Read temperature from STS35 (°C)
 */
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

/* USER CODE END 0 */

int main(void)
{
    HAL_Init();
    SystemClock_Config();

    MX_GPIO_Init();
    MX_USART2_UART_Init();
    MX_I2C1_Init();

    /* USER CODE BEGIN 2 */

    I2C_LCD_HandleTypeDef lcd1;
    lcd1.hi2c    = &hi2c1;
    lcd1.address = 0x4E;   // PCF8574 at 0x27<<1

    lcd_init(&lcd1);
    lcd_clear(&lcd1);
    lcd_puts(&lcd1, "STS35 Sensor Init");
    HAL_Delay(1000);

    /* USER CODE END 2 */

    while (1)
    {
        float tempC = STS35_ReadTemperature();

        int t = (int)(tempC * 100);

        // Clear ONLY first row
        lcd_gotoxy(&lcd1, 0, 0);
        lcd_puts(&lcd1, "                ");  // 16 spaces

        // Write updated value
        lcd_gotoxy(&lcd1, 0, 0);
        sprintf(buffer, "t=%d.%02d C", t / 100, abs(t % 100));
        lcd_puts(&lcd1, buffer);

        // UART output
        printf("RAW tempC: %d.%02d C\r\n", t / 100, abs(t % 100));

        HAL_Delay(1000);
    }
}
/**
  * @brief System Clock Configuration
  */
void SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    RCC_OscInitStruct.OscillatorType      = RCC_OSCILLATORTYPE_HSI;
    RCC_OscInitStruct.HSIState            = RCC_HSI_ON;
    RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
    RCC_OscInitStruct.PLL.PLLState        = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource       = RCC_PLLSOURCE_HSI;
    RCC_OscInitStruct.PLL.PLLMUL          = RCC_PLL_MUL6;
    RCC_OscInitStruct.PLL.PLLDIV          = RCC_PLL_DIV3;

    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
        Error_Handler();

    RCC_ClkInitStruct.ClockType     = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK |
                                      RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource  = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
        Error_Handler();
}

/**
  * @brief USART2 Initialization Function
  */
static void MX_USART2_UART_Init(void)
{
    huart2.Instance        = USART2;
    huart2.Init.BaudRate   = 115200;
    huart2.Init.WordLength = UART_WORDLENGTH_8B;
    huart2.Init.StopBits   = UART_STOPBITS_1;
    huart2.Init.Parity     = UART_PARITY_NONE;
    huart2.Init.Mode       = UART_MODE_TX_RX;
    huart2.Init.HwFlowCtl  = UART_HWCONTROL_NONE;
    huart2.Init.OverSampling = UART_OVERSAMPLING_16;

    if (HAL_UART_Init(&huart2) != HAL_OK)
        Error_Handler();
}

/**
  * @brief I2C1 Initialization Function (PB6 = SCL, PB7 = SDA)
  */
static void MX_I2C1_Init(void)
{
    hi2c1.Instance             = I2C1;
    hi2c1.Init.ClockSpeed      = 100000;
    hi2c1.Init.DutyCycle       = I2C_DUTYCYCLE_2;
    hi2c1.Init.OwnAddress1     = 0;
    hi2c1.Init.AddressingMode  = I2C_ADDRESSINGMODE_7BIT;
    hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
    hi2c1.Init.OwnAddress2     = 0;
    hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
    hi2c1.Init.NoStretchMode   = I2C_NOSTRETCH_DISABLE;

    if (HAL_I2C_Init(&hi2c1) != HAL_OK)
        Error_Handler();
}

/**
  * @brief GPIO Initialization Function
  */
static void MX_GPIO_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOH_CLK_ENABLE();

    /* I2C: PB6=SCL, PB7=SDA */
    GPIO_InitStruct.Pin       = GPIO_PIN_6 | GPIO_PIN_7;
    GPIO_InitStruct.Mode      = GPIO_MODE_AF_OD;
    GPIO_InitStruct.Pull      = GPIO_NOPULL;
    GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF4_I2C1;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /* LD2 + PA7 output */
    HAL_GPIO_WritePin(GPIOA, LD2_Pin | GPIO_PIN_7, GPIO_PIN_RESET);

    GPIO_InitStruct.Pin   = LD2_Pin | GPIO_PIN_7;
    GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull  = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* PA1 input */
    GPIO_InitStruct.Pin  = GPIO_PIN_1;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* PA8, PA9, PA10 input buttons */
    GPIO_InitStruct.Pin  = GPIO_PIN_8 | GPIO_PIN_9 | GPIO_PIN_10;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* User button (B1) */
    GPIO_InitStruct.Pin  = B1_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);
}

/**
  * @brief Generic Error Handler
  */
void Error_Handler(void)
{
    __disable_irq();
    while (1) {}
}
