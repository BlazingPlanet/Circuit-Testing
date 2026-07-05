/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
#include <stdio.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct {
    uint16_t dig_T1;
    int16_t dig_T2;
    int16_t dig_T3;
    uint16_t dig_P1;
    int16_t dig_P2;
    int16_t dig_P3;
    int16_t dig_P4;
    int16_t dig_P5;
    int16_t dig_P6;
    int16_t dig_P7;
    int16_t dig_P8;
    int16_t dig_P9;
} BMP280_Calib;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef hspi1;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
BMP280_Calib calib; // Structure to hold calibration data
int32_t t_fine; // Variable to hold the fine temperature value for compensation
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_SPI1_Init(void);
/* USER CODE BEGIN PFP */
uint8_t BMP280_ReadRegister(uint8_t reg); // Function to read a register from the BMP280 sensor

void BMP280ReadRegisters(uint8_t reg, uint8_t *buffer, uint16_t len); // Function to read multiple registers from the BMP280 sensor
void BMP280_ReadCalibration(void); // Function to read calibration data from the BMP280 sensor

void BMP280ReadRaw(int32_t *raw_temp, int32_t *raw_press); // Function to read raw temperature and pressure data from the BMP280 sensor
double BMP280CompensateTemp(int32_t raw_temp); // Function to compensate the raw temperature data using calibration data
double BMP280CompensatePress(int32_t raw_press); // Function to compensate the raw pressure data using calibration data

void BMP280_WriteRegister(uint8_t reg, uint8_t value);

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
  MX_SPI1_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  uint8_t chip_id = BMP280_ReadRegister(0xD0); // Read the chip ID register
  printf("BMP280 Chip ID: 0x%02X\r\n", chip_id); // Print the chip ID to UART

  BMP280_ReadCalibration(); // Read calibration data from the sensor
  printf("dig_T1 = %u\r\n", calib.dig_T1);
  printf("dig_T2 = %d\r\n", calib.dig_T2);
  printf("dig_T3 = %d\r\n", calib.dig_T3);
  printf("dig_P1 = %u\r\n", calib.dig_P1);
  printf("dig_P9 = %d\r\n", calib.dig_P9);

  BMP280_WriteRegister(0xF4, 0x57); // ← ADD: wake sensor, temp ×2, press ×16, normal mode

  int32_t raw_temp, raw_press;

  while (1)
  {
    BMP280ReadRaw(&raw_temp, &raw_press); // Read raw temperature and pressure data
    double tempC = BMP280CompensateTemp(raw_temp); // Compensate the raw temperature data
    double pressPa = BMP280CompensatePress(raw_press); // Compensate the raw pressure data
    printf("Temperature: %.2f C, Pressure: %.2f Pa\r\n", tempC, pressPa); // Print the compensated temperature and pressure
    HAL_Delay(1000); // Delay for 1 second
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
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

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
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
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
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_HIGH;
  hspi1.Init.CLKPhase = SPI_PHASE_2EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_64;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

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
  HAL_GPIO_WritePin(IMU_CS_GPIO_Port, IMU_CS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(BMP_CS_GPIO_Port, BMP_CS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : IMU_CS_Pin */
  GPIO_InitStruct.Pin = IMU_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(IMU_CS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : BMP_CS_Pin */
  GPIO_InitStruct.Pin = BMP_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(BMP_CS_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
int _write(int file, char *ptr, int len)
{
  HAL_UART_Transmit(&huart2, (uint8_t *)ptr, len, HAL_MAX_DELAY);
  return len;
}

uint8_t BMP280_ReadRegister(uint8_t reg)
{
  uint8_t tx[2];
  uint8_t rx[2];

  tx[0] = reg | 0x80; // Address byte, bit 7 set = READ
  tx[1] = 0x00; // Dummy byte to clock the answer back

  HAL_GPIO_WritePin(BMP_CS_GPIO_Port, BMP_CS_Pin,  GPIO_PIN_RESET); // CS Low: Start communication
  HAL_SPI_TransmitReceive(&hspi1, tx, rx, 2, HAL_MAX_DELAY); // exchange 2 bytes
  HAL_GPIO_WritePin(BMP_CS_GPIO_Port, BMP_CS_Pin, GPIO_PIN_SET); // CS High: End communication

  return rx[1]; // The sensors answers landed in the second byte   
}

void BMP280_WriteRegister(uint8_t reg, uint8_t value)
{
    uint8_t tx[2];
    tx[0] = reg & 0x7F;   // address byte, bit 7 cleared = WRITE
    tx[1] = value;        // the byte to write into that register

    HAL_GPIO_WritePin(BMP_CS_GPIO_Port, BMP_CS_Pin, GPIO_PIN_RESET);  // CS low
    HAL_SPI_Transmit(&hspi1, tx, 2, HAL_MAX_DELAY);                   // send both bytes
    HAL_GPIO_WritePin(BMP_CS_GPIO_Port, BMP_CS_Pin, GPIO_PIN_SET);    // CS high
}

void BMP280ReadRegisters(uint8_t reg, uint8_t *buffer, uint16_t len)
{
  uint8_t addr = reg | 0x80; // Address byte, bit 7 set = READ

  HAL_GPIO_WritePin(BMP_CS_GPIO_Port, BMP_CS_Pin, GPIO_PIN_RESET); // CS Low: Start communication
  HAL_SPI_Transmit(&hspi1, &addr, 1, HAL_MAX_DELAY); // exchange registers
  HAL_SPI_Receive(&hspi1, buffer, len, HAL_MAX_DELAY); // receive data
  HAL_GPIO_WritePin(BMP_CS_GPIO_Port, BMP_CS_Pin, GPIO_PIN_SET); // CS High: End communication
}

void BMP280_ReadCalibration(void)
{
  uint8_t buf[24]; // Buffer to hold calibration data
  BMP280ReadRegisters(0x88, buf, 24); // Read 24 bytes of calibration data starting from register 0x88

  calib.dig_T1 = (uint16_t)((buf[1] << 8) | buf[0]);
  calib.dig_T2 = (int16_t)((buf[3] << 8) | buf[2]);
  calib.dig_T3 = (int16_t)((buf[5] << 8) | buf[4]);
  calib.dig_P1 = (uint16_t)((buf[7] << 8) | buf[6]);
  calib.dig_P2 = (int16_t)((buf[9] << 8) | buf[8]);
  calib.dig_P3 = (int16_t)((buf[11] << 8) | buf[10]);
  calib.dig_P4 = (int16_t)((buf[13] << 8) | buf[12]);
  calib.dig_P5 = (int16_t)((buf[15] << 8) | buf[14]);
  calib.dig_P6 = (int16_t)((buf[17] << 8) | buf[16]);
  calib.dig_P7 = (int16_t)((buf[19] << 8) | buf[18]);
  calib.dig_P8 = (int16_t)((buf[21] << 8) | buf[20]);
  calib.dig_P9 = (int16_t)((buf[23] << 8) | buf[22]);
}

void BMP280ReadRaw(int32_t *raw_temp, int32_t *raw_press)
{
  uint8_t buf[6]; // Buffer to hold raw data
  BMP280ReadRegisters(0xF7, buf, 6); // Read 6 bytes of raw data starting from register 0xF7

  *raw_press = (int32_t)((buf[0] << 12) | (buf[1] << 4) | (buf[2] >> 4)); // Combine bytes to get raw pressure
  *raw_temp = (int32_t)((buf[3] << 12) | (buf[4] << 4) | (buf[5] >> 4)); // Combine bytes to get raw temperature
}

double BMP280CompensateTemp(int32_t raw_temp)
{
  double var1, var2, T;
  var1 = (((double)raw_temp) / 16384.0 - ((double)calib.dig_T1) / 1024.0) * ((double)calib.dig_T2);
  var2 = ((((double)raw_temp)/131072.0 - ((double)calib.dig_T1)/8192.0) * 
          (((double)raw_temp)/131072.0 - ((double)calib.dig_T1)/8192.0)) * ((double)calib.dig_T3);
  t_fine = (int32_t)(var1 + var2);
  T = (var1 + var2) / 5120.0;
  return T; // Return temperature in degrees Celsius
}

double BMP280CompensatePress(int32_t raw_press)
{
  double var1, var2, p;
  var1 = ((double)t_fine / 2.0) - 64000.0;
  var2 = var1 * var1 * ((double)calib.dig_P6) / 32768.0;
  var2 = var2 + var1 * ((double)calib.dig_P5) * 2.0;
  var2 = (var2 / 4.0) + (((double)calib.dig_P4) * 65536.0);
  var1 = (((double)calib.dig_P3) * var1 * var1 / 524288.0 + ((double)calib.dig_P2) * var1) / 524288.0;
  var1 = (1.0 + var1 / 32768.0) * ((double)calib.dig_P1);
  
  if (var1 == 0.0)
    return 0; // Avoid division by zero

  p = 1048576.0 - (double)raw_press;
  p = (p - (var2 / 4096.0)) * 6250.0 / var1;
  var1 = ((double)calib.dig_P9) * p * p / 2147483648.0;
  var2 = p * ((double)calib.dig_P8) / 32768.0;
  
  p = p + (var1 + var2 + ((double)calib.dig_P7)) / 16.0;

  return p; // Return pressure in Pascals
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
