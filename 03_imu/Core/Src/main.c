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
#include <math.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct {
  float w, x, y, z;
} Quaternion;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define ISM_WHO_AM_I  0x0F
#define ISM_CTRL1_XL  0x10 // Accelerometer Control: 0DR + full-scale
#define ISM_CTRL2_G   0x11 // Gyroscope Control: 0DR + full-scale
#define ISM_OUTX_L_G  0x22 // Gyroscope X-axis output, low byte (data block starts here)
#define ISM_OUTX_L_A  0x28 // Accelerometer X-axis output, low byte (data block starts here)
#define ACCEL_SENS_2G 0.061f // milli g's per count
#define GYRO_SENS_250DPS 8.75f // milli dps per count
#define GRAVITY 9.80665f // m/s^2
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef hspi1;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_SPI1_Init(void);
/* USER CODE BEGIN PFP */
uint8_t IMU_ReadRegister(uint8_t reg);
void IMU_WriteRegister(uint8_t reg, uint8_t value);
void IMU_ReadRegisters(uint8_t reg, uint8_t *buffer, uint8_t len);

Quaternion quat_multiply(Quaternion a, Quaternion b);
Quaternion quat_conjugate(Quaternion q);
Quaternion quat_normalize(Quaternion q);
void quat_print(const char *label, Quaternion q);

Quaternion quat_integrate(Quaternion q, float wx, float wy, float wz, float dt); // integrate angular velocity into quaternion
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
  // CS rests HIGH (deselected). Overrides CubeMX's startup LOW
  HAL_GPIO_WritePin(IMU_CS_GPIO_Port, IMU_CS_Pin, GPIO_PIN_SET);

  uint8_t who_am_i = IMU_ReadRegister(ISM_WHO_AM_I);
  printf("ISM330DHCX IMU WHO_AM_I: 0x%02X (expected: 0x6B)\r\n", who_am_i);

  // Wake accelerometer: 104 Hz 0DR, +-2g
  IMU_WriteRegister(ISM_CTRL1_XL, 0x40);
  // Wake gyroscope: 104 Hz 0DR, +-250 dps
  IMU_WriteRegister(ISM_CTRL2_G, 0x40);

  // Read the control registers back to confirm the wake-up took
  printf("CTRL1_XL: 0x%02X (expected 0x40)\r\n", IMU_ReadRegister(ISM_CTRL1_XL));
  printf("CTRL2_G:  0x%02X (expected 0x40)\r\n", IMU_ReadRegister(ISM_CTRL2_G));

  //Primitive tests for quaternion functions
  printf("\r\n-- Quaternion primitive tests --\r\n");

  Quaternion identity = {1.0f, 0.0f, 0.0f, 0.0f};
  Quaternion q90z = {0.7071068f, 0.0f, 0.0f, 0.7071068f}; // 90 deg rotation about Z
  Quaternion q60x = {0.8660254f, 0.5f, 0.0f, 0.0f}; // 60 deg rotation about X

  // Test 1: anything times identity is unchnaged
  quat_print("T1 got: ", quat_multiply(q90z, identity));
  printf("T1 expected: w=0.7071  x=0.0000  y=0.0000  z=0.7071\r\n");

  // Test 2: 90 about Z twice is 180 about Z
  quat_print("T2 got: ", quat_multiply(q90z, q90z));
  printf("T2 expected: w=0.0000  x=0.0000  y=0.0000  z=1.0000\r\n");

  // Test 3: Rotate then unrotate is identity
  quat_print("T3 got: ", quat_multiply(q60x, quat_conjugate(q60x)));
  printf("T3 expected: w=1.0000  x=0.0000  y=0.0000  z=0.0000\r\n");

  // Test 4: normalize undoes scaling
  Quaternion scaled = {3.0f*0.7071068f, 0.0f, 0.0f, 3.0f*0.7071068f};
  quat_print("T4 got: ", quat_normalize(scaled));
  printf("T4 expected: w=0.7071  x=0.0000  y=0.0000  z=0.7071\r\n");

  printf("\r\n-- End Quaternion primitive tests --\r\n\r\n");

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  uint8_t gyro_buf[6];

  Quaternion q = {1.0f, 0.0f, 0.0f, 0.0f}; // start at identity (no rotation)
  const float DEG2RAD = 3.14159265358979323846f / 180.0f;
  uint32_t last_tick = HAL_GetTick();

  while (1)
  {
    uint32_t now = HAL_GetTick();
    float dt = (now - last_tick) / 1000.0f; // convert ms to seconds
    last_tick = now;

    IMU_ReadRegisters(ISM_OUTX_L_G, gyro_buf, 6);
    int16_t gx = (int16_t)(gyro_buf[1] << 8 | gyro_buf[0]);
    int16_t gy = (int16_t)(gyro_buf[3] << 8 | gyro_buf[2]);
    int16_t gz = (int16_t)(gyro_buf[5] << 8 | gyro_buf[4]);

    // raw -> dps -> rad/s
    float wx = gx * GYRO_SENS_250DPS / 1000.0f * DEG2RAD;
    float wy = gy * GYRO_SENS_250DPS / 1000.0f * DEG2RAD;
    float wz = gz * GYRO_SENS_250DPS / 1000.0f * DEG2RAD;

    q = quat_integrate(q, wx, wy, wz, dt);

    quat_print("attitude: ", q);

    HAL_Delay(200); // 200 ms delay

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

uint8_t IMU_ReadRegister(uint8_t reg)
{
  uint8_t tx[2];
  uint8_t rx[2];

  tx[0] = reg | 0x80; // Address byte, bit 7 set = READ
  tx[1] = 0x00; // Dummy byte to clock the answer back

  HAL_GPIO_WritePin(IMU_CS_GPIO_Port, IMU_CS_Pin,  GPIO_PIN_RESET); // select
  HAL_SPI_TransmitReceive(&hspi1, tx, rx, 2, HAL_MAX_DELAY); // exchange 2 bytes
  HAL_GPIO_WritePin(IMU_CS_GPIO_Port, IMU_CS_Pin, GPIO_PIN_SET); // deselect

  return rx[1]; // The sensors answers landed in the second byte   
}

void IMU_WriteRegister(uint8_t reg, uint8_t value)
{
  uint8_t tx[2];
  tx[0] = reg & 0x7F; // bit 7 = 0 -> Write
  tx[1] = value;

  HAL_GPIO_WritePin(IMU_CS_GPIO_Port, IMU_CS_Pin, GPIO_PIN_RESET); // select
  HAL_SPI_Transmit(&hspi1, tx, 2, HAL_MAX_DELAY);
  HAL_GPIO_WritePin(IMU_CS_GPIO_Port, IMU_CS_Pin, GPIO_PIN_SET); // deselect
}

void IMU_ReadRegisters(uint8_t reg, uint8_t *buffer, uint8_t len)
{
  uint8_t addr = reg | 0x80; // bit 7 = 1 -> Read

  HAL_GPIO_WritePin(IMU_CS_GPIO_Port, IMU_CS_Pin, GPIO_PIN_RESET); // select
  HAL_SPI_Transmit(&hspi1, &addr, 1, HAL_MAX_DELAY); // send address
  HAL_SPI_Receive(&hspi1, buffer, len, HAL_MAX_DELAY); // read data
  HAL_GPIO_WritePin(IMU_CS_GPIO_Port, IMU_CS_Pin, GPIO_PIN_SET); // deselect

}

// Quaternion Functions
// Compose two rotations: applies b first, then a.
// Order matters -- quaternion multiplication is not commutative.
Quaternion quat_multiply(Quaternion a, Quaternion b)
{
  Quaternion r;
  r.w = a.w*b.w - a.x*b.x - a.y*b.y - a.z*b.z;
  r.x = a.w*b.x + a.x*b.w + a.y*b.z - a.z*b.y;
  r.y = a.w*b.y - a.x*b.z + a.y*b.w + a.z*b.x;
  r.z = a.w*b.z + a.x*b.y - a.y*b.x + a.z*b.w;
  return r;
}

//Reverse the rotation: same axis, opposite direction.
Quaternion quat_conjugate(Quaternion q)
{
  Quaternion r = {q.w, -q.x, -q.y, -q.z};
  return r;
}

// Resscale to unit length. Re-imposes w^2+x^2+y^2+z^2=1.
Quaternion quat_normalize(Quaternion q)
{
  float n = sqrtf(q.w*q.w + q.x*q.x + q.y*q.y + q.z*q.z);
  if (n < 1e-6f) {
    Quaternion id = {1.0f, 0.0f, 0.0f, 0.0f};
    return id; // Return identity quaternion if input is too small
  }
  Quaternion r = {q.w/n, q.x/n, q.y/n, q.z/n};
  return r;
}

void quat_print(const char *label, Quaternion q)
{
  printf("%-12s w=%7.4f x=%7.4f y=%7.4f z=%7.4f\r\n", label, q.w, q.x, q.y, q.z); 
}

//Integrate gyro rates (rad/s) into the orientation quaternion over dt seconds.
// q_new = q + (1/2)(q (x) (o,wx,wy,wz)) * dt, then normalize
Quaternion quat_integrate(Quaternion q, float wx, float wy, float wz, float dt)
{
  Quaternion omega = {0.0f, wx, wy, wz};    // gyro as a pure quaternion
  Quaternion qdot = quat_multiply(q, omega); // quaternion (x) (0, w)

  // scale by 1/2, then by dt, and add to q
  Quaternion q_new;
  q_new.w = q.w + 0.5f * qdot.w * dt;
  q_new.x = q.x + 0.5f * qdot.x * dt;
  q_new.y = q.y + 0.5f * qdot.y * dt;
  q_new.z = q.z + 0.5f * qdot.z * dt;

  return quat_normalize(q_new); // normalize to unit length
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
