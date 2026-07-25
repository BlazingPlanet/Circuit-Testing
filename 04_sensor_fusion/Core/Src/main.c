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
#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_tim.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdint.h>
#include <stdio.h>
#include <math.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct {
  float w, x, y, z;
} Quaternion;

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

TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
volatile uint8_t tick_ready = 0;

BMP280_Calib calib; // Structure to hold calibration data
int32_t t_fine; // Variable to hold the fine temperature value for compensation

float p0; // Pressure at ground
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_SPI1_Init(void);
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */

// IMU & Orientation Functions
uint8_t IMU_ReadRegister(uint8_t reg);
void IMU_WriteRegister(uint8_t reg, uint8_t value);
void IMU_ReadRegisters(uint8_t reg, uint8_t *buffer, uint8_t len);

Quaternion quat_multiply(Quaternion a, Quaternion b);
Quaternion quat_conjugate(Quaternion q);
Quaternion quat_normalize(Quaternion q);
void quat_print(const char *label, Quaternion q);

Quaternion quat_integrate(Quaternion q, float wx, float wy, float wz, float dt); // integrate angular velocity into quaternion

void quat_rotate_vector(Quaternion q, float vx, float vy, float vz, float *rx, float *ry, float *rz);

void quat_to_euler(Quaternion q, float *roll, float *pitch, float *yaw);

// BMP 280 Functions
uint8_t BMP280_ReadRegister(uint8_t reg); // Function to read a register from the BMP280 sensor

void BMP280ReadRegisters(uint8_t reg, uint8_t *buffer, uint16_t len); // Function to read multiple registers from the BMP280 sensor
void BMP280_ReadCalibration(void); // Function to read calibration data from the BMP280 sensor
void BMP280ReadRaw(int32_t *raw_temp, int32_t *raw_press); // Function to read raw temperature and pressure data from the BMP280 sensor
void BMP280_WriteRegister(uint8_t reg, uint8_t value);
float BMP280CompensateTemp(int32_t raw_temp); // Function to compensate the raw temperature data using calibration data
float BMP280CompensatePress(int32_t raw_press); // Function to compensate the raw pressure data using calibration data

// Altitude Functions
float pressure_to_altitude(float pressure_pa, float p0_pa);

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
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
  // CS rests HIGH (deselected). Overrides CubeMX's startup LOW
  HAL_GPIO_WritePin(IMU_CS_GPIO_Port, IMU_CS_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(BMP_CS_GPIO_Port, BMP_CS_Pin, GPIO_PIN_SET);
  HAL_Delay(10);

  uint8_t who_am_i = IMU_ReadRegister(ISM_WHO_AM_I);
  printf("ISM330DHCX IMU WHO_AM_I: 0x%02X (expected: 0x6B)\r\n", who_am_i);

  uint8_t BMP280_ID = BMP280_ReadRegister(0xD0); // Read the chip ID register
  printf("BMP280 Chip ID: 0x%02X (expected: 0x58)\r\n", BMP280_ID); // Print the chip ID to UART

  BMP280_ReadCalibration();
  printf("dig_T1 = %u  dig_P1 = %u\r\n", calib.dig_T1, calib.dig_P1);

  BMP280_WriteRegister(0xF4, 0x57);  // temp x2, press x16, normal mode
  HAL_Delay(100);

  // Establish ground reference pressure by averaging 32 samples
  float p0_sum = 0.0f;
  for (int i = 0; i < 32; i++) {
    int32_t rt, rp;
    BMP280ReadRaw(&rt, &rp);
    BMP280CompensateTemp(rt);
    float sample = BMP280CompensatePress(rp);
    p0_sum += sample;
    HAL_Delay(50);
  }
  p0 = p0_sum / 32.0f;
  printf("Ground reference P0: %.2f Pa\r\n", p0);

  // Wake accelerometer: 104 Hz 0DR, +-2g
  IMU_WriteRegister(ISM_CTRL1_XL, 0x40);
  // Wake gyroscope: 104 Hz 0DR, +-250 dps
  IMU_WriteRegister(ISM_CTRL2_G, 0x40);

  // Read the control registers back to confirm the wake-up took
  printf("CTRL1_XL: 0x%02X (expected 0x40)\r\n", IMU_ReadRegister(ISM_CTRL1_XL));
  printf("CTRL2_G:  0x%02X (expected 0x40)\r\n", IMU_ReadRegister(ISM_CTRL2_G));

  HAL_TIM_Base_Start_IT(&htim2);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  uint8_t accel_buf[6];
  uint8_t gyro_buf[6];

  Quaternion q = {1.0f, 0.0f, 0.0f, 0.0f}; // start at identity (no rotation)
  float bx = 0.0f, by = 0.0f, bz = 0.0f; // estimated gyro bias (rad/s)
  const float DEG2RAD = 3.14159265358979323846f / 180.0f;

  const float Kp = 2.0f; // proportional gain for Mahoney filter (tubing knob)
  const float Ki = 0.01f; // integral gain
  const float TRUST_BAND = 0.10f;  // full trust within +/- this many g of 1.0
  const float TRUST_ZERO = 0.30f;  // zero trust beyond this many g from 1.0

  uint32_t last_tick = HAL_GetTick();
  uint32_t pass_count = 0;
  float altitude = 0.0f;

  float h = 0.0f; // fused altitude estimate (m above pad)
  float v = 0.0f; // fused vertical velocity estimate (m/s)
  const float Kh = 0.30f; // how hard baro pulls the altitude estimate
  const float Kv = 0.60f; // how hard baro pulls the velocity estimate

  while (1)
  {
    if (tick_ready) {
      tick_ready = 0;
      pass_count++;

      uint32_t now = HAL_GetTick();
      float dt = (now - last_tick) / 1000.0f; // convert ms to seconds
      last_tick = now;

      if (pass_count % 8 == 0) {       // 200 Hz / 8 = 25 Hz
        int32_t rt, rp;
        BMP280ReadRaw(&rt, &rp);
        BMP280CompensateTemp(rt);
        float press = BMP280CompensatePress(rp);
        altitude = pressure_to_altitude(press, p0);

        // Correction step (25 Hz)
        float alt_err = altitude - h;
        h += Kh * alt_err;
        v += Kv * alt_err;
      }

      IMU_ReadRegisters(ISM_OUTX_L_G, gyro_buf, 6);
      IMU_ReadRegisters(ISM_OUTX_L_A, accel_buf, 6);

      int16_t ax = (int16_t)(accel_buf[1] << 8 | accel_buf[0]);
      int16_t ay = (int16_t)(accel_buf[3] << 8 | accel_buf[2]);
      int16_t az = (int16_t)(accel_buf[5] << 8 | accel_buf[4]);
      int16_t gx = (int16_t)(gyro_buf[1] << 8 | gyro_buf[0]);
      int16_t gy = (int16_t)(gyro_buf[3] << 8 | gyro_buf[2]);
      int16_t gz = (int16_t)(gyro_buf[5] << 8 | gyro_buf[4]);

      // raw -> dps -> rad/s
      float wx = gx * GYRO_SENS_250DPS / 1000.0f * DEG2RAD;
      float wy = gy * GYRO_SENS_250DPS / 1000.0f * DEG2RAD;
      float wz = gz * GYRO_SENS_250DPS / 1000.0f * DEG2RAD;

      // --- Step 2/3: accel correction ---
      // normalize the accelermoeter vector (we only care about DIRECTION)
      float an = sqrtf((float)ax*ax + (float)ay*ay + (float)az*az);
      float an_g = an * ACCEL_SENS_2G / 1000.0f;  // raw counts -> g

      float g_err = fabsf(an_g - 1.0f); //how far from pure gravity?

      // Trust factor
      float trust;
      if (g_err <= TRUST_BAND) {
        trust = 1.0f;
      } else if (g_err >= TRUST_ZERO) {
        trust = 0.0f;
      } else {
        trust = 1.0f - (g_err - TRUST_BAND) / (TRUST_ZERO - TRUST_BAND);
      }

      if (an > 1e-3f) {
        float max = ax / an, may = ay / an, maz = az / an; // measured gravity direction

        // predicted gravity in body frame = q* applied to world down (0,0,1)
        float px, py, pz;
        quat_rotate_vector(quat_conjugate(q), 0.0f, 0.0f, 1.0f, &px, &py, &pz);

        // error = measured x predicted (cross product)
        float ex = may*pz - maz*py;
        float ey = maz*px - max*pz;
        float ez = max*py - may*px;

        // Accumulate the error into bias
        bx += Ki * trust * ex * dt;
        by += Ki * trust * ey * dt;
        bz += Ki * trust * ez * dt;

        // anti-windup: real gyro bias is tiny (~a few dps). Cap it.
        const float BIAS_MAX = 0.05f;  // rad/s, ~3 dps
        if (bx > BIAS_MAX) bx = BIAS_MAX;
        if (bx < -BIAS_MAX) bx = -BIAS_MAX;
        if (by > BIAS_MAX) by = BIAS_MAX;
        if (by < -BIAS_MAX) by = -BIAS_MAX;
        if (bz > BIAS_MAX) bz = BIAS_MAX;
        if (bz < -BIAS_MAX) bz = -BIAS_MAX;

        // --- Step 4: feed error back into the gyro rate ---
        wx += Kp * trust * ex;
        wy += Kp * trust * ey;
        wz += Kp * trust * ez;
      }

      wx -= bx;
      wy -= by;
      wz -= bz;

      q = quat_integrate(q, wx, wy, wz, dt);

      // --- world-frame linear acceleration
      // Raw counts --> g (physical units, not the normalized direction)
      // vector the Mahony correction uses
      float ax_g = ax * ACCEL_SENS_2G / 1000.0f;
      float ay_g = ay * ACCEL_SENS_2G / 1000.0f;
      float az_g = az * ACCEL_SENS_2G / 1000.0f;

      // Body --> world (pass q, not its conjugate)
      float wax, way, waz;
      quat_rotate_vector(q, ax_g, ay_g, az_g, &wax, &way, &waz);

      // Remove gravity: world down is +Z, so a stationary sensor reads +1g
      float lin_accel_z = (waz - 1.0f) * GRAVITY;   // m/s^2

      // Vertical channel, predict step (200 Hz)
      h += v * dt + 0.5f * lin_accel_z * dt * dt;
      v += lin_accel_z * dt;

      // print out the Euler values for us to read
      float roll, pitch, yaw;
      quat_to_euler(q, &roll, &pitch, &yaw);
      
      if (pass_count % 10 == 0) {    // 200 Hz / 40 = ~5 lines per second
        //printf("R:%7.2f P:%7.2f Y:%7.2f | mag:%4.2fg trust:%4.2f alt:%6.2fm  linZ:%6.2f m/s^2\r\n",
          //roll, pitch, yaw, an_g, trust, altitude, lin_accel_z);
        printf("alt:%6.2f  h:%6.2f  v:%6.2f m/s  linZ:%6.2f\r\n",
             altitude, h, v, lin_accel_z);
      }
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
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 8399;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 49;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

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
  HAL_GPIO_WritePin(IMU_CS_GPIO_Port, IMU_CS_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(BMP_CS_GPIO_Port, BMP_CS_Pin, GPIO_PIN_SET);

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

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if (htim->Instance == TIM2) {
    tick_ready = 1;
  }
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

// Rotate vector v by quaternion q: v' = q (x) (0,v) (x) q*
// Pass q for body -> world, or pass q* for world -> body
void quat_rotate_vector(Quaternion q, float vx, float vy, float vz, float *rx, float *ry, float *rz)
{
  Quaternion v = {0.0f, vx, vy, vz};
  Quaternion qc = quat_conjugate(q);
  Quaternion tmp = quat_multiply(q, v);
  Quaternion res = quat_multiply(tmp, qc);
  *rx = res.x;
  *ry = res.y;
  *rz = res.z;
}

// Convert quaternion to roll/pitch/yaw in degrees (ZYX convention)
// FOR DISPLAY ONLY -- the quaternion remains the authoritative state
// Degenerates near pitch = +/- 90 deg (singularity point in OUTPUT, not the real estimate)
void quat_to_euler(Quaternion q, float *roll, float *pitch, float *yaw)
{
  const float RAD2DEG = 180.0f / 3.14159265f;

  // roll (rotation about X)
  float sinr_cosp = 2.0f * (q.w*q.x + q.y*q.z);
  float cosr_cosp = 1.0f - 2.0f * (q.x*q.x + q.y*q.y);
  *roll = atan2f(sinr_cosp, cosr_cosp) * RAD2DEG;

  // pitch (rotation about Y)
  float sinp = 2.0f * (q.w*q.y - q.z*q.x);
  if (sinp > 1.0f) sinp = 1.0f;    // clamp -- guards asinf against
  if (sinp < -1.0f) sinp = -1.0f;  // domain error from float rounding
  *pitch = asinf(sinp) * RAD2DEG;

  // yaw (rotation about Z)
  float siny_cosp = 2.0f * (q.w*q.z + q.x*q.y);
  float cosy_cosp = 1.0f - 2.0f * (q.y*q.y + q.z*q.z);
  *yaw = atan2f(siny_cosp, cosy_cosp) * RAD2DEG;
}

// ---
// BMP 280 Functions
// ---

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

void BMP280_WriteRegister(uint8_t reg, uint8_t value)
{
    uint8_t tx[2];
    tx[0] = reg & 0x7F;   // address byte, bit 7 cleared = WRITE
    tx[1] = value;        // the byte to write into that register

    HAL_GPIO_WritePin(BMP_CS_GPIO_Port, BMP_CS_Pin, GPIO_PIN_RESET);  // CS low
    HAL_SPI_Transmit(&hspi1, tx, 2, HAL_MAX_DELAY);                   // send both bytes
    HAL_GPIO_WritePin(BMP_CS_GPIO_Port, BMP_CS_Pin, GPIO_PIN_SET);    // CS high
}

float BMP280CompensateTemp(int32_t raw_temp)
{
  float var1, var2, T;
  var1 = (((float)raw_temp) / 16384.0f - ((float)calib.dig_T1) / 1024.0f) * ((float)calib.dig_T2);
  var2 = ((((float)raw_temp)/131072.0f - ((float)calib.dig_T1)/8192.0f) * 
          (((float)raw_temp)/131072.0f - ((float)calib.dig_T1)/8192.0f)) * ((float)calib.dig_T3);
  t_fine = (int32_t)(var1 + var2);
  T = (var1 + var2) / 5120.0f;
  return T; // Return temperature in degrees Celsius
}

float BMP280CompensatePress(int32_t raw_press)
{
  float var1, var2, p;
  var1 = ((float)t_fine / 2.0f) - 64000.0f;
  var2 = var1 * var1 * ((float)calib.dig_P6) / 32768.0f;
  var2 = var2 + var1 * ((float)calib.dig_P5) * 2.0f;
  var2 = (var2 / 4.0) + (((float)calib.dig_P4) * 65536.0f);
  var1 = (((float)calib.dig_P3) * var1 * var1 / 524288.0f + ((float)calib.dig_P2) * var1) / 524288.0f;
  var1 = (1.0f + var1 / 32768.0f) * ((float)calib.dig_P1);
  
  if (var1 == 0.0f)
    return 0; // Avoid division by zero

  p = 1048576.0f - (float)raw_press;
  p = (p - (var2 / 4096.0f)) * 6250.0f / var1;
  var1 = ((float)calib.dig_P9) * p * p / 2147483648.0f;
  var2 = p * ((float)calib.dig_P8) / 32768.0f;
  
  p = p + (var1 + var2 + ((float)calib.dig_P7)) / 16.0f;

  return p; // Return pressure in Pascals
}

// Convert pressure to altitude relative to a ground reference
// Returns meters above the pad. Positive = higher than P0 (lower altitude)
float pressure_to_altitude(float pressure_pa, float p0_pa)
{
  return 44330.0f * (1.0f - powf(pressure_pa / p0_pa, 0.1903f));
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
