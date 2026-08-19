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

typedef enum {
  FLIGHT_DISARMED = 0,
  FLIGHT_PAD,
  FLIGHT_BOOST,
  FLIGHT_COAST,
  FLIGHT_DESCENT,
} FlightState;

typedef struct __attribute__((packed)) {
  uint32_t t_ms;                  // 4
  float qw, qx, qy, qz;           // 16
  float wx, wy, wz;               // 12 (post bias correction, rad/s)
  float ax_g, ay_g, az_g;         // 12 (raw accel, g)
  float err_y, err_z;             // 8
  float    cmd_y, cmd_z;          // 8   (gimbal degrees)
  uint16_t pulse_y, pulse_z;      // 4   (µs)
  float    kf_h, kf_v, kf_b;      // 12
  uint8_t  state;                 // 1
  uint8_t  flags;                 // 1
} LogRecord;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define ISM_WHO_AM_I  0x0F
#define ISM_CTRL1_XL  0x10 // Accelerometer Control: 0DR + full-scale
#define ISM_CTRL2_G   0x11 // Gyroscope Control: 0DR + full-scale
#define ISM_OUTX_L_G  0x22 // Gyroscope X-axis output, low byte (data block starts here)
#define ISM_OUTX_L_A  0x28 // Accelerometer X-axis output, low byte (data block starts here)
#define ACCEL_SENS_8G 0.244f // milli g's per count (+-8g range)
#define GYRO_SENS_1000DPS 35.0f // milli dps per count (+-1000 dps range)
#define GRAVITY 9.80665f // m/s^2

// Flight state machine thresholds
#define LAUNCH_ACCEL_THRESH 15.0f // m/s^2, well above noise. replace with expected launch accel from Open Rocket
#define BURNOUT_ACCEL_THRESH 5.0f // m/s^2, thrust gone
#define APOGEE_VEL_THRESH -2.0f // m/s, definitively descending
#define DEBOUNCE_PASSES 20 // 20 passes at 200 Hz = 100ms
#define SIM_MODE 0  // 1 = synthetic flight profile, 0 = real sensors

#define ARM_TILT_MAX     35.0f  // degrees from vertical
#define ARM_GYRO_MAX     0.25f  // rad/s, any axis, must be below to enter armed state
#define ARM_ACCEL_BAND   0.30f  // g, deviation from 1.0g must be below to enter armed state
#define ARM_HOLD_PASSES  6000   // 30s hold time

// Kalman filter parameters
#define KF_ACCEL_NOISE 0.15f // m/s^2, accelerometer noise std dev
#define KF_BIAS_WALK  0.005f  // m/s^2 per sqrt(s), how fast bias drifts
#define KF_BARO_NOISE  0.40f  // m, barometer noise std dev (I measured this)

// FLAG BITS
#define LOG_FLAG_SAT_Y     0x01   // cmd_y clamped at MAX_DEFLECT
#define LOG_FLAG_SAT_Z     0x02   // cmd_z clamped at MAX_DEFLECT
#define LOG_FLAG_PULSE_Y   0x04   // pulse_y hit µs bound -- should never fire
#define LOG_FLAG_PULSE_Z   0x08   // pulse_z hit µs bound -- should never fire
#define LOG_FLAG_NO_TRUST  0x10   // accel trust was zero
#define LOG_FLAG_OVERRUN   0x20   // loop didn't finish before next tick

// ---- Servo calibration (measured 2026-07-31, 6.75" lever arm) ----
// Channel map:
//   TIM3_CH1 = PB4 = Arduino D5 = mount "Y" axis servo
//   TIM3_CH2 = PB5 = Arduino D4 = mount "X" axis servo
#define SERVO_MY_TRIM   1575    // µs at gimbal neutral
#define SERVO_MY_USPD   44.4f   // µs per gimbal degree
#define SERVO_MY_MIN    1200    // µs, inside mechanical stop at 1150
#define SERVO_MY_MAX    1900    // µs, inside mechanical stop at 1950

#define SERVO_MX_TRIM   1825    // µs, at gimbal neutral
#define SERVO_MX_USPD   48.5f   // µs, per gimbal degree
#define SERVO_MX_MIN    1475    // µs, inside mechanical stop at 1425
#define SERVO_MX_MAX    2300    // µs, inside mechanical stop at 2350

#define MAX_DEFLECT     6.0f    // gimbal degrees, both axes

#define SERVO_MY_SIGN   (+1.0f) // TBD -- verify by hand before flight
#define SERVO_MX_SIGN   (+1.0f) // TBD -- verify by hand before flight

#define BENCH_TEST 0  // TEMPORARY: force controller active for bench sign check

// SPI Timeouts
#define SPI_TIMEOUT_MS  10 // ms

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

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
static void MX_TIM3_Init(void);
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

void attitude_tilt(Quaternion q, float *tilt_deg, float *err_y, float *err_z);

Quaternion quat_from_two_vectors(float ax, float ay, float az, float bx_, float by_, float bz_);

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

// SIM Functions
#if SIM_MODE
float sim_lin_accel_z(float t);
#endif

// Flash Chip Functions
void Flash_ReadJEDEC(uint8_t *buf);

// Control Functions
static uint16_t gimbal_to_pulse(float cmd_deg, float sign, uint16_t trim, float us_per_deg, uint16_t min_us, uint16_t max_us);
                                
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
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */
  printf("LogRecord size: %u bytes (expected 78)\r\n", (unsigned)sizeof(LogRecord));

  // CS rests HIGH (deselected). Overrides CubeMX's startup LOW
  HAL_GPIO_WritePin(IMU_CS_GPIO_Port, IMU_CS_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(BMP_CS_GPIO_Port, BMP_CS_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(FLASH_CS_GPIO_Port, FLASH_CS_Pin, GPIO_PIN_SET);
  HAL_Delay(10);

  // Enable DWT cycle counter for loop timing instrumentaion
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  DWT->CYCCNT = 0;
  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

  uint8_t who_am_i = IMU_ReadRegister(ISM_WHO_AM_I);
  printf("ISM330DHCX IMU WHO_AM_I: 0x%02X (expected: 0x6B)\r\n", who_am_i);

  uint8_t BMP280_ID = BMP280_ReadRegister(0xD0); // Read the chip ID register
  printf("BMP280 Chip ID: 0x%02X (expected: 0x58)\r\n", BMP280_ID); // Print the chip ID to UART

  uint8_t jedec[3];
  Flash_ReadJEDEC(jedec);
  printf("W25Q128 JEDEC: %02X %02X %02X (expected EF 40 18)\r\n", jedec[0], jedec[1], jedec[2]);

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

  // Wake accelerometer: 208 Hz 0DR, +-8g
  IMU_WriteRegister(ISM_CTRL1_XL, 0x5C);
  // Wake gyroscope: 208 Hz 0DR, +-1000 dps
  IMU_WriteRegister(ISM_CTRL2_G, 0x58);

  // Read the control registers back to confirm the wake-up took
  printf("CTRL1_XL: 0x%02X (expected 0x5C)\r\n", IMU_ReadRegister(ISM_CTRL1_XL));
  printf("CTRL2_G:  0x%02X (expected 0x58)\r\n", IMU_ReadRegister(ISM_CTRL2_G));

  HAL_TIM_Base_Start_IT(&htim2);

  // Servo PWM on TIM3: CH1 = PB4, CH2 = PB5
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, SERVO_MY_TRIM);   // y axis servo
  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, SERVO_MX_TRIM);   // x axis servo

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  uint8_t accel_buf[6];
  uint8_t gyro_buf[6];

   // Seed attitude from gravity. Board must be stationary at boot
  HAL_Delay(50);   // let the accel produce a valid sample after wake up

  uint8_t init_buf[6];
  IMU_ReadRegisters(ISM_OUTX_L_A, init_buf, 6);
  int16_t iax = (int16_t)(init_buf[1] << 8 | init_buf[0]);
  int16_t iay = (int16_t)(init_buf[3] << 8 | init_buf[2]);
  int16_t iaz = (int16_t)(init_buf[5] << 8 | init_buf[4]);
  float inorm = sqrtf((float)iax*iax + (float)iay*iay + (float)iaz*iaz);

  Quaternion q = {1.0f, 0.0f, 0.0f, 0.0f}; // start at identity (no rotation)
  if (inorm > 1e-3f) {
    // Measured world-up direction in body coordinates
    float ux0 = iax / inorm, uy0 = iay / inorm, uz0 = iaz / inorm;
    // Rotate carrying world-up (0,0,1) onto the measured direction
    q = quat_from_two_vectors(ux0, uy0, uz0, 0.0f, 0.0f, 1.0f);
  }
  quat_print("seed q:", q);

  float bx = 0.0f, by = 0.0f, bz = 0.0f; // estimated gyro bias (rad/s)
  const float DEG2RAD = 3.14159265358979323846f / 180.0f;

  const float Kp = 2.0f; // proportional gain for Mahoney filter (tubing knob)
  const float Ki = 0.01f; // integral gain
  const float TRUST_BAND = 0.10f;  // full trust within +/- this many g of 1.0
  const float TRUST_ZERO = 0.30f;  // zero trust beyond this many g from 1.0

  uint32_t last_tick = HAL_GetTick();
  uint32_t pass_count = 0;
  float altitude = 0.0f;

  uint32_t worst_cycles = 0;    // longest tick seen, in CPU cycles
  uint32_t overrun_count = 0;   // ticks that missed their deadline
  uint32_t nan_count = 0;       // NaN resets encountered (should be zero)

  // Complimentary filter values for altitude/velocity fusion (OLD)

  //float h = 0.0f; // fused altitude estimate (m above pad)
  //float v = 0.0f; // fused vertical velocity estimate (m/s)
  //const float Kh = 0.30f; // how hard baro pulls the altitude estimate
  //const float Kv = 0.60f; // how hard baro pulls the velocity estimate

  FlightState flight_state = FLIGHT_DISARMED;
  uint16_t debounce_count = 0;
  uint32_t arm_count = 0;

  // --- Kalman state ---
  float kf_h = 0.0f;   // altitude estimate (m)
  float kf_v = 0.0f;   // velocity estimate (m/s)
  float kf_b = 0.0f;   // accelerometer bias (m/s^2)

  // Covariance, symmetric 3x3 (six unique terms)
  float p00 = 1.0f,  p01 = 0.0f,  p02 = 0.0f;
  float p11 = 1.0f,  p12 = 0.0f;
  float p22 = 1.0f;  
  
  // Control gain
  const float Kp_att = 30.0f;  // gimbal degrees per radian of attitude error <----- MUST BE TUNED
  const float Kd_att = 3.0f;   // gimbal degrees per rad/s of body rate <----- MUST BE TUNED

  while (1)
  {
    if (tick_ready) {
      tick_ready = 0;
      uint32_t t_start = DWT->CYCCNT;
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
#if !SIM_MODE        
        // OLD COMPLIMENTARY FILTER
        // float alt_err = altitude - h;
        // h += Kh * alt_err;
        // v += Kv * alt_err;
        
        // --- Kalman Update (25 Hz) --- 
        {
          float y = altitude - kf_h;
          float S = p00 + KF_BARO_NOISE * KF_BARO_NOISE;
          float k0 = p00 / S;
          float k1 = p01 / S;
          float k2 = p02 / S;

          kf_h += k0 * y;
          kf_v += k1 * y;
          kf_b += k2 * y;

          float m00 = (1.0f - k0) * p00;
          float m01 = (1.0f - k0) * p01;
          float m02 = (1.0f - k0) * p02;
          float m11 = p11 - k1 * p01;
          float m12 = p12 - k1 * p02;
          float m22 = p22 - k2 * p02;

          p00 = m00; p01 = m01; p02 = m02;
          p11 = m11; p12 = m12; p22 = m22;
        }
#endif
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
      float wx = gx * GYRO_SENS_1000DPS / 1000.0f * DEG2RAD;
      float wy = gy * GYRO_SENS_1000DPS / 1000.0f * DEG2RAD;
      float wz = gz * GYRO_SENS_1000DPS / 1000.0f * DEG2RAD;

      // --- Step 2/3: accel correction ---
      // Calcualte magnitude of accel readings
      float an = sqrtf((float)ax*ax + (float)ay*ay + (float)az*az);
      // Convert magnitude to g's and compare to 1.0g. g_err should read 0 at rest
      float an_g = an * ACCEL_SENS_8G / 1000.0f;  // raw counts -> g

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

        // predicted gravity in body frame = q* applied to world up (0,0,1)
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

      // NaN guard: a NaN quaternion is unrecoverable and defeats every
      // downstream clamp (all NaN comparisons are false). Reset and let the
      // Mahony filter re-converge from the accelerometer.
      if (isnan(q.w) || isnan(q.x) || isnan(q.y) || isnan(q.z)) {
        q.w = 1.0f; q.x = 0.0f; q.y = 0.0f; q.z = 0.0f;
        bx = 0.0f; by = 0.0f; bz = 0.0f;   // bias accumulated against bad attitude, reset it
        nan_count++;
      }
      // Attitude error for control
      float tilt, err_y, err_z;
      attitude_tilt(q, &tilt, &err_y, &err_z);

      // --- world-frame linear acceleration
      // Raw counts --> g (physical units, not the normalized direction)
      // vector the Mahony correction uses
      float ax_g = ax * ACCEL_SENS_8G / 1000.0f;
      float ay_g = ay * ACCEL_SENS_8G / 1000.0f;
      float az_g = az * ACCEL_SENS_8G / 1000.0f;

      // Body --> world (pass q, not its conjugate)
      float wax, way, waz;
      quat_rotate_vector(q, ax_g, ay_g, az_g, &wax, &way, &waz);

      // Remove gravity: world up is +Z, so a stationary sensor reads +1g
      float lin_accel_z = (waz - 1.0f) * GRAVITY;   // m/s^2

// SIM MODE LIN Z
#if SIM_MODE
      static float sim_t = 0.0f;
      sim_t += dt;
      lin_accel_z = sim_lin_accel_z(sim_t);
#endif

      // Vertical channel, predict step (200 Hz) (OLD COMPLIMENTARY FILER)
      // h += v * dt + 0.5f * lin_accel_z * dt * dt;
      // v += lin_accel_z * dt;

      // --- Kalman predict (200Hz) ---
      {
        float a = lin_accel_z - kf_b;

        kf_h += kf_v * dt + 0.5f * a *dt * dt;
        kf_v += a * dt;

        float c = -0.5f * dt * dt;
        float d = -dt;

        float f00 = p00 + dt*p01 + c*p02;
        float f01 = p01 + dt*p11 + c*p12;
        float f02 = p02 + dt*p12 + c*p22;
        float f10 = p01 + d*p02;
        float f11 = p11 + d*p12;
        float f12 = p12 + d*p22;
        float f22 = p22;

        float n00 = f00 + dt*f01 + c*f02;
        float n01 = f01 + d*f02;
        float n02 = f02;
        float n11 = f11 + d*f12;
        float n12 = f12;
        float n22 = f22;

        float sa2 = KF_ACCEL_NOISE * KF_ACCEL_NOISE;
        float dt2 = dt * dt;
        n00 += sa2 * dt2 * dt2 * 0.25f;
        n01 += sa2 * dt2 * dt * 0.5f;
        n11 += sa2 * dt2;
        n22 += KF_BIAS_WALK * KF_BIAS_WALK * dt;

        p00 = n00; p01 = n01; p02 = n02;
        p11 = n11; p12 = n12; p22 = n22;
      }

      // NaN guard on the vertical channel
      if (isnan(kf_h) || isnan(kf_v) || isnan(kf_b)) {
        kf_h = 0.0f; kf_v = 0.0f; kf_b = 0.0f;
        p00 = 1.0f; p01 = 0.0f; p02 = 0.0f;
        p11 = 1.0f; p12 = 0.0f; p22 = 1.0f;
        nan_count++;
      }

      // -- Flight State Machine --
      switch (flight_state) {

        case FLIGHT_DISARMED: 
          float gyro_mag = sqrtf(wx*wx + wy*wy + wz*wz);
          int still = (gyro_mag < ARM_GYRO_MAX) && (g_err < ARM_ACCEL_BAND);
          int upright = (tilt < ARM_TILT_MAX);

          if (still && upright) {
            arm_count++;
            if (arm_count >= ARM_HOLD_PASSES) {
              flight_state = FLIGHT_PAD;
              printf("*** ARMED ***\r\n");
            }
          } else {
            arm_count = 0;
          }
          break;
        
        case FLIGHT_PAD:
          if (lin_accel_z > LAUNCH_ACCEL_THRESH) {
            debounce_count++;
            if (debounce_count >= DEBOUNCE_PASSES) {
              flight_state = FLIGHT_BOOST;
              debounce_count = 0;
              printf("*** LAUNCH DETECTED ***\r\n");
            }
          } else {
            debounce_count = 0;
          }
          break;

        case FLIGHT_BOOST:
          if (lin_accel_z < BURNOUT_ACCEL_THRESH && kf_v > 0.0f) {
            debounce_count++;
            if (debounce_count >= DEBOUNCE_PASSES) {
              flight_state = FLIGHT_COAST;
              debounce_count = 0;
              printf("*** BURNOUT  v=%.1f m/s  h=%.1f m ***\r\n", kf_v, kf_h);
            }
          } else {
            debounce_count = 0;
          }
          break;

        case FLIGHT_COAST:
          if (kf_v < APOGEE_VEL_THRESH) {
            debounce_count++;
            if (debounce_count >= DEBOUNCE_PASSES) {
              flight_state = FLIGHT_DESCENT;
              debounce_count = 0;
              printf("*** APOGEE  h=%.1f m ***\r\n", kf_h);
            }
          } else {
            debounce_count = 0;
          }
          break;

        case FLIGHT_DESCENT:
          break;
      }
      
      // --- Control law: only active during BOOST phase ---
      float cmd_y = 0.0f;
      float cmd_z = 0.0f; 
  
    #if BENCH_TEST
      if (1) {
    #else
      if (flight_state == FLIGHT_BOOST) {
    #endif    

        float p_y = Kp_att * err_y;  // Proportional term Y
        float d_y = -Kd_att * wy;    // Derivative term Y
        cmd_y = p_y + d_y;     // command = P+D

        float p_z = Kp_att * err_z;  // Proportional term Y
        float d_z = -Kd_att * wz;    // Derivative term Z
        cmd_z = p_z + d_z;     // command = P+D

        // servo hard stop limits
        if (cmd_y > MAX_DEFLECT) cmd_y = MAX_DEFLECT;
        if (cmd_y < -MAX_DEFLECT) cmd_y = -MAX_DEFLECT;
        if (cmd_z > MAX_DEFLECT) cmd_z = MAX_DEFLECT;
        if (cmd_z < -MAX_DEFLECT) cmd_z = -MAX_DEFLECT;
      }

      // --- Mixing: runs every tick, sets servo to trim if not in BOOST ---
      uint16_t pulse_y = gimbal_to_pulse(cmd_y, SERVO_MY_SIGN, SERVO_MY_TRIM, SERVO_MY_USPD, SERVO_MY_MIN, SERVO_MY_MAX);
      uint16_t pulse_z = gimbal_to_pulse(cmd_z, SERVO_MX_SIGN, SERVO_MX_TRIM, SERVO_MX_USPD, SERVO_MX_MIN, SERVO_MX_MAX);

      __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, pulse_y);   // y axis servo
      __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, pulse_z);   // x axis servo

      // --- Log flags for this tick ---
      uint8_t flags = 0;
      if (cmd_y >= MAX_DEFLECT || cmd_y <= -MAX_DEFLECT) flags |= LOG_FLAG_SAT_Y;
      if (cmd_z >= MAX_DEFLECT || cmd_z <= -MAX_DEFLECT) flags |= LOG_FLAG_SAT_Z;
      if (pulse_y == SERVO_MY_MIN || pulse_y == SERVO_MY_MAX) flags |= LOG_FLAG_PULSE_Y;
      if (pulse_z == SERVO_MX_MIN || pulse_z == SERVO_MX_MAX) flags |= LOG_FLAG_PULSE_Z;
      if (trust == 0.0f) flags |= LOG_FLAG_NO_TRUST;

      static const char *state_names[] = {"DISARM", "PAD", "BOOST", "COAST", "DESCENT"};
      
      if (pass_count % 10 == 0) {    // 200 Hz / 10 = ~20 lines per second
              printf("%-7s tilt:%6.2f eY:%6.3f eZ:%6.3f | cy:%6.2f cz:%6.2f | py:%4u pz:%4u | f:%02X\r\n",
               state_names[flight_state], tilt, err_y, err_z, cmd_y, cmd_z,
               pulse_y, pulse_z, flags);     
      }

      // --- Loop timing instrumentation ---
      uint32_t t_elapsed = DWT->CYCCNT - t_start;
      if (t_elapsed > worst_cycles) worst_cycles = t_elapsed;
      if (tick_ready) {
        overrun_count++;
        flags |= LOG_FLAG_OVERRUN;
      }

      if (pass_count % 200 == 0) {   // once per second
        printf(" [timing] worst:%lu us  overruns:%lu\r\n",
               (unsigned long)(worst_cycles / 84), (unsigned long)overrun_count);
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
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 83;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 19999;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

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
  huart2.Init.BaudRate = 921600;
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
  HAL_GPIO_WritePin(GPIOB, FLASH_CS_Pin|BMP_CS_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(IMU_CS_GPIO_Port, IMU_CS_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : FLASH_CS_Pin BMP_CS_Pin */
  GPIO_InitStruct.Pin = FLASH_CS_Pin|BMP_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : IMU_CS_Pin */
  GPIO_InitStruct.Pin = IMU_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(IMU_CS_GPIO_Port, &GPIO_InitStruct);

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
  HAL_SPI_TransmitReceive(&hspi1, tx, rx, 2, SPI_TIMEOUT_MS); // exchange 2 bytes
  HAL_GPIO_WritePin(IMU_CS_GPIO_Port, IMU_CS_Pin, GPIO_PIN_SET); // deselect

  return rx[1]; // The sensors answers landed in the second byte   
}

void IMU_WriteRegister(uint8_t reg, uint8_t value)
{
  uint8_t tx[2];
  tx[0] = reg & 0x7F; // bit 7 = 0 -> Write
  tx[1] = value;

  HAL_GPIO_WritePin(IMU_CS_GPIO_Port, IMU_CS_Pin, GPIO_PIN_RESET); // select
  HAL_SPI_Transmit(&hspi1, tx, 2, SPI_TIMEOUT_MS);
  HAL_GPIO_WritePin(IMU_CS_GPIO_Port, IMU_CS_Pin, GPIO_PIN_SET); // deselect
}

void IMU_ReadRegisters(uint8_t reg, uint8_t *buffer, uint8_t len)
{
  uint8_t addr = reg | 0x80; // bit 7 = 1 -> Read

  HAL_GPIO_WritePin(IMU_CS_GPIO_Port, IMU_CS_Pin, GPIO_PIN_RESET); // select
  HAL_SPI_Transmit(&hspi1, &addr, 1, SPI_TIMEOUT_MS); // send address
  HAL_SPI_Receive(&hspi1, buffer, len, SPI_TIMEOUT_MS); // read data
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

// Shortest arc quaternion rotating vector a onto vector b
// Both inputs must be unit length. Returns identity if a and b are antiparallel
Quaternion quat_from_two_vectors(float ax, float ay, float az, float bx_, float by_, float bz_)
{
  float dot = ax*bx_ + ay*by_ + az*bz_;

  Quaternion q;
  q.w = 1.0f + dot;
  q.x = ay*bz_ - az*by_;  // a x b
  q.y = az*bx_ - ax*bz_;
  q.z = ax*by_ - ay*bx_;

  return quat_normalize(q);
}

// Nose attitude relative to vertical
// tilt_deg: total angle of the nose (body +X) away from straight up. No singularity.
// err_y, err_z: attitude error as rotation about body +Y and +Z -- the control error signals
void attitude_tilt(Quaternion q, float *tilt_deg, float *err_y, float *err_z)
{
  const float RAD2DEG = 180.0f / 3.14159265f;

  // Rotate world up into the body frame
  float ux, uy, uz;
  quat_rotate_vector(quat_conjugate(q), 0.0f, 0.0f, 1.0f, &ux, &uy, &uz);

  float c = ux;
  if (c >  1.0f) c =  1.0f;
  if (c < -1.0f) c = -1.0f;
  *tilt_deg = acosf(c) * RAD2DEG;

  // Attitude error as a rotation vector: (body +X) x (world up) = (0, -uz, uy)
  // Magnitude is sin(tilt), so ~= tilt in radians for small angles.
  *err_y = -uz;
  *err_z = uy;
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
  HAL_SPI_TransmitReceive(&hspi1, tx, rx, 2, SPI_TIMEOUT_MS); // exchange 2 bytes
  HAL_GPIO_WritePin(BMP_CS_GPIO_Port, BMP_CS_Pin, GPIO_PIN_SET); // CS High: End communication

  return rx[1]; // The sensors answers landed in the second byte   
}

void BMP280ReadRegisters(uint8_t reg, uint8_t *buffer, uint16_t len)
{
  uint8_t addr = reg | 0x80; // Address byte, bit 7 set = READ

  HAL_GPIO_WritePin(BMP_CS_GPIO_Port, BMP_CS_Pin, GPIO_PIN_RESET); // CS Low: Start communication
  HAL_SPI_Transmit(&hspi1, &addr, 1, SPI_TIMEOUT_MS); // exchange registers
  HAL_SPI_Receive(&hspi1, buffer, len, SPI_TIMEOUT_MS); // receive data
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
    HAL_SPI_Transmit(&hspi1, tx, 2, SPI_TIMEOUT_MS);                   // send both bytes
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

// --- Control Functions ---
// Convert a commanded gimbal deflection (degrees) to a servo pulse width (microseconds)
// sign: +1 or -1, set emprically so that a positive command produces a 
// deflection that CORRECTS the attitude error. Not derivable on paper.
static uint16_t gimbal_to_pulse(float cmd_deg, float sign, uint16_t trim, float us_per_deg, uint16_t min_us, uint16_t max_us)
{
  // Guard 1: limit deflection to the verifiied linear working range
  if (cmd_deg > MAX_DEFLECT) cmd_deg = MAX_DEFLECT;
  if (cmd_deg < -MAX_DEFLECT) cmd_deg = -MAX_DEFLECT;

  float pulse = (float)trim + sign * cmd_deg * us_per_deg;

  // Guard 2: hard bound inside the mechanical stops, independent of guard 1
  if (pulse < (float)min_us) pulse = (float)min_us;
  if (pulse > (float)max_us) pulse = (float)max_us;

  return (uint16_t)(pulse + 0.5f); // round rather than truncate
}

// Flash Chip Functions
// Read the 3-byte JEDEC ID: manufacturer, memory type, capacity
// W25128JV should return EF 40 18
void Flash_ReadJEDEC(uint8_t *buf)
{
  uint8_t cmd = 0x9F; // JEDEC ID command

  HAL_GPIO_WritePin(FLASH_CS_GPIO_Port, FLASH_CS_Pin, GPIO_PIN_RESET); // CS low
  HAL_SPI_Transmit(&hspi1, &cmd, 1, SPI_TIMEOUT_MS); // send command
  HAL_SPI_Receive(&hspi1, buf, 3, SPI_TIMEOUT_MS); // read 3 bytes
  HAL_GPIO_WritePin(FLASH_CS_GPIO_Port, FLASH_CS_Pin, GPIO_PIN_SET); // CS high
}

// Synthetic vertical acceleration
// Returns net acceleration (gravity already removed), matching lin_accel_z
#if SIM_MODE
float sim_lin_accel_z(float t)
{
  if (t < 2.0f) {
    return 0.0f;
  } else if (t < 15.0f) {
    return 34.0f;
  } else {
    return -10.0f;
  }
}
#endif

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  // Safe the gimbal before halting. PWM is generated by the timer hardware, so the
  // servos would otherwise hold their last commanded deflection forever
  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, SERVO_MY_TRIM);   // y axis servo
  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, SERVO_MX_TRIM);   // x axis servo

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
