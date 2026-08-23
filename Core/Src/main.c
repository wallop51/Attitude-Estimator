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
#include <string.h>
#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_def.h"
#include "telemetry.h"
#include "mpu6050.h"
#include "stdio.h"
#include "quaternion.h"
#include "vector3.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define RAD_PER_DEG_MS (3.14159265f / 180000.0f)
#define RAW_TO_DPS 131
#define RAW_TO_G 16384
#define KP 0.02f
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

UART_HandleTypeDef huart2;

volatile uint32_t imu_interrupt_count = 0;

volatile uint8_t data_ready_flag = 0; // Flag to indicate data ready interrupt

/* USER CODE BEGIN PV */

// CALIBRATION PARAMETERS
const Vector3 GYRO_BIAS = {-417.08f, 161.904f, 19.94f};
const Vector3 ACCEL_OFFSET = {573.654f, -126.384f, 680.564f};
const Vector3 ACCEL_SCALE = {1.008102f, 0.993418f, 0.989793f};
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin);
HAL_StatusTypeDef IMU_Read_Sample(Vector3_i *accel, Vector3_i *gyro);
void Calibrate_Sample(Vector3_i *accel, Vector3_i* gyro, Vector3 *accel_calibrated, Vector3 *gryo_calibrated);
void Apply_Error_Correction(Vector3 *v, Vector3 error);
Quaternion Get_Delta_Quaternion(Vector3 gyros, uint32_t dt);
void Send_Data(Quaternion data);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
HAL_StatusTypeDef IMU_Read_Sample(Vector3_i *accel, Vector3_i *gyro) {
  HAL_StatusTypeDef status;

  status = MPU6050_Read_Accel(&accel->x, &accel->y, &accel->z);
  if (status != HAL_OK) return status;

  status = MPU6050_Read_Gyro(&gyro->x, &gyro->y, &gyro->z);
  if(status != HAL_OK) return status;

  return HAL_OK;
}

void Calibrate_Sample(Vector3_i *accel, Vector3_i* gyro, Vector3 *accel_calibrated, Vector3 *gyro_calibrated) {
  accel_calibrated->x = (accel->x - ACCEL_OFFSET.x) * ACCEL_SCALE.x / RAW_TO_G;
  accel_calibrated->y = (accel->y - ACCEL_OFFSET.y) * ACCEL_SCALE.y / RAW_TO_G;
  accel_calibrated->z = (accel->z - ACCEL_OFFSET.z) * ACCEL_SCALE.z / RAW_TO_G;

  gyro_calibrated->x = (gyro->x - GYRO_BIAS.x) / RAW_TO_DPS;
  gyro_calibrated->y = (gyro->y - GYRO_BIAS.y) / RAW_TO_DPS;
  gyro_calibrated->z = (gyro->z - GYRO_BIAS.z) / RAW_TO_DPS;
}

void Apply_Error_Correction(Vector3 *v, Vector3 error) {
  v->x += KP * error.x;
  v->y += KP * error.y;
  v->z += KP * error.z;
}

Quaternion Get_Delta_Quaternion(Vector3 gyros, uint32_t dt) {
  Vector3 delta = {
    gyros.x * dt * RAD_PER_DEG_MS,
    gyros.y * dt * RAD_PER_DEG_MS,
    gyros.z * dt * RAD_PER_DEG_MS
  };

  float angle = vector3_magnitude(delta);
  Vector3 axis = vector3_normalise(delta);

  return quaternion_from_axis_angle(axis, angle);
}

void Send_Data(Quaternion data) {
  char message[32];
  snprintf(message, sizeof(message), "%0.3f,%0.3f,%0.3f,%0.3f\r\n", data.w, data.x, data.y, data.z);
  Telemetry_Print(message);
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
  MX_I2C1_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */

  // Initialize the MPU6050
  if (MPU6050_Init() == HAL_OK) {
      Telemetry_Print("MPU6050 initialized successfully\r\n");
      HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_SET); // Turn on LED
  } else {
      Telemetry_Print("Failed to initialize MPU6050\r\n");
      HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET); // Turn off LED
  }

  // INITIALISE T0
  uint32_t t0 = HAL_GetTick();
  uint32_t t1;
  uint32_t dt;


  // SET CURRENT Q = 1,0,0,0
  Quaternion current_attitude = {
    1.0f,
    0.0f,
    0.0f,
    0.0f
  };
  // SET GRAVITY VECTOR = 1,0,0
  Vector3 gravity_vector = {
    1.0f,
    0.0f,
    0.0f
  };

  Vector3 predicted_gravity_vector;

  Vector3_i accels_raw;
  Vector3_i gyros_raw;

  Vector3 accels;
  Vector3 gyros;

  Vector3 error;

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    if (data_ready_flag) {
      data_ready_flag = 0;

      if(IMU_Read_Sample(&accels_raw, &gyros_raw) != HAL_OK) {
        continue;
      }

      t1 = HAL_GetTick();
      dt = t1 - t0;
      t0 = t1;

      Calibrate_Sample(&accels_raw, &gyros_raw, &accels, &gyros);

      gravity_vector = vector3_normalise(accels);
      predicted_gravity_vector = quaternion_apply_rotation(current_attitude, (Vector3){1.0f, 0.0f, 0.0f});

      error = vector3_cross(predicted_gravity_vector, gravity_vector);
      Apply_Error_Correction(&gyros, error);

      current_attitude = quaternion_multiply(current_attitude, Get_Delta_Quaternion(gyros, dt));
      Send_Data(current_attitude);
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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

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
  RCC_OscInitStruct.PLL.PLLQ = 7;
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
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PA1 */
  GPIO_InitStruct.Pin = GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI1_IRQn);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
    if (GPIO_Pin == GPIO_PIN_1) {
        // Handle the interrupt triggered by PA1
        data_ready_flag = 1; 
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
