/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
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
#include "fatfs.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>
#include "retarget.h"
#include "sensor_service.h"
#include "data_service.h"
#include "bluetooth_service.h"
#include "fatfs_sd.h"
#include "batt_monitor.h"
#include "sd_logger.h"
#include "bno055_stm32.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#define FORCE_BUF_CAPACITY 500
#define IMPACT_ANGLE_BUF_CAPACITY 50
#define IMPACT_FORCE_BUF_CAPACITY 50

#define PEAK_TIME_THRESH_MILLIS 100
#define PEAK_FORCE_BUF_CAPACITY 100

#define MAX_INT_32 0xffffffff

#define LOG_SYNC_INTERVAL 5000

#define SEND_STEP_DATA_INTERVAL 500
// low power interval
#define SEND_STEP_DATA_LP_INTERVAL 1000
#define SEND_BATT_DATA_INTERVAL 60000
#define LOW_POWER_THRESHOLD 20
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define max(a,b) \
   ({ __typeof__ (a) _a = (a); \
       __typeof__ (b) _b = (b); \
     _a > _b ? _a : _b; })

#define min(a,b) \
   ({ __typeof__ (a) _a = (a); \
       __typeof__ (b) _b = (b); \
     _a < _b ? _a : _b; })
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
 I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c3;

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

float user_mass;
float new_user_mass;
float batt_percent;

force_point force_buf[FORCE_BUF_CAPACITY];
float impulse_buffer;
float impact_angle_buffer[IMPACT_ANGLE_BUF_CAPACITY];
uint32_t impact_angle_buffer_size;
float impact_force_buffer[IMPACT_FORCE_BUF_CAPACITY];
uint32_t impact_force_buffer_size;

uint8_t test_buf[100];

uint8_t is_batt_connected = 1;
uint8_t is_logging = 1;
FIL log_file;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_I2C1_Init(void);
static void MX_SPI1_Init(void);
static void MX_I2C3_Init(void);
/* USER CODE BEGIN PFP */

void print_hex(char* data, uint32_t len);
uint32_t diff_time(uint32_t start, uint32_t end);
void bt_roundtrip_test(uint32_t count);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void print_hex(char* data, uint32_t len) {
	for (uint32_t i = 0; i < len; i++) {
		printf("%02x ", data[i]);
	}
	printf("\r\n");
}

uint32_t diff_time(uint32_t start, uint32_t end) {
	if (end < start) {
		return MAX_INT_32 - end + start;
	} else {
		return end - start;
	}
}

void bt_roundtrip_test(uint32_t count) {
	float sample_data[1] = {0};
	uint8_t size = 1;
//	for (uint32_t i = 0; i < size; i++) {
//		sample_data[i] = i;
//	}

//	__HAL_TIM_SET_COUNTER(&htim2, 0);
	bt_send(&huart2, IMPULSE_CMD, sample_data, size * sizeof(uint32_t));

//	char recv_data[256];
//	if (bt_recv(&huart2, &header, &recv_data) == 0) {
////		uint32_t elapsed_time = __HAL_TIM_GetCounter(&htim2);
//
//		printf("%x\r\n", header.len);
//		if (header.cmd == RESPONSE_CMD) {
////		  printf("%ld, %ld\r\n", count, elapsed_time);
//		  print_hex(recv_data, header.len);
//		} else {
//		  printf("Unknown response\r\n");
//		}
//		HAL_Delay(1000);
//	} else {
//		printf("Timed out\r\n");
//	}
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if(GPIO_Pin == PWR_BTN_Pin)
    {
//    	HAL_GPIO_TogglePin(LD3_GPIO_Port, LD3_Pin);
    }
}

//void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
//printf("test\r\n");
//}

// void writeRegister(uint8_t address, uint8_t * value, uint8_t num)
// {
// 	HAL_I2C_Mem_Write(&hi2c1, 0x0b << 1, address, 1, value, num, 100);
// }
//
// void readRegister(uint8_t address,uint8_t * value, uint8_t num)
// {
// 	HAL_I2C_Mem_Read (&hi2c1, 0x0b << 1, address, 1, value, num, 100);
// }

//
//uint32_t get_timer_elapsed_time(TIM_HandleTypeDef* timer, uint32_t last_count) {
//	return get_timer_elapsed_count(_HAL_TIM_GetCounter(timer), last_count, timer->Init.Period);
//}


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
  MX_USART1_UART_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_FATFS_Init();
  MX_I2C1_Init();
  MX_SPI1_Init();
  MX_I2C3_Init();
  /* USER CODE BEGIN 2 */

  RetargetInit(&huart1);

  // if (accel_init() != SENSOR_OK) {
	//   printf("Accelerometer initialization failed\r\n");
	//   return 0;
  // }
  if (imu_init(&hi2c1) != SENSOR_OK) {
    printf("[Fatal] IMU initialization failed\r\n");
    return 0;
  }
  if (batt_init() != BATT_OK) {
	  printf("[Warning] Battery monitor initialization failed\r\n");
	  is_batt_connected = 0;
  }
  if (sd_logger_init(&log_file) != SD_OK) {
	  printf("[Warning] SD card logger initialization failed\r\n");
	  is_logging = 0;
  }

  HAL_TIM_Base_Start(&htim1);
  HAL_TIM_Base_Start(&htim2);

  uint32_t data_buf_idx = 0;
  accel_vec accel_data;
  uint8_t is_first_loop = 1;
  uint8_t is_force_spike = 0;
  angle_vec angle;

  peak_force_point peak_force_buf[PEAK_FORCE_BUF_CAPACITY];
  uint32_t peak_force_buf_size = 0;
  peak_force_point new_peak_force;
  uint32_t last_peak_force_time = 0;

  user_mass = 65;

  batt_percent = is_batt_connected ? batt_get_percent() : 100;

  impulse_buffer = 0;
  impact_angle_buffer_size = 0;
  impact_force_buffer_size = 0;

  float force_baseline = user_mass * GRAV_ACCEL;
  float high_force_threshold = force_baseline + 400;
  float low_force_threshold = force_baseline - 200;
  float step_threshold = high_force_threshold * 2;
  float low_force_lookback_size = 75;

  uint32_t last_log_sync_time = HAL_GetTick();
  uint32_t last_send_step_time = HAL_GetTick();
  uint32_t last_send_batt_time = HAL_GetTick();

//  uint8_t count = 0xff;
//  char recv;
//  while (1) {
//	  bt_roundtrip_test(count);
//	  printf("%ld%%\r\n", (uint32_t)batt_get_percent());
//	  uint8_t data[5] = {1, 2, 0, 3, 4};
//	  HAL_UART_Transmit(&huart2, data, 5, BT_SEND_TIMEOUT);
//	  if (HAL_UART_Receive(&huart2, &recv, 1, BT_RECV_TIMEOUT) == 0) {
//		  printf("%c", recv);
//	  } else {
//		  printf("\r\nTimeout\r\n");
//	  }
//	  count++;
//	  HAL_GPIO_TogglePin(PWR_LED_GPIO_Port, PWR_LED_Pin);
//	  HAL_Delay(5000);
//  }

//  HAL_UART_Receive_IT(&huart2, test_buf, 1);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  while (1)
  {

	  force_point* force_pt = &force_buf[data_buf_idx];
	  force_point* prev_force_pt = &force_buf[(data_buf_idx - 1) % FORCE_BUF_CAPACITY];

    imu_sample(&accel_data, &force_pt->angle);
	  force_pt->force = compute_force(&accel_data);
	  force_pt->elapsed_time = __HAL_TIM_GetCounter(&htim1);
	  __HAL_TIM_SET_COUNTER(&htim1, 0);
	  
	  if (is_logging) {
		  f_printf(&log_file, "%ld, %ld, %ld, %ld\n", force_pt->elapsed_time, (int32_t)(accel_data.x * 1000), (int32_t)(accel_data.y * 1000), (int32_t)(accel_data.z * 1000));
	  }
	  // printf("%2.f, %2.f, %2.f\r\n", accel_data.x, accel_data.y, accel_data.z);
    // HAL_Delay(100);
//	  printf("%ld, %ld\r\n", (uint32_t)force_pt->elapsed_time, (uint32_t)force_pt->force);

	  new_peak_force.index = -1; // -1 means no new peak force
	  if (!is_first_loop) {
		  uint8_t force_is_increasing = force_pt->force > prev_force_pt->force;
		  if (!is_force_spike) {
			  // decreasing -> decreasing
			  if (force_pt->force > high_force_threshold && force_is_increasing) {
				  // decreasing -> increasing
				  is_force_spike = 1;
			  }
		  } else if (!force_is_increasing) {
			  // increasing -> decreasing
			  new_peak_force.force = prev_force_pt->force;
			  new_peak_force.index = (data_buf_idx - 1) % FORCE_BUF_CAPACITY;
			  is_force_spike = 0;
		  }
		  // increasing -> increasing: do nothing
	  }

	  // if force has peaked, add it to list of force peaks
	  if (new_peak_force.index != -1) {
		  last_peak_force_time = HAL_GetTick();
		  peak_force_buf[peak_force_buf_size] = new_peak_force;
		  peak_force_buf_size++;
//		   printf("Peak %ld %ld\r\n", (uint32_t)new_peak_force.force, last_peak_force_time);
	  }

	  // batch forces together, each batch of forces are caused by the same step
	  if (diff_time(last_peak_force_time, HAL_GetTick()) > PEAK_TIME_THRESH_MILLIS) {
		  uint8_t is_step = 0;
		  uint32_t last_low_force_idx = 0;
		  peak_force_point* max_peak_force = 0;

		  if (peak_force_buf_size > 0) {
//		      uint8_t is_max_peak_force_last = (peak_force_buf_size == 1);
			  max_peak_force = &peak_force_buf[0];

			  for (uint8_t i = 1; i < peak_force_buf_size; i++) {
				  if (peak_force_buf[i].force > max_peak_force->force) {
					  max_peak_force = &peak_force_buf[i];
//					  is_max_peak_force_last = (i == peak_force_buf_size - 1);
				  }
			  }

			  // expect there to be vibrations after step (maximum peak should not be last in batch)
			  // and maximum force should exceed threshold
//			  if (!is_max_peak_force_last) {
//				  NOT EFFECTIVE
//			  }

			  if (max_peak_force->force > step_threshold) {
				  // expect a dip in force before the foot lands, indicating a step
				  float min_force = 10000000;
				  for (uint8_t i = 0; i < low_force_lookback_size; i++) {
					  uint32_t buf_idx = (max_peak_force->index - i) % FORCE_BUF_CAPACITY;
					  min_force = min(min_force, force_buf[buf_idx].force);
					  if (force_buf[buf_idx].force < low_force_threshold) {
						  is_step = 1;
						  last_low_force_idx = buf_idx;
					  }
				  }
				  if (!is_step) {
					  printf("No dip %f > %f\r\n", min_force, low_force_threshold);
				  }
			  }
			  peak_force_buf_size = 0;
		  }

		  if (is_step) {
			  int32_t peak_start_index;
			  int32_t peak_end_index;

			  // find where max peak started
			  peak_start_index = max_peak_force->index;
			  while (force_buf[peak_start_index].force > high_force_threshold) {
				  peak_start_index = (peak_start_index - 1) % FORCE_BUF_CAPACITY;
			  }

			  // find where max peak ends
			  peak_end_index = max_peak_force->index;
			  while (force_buf[peak_end_index].force > high_force_threshold) {
				  peak_end_index = (peak_end_index + 1) % FORCE_BUF_CAPACITY;
			  }

			  // account for possible wrap-around in force buffer
			  if (peak_end_index < peak_start_index) {
				  peak_end_index = FORCE_BUF_CAPACITY + peak_end_index;
			  }

			  // riemann sum over max force peak
			  for (uint32_t i = peak_start_index + 1; i < peak_end_index; i++) {
				  impulse_buffer += force_buf[i % FORCE_BUF_CAPACITY].force * force_buf[i % FORCE_BUF_CAPACITY].elapsed_time / 1000000.0f;
			  }

			  angle_vec* peak_angle = &force_buf[last_low_force_idx].angle;
			  // get maximum of the 3 angles
			  impact_angle_buffer[impact_angle_buffer_size] = max(peak_angle->roll, max(peak_angle->pitch, peak_angle->yaw));
			  impact_force_buffer[impact_force_buffer_size] = max_peak_force->force;

			  impact_angle_buffer_size = min(impact_angle_buffer_size + 1, IMPACT_ANGLE_BUF_CAPACITY);
			  impact_force_buffer_size = min(impact_force_buffer_size + 1, IMPACT_FORCE_BUF_CAPACITY);
			  printf("Step - Impulse: %f, Max Force: %f, Angle: %f %f %f\r\n", impulse_buffer, max_peak_force->force, peak_angle->yaw, peak_angle->roll, peak_angle->pitch);
		  } else {
//			  printf("No step\r\n");
		  }
		  last_peak_force_time = HAL_GetTick();
	  }

	  if (is_logging && HAL_GetTick() - last_log_sync_time > LOG_SYNC_INTERVAL) {
		  printf("Log sync\r\n");
	  	  f_sync(&log_file);
	  	  last_log_sync_time = HAL_GetTick();
	  }

	  // send step data to mobile device via bluetooth (bt)
	  uint8_t is_low_power = (batt_percent < LOW_POWER_THRESHOLD);
	  if (HAL_GetTick() - last_send_step_time > (is_low_power ? SEND_STEP_DATA_LP_INTERVAL : SEND_STEP_DATA_INTERVAL)) {
		  bt_send_str_float(&huart2, IMPULSE_CMD, impulse_buffer);
		  bt_send_str_float_array(&huart2, ANGLE_CMD, impact_angle_buffer, impact_angle_buffer_size);
		  bt_send_str_float_array(&huart2, MAX_FORCE_CMD, impact_force_buffer, impact_force_buffer_size);
		  impulse_buffer = 0;
		  impact_angle_buffer_size = 0;
		  impact_force_buffer_size = 0;
		  last_send_step_time = HAL_GetTick();
	  }

	  if (HAL_GetTick() - last_send_batt_time > SEND_BATT_DATA_INTERVAL) {
		  batt_percent = is_batt_connected ? batt_get_percent() : 100;
		  bt_send_str_float(&huart2, BATT_CMD, batt_percent);
		  last_send_batt_time = HAL_GetTick();
	  }

	  uint8_t recv_result = bt_try_recv_user_mass(&huart2, &new_user_mass);
	  if (recv_result == BT_OK) {
		  printf("New user mass: %f\r\n", new_user_mass);
		  user_mass = new_user_mass;
	  }

	  data_buf_idx = (data_buf_idx + 1) % FORCE_BUF_CAPACITY;
	  is_first_loop = 0;
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
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure LSE Drive Capability
  */
  HAL_PWR_EnableBkUpAccess();
  __HAL_RCC_LSEDRIVE_CONFIG(RCC_LSEDRIVE_LOW);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSE|RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 16;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
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

  /** Enable MSI Auto calibration
  */
  HAL_RCCEx_EnableMSIPLLMode();
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
  hi2c1.Init.Timing = 0x00707CBB;
  hi2c1.Init.OwnAddress1 = 22;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief I2C3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C3_Init(void)
{

  /* USER CODE BEGIN I2C3_Init 0 */

  /* USER CODE END I2C3_Init 0 */

  /* USER CODE BEGIN I2C3_Init 1 */

  /* USER CODE END I2C3_Init 1 */
  hi2c3.Instance = I2C3;
  hi2c3.Init.Timing = 0x00707CBB;
  hi2c3.Init.OwnAddress1 = 0;
  hi2c3.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c3.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c3.Init.OwnAddress2 = 0;
  hi2c3.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c3.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c3.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c3) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c3, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c3, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C3_Init 2 */

  /* USER CODE END I2C3_Init 2 */

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
  hspi1.Init.DataSize = SPI_DATASIZE_4BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 7;
  hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 32;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 65535;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

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
  htim2.Init.Prescaler = 32;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 4294967295;
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
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

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
  huart2.Init.BaudRate = 9600;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
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
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(SD_CS_GPIO_Port, SD_CS_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(PWR_LED_GPIO_Port, PWR_LED_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin : PWR_BTN_Pin */
  GPIO_InitStruct.Pin = PWR_BTN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(PWR_BTN_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : SD_CS_Pin */
  GPIO_InitStruct.Pin = SD_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(SD_CS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PWR_LED_Pin */
  GPIO_InitStruct.Pin = PWR_LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(PWR_LED_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

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
