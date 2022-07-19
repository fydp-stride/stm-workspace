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
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#define FORCE_BUF_CAPACITY 500
#define IMPACT_ANGLE_BUF_CAPACITY 50
#define IMPACT_FORCE_BUF_CAPACITY 50

#define GRAV_ACCEL 9.81
#define PEAK_TIME_THRESH_MILLIS 100
#define PEAK_FORCE_BUF_CAPACITY 100

#define MAX_INT_32 0xffffffff
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

SPI_HandleTypeDef hspi1;
SPI_HandleTypeDef hspi3;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

float user_mass;

force_point force_buf[FORCE_BUF_CAPACITY];
float impulse_buffer;
triple_axis_angle impact_angle_buffer[IMPACT_ANGLE_BUF_CAPACITY];
uint32_t impact_angle_buffer_size;
float impact_force_buffer[IMPACT_FORCE_BUF_CAPACITY];
uint32_t impact_force_buffer_size;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_SPI1_Init(void);
static void MX_SPI3_Init(void);
static void MX_I2C1_Init(void);
/* USER CODE BEGIN PFP */

void print_hex(uint8_t* data, uint32_t len);

void bt_roundtrip_test(uint32_t count);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void print_hex(uint8_t* data, uint32_t len) {
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
	float sample_data[1] = {640.54};
	uint8_t size = 1;
//	for (uint32_t i = 0; i < size; i++) {
//		sample_data[i] = i;
//	}

	__HAL_TIM_SET_COUNTER(&htim2, 0);

	bt_header header;
	header.cmd = IMPULSE_CMD;
	header.len = size * sizeof(uint32_t);
	bt_send(&huart2, &header, sample_data);

	char recv_data[256];
	if (bt_recv(&huart2, &header, &recv_data) == 0) {
		uint32_t elapsed_time = __HAL_TIM_GetCounter(&htim2);

		printf("%x\r\n", header.len);
		if (header.cmd == RESPONSE_CMD) {
//		  printf("%ld, %ld\r\n", count, elapsed_time);
		  print_hex(recv_data, header.len);
		} else {
		  printf("Unknown response\r\n");
		}
		HAL_Delay(1000);
	} else {
		printf("Timed out\r\n");
	}
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if(GPIO_Pin == PWR_BTN_Pin)
    {
//    	HAL_GPIO_TogglePin(LD3_GPIO_Port, LD3_Pin);
    }
}

void adxl_write (uint8_t address, uint8_t value)
{
	uint8_t data[2];
	data[0] = address|0x40;  // multibyte write
	data[1] = value;
	HAL_GPIO_WritePin (ACCEL_CS_GPIO_Port, ACCEL_CS_Pin, GPIO_PIN_RESET);  // pull the cs pin low
	HAL_SPI_Transmit (&hspi1, data, 2, 100);  // write data to register
	HAL_GPIO_WritePin (ACCEL_CS_GPIO_Port, ACCEL_CS_Pin, GPIO_PIN_SET);  // pull the cs pin high
}

void adxl_read (uint8_t address, uint8_t* value, uint8_t size)
{
	address |= 0x80;  // read operation
	address |= 0x40;
	HAL_GPIO_WritePin (ACCEL_CS_GPIO_Port, ACCEL_CS_Pin, GPIO_PIN_RESET);  // pull the pin low
	HAL_SPI_Transmit (&hspi1, &address, 1, 100);  // send address
	HAL_SPI_Receive (&hspi1, value, size, 100);  // receive 6 bytes data
	HAL_GPIO_WritePin (ACCEL_CS_GPIO_Port, ACCEL_CS_Pin, GPIO_PIN_SET);  // pull the pin high
}

 void writeRegister(uint8_t address, uint8_t * value, uint8_t num)
 {
 	HAL_I2C_Mem_Write(&hi2c1, 0x0b << 1, address, 1, value, num, 100);
 }

 void readRegister(uint8_t address,uint8_t * value, uint8_t num)
 {
 	HAL_I2C_Mem_Read (&hi2c1, 0x0b << 1, address, 1, value, num, 100);
 }

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
  MX_SPI1_Init();
  MX_FATFS_Init();
  MX_SPI3_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */

  RetargetInit(&huart1);

  if (accel_init() != SENSOR_OK) {
	  printf("Accelerometer initialization failed\r\n");
	  return 0;
  }
  if (batt_init() != BATT_OK) {
	  printf("Battery monitor initialization failed\r\n");
	  return 0;
  }

  HAL_TIM_Base_Start(&htim1);
  HAL_TIM_Base_Start(&htim2);

  uint32_t data_buf_idx = 0;
  triple_axis_accel accel_data;
  uint8_t is_first_loop = 1;
  uint8_t is_force_spike = 0;
  triple_axis_angle angle;
  triple_axis_angle prev_angle;

  peak_force_point peak_force_buf[PEAK_FORCE_BUF_CAPACITY];
  uint32_t peak_force_buf_size = 0;
  peak_force_point new_peak_force;
  uint32_t last_peak_force_time = 0;

  user_mass = 65;
  impulse_buffer = 0;
  impact_angle_buffer_size = 0;
  impact_force_buffer_size = 0;

  float force_threshold = user_mass * GRAV_ACCEL + 400;

//  uint16_t reg = 1;
//  writeRegister(0x15, &reg, 2);
//  reg = 16;
//  writeRegister(0x0b, &reg, 2);
//  reg = 1;
//  writeRegister(0x12, &reg, 2);
//  reg = 1;
//  writeRegister(0x16, &reg, 2);
//
//  while (1) {
//		readRegister(0x11, &reg, 2);
//		printf("%d\r\n", reg);
//		HAL_Delay(10);
//  }

//  FATFS fs;
//  FATFS *pfs;
//  FIL fil;
//  FRESULT fres;
//  DWORD fre_clust;
//  uint32_t totalSpace, freeSpace;
//  char buffer[100];

//  if(f_mount(&fs, "", 0) != FR_OK) {
//	  printf("fail1\r\n");
//  }
//
//  if(f_open(&fil, "first.txt", FA_OPEN_ALWAYS | FA_READ | FA_WRITE) != FR_OK) {
//	  printf("fail2\r\n");
//  }
//
//  f_puts("STM32 SD Card I/O Example via SPI\n", &fil);
//
//  if(f_close(&fil) != FR_OK) {
//	  printf("fail3\r\n");
//  }

//  uint8_t count = 0xff;
//  char recv;
//  whi          le (1) {

//	  bt_roundtrip_test(count);
	  printf("%ld%%\r\n", (uint32_t)batt_get_percent());
//	  char data[1] = {0x01};
//	  HAL_UART_Transmit(&huart2, &count, 1, BT_SEND_TIMEOUT);
//	  if (HAL_UART_Receive(&huart2, &recv, 1, BT_RECV_TIMEOUT) == 0) {
//		  printf("%c", recv);
//	  } else {
//		  printf("\r\nTimeout\r\n");
//	  }
//	  count++;
//	  HAL_GPIO_TogglePin(PWR_LED_GPIO_Port, PWR_LED_Pin);
//	  HAL_Delay(1000);
//  }

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  while (1)
  {

	  force_point* force_pt = &force_buf[data_buf_idx];
	  force_point* prev_force_pt = &force_buf[(data_buf_idx - 1) % FORCE_BUF_CAPACITY];

	  prev_angle = angle;

	  accel_sample(&accel_data, &angle);
	  force_pt->force = compute_force(&accel_data);
	  force_pt->elapsed_time = __HAL_TIM_GetCounter(&htim1);
	  __HAL_TIM_SET_COUNTER(&htim1, 0);

//	  printf("%ld, %ld, %ld, %ld\r\n", force_pt->elapsed_time, (int32_t)(accel_data.x * 1000), (int32_t)(accel_data.y * 1000), (int32_t)(accel_data.z * 1000));
//	  printf("%ld, %ld\r\n", (uint32_t)force_pt->elapsed_time, (uint32_t)force_pt->force);

	  new_peak_force.index = -1;
	  if (!is_first_loop) {
		  uint8_t force_is_increasing = force_pt->force > prev_force_pt->force;
		  if (!is_force_spike) {
			  if (force_pt->force > force_threshold && force_is_increasing) {
				  is_force_spike = 1;
			  }
		  } else if (!force_is_increasing) {
			  new_peak_force.force = prev_force_pt->force;
			  new_peak_force.angle = prev_angle;
			  new_peak_force.index = (data_buf_idx - 1) % FORCE_BUF_CAPACITY;
			  is_force_spike = 0;
		  }
	  }

	  if (new_peak_force.index != -1) {
		  last_peak_force_time = HAL_GetTick();
		  peak_force_buf[peak_force_buf_size] = new_peak_force;
		  peak_force_buf_size++;
//		   printf("Peak %ld %ld\r\n", (uint32_t)new_peak_force.force, last_peak_force_time);
	  }

	  if (diff_time(last_peak_force_time, HAL_GetTick()) > PEAK_TIME_THRESH_MILLIS) {
		  if (peak_force_buf_size > 0) {
			  uint8_t is_max_peak_force_last = (peak_force_buf_size == 1);
			  peak_force_point* max_peak_force = &peak_force_buf[0];
			  for (uint8_t i = 1; i < peak_force_buf_size; i++) {
				  if (peak_force_buf[i].force > max_peak_force->force) {
					  max_peak_force = &peak_force_buf[i];
					  is_max_peak_force_last = (i == peak_force_buf_size - 1);
				  }
			  }

			  if (!is_max_peak_force_last && max_peak_force->force > force_threshold * 2) {
				  int32_t peak_start_index;
				  int32_t peak_end_index;
				  int32_t prev_idx;
				  int32_t cur_idx;

				  prev_idx = max_peak_force->index;
				  cur_idx = (prev_idx - 1) % FORCE_BUF_CAPACITY;
				  while (force_buf[cur_idx].force < force_buf[prev_idx].force) {
					  prev_idx = cur_idx;
					  cur_idx = (prev_idx - 1) % FORCE_BUF_CAPACITY;
				  }
				  peak_start_index = prev_idx;

				  prev_idx = max_peak_force->index;
				  cur_idx = (prev_idx + 1) % FORCE_BUF_CAPACITY;
				  while (force_buf[cur_idx].force < force_buf[prev_idx].force) {
					  prev_idx = cur_idx;
					  cur_idx = (prev_idx + 1) % FORCE_BUF_CAPACITY;
				  }
				  peak_end_index = prev_idx;

				  if (peak_end_index < peak_start_index) {
					  peak_end_index = FORCE_BUF_CAPACITY + peak_end_index;
				  }

				  for (uint32_t i = peak_start_index + 1; i < peak_end_index; i++) {
					  impulse_buffer += force_buf[i % FORCE_BUF_CAPACITY].force * force_buf[i % FORCE_BUF_CAPACITY].elapsed_time / 1000000.0f;
				  }

				  impact_angle_buffer[impact_angle_buffer_size] = max_peak_force->angle;
				  impact_force_buffer[impact_force_buffer_size] = max_peak_force->force;

				  impact_angle_buffer_size = min(impact_angle_buffer_size + 1, IMPACT_ANGLE_BUF_CAPACITY);
				  impact_force_buffer_size = min(impact_force_buffer_size + 1, IMPACT_FORCE_BUF_CAPACITY);
				  printf("Step - Impulse: %ld, Max Force: %ld, Angle: %ld %ld %ld\r\n", (uint32_t)impulse_buffer, (uint32_t)max_peak_force->force, (uint32_t)max_peak_force->angle.yaw, (uint32_t)max_peak_force->angle.roll, (uint32_t)max_peak_force->angle.pitch);

				  bt_send_impulse(&huart2, impulse_buffer);
				  impulse_buffer = 0;
			  } else {
//				  printf("No step\r\n");
			  }
			  peak_force_buf_size = 0;
		  }
		  last_peak_force_time = HAL_GetTick();
	  }

//	  HAL_Delay(1000);

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
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_256;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 7;
  hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief SPI3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI3_Init(void)
{

  /* USER CODE BEGIN SPI3_Init 0 */

  /* USER CODE END SPI3_Init 0 */

  /* USER CODE BEGIN SPI3_Init 1 */

  /* USER CODE END SPI3_Init 1 */
  /* SPI3 parameter configuration*/
  hspi3.Instance = SPI3;
  hspi3.Init.Mode = SPI_MODE_MASTER;
  hspi3.Init.Direction = SPI_DIRECTION_2LINES;
  hspi3.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi3.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi3.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi3.Init.NSS = SPI_NSS_SOFT;
  hspi3.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_4;
  hspi3.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi3.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi3.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi3.Init.CRCPolynomial = 7;
  hspi3.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi3.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  if (HAL_SPI_Init(&hspi3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI3_Init 2 */

  /* USER CODE END SPI3_Init 2 */

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
  HAL_GPIO_WritePin(GPIOA, SD_CS_Pin|ACCEL_CS_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(PWR_LED_GPIO_Port, PWR_LED_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin : PWR_BTN_Pin */
  GPIO_InitStruct.Pin = PWR_BTN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(PWR_BTN_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : SD_CS_Pin ACCEL_CS_Pin */
  GPIO_InitStruct.Pin = SD_CS_Pin|ACCEL_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

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
