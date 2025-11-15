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
#include <stdio.h>
#include <stdlib.h>// only for absolute value in calc depth function could remove if we get low on space
#include <math.h>
#include "bmi323.h"
#include "common.h"
#include "bmi3.h"
#include <string.h>

#include"ST7565.h"
#include"RDB_depth_digits_36x58.h"
#include"RDB_Functions.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */




/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart4;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void PeriphCommonClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM1_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_UART4_Init(void);
static void MX_SPI1_Init(void);
static void MX_NVIC_Init(void);
/* USER CODE BEGIN PFP */
static int8_t set_accel_gyro_config(struct bmi3_dev *dev);
static float  lsb_to_g  (int16_t val, float g_range, uint8_t bit_width);
static float  lsb_to_dps(int16_t val, float dps,     uint8_t bit_width);
void check_buffer();
void clear_buff();
static void on_A_rise();
static void on_A_fall();
static void on_B_rise();
static void on_B_fall();
void calculate_depth();
void check_angle();

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
//encoder stuff
uint32_t position=0;
float angle=0.0;

uint32_t hall_data =  0;

//nmea stuff
uint8_t rx_buff[1];
uint8_t nmea[87];
uint8_t i=0;


//hall effect stuff
volatile uint8_t HallA = 0;
volatile uint8_t HallB = 0;

volatile uint32_t magnets_total = 0;
volatile uint32_t magnets_cw    = 0;
volatile uint32_t magnets_ccw   = 0;
volatile int32_t  magnets_net   = 0;// cw - ccw

float magnets_in_array=4.0;
float circumference_inches = 7.22; // measured 2.3 inches diameter by calipers

float line_out=0.0;
float depth_ft=0.0;
uint8_t Mosfet_state=0;



typedef enum {
  IDLE = 0,
  WAIT_B_AFTER_A=1,
  WAIT_A_AFTER_B=2
} state;

static volatile state  Magnet_st= IDLE;


/* Sensor initialization configuration. */
 struct bmi3_dev dev = { 0 };

 /* Status of API are returned to this variable. */
 int8_t rslt;

 /* Variable to define limit to print accel data. */
 uint16_t limit = 100;

 /* Create an instance of sensor data structure. */
 struct bmi3_sensor_data sensor_data[3] = { 0 };

 /* Initialize the interrupt status of accel. */
 uint16_t int_status = 0;

 /* Variable to store temperature */
 float temperature_value;

 uint8_t indx = 0;
 float acc_x = 0, acc_y = 0, acc_z = 0;
 float gyr_x = 0, gyr_y = 0, gyr_z = 0;



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

  /* Configure the peripherals common clocks */
  PeriphCommonClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_TIM1_Init();
  MX_I2C1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_UART4_Init();
  MX_SPI1_Init();

  /* Initialize interrupts */
  MX_NVIC_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_Encoder_Start(&htim1, TIM_CHANNEL_ALL);
  //HAL_ADC_Start(&hadc1);
  //HAL_ADC_Start_DMA(&hadc1, &hall_data, 1);


  HAL_UART_Receive_IT(&huart4, rx_buff, 1);
  //copied code from library Bosch bmi323


   /* Function to select interface between SPI and I2C, according to that the device structure gets updated.
    * Interface reference is given as a parameter
    * For I2C : BMI3_I2C_INTF
    * For SPI : BMI3_SPI_INTF
    */
  /* Select accel and gyro sensor. */
   sensor_data[0].type = BMI323_ACCEL;
   sensor_data[1].type = BMI323_GYRO;
   sensor_data[2].type = BMI323_TEMP;
   rslt = bmi3_interface_init(&dev, BMI3_I2C_INTF);
   bmi3_error_codes_print_result("bmi3_interface_init", rslt);

   /* Initialize bmi323. */
   rslt = bmi323_init(&dev);
   bmi3_error_codes_print_result("bmi323_init", rslt);
   ST7565_init();

//end of copied code the rest is in while loop sd



  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  /* Infinite loop
	  position= TIM1->CNT;

	  if (!HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13)){
		  __HAL_TIM_SET_COUNTER(&htim1,0);
	  }


	  angle = (position/2048.0)*360;
	  //angle = (int)((position/2048.0)*360);

	  if (angle>180){
		angle = 360-angle;

	  }

      //copied  direct read (no interrupt status check)
	    if (rslt == BMI323_OK)
	    {
	        // Accel and Gyro configuration settings.
	        rslt = set_accel_gyro_config(&dev);

	        if (rslt == BMI323_OK)
	        {
	            //printf(
	            //    "\nData set, Acc_Raw_X, Acc_Raw_Y, Acc_Raw_Z, Acc_G_X, Acc_G_Y, Acc_G_Z,  Gyr_Raw_X, Gyr_Raw_Y, Gyr_Raw_Z, Gyr_dps_X, Gyr_dps_Y, Gyr_dps_Z, Temperature data (Degree celsius), SensorTime(secs)\n\n");


	                // To get the status of accel data ready interrupt.
	                rslt = bmi323_get_int1_status(&int_status, &dev);
	                bmi3_error_codes_print_result("bmi323_get_int1_status", rslt);

	            // To check the accel data ready interrupt status and print the status for 100 samples.
	                if ((int_status & BMI3_INT_STATUS_ACC_DRDY) && (int_status & BMI3_INT_STATUS_GYR_DRDY) &&
	                    (int_status & BMI3_INT_STATUS_TEMP_DRDY))
	                {
	                    //Get accelerometer data for x, y and z axis.
	                    rslt = bmi323_get_sensor_data(sensor_data, 3, &dev);
	                    bmi3_error_codes_print_result("bmi323_get_sensor_data", rslt);

	                     //Converting lsb to gravity for 16 bit accelerometer at 2G range.
	                    gyr_x = lsb_to_g(sensor_data[0].sens_data.acc.x, 2.0f, dev.resolution);
	                    gyr_y = lsb_to_g(sensor_data[0].sens_data.acc.y, 2.0f, dev.resolution);
	                    gyr_z = lsb_to_g(sensor_data[0].sens_data.acc.z, 2.0f, dev.resolution);

	                   // Converting lsb to degree per second for 16 bit gyro at 2000dps range.
	                    acc_x = lsb_to_dps(sensor_data[1].sens_data.gyr.x, (float)2000, dev.resolution);
	                    acc_y = lsb_to_dps(sensor_data[1].sens_data.gyr.y, (float)2000, dev.resolution);
	                    acc_z = lsb_to_dps(sensor_data[1].sens_data.gyr.z, (float)2000, dev.resolution);

	                    temperature_value =
	                        (float)((((float)((int16_t)sensor_data[2].sens_data.temp.temp_data)) / 512.0) + 23.0);

	                    // Print the accel data in gravity and gyro data in dps.
	                    printf("%d, %d, %d, %d, %4.2f, %4.2f, %4.2f, %d, %d, %d, %4.2f, %4.2f, %4.2f, %f, %.4lf\n",
	                           indx,
	                           sensor_data[0].sens_data.acc.x,
	                           sensor_data[0].sens_data.acc.y,
	                           sensor_data[0].sens_data.acc.z,
	                           acc_x,
	                           acc_y,
	                           acc_z,
	                           sensor_data[1].sens_data.gyr.x,
	                           sensor_data[1].sens_data.gyr.y,
	                           sensor_data[1].sens_data.gyr.z,
	                           gyr_x,
	                           gyr_y,
	                           gyr_z,
	                           temperature_value,
	                           (sensor_data[1].sens_data.temp.sens_time * BMI3_SENSORTIME_RESOLUTION));



	            }
	        }
	    }


  	  	 */



	  check_buffer();
	  check_angle();
	  calculate_depth(); //calculates depth
	  parseDepthVal((int)depth_ft);
	  if(((int)depth_ft) > 10){
		  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, 1);
		  Mosfet_state=1;
	  }else{
		  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, 0);
		  Mosfet_state=0;
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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 180;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Activate the Over-Drive mode
  */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief Peripherals Common Clock Configuration
  * @retval None
  */
void PeriphCommonClock_Config(void)
{
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  /** Initializes the peripherals clock
  */
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_TIM;
  PeriphClkInitStruct.TIMPresSelection = RCC_TIMPRES_ACTIVATED;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief NVIC Configuration.
  * @retval None
  */
static void MX_NVIC_Init(void)
{
  /* TIM2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(TIM2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(TIM2_IRQn);
  /* TIM3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(TIM3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(TIM3_IRQn);
  /* UART4_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(UART4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(UART4_IRQn);
  /* EXTI9_5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);
  /* EXTI15_10_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);
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
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
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
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 2048;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI1;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
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
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 37500;
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

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 10-1;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 10000-1;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
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
  * @brief UART4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART4_Init(void)
{

  /* USER CODE BEGIN UART4_Init 0 */

  /* USER CODE END UART4_Init 0 */

  /* USER CODE BEGIN UART4_Init 1 */

  /* USER CODE END UART4_Init 1 */
  huart4.Instance = UART4;
  huart4.Init.BaudRate = 4800;
  huart4.Init.WordLength = UART_WORDLENGTH_8B;
  huart4.Init.StopBits = UART_STOPBITS_1;
  huart4.Init.Parity = UART_PARITY_NONE;
  huart4.Init.Mode = UART_MODE_RX;
  huart4.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart4.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART4_Init 2 */

  /* USER CODE END UART4_Init 2 */

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
  HAL_GPIO_WritePin(GPIOA, CS_Pin|RESET_Pin|A0_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : CS_Pin RESET_Pin A0_Pin */
  GPIO_InitStruct.Pin = CS_Pin|RESET_Pin|A0_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : USART_RX_Pin */
  GPIO_InitStruct.Pin = USART_RX_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF7_USART2;
  HAL_GPIO_Init(USART_RX_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PB10 */
  GPIO_InitStruct.Pin = GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : Hall2_Pin Hall1_Pin */
  GPIO_InitStruct.Pin = Hall2_Pin|Hall1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
int __io_putchar(int ch)
{
	ITM_SendChar(ch);
	return 0;
}
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if(huart->Instance == UART4){
		nmea[i]=rx_buff[0];
		i++;
		HAL_UART_Receive_IT(&huart4, rx_buff, 1);
	}
}
void check_buffer(){
	if(nmea[0]!=36){
		i=0;
	}
	if(nmea[i-1]==10){

		i=0;
		if (nmea[0]!='$'){
			printf("error");
		}
		printf(nmea);
		clear_buff();
	}
}
void clear_buff(){
//	for(int x=86; x>-1;--x){
//		nmea[x]= '';
//
//	}
	//actually only need this one line
	memset(&nmea,0,87);
}

//taken from bmi323 lib
static int8_t set_accel_gyro_config(struct bmi3_dev *dev)
{
    /* Status of API are returned to this variable. */
    int8_t rslt;

    /* Structure to define accelerometer configuration. */
    struct bmi3_sens_config config[2];

    /* Structure to map interrupt */
    struct bmi3_map_int map_int = { 0 };

    /* Configure the type of feature. */
    config[0].type = BMI323_ACCEL;
    config[1].type = BMI323_GYRO;

    /* Get default configurations for the type of feature selected. */
    rslt = bmi323_get_sensor_config(config, 2, dev);
    bmi3_error_codes_print_result("bmi323_get_sensor_config", rslt);

    if (rslt == BMI323_OK)
    {
        map_int.acc_drdy_int = BMI3_INT1;
        map_int.gyr_drdy_int = BMI3_INT1;
        map_int.temp_drdy_int = BMI3_INT1;

        /* Map data ready interrupt to interrupt pin. */
        rslt = bmi323_map_interrupt(map_int, dev);
        bmi3_error_codes_print_result("bmi323_map_interrupt", rslt);

        if (rslt == BMI323_OK)
        {
            /* NOTE: The user can change the following configuration parameters according to their requirement. */
            /* Output Data Rate. By default ODR is set as 100Hz for accel. */
            config[0].cfg.acc.odr = BMI3_ACC_ODR_50HZ;

            /* Gravity range of the sensor (+/- 2G, 4G, 8G, 16G). */
            config[0].cfg.acc.range = BMI3_ACC_RANGE_2G;

            /* The Accel bandwidth coefficient defines the 3 dB cutoff frequency in relation to the ODR. */
            config[0].cfg.acc.bwp = BMI3_ACC_BW_ODR_QUARTER;

            /* Set number of average samples for accel. */
            config[0].cfg.acc.avg_num = BMI3_ACC_AVG64;

            /* Enable the accel mode where averaging of samples
             * will be done based on above set bandwidth and ODR.
             * Note : By default accel is disabled. The accel will get enable by selecting the mode.
             */
            config[0].cfg.acc.acc_mode = BMI3_ACC_MODE_NORMAL;

            /* Output Data Rate. By default ODR is set as 100Hz for gyro. */
            config[1].cfg.gyr.odr = BMI3_GYR_ODR_50HZ;

            /* Gyroscope Angular Rate Measurement Range. By default the range is 2000dps. */
            config[1].cfg.gyr.range = BMI3_GYR_RANGE_2000DPS;

            /*  The Gyroscope bandwidth coefficient defines the 3 dB cutoff frequency in relation to the ODR
             *  Value   Name      Description
             *    0   odr_half   BW = gyr_odr/2
             *    1  odr_quarter BW = gyr_odr/4
             */
            config[1].cfg.gyr.bwp = BMI3_GYR_BW_ODR_HALF;

            /* By default the gyro is disabled. Gyro is enabled by selecting the mode. */
            config[1].cfg.gyr.gyr_mode = BMI3_GYR_MODE_NORMAL;

            /* Value    Name    Description
             *  0b000     avg_1   No averaging; pass sample without filtering
             *  0b001     avg_2   Averaging of 2 samples
             *  0b010     avg_4   Averaging of 4 samples
             *  0b011     avg_8   Averaging of 8 samples
             *  0b100     avg_16  Averaging of 16 samples
             *  0b101     avg_32  Averaging of 32 samples
             *  0b110     avg_64  Averaging of 64 samples
             */
            config[1].cfg.gyr.avg_num = BMI3_GYR_AVG1;

            /* Set the accel and gyro configurations. */
            rslt = bmi323_set_sensor_config(config, 2, dev);
            bmi3_error_codes_print_result("bmi323_set_sensor_config", rslt);
        }
    }

    return rslt;
}

/*! @brief Converts raw sensor values(LSB) to G value
 *
 *  @param[in] val        : Raw sensor value.
 *  @param[in] g_range    : Accel Range selected (4G).
 *  @param[in] bit_width  : Resolution of the sensor.
 *
 *  @return Accel values in Gravity(G)
 *
 */
static float lsb_to_g(int16_t val, float g_range, uint8_t bit_width)
{
    double power = 2;

    float half_scale = (float)((pow((double)power, (double)bit_width) / 2.0f));

    return (val * g_range) / half_scale;
}

/*!
 * @brief This function converts lsb to degree per second for 16 bit gyro at
 * range 125, 250, 500, 1000 or 2000dps.
 */
static float lsb_to_dps(int16_t val, float dps, uint8_t bit_width)
{
    double power = 2;

    float half_scale = (float)((pow((double)power, (double)bit_width) / 2.0f));

    return (dps / (half_scale)) * (val);
}

static void on_A_rise()
{
  switch (Magnet_st) {
    case IDLE:

    	Magnet_st = WAIT_B_AFTER_A;

    	break;
    case WAIT_A_AFTER_B:

      magnets_ccw++;
      magnets_total++;
      magnets_net= magnets_cw-magnets_ccw ;

      Magnet_st = IDLE; break;
    case WAIT_B_AFTER_A:
    	break;
  }
}

static void on_A_fall()
{
  if (Magnet_st == WAIT_B_AFTER_A && HallB) {

	  Magnet_st = IDLE;
  }
}

static void on_B_rise()
{
  switch (Magnet_st) {
    case IDLE:
    	Magnet_st = WAIT_A_AFTER_B;
    	break;
    case WAIT_B_AFTER_A:
      magnets_cw++;
      magnets_total++;
      magnets_net++;
      Magnet_st = IDLE;
      break;
    case WAIT_A_AFTER_B:
    	break;
  }
}

static void on_B_fall()
{
  if (Magnet_st == WAIT_A_AFTER_B && HallA) {

	  Magnet_st = IDLE;
  }
}
void calculate_depth(){
	magnets_net = magnets_cw - magnets_ccw;
	line_out = abs(magnets_net) * (circumference_inches/magnets_in_array)*(1.0/12.0);


	if((angle < 90.001)){
		depth_ft = line_out*(cos(angle*0.0174532925));
	}




}
void check_angle(){
	  position= TIM1->CNT;

	  if (!HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_10)){
		  __HAL_TIM_SET_COUNTER(&htim1,0);
	  }


	  angle = (position/2048.0)*360;
	  //angle = (int)((position/2048.0)*360);

	  if (angle>180){
		angle = 360-angle;

	  }

}




void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{

	if( (GPIO_Pin==Hall2_Pin) || (GPIO_Pin==Hall1_Pin) ){
		HallA = ((GPIOB->IDR) >> 9) & 1;
		HallB = ((GPIOB->IDR) >> 8) & 1;
	}
	  if (GPIO_Pin == Hall1_Pin) {

	    if (!HallA) {
	    	on_A_rise();
	    }
	    else
	    	on_A_fall();
	  }
	  else if (GPIO_Pin == Hall2_Pin) {
	    if (!HallB) {
	    	on_B_rise();
	    }
	    else
	    	on_B_fall();
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
