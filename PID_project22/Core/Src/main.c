/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2023 STMicroelectronics.
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

#include "user_function.h"
#include "math.h"

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
ADC_HandleTypeDef hadc1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim5;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

float position_milli = 0;

// time
float dt = 0;
uint64_t _micros = 0;

//position
// PID position term
float P_position_term = 0;
float I_position_term = 0;
float D_position_term = 0;

// constant
float Kp_position = 0;
float Ki_position = 0;
float Kd_position = 0;

// error of position
float error_position = 0;

// position
float position_now = 0;
float position_past = 0;
float position_setpoint = 0;

//integrate position
float integrate_position = 0;
float PID_position_total = 0;
float Duty_feedback_position = 0;

//velocity
// PID velocity term
float P_velocity_term = 0;
float I_velocity_term = 0;
float D_velocity_term = 0;

// constant
float Kp_velocity = 0.0063;
float Ki_velocity = 0;
float Kd_velocity = 0;

// error of velocity
float error_velocity = 0;

//integrate velocity
float integrate_velocity = 0;
float PID_velocity_total = 0;
float Duty_feedback_velocity = 0;

// velocity
float velocity_now = 0;
float velocity_past = 0;
float velocity_setpoint = 0;

// trajectory
// position
float position_acc = 0;
float position_const = 0;
float position_dec = 0;
float position_now_acc = 0;
float position_now_const = 0;
float position_now_dec = 0;
float position_segment = 0;
float distance_one_travel = 0;
float abs_distance_one_travel = 0;

// velocity
float rpm = 0;
float velocity_max = 17000;
float velocity_start = 0;
float velocity_end = 0;
float velocity_triangle = 0;

// acceleration
float acceleration_max = 9000;

// time
float time_acc = 0;
float time_const = 0;
float time_dec = 0;
float time_total = 0;
float time_now = 0;
float time_trajectory = 0;
int sign_of_distance = 0;

float setpoint_past = 0;
float setpoint_now = 0;

//bababababa
uint32_t QEIReadPosition;

float velocity_check = 0;
float velocity_check_filter = 0;

float duty = 500;
int reset = 0;

float BITtoDegree = 0;
float DegreeOfMotor = 0;
float setposition = 0;
float Vfeedback = 0;

typedef struct _QEIStructure {
	float data[2];
	float QETPosition;
} QEIStructureTypedef;
QEIStructureTypedef QEIData = { 0 };

float one = 0;
float two = 0;

float filter_position_past = 0;
float filter_position_now = 0;
float filter_velocity = 0;

float C = 0;

int32_t QEIReadRaw_now;
int32_t QEIReadRaw_past;
float voltage = 0;
int32_t setpoint = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM5_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */

inline uint64_t micros();
void Trajectory();
void motor(float voltage);
void distance();
void VelocityControlPID();

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void) {
	/* USER CODE BEGIN 1 */

	C = ComputeLowpassConstant(1000, 5000);
	// C1 = ComputeLowpassConstant(1000, 5000);
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
	MX_TIM3_Init();
	MX_TIM1_Init();
	MX_TIM5_Init();
	MX_ADC1_Init();
	MX_TIM2_Init();
	/* USER CODE BEGIN 2 */

	HAL_TIM_Base_Start(&htim1);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);

	HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_1 | TIM_CHANNEL_2);

	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1) {
		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */
		static uint32_t timestamp = 0;
		int64_t currentTime = micros();
		if (HAL_GetTick() > timestamp) {
			timestamp = currentTime + 100;
			dt = 0.0001;
			distance();
			Trajectory();
			//velocity_check = (QEIReadRaw_now - QEIReadRaw_past)/dt;
////		  PositionControlPID();
			QEIReadRaw_now = __HAL_TIM_GET_COUNTER(&htim2);
			VelocityControlPID();
			velocity_check = (QEIReadRaw_now - QEIReadRaw_past)/dt;
			velocity_check_filter = (C * velocity_check) + ((1-C)*velocity_check_filter);
			motor(voltage);
			QEIReadRaw_past = QEIReadRaw_now;

		}

		// auto run
		static uint32_t timestamp2 = 0;
		if (HAL_GetTick() > timestamp2) {
			timestamp2 = HAL_GetTick() + 1400;
			if(!setpoint_now)
			{
				setpoint_now = 20000;
			}
			else
			{
				setpoint_now = 0;
			}
		}

//	  static uint32_t timestamp = 0;
//	  if(HAL_GetTick() > timestamp)
//	  {
//		  timestamp = HAL_GetTick() + 5;
//		  dt = 0.005;
//		  QEIEncoderPosition();
//		  VelocityControlPID();
//		  Drivemotor();
//	  }
	}
	/* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void) {
	RCC_OscInitTypeDef RCC_OscInitStruct = { 0 };
	RCC_ClkInitTypeDef RCC_ClkInitStruct = { 0 };

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
	RCC_OscInitStruct.PLL.PLLN = 100;
	RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
	RCC_OscInitStruct.PLL.PLLQ = 4;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
		Error_Handler();
	}

	/** Initializes the CPU, AHB and APB buses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
			| RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK) {
		Error_Handler();
	}
}

/**
 * @brief ADC1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_ADC1_Init(void) {

	/* USER CODE BEGIN ADC1_Init 0 */

	/* USER CODE END ADC1_Init 0 */

	ADC_ChannelConfTypeDef sConfig = { 0 };

	/* USER CODE BEGIN ADC1_Init 1 */

	/* USER CODE END ADC1_Init 1 */

	/** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
	 */
	hadc1.Instance = ADC1;
	hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
	hadc1.Init.Resolution = ADC_RESOLUTION_12B;
	hadc1.Init.ScanConvMode = DISABLE;
	hadc1.Init.ContinuousConvMode = DISABLE;
	hadc1.Init.DiscontinuousConvMode = DISABLE;
	hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
	hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
	hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
	hadc1.Init.NbrOfConversion = 1;
	hadc1.Init.DMAContinuousRequests = DISABLE;
	hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
	if (HAL_ADC_Init(&hadc1) != HAL_OK) {
		Error_Handler();
	}

	/** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
	 */
	sConfig.Channel = ADC_CHANNEL_10;
	sConfig.Rank = 1;
	sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN ADC1_Init 2 */

	/* USER CODE END ADC1_Init 2 */

}

/**
 * @brief TIM1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM1_Init(void) {

	/* USER CODE BEGIN TIM1_Init 0 */

	/* USER CODE END TIM1_Init 0 */

	TIM_ClockConfigTypeDef sClockSourceConfig = { 0 };
	TIM_MasterConfigTypeDef sMasterConfig = { 0 };
	TIM_OC_InitTypeDef sConfigOC = { 0 };
	TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = { 0 };

	/* USER CODE BEGIN TIM1_Init 1 */

	/* USER CODE END TIM1_Init 1 */
	htim1.Instance = TIM1;
	htim1.Init.Prescaler = 125 - 1;
	htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim1.Init.Period = 999;
	htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim1.Init.RepetitionCounter = 0;
	htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_Base_Init(&htim1) != HAL_OK) {
		Error_Handler();
	}
	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK) {
		Error_Handler();
	}
	if (HAL_TIM_PWM_Init(&htim1) != HAL_OK) {
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig)
			!= HAL_OK) {
		Error_Handler();
	}
	sConfigOC.OCMode = TIM_OCMODE_PWM1;
	sConfigOC.Pulse = 0;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
	sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
	if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1)
			!= HAL_OK) {
		Error_Handler();
	}
	sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
	sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
	sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
	sBreakDeadTimeConfig.DeadTime = 0;
	sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
	sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
	sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
	if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig)
			!= HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN TIM1_Init 2 */

	/* USER CODE END TIM1_Init 2 */
	HAL_TIM_MspPostInit(&htim1);

}

/**
 * @brief TIM2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM2_Init(void) {

	/* USER CODE BEGIN TIM2_Init 0 */

	/* USER CODE END TIM2_Init 0 */

	TIM_Encoder_InitTypeDef sConfig = { 0 };
	TIM_MasterConfigTypeDef sMasterConfig = { 0 };

	/* USER CODE BEGIN TIM2_Init 1 */

	/* USER CODE END TIM2_Init 1 */
	htim2.Instance = TIM2;
	htim2.Init.Prescaler = 0;
	htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim2.Init.Period = 4294967295;
	htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
	sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
	sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
	sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
	sConfig.IC1Filter = 0;
	sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
	sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
	sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
	sConfig.IC2Filter = 0;
	if (HAL_TIM_Encoder_Init(&htim2, &sConfig) != HAL_OK) {
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig)
			!= HAL_OK) {
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
static void MX_TIM3_Init(void) {

	/* USER CODE BEGIN TIM3_Init 0 */

	/* USER CODE END TIM3_Init 0 */

	TIM_Encoder_InitTypeDef sConfig = { 0 };
	TIM_MasterConfigTypeDef sMasterConfig = { 0 };

	/* USER CODE BEGIN TIM3_Init 1 */

	/* USER CODE END TIM3_Init 1 */
	htim3.Instance = TIM3;
	htim3.Init.Prescaler = 0;
	htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim3.Init.Period = 3071;
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
	if (HAL_TIM_Encoder_Init(&htim3, &sConfig) != HAL_OK) {
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig)
			!= HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN TIM3_Init 2 */

	/* USER CODE END TIM3_Init 2 */

}

/**
 * @brief TIM5 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM5_Init(void) {

	/* USER CODE BEGIN TIM5_Init 0 */

	/* USER CODE END TIM5_Init 0 */

	TIM_ClockConfigTypeDef sClockSourceConfig = { 0 };
	TIM_MasterConfigTypeDef sMasterConfig = { 0 };

	/* USER CODE BEGIN TIM5_Init 1 */

	/* USER CODE END TIM5_Init 1 */
	htim5.Instance = TIM5;
	htim5.Init.Prescaler = 83;
	htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim5.Init.Period = 4294967295;
	htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim5.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_Base_Init(&htim5) != HAL_OK) {
		Error_Handler();
	}
	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	if (HAL_TIM_ConfigClockSource(&htim5, &sClockSourceConfig) != HAL_OK) {
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim5, &sMasterConfig)
			!= HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN TIM5_Init 2 */

	/* USER CODE END TIM5_Init 2 */

}

/**
 * @brief USART2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART2_UART_Init(void) {

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
	if (HAL_UART_Init(&huart2) != HAL_OK) {
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
static void MX_GPIO_Init(void) {
	GPIO_InitTypeDef GPIO_InitStruct = { 0 };

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOC_CLK_ENABLE();
	__HAL_RCC_GPIOH_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOA, LD2_Pin | GPIO_PIN_11, GPIO_PIN_RESET);

	/*Configure GPIO pin : B1_Pin */
	GPIO_InitStruct.Pin = B1_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pins : PC2 PC3 */
	GPIO_InitStruct.Pin = GPIO_PIN_2 | GPIO_PIN_3;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

	/*Configure GPIO pins : LD2_Pin PA11 */
	GPIO_InitStruct.Pin = LD2_Pin | GPIO_PIN_11;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

void QEIEncoderPosition() {
	QEIReadPosition = __HAL_TIM_GET_COUNTER(&htim3);
	BITtoDegree = (QEIReadPosition * 360.0) / 3072.0;

	//collect data
	QEIData.data[0] = BITtoDegree;

	//calculation
	float diffposition = QEIData.data[0] - QEIData.data[1];

	//handle wrap-around
	if (diffposition > 180) {
		diffposition = diffposition - 360.0;
		velocity_now = velocity_now + diffposition;
	} else if (diffposition < -180) {
		diffposition = diffposition + 360.0;
		velocity_now = velocity_now + diffposition;
	} else {
		velocity_now = velocity_now + diffposition;
	}

	//velocity_check = ((QEIData.data[0]-QEIData.data[1]) * 1000000)/100;
	//velocity_now = (C * velocity_check) + ((1-C)*velocity_now);
	//velocity_now = position_now;
	QEIData.data[1] = QEIData.data[0];

}

void PositionControlPID() {
	error_position = position_setpoint - position_now;

	// P-term-position
	P_position_term = Kp_position * error_position;

	// I-term-position
	if (((distance_one_travel - 0.1) < position_now)
			&& (position_now < (distance_one_travel + 0.1))) {
		integrate_position = 0;
	} else {
		integrate_position += (error_position * dt);
	}
	I_position_term = Ki_position * integrate_position;

	// D-term-position
	D_position_term = Kd_position * (error_position / dt);

	// PID-position
	PID_position_total = P_position_term + I_position_term + D_position_term;
}

void VelocityControlPID() {
	error_velocity = velocity_setpoint - QEIReadRaw_now;
	//+ PID_position_total

	// P-term-velocity
	P_velocity_term = Kp_velocity * error_velocity;

	// I-term-velocity
	if (((setpoint_now - 0.1) < QEIReadRaw_now)
			&& (QEIReadRaw_now < (setpoint_now + 0.1)))
			//(((velocity_end - 0.1) < velocity_now) && (velocity_now < (velocity_end + 0.1))&& ((time_trajectory - 0.01) < time_total) && (time_total < (time_trajectory + 0.01)))
			{
		integrate_velocity = 0;
	} else {
		integrate_velocity += (error_velocity * dt);
	}
	I_velocity_term = Ki_velocity * integrate_velocity;

	// D-term-velocity
	D_velocity_term = Kd_velocity * error_velocity / dt;

	// PID-velocity
	voltage = P_velocity_term + I_velocity_term + D_velocity_term;
}

void distance()
{
	if (setpoint_past != setpoint_now)
	{
		distance_one_travel = setpoint_now - setpoint_past;
		setpoint_past = setpoint_now;
		if (distance_one_travel >= 0)
		{
			sign_of_distance = 1;
			abs_distance_one_travel = distance_one_travel;
		}
		else if (distance_one_travel < 0)
		{
			sign_of_distance = -1;
			abs_distance_one_travel = distance_one_travel * (-1);
		}
	}
	else
	{
		setpoint_past = setpoint_past;
	}
}

void Trajectory()
{
	if (abs_distance_one_travel > ((velocity_max * velocity_max) / acceleration_max))
	{
			time_acc = ((velocity_max - 0) / acceleration_max);
			time_const = ((1.0 / velocity_max)
					* ((abs_distance_one_travel)
							- ((velocity_max * velocity_max) / acceleration_max)));
			time_dec = ((0 - velocity_max) / ((-1) * acceleration_max));
			time_total = time_acc + time_const + time_dec;
	}
	else
	{
		time_acc = sqrt(abs_distance_one_travel / (acceleration_max));
		time_total = 2 * time_acc;
		velocity_triangle = time_acc * acceleration_max;
	}

	if (abs_distance_one_travel > ((velocity_max * velocity_max) / acceleration_max))
	{
		if ((0 <= time_trajectory) && (time_trajectory < time_acc)) {
			time_trajectory += 0.0001;
			time_now = time_trajectory;
			position_segment = (velocity_start * time_now)
					+ (1 / 2 * acceleration_max * (time_now * time_now));

			position_now_acc = position_now_dec
					+ (position_segment * sign_of_distance);
			position_setpoint = position_now_dec
					+ (position_segment * sign_of_distance);

			//velocity_setpoint = (acceleration_max * sign_of_distance * time_now) + velocity_start;
		}

		// constant segment
		else if ((time_acc <= time_trajectory)
				&& (time_trajectory < (time_acc + time_const))) {
			time_trajectory += 0.0001;
			time_now = time_trajectory - time_acc;
			position_segment = (velocity_max * time_now);

			position_now_const = position_now_acc
					+ (position_segment * sign_of_distance);
			position_setpoint = position_now_acc
					+ (position_segment * sign_of_distance);

			//velocity_setpoint = sign_of_distance*velocity_max;
		}

		// deceleration segment
		else if (((time_acc + time_const) <= time_trajectory)
				&& (time_trajectory <= time_total)) {
			time_trajectory += 0.0001;
			time_now = time_trajectory - (time_acc + time_const);
			position_segment = (velocity_max * time_now)
					+ (1 / 2 * (-1) * acceleration_max * (time_now * time_now));

			position_now_dec = position_now_const
					+ (position_segment * sign_of_distance);
			position_setpoint = position_now_const
					+ (position_segment * sign_of_distance);

			//velocity_setpoint = ((-1)*acceleration_max * time_now *sign_of_distance) + (velocity_max*sign_of_distance);

		}
	}

	else {
		if ((0 <= time_trajectory) && (time_trajectory < time_acc)) {
			time_trajectory += 0.0001;
			time_now = time_trajectory;
			position_segment = (velocity_start * time_now)
					+ (1 / 2 * acceleration_max * (time_now * time_now));

			position_now_acc = position_now_dec
					+ (position_segment * sign_of_distance);
			position_setpoint = position_now_dec
					+ (position_segment * sign_of_distance);

			//velocity_setpoint = (acceleration_max * sign_of_distance * time_now) + velocity_start;
		}

		else if ((time_acc <= time_trajectory)
				&& (time_trajectory < time_total)) {
			time_trajectory += 0.0001;
			time_now = time_trajectory - (time_acc);
			position_segment = (velocity_triangle * time_now)
					+ (1 / 2 * (-1) * acceleration_max * (time_now * time_now));

			position_now_dec = position_now_acc
					+ (position_segment * sign_of_distance);
			position_setpoint = position_now_acc
					+ (position_segment * sign_of_distance);

			//velocity_setpoint = ((-1)*acceleration_max * time_now * sign_of_distance) + (velocity_max*sign_of_distance);

		}

	}

	if ((setpoint_now - 0.5 < velocity_setpoint)
			&& (velocity_setpoint < setpoint_now + 0.5)) {
		time_trajectory = 0;
	}

	velocity_setpoint = position_setpoint;
}

void motor(float voltage) {
	if (voltage > 0) {
		// forward
		if (voltage > 12.0) {
			voltage = 12.0;
		}
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, RESET);
	} else if (voltage < 0) {
		// backward
		voltage *= -1.0;
		if (voltage > 12.0) {
			voltage = 12.0;
		}
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, SET);
	} else {
		// stop
		voltage = 0;
	}

	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, voltage * 1000.0 / 12.0);
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
	if (htim == &htim5) {
		_micros += UINT32_MAX;
	}
}

uint64_t micros() {
	return __HAL_TIM_GET_COUNTER(&htim5) + _micros;
}

/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void) {
	/* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	__disable_irq();
	while (1) {
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
