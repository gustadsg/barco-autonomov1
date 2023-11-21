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
#include "BLE-JDY18.h"
#include "HMC5883L.h"
#include "Servo.h"
#include "PID.h"
#include "Positioning_BLE.h"
#include "DCMotor.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define SERVO_PERIOD 1250;
#define SERVO_MIN_DUTY_CICLE 0.05;
#define SERVO_MAX_DUTY_CICLE 0.115;
#define SERVO_CALIBRATION_GAIN 1.47;
#define SERVO_CALIBRATION_OFFSET -12.6;

#define DCMOTOR_PERIOD 1250;

#define PID_SERVO_KP 1;
#define PID_SERVO_KI 1;
#define PID_SERVO_KD 1;
#define PID_PERIOD_MS 500;
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM3_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART3_UART_Init(void);
/* USER CODE BEGIN PFP */
void setPWMM(TIM_HandleTypeDef timer, uint32_t channel, uint32_t period,
		uint16_t pulseLength);
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
	MX_TIM3_Init();
	MX_I2C1_Init();
	MX_USART2_UART_Init();
	MX_USART3_UART_Init();
	/* USER CODE BEGIN 2 */

	//UART2 CONFIGURATION
	static uint8_t HAL_Status = 0x01U; // HAL_StatusTypeDef
	static uint32_t Timeout = 1000; // In miliseconds
	static char vetor[15] = "Estou de volta\n";

	// POSITION AND DC MOTOR CONFIGURATION
	JDY18_Setup(&huart3);
	JDY18_SetRole(JDY18_ROLE_MASTER);

	JDY18_Device_t devices[JDY18_MAX_DEVICES];
	POSITIONING_BLE_Config_t beaconPositioningConfig;

	POSITIONING_BLE_Devices_Info_t devicesInfo;
	strcpy(devicesInfo.departureDevice.name,"PSE2022_B1");
	devicesInfo.departureDevice.x = 11.495;
	devicesInfo.departureDevice.y = 34.342;

	strcpy(devicesInfo.arrivalDevice.name,"PSE2022_B2");
	devicesInfo.arrivalDevice.x = 0;
	devicesInfo.arrivalDevice.y = 0;

	strcpy(devicesInfo.otherDevice.name,"PSE2022_B3");
	devicesInfo.otherDevice.x = -12.64;
	devicesInfo.otherDevice.y = 16.948;

	// TODO: verify configuration and adjust according to the pin map (not released yet)
	int dcMotorPeriod = DCMOTOR_PERIOD;
	DCMOTOR_TimerConfig_t dcmotorConfig;
	dcmotorConfig.channel = TIM_CHANNEL_3;
	dcmotorConfig.handle = htim3;
	dcmotorConfig.period = dcMotorPeriod;

	// SERVO AND HMC588L CONFIGURATION
	SERVO_TimerConfig_t servoPWMConfig;
	servoPWMConfig.handle = htim3;
	servoPWMConfig.channel = TIM_CHANNEL_2;
	const uint32_t servoPeriod = SERVO_PERIOD;
	servoPWMConfig.period = servoPeriod;
	const float servoMinDutyCicle = SERVO_MIN_DUTY_CICLE;
	const float servoMaxDutyCicle = SERVO_MAX_DUTY_CICLE;
	servoPWMConfig.minDutyCyclePercentage = servoMinDutyCicle;
	servoPWMConfig.maxDutyCyclePercentage = servoMaxDutyCicle;

	SERVO_Calibration_t servoCalibration;
	const float servoCalibrationGain = SERVO_CALIBRATION_GAIN;
	const float servoOffset = SERVO_CALIBRATION_OFFSET;
	servoCalibration.gain = servoCalibrationGain;
	servoCalibration.offset = servoOffset;

	SERVO_Config_t servoConfig;
	servoConfig.timerConfig = servoPWMConfig;
	servoConfig.calibration = servoCalibration;

	HMC5883L_Config_t magnetometerConfig;
	magnetometerConfig.dataOutputRate = HMC5883L_DOR_15;
	magnetometerConfig.gain = HMC5883L_GAIN_0_88;
	magnetometerConfig.measurementMode = HMC5883L_MESUAREMENT_NORMAL;
	magnetometerConfig.operatingMode = HMC5883L_CONTINUOUS_MODE;
	magnetometerConfig.samplesNum = HMC5883L_SAMPLES_8;
	magnetometerConfig.handle = &hi2c1;
	HMC5883L_Init(magnetometerConfig);
	HMC5883L_GetCalibrationData(magnetometerConfig, &huart2);

	HMC5883L_Data_t magnetometerData = { 0, 0, 0, 0, 0 };

	
	// CONTROLLERS CONFIGURATION
	const float pidServoKp = PID_SERVO_KP;
	const float pidServoKi = PID_SERVO_KI;
	const float pidServoKd = PID_SERVO_KD;
	const int pidPeriodMs = PID_PERIOD_MS;
	const float servoMinAngle = SERVO_MIN_ANGLE;
	const float servoMaxAngle = SERVO_MAX_ANGLE;
	const float servoMiddleAngle = SERVO_MIDDLE_ANGLE;
	float servoAction = 0;

	PID_Controller_t controllerServo;
	PID_Create(&controllerServo, pidServoKp, pidServoKi, pidServoKd, pidPeriodMs);
	PID_SetSaturationLimits(&controllerServo, servoMinAngle, servoMaxAngle);
	PID_SetSetpoint(&controllerServo, servoMiddleAngle);


	const float pidDcMotorKp = 1;
	const float pidDcMotorKi = 0;
	const float pidDcMotorKd = 0;
	const int pidPeriodMs = PID_PERIOD_MS;
	const float DcMotorMinPercentage = 50;
	const float DcMotorMaxPercentage = 100;
	float dcMotorAction = 0;

	PID_Controller_t controllerDcMotor;
	PID_Create(&controllerDcMotor, pidDcMotorKp, pidDcMotorKi, pidDcMotorKd, pidPeriodMs);
	PID_SetSaturationLimits(&controllerDcMotor, DcMotorMinPercentage, DcMotorMaxPercentage);
	PID_SetSetpoint(&controllerDcMotor, 0);

	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	//Device_t devices[MAX_DEVICES];
	while (1) {
		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */

		// DC Motor control
		int numOfDevices = JDY18_Scan(devices);
		POSITIONING_BLE_CreateConfig(&beaconPositioningConfig, devicesInfo, devices, numOfDevices);
		const POSITIONING_BLE_Cartesian_Point_t currentPosition = POSITIONING_BLE_GetPosition(&beaconPositioningConfig);
		
		HAL_Status =  HAL_UART_Transmit(&huart2, (uint8_t*) vetor, strlen(vetor), Timeout);

		float distanceFromArrival = sqrt(currentPosition.x*currentPosition.x + currentPosition.y*currentPosition.y);
		PID_ProcessInput(&controllerDcMotor, distanceFromArrival);
		dcMotorAction = PID_CalculateControlAction(&controllerDcMotor);
		DCMOTOR_SetSpeedPercentage(dcmotorConfig, dcMotorAction);

		// Servo control
		HMC5883L_Read(magnetometerConfig, &magnetometerData);
		PID_ProcessInput(&controllerServo, magnetometerData.degrees);
		servoAction = PID_CalculateControlAction(&controllerServo);
		SERVO_SetAngle(servoConfig, servoAction);

		// TODO: create bare metal final state machine (or freeRTOS program)
		HAL_Delay(5000);
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
	__HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

	/** Initializes the RCC Oscillators according to the specified parameters
	 * in the RCC_OscInitTypeDef structure.
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
		Error_Handler();
	}

	/** Initializes the CPU, AHB and APB buses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
			| RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV8;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV8;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK) {
		Error_Handler();
	}
}

/**
 * @brief I2C1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_I2C1_Init(void) {

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
	if (HAL_I2C_Init(&hi2c1) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN I2C1_Init 2 */

	/* USER CODE END I2C1_Init 2 */

}

/**
 * @brief TIM3 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM3_Init(void) {

	/* USER CODE BEGIN TIM3_Init 0 */

	/* USER CODE END TIM3_Init 0 */

	TIM_MasterConfigTypeDef sMasterConfig = { 0 };
	TIM_OC_InitTypeDef sConfigOC = { 0 };

	/* USER CODE BEGIN TIM3_Init 1 */

	/* USER CODE END TIM3_Init 1 */
	htim3.Instance = TIM3;
	htim3.Init.Prescaler = 0;
	htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim3.Init.Period = 1250;
	htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_PWM_Init(&htim3) != HAL_OK) {
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig)
			!= HAL_OK) {
		Error_Handler();
	}
	sConfigOC.OCMode = TIM_OCMODE_PWM1;
	sConfigOC.Pulse = 94;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2)
			!= HAL_OK) {
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
 * @brief USART3 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART3_UART_Init(void) {

	/* USER CODE BEGIN USART3_Init 0 */

	/* USER CODE END USART3_Init 0 */

	/* USER CODE BEGIN USART3_Init 1 */

	/* USER CODE END USART3_Init 1 */
	huart3.Instance = USART3;
	huart3.Init.BaudRate = 9600;
	huart3.Init.WordLength = UART_WORDLENGTH_8B;
	huart3.Init.StopBits = UART_STOPBITS_1;
	huart3.Init.Parity = UART_PARITY_NONE;
	huart3.Init.Mode = UART_MODE_TX_RX;
	huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart3.Init.OverSampling = UART_OVERSAMPLING_16;
	if (HAL_UART_Init(&huart3) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN USART3_Init 2 */

	/* USER CODE END USART3_Init 2 */

}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void) {
	GPIO_InitTypeDef GPIO_InitStruct = { 0 };
	/* USER CODE BEGIN MX_GPIO_Init_1 */
	/* USER CODE END MX_GPIO_Init_1 */

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOC_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin : LED_Pin */
	GPIO_InitStruct.Pin = LED_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(LED_GPIO_Port, &GPIO_InitStruct);

	/* USER CODE BEGIN MX_GPIO_Init_2 */
	/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void setPWMM(TIM_HandleTypeDef timer, uint32_t channel, uint32_t period,
		uint16_t pulseLength) {
	HAL_TIM_PWM_Stop(&timer, channel); // stop generation of pwm
	TIM_OC_InitTypeDef sConfigOC;
	timer.Init.Period = period; // set the period duration
	HAL_TIM_PWM_Init(&timer); // reinitialise with new period value
	sConfigOC.OCMode = TIM_OCMODE_PWM1;
	sConfigOC.Pulse = pulseLength; // set the pulse duration
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	HAL_TIM_PWM_ConfigChannel(&timer, &sConfigOC, channel);
	HAL_TIM_PWM_Start(&timer, channel); // start PWM generation
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
