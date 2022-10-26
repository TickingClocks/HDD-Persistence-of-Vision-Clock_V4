/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  *CubeMX was used to generate code to setup the microcontroller.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <stdlib.h>
#include "APA102-2020.h"
#include "DS1307_Basic.h"
#include "NCT175.h"

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

SPI_HandleTypeDef hspi2;
SPI_HandleTypeDef hspi5;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim5;
TIM_HandleTypeDef htim11;

/* USER CODE BEGIN PV */
HAL_StatusTypeDef 	status;

//--DISPLAY & CLOCK RGB AND GLOBAL VARIABLES--//
//flag indicating the need to update the display buffer
uint8_t updateDisplay 		= 1;
//flag indicating if the motor has successfully started
uint8_t motorSpinup 		= 0;
//flag to indicate stop the motor
uint8_t motorStop 			= 1;
//microsecond period of motor revolution
uint16_t motorMicroPeriod 	= 0;
//reduces microseconds off of the buffer time (use if needed)
uint8_t microNegBuf			= 1;
//LED buffer variables
uint8_t ledBuffer[120][4];
uint8_t ledBuffer2[3][4];
//display slices RGB array
uint8_t displaySlice[60][3];
uint8_t numberOfSlices		= 60;
//second hand RGB values
uint8_t secondR				= 20;
uint8_t secondG				= 50;
uint8_t secondB				= 255;
//minute hand RGB values
uint8_t minuteR				= 20;
uint8_t minuteG				= 255;
uint8_t minuteB				= 20;
//hour hand RGB values
uint8_t hourR				= 255;
uint8_t hourG				= 200;
uint8_t hourB				= 0;
//background color RGB values
uint8_t backgroundR			= 0;
uint8_t backgroundG			= 0;
uint8_t backgroundB			= 0;
//clock face index marks RGB values
uint8_t clockIndexR			= 15;
uint8_t clockIndexG			= 5;
uint8_t clockIndexB			= 2;
//edge running LED RGB values
uint8_t edgeRunnerR			= 5;
uint8_t edgeRunnerG			= 0;
uint8_t edgeRunnerB			= 0;
//edge runner toggle
uint8_t edgeRunner			= 0;

//--TIME/RTC VARIABLES--//
uint8_t testHour 			= 1;
uint8_t testMinute			= 5;
uint8_t testSecond 			= 0;

//--1 MILLISECOND TIMER--/
//setup timer 11 to be 1ms timer
//timer is currently pulsing 1,000,000times/sec (microsecond) loops at 1000 counts(1ms)
uint32_t timer11LoopCount 		= 0; //counts how many times timer11 has overflowed since startup
uint32_t timer11CountMarker 	= 0; //timer11LoopCount place marker. Used to count milliseconds
uint32_t timer11CountMarkerSec 	= 0; //timer11LoopCount place marker for seconds increment

//--Motor Variables--//
uint8_t updateMotorSpeed 		= 1;
uint8_t motorPWMSpeed 			= 0;

/*NOT USED YET
//Temperature Sensor Variables
float 	temperature1 				= 0.0;
float 	temperature2 				= 0.0;
float 	temperature3 				= 0.0;
float 	displayMaxTemperature 		= 140.0;
float 	displayMaxStartTemperature 	= 130.0;
*/
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_SPI2_Init(void);
static void MX_SPI5_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM11_Init(void);
static void MX_TIM5_Init(void);
/* USER CODE BEGIN PFP */
void clock(uint32_t secondInterval);
void motorInit(void);
void motorStartSequence(void);
void basicClockFace(uint8_t dotsR, uint8_t dotsG, uint8_t dotsB);
void setOuterEdgeLED(uint32_t timerCounterTotal, uint32_t timerCounterCurrent, uint32_t timerCounterBeginning);
void drawClockFrame(void);
void clearClockDisplay(void);
void setClockDisplay(uint8_t hour, uint8_t minute, uint8_t second);
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
  MX_I2C1_Init();
  MX_SPI2_Init();
  MX_SPI5_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM11_Init();
  MX_TIM5_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  //timer 3 (microsecond timer used for the display algorithm)
  HAL_TIM_Base_Start(&htim3);

  //timer11 (microsecond timer) used to count milliseconds.
  //interrupt increases timer11LoopCount variable by 1 each ms
  HAL_TIM_Base_Start_IT(&htim11);

  //start motor PWM at 0 duty cycle
  motorInit();

  //initialize RTC
  RTCInit();
  /*
  //set time
  RTCSetSeconds(1);
  RTCSetMinutes(29);
  RTCSetHours(8);
  */

  //turn all LEDs OFF
  //reset LED buffers
  LEDsReset();
  //update the DISPLAY
  updateLEDs(1);
  //update the REAR LED
  updateLEDs(2);

  //set time variables
  testSecond = RTCReadSeconds();
  testMinute = RTCReadMinutes();
  testHour = RTCReadHours();

  while (1)
  {

	  //check for motor stop command
	  if(motorStop){
		  motorPWMSpeed = 0;
		  updateMotorSpeed = 1;
		  setDisplayAllOneColor(0, 0, 0);
		  updateLEDs(1);
	  }

	  //check for motor speed update
	  if(updateMotorSpeed){
		  //update PWM signal
		  TIM2->CCR1 = motorPWMSpeed;
		  updateMotorSpeed = 0;
	  }

	  //check if the display needs to be updated
	  if(updateDisplay){
		  updateDisplay=0;
		  //remember mask spins counter clockwise so the variables need to be adjusted for that
		  setClockDisplay(12-testHour, 60-testMinute, 60-testSecond);
	  }

	  //check for the light mask index point
	  if((!HAL_GPIO_ReadPin(GPIOC, opticalSensor_Pin)) && (!motorStop)){ //sensor is NOT triggered (sees the slit)

		  if(motorSpinup){
			  //vary motor speed for 0.1s after initial motor spin up to
			  //ensure algorithm is working as it should
			  TIM2->CCR1 = 1; //slow motor
			  if(timer11LoopCount > (timer11CountMarker + 100)){
				  //revert to programmed speed
				  updateMotorSpeed = 1;
				  motorSpinup = 0;
			  }

		  }
		  //display algorithm
		  drawClockFrame();
	  }

	  //advance every second
	  clock(1000);

	  //button 1 code (BLOCKING)
	  if(!HAL_GPIO_ReadPin(GPIOB, button1_Pin)){
		  HAL_Delay(50);
		  if(!HAL_GPIO_ReadPin(GPIOB, button1_Pin)){
			  if(!motorStop){motorStop=1; updateMotorSpeed=1;}else{motorStop=0; motorPWMSpeed=15; motorStartSequence();} //add update motor speed after start to set speed
			  while(!HAL_GPIO_ReadPin(GPIOB, button1_Pin)){
				  //stuck here
			  }
		  }
	  }

	  //button 2 code (BLOCKING)
	  if(!HAL_GPIO_ReadPin(GPIOB, button2_Pin)){
		  HAL_Delay(50);
		  if(!HAL_GPIO_ReadPin(GPIOB, button2_Pin)){
			  while(!HAL_GPIO_ReadPin(GPIOB, button2_Pin)){
				  //stuck here
			  }
		  }
	  }

	  //button 3 code (BLOCKING)
	  if(!HAL_GPIO_ReadPin(GPIOB, button3_Pin)){
		  HAL_Delay(50);
		  if(!HAL_GPIO_ReadPin(GPIOB, button3_Pin)){
			  //toggle edge LED animation
			  if(edgeRunner){
				  edgeRunner = 0;
			  }else{
				  edgeRunner = 1;
				  motorSpinup = 1;
			  }
			  while(!HAL_GPIO_ReadPin(GPIOB, button3_Pin)){
				  //stuck here
			  }
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
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
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_2EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

}

/**
  * @brief SPI5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI5_Init(void)
{

  /* USER CODE BEGIN SPI5_Init 0 */

  /* USER CODE END SPI5_Init 0 */

  /* USER CODE BEGIN SPI5_Init 1 */

  /* USER CODE END SPI5_Init 1 */
  /* SPI5 parameter configuration*/
  hspi5.Instance = SPI5;
  hspi5.Init.Mode = SPI_MODE_MASTER;
  hspi5.Init.Direction = SPI_DIRECTION_2LINES;
  hspi5.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi5.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi5.Init.CLKPhase = SPI_PHASE_2EDGE;
  hspi5.Init.NSS = SPI_NSS_SOFT;
  hspi5.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_4;
  hspi5.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi5.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi5.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi5.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi5) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI5_Init 2 */

  /* USER CODE END SPI5_Init 2 */

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
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 100-1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 100-1;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

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
  htim3.Init.Prescaler = 100-1;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535;
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
  * @brief TIM5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM5_Init(void)
{

  /* USER CODE BEGIN TIM5_Init 0 */

  /* USER CODE END TIM5_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM5_Init 1 */

  /* USER CODE END TIM5_Init 1 */
  htim5.Instance = TIM5;
  htim5.Init.Prescaler = 0;
  htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim5.Init.Period = 4294967295;
  htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim5.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI1;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim5, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim5, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM5_Init 2 */

  /* USER CODE END TIM5_Init 2 */

}

/**
  * @brief TIM11 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM11_Init(void)
{

  /* USER CODE BEGIN TIM11_Init 0 */

  /* USER CODE END TIM11_Init 0 */

  /* USER CODE BEGIN TIM11_Init 1 */

  /* USER CODE END TIM11_Init 1 */
  htim11.Instance = TIM11;
  htim11.Init.Prescaler = 100-1;
  htim11.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim11.Init.Period = 1000;
  htim11.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim11.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim11) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM11_Init 2 */

  /* USER CODE END TIM11_Init 2 */

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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(testLED_GPIO_Port, testLED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : tempAlert2_Pin opticalSensor_Pin */
  GPIO_InitStruct.Pin = tempAlert2_Pin|opticalSensor_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : tempAlert1_Pin motor_FG_Pin PA4 fastMotor_slowMotor_Pin
                           motor_fault_Pin */
  GPIO_InitStruct.Pin = tempAlert1_Pin|motor_FG_Pin|GPIO_PIN_4|fastMotor_slowMotor_Pin
                          |motor_fault_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : button1_Pin button2_Pin button3_Pin buttonEncoder_Pin */
  GPIO_InitStruct.Pin = button1_Pin|button2_Pin|button3_Pin|buttonEncoder_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : testLED_Pin */
  GPIO_InitStruct.Pin = testLED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(testLED_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/*
 * @Summary	sets the display buffer for a clock face based on a time input
 * @Inputs	hour	(1 -12) value for the hours hand
 * 			minute	(0 - 60) value for the minutes hand
 * 			second	(0 - 60) value for the seconds hand
 * @Return	none
 */
void setClockDisplay(uint8_t hour, uint8_t minute, uint8_t second){

	uint8_t sliceOffset = 30; //this offsets the display by a set number of slices

	uint8_t offsetHour;
	uint8_t offsetMinute;
	uint8_t offsetSecond;

	//each clock hand takes up 1 display slice. There are 60 total slices
	//second hand
	offsetSecond = second + sliceOffset;
	if(offsetSecond > 59){offsetSecond-=60;}

	//minute hand
	offsetMinute = minute + sliceOffset;
	if(offsetMinute > 59){offsetMinute-=60;}

	//hour hand
	if(hour != 12){offsetHour = (hour * 5) + sliceOffset;}else{offsetHour=sliceOffset;}
	if(offsetHour > 59){offsetHour-=60;}

	//clear the clock display array
	clearClockDisplay();

	//hour hand
	displaySlice[offsetHour][0]=hourR;
	displaySlice[offsetHour][1]=hourG;
	displaySlice[offsetHour][2]=hourB;

	//minute hand
	displaySlice[offsetMinute][0]=minuteR;
	displaySlice[offsetMinute][1]=minuteG;
	displaySlice[offsetMinute][2]=minuteB;

	//second hand
	displaySlice[offsetSecond][0]=secondR;
	displaySlice[offsetSecond][1]=secondG;
	displaySlice[offsetSecond][2]=secondB;

}

/*
 * @Summary	This 'clears' the display by setting each slice to the set
 * 			clock background color
 * @Inputs	none
 * @Return	none
 */
void clearClockDisplay(void){
	for(uint8_t e=0; e<60; e++){
		displaySlice[e][0]=backgroundR;
		displaySlice[e][1]=backgroundG;
		displaySlice[e][2]=backgroundB;
	}
}

/*
 * @Summary	Draws a full frame which is 1 full motor rotation which
 * 			contains 60 slices which are the equivalent of pixels.
 * 			The display speeds are based on the previous motor
 * 			rotation speed.
 * @Inputs	none
 * @Return	none
 */
void drawClockFrame(void){
	uint16_t pieSlice = 0;

	motorMicroPeriod = TIM3->CNT;
	TIM3->CNT=0;
	pieSlice=(motorMicroPeriod/numberOfSlices)-microNegBuf;
	if(pieSlice > 10000){ //this got rid of a bug
		pieSlice = 600;
	}
	//draw each slice
	for(uint8_t e=0; e<numberOfSlices; e++){
		setDisplayAllOneColor(displaySlice[e][0], displaySlice[e][1], displaySlice[e][2]);
		basicClockFace(clockIndexR, clockIndexG, clockIndexB);
		if(edgeRunner){
			setOuterEdgeLED((timer11CountMarkerSec + 1000), timer11LoopCount, timer11CountMarkerSec);
		}
		updateLEDs(1);
		//wait until its time to display the next slice
		motorMicroPeriod = TIM3->CNT;
		while(motorMicroPeriod<(pieSlice*(1+e))){
			motorMicroPeriod=TIM3->CNT;
		}
	} //end for(e e<numberOfSlices e++)
}

/*
 * @Summary	sets the current position of the outer edge LED
 * @Inputs	timerCounterTotal		Timer count when the animation will end
 * 			timerCounterCurrent		Desired timer count (usually the current timer count)
 * 			timerCounterBeginning	Timer count when the animation starts
 * @Return	none
 */
void setOuterEdgeLED(uint32_t timerCounterTotal, uint32_t timerCounterCurrent, uint32_t timerCounterBeginning){
	uint32_t LEDOnTime = 0;
	uint32_t timerTotalTime = timerCounterTotal - timerCounterBeginning;
	uint8_t LEDIndex = 0;

	LEDOnTime=timerTotalTime/48;
	LEDIndex=(timerCounterCurrent - timerCounterBeginning)/LEDOnTime; //this offsets the pre-load on the counter
	setLED(1, 47-LEDIndex, 255, 5, 0, 5);
}

/*
 * @Summary	sets the LEDs for the clock face index points
 * @Inputs	dotsR	(0-255) RED RGB value
 * 			dotsG	(0-255) GREEN RGB value
 * 			dotsB	(0-255) BLUE RGB value
 * @Return	none
 */
void basicClockFace(uint8_t dotsR, uint8_t dotsG, uint8_t dotsB){
	//12
	if((testSecond != 0) && (testMinute != 0) && (testHour != 12)){
		setLED(1, 0, 255, dotsR, dotsG, dotsB);
		setLED(1, 48, 255, dotsR, dotsG, dotsB);
	}
	//11
	if((testSecond != 55) && (testMinute != 55) && (testHour != 11)){
		setLED(1, 4, 255, dotsR, dotsG, dotsB);
	}
	//10
	if((testSecond != 50) && (testMinute != 50) && (testHour != 10)){
		setLED(1, 8, 255, dotsR, dotsG, dotsB);
	}
	//9
	if((testSecond != 45) && (testMinute != 45) && (testHour != 9)){
		setLED(1, 12, 255, dotsR, dotsG, dotsB);
		setLED(1, 54, 255, dotsR, dotsG, dotsB);
	}
	//8
	if((testSecond != 40) && (testMinute != 40) && (testHour != 8)){
		setLED(1, 16, 255, dotsR, dotsG, dotsB);
	}
	//7
	if((testSecond != 35) && (testMinute != 35) && (testHour != 7)){
		setLED(1, 20, 255, dotsR, dotsG, dotsB);
	}
	//6
	if((testSecond != 30) && (testMinute != 30) && (testHour != 6)){
		setLED(1, 24, 255, dotsR, dotsG, dotsB);
		setLED(1, 60, 255, dotsR, dotsG, dotsB);
	}
	//5
	if((testSecond != 25) && (testMinute != 25) && (testHour != 5)){
		setLED(1, 28, 255, dotsR, dotsG, dotsB);
	}
	//4
	if((testSecond != 20) && (testMinute != 20) && (testHour != 4)){
		setLED(1, 32, 255, dotsR, dotsG, dotsB);
	}
	//3
	if((testSecond != 15) && (testMinute != 15) && (testHour != 3)){
		setLED(1, 36, 255, dotsR, dotsG, dotsB);
		setLED(1, 66, 255, dotsR, dotsG, dotsB);
	}
	//2
	if((testSecond != 10) && (testMinute != 10) && (testHour != 2)){
		setLED(1, 40, 255, dotsR, dotsG, dotsB);
	}
	//1
	if((testSecond != 5) && (testMinute != 5) && (testHour != 1)){
		setLED(1, 44, 255, dotsR, dotsG, dotsB);
	}
}

/*
 * @Summary	Starts the motor. This function cannot be exited until the motor
 * 			has successfully started and stable
 * @Inputs	none
 * @Return	none
 */
void motorStartSequence(void){

	/*
	 * Two types of error states
	 * 		1. Spins way too fast-imidiately, slows down to near zero, repeat
	 * 		2. Sputter spins to a stop. Stays stopped for a while until
	 * 		   it tries again.
	 *
	 * Successful spin up
	 * 		1. Successful motor start usually takes ~2 seconds for the motor to
	 * 		   be spinning quickly at a stable rate and a guarantee of no errors.
	 * 		   The motor usually spins several rotations (this number is in the
	 * 		   data sheet) clockwise somewhat slowly, then it spins up
	 * 		   counterclockwise (the intended direction) rapidly up to speed.
	 *
	 * I can use the optical sensor to tell if the motor is spinning, and
	 * how fast, but I cannot tell the direction.
	 *
	 * The error states are too fast too quickly and sputter to stop. Both
	 * of these error states should be able to be detected by the speed of
	 * the spinning light mask.
	 *
	 * Here is the code plan/outline:
	 * 	1.	Check the current motor speed. Don't spin up the motor if the motor is
	 * 		spinning too rapidly.
	 * 	2. 	Start PWM with the motor startup value
	 * 	3. 	Check the motor period at 0.5s after PWM started if the speed of
	 * 		the motor is greater than the endPWMSpeed we can safely assume
	 * 		error state 1 has occurred.
	 * 	4. 	Check motor speed at 3s. If the speed is 0 or deviates from the set
	 * 		speed by too much, I should assume error 2
	 * 	5. 	If no errors are detected, it can be assumed the motor has started
	 * 		successfully.
	 */

	uint16_t revolutionCount 	= 0;
	uint8_t sensorValue 		= 0;
	uint8_t prevSensorValue 	= 0;
	uint8_t motorStartSpeed 	= 15;
	uint8_t error1Catch 		= 5;
	uint8_t motorErrorSpeed 	= 0;

	uint8_t motorStarted 		= 0;
	uint8_t errorState 			= 0;

	//--NOTE--//
	//timer11 interrupts increase the timer11LoopCount variable
	//-------//

	//see how fast the motor is currently spinning
	revolutionCount = 0;
	timer11CountMarker = timer11LoopCount;
	while(timer11LoopCount < (timer11CountMarker + 500)){
		//index point registers LOW
		sensorValue = HAL_GPIO_ReadPin(GPIOC, opticalSensor_Pin);
		if((!sensorValue) && (prevSensorValue)){
			//new trigger
			revolutionCount++;
		}
		prevSensorValue = sensorValue;
	}

	//check for motor error 1
	if(revolutionCount>error1Catch){
		errorState = 1;
	}

	//while the motor is not started
	while(!motorStarted){
		//send speed to motor if no error
		if(errorState != 1){
			TIM2->CCR1 = motorStartSpeed;
		}

		//display blue to indicate motor is starting
		setDisplayAllOneColor(0, 0, 10);
		updateLEDs(1);

		//sample sensor for 0.5s and count the revolutions
		revolutionCount = 0;
		timer11CountMarker = timer11LoopCount;
		while(timer11LoopCount < (timer11CountMarker + 500)){
			sensorValue = HAL_GPIO_ReadPin(GPIOC, opticalSensor_Pin);
			if((!sensorValue) && (prevSensorValue)){
				revolutionCount++;
				if(revolutionCount > error1Catch){
					//error 1 is detected, don't wait to set speed to 0
					timer11LoopCount = timer11CountMarker + 501;
				}
			}
			prevSensorValue = sensorValue;
		}

		//check for motor error 1
		if(revolutionCount > error1Catch){
			errorState = 1; //set error flag
			TIM2->CCR1 = motorErrorSpeed; //set motor speed to 0
			//do LED animation for 1.5s
			timer11CountMarker = timer11LoopCount;
			while(timer11LoopCount < (timer11CountMarker + 1500)){
				setDisplayAllOneColor(10, 0, 0);
				updateLEDs(1);
			}
			setDisplayAllOneColor(0, 0, 10);
			updateLEDs(1);

		}else{
			errorState = 0;
		}

		//if no error, wait for 2.0s to check for next error
		if(!errorState){
			timer11CountMarker = timer11LoopCount;
			while(timer11LoopCount < (timer11CountMarker + 2000)){
			}
		}

		//if no error - sample sensor for 0.5s and count the revolutions
		if(!errorState){
			revolutionCount = 0;
			timer11CountMarker = timer11LoopCount;
			while(timer11LoopCount < (timer11CountMarker + 500)){
				sensorValue = HAL_GPIO_ReadPin(GPIOC, opticalSensor_Pin);
				if((!sensorValue) && (prevSensorValue)){
					revolutionCount++;
				}
				prevSensorValue = sensorValue;
			}
		}

		// if no error - check for motor error 2
		if(!errorState){
			//expecting ~60 revolutions/second
			if((revolutionCount > 40) || (revolutionCount < 20)){
				errorState = 2;
				TIM2->CCR1 = motorErrorSpeed; //set motor speed to 0
				//do LED animation for 1.0s
				timer11CountMarker = timer11LoopCount;
				while(timer11LoopCount < (timer11CountMarker + 1000)){
					setDisplayAllOneColor(10, 0, 0);
					updateLEDs(1);
				}
				setDisplayAllOneColor(0, 0, 10);
			}else{
				errorState = 0;
			}
		}

		//if no errors here, motor is started and ready to go
		if(!errorState){
			motorStarted = 1;
		}
	} //end if motorStarted

	//wait 2 seconds to start things up
	//STARTING TO WONDER IF IT WOULD BE BENEFICIAL TO REMOVE THIS?
	setDisplayAllOneColor(0, 10, 0);
	updateLEDs(1);
	prevSensorValue = 0;
	motorMicroPeriod = 0;
	timer11CountMarker = timer11LoopCount;
	while(timer11LoopCount < (timer11CountMarker + 2000)){
		sensorValue = HAL_GPIO_ReadPin(GPIOC, opticalSensor_Pin);
		if((!sensorValue) && (prevSensorValue)){
			//new trigger
			motorMicroPeriod = TIM3->CNT;
			TIM3->CNT=0;
		}
	}

	//THIS IS THE END OF THE NOTE ABOVE I MAY WANT TO REMOVE THE CODE ABOVE
	//IF I REMOVE THE CODE ABOVE THERE IS A CHANCE THAT THE CODE BELOW WILL
	//NO LONGER BE NEEDED
	motorSpinup = 1; //vary motor speed for 0.5s to help algorithm
	timer11CountMarker = timer11LoopCount; //note the ms counter so we know when 0.5s has passed after returning to main loop
}

/*
 * @Summary	starts the PWM signal going to the motor. The duty cycle and the motorPWMSpeed
 * 			variable are set to 0.
 * @Inputs	none
 * @Return	none
 */
void motorInit(void){
	updateMotorSpeed = 1;
	motorPWMSpeed = 0;
	TIM2->CCR1 = motorPWMSpeed;
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
}

/*
 * @Summary	advances the seconds value each user-defined period of milliseconds
 * @Inputs	seconds period in ms
 * @Return	none
 */
void clock(uint32_t secondInterval){
	//advance every desired period
	if(timer11LoopCount > (timer11CountMarkerSec + secondInterval)){
		timer11CountMarkerSec = timer11LoopCount;
		updateDisplay = 1;
		testSecond++;
		if(testSecond > 59){
			testSecond=0;
			testMinute++;
			if(testMinute > 59){
				testMinute=0;
				testHour++;
				if(testHour > 12){
					testHour = 1;
				}
			}
		}
	}
}

/*
 * @Summary	This is the code for the timer11 interrupt. This interrupt is triggered
 * 			every 1ms. The interrupt increases the timer11LoopCount global variable.
 * @Inputs	callback function
 * @Return	none
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){
	//timer 11 is the only timer setup with interrupts
	timer11LoopCount++;
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
