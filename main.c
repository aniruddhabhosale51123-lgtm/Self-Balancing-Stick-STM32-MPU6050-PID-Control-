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
#include <math.h>
#include <string.h>
#include <stdio.h>

#define MPU6050_ADDR  (0x68 << 1)
//#define MPU6050_ADDR  0xD0
#define ACCL_START 0x3B


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

TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM3_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
int __io_putchar(int);
int   wakeup_mpu(void);
int read_raw_data(void);
void PID_calculation();

void Motor_Forward();
void Motor_Backward();


int __io_putchar(int ch){
	HAL_UART_Transmit(&huart1, (uint8_t*) &ch, 1, HAL_MAX_DELAY);
	return ch;

}
int _write(int file, char *ptr, int len)
{
 (void)file;
 int DataIdx;

 for (DataIdx = 0; DataIdx < len; DataIdx++)
 {
   ITM_SendChar(*ptr++);
 }
 return len;
}



// old:
// int8_t raw_data[14];

// new:

int16_t  AccX =0, AccY =0, AccZ =0, Temp =0, GyroX =0, GyroY =0, GyroZ =0;

float angle = 0.0f;
float gyro_angle = 0.0f;
uint32_t prev_time = 0;
// after wakeup_mpu()

float dt = 0;
// Gyro integrated angles
float gyro_angle_x = 0.0f;
float gyro_angle_y = 0.0f;

// Final angles after filtering
float pitch = 0.0f;
float roll  = 0.0f;
// pid calculation variable

float Kp = 15;  // proportional
float Ki = 0.0;   //integral  remove teady error
float Kd = 0.8;   // derivative


float setpoint = 0.0f;      // target balancing angle which we want to stand
float error, prev_error = 0.0f;  // error is difference betwn actuall angle and set angle
float error_sum = 0.0f;    //

// wake up function of mpuz6050
int   wakeup_mpu(void){
	uint8_t wake = 0x00;  //making variable which contain 00 hexvalue
	uint8_t Data;




	HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, 0x6B, 1, &wake, 1, 100);
	HAL_Delay(100);

	// in wake up we need to assign 0x00 to (0x6B wake up register) so we use write i2c  mem
	//HAL_I2C_Mem_Write(hi2c, DevAddress, MemAddress, MemAddSize, pData, Size, Timeout)

//
//	Data = 0x07;
//	HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, 0x19, 1, &Data, 1, 1000);
//
//	 //Set Gyroscopic configuration in GYRO_CONFIG Register
//			Data = 0x00;  // XG_ST=0,YG_ST=0,ZG_ST=0, FS_SEL=0 -> ± 250 ̐/s
//			HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, 0x1B, 1, &Data, 1, 1000);
//
//	// Set accelerometer configuration in ACCEL_CONFIG Register
//			Data = 0x00;  // XA_ST=0,YA_ST=0,ZA_ST=0, FS_SEL=0 -> ± 2g
//			HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, 0x1C, 1, &Data, 1, 1000);
//


	    Data = 0x07;
	    HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, 0x19, 1, &Data, 1, 100);
	    // Enable DLPF = 42 Hz bandwidth
	    Data = 0x03;
	    HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, 0x1A, 1, &Data, 1, 100);
	    // ±250 deg/s
	    Data = 0x00;
	    HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, 0x1B, 1, &Data, 1, 100);
	    // ±2g
	    Data = 0x00;
	    HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, 0x1C, 1, &Data, 1, 100);

	    return 0;
}

// to read values of sensor

int16_t Accel_X_RAW = 0;
int16_t Accel_Y_RAW = 0;
int16_t Accel_Z_RAW = 0;

int16_t Gyro_X_RAW = 0;
int16_t Gyro_Y_RAW = 0;
int16_t Gyro_Z_RAW = 0;

float Ax, Ay, Az, Gx, Gy, Gz;


void MPU6050_Read_Accel (void)
{
	uint8_t Rec_Data[6];

	// Read 6 BYTES of data starting from ACCEL_XOUT_H (0x3B) register
	HAL_I2C_Mem_Read (&hi2c1, MPU6050_ADDR, 0x3B, 1, Rec_Data, 6, HAL_MAX_DELAY);

	Accel_X_RAW = (int16_t)(Rec_Data[0] << 8 | Rec_Data [1]);
	Accel_Y_RAW = (int16_t)(Rec_Data[2] << 8 | Rec_Data [3]);
	Accel_Z_RAW = (int16_t)(Rec_Data[4] << 8 | Rec_Data [5]);

	/*** convert the RAW values into acceleration in 'g'
	     we have to divide according to the Full scale value set in FS_SEL
	    configured FS_SEL = 0. So  dividing by 16384.0
	     for more details check ACCEL_CONFIG Register              ****/

	Ax = (float)Accel_X_RAW/16384.0;
	Ay = (float)Accel_Y_RAW/16384.0;
	Az = (float)Accel_Z_RAW/16384.0;
}


void MPU6050_Read_Gyro (void)
{
	uint8_t Rec_Data[6];

	// Read 6 BYTES of data starting from GYRO_XOUT_H register
	HAL_I2C_Mem_Read (&hi2c1, MPU6050_ADDR, 0x43, 1, Rec_Data, 6, HAL_MAX_DELAY);

	Gyro_X_RAW = (int16_t)(Rec_Data[0] << 8 | Rec_Data [1]);
	Gyro_Y_RAW = (int16_t)(Rec_Data[2] << 8 | Rec_Data [3]);
	Gyro_Z_RAW = (int16_t)(Rec_Data[4] << 8 | Rec_Data [5]);

	/*** convert the RAW values into dps (ｰ/s)
	     we have to divide according to the Full scale value set in FS_SEL
	     configured FS_SEL = 0. So  dividing by 131.0
	     for more details check GYRO_CONFIG Register   /           ****/

	Gx = (float)Gyro_X_RAW/131.0;
	Gy = (float)Gyro_Y_RAW/131.0;
	Gz = (float)Gyro_Z_RAW/131.0;
}




void Motor_SetSpeed(int speed)
{
    if (speed > 999) speed = 999;
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_RESET);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, speed);
}


void Motor_Backward(int speed)
{
    if (speed > 999) speed = 999;
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_SET);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, speed);
}


// pid caluculation func
void PID_calculation()
{
    // 1) Get filtered angle from complementary filter
    float current_angle = pitch;

    // 2) Compute error
    error = setpoint - current_angle;

    // 3) Integral term
    error_sum += error * dt;

    // anti wind-up
    if(error_sum > 200) error_sum = 200;
    if(error_sum < -200) error_sum = -200;

    // 4) Derivative term
    float error_diff = (error - prev_error) / dt;

    // 5) PID output
    float PID =  (Kp * error)+ (Ki * error_sum) + (Kd * error_diff);

    prev_error = error;

    // 6) Limit output
    if(PID > 1000) PID = 1000;
    if(PID < -1000) PID = -1000;

    // 7) Send to motors
    if(PID > 0)
    {
//        Motor_Forward(PID); // tilt forward
    	Motor_SetSpeed(PID);

    }
    else
    {
        Motor_Backward(-PID);   // tilt backward
    }
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
  MX_TIM3_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
  // int i;
  HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_1);
  wakeup_mpu();

  prev_time = HAL_GetTick();   // avoid huge dt on first loop

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

	  // read the Accelerometer and Gyro values
	  MPU6050_Read_Accel();
	  MPU6050_Read_Gyro();
//	  Gx = (float)Gyro_X_RAW/131.0;
//	  Gy = (float)Gyro_Y_RAW/131.0;
//	  Gz = (float)Gyro_Z_RAW/131.0;

         //  atan formula
	    // Convert accelerometer values to angle (in degrees) ( not in m/sec^2)
	  	float acc_pitch = atan2f(Ay, Az) * 180.0f / M_PI; // for pittch
	  	float acc_roll = atan2f(-Ax, sqrtf(Ay*Ay + Az*Az)) * 180.0f / M_PI;   // for roll



// to measure passing time  per next value we get

	    uint32_t now = HAL_GetTick(); //hal tick give  time in milliseconds
	    float raw_dt = (now - prev_time) / 1000.0f;  // convert ms to seconds and store in raw dt
	    prev_time = now;           // assign now time to check difference between naxt time
	    if (raw_dt <= 0.0f) raw_dt = 0.001f;
	    if (raw_dt > 0.05f) raw_dt = 0.05f;   // max 50 ms for safety
	    dt = raw_dt;




	 	 // ***step for gyro integration to estimate angle
	 	 // gx, gy are gyro rotation speeds (deg/sec)
	 	 // dt is loop time (seconds)
	 	 // Multiplying gx*dt gives how many degrees the board rotated in this loop

	 	 gyro_angle_x += Gx * dt;   // Pitch angle from gyro
	 	 gyro_angle_y += Gy * dt;   // Roll angle from gyro

	    // implimenting complimentary filter
        // complementery filter is mixing two sensor data to get a stable output or a fusion of sensor ,here we mix 98% gyro & 2%acc
	    // gyro has lowest noise but value automatically change or increase with time called as drift
	    // acc is used less 2% becaus has more noise if use only acc makes unstable value
	    // but  acc give fast value for long term usefull to take some refrence
	 	float alpha = 0.98f; // gyro usage

	 	//pitch = 0.98 * gyro +(1.0f - alpha)= 0.02 * accelerometer deg value

	 	pitch = alpha * (pitch + Gx * dt) + (1.0f - alpha) * acc_pitch;
	 	roll  = alpha * (roll  + Gy * dt) + (1.0f - alpha) * acc_roll;

       //now pid



	 	PID_calculation();

            //  printf ( "Ax=%.2fg ", Ax); // @suppress("Float formatting support")
            //HAL_Delay(100);


	 	  	  /*
	 	  	  for(i = 0; i < 100; i++) {
	 	  	         __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, i);  // Set PWM duty cycle (0 to 299)
	 	  	         HAL_Delay(100);  // Wait 5 milliseconds between steps (smooth increase)
	 	  	     }


	 	  	     HAL_Delay(10);  // Short delay before decreasing

	 	  	     // Decrease PWM duty cycle gradually
	 	  	     for(i = 100; i >= 0; i--) {
	 	  	         __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, i);  // Set PWM duty cycle (300 down to 0)
	 	  	         HAL_Delay(100);  // Wait 5 milliseconds between steps (smooth decrease)
	 	  	     }

	 	  	     */
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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
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
  htim3.Init.Prescaler = 72;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 999;
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
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

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
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

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
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2, GPIO_PIN_RESET);

  /*Configure GPIO pins : PA0 PA1 PA2 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
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
