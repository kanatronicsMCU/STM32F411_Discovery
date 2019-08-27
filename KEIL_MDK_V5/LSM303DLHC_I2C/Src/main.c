/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  * I2C Example, pulling method for LSM303DLHC accelerometer
  * K.Kanas 27/8/2019	
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "math.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define LSM303_ACC_ADDRESS (0x19 << 1) // accelerometer address: 0011 001x	
#define LSM303_ACC_CTRL_REG1_A 0x20    // control register address

// CTRL_REG1_A = [ODR3][ODR2][ODR1][ODR0][LPEN][ZEN][YEN][XEN]

#define LSM303_ACC_Z_ENABLE 0x07 // 0000 0100
#define LSM303_ACC_100HZ 0x50 // 0101 0000
#define LSM303_ACC_Z_H_A 0x2D // 
#define LSM303_ACC_Z_L_A 0x2C //
#define LSM303_ACC_Y_H_A 0x2B // 
#define LSM303_ACC_Y_L_A 0x2A //
#define LSM303_ACC_X_H_A 0x29 // 
#define LSM303_ACC_X_L_A 0x28 //
#define LSM303_ACC_Z_L_A_MULTI_READ (LSM303_ACC_Z_L_A | 0x80) // addres incrementation for multiple reading
#define LSM303_ACC_RESOLUTION 2.0 
#define X 0
#define Y 1
#define Z 2

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;


// control register configuration
uint8_t Settings = LSM303_ACC_Z_ENABLE | LSM303_ACC_100HZ;
uint8_t LDataXYZ [3]; // LOW  XYZ 
uint8_t HDataXYZ [3]; // HIGH XYZ 
uint16_t DataXYZ[3];  // 16 bit XYZ data
uint8_t Data[6];
float Xaxis_g = 0; // Xg
float Yaxis_g = 0; // Yg
float Zaxis_g = 0; // Zg

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
/* USER CODE BEGIN PFP */

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
  /* USER CODE BEGIN 2 */


HAL_I2C_Mem_Write(&hi2c1, LSM303_ACC_ADDRESS, LSM303_ACC_CTRL_REG1_A, 1, &Settings, 1, 100);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */
		 // Pobranie wyzszego bajtu danych osi Z
	HAL_GPIO_TogglePin(GPIOD,GPIO_PIN_13);
/*	
	HAL_I2C_Mem_Read(&hi2c1, LSM303_ACC_ADDRESS, (LSM303_ACC_X_L_A), 1, &LDataXYZ[X], 1, 100);	 //X
  HAL_I2C_Mem_Read(&hi2c1, LSM303_ACC_ADDRESS, (LSM303_ACC_X_H_A), 1, &HDataXYZ[X], 1, 100);
	DataXYZ[X]=HDataXYZ[X];
  DataXYZ[X]=(DataXYZ[X]<<8)|(LDataXYZ[X]);
		
  HAL_I2C_Mem_Read(&hi2c1, LSM303_ACC_ADDRESS, (LSM303_ACC_Y_L_A), 1, &LDataXYZ[Y], 1, 100);   //Y 
  HAL_I2C_Mem_Read(&hi2c1, LSM303_ACC_ADDRESS, (LSM303_ACC_Y_H_A), 1, &HDataXYZ[Y], 1, 100);
  DataXYZ[Y]=HDataXYZ[Y];
  DataXYZ[Y]=(DataXYZ[Y]<<8)|(LDataXYZ[Y]);	
		
  HAL_I2C_Mem_Read(&hi2c1, LSM303_ACC_ADDRESS, (LSM303_ACC_Z_L_A), 1, &LDataXYZ[Z], 1, 100);   //Z
  HAL_I2C_Mem_Read(&hi2c1, LSM303_ACC_ADDRESS, (LSM303_ACC_Z_H_A), 1, &HDataXYZ[Z], 1, 100);
  DataXYZ[2]=HDataXYZ[Z];
  DataXYZ[2]=(DataXYZ[Z]<<8)|(LDataXYZ[Z]);

		
	//Calculating G

 Xaxis_g = ((float) DataXYZ[X] * LSM303_ACC_RESOLUTION) / (float) INT16_MAX;
 Yaxis_g = ((float) DataXYZ[Y] * LSM303_ACC_RESOLUTION) / (float) INT16_MAX;
 Zaxis_g = ((float) DataXYZ[Z] * LSM303_ACC_RESOLUTION) / (float) INT16_MAX;
*/		
HAL_I2C_Mem_Read(&hi2c1, LSM303_ACC_ADDRESS, LSM303_ACC_Z_L_A_MULTI_READ, 1, Data, 6, 100);

DataXYZ[X]=  ((Data[1] << 8) | Data[0]);
DataXYZ[Y] = ((Data[3] << 8) | Data[2]);
DataXYZ[Z] = ((Data[5] << 8) | Data[4]);

Xaxis_g = ((float) DataXYZ[X] * LSM303_ACC_RESOLUTION) / (float) INT16_MAX;
Yaxis_g = ((float) DataXYZ[Y] * LSM303_ACC_RESOLUTION) / (float) INT16_MAX;
Zaxis_g = ((float) DataXYZ[Z] * LSM303_ACC_RESOLUTION) / (float) INT16_MAX;

	HAL_Delay(80);
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
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 192;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 8;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
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
  hi2c1.Init.ClockSpeed = 400000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 50;
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
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(Orange_LED_GPIO_Port, Orange_LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : Orange_LED_Pin */
  GPIO_InitStruct.Pin = Orange_LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(Orange_LED_GPIO_Port, &GPIO_InitStruct);

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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
