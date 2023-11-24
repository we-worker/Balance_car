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
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"
#include "oled.h"
#include "mpu6050.h"
#include "stdio.h"
#include "Motor.h"
#include "PID.h"

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

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
float MotorSpeed1;
float MotorSpeed2;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
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
  MX_I2C2_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_USART1_UART_Init();
  MX_TIM5_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_Base_Start_IT(&htim5);
  OLED_Init();
  OLED_CLS();

  while (w_mpu_init() != mpu_ok)
  {
  }
  dmp_init();

  HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_1); // 开启编码器A
  HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_2); // 开启编码器B
  HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_1); // 开启编码器A
  HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_2); // 开启编码器B

  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3); // 开启PWM-CH3
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_4); // 开启PWM-CH4

  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_RESET);
  OLED_ShowStr(24, 5, "hello", 1);

  float kp = 160, kd = 40, velocity_KP = 100, velocity_KI = 0.05;
  // float kp=160,kd=40,velocity_KP=-100,velocity_KI=-1;

  int pwm_val = 4000;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

    /*
    //闪烁的led灯
    HAL_GPIO_WritePin(LED1_GPIO_Port,LED1_Pin,GPIO_PIN_SET);
    HAL_Delay(1000);
    HAL_GPIO_WritePin(LED1_GPIO_Port,LED1_Pin,GPIO_PIN_RESET);
    HAL_Delay(1000);

    */

    /*
    MotorDriver_SetPWMDuty(1,pwm_val);
    MotorDriver_SetPWMDuty(2,pwm_val);

    //读取电机转动速度
    char strx[20],stry[20],strz[20];
    sprintf(strx, "right== %.3f  ", MotorSpeed2);
    OLED_ShowStr(24,1,strx,1);
    sprintf(strx, "left== %.3f   ", MotorSpeed1);
    OLED_ShowStr(24,2,strx,1);
    sprintf(strz, "pwm== %d   ", pwm_val);
    OLED_ShowStr(24,3,strz,1);

    */

    w_mpu_read_all_raw_data(&mpu_raw_msg);
    read_dmp(&mpu_pose_msg);

    int Balance_Pwm;
    int target = 3000;
    int velocity_pwm;
    Balance_Pwm = balance(mpu_pose_msg.pitch, mpu_raw_msg.mpu_gyro[1], kp, kd);
    velocity_pwm = velocity(MotorSpeed1, MotorSpeed2, velocity_KP, velocity_KI);

    if (Balance_Pwm > 0)
      target = 3000 + Balance_Pwm;
    else
      target = -3000 + Balance_Pwm;
    if (velocity_pwm > 0)
      target = target + velocity_pwm;
    else
      target = target + velocity_pwm;

    MotorDriver_SetPWMDuty(1, -target);
    MotorDriver_SetPWMDuty(2, -target);

    /*
  //读取mpu所有原始数据
    w_mpu_read_all_raw_data(&mpu_raw_msg);

    char strx[20],stry[20],strz[20];
    sprintf(strx, "gx== %.3f", mpu_raw_msg.mpu_gyro[0]);
    sprintf(stry, "gy== %.3f", mpu_raw_msg.mpu_gyro[1]);
    sprintf(strz, "gz== %.3f", mpu_raw_msg.mpu_gyro[2]);
    OLED_ShowStr(24,1,strx,1);
    OLED_ShowStr(24,2,stry,1);
    OLED_ShowStr(24,3,strz,1);

    //读取mpu姿态
    read_dmp(&mpu_pose_msg);
    //char strx[20],stry[20],strz[20];
    sprintf(strx, "pitch== %.3f", mpu_pose_msg.pitch);
    sprintf(stry, "roll== %.3f", mpu_pose_msg.roll);
    sprintf(strz, "yaw== %.3f", mpu_pose_msg.yaw);
    OLED_ShowStr(24,4,strx,1);
    OLED_ShowStr(24,5,stry,1);
    OLED_ShowStr(24,6,strz,1);
    */
  }
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
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  // HAL_GPIO_TogglePin(LED1_GPIO_Port,LED1_Pin);
  if (htim == (&htim5))
  {
    // 1.获取电机速度
    MotorSpeed1 = (float)(__HAL_TIM_GET_COUNTER(&htim3));
    // TIM4计数器获得电机脉冲，该电机在10ms采样的脉冲/18则为实际转速的rpm
    __HAL_TIM_SET_COUNTER(&htim3, 0); // 计数器清零

    if (MotorSpeed1 > 55000)
    {
      MotorSpeed1 -= 65536;
    }
    MotorSpeed1 = MotorSpeed1 / 10.0 * 100;
    // 2.获取电机速度
    MotorSpeed2 = (float)(__HAL_TIM_GET_COUNTER(&htim2));
    // TIM4计数器获得电机脉冲，该电机在10ms采样的脉冲/18则为实际转速的rpm
    __HAL_TIM_SET_COUNTER(&htim2, 0); // 计数器清零

    if (MotorSpeed2 > 55000)
    {
      MotorSpeed2 -= 65536;
    }
    MotorSpeed2 = MotorSpeed2 / 10.0 * 100;
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
