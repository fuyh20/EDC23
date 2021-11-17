/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "dma.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"
#include "jy62.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#define PID_MAX 900
#define PID_MIN 0
float INTEGRAL[4] = {0, 0, 0, 0}, pre_err[4] = {0, 0, 0, 0};
const float Kp[4] = {40, 40, 40, 40}, Ki[4] = {0.2, 0.2, 0.2, 0.2}, Kd[4] = {100, 100, 100, 100};
float target[4] = {10, 10, 10, 10};
float pid[4] = {0, 0, 0, 0};

float PID(float in, float target, int i) //位移式PID
{
  float err = target - in;
  INTEGRAL[i] += err * 20;
  if (INTEGRAL[i] > 8000)
    INTEGRAL[i] = 8000;
  float derr = (err - pre_err[i]) / 20;
  pre_err[i] = err;
  pid[i] = (Kp[i] * err) + (Ki[i] * INTEGRAL[i]) + (Kd[i] * derr);
  float output;
  if (pid[i] < PID_MIN)
    output = PID_MIN;
  else if (pid[i] > PID_MAX)
    output = PID_MAX;
  else
    output = pid[i];
  return output;
}
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
uint8_t u2_RX_Buf[MAX_LEN];
uint8_t u2_RX_ReceiveBit;
int rx_len = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
float angle0 = 0;
void Calibrate_Angle()
{
  float temp = GetYaw();
  angle0 = temp / 20;
  if (temp < 60)
    for (int i = 1; i < 20; i++)
    {
      if ((temp = GetYaw()) > 300)
        temp -= 360;
      angle0 += temp / 20;
    }
  else if (temp > 300)
    for (int i = 1; i < 20; i++)
    {
      if ((temp = GetYaw()) < 60)
        temp += 360;
      angle0 += temp / 20;
    }
  else
    for (int i = 1; i < 20; i++)
      angle0 += GetYaw() / 20;
}

float getAngle()
{
  float temp = GetYaw() - angle0;
  return temp > 180 ? temp - 360 : temp < -180 ? temp + 360
                                               : temp;
}

void Set_Left_Direction(int direction)
{
  HAL_GPIO_WritePin(IN1A_GPIO_Port, IN1A_Pin, RESET);
  HAL_GPIO_WritePin(IN1B_GPIO_Port, IN1B_Pin, RESET);
  HAL_GPIO_WritePin(IN2A_GPIO_Port, IN2A_Pin, RESET);
  HAL_GPIO_WritePin(IN2B_GPIO_Port, IN2B_Pin, RESET);
  if (direction == 1)
  {
    HAL_GPIO_WritePin(IN1A_GPIO_Port, IN1A_Pin, SET);
    HAL_GPIO_WritePin(IN2B_GPIO_Port, IN2B_Pin, SET);
  }
  else if (direction == -1)
  {
    HAL_GPIO_WritePin(IN1B_GPIO_Port, IN1B_Pin, SET);
    HAL_GPIO_WritePin(IN2A_GPIO_Port, IN2A_Pin, SET);
  }
}

void Set_Right_Direction(int direction)
{
  HAL_GPIO_WritePin(IN3A_GPIO_Port, IN3A_Pin, RESET);
  HAL_GPIO_WritePin(IN3B_GPIO_Port, IN3B_Pin, RESET);
  HAL_GPIO_WritePin(IN4A_GPIO_Port, IN4A_Pin, RESET);
  HAL_GPIO_WritePin(IN4B_GPIO_Port, IN4B_Pin, RESET);
  if (direction == 1)
  {
    HAL_GPIO_WritePin(IN3A_GPIO_Port, IN3A_Pin, SET);
    HAL_GPIO_WritePin(IN4B_GPIO_Port, IN4B_Pin, SET);
  }
  else if (direction == -1)
  {
    HAL_GPIO_WritePin(IN3B_GPIO_Port, IN3B_Pin, SET);
    HAL_GPIO_WritePin(IN4A_GPIO_Port, IN4A_Pin, SET);
  }
}

void Set_Speed(float speed, int i) { target[i] = speed; }

void Turn(float angle)
{
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if (htim->Instance == TIM2)
  {
    int cnt0 = __HAL_TIM_GetCounter(&htim3);
    int cnt1 = __HAL_TIM_GetCounter(&htim4);
    int cnt2 = __HAL_TIM_GetCounter(&htim5);
    int cnt3 = __HAL_TIM_GetCounter(&htim8);

    __HAL_TIM_SetCounter(&htim3, 0);
    __HAL_TIM_SetCounter(&htim4, 0);
    __HAL_TIM_SetCounter(&htim5, 0);
    __HAL_TIM_SetCounter(&htim8, 0);

    float speed0 = (float)cnt0 * 20 / 532 * 20.4;
    float speed1 = (float)cnt1 * 20 / 531 * 20.4;
    float speed2 = (float)cnt2 * 20 / 531 * 20.4;
    float speed3 = (float)cnt3 * 20 / 530 * 20.4;

    int pwm0 = (int)PID(speed0, target[0], 0);
    int pwm1 = (int)PID(speed1, target[1], 1);
    int pwm2 = (int)PID(speed2, target[2], 2);
    int pwm3 = (int)PID(speed3, target[3], 3);

    __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_1, pwm0);
    __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_2, pwm1);
    __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_3, pwm2);
    __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_4, pwm3);

    HAL_GPIO_WritePin(IN1A_GPIO_Port, IN1A_Pin, SET);
    HAL_GPIO_WritePin(IN2B_GPIO_Port, IN2B_Pin, SET);
    HAL_GPIO_WritePin(IN3A_GPIO_Port, IN3A_Pin, SET);
    HAL_GPIO_WritePin(IN4B_GPIO_Port, IN4B_Pin, SET);
  }
}

int flag;
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  flag = 1;
  if (huart == &huart2)
  {
    jy62MessageRecord();
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
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_TIM5_Init();
  MX_TIM8_Init();
  MX_USART3_UART_Init();
  MX_DMA_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
  jy62_Init(&huart2);
  HAL_TIM_Base_Start_IT(&htim2);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);
  HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);
  HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL);
  HAL_TIM_Encoder_Start(&htim5, TIM_CHANNEL_ALL);
  HAL_TIM_Encoder_Start(&htim8, TIM_CHANNEL_ALL);
  HAL_Delay(100);
  Calibrate_Angle();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    float angle = getAngle();
    HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
    u3_printf("%d,%f,%f\n",flag, angle, angle0);
    HAL_Delay(100);
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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
