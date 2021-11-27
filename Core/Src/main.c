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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <math.h>
#include "jy62.h"
#include "zigbee.h"
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
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
#define Turn_Angle_Bias 10
#define PID_MAX 900
#define PID_MIN 0
#define GET_PACKAGE 0
#define PUT_PACKAGE 1
#define SET_BEACON 2

float packageX[2], packageY[2];
/*PID参数----------------------------------------------------------------------*/
float INTEGRAL[4] = {0, 0, 0, 0}, pre_err[4] = {0, 0, 0, 0};
const float Kp[4] = {40, 40, 40, 40}, Ki[4] = {0.2, 0.2, 0.2, 0.2}, Kd[4] = {100, 100, 100, 100};
float pid[4] = {0, 0, 0, 0};
float target[4] = {20, 20, 20, 20};

float distance;

uint16_t BeaconDir[3][2]; // 信标坐标
uint8_t STATE;

float angle0;
float absoluteAngle;

struct PARK
{
  uint16_t x, y, mineType;
};

struct PARK Park[8] = {{15, 15, 0}, {127, 15, 0}, {239, 15, 0}, {239, 127, 0}, {239, 239, 0}, {127, 239, 0}, {15, 239, 0}, {15, 127, 0}};
void clearDistance()
{
  distance = 0;
}

float getDistance()
{
  return distance;
}

float PID(float in, float target, int i)
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
  absoluteAngle = angle0;
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
  else
  {
    HAL_GPIO_WritePin(IN1A_GPIO_Port, IN1A_Pin, SET);
    HAL_GPIO_WritePin(IN1B_GPIO_Port, IN1B_Pin, SET);
    HAL_GPIO_WritePin(IN2A_GPIO_Port, IN2A_Pin, SET);
    HAL_GPIO_WritePin(IN2B_GPIO_Port, IN2B_Pin, SET);
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
  else
  {
    HAL_GPIO_WritePin(IN1A_GPIO_Port, IN3A_Pin, SET);
    HAL_GPIO_WritePin(IN1B_GPIO_Port, IN3B_Pin, SET);
    HAL_GPIO_WritePin(IN2A_GPIO_Port, IN4A_Pin, SET);
    HAL_GPIO_WritePin(IN2B_GPIO_Port, IN4B_Pin, SET);
  }
}

void Set_Speed(float speed, int i) { target[i] = speed > 0 ? speed : 0; }

float Add_Angle(float angleA, float angleB)
{
  float result = angleA + angleB;
  return result < -180 ? result + 360 : result > 180 ? result - 360
                                                     : result;
}

void Turn(float angle)
{
  Set_Right_Direction(0);
  Set_Left_Direction(0);
  float v;
  angle0 = GetYaw();
  HAL_Delay(500);
  angle0 = Add_Angle(angle0, angle);
  if (getAngle() < 0)
  {
    Set_Left_Direction(-1);
    Set_Right_Direction(1);
    for (int i = 0; i < 4; i++)
      Set_Speed(6, i);
    while (getAngle() < -50)
      ;
    while (getAngle() < -5)
    {
      v = 3;
      for (int i = 0; i < 4; i++)
        Set_Speed(v, i);
    }
  }
  else
  {
    Set_Left_Direction(1);
    Set_Right_Direction(-1);
    for (int i = 0; i < 4; i++)
      Set_Speed(6, i);
    while (getAngle() > 50)
      ;
    while (getAngle() > 5)
    {
      v = 3;
      for (int i = 0; i < 4; i++)
        Set_Speed(v, i);
    }
  }
  Set_Left_Direction(0);
  Set_Right_Direction(0);
  HAL_Delay(500);
}

// void Turn(float angle)
// {
//   Set_Right_Direction(0);
//   Set_Left_Direction(0);
//   HAL_Delay(500);
//   angle0 = Add_Angle(angle0, angle);
//   if (angle > 0)
//   {
//     Set_Left_Direction(-1);
//     Set_Right_Direction(1);
//     for (int i = 0; i < 4; i++)
//       Set_Speed(15, i);
//     while (getAngle() < -Turn_Angle_Bias)
//       ;
//   }
//   else
//   {
//     Set_Left_Direction(1);
//     Set_Right_Direction(-1);
//     for (int i = 0; i < 4; i++)
//       Set_Speed(15, i);
//     while (getAngle() > Turn_Angle_Bias)
//       ;
//   }
//   Set_Right_Direction(0);
//   Set_Left_Direction(0);
// }

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if (htim->Instance == TIM2)
  {
    unsigned int cnt0 = __HAL_TIM_GetCounter(&htim3);
    unsigned int cnt1 = __HAL_TIM_GetCounter(&htim4);
    unsigned int cnt2 = __HAL_TIM_GetCounter(&htim5);
    unsigned int cnt3 = __HAL_TIM_GetCounter(&htim8);

    if (cnt0 > 32676)
      cnt0 = 65535 - cnt0;
    if (cnt1 > 32676)
      cnt1 = 65535 - cnt1;
    if (cnt2 > 32676)
      cnt2 = 65535 - cnt2;
    if (cnt3 > 32676)
      cnt3 = 65535 - cnt3;

    __HAL_TIM_SetCounter(&htim3, 0);
    __HAL_TIM_SetCounter(&htim4, 0);
    __HAL_TIM_SetCounter(&htim5, 0);
    __HAL_TIM_SetCounter(&htim8, 0);

    float speed0 = (float)cnt0 * 20 / 532 * 20.4;
    float speed1 = (float)cnt1 * 20 / 531 * 20.4;
    float speed2 = (float)cnt2 * 20 / 531 * 20.4;
    float speed3 = (float)cnt3 * 20 / 530 * 20.4;

    distance += (speed0 + speed1 + speed2 + speed3) * 0.0125;

    int pwm0 = (int)PID(speed0, target[0], 0);
    int pwm1 = (int)PID(speed1, target[1], 1);
    int pwm2 = (int)PID(speed2, target[2], 2);
    int pwm3 = (int)PID(speed3, target[3], 3);

    __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_1, pwm0);
    __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_2, pwm1);
    __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_3, pwm2);
    __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_4, pwm3);
  }
}

uint16_t getCarPosX2()
{
  uint16_t now_X, BeaDis[3];
  for (int i = 0; i < 3; i++)
    BeaDis[i] = getDistanceOfMyBeacon(i);
  float k = 1.0 * (BeaDis[0] - BeaDis[1] + BeaconDir[1][1] * BeaconDir[1][1] - BeaconDir[0][1] * BeaconDir[0][1] + BeaconDir[1][0] * BeaconDir[1][0] - BeaconDir[0][0] * BeaconDir[0][0]) / 2.0 / (BeaconDir[1][1] - BeaconDir[0][1]);
  float b = 1.0 * (BeaconDir[1][0] - BeaconDir[0][0]) / (BeaconDir[1][1] - BeaconDir[0][1]);
  float delta = 1.0 * (2 * b * BeaconDir[0][1] - 2 * k * b - 2 * BeaconDir[0][0]) * (2 * b * BeaconDir[0][1] - 2 * k * b - 2 * BeaconDir[0][0]) - 4 * (1 + b * b) * (BeaconDir[0][0] * BeaconDir[0][0] + k * k + BeaconDir[0][1] * BeaconDir[0][1] - 2 * k * BeaconDir[0][0] - BeaDis[0]);
  now_X = 1.0 * (-(2 * b * BeaconDir[0][1] - 2 * k * b - 2 * BeaconDir[0][0]) + sqrt(delta)) / 2.0 / (1 + b * b);
  return now_X;
}

uint16_t getCarPosY2()
{
  uint16_t now_X, BeaDis[3];
  for (int i = 0; i < 3; i++)
    BeaDis[i] = getDistanceOfMyBeacon(i);
  float k = 1.0 * (BeaDis[0] - BeaDis[1] + BeaconDir[1][1] * BeaconDir[1][1] - BeaconDir[0][1] * BeaconDir[0][1] + BeaconDir[1][0] * BeaconDir[1][0] - BeaconDir[0][0] * BeaconDir[0][0]) / 2.0 / (BeaconDir[1][1] - BeaconDir[0][1]);
  float b = 1.0 * (BeaconDir[1][0] - BeaconDir[0][0]) / (BeaconDir[1][1] - BeaconDir[0][1]);
  float delta = 1.0 * (2 * b * BeaconDir[0][1] - 2 * k * b - 2 * BeaconDir[0][0]) * (2 * b * BeaconDir[0][1] - 2 * k * b - 2 * BeaconDir[0][0]) - 4 * (1 + b * b) * (BeaconDir[0][0] * BeaconDir[0][0] + k * k + BeaconDir[0][1] * BeaconDir[0][1] - 2 * k * BeaconDir[0][0] - BeaDis[0]);
  now_X = 1.0 * (-(2 * b * BeaconDir[0][1] - 2 * k * b - 2 * BeaconDir[0][0]) + sqrt(delta)) / 2.0 / (1 + b * b);
  return k - b * now_X;
}

uint16_t userGetCarPosX()
{
  if (getCarTask() == 0)
    return getCarPosX();
  else
    return getCarPosX2();
}

uint16_t userGetCarPosY()
{
  if (getCarTask() == 0)
    return getCarPosY();
  else
    return getCarPosY2();
}

// void MoveTo(float x, float y)
// {
//   float TurningCoefficient = 1; //调参
//   uint16_t errX = x - getCarPosX();
//   uint16_t errY = y - getCarPosY();
//   float targetAngle = (float)atan2(errY, errX) * 57.296, errAngle, velosity = 20;
//   targetAngle = Add_Angle(targetAngle, absoluteAngle - angle0);
//   Turn(targetAngle);
//   Set_Left_Direction(1);
//   Set_Right_Direction(1);
//   for (int i = 0; i < 4; i++)
//     Set_Speed(10, i);
//   while (1)
//   {
//     if (x * x + y * y < 25) //调参
//       break;
//     if (x * x + y * y < 200) //调参
//       velosity = 5;          //调参
//     errX = x - getCarPosX();
//     errY = y - getCarPosY();
//     errAngle = Add_Angle(atan2(errY, errX), absoluteAngle - GetYaw());
//     Set_Speed(velosity + TurningCoefficient * errAngle, 0);
//     Set_Speed(velosity + TurningCoefficient * errAngle, 1);
//     Set_Speed(velosity - TurningCoefficient * errAngle, 2);
//     Set_Speed(velosity - TurningCoefficient * errAngle, 3);
//   }
//   Set_Left_Direction(0);
//   Set_Right_Direction(0);
//   angle0 = GetYaw();
// }

void MoveTo(float x, float y)
{
  HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
  float velocity;
  float errX = x - userGetCarPosX();
  float errY = y - userGetCarPosY();
  float targetAngle = 0, difference = sqrt(errX * errX + errY * errY);
  clearDistance();
  while (difference > 5)
  {
    if (difference < 10)
      velocity = 5;
    else if (difference < 30)
      velocity = 10;
    else if (difference < 50)
      velocity = 15;
    else if (difference < 100)
      velocity = 20;
    else
      velocity = 25;
    targetAngle = Add_Angle(atan2(errX, errY) * 57.296, absoluteAngle - angle0);
    Turn(targetAngle);
    clearDistance();
    Set_Left_Direction(1);
    Set_Right_Direction(1);
    for (int i = 0; i < 4; i++)
      Set_Speed(velocity, i);
    while (getDistance() < 0.8 * difference)
      HAL_Delay(20);
    Set_Left_Direction(0);
    Set_Right_Direction(0);
    HAL_Delay(1000);
    errX = x - userGetCarPosX();
    errY = y - userGetCarPosY();
    difference = sqrt(errX * errX + errY * errY);
  }
}

void Init()
{
  while (getGameState() == 0)
    ;
  HAL_Delay(100);
  MoveTo(127, 127);
}

void Get_Survey_Data(float *x, float *y, float *I)
{
  *x = userGetCarPosX();
  *y = userGetCarPosY();
  I[0] = getMineIntensity(0);
  I[1] = getMineIntensity(1);
}

void Calculate_Package_Position(void)
{
  float x[3], y[3];
  float I[3][2];
  float A[2], B[2], C[2][2], invdet;
  Get_Survey_Data(&x[0], &y[0], I[0]);
  do
  {
    Get_Survey_Data(&x[0], &y[0], I[0]);
  } while (x[0] < 0 || y[0] < 0);
  // u2_printf("%d, %d, %d, %d\n", x[0], y[0], I[0][0], I[0][1]);
  Set_Speed(5, 0);
  Set_Speed(5, 1);
  Set_Speed(10, 2);
  Set_Speed(10, 3);
  Set_Left_Direction(-1);
  Set_Right_Direction(1);
  do
  {
    HAL_Delay(100);
    Get_Survey_Data(&x[1], &y[1], I[1]);
    if (x[1] < 0 || y[1] < 0)
      continue;
  } while (((int)x[1] - (int)x[0]) == 0 && ((int)y[1] - (int)y[0]) == 0);
  do
  {
    HAL_Delay(100);
    Get_Survey_Data(&x[2], &y[2], I[2]);
    if (x[2] < 0 || y[2] < 0)
      continue;
  } while (((int)x[1] - (int)x[0]) * ((int)y[2] - (int)y[0]) - ((int)x[2] - (int)x[0]) * ((int)y[1] - (int)y[0]) == 0);
  A[0] = (x[1] - x[0]);
  A[1] = (x[2] - x[1]);
  B[0] = (y[1] - y[0]);
  B[1] = (y[2] - y[1]);
  C[0][0] = 2.0e9 * (I[1][0] - I[0][0]) / (I[1][0] + I[0][0]) / (I[1][0] + I[0][0]) + (x[1] + x[0]) * (x[1] - x[0]) / 2 + (y[1] + y[0]) * (y[1] - y[0]) / 2;
  C[1][0] = 2.0e9 * (I[2][0] - I[1][0]) / (I[2][0] + I[1][0]) / (I[2][0] + I[1][0]) + (x[2] + x[1]) * (x[2] - x[1]) / 2 + (y[2] + y[1]) * (y[2] - y[1]) / 2;
  C[0][1] = 2.0e9 * (I[1][1] - I[0][1]) / (I[1][1] + I[0][1]) / (I[1][1] + I[0][1]) + (x[1] + x[0]) * (x[1] - x[0]) / 2 + (y[1] + y[0]) * (y[1] - y[0]) / 2;
  C[1][1] = 2.0e9 * (I[2][1] - I[1][1]) / (I[2][1] + I[1][1]) / (I[2][1] + I[1][1]) + (x[2] + x[1]) * (x[2] - x[1]) / 2 + (y[2] + y[1]) * (y[2] - y[1]) / 2;
  invdet = 1.0 / (A[0] * B[1] - A[1] * B[0]);
  packageX[0] = (B[1] * C[0][0] - B[0] * C[1][0]) * invdet;
  packageY[0] = (A[0] * C[1][0] - A[1] * C[0][0]) * invdet;
  packageX[1] = (B[1] * C[0][1] - B[0] * C[1][1]) * invdet;
  packageY[1] = (A[0] * C[1][1] - A[1] * C[0][1]) * invdet;
}

void Set_Beacon()
{
  Set_Right_Direction(0);
  Set_Left_Direction(0);
  HAL_Delay(500);
  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, SET);
  HAL_Delay(1000);
  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, RESET);
}

uint8_t getNearestPark()
{
  uint8_t tmp = 0;
  uint16_t x = userGetCarPosX(), y = userGetCarPosY();
  float dist = 0, distMin = 0;
  for (int i = 0; i < 8; i++)
  {
    dist = (Park[i].x - x) * (Park[i].x - x) + (Park[i].y - y) * (Park[i].y - Park[i].y);
    if (dist < distMin)
    {
      distMin = dist;
      tmp = i;
    }
  }
  return tmp;
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
  MX_DMA_Init();
  MX_USART3_UART_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_Base_Start_IT(&htim2);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);
  HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);
  HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL);
  HAL_TIM_Encoder_Start(&htim5, TIM_CHANNEL_ALL);
  HAL_TIM_Encoder_Start(&htim8, TIM_CHANNEL_ALL);
  HAL_Delay(500);
  jy62_Init(&huart2);
  zigbee_Init(&huart3);
  Calibrate_Angle();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    // if (getGameState() == 1)
    //   HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
    // HAL_Delay(500);
    // for (int i = 0; i < 3; i++)
    // {
    //   BeaconDir[i][0] = getMyBeaconPosX(i);
    //   BeaconDir[i][1] = getMyBeaconPosY(i);
    // }
    // CalculateNowDir();
    // u2_printf("%d,%d,%d,%d\n", BeaconDir[0][0], BeaconDir[0][1], now_X, now_Y);
    // HAL_Delay(1000);

    // 测量资源位置 work了
    // Calculate_Package_Position();
    // u2_printf("%f, %f, %f, %f\n", packageX[0], packageY[0], packageX[1], packageY[1]);
    // HAL_Delay(1000);

    if (getGameState() == 1)
    {
      MoveTo(127, 127);
      MoveTo(15, 15);
      MoveTo(127, 127);
      MoveTo(160, 170);
    }
    
    // uint16_t x = getMyBeaconPosX(0), y = getMyBeaconPosY(0);
    // u2_printf("%d, %d\n", x, y);
    // HAL_Delay(500);
    // Set_Right_Direction(0);
    // Set_Left_Direction(0);
    // u3_printf("%f\n", getDistance());
    // HAL_Delay(500);
    /*********************主逻辑******************/
    // uint8_t nearParkId;
    // while (getGameState() == 1)
    // {
    //   uint8_t ROUND = getCarTask();

    //   if (ROUND == 0)
    //   {
    //     switch (STATE)
    //     {
    //     case GET_PACKAGE:
    //       Calculate_Package_Position();
    //       MoveTo(packageX[0], packageY[0]);
    //       MoveTo(packageX[1], packageY[1]);
    //       if (getCarMineSumNum() == 2)
    //         STATE = Set_Beacon;
    //       break;

    //     case PUT_PACKAGE:
    //       nearParkId = getNearestPark();
    //       MoveTo(Park[nearParkId].x, Park[nearParkId].y);
    //       break;

    //     case SET_BEACON:
    //       MoveTo(30, 30);
    //       Set_Beacon();
    //       MoveTo();
    //       Set_Beacon();
    //       MoveTo();
    //       Set_Beacon();
    //       STATE = PUT_PACKAGE;
    //       break;

    //     default:
    //       break;
    //     }
    //   }
    //   else
    //   {
    //   }
    // }
    /* USER CODE END 3 */
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
