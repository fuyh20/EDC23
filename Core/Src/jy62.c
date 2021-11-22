#include "jy62.h"

uint8_t jy62Receive[JY62_MESSAGE_LENGTH]; //实时记录收到的信息
uint8_t jy62Message[JY62_MESSAGE_LENGTH]; //确认无误后用于解码的信息
uint8_t PrevLen, SuccLen;
uint8_t jy62PrevInfoHolder[11];
uint8_t initAngle[3] = {0xFF, 0xAA, 0x52};
uint8_t calibrateAcce[3] = {0xFF, 0xAA, 0x67};
uint8_t setBaud115200[3] = {0xFF, 0xAA, 0x63};
uint8_t setBaud9600[3] = {0xFF, 0xAA, 0x64};
uint8_t setHorizontal[3] = {0xFF, 0xAA, 0x65};
uint8_t setVertical[3] = {0xFF, 0xAA, 0x66};
uint8_t sleepAndAwake[3] = {0xFF, 0xAA, 0x60};
UART_HandleTypeDef *jy62_huart;

struct Acce Accelerate;	 //储存加速度
struct Velo Velocity;	 //储存角速度
struct Angl Angle;		 //储存角度值
struct Temp Temperature; //储存温度值
/***********************接口****************************/

void jy62_Init(UART_HandleTypeDef *huart)
{
	jy62_huart = huart;
	HAL_UART_Receive_DMA(jy62_huart, jy62Receive, JY62_MESSAGE_LENGTH);
}

// void jy62MessageRecord(void)
// {
// 	if (jy62Receive[0] == 0x55)
// 	{
// 		uint8_t sum = 0x00;
// 		for (int i = 0; i < JY62_MESSAGE_LENGTH - 1; i++)
// 		{
// 			sum += jy62Receive[i];
// 		}
// 		if (sum == jy62Receive[JY62_MESSAGE_LENGTH - 1])
// 		{
// 			for (int i = 0; i < JY62_MESSAGE_LENGTH; i++)
// 			{
// 				jy62Message[i] = jy62Receive[i];
// 			}
// 			Decode();
// 		}
// 	}
// 	HAL_UART_Receive_DMA(jy62_huart, jy62Receive, JY62_MESSAGE_LENGTH);
// }

void jy62MessageRecord(void)
{
    // PrevLen = 累计丢位数 % 11 = 本条信息被上次接收操作接收的位数（头部分）
    // SuccLen = 11 - PrevLen = 本条信息中被本次接收操作接收的位数（尾部分）
    //特别的，如果丢位不发生，那么PrevLen始终为0，SuccLen始终为11，此即trivial情况
    //仅当丢位再次发生时才需要更新PrevLen和SuccLen
    //首先进行校验：按照SuccLen检查理论上剩余部分的后一位是否为0x55，即检查丢位是否再次发生
    if (jy62Receive[SuccLen % JY62_MESSAGE_LENGTH] == 0x55)
    {
        //如果是，将本次接收到信息属于上一条的部分录入Holder中，接在已接收长度的后边
        for (int i = 0; i < SuccLen % JY62_MESSAGE_LENGTH; i++)
        {
            jy62PrevInfoHolder[PrevLen + i] = jy62Receive[i];
        }
        //求和，对校验和进行检查
        uint8_t sum = 0x00;
        for (int i = 0; i < JY62_MESSAGE_LENGTH - 1; i++)
        {
            sum += jy62PrevInfoHolder[i];
        }
        //如果符合就输入到Message中，然后解码
        if (sum == jy62PrevInfoHolder[JY62_MESSAGE_LENGTH - 1])
        {
            for (int i = 0; i < JY62_MESSAGE_LENGTH; i++)
            {
                jy62Message[i] = jy62PrevInfoHolder[i];
            }
            Decode();
        }
    }
    else
    {
        //否则需要更新SuccLen和PrevLen。由于不能确定丢位多少（可能超过11），需要从头找
        SuccLen = 0;
        while (SuccLen < JY62_MESSAGE_LENGTH && jy62Receive[SuccLen] != 0x55)
        {
            SuccLen++;
        }
        PrevLen = JY62_MESSAGE_LENGTH - SuccLen;
    }
    //然后统一进入Holder的覆盖，将本次接收操作中收到的下一次信息的头部置入Holder中
    for (int i = SuccLen % JY62_MESSAGE_LENGTH; i < JY62_MESSAGE_LENGTH; i++)
    {
        jy62PrevInfoHolder[i - SuccLen % JY62_MESSAGE_LENGTH] = jy62Receive[i];
    }
    //重新开始接收
    HAL_UART_Receive_DMA(jy62_huart, jy62Receive, JY62_MESSAGE_LENGTH);
}

void SetBaud(int Baud)
{
	if (Baud == 115200)
	{
		HAL_UART_Transmit(jy62_huart, setBaud115200, 3, HAL_MAX_DELAY);
	}
	else if (Baud == 9600)
	{
		HAL_UART_Transmit(jy62_huart, setBaud115200, 3, HAL_MAX_DELAY);
	}
}

void SetHorizontal()
{
	HAL_UART_Transmit(jy62_huart, setHorizontal, 3, HAL_MAX_DELAY);
}

void SetVertical()
{
	HAL_UART_Transmit(jy62_huart, setVertical, 3, HAL_MAX_DELAY);
}

void InitAngle()
{
	HAL_UART_Transmit(jy62_huart, initAngle, 3, HAL_MAX_DELAY);
}

void Calibrate()
{
	HAL_UART_Transmit(jy62_huart, calibrateAcce, 3, HAL_MAX_DELAY);
}

void SleepOrAwake()
{
	HAL_UART_Transmit(jy62_huart, sleepAndAwake, 3, HAL_MAX_DELAY);
}

float GetRoll()
{
	return Angle.roll;
}
float GetPitch()
{
	return Angle.pitch;
}
float GetYaw()
{
	return Angle.yaw;
}

float GetTemperature()
{
	return Temperature.temperature;
}

float GetAccX()
{
	return Accelerate.accelerate_x;
}
float GetAccY()
{
	return Accelerate.accelerate_y;
}
float GetAccZ()
{
	return Accelerate.accelerate_z;
}

float GetVeloX()
{
	return Velocity.velocity_x;
}
float GetVeloY()
{
	return Velocity.velocity_y;
}
float GetVeloZ()
{
	return Velocity.velocity_z;
}

/***************************************************/

void DecodeAngle()
{
	Angle.roll = (float)((jy62Message[3] << 8) | jy62Message[2]) / 32768 * 180;
	Angle.pitch = (float)((jy62Message[5] << 8) | jy62Message[4]) / 32768 * 180;
	Angle.yaw = (float)((jy62Message[7] << 8) | jy62Message[6]) / 32768 * 180;
}

void DecodeAccelerate()
{
	Accelerate.accelerate_x = (float)((jy62Message[3] << 8) | jy62Message[2]) / 32768 * 16 * g;
	Accelerate.accelerate_y = (float)((jy62Message[5] << 8) | jy62Message[4]) / 32768 * 16 * g;
	Accelerate.accelerate_z = (float)((jy62Message[7] << 8) | jy62Message[6]) / 32768 * 16 * g;
}

void DecodeVelocity()
{
	Velocity.velocity_x = (float)((jy62Message[3] << 8) | jy62Message[2]) / 32768 * 2000;
	Velocity.velocity_y = (float)((jy62Message[5] << 8) | jy62Message[4]) / 32768 * 2000;
	Velocity.velocity_z = (float)((jy62Message[7] << 8) | jy62Message[6]) / 32768 * 2000;
}

void DecodeTemperature()
{
	Temperature.temperature = ((short)(jy62Message[9]) << 8 | jy62Message[8]) / 340 + 36.53;
}

void Decode()
{
	switch (jy62Message[1])
	{
	case 0x51:
		DecodeAccelerate();
		break;
	case 0x52:
		DecodeVelocity();
		break;
	case 0x53:
		DecodeAngle();
		break;
	}
	DecodeTemperature();
}