#include "Motor.h"
#include "stm32f1xx.h"
#include "math.h"



uint16_t Encoder_GetCNT(uint8_t nEncoder)
{
	switch (nEncoder)
	{
		case 1:
			return TIM3->CNT;
		case 2:
			return TIM2->CNT;
		default:
			return 0;
	}
}

float Get_Motor_Speed(uint8_t nEncoder)
{
	float speed;
	uint16_t last_CNT=Encoder_GetCNT(nEncoder);
	HAL_Delay(10);
	uint16_t now_CNT=Encoder_GetCNT(nEncoder);
	speed=now_CNT-last_CNT;
	if(abs(speed)>55000)
	{
		if(speed>0)speed-=65536;
		else speed+=65536;
	}
	return speed/10.0;  //除去延时的时间
		
}

void Motor_forward(void)
{
	HAL_GPIO_WritePin(GPIOB,GPIO_PIN_12,GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOB,GPIO_PIN_13,GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOB,GPIO_PIN_14,GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOB,GPIO_PIN_15,GPIO_PIN_SET);
}

void Motor_back(void)
{
	HAL_GPIO_WritePin(GPIOB,GPIO_PIN_12,GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOB,GPIO_PIN_13,GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOB,GPIO_PIN_14,GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOB,GPIO_PIN_15,GPIO_PIN_RESET);
}

void Motor_left(void)
{
	HAL_GPIO_WritePin(GPIOB,GPIO_PIN_12,GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOB,GPIO_PIN_13,GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOB,GPIO_PIN_14,GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOB,GPIO_PIN_15,GPIO_PIN_SET);
}

void Motor_right(void)
{
	HAL_GPIO_WritePin(GPIOB,GPIO_PIN_12,GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOB,GPIO_PIN_13,GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOB,GPIO_PIN_14,GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOB,GPIO_PIN_15,GPIO_PIN_RESET);
}

void MotorDriver_SetPWMDuty(uint8_t nMotor, uint16_t nDuty)
{
	uint16_t nDutySet;
	if(nDuty>PWM_DUTY_LIMIT) nDutySet = PWM_DUTY_LIMIT;
	else nDutySet = nDuty;
	switch (nMotor)
	{
		case 1:
			TIM4->CCR4 = nDutySet;
			break;
		case 2:
			TIM4->CCR3 = nDutySet;
			break;
		default:
			;
	}
	HAL_Delay(100);
}