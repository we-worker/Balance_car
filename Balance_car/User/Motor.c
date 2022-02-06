#include "Motor.h"
#include "stm32f1xx.h"


int myabs(int a)
{ 		   
	  int temp;
		if(a<0)  temp=-a;  
	  else temp=a;
	  return temp;
}


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
	if(myabs(speed)>55000)
	{
		if(speed>0)speed-=65536;
		else speed+=65536;
	}
	return speed/10.0;  //除去延时的时间
		
}

void MotorDriver_SetPWMDuty(uint8_t nMotor, int nDuty)
{
	uint16_t nDutySet;
	if(myabs(nDuty)>PWM_DUTY_LIMIT-1) nDutySet = PWM_DUTY_LIMIT-1;
	else nDutySet = myabs(nDuty);
	

	switch (nMotor)
	{
		case 1:
			TIM4->CCR4 = nDutySet;	
			if(nDuty>0){
				HAL_GPIO_WritePin(GPIOB,GPIO_PIN_12,GPIO_PIN_SET);
				HAL_GPIO_WritePin(GPIOB,GPIO_PIN_13,GPIO_PIN_RESET);
			}else{
				HAL_GPIO_WritePin(GPIOB,GPIO_PIN_12,GPIO_PIN_RESET);
				HAL_GPIO_WritePin(GPIOB,GPIO_PIN_13,GPIO_PIN_SET);
			}
			break;
		case 2:
			TIM4->CCR3 = nDutySet;
			if(nDuty>0){
				HAL_GPIO_WritePin(GPIOB,GPIO_PIN_14,GPIO_PIN_RESET);
				HAL_GPIO_WritePin(GPIOB,GPIO_PIN_15,GPIO_PIN_SET);
			}else{
				HAL_GPIO_WritePin(GPIOB,GPIO_PIN_14,GPIO_PIN_SET);
				HAL_GPIO_WritePin(GPIOB,GPIO_PIN_15,GPIO_PIN_RESET);
			}
			break;
		default:
			;
	}
	//HAL_Delay(1);
}