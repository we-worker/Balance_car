#include "stm32f1xx_hal.h"

#define min(a,b) ((a<b)?a:b)


#define PWM_DUTY_LIMIT 7200   //7200��׼
//������
uint16_t Encoder_GetCNT(uint8_t nEncoder); //���ر������ļ���ֵ��nEncoder=1���ر�����A���Դ�����
float Get_Motor_Speed(uint8_t nEncoder); //���ر������õ��ĵ���ٶȣ�nEncoder=1���ر�����A���Դ�����


void MotorDriver_SetPWMDuty(uint8_t nMotor, int nDuty); //PWM_DUTY_LIMIT/2Լ����ת��0����������3000 
