#include "stm32f1xx_hal.h"

#define min(a,b) ((a<b)?a:b)


#define PWM_DUTY_LIMIT 7200
//������
uint16_t Encoder_GetCNT(uint8_t nEncoder); //���ر������ļ���ֵ��nEncoder=1���ر�����A���Դ�����
float Get_Motor_Speed(uint8_t nEncoder); //���ر������õ��ĵ���ٶȣ�nEncoder=1���ر�����A���Դ�����

void Motor_forward(void);
void Motor_back(void);
void Motor_left(void);
void Motor_right(void);


void MotorDriver_SetPWMDuty(uint8_t nMotor, uint16_t nDuty);
