#include "stm32f1xx_hal.h"

#define min(a,b) ((a<b)?a:b)


#define PWM_DUTY_LIMIT 7200   //7200标准
//编码器
uint16_t Encoder_GetCNT(uint8_t nEncoder); //返回编码器的计数值，nEncoder=1返回编码器A，以此类推
float Get_Motor_Speed(uint8_t nEncoder); //返回编码器得到的电机速度，nEncoder=1返回编码器A，以此类推


void MotorDriver_SetPWMDuty(uint8_t nMotor, int nDuty); //PWM_DUTY_LIMIT/2约等于转速0，我这里是3000 
