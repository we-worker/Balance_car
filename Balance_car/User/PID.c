#include "PID.h"
#include "main.h"
#include "mpu6050.h"
/**************************************************************************
�������ܣ�ֱ��PD����
��ڲ������Ƕȡ����ٶ�
����  ֵ��ֱ������PWM
**************************************************************************/
int balance(float Angle,float Gyro,float kp,float kd)
{  
   float Bias;
	 int balance;
	 Bias=Angle-0;       //===���ƽ��ĽǶ���ֵ0 
	 balance=kp*Bias+Gyro*kd;   //===����ƽ����Ƶĵ��PWM  PD����   kp��Pϵ�� kd��Dϵ�� 
	 return balance;
}


int velocity(int encoder_left,int encoder_right,float velocity_KP,float velocity_KI)
{  
    static float Velocity,Encoder_Least,Encoder,Movement = 0;
    static float Encoder_Integral;
    //=============speed PI controller=======================//	
    Encoder_Least =(MotorSpeed1+MotorSpeed2)-0;                    //get the newest speed defference
    Encoder *= 0.8;		                                              //1st order low pass filter     
    Encoder += Encoder_Least*0.2;	                                  //1st order low pass filter
    Encoder_Integral +=Encoder;                                       //intergral displacement t=10ms
    Encoder_Integral=Encoder_Integral-Movement;                       //receive remote control data
    if(Encoder_Integral>1500)  	
        Encoder_Integral=1500;             //intergral limit
    if(Encoder_Integral<-1500)		
        Encoder_Integral=-1500;            //intergral limit
    Velocity=Encoder*velocity_KP+Encoder_Integral*velocity_KI;        //speed control
    if(mpu_pose_msg.pitch<-30	||mpu_pose_msg.pitch>30) 			
        Encoder_Integral=0;     		  //clean intergral if motor close
    return -Velocity;
}
