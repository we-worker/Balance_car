#include "mpu6050.h" 


//初始化MPU6050
//返回值:0,成功
//    其他,错误代码

uint8_t MPU_Init(void)
{ 
  uint8_t res;
	
  res=MPU_Read_Byte(MPU_DEVICE_ID_REG);
	
  if(res==0x68)//器件ID正确
  {
	MPU_Write_Byte(MPU_PWR_MGMT1_REG,0X00);	//唤醒MPU6050 
	MPU_Set_Gyro_Fsr(0);					//陀螺仪传感器,±250dps
	MPU_Set_Accel_Fsr(0);					//加速度传感器,±2g
	MPU_Set_Rate(50);						//设置采样率50Hz
  }else 
		return 1;
  return 0;
}

//设置MPU6050陀螺仪传感器满量程范围
//fsr:0,±250dps;1,±500dps;2,±1000dps;3,±2000dps
//返回值:0,设置成功
//    其他,设置失败 
uint8_t MPU_Set_Gyro_Fsr(uint8_t fsr)
{
	return MPU_Write_Byte(MPU_GYRO_CFG_REG,fsr<<3);//设置陀螺仪满量程范围  
}
//设置MPU6050加速度传感器满量程范围
//fsr:0,±2g;1,±4g;2,±8g;3,±16g
//返回值:0,设置成功
//    其他,设置失败 
uint8_t MPU_Set_Accel_Fsr(uint8_t fsr)
{
	return MPU_Write_Byte(MPU_ACCEL_CFG_REG,fsr<<3);//设置加速度传感器满量程范围  
}
//设置MPU6050的数字低通滤波器
//lpf:数字低通滤波频率(Hz)
//返回值:0,设置成功
//    其他,设置失败 
uint8_t MPU_Set_LPF(uint16_t lpf)
{
	uint8_t data=0;
	if(lpf>=188)data=1;
	else if(lpf>=98)data=2;
	else if(lpf>=42)data=3;
	else if(lpf>=20)data=4;
	else if(lpf>=10)data=5;
	else data=6; 
	return MPU_Write_Byte(MPU_CFG_REG,data);//设置数字低通滤波器  
}
//设置MPU6050的采样率(假定Fs=1KHz)
//rate:4~1000(Hz)
//返回值:0,设置成功
//    其他,设置失败 
uint8_t MPU_Set_Rate(uint16_t rate)
{
	uint8_t data;
	if(rate>1000)rate=1000;
	if(rate<4)rate=4;
	data=1000/rate-1;
	data=MPU_Write_Byte(MPU_SAMPLE_RATE_REG,data);	//设置数字低通滤波器
 	return MPU_Set_LPF(rate/2);	//自动设置LPF为采样率的一半
}

//得到温度值
//返回值:温度值(扩大了100倍)
float MPU_Get_Temperature(void)
{
  unsigned char  buf[2]; 
  short raw;
  float temp;
  
  MPU_Read_Len(MPU_TEMP_OUTH_REG,2,buf); 
  raw=(buf[0]<<8)| buf[1];  
  temp=(36.53+((double)raw)/340)*100;  
//  temp = (long)((35 + (raw / 340)) * 65536L);
  return temp/100.0f;
}
//得到陀螺仪值(原始值)
//gx,gy,gz:陀螺仪x,y,z轴的原始读数(带符号)
//返回值:0,成功
//    其他,错误代码
uint8_t MPU_Get_Gyroscope(float *gx,float *gy,float *gz)
{
    uint8_t buf[6],res;  
	res=MPU_Read_Len(MPU_GYRO_XOUTH_REG,6,buf);
	if(res==0)
	{
		*gx=(int16_t)(buf[0]<<8|buf[1]);  
		*gy=(int16_t)(buf[2]<<8|buf[3]);  
		*gz=(int16_t)(buf[4]<<8|buf[5]);
		
		*gx = *gx/131.0;
		*gy = *gy/131.0;
		*gz = *gz/131.0;
		//16.4
	} 	
    return res;
}

//得到加速度值(原始值)
//gx,gy,gz:陀螺仪x,y,z轴的原始读数(带符号)
//返回值:0,成功
//    其他,错误代码
uint8_t MPU_Get_Accelerometer(float *ax,float *ay,float *az)
{
    uint8_t buf[6],res;  
	res=MPU_Read_Len(MPU_ACCEL_XOUTH_REG,6,buf);
	if(res==0)
	{
		*ax=((int16_t)buf[0]<<8)|buf[1];  
		*ay=((int16_t)buf[2]<<8)|buf[3];  
		*az=((int16_t)buf[4]<<8)|buf[5];
		*ax = *ax/16384.0;
		*ay = *ay/16384.0;
		*az = *az/16384.0;
	} 	
    return res;
}


//IIC连续写
uint8_t MPU_Write_Len(uint8_t reg,uint8_t len,uint8_t *buf)
{
  extern I2C_HandleTypeDef hi2c2;
  HAL_I2C_Mem_Write(&hi2c2, MPU_WRITE, reg, I2C_MEMADD_SIZE_8BIT, buf, len, 0xfff);
  HAL_Delay(100);
  
  return 0;
}
//IIC连续读
//addr:器件地址
//reg:要读取的寄存器地址
//len:要读取的长度
//buf:读取到的数据存储区
//返回值:0,正常
//    其他,错误代码
uint8_t MPU_Read_Len(uint8_t reg,uint8_t len,uint8_t *buf)
{ 
  extern I2C_HandleTypeDef hi2c2;
  HAL_I2C_Mem_Read(&hi2c2, MPU_READ, reg, I2C_MEMADD_SIZE_8BIT, buf, len, 0xfff);
  HAL_Delay(100);
  
  return 0;	
}
//IIC写一个字节 
//reg:寄存器地址
//data:数据
//返回值:0,正常
//    其他,错误代码
uint8_t MPU_Write_Byte(uint8_t reg,uint8_t data) 				 
{ 
  extern I2C_HandleTypeDef hi2c2;
  unsigned char W_Data=0;

  W_Data = data;
  HAL_I2C_Mem_Write(&hi2c2, MPU_WRITE, reg, I2C_MEMADD_SIZE_8BIT, &W_Data, 1, 0xfff);
  HAL_Delay(100);
  
  return 0;
}
//IIC读一个字节 
//reg:寄存器地址 
//返回值:读到的数据
uint8_t MPU_Read_Byte(uint8_t reg)
{
  extern I2C_HandleTypeDef hi2c2;
  unsigned char R_Data=0;
  
  HAL_I2C_Mem_Read(&hi2c2, MPU_READ, reg, I2C_MEMADD_SIZE_8BIT, &R_Data, 1, 0xfff);
  HAL_Delay(100);
  
  return R_Data;		
}

//MPU6050_dmpWrite
int MPU6050_dmpWrite(uint8_t addr, uint8_t reg, uint8_t len, uint8_t *data)
{
		extern I2C_HandleTypeDef hi2c2;
		HAL_I2C_Mem_Write(&hi2c2, addr, reg, I2C_MEMADD_SIZE_8BIT, data, len, 0xfff);
    return 0;
}
//MPU6050_dmpRead
int MPU6050_dmpRead(uint8_t addr, uint8_t reg, uint8_t len, uint8_t *buf)
{
    extern I2C_HandleTypeDef hi2c2;
		HAL_I2C_Mem_Read(&hi2c2, addr, reg, I2C_MEMADD_SIZE_8BIT, buf, len, 0xfff);
    return 0;
}

void myget_ms(unsigned long *time)
{

}
