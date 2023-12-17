#include "AS5600.h"

AS5600_measure AS5600;

/*
AS5600的7位地址是(二进制为0110110)。
7位地址后面跟着读写方向位，0（低）为写。
*/

uint8_t AS5600_IIC_Read_OneByte(uint8_t deviceaddr,uint8_t readaddr)
{
  uint8_t temp;
  IIC_Start();
  IIC_Send_Byte(deviceaddr&0xfe);        //写
  IIC_Wait_ack();
  IIC_Send_Byte(readaddr);
  IIC_Wait_ack();

  IIC_Start();
  IIC_Send_Byte(deviceaddr|0x01);       //读
  IIC_Wait_ack();
  temp=IIC_Read_Byte(0);
  IIC_Stop();
  return temp;
} 

void angle_Calculate(AS5600_measure *as5600)
{
		as5600->last_total_ecd = as5600->total_ecd;
    if (as5600->last_ecd - as5600->ecd >= 2048)
        {
          as5600->total_ecd += 4096 - as5600->last_ecd + as5600->ecd;
          as5600->rad++;
        }
    else if(as5600->ecd - as5600->last_ecd >= 2048)
        {
          as5600->total_ecd -= 4096 - as5600->ecd + as5600->last_ecd;
          as5600->rad--;
        }
    else
        as5600->total_ecd -= as5600->last_ecd - as5600->ecd;

}

/**
 * @brief  AS5600初始化，获取零点补偿
 * @param  as5600：AS5600数据结构体指针
 * @retval 无
 */
void AS5600_Init(AS5600_measure *as5600)
{
  IIC_Init();
  for (int i = 0; i < 5; i++)
  {
		as5600->last_ecd = as5600->ecd;
		int32_t value = 0;
		value =  AS5600_IIC_Read_OneByte((0x36<<1),0x0e);   
		value <<= 8;
		value |= AS5600_IIC_Read_OneByte((0x36<<1),0x0f);
		as5600->ecd = value;
		as5600->angle = as5600->ecd*360.0f/4096;
		angle_Calculate(as5600);
		as5600->total_ecd = 0;
  }
}

/**
 * @brief  AS5600获取角度
 * @param  as5600：AS5600数据结构体指针
 * @retval 无
 */
void AS5600_Get_Angle(AS5600_measure *as5600)
{
  as5600->last_ecd = as5600->ecd;
  int32_t value = 0;
  value =  AS5600_IIC_Read_OneByte((0x36<<1),0x0e);   
	value <<= 8;
	value |= AS5600_IIC_Read_OneByte((0x36<<1),0x0f);
	if(value-as5600->ecd > 200 || value-as5600->ecd < -200)
	{
		value =  AS5600_IIC_Read_OneByte((0x36<<1),0x0e);   
		value <<= 8;
		value |= AS5600_IIC_Read_OneByte((0x36<<1),0x0f);
		as5600->ecd = value;
	}
  else
	{
		as5600->ecd = value;
	}
  as5600->angle = as5600->ecd*360.0f/4096;
  angle_Calculate(as5600);
}

/**
 * @brief  AS5600获取速度，放到10khz的定时器中断内
 * @param  as5600：AS5600数据结构体指针
 * @retval 无
 */
void AS5600_Speed_Cal_10khz(AS5600_measure *as5600)
{
	static float last = 0;
	last = as5600->speed_rmp;
	float temp = (as5600->total_ecd - as5600->last_total_ecd)*600000.0f/4096;
	if(temp - last > 3300 || temp - last < -3300)
  as5600->speed_rmp = last*0.999f + temp*0.001f;
	else
	as5600->speed_rmp = last*0.99f + temp*0.01f;
//	static int32_t last[20] = {0}, sum = 0;
//	static int32_t temp = 0;
//	static uint8_t t = 0;
//	sum -=last[t];
//	last[t] = (as5600->total_ecd - as5600->last_total_ecd);
//	sum +=last[t];
//	t++;
//	if(t>19)
//	{
//		t=0;
//	}
//	as5600->speed_rmp = sum*3000.0f/4096;
}
