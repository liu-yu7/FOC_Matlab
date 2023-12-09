/***************************************************************************************************************
 * 软件IIC
 * 需要在CubeMX配置GPIO，SCL和SDA都要高速上拉，SCL推挽宏定义IIC_SCL,SDA开漏宏定义IIC_SDA
 * Create by 羽栖 on 2023/9/9
***************************************************************************************************************/

#include "IIC.h"

/**
 * @brief       初始化IIC
 * @param       无
 * @retval      无
 */
 void IIC_Init(void)
{
  IIC_Stop();     /* 停止总线上所有设备 */
}

/**
 * @brief       IIC延时函数,用于控制IIC读写速度
 * @param       无
 * @retval      无
 */
static void IIC_Delay(void)
{
  uint16_t time = 79;    /* 2us的延时, 读写速度在250Khz以内 */
  while(time--)
  {
  
  }
  //HAL_Delay(1);
}

/**
 * @brief       产生IIC起始信号
 * @param       无
 * @retval      无
 */
void IIC_Start(void)
{
  /* START信号: 当SCL为高时, SDA从高变成低, 表示起始信号 */
  IIC_SDA(1);
  IIC_SCL(1);
  IIC_Delay();
  IIC_SDA(0);     
  IIC_Delay();
  IIC_SCL(0);     /* 钳住I2C总线，准备发送或接收数据 */
  IIC_Delay();
}

/**
 * @brief       产生IIC停止信号
 * @param       无
 * @retval      无
 */
void IIC_Stop(void)
{
  /* STOP号: 当SCL为高时, SDA从低变成高, 表示停止信号 */
  IIC_SDA(0);
  IIC_SCL(1);
  IIC_Delay();
  IIC_SDA(1);        /* 发送I2C总线结束信号 */
  IIC_Delay();
}

/**
 * @brief       产生ACK应答
 * @param       无
 * @retval      无
 */
void IIC_Ack()
{
  /* SCL 0 -> 1  时 SDA = 0,表示应答 */
  IIC_SDA(0);
  IIC_Delay();
  IIC_SCL(1);   //应答
  IIC_Delay();
  IIC_SCL(0);
  IIC_Delay();
  IIC_SDA(1);   //释放SDA
  IIC_Delay();
}

/**
 * @brief       不产生ACK应答
 * @param       无
 * @retval      无
 */
void IIC_Nack(void)
{ 
  /* SCL 0 -> 1  时 SDA = 1,表示不应答 */
  IIC_SDA(1);     
  IIC_Delay();
  IIC_SCL(1);     /* 产生一个时钟 */
  IIC_Delay();
  IIC_SCL(0);
  IIC_Delay();
}

/**
 * @brief       等待应答信号到来
 * @param       无
 * @retval      1，接收应答失败
 *              0，接收应答成功
 */
uint8_t IIC_Wait_ack(void)
{
  uint8_t WaitTime = 0;
  uint8_t back = 0;
  
  IIC_SDA(1);     /*释放SDA此时外部从机可以拉低SDA*/
  IIC_Delay();
  IIC_SCL(1);     /*此时从机可以返回ACK*/
  IIC_Delay();
  
  while(IIC_READ_SDA)
  {
    WaitTime ++;
    if(WaitTime > 250)    /*等待超时*/
    {
      IIC_Stop();         /*停止主线上所有设备*/
      back = 1;
      break;
    }    
  }
  
  IIC_SCL(0);     /* SCL=0, 结束ACK检查 */
  IIC_Delay();
  return back;
}

/**
 * @brief       IIC发送一个字节
 * @param       data: 要发送的数据
 * @retval      无
 */
void IIC_Send_Byte(uint8_t data)
{
  IIC_SCL(0);
  IIC_Delay();
  for(uint8_t i = 0; i < 8; i++)
  {
    IIC_SDA((data & 0x80) >> 7);    /*发送高位*/
    IIC_Delay();
    IIC_SCL(1);
    IIC_Delay();
    IIC_SCL(0);
    data <<= 1;          /*左移一位方便下一位发送*/
  }
  IIC_SDA(1);            /*释放SDA*/
}

/**
 * @brief       IIC读取一个字节
 * @param       ack:  ack=1时，发送ack; ack=0时，发送nack
 * @retval      接收到的数据
 */
uint8_t IIC_Read_Byte(uint8_t ack)
{
  uint8_t i,receive = 0;
  for(i = 0; i < 8; i++)      
  {
    IIC_SCL(1);
    receive <<= 1;              /*高位先接收*/
    IIC_Delay();
    if(IIC_READ_SDA)
    {
      receive++;
    }
    IIC_SCL(0);
    IIC_Delay();
  }
  if(ack)
  {
    IIC_Ack();
  }
  else
  {
    IIC_Nack();
  }
  return receive;
}
