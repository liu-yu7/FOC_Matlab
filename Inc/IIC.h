#ifndef __IIC_H__
#define __IIC_H__

#include "main.h"

#define IIC_SCL(x)        do{ x ? \
                              HAL_GPIO_WritePin(IIC_SCL_GPIO_Port, IIC_SCL_Pin, GPIO_PIN_SET) : \
                              HAL_GPIO_WritePin(IIC_SCL_GPIO_Port, IIC_SCL_Pin, GPIO_PIN_RESET); \
                          }while(0)       /* SCL */

#define IIC_SDA(x)        do{ x ? \
                              HAL_GPIO_WritePin(IIC_SDA_GPIO_Port, IIC_SDA_Pin, GPIO_PIN_SET) : \
                              HAL_GPIO_WritePin(IIC_SDA_GPIO_Port, IIC_SDA_Pin, GPIO_PIN_RESET); \
                          }while(0)       /* SDA */
#define IIC_READ_SDA     HAL_GPIO_ReadPin(IIC_SDA_GPIO_Port, IIC_SDA_Pin) /* 读取SDA */

void IIC_Init(void);                          
void IIC_Start(void);
void IIC_Stop(void);
uint8_t IIC_Wait_ack(void);
void IIC_Send_Byte(uint8_t data);
uint8_t IIC_Read_Byte(uint8_t ack);      
                          
#endif
                          