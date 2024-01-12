/****************************************************************************************************************************************
                                              MT6701编码器读取角度计算速度
    * cubeMX配置spi主仅读,LOW,2SPI_PHASE_2EDGE一次读8bit
****************************************************************************************************************************************/


#include "MT6701.h"
#include "spi.h"

#define MT6701_CS_Enable() do{HAL_GPIO_WritePin(MT6701_CS_GPIO_Port, MT6701_CS_Pin, GPIO_PIN_RESET);}while(0);
#define MT6701_CS_Disable() do{HAL_GPIO_WritePin(MT6701_CS_GPIO_Port, MT6701_CS_Pin, GPIO_PIN_SET);}while(0);

MT6701_measure MT6701;

static uint8_t crc6_mt(uint8_t *data, uint32_t length)                          //mt6701的crc校验
{
	uint8_t i;
	uint8_t crc = 0;
	while(length--)
	{
		crc ^= *data++;
		for(i = 6; i>0; --i)
		{
			if(crc & 0x20)
			{
				crc = (crc<<1) ^ 0x03;
			}
			else
			{
				crc = (crc<<1);
			}
		}
		return (crc&0x3f);
	}
}

static void angle_Calculate(MT6701_measure *mt6701)                        //角度计算（过零检测）
{
    mt6701->last_total_ecd = mt6701->total_ecd;
    if (mt6701->last_ecd - mt6701->ecd >= 8192)
        {
          mt6701->total_ecd += 16384 - mt6701->last_ecd + mt6701->ecd;
          mt6701->rad++;
        }
    else if(mt6701->ecd - mt6701->last_ecd >= 8192)
        {
          mt6701->total_ecd -= 16384 - mt6701->ecd + mt6701->last_ecd;
          mt6701->rad--;
        }
    else
        mt6701->total_ecd -= mt6701->last_ecd - mt6701->ecd;
}

/**
 * @brief  MT6701初始化，获取零点补偿
 * @param  mt6701：MT6701数据结构体指针
 * @retval 无
 */
void MT6701_Init(MT6701_measure *mt6701)
{
    for (int i = 0; i < 10; i++)
    {
        mt6701->last_ecd = mt6701->ecd;
        uint8_t rxdata[3];

        MT6701_CS_Enable();                                        
        HAL_SPI_Receive(&hspi1, (uint8_t *)&rxdata[0], 3, 200);     
        MT6701_CS_Disable();
        uint32_t temp = ((uint32_t)rxdata[0]<<16)|((uint32_t)rxdata[1]<<8)|(rxdata[2]);
        mt6701->ecd = (temp>>10);
				mt6701->angle = mt6701->ecd*360.0f/16384;
        angle_Calculate(mt6701);
        mt6701->total_ecd = 0;
    }
}

/**
 * @brief  MT6701获取角度
 * @param  mt6701：MT6701数据结构体指针
 * @retval 无
 */
void MT6701_GetAngle(MT6701_measure *mt6701)
{
		mt6701->last_ecd = mt6701->ecd;
		uint8_t rxdata[3];

		MT6701_CS_Enable();                                         //拉低片选开始通讯
		HAL_SPI_Receive(&hspi1, (uint8_t *)&rxdata[0], 3, 200);     //
		MT6701_CS_Disable();
		uint32_t temp = ((uint32_t)rxdata[0]<<16)|((uint32_t)rxdata[1]<<8)|(rxdata[2]);
		// uint8_t crc = crc6_mt((uint8_t *)&temp, 18);
		// if((temp&0x3f) == crc)
		// 	flag = 1;
		// else
		// 	flag = 0;
		mt6701->ecd = (temp>>10);
		mt6701->angle = mt6701->ecd*360.0f/16384;
		angle_Calculate(mt6701);
}

/**
 * @brief  MT6701获取速度，放到10khz的定时器中断内
 * @param  mt6071：MT6701数据结构体指针
 * @retval 无
 */
void MT6701_Speed_Cal_10khz(MT6701_measure *mt6071)
{
	static float last = 0;
	last = mt6071->speed_rmp;
	float temp = (mt6071->total_ecd - mt6071->last_total_ecd)*600000.0f/16384;
	if(temp - last > 3300 || temp - last < -3300)
    mt6071->speed_rmp = last*0.999f + temp*0.001f;
	else
		mt6071->speed_rmp = last*0.99f + temp*0.01f;
//	static int32_t last[20] = {0}, sum = 0;
//	static int32_t temp = 0;
//	static uint8_t t = 0;
//	sum -=last[t];
//	last[t] = (mt6071->total_ecd - mt6071->last_total_ecd);
//	sum +=last[t];
//	t++;
//	if(t>19)
//	{
//		t=0;
//	}
//	mt6071->speed_rmp = sum*3000.0f/4096;
}