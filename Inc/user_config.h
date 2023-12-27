#ifndef __USER_CONFIG_H__
#define __USER_CONFIG_H__

#include "main.h"


typedef struct UserConfig{
	//电机参数
	uint8_t Pole_pairs;        			//电机极对数
	float Phase_resistance;					//电机相电阻
	float Phase_inductance;					//电机相电感
	float Inertia;									//电机惯量（）
	float Torque_constant;					//力矩常数（Nm/A）
	
	//补偿常数
	int32_t Ele_offset; 					  //电角度机械补偿
	int32_t IN1_offset; 						//IN1电流ADC补偿
	int32_t IN2_offset; 					  //IN2电流ADC补偿
	
	//PID参数
	float Current_p;								
	float Current_i;								//电流环pi
	float Vel_p;
	float Vel_i;										
	float Vel_limit;								//速度环pi和限制
	int16_t Current_bandwidth;			//电流环带宽

} tUser_Config;

extern tUser_Config UserConfig;


#endif
