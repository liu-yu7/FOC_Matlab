#ifndef __FOC_H__
#define __FOC_H__

#include "main.h"
#include "user_config.h"
#include "MT6701.h"
#include "pid.h"
#include "mw_cmsis.h"



// #define IN1_offset 2069   //I相电流ADC补偿
// #define IN2_offset 2137   //W相电流ADC补偿


typedef struct
{
    float I_Alpha;     //α轴电流
    float I_Beta;      //β轴电流
    float U_Alpha;     //α轴电压
    float U_Beta;      //β轴电压
} Alpha_Beta;

typedef struct
{
    float I_U;
    float I_V;
    float I_W;        //UVW相电流
    uint16_t T_U;
    uint16_t T_V;
    uint16_t T_W;     //UVW相ccr

} UVW;

typedef struct
{
    float I_q;
    float I_d;        //qd轴电流
		float I_q_filt;
		float I_d_filt;		//用于报告的qd轴电流
    float U_q;        
    float U_d;        //qd轴电压

} QD;

typedef enum FOC_Mode{
	Torque_Mode = 0,		//力矩模式
	Speed_Mode,					//速度模式
	Position_Mode,			//位置模式
} tFOC_Mode;

typedef enum FOC_State{
	Cilibration = 0,			//校准
	Ready,							//准备完成
	Run,								//运行
	Err,								//出错
} tFOC_State;

typedef struct
{
		TIM_HandleTypeDef *htim;          //pwm定时器
		TIM_TypeDef 			*TIM;           //pwm定时器   
		ADC_HandleTypeDef *hadc;					//采样adc
    MT6701_measure *Encoder_measure;
    float          Theta;             //电角度（rad）
    Alpha_Beta     Alpha_Beta;
    UVW            UVW;
    QD             qd;
    pid_t          Pid_Vel;          //速度环pid
    pid_t          Pid_Pos;          //位置环pid
		tFOC_Mode			 mode;             //Torque_Mode：力矩；Speed_Mode：速度；Position_Mode：位置
		tFOC_State		 state;							//FOC状态 Clibration:校准Ready:准备完成Run:运行Err:出错
		tUser_Config 	 *usrConfig;			 //配置参数
		float          target_i;			 //q轴电流目标
		float          target;
} FOC;

extern FOC FOC1_Handler;

void FOC_Init(FOC *handler);
void FOC_Park(FOC *handler);
void FOC_Clark(FOC *handler);
void FOC_SVPWM(FOC *handler);
void FOC_Pid_Cal(FOC *handler);


#endif
