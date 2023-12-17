#ifndef __FOC_H__
#define __FOC_H__

#include "main.h"
#include "AS5600.h"
#include "pid.h"
#include "mw_cmsis.h"

#define Pole_pairs 7
#define Ele_offset 544   	//电角度机械补偿
#define IN1_offset 2011   //I相电流ADC补偿
#define IN2_offset 2081   //W相电流ADC补偿

#define Torque_Mode 	0
#define Speed_Mode  	1
#define Position_Mode 2

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
    float U_q;        
    float U_d;        //qd轴电压

} QD;

typedef struct
{
    AS5600_measure *Encoder_measure;
    float          Theta;             //电角度（rad）
    Alpha_Beta     Alpha_Beta;
    UVW            UVW;
    QD             qd;
    pid_t          Pid_I;            //电流环pid
    pid_t          Pid_Vel;          //速度环pid
    pid_t          Pid_Pos;          //位置环pid
		uint8_t				 mode;             //0：力矩；1：速度；2：位置
		float          target;
} FOC;

extern FOC FOC1_Handler;

void FOC_Init(FOC *handler);
void FOC_Park(FOC *handler);
void FOC_Clark(FOC *handler);
void FOC_SVPWM(FOC *handler);
void FOC_Pid_Cal(FOC *handler);


#endif
