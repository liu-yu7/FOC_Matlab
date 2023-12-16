#ifndef __FOC_H__
#define __FOC_H__

#include "main.h"
#include "AS5600.h"
#include "mw_cmsis.h"

#define Pole_pairs 7

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
} FOC;

extern FOC FOC1_Handler;

void FOC_Init(FOC *handler);
void FOC_AntiPark(FOC *handler);
void FOC_SVPWM(FOC *handler);



#endif
