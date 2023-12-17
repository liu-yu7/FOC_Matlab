#include "FOC.h"

FOC FOC1_Handler;


/**
 * @brief  初始化FOC
 * @param  handler：FOC句柄
 * @retval 无
 */
void FOC_Init(FOC *handler)
{
    handler->Encoder_measure = &AS5600;
    AS5600_Init(handler->Encoder_measure);
    pid_param_init_deadband(&handler->Pid_I, POSITION_PID, 0.2f, 0.1f, 0.6f, 0.0001f, 0, 0);
    pid_param_init(&handler->Pid_Vel, POSITION_PID, 0.3f, 0.1f, 0.01f, 0.001f, 0);
    pid_param_init(&handler->Pid_Pos, POSITION_PID, 0.3f, 0.1f, 0.01f, 0.001f, 0);
}

/**
 * @brief  Clark变换
 *         I_Alpha = I_U*2/3-(I_V+I_W)/3;
 *         I_Beta = (I_V-I_W)*sqrt(3)/3;
 * @param  handler：FOC句柄
 * @retval 无
 */
void FOC_Clark(FOC *handler)
{
    handler->Alpha_Beta.I_Alpha = 0.66666666666666663f * handler->UVW.I_U - (handler->UVW.I_V + handler->UVW.I_W) * 0.33333333333333331f;
    handler->Alpha_Beta.I_Beta = (handler->UVW.I_V - handler->UVW.I_W) * 0.57735026918962573f;
}

/**
 * @brief  Park变换
 *         I_d = I_Alpha*cos(Theta)+I_Beta*sin(Theta);
 *         I_q = -I_Alpha*sin(Theta)+I_Beta*cos(Theta);
 * @param  handler：FOC句柄
 * @retval 无
 */
void FOC_Park(FOC *handler)
{
		static float last_q;
    float cos_Theta;
    float sin_Theta;
    sin_Theta = arm_cos_f32(handler->Theta);
    cos_Theta = arm_sin_f32(handler->Theta);
    handler->qd.I_d = handler->Alpha_Beta.I_Alpha * sin_Theta + handler->Alpha_Beta.I_Beta * cos_Theta;
    handler->qd.I_q = (handler->Alpha_Beta.I_Beta * sin_Theta - handler->Alpha_Beta.I_Alpha * cos_Theta)*0.01f + last_q*0.99f;
		last_q = handler->qd.I_q;		
}

/**
 * @brief  反Park变换
 *         U_Alpha = U_d*cos(Theta) - U_q*sin(Theta);
 *         U_Beta =U_d*sin(Theta) + U_q*cos(Theta);
 * @param  handler：FOC句柄
 * @retval 无
 */

void FOC_AntiPark(FOC *handler)
{
    float cos_Theta;
    float sin_Theta;
    cos_Theta = arm_cos_f32(handler->Theta);
    sin_Theta = arm_sin_f32(handler->Theta);
    handler->Alpha_Beta.U_Alpha = cos_Theta * handler->qd.U_d - sin_Theta * handler->qd.U_q;
    handler->Alpha_Beta.U_Beta = handler->qd.U_d * sin_Theta + handler->qd.U_q * cos_Theta;
}

/**
 * @brief  SVPWM
 * @param  handler：FOC句柄
 * @retval 无
 */
void FOC_SVPWM(FOC *handler)
{
    FOC_AntiPark(handler);
    float rtb_MinMax;
    float rtb_Sum1_c;
    float rtb_Sum_l;
    rtb_MinMax = -0.5 * handler->Alpha_Beta.U_Alpha;
    rtb_Sum1_c = 0.8660254037844386f * handler->Alpha_Beta.U_Beta;
    rtb_Sum_l = rtb_MinMax + rtb_Sum1_c;
    rtb_Sum1_c = rtb_MinMax - rtb_Sum1_c;
    rtb_MinMax = (fmin(fmin(handler->Alpha_Beta.U_Alpha, rtb_Sum_l), rtb_Sum1_c) + fmax(fmax(handler->Alpha_Beta.U_Alpha, rtb_Sum_l), rtb_Sum1_c)) * -0.5;
    handler->UVW.T_V = ((rtb_MinMax + rtb_Sum_l)*0.5f + 0.5f) * PWM_PER;
    handler->UVW.T_W = ((rtb_MinMax + rtb_Sum1_c)*0.5f + 0.5f) * PWM_PER;
    handler->UVW.T_U = ((rtb_MinMax + handler->Alpha_Beta.U_Alpha)*0.5f + 0.5f) * PWM_PER;
    TIM1->CCR1 = handler->UVW.T_V;
    TIM1->CCR2 = handler->UVW.T_U;
    TIM1->CCR3 = handler->UVW.T_W;
//    handler->UVW.T_V = ((rtb_MinMax + rtb_Sum_l) + 1.0f)*0.5f;
//    handler->UVW.T_W = ((rtb_MinMax + rtb_Sum1_c) + 1.0f)*0.5f;
//    handler->UVW.T_U = ((rtb_MinMax + handler->Alpha_Beta.U_Alpha) + 1.0f)*0.5f;
}

/**
 * @brief  FOC闭环pid计算
 * @param  handler：FOC句柄
 * @retval 无
 */
#include "vofa.h"
void FOC_Pid_Cal(FOC *handler)
{
	if(handler->mode == Torque_Mode)
	{
		handler->qd.U_q = pid_calc(&handler->Pid_I, handler->qd.I_q, handler->target);
	}
	else if(handler->mode == Speed_Mode)
	{
		handler->qd.U_q = pid_calc(&handler->Pid_I, handler->qd.I_q, pid_calc(&handler->Pid_Vel, handler->Encoder_measure->speed_rmp, handler->target));
	}
	else if(handler->mode == Speed_Mode)
	{
		handler->qd.U_q = pid_calc(&handler->Pid_I, handler->qd.I_q, pid_calc(&handler->Pid_Vel, handler->Encoder_measure->speed_rmp, pid_calc(&handler->Pid_Pos,handler->Encoder_measure->total_ecd, handler->target)));
	}
		tempFloat[0] = handler->target;
}
