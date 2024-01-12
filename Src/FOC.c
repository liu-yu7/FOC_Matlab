#include "FOC.h"
#include "main.h"
FOC FOC1_Handler;


/**
 * @brief  初始化FOC
 * @param  handler：FOC句柄
 * @retval 无
 */
 int i=0;
void FOC_Init(FOC *handler)
{
    
	//校准开启ADC
	HAL_ADCEx_Calibration_Start(handler->hadc, ADC_SINGLE_ENDED);
	HAL_ADCEx_InjectedStart_IT(handler->hadc);

/*******************************开启定时器*************************************/
	handler->TIM->CCR4 = PWM_PER-2;
	HAL_TIM_Base_Start(handler->htim);
	HAL_TIM_PWM_Start(handler->htim, TIM_CHANNEL_4);
/****************************************************************************/


	
    pid_param_init(&handler->Pid_Vel, POSITION_PID, 5.0f, 3.0f, 0.002f, 0.000005f, 0);
    pid_param_init(&handler->Pid_Pos, POSITION_PID, 3000.0f, 1000.0f, 3.0f, 0.008f, 0);

	//等待准备就绪
	HAL_Delay(1000);
	
	HAL_TIM_PWM_Start(handler->htim, TIM_CHANNEL_1);
	HAL_TIMEx_PWMN_Start(handler->htim, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(handler->htim, TIM_CHANNEL_2);
	HAL_TIMEx_PWMN_Start(handler->htim, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(handler->htim, TIM_CHANNEL_3);
	HAL_TIMEx_PWMN_Start(handler->htim, TIM_CHANNEL_3);
	
	
    
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
    float cos_Theta;
    float sin_Theta;
    sin_Theta = arm_cos_f32(handler->Theta);
    cos_Theta = arm_sin_f32(handler->Theta);
    handler->qd.I_d = handler->Alpha_Beta.I_Alpha * sin_Theta + handler->Alpha_Beta.I_Beta * cos_Theta;
    handler->qd.I_q = handler->Alpha_Beta.I_Beta * sin_Theta - handler->Alpha_Beta.I_Alpha * cos_Theta;
	//接线不同改变方向
	FOC1_Handler.qd.I_d = -FOC1_Handler.qd.I_d;
	FOC1_Handler.qd.I_q  = -FOC1_Handler.qd.I_q;
	//这个低通滤波用于报告
	handler->qd.I_d_filt = handler->qd.I_d_filt*0.95f + handler->qd.I_d*0.05f;
	handler->qd.I_q_filt = handler->qd.I_q_filt*0.95f + handler->qd.I_q*0.05f;
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
    handler->TIM->CCR1 = handler->UVW.T_V;
    handler->TIM->CCR2 = handler->UVW.T_U;
    handler->TIM->CCR3 = handler->UVW.T_W;
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
float limit = 5.0f;
void FOC_Pid_Cal(FOC *handler)
{
	static uint16_t i = 0;
	if(i%4 == 0)
	{
		if(handler->mode == Speed_Mode)
		{
			handler->target_i = pid_calc(&handler->Pid_Vel, handler->Encoder_measure->speed_rmp, handler->target);
		}
		else if(handler->mode == Position_Mode)
		{
			float temp = pid_calc(&handler->Pid_Pos,handler->Encoder_measure->total_ecd, handler->target);
			handler->target_i = pid_calc(&handler->Pid_Vel, handler->Encoder_measure->speed_rmp, temp);
		}
		else
		{
			handler->target_i = handler->target;
		}
	}
	else
	{
		handler->target_i = handler->target;
	}
	
	
	
	static float current_ctrl_integral_d,current_ctrl_integral_q;

	float Ierr_d = 0 - handler->qd.I_d;
	float Ierr_q = handler->target_i - handler->qd.I_q;
	
	float U_d = (handler->usrConfig->Current_p*Ierr_d + current_ctrl_integral_d)/12.0f;
	float U_q = (handler->usrConfig->Current_p*Ierr_q + current_ctrl_integral_q)/12.0f;
	float U;
	arm_sqrt_f32(U_d*U_d + U_q*U_q,&U);
	float scalefactor = 0.6928f/U;
	if(scalefactor < limit)
	{
		U_d *= scalefactor;
		U_q *= scalefactor;
		current_ctrl_integral_d *=0.99f;
		current_ctrl_integral_q *=0.99f;
	}
	else
	{
		current_ctrl_integral_d +=Ierr_d * (handler->usrConfig->Current_i * 0.00005f);
		current_ctrl_integral_q +=Ierr_q * (handler->usrConfig->Current_i * 0.00005f);
	}
	handler->qd.U_d = U_d;
	handler->qd.U_q = U_q;
	
		tempFloat[0] = handler->target;
}


/**
 * @brief  外部中断回调函数
 * @param  GPIO_PIN:外部中断的引脚号
 * @retval none
 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_PIN)
{
  if(GPIO_PIN == KEY1_Pin)
	{
	}
}

/**
 * @brief  注入组完成回调函数
 * @param  hadc:adc句柄
 * @retval none
 */
void HAL_ADCEx_InjectedConvCpltCallback(ADC_HandleTypeDef *hadc)
{
	if(FOC1_Handler.state == Cilibration)
	{	
		static uint8_t i=0;
		FOC1_Handler.usrConfig->IN1_offset += hadc->Instance->JDR1;
		FOC1_Handler.usrConfig->IN2_offset += hadc->Instance->JDR2;
		if(i++ == 199)
		{
			FOC1_Handler.usrConfig->IN1_offset = FOC1_Handler.usrConfig->IN1_offset/200;
			FOC1_Handler.usrConfig->IN2_offset = FOC1_Handler.usrConfig->IN2_offset/200;
			FOC1_Handler.state = Ready;
		}
	}
  /*******************************电流计算*************************************/
  //采样电阻0.001运放x30.4818，电流I=adc/4096*3.3/30.4818/0.001
  FOC1_Handler.UVW.I_V = ((int32_t)hadc->Instance->JDR1 - FOC1_Handler.usrConfig->IN1_offset)*0.0264309870972f*1.129033f;
  FOC1_Handler.UVW.I_W = ((int32_t)hadc->Instance->JDR2 - FOC1_Handler.usrConfig->IN2_offset)*0.0264309870972f*1.0f;
  FOC1_Handler.UVW.I_U = (-FOC1_Handler.UVW.I_V - FOC1_Handler.UVW.I_W);     //I_U+I_V+I_W=0
  /***************************************************************************/

  /*******************************角度处理*************************************/
  MT6701_GetAngle(&MT6701);
  //转化为电角度，*0.0015339825195f转化为弧度
  FOC1_Handler.Theta = (MT6701.ecd - FOC1_Handler.usrConfig->Ele_offset)*FOC1_Handler.usrConfig->Pole_pairs*0.0015339825195f;
  /***************************************************************************/

  FOC_Clark(&FOC1_Handler);           //由I_UVW得到I_αβ
  FOC_Park(&FOC1_Handler);            //由I_αβ得到I_qd

  /*******************************Pid计算*************************************/
  FOC_Pid_Cal(&FOC1_Handler);

  /***************************************************************************/
	FOC_SVPWM(&FOC1_Handler);
}