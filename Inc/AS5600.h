#ifndef __AS5600_H__
#define __AS5600_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include "IIC.h"

typedef struct
{
  uint16_t ecd;             //绝对角度
  uint16_t last_ecd;        //上次角度
  float    angle;           //绝对角度（360°）
  int32_t  rad;             //圈数
  int32_t  total_ecd;       //总角度
  int32_t  last_total_ecd;  //上次总角度
  float    speed_rmp;       //速度（圈/s）

} AS5600_measure;

extern AS5600_measure AS5600;

void AS5600_Init(AS5600_measure *as5600);
void AS5600_Get_Angle(AS5600_measure *as5600);
void AS5600_Speed_Cal_10khz(AS5600_measure *as5600);
  
#endif
