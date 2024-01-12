#ifndef __MT6701_H__
#define __MT6701_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

typedef struct
{
  uint16_t ecd;             //绝对角度
  uint16_t last_ecd;        //上次角度
  float    angle;           //绝对角度（360°）
  int32_t  rad;             //圈数
  int32_t  total_ecd;       //总角度
  int32_t  last_total_ecd;  //上次总角度
  float    speed_rmp;       //速度（圈/s）

} MT6701_measure;

extern MT6701_measure MT6701;


void MT6701_Init(MT6701_measure *mt6701);
void MT6701_GetAngle(MT6701_measure *mt6701);
void MT6701_Speed_Cal_10khz(MT6701_measure *mt6071);







#endif