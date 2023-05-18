/*
 * Fuzzy.h
 *
 *  Created on: 2021年2月22日
 *      Author: 周文奇
 */

#ifndef CODE_FUZZY_H_
#define CODE_FUZZY_H_

#include "Platform_Types.h"

typedef struct {
  //基础模糊表
  uint8         ui8_Table[4][4];
} MH_Table;


typedef struct {
  //舵机P值表L
  float         f_DuoJiP_TableL[7];
  //舵机P值表R
  float         f_DuoJiP_TableR[7];//没用  就用一个  没必要用两个舵机表 麻烦
  //舵机模糊表
  MH_Table      mt_Duoji[4];
    //电机P值表
  float         f_DianJiP_Table[7];   //没用
  //电机I值表
  float         f_DianJiI_Table[7];   //没用
  //电机模糊表
  MH_Table      mt_DianJi[4];            //没用
  //出入弯标志
  uint8         ui8_IO;              //没用
  //左右出入弯标志
  uint8         ui8_IOLR;            //没用
  //出入弯加减速标志
  uint8         ui8_IOAS;          //没用
  //反向可视距离变化范围
  float         f_SizeOfViewH;      //图像视野  由摄像头的角度决定
  //中值偏差变化范围
  float         f_SizeOfViewE;
    //脉冲偏差变化范围                          //没用看不懂
  float         f_SizeOfPulseE;
  //上次反向可视距离                                //和OFFLINE一个意思
  sint16         i16_ViewH;
  //
} MH;
float f_Get_E_approximation(sint16 i16_E, float f_E_Size);
float f_Get_H_approximation(sint16 i16_ViewH);
void DuoJi_GetP(float *i32p_P, sint16 i16_ViewH, sint16 i16_ViewE);
void InitMH(void);

extern MH      MHstruct;
extern MH      MHstructFastCar;


#endif /* CODE_FUZZY_H_ */
