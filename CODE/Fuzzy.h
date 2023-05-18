/*
 * Fuzzy.h
 *
 *  Created on: 2021��2��22��
 *      Author: ������
 */

#ifndef CODE_FUZZY_H_
#define CODE_FUZZY_H_

#include "Platform_Types.h"

typedef struct {
  //����ģ����
  uint8         ui8_Table[4][4];
} MH_Table;


typedef struct {
  //���Pֵ��L
  float         f_DuoJiP_TableL[7];
  //���Pֵ��R
  float         f_DuoJiP_TableR[7];//û��  ����һ��  û��Ҫ����������� �鷳
  //���ģ����
  MH_Table      mt_Duoji[4];
    //���Pֵ��
  float         f_DianJiP_Table[7];   //û��
  //���Iֵ��
  float         f_DianJiI_Table[7];   //û��
  //���ģ����
  MH_Table      mt_DianJi[4];            //û��
  //�������־
  uint8         ui8_IO;              //û��
  //���ҳ������־
  uint8         ui8_IOLR;            //û��
  //������Ӽ��ٱ�־
  uint8         ui8_IOAS;          //û��
  //������Ӿ���仯��Χ
  float         f_SizeOfViewH;      //ͼ����Ұ  ������ͷ�ĽǶȾ���
  //��ֵƫ��仯��Χ
  float         f_SizeOfViewE;
    //����ƫ��仯��Χ                          //û�ÿ�����
  float         f_SizeOfPulseE;
  //�ϴη�����Ӿ���                                //��OFFLINEһ����˼
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
