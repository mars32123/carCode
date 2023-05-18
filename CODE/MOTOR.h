/*
 * MOTOR.h
 *
 *  Created on: 2021��1��30��
 *      Author: ������
 */
// 3000=52.5CM
#ifndef CODE_MOTOR_H_
#define CODE_MOTOR_H_
#define OX  (50/ 3000.0)  //��ȱ任

#include "StdIf/IfxStdIf_DPipe.h"

typedef struct {
  float P;
  float I;
  float D;
  int LastError;  // Error[-1]
  int PrevError;  // Error[-2]
  int EC;
  float Kdin;   //����D
  float Kdout;  //����D

} PID_Datatypedef;

typedef struct {
  float nowspeed;     // pulse��ʾnowspeed
  int expectspeed;    // speed��ʾexpectspeed
  int motor_duty;     //���ռ�ձ�
  float Length;       //�߹�·��
  int Circle_OUT_th;
  int MinSpeed;             //����ٶ�
  int MaxSpeed;             //����ٶ�
  float expect_True_speed;  //ʵ�������ٶ�
  int straight_speed;       //ֱ���ٶ�
} SpeedDatatypedef;

boolean max_voffline_fun(pchar args, void* data, IfxStdIf_DPipe* io);
extern PID_Datatypedef MotorPID_Data;
void MotorPWM_output(int M);
void MotorPWM_outputE(int M);
void GetSpeed(SpeedDatatypedef* Q);
void MOTOR(int setspeed);
void Control_Speed(void);
void MOTOR_init(void);
void Encoder_init(void);

#endif /* CODE_MOTOR_H_ */
