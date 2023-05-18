/*
 * MOTOR.h
 *
 *  Created on: 2021年1月30日
 *      Author: 周文奇
 */
// 3000=52.5CM
#ifndef CODE_MOTOR_H_
#define CODE_MOTOR_H_
#define OX  (50/ 3000.0)  //标度变换

#include "StdIf/IfxStdIf_DPipe.h"

typedef struct {
  float P;
  float I;
  float D;
  int LastError;  // Error[-1]
  int PrevError;  // Error[-2]
  int EC;
  float Kdin;   //入弯D
  float Kdout;  //出弯D

} PID_Datatypedef;

typedef struct {
  float nowspeed;     // pulse表示nowspeed
  int expectspeed;    // speed表示expectspeed
  int motor_duty;     //电机占空比
  float Length;       //走过路程
  int Circle_OUT_th;
  int MinSpeed;             //最低速度
  int MaxSpeed;             //最高速度
  float expect_True_speed;  //实际期望速度
  int straight_speed;       //直道速度
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
