/*
 * Steer.h
 *
 *  Created on: 2021��1��30��
 *      Author: ������
 */

#ifndef CODE_STEER_H_
#define CODE_STEER_H_

#include "motor.h"

#define SD_HZ 300         //��ֵ40   ��ת96       ��ת70   ��ֵ  83
extern int steer_middle;  //�����ֵ
extern int steer_left;    //��ת
extern int steer_right;   //��ת
extern float steer_k_r;   //�����������ϵ��
extern float steer_k_l;
extern int err_v;
extern int outacc_flag;
//#define steer_middle   440
//#define steer_left  356              //������ת������Ҵ�
//#define steer_right 522                    //������ת��������
#define LimitLeft(Left)    (Left = ((Left < steer_left) ? steer_left : Left))
#define LimitRight(Right)  (Right = ((Right > steer_right) ? steer_right : Right))

//#define SD_HZ    50                    //��ֵ40   ��ת96       ��ת70   ��ֵ
//83 #define steer_middle   66//75//69 #define steer_left  79//80//89 //84
////������ת������Ҵ� #define steer_right 51//52//61           //54
////������ת��������
#define SD_Intertal_L 82  //������ת������Ҵ�
#define SD_Intertal_R 84  //������ת��������
//
//
//#define LimitLeft(Left)     (Left=((Left>steer_left)? steer_left:Left))
//#define LimitRight(Right)   (Right=((Right< steer_right) ? steer_right :
//Right))

typedef struct {
  PID_Datatypedef Straight;
  PID_Datatypedef Bend_Large;
  PID_Datatypedef Bend_Small;
} Piord_PD;
extern Piord_PD Po_PID;
extern PID_Datatypedef SteerPIDdata;
extern float Steer_pl;
extern float Steer_ph;
extern float Steer_d;
extern int sterr;
void Steer_init(void);
void SteerControl(uint32 duty);
void SteerPID_Realize(float offset);
void SteerPID_Realize_4(int offset);
void SteerPID_Realize_5(int offset);
void SteerPID_Realize_6(int offset);
void SteerPID_Realize_7(int offset);
void SteerPID_Realize_8(int offset);

#endif /* CODE_STEER_H_ */
