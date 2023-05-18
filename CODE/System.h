/*
 * System.h
 *
 *  Created on: 2021��1��30��
 *      Author: ������
 */
#ifndef CODE_SYSTEM_H_
#define CODE_SYSTEM_H_

#include "IfxCpu_Intrinsics.h"
#include "Platform_Types.h"
#include "shell.h"
#include "MOTOR.h"
#include "vt100.h"
#include "zf_stm_systick.h"
#include <math.h>

typedef struct {
  SpeedDatatypedef SpeedData;
  uint8 SteerOK;      //���������־
  uint8 CameraOK;     //����ͷ������ʾ��־
  uint8 OldCameraOK;  //�Ҷȴ����־
  uint8 MotorOK;      //�������
  uint8 Point;        //����ʵ������
  uint8 UpdataOK;     //���ݸ���
  uint8 Stop;         //ֹͣ��־
  uint8 GO_OK;        //����
  int Model;          //����ģʽ

  //ͼ�����
  int OutCicle_line;  //�����ж�ͼ���ֹ��  Խ������ж�Խ�ϸ�Խ��
  int L_T_R_W;  // �������б����ޱ�����  Խ���������Խ�ϸ�
  int Circleoff_offline;  //����Զ���ľ����жϳ�������  ԽСԽ�ϸ�
  int CircleP;            //����P


//�����о���
  int fork_start_lenth;  //��·�ڼ����ʼ
  int fork_off_lenth;//��·�ڼ���ֹ

  int circle_off_lenth;//���������о���

  //СԲ��
  int circles_pl;
  int circles_pr;
  int circles_off_lenth;//СԲ�����������о���

   //��Բ��
  int circlem_pl;
  int circlem_pr;
  int circlem_off_lenth;//СԲ�����������о���

   //��Բ��
  int circlel_pl;
  int circlel_pr;
  int circlel_off_lenth;//СԲ�����������о���
  int clrcle_priority[3];//Բ������
  int clrcle_num;// �ڵڼ���Բ��


  int circle_kin;//�뻷���߰뾶
  float circle_kout;//��������б��
  int circle_max_ang;//���ڴ���޷�


  //ֱ��
  float straight_p;//ֱ��P
  float straight_d;//ֱ��D 
  int   straighet_towpoint;//ֱ��ǰհ


  int debug_lenth;//���Ծ���

  //����ͷ����
  int exp_time; //�ع�ʱ��
  int mtv_lroffset;//����ͷ����ƫ��
  int mtv_gain;//����ͷ����

  int ramp_lenth_start;//�µ�����
  int fork_lenth_start;//�������
  int barn_lenth;//Բ������

  int outbent_acc;//�������


  int rounds; // Ȧ��
  int speed_per_round; // ÿȦ���ٶ���

} SystemDatatypdef;

extern SystemDatatypdef SystemData;
void GetSystemData(void);
int Limit(int num, int numH, int numL);
float get_speed_convert(float val);
int set_speed_convert(float speed);

static IFX_INLINE float abs(float val) {
  return Ifx__absf(val);
}

#define RUN_TIME(func)                             \
  {                                                \
    systick_start(STM0);                           \
    func;                                          \
    float t = systick_getval(STM0);                \
    vt_clear_attr();                               \
    PRINTF("func ");                               \
    vt_set_bg_color(VT_F_PURPLE);                  \
    PRINTF(#func);                                 \
    vt_clear_attr();                               \
    PRINTF(" takes %.3fms in cpu", t / 100000.0f); \
    vt_set_bg_color(VT_F_CYAN);                    \
    PRINTF("%d.\n\r", IfxCpu_getCoreIndex());      \
    vt_clear_attr();                               \
  }

#endif /* CODE_SYSTEM_H_ */
