/*
 * Steer.c
 *
 *  Created on: 2021��1��30��
 *      Author: ������
 */

#include "Steer.h"

#include "C_H.h"
#include "Fuzzy.h"
#include "System.h"
#include "buzzer.h"
#include "headfile.h"
int steer_middle;                //�����ֵ
int steer_left;                  //��ת
int steer_right;                 //��ת
PID_Datatypedef SteerPIDdata;    //�����PID����
PID_Datatypedef SteerPIDdata_2;  //��̬����
Piord_PD Po_PID;



void Steer_init(void) {
  gtm_pwm_init(ATOM0_CH1_P33_9, SD_HZ, 0);
}

void SteerControl(uint32 duty) {
  LimitLeft(duty);
  LimitRight(duty);  //�޷�

  pwm_duty(ATOM0_CH1_P33_9, duty);
}

/*------------------------------------------------------------------------------------------------------
����    ����SteerPID_Realizeλ��ʽ
����    �ܡ����PID
����    ����offset ƫ��
���� �� ֵ����
--------------------------------------------------------------------------------------------------------*/
void SteerPID_Realize(float offset) {
  //��ǰ������Ϊ�Ĵ���������ֻ���������ͺ��ַ��ͱ�������������ٶ�

  float iError,  //��ǰ���
      SteerErr;  //
  int PWM;
  //  static int err_sum = 0;
  iError = offset;  //���㵱ǰ���

  SteerErr =
      SteerPIDdata.P * iError +
      SteerPIDdata.D * (iError - SteerPIDdata.LastError);  //λ��ʽPID��ʽ
  SteerPIDdata.LastError = iError;                         //�����ϴ����
  PWM = steer_middle - SteerErr;

  if (PWM < 0)
    PWM = 0;

  LimitLeft(PWM);
  LimitRight(PWM);  //�޷�

  //    if(SystemData.Point>70) { PWM=steer_right;}
  //    if(SystemData.Point<10) { PWM=steer_left;}
  SteerControl(PWM);
}

/*------------------------------------------------------------------------------------------------------
����    ������̬����P
����    �ܡ����PID
����    ����offset ƫ��
���� �� ֵ����
--------------------------------------------------------------------------------------------------------*/

float Steer_pl; 
float Steer_ph;
float Steer_d;
int critic_offset = 7;

void SteerPID_Realize_2(int offset) {
  //����Ϊ�Ĵ���������ֻ���������ͺ��ַ��ͱ�������������ٶ�
  int32 iError, SteerErr;  //��ǰ���
  static int32 LastError;  //ǰ�����
  int PWM;
  float Kp;  //��̬P
  float Kd;
  int postive_offset;
  iError = offset;  //���㵱ǰ���
  postive_offset = abs(offset);
  Kp = 0.1 * (iError * iError) / (Steer_pl) + (Steer_ph);   // Pֵ���ֵ�ɶ��κ�����ϵ

  //    //���Զ�̬D
  //    Kd = (iError * LastError > 0) ?
  //            3+Steer_d*postive_offset/20:        // ����
  //            4+Steer_d*postive_offset/20;        // ����

  // if(postive_offset<=8)
  //    Kd=Steer_d-5;
  // else if(postive_offset<=12)
  //    Kd=Steer_d-4;
  // else if(postive_offset<=16)
  //    Kd=Steer_d-3;
  // else if(postive_offset<=20)
  //    Kd=Steer_d-2;
  // else if(postive_offset<=24)
  //    Kd=Steer_d-1;
  // else
  Kd = Steer_d;

  SteerErr = Kp * iError + (Kd) * (iError - LastError) + 0.5;  //ֻ��PD
  SteerPIDdata.EC = iError - LastError;  // EC>0 ��ת  EC<��ת
  LastError = iError;                    //�����ϴ����

  PWM = steer_middle + SteerErr;
  //    if(PWM<0)
  //        PWM=0;
  //    if(SystemData.Point>70) { PWM=steer_right;}
  //    if(SystemData.Point<10) { PWM=steer_left;}

  if (PWM < 0)
    PWM = 0;

  SteerControl(PWM);
}

void SteerPID_Realize_3(int offset) {
  //����Ϊ�Ĵ���������ֻ���������ͺ��ַ��ͱ�������������ٶ�
  int32 iError, SteerErr;  //��ǰ���
  static int32 LastError;  //ǰ�����
  int PWM;
  float Kp = 0;  //��̬P
  int postive_offset;
  iError = offset;  //���㵱ǰ���
  postive_offset = abs(offset);

  //��̬P�νӶ�̬2��
  if (postive_offset <= critic_offset)
    Kp = Steer_ph;
  else if (offset > critic_offset)
    Kp = (iError - critic_offset) * (iError - critic_offset) / (Steer_pl) +
         (Steer_ph);  // Pֵ���ֵ�ɶ��κ�����ϵ
  else if (offset < (-critic_offset))
    Kp = (iError + critic_offset) * (iError + critic_offset) / (Steer_pl) +
         (Steer_ph);

  SteerErr = Kp * iError + (Steer_d) * (iError - LastError) + 0.5;  //ֻ��PD

  LastError = iError;  //�����ϴ����

  PWM = steer_middle - SteerErr;
  if (PWM < 0)
    PWM = 0;
  if (SystemData.Point > 70) {
    PWM = steer_right;
  }
  if (SystemData.Point < 10) {
    PWM = steer_left;
  }
  SteerControl(PWM);
}

void SteerPID_Realize_4(int offset) {
  int32 iError, SteerErr;  //��ǰ���
  static int32 LastError;  //ǰ�����
  int PWM;
  float Kd;
  int postive_offset;
  iError = offset;  //���㵱ǰ���
  postive_offset = abs(offset);

  Kd = (iError * LastError > 0) ? iError * (8 + postive_offset / 300) / 20
                                :                      // ����
  iError * (12 + postive_offset / 300) / 20;  // ����

  // t = BigArcFlag ? 4 : 0;     // ��Բ��

  SteerErr = (postive_offset / 10) * (iError / 10) / (Steer_pl) +
             iError * (Steer_d) / 300 + Kd;

  LastError = iError;  //�����ϴ����

  PWM = steer_middle + SteerErr;
  if (PWM < 0)
    PWM = 0;
  if (SystemData.Point > 70) {
    PWM = steer_right;
  }
  if (SystemData.Point < 10) {
    PWM = steer_left;
  }
  SteerControl(PWM);
}

/*------------------------------------------------------------------------------------------------------
����    ����ģ������
����    �ܡ����PID
����    ����offset ƫ��
���� �� ֵ����
--------------------------------------------------------------------------------------------------------*/
void SteerPID_Realize_5(int offset)  //λ��ʽ    //ģ����ƫ��ά�Ȳ��������仯
{
  //��ǰ������Ϊ�Ĵ���������ֻ���������ͺ��ַ��ͱ�������������ٶ�

  int iError,    //��ǰ���
      SteerErr;  //
  int PWM;
  iError = offset;  //���㵱ǰ���
  DuoJi_GetP((float*)&ImageStatus.MU_P, ImageStatus.OFFLine,offset);  //�����ģ������
                       //

  //    DuoJi_GetP((float*)&ImageStatus.MU_P,ImageStatus.OFFLine,ImageStatus.Det_all);//�����ģ����������
  //    if(ImageStatus.Road_type==LeftCirque||ImageStatus.Road_type==RightCirque)
  //        {
  //        if(SystemData.SpeedData.Length*OX<2500)//˫��б�ʲ�һ��//g5.23
  //            ImageStatus.MU_P=((float)SystemData.CircleP)/100+0.02;//Բ������P
  //        else
  //            ImageStatus.MU_P=((float)SystemData.CircleP)/100;//Բ������P
  //
  //        }
  //     if(Fork_dowm==1)
  //         ImageStatus.MU_P=ImageStatus.MU_P-0.02;
  SteerErr = (int)(ImageStatus.MU_P * iError +
                   SteerPIDdata.D * (iError - SteerPIDdata.LastError) +
                   0.5);            //λ��ʽPID��ʽ
  SteerPIDdata.LastError = iError;  //�����ϴ����

  //    if(SteerErr >= SD_Intertal_L)
  //        SteerErr = SD_Intertal_L;
  //    else if(SteerErr <= -SD_Intertal_R)
  //        SteerErr = -SD_Intertal_R;
  //

  PWM = steer_middle + SteerErr;  //-+

  if (PWM < 0)
    PWM = 0;

  //    if(SystemData.Point>70) { PWM=steer_right;}
  //    if(SystemData.Point<10) { PWM=steer_left;}
  SteerControl(PWM);
}

float steer_k_r;
float steer_k_l;                     //�����������ϵ��
int err_v;                           //ƫ���������
void SteerPID_Realize_6(int offset)  //λ��ʽ  //�������D��һ��  �����Ϸ
{
  //��ǰ������Ϊ�Ĵ���������ֻ���������ͺ��ַ��ͱ�������������ٶ�

  int iError,    //��ǰ���
      SteerErr;  //
  int PWM;
  float kd;
  int max_sterr;
  static int d_error = 0;
  iError = offset;  //���㵱ǰ���
  DuoJi_GetP((float*)&ImageStatus.MU_P, ImageStatus.OFFLine,
             offset);  //�����ģ������//p
  d_error =
      (int16)((iError - SteerPIDdata.LastError) * (0.5) + d_error * (0.5));
  //  d_error = (int16)(pp->error - pp->prepreError);
  //    DuoJi_GetP((float*)&ImageStatus.MU_P,ImageStatus.OFFLine,ImageStatus.Det_all);//�����ģ����������
  if ((d_error > 0 && iError < 0) || (d_error < 0 && iError > 0)) {  //�����
    kd = (SteerPIDdata.Kdout);
    buzzer_on();
  } else {  //�����
    kd = (SteerPIDdata.Kdin);
    buzzer_off();
  }

  SteerErr =(int)(ImageStatus.MU_P * iError + kd * (iError - SteerPIDdata.LastError) +0.5);                   //λ��ʽPID��ʽ
  SteerPIDdata.LastError = iError;  //�����ϴ����

  if (SteerErr >= 0)  //��ת
  {
    PWM = steer_middle + steer_k_r * SteerErr;  //-+
    err_v = steer_k_r * SteerErr;
  } else  //��ת
  {
    PWM = steer_middle + steer_k_l * SteerErr;
    err_v = -steer_k_l * SteerErr;
  }

  if (PWM < 0)
    PWM = 0;

  //    if(SystemData.Point>70) { PWM=steer_right;}
  //    if(SystemData.Point<10) { PWM=steer_left;}
  SteerControl(PWM);
}

void SteerPID_Realize_7(int offset)  //λ��ʽ    D��P��ģ��ϵ����
{
  //��ǰ������Ϊ�Ĵ���������ֻ���������ͺ��ַ��ͱ�������������ٶ�

  int iError,    //��ǰ���
      SteerErr;  //
  int PWM;
  float k;
  float kd;
  iError = offset;  //���㵱ǰ���
  DuoJi_GetP((float*)&ImageStatus.MU_P, ImageStatus.OFFLine,offset);  //�����ģ������
  //    DuoJi_GetP((float*)&ImageStatus.MU_P,ImageStatus.OFFLine,ImageStatus.Det_all);//�����ģ����������

  k = ImageStatus.MU_P / MHstruct.f_DuoJiP_TableL[1];
  kd = SteerPIDdata.D * k;  //
  SteerErr =(int)(ImageStatus.MU_P * iError + kd * (iError - SteerPIDdata.LastError) + 0.5);                   //λ��ʽPID��ʽ
  SteerPIDdata.LastError = iError;  //�����ϴ����

  PWM = steer_middle + SteerErr;  //-+

  if (PWM < 0)
    PWM = 0;
  SteerControl(PWM);
}

int sterr;
int outacc_flag;                       //���������
void SteerPID_Realize_8(int offset) {  //λ��ʽ    D��P��ģ��ϵ����

  int iError,    //��ǰ���
      SteerErr;  //
  int PWM;
  float kp;
  float kd;
  static int d_error = 0;
  int max_sterr;    //Բ���޷�
  iError = offset;  //���㵱ǰ���
  DuoJi_GetP((float*)&ImageStatus.MU_P, ImageStatus.OFFLine,offset);  //�����ģ������//p
  d_error = (int16)((iError - SteerPIDdata.LastError) * (0.5) + d_error * (0.5));
  if ((d_error > 0 && iError < 0) || (d_error < 0 && iError > 0)) {  //�����
    kd = (SteerPIDdata.Kdout);
    outacc_flag = 1;

  } else {  //�����
    kd = (SteerPIDdata.Kdin);
    outacc_flag = 0;
  }

  if (ImageStatus.Road_type == LeftCirque&&ImageStatus.CirqueOff=='F') {

        ImageStatus.MU_P = (float)SystemData.circles_pl / 100;
        circle_r = 16;    // 9
        circlk_k = 0.78;  // 0.98
        max_sterr = 78;   // 73
        circleoff_lenth = 150;


  }

  if (ImageStatus.Road_type == RightCirque&&ImageStatus.CirqueOff=='F') {

        ImageStatus.MU_P = (float)SystemData.circles_pr / 100;
        circle_r =14;
        circlk_k = 0.83;
        max_sterr = 78;
        circleoff_lenth = 150;

  }


  if (ImageStatus.Road_type == Straight) {
    ImageStatus.MU_P = SystemData.straight_p;
    kd = SystemData.straight_d;
  }



  SteerErr = (int)(ImageStatus.MU_P * iError + kd * (iError - SteerPIDdata.LastError) +0.5);                   //λ��ʽPID��ʽ
  SteerPIDdata.LastError = iError;  //�����ϴ����
  sterr = SteerErr;

  if (ImageStatus.Road_type == Ramp)
    SteerErr = Limit(SteerErr, 20, -20);  // 73

  if (SteerErr >= 0)                            //��ת
    PWM = steer_middle + steer_k_r * SteerErr;  //-+
  else                                          //��ת
    PWM = steer_middle + steer_k_l * SteerErr;

  if (PWM < 0)
    PWM = 0;

  SteerControl(PWM);
}
