/*
 * Steer.c
 *
 *  Created on: 2021年1月30日
 *      Author: 周文奇
 */

#include "Steer.h"

#include "C_H.h"
#include "Fuzzy.h"
#include "System.h"
#include "buzzer.h"
#include "headfile.h"
int steer_middle;                //舵机中值
int steer_left;                  //左转
int steer_right;                 //右转
PID_Datatypedef SteerPIDdata;    //舵机的PID参数
PID_Datatypedef SteerPIDdata_2;  //动态二次
Piord_PD Po_PID;



void Steer_init(void) {
  gtm_pwm_init(ATOM0_CH1_P33_9, SD_HZ, 0);
}

void SteerControl(uint32 duty) {
  LimitLeft(duty);
  LimitRight(duty);  //限幅

  pwm_duty(ATOM0_CH1_P33_9, duty);
}

/*------------------------------------------------------------------------------------------------------
【函    数】SteerPID_Realize位置式
【功    能】舵机PID
【参    数】offset 偏差
【返 回 值】无
--------------------------------------------------------------------------------------------------------*/
void SteerPID_Realize(float offset) {
  //当前误差，定义为寄存器变量，只能用于整型和字符型变量，提高运算速度

  float iError,  //当前误差
      SteerErr;  //
  int PWM;
  //  static int err_sum = 0;
  iError = offset;  //计算当前误差

  SteerErr =
      SteerPIDdata.P * iError +
      SteerPIDdata.D * (iError - SteerPIDdata.LastError);  //位置式PID算式
  SteerPIDdata.LastError = iError;                         //更新上次误差
  PWM = steer_middle - SteerErr;

  if (PWM < 0)
    PWM = 0;

  LimitLeft(PWM);
  LimitRight(PWM);  //限幅

  //    if(SystemData.Point>70) { PWM=steer_right;}
  //    if(SystemData.Point<10) { PWM=steer_left;}
  SteerControl(PWM);
}

/*------------------------------------------------------------------------------------------------------
【函    数】动态二次P
【功    能】舵机PID
【参    数】offset 偏差
【返 回 值】无
--------------------------------------------------------------------------------------------------------*/

float Steer_pl; 
float Steer_ph;
float Steer_d;
int critic_offset = 7;

void SteerPID_Realize_2(int offset) {
  //定义为寄存器变量，只能用于整型和字符型变量，提高运算速度
  int32 iError, SteerErr;  //当前误差
  static int32 LastError;  //前次误差
  int PWM;
  float Kp;  //动态P
  float Kd;
  int postive_offset;
  iError = offset;  //计算当前误差
  postive_offset = abs(offset);
  Kp = 0.1 * (iError * iError) / (Steer_pl) + (Steer_ph);   // P值与差值成二次函数关系

  //    //试试动态D
  //    Kd = (iError * LastError > 0) ?
  //            3+Steer_d*postive_offset/20:        // 入弯
  //            4+Steer_d*postive_offset/20;        // 出弯

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

  SteerErr = Kp * iError + (Kd) * (iError - LastError) + 0.5;  //只用PD
  SteerPIDdata.EC = iError - LastError;  // EC>0 右转  EC<左转
  LastError = iError;                    //更新上次误差

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
  //定义为寄存器变量，只能用于整型和字符型变量，提高运算速度
  int32 iError, SteerErr;  //当前误差
  static int32 LastError;  //前次误差
  int PWM;
  float Kp = 0;  //动态P
  int postive_offset;
  iError = offset;  //计算当前误差
  postive_offset = abs(offset);

  //静态P衔接动态2次
  if (postive_offset <= critic_offset)
    Kp = Steer_ph;
  else if (offset > critic_offset)
    Kp = (iError - critic_offset) * (iError - critic_offset) / (Steer_pl) +
         (Steer_ph);  // P值与差值成二次函数关系
  else if (offset < (-critic_offset))
    Kp = (iError + critic_offset) * (iError + critic_offset) / (Steer_pl) +
         (Steer_ph);

  SteerErr = Kp * iError + (Steer_d) * (iError - LastError) + 0.5;  //只用PD

  LastError = iError;  //更新上次误差

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
  int32 iError, SteerErr;  //当前误差
  static int32 LastError;  //前次误差
  int PWM;
  float Kd;
  int postive_offset;
  iError = offset;  //计算当前误差
  postive_offset = abs(offset);

  Kd = (iError * LastError > 0) ? iError * (8 + postive_offset / 300) / 20
                                :                      // 入弯
  iError * (12 + postive_offset / 300) / 20;  // 出弯

  // t = BigArcFlag ? 4 : 0;     // 大圆弧

  SteerErr = (postive_offset / 10) * (iError / 10) / (Steer_pl) +
             iError * (Steer_d) / 300 + Kd;

  LastError = iError;  //更新上次误差

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
【函    数】模糊控制
【功    能】舵机PID
【参    数】offset 偏差
【返 回 值】无
--------------------------------------------------------------------------------------------------------*/
void SteerPID_Realize_5(int offset)  //位置式    //模糊的偏差维度参数发生变化
{
  //当前误差，定义为寄存器变量，只能用于整型和字符型变量，提高运算速度

  int iError,    //当前误差
      SteerErr;  //
  int PWM;
  iError = offset;  //计算当前误差
  DuoJi_GetP((float*)&ImageStatus.MU_P, ImageStatus.OFFLine,offset);  //经典的模糊控制
                       //

  //    DuoJi_GetP((float*)&ImageStatus.MU_P,ImageStatus.OFFLine,ImageStatus.Det_all);//经典的模糊控制试试
  //    if(ImageStatus.Road_type==LeftCirque||ImageStatus.Road_type==RightCirque)
  //        {
  //        if(SystemData.SpeedData.Length*OX<2500)//双环斜率不一样//g5.23
  //            ImageStatus.MU_P=((float)SystemData.CircleP)/100+0.02;//圆环给单P
  //        else
  //            ImageStatus.MU_P=((float)SystemData.CircleP)/100;//圆环给单P
  //
  //        }
  //     if(Fork_dowm==1)
  //         ImageStatus.MU_P=ImageStatus.MU_P-0.02;
  SteerErr = (int)(ImageStatus.MU_P * iError +
                   SteerPIDdata.D * (iError - SteerPIDdata.LastError) +
                   0.5);            //位置式PID算式
  SteerPIDdata.LastError = iError;  //更新上次误差

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
float steer_k_l;                     //左右弯道补偿系数
int err_v;                           //偏差控速因子
void SteerPID_Realize_6(int offset)  //位置式  //入弯出弯D不一样  这个有戏
{
  //当前误差，定义为寄存器变量，只能用于整型和字符型变量，提高运算速度

  int iError,    //当前误差
      SteerErr;  //
  int PWM;
  float kd;
  int max_sterr;
  static int d_error = 0;
  iError = offset;  //计算当前误差
  DuoJi_GetP((float*)&ImageStatus.MU_P, ImageStatus.OFFLine,
             offset);  //经典的模糊控制//p
  d_error =
      (int16)((iError - SteerPIDdata.LastError) * (0.5) + d_error * (0.5));
  //  d_error = (int16)(pp->error - pp->prepreError);
  //    DuoJi_GetP((float*)&ImageStatus.MU_P,ImageStatus.OFFLine,ImageStatus.Det_all);//经典的模糊控制试试
  if ((d_error > 0 && iError < 0) || (d_error < 0 && iError > 0)) {  //出弯道
    kd = (SteerPIDdata.Kdout);
    buzzer_on();
  } else {  //入弯道
    kd = (SteerPIDdata.Kdin);
    buzzer_off();
  }

  SteerErr =(int)(ImageStatus.MU_P * iError + kd * (iError - SteerPIDdata.LastError) +0.5);                   //位置式PID算式
  SteerPIDdata.LastError = iError;  //更新上次误差

  if (SteerErr >= 0)  //右转
  {
    PWM = steer_middle + steer_k_r * SteerErr;  //-+
    err_v = steer_k_r * SteerErr;
  } else  //左转
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

void SteerPID_Realize_7(int offset)  //位置式    D和P按模糊系数来
{
  //当前误差，定义为寄存器变量，只能用于整型和字符型变量，提高运算速度

  int iError,    //当前误差
      SteerErr;  //
  int PWM;
  float k;
  float kd;
  iError = offset;  //计算当前误差
  DuoJi_GetP((float*)&ImageStatus.MU_P, ImageStatus.OFFLine,offset);  //经典的模糊控制
  //    DuoJi_GetP((float*)&ImageStatus.MU_P,ImageStatus.OFFLine,ImageStatus.Det_all);//经典的模糊控制试试

  k = ImageStatus.MU_P / MHstruct.f_DuoJiP_TableL[1];
  kd = SteerPIDdata.D * k;  //
  SteerErr =(int)(ImageStatus.MU_P * iError + kd * (iError - SteerPIDdata.LastError) + 0.5);                   //位置式PID算式
  SteerPIDdata.LastError = iError;  //更新上次误差

  PWM = steer_middle + SteerErr;  //-+

  if (PWM < 0)
    PWM = 0;
  SteerControl(PWM);
}

int sterr;
int outacc_flag;                       //出弯道加速
void SteerPID_Realize_8(int offset) {  //位置式    D和P按模糊系数来

  int iError,    //当前误差
      SteerErr;  //
  int PWM;
  float kp;
  float kd;
  static int d_error = 0;
  int max_sterr;    //圆环限幅
  iError = offset;  //计算当前误差
  DuoJi_GetP((float*)&ImageStatus.MU_P, ImageStatus.OFFLine,offset);  //经典的模糊控制//p
  d_error = (int16)((iError - SteerPIDdata.LastError) * (0.5) + d_error * (0.5));
  if ((d_error > 0 && iError < 0) || (d_error < 0 && iError > 0)) {  //出弯道
    kd = (SteerPIDdata.Kdout);
    outacc_flag = 1;

  } else {  //入弯道
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



  SteerErr = (int)(ImageStatus.MU_P * iError + kd * (iError - SteerPIDdata.LastError) +0.5);                   //位置式PID算式
  SteerPIDdata.LastError = iError;  //更新上次误差
  sterr = SteerErr;

  if (ImageStatus.Road_type == Ramp)
    SteerErr = Limit(SteerErr, 20, -20);  // 73

  if (SteerErr >= 0)                            //右转
    PWM = steer_middle + steer_k_r * SteerErr;  //-+
  else                                          //左转
    PWM = steer_middle + steer_k_l * SteerErr;

  if (PWM < 0)
    PWM = 0;

  SteerControl(PWM);
}
