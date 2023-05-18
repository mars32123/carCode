/*
 * MOTOR.c
 *
 *  Created on: 2021年1月30日
 *      Author: 周文奇
 */

#include "MOTOR.h"
#include "C_H.h"
#include "DataStore.h"
#include "Steer.h"
#include "System.h"
#include "car.h"
#include "icm.h"
#include "shell.h"
#include "zf_gpt12.h"
#include "zf_gtm_pwm.h"
////根据测速方式选择，如果是正交解码则可以到负值，现仅为正值
#define PWM_H (950)
#define PWM_L (-950)

#define PWM_Bang 950
#define Divaite_Bang 20

extern int Fork_dowm;
void MOTOR_init(void) {
  gtm_pwm_init(ATOM0_CH4_P02_4, 30000, 0);
  gtm_pwm_init(ATOM0_CH5_P02_5, 30000, 0);
}

void Encoder_init(void) {
  gpt12_init(GPT12_T5, GPT12_T5INB_P10_3, GPT12_T5EUDB_P10_1);
}

////////电机输出占空比限幅
/*static*/
int PWM_Limit(int outPWM) {
    if(SystemData.SpeedData.nowspeed>20){

  if (outPWM >= PWM_H)
    return PWM_H;
  else if (outPWM <= PWM_L)
    return PWM_L;
  else
    return outPWM;


    }

    if(SystemData.SpeedData.nowspeed<20){

     if (outPWM >= 400)
       return 400;
     else if (outPWM <= -400)
       return -400;
     else
       return outPWM;


       }


}

///////电机输出
void MotorPWM_output(int M)  //电机输出函数(占空比)
{
  M = PWM_Limit(M);

  if (M < 0)  //说明正转
  {
    pwm_duty(ATOM0_CH4_P02_4, -M);
    pwm_duty(ATOM0_CH5_P02_5, 0);
  } else {
    pwm_duty(ATOM0_CH4_P02_4, 0);
    pwm_duty(ATOM0_CH5_P02_5, M);
  }
}

//编码器采样端口初始化
////////速度采集
void GetSpeed(SpeedDatatypedef* Q) {
  Q->nowspeed = gpt12_get(GPT12_T5);
  gpt12_clear(GPT12_T5);

  if (car_which == 0)
    Q->nowspeed /= 2;
  else if (car_which == 1)
    Q->nowspeed /= 8;

  Q->Length += Q->nowspeed;

  //  if (ImageStatus.Road_type == Barn_in)
  //    Q->Length = 0;

  if (ImageStatus.CirquePass == 'T')  //成功入环后用距离消抖 防止直接判断出环
    ImageStatus.Pass_Lenth += Q->nowspeed;
  else
    ImageStatus.Pass_Lenth = 0;

  if (ImageStatus.CirqueOff == 'T')  //防止二次入环
    ImageStatus.Out_Lenth += Q->nowspeed;
  else
    ImageStatus.Out_Lenth = 0;

  if (ImageStatus.Road_type == Cross)  //十字防三叉误判
    ImageStatus.Cross_Lenth += Q->nowspeed;
  else
    ImageStatus.Cross_Lenth = 0;
  // g
  if (ImageStatus.Road_type == Barn_in)  ///检测到车库
    ImageStatus.Barn_Lenth += Q->nowspeed;
  else
    ImageStatus.Barn_Lenth = 0;

  if (ImageStatus.Road_type == Forkout)
    ImageStatus.Fork_Out_Len += Q->nowspeed;
  else
    ImageStatus.Fork_Out_Len = 0;
  // g5.13
  if (Fork_dowm == 1)  //  三叉路径标志
    ImageStatus.Dowm_lenth += Q->nowspeed;
  else
    ImageStatus.Dowm_lenth = 0;
  // g5.22
  if (SystemData.Stop == 2)
    ImageStatus.Stop_lenth += Q->nowspeed;
  else
    ImageStatus.Stop_lenth = 0;

  if (ImageStatus.Road_type == Ramp)
    ImageStatus.Ramp_lenth += Q->nowspeed;
  else
    ImageStatus.Ramp_lenth = 0;

  if (ImageStatus.Road_type == Cross_ture)
    ImageStatus.Cross_ture_lenth += Q->nowspeed;
  else
    ImageStatus.Cross_ture_lenth = 0;

  if (ramptestflag == 0)
    ImageStatus.ramptestlenth += Q->nowspeed;
  else
    ImageStatus.ramptestlenth = 0;
}

//********************************电机PID调节****************
/***********************************************************/
PID_Datatypedef MotorPID_Data;  //前三个是PID

/*------------------------------------------------------------------------------------------------------
【函    数】MotorPID_output增量式
【功    能】电机增量式PID
【参    数】NowSpeed实测速度  ExpectSpeed目标速度
【返 回 值】Increase 反馈增量值
--------------------------------------------------------------------------------------------------------*/
int MotorPID_output(PID_Datatypedef* sptr, float NowSpeed, int ExpectSpeed) {
  //当前误差，定义为寄存器变量，只能用于整型和字符型变量，提高运算速度

  int iError,    //当前误差
      Increase;  //最后得出的实际增量

  iError = ExpectSpeed - NowSpeed;  //计算当前误差

  Increase = (int)(sptr->P * (iError - sptr->LastError) + sptr->I * iError +
                   sptr->D * (iError - 2 * sptr->LastError + sptr->PrevError));
  sptr->PrevError = sptr->LastError;  //更新前次误差
  sptr->LastError = iError;           //更新上次误差
  return Increase;
}

/*------------------------------------------------------------------------------------------------------
【函    数】MotorPID_output增量式
【功    能】电机增量式PID
【参    数】NowSpeed实测速度  ExpectSpeed目标速度
【返 回 值】Increase 反馈增量值
--------------------------------------------------------------------------------------------------------*/
int MotorPID_output_1(PID_Datatypedef* sptr,
                      int NowSpeed,
                      int ExpectSpeed)  // g5.14
{
  //当前误差，定义为寄存器变量，只能用于整型和字符型变量，提高运算速度

  int iError,  //当前误差
      OUT;     //最后得出的实际增量
  static int iError_sum;
  iError = ExpectSpeed - NowSpeed;  //计算当前误差
  iError_sum += iError;             //误差累计

  OUT = sptr->P * iError + sptr->I * iError_sum;
  sptr->PrevError = sptr->LastError;  //更新前次误差
  sptr->LastError = iError;           //更新上次误差
  return OUT;                         //输出PWM
}

int MotorPID_output_3(int NowSpeed, int ExpectSpeed)  // g5.14
{
  //当前误差，定义为寄存器变量，只能用于整型和字符型变量，提高运算速度

  int iError;      //当前误差
  static int OUT;  //最后得出的实际增量

  iError = ExpectSpeed - NowSpeed;  //计算当前误差

  OUT += (int)(MotorPID_Data.P * (iError - MotorPID_Data.LastError) +
               MotorPID_Data.I * iError);

  PWM_Limit(OUT);

  MotorPID_Data.LastError = iError;  //更新上次误差
  return OUT;                        //输出PWM
}

void MOTOR(int setspeed) {  // BANG//g
  int spe_err = 0;
  spe_err = setspeed - SystemData.SpeedData.nowspeed;

  if (spe_err > Divaite_Bang)
    MotorPWM_output(PWM_Bang);
  else if (spe_err < -Divaite_Bang)
    MotorPWM_output(-PWM_Bang);
  else {
    SystemData.SpeedData.motor_duty += MotorPID_output(
        &MotorPID_Data, SystemData.SpeedData.nowspeed, setspeed);
    SystemData.SpeedData.motor_duty =
        PWM_Limit(SystemData.SpeedData.motor_duty);
    MotorPWM_output(SystemData.SpeedData.motor_duty);
  }
}

/******** 变速控制 *********/

/**
 * @brief speed decision-making by Foresight
 * @param void
 * @return void
 */

int down_fast = 0;
void Control_Speed(void) {
  /****************************     变速     ****************************/

  SystemData.SpeedData.expect_True_speed =
      SystemData.SpeedData.MaxSpeed -
      (SystemData.SpeedData.MaxSpeed - SystemData.SpeedData.MinSpeed) *
          ImageStatus.Foresight * ImageStatus.Foresight / 50;

  if (outacc_flag == 1) {
    SystemData.SpeedData.expect_True_speed += SystemData.outbent_acc;
  }

  if (SystemData.SpeedData.expect_True_speed > SystemData.SpeedData.MaxSpeed)
    SystemData.SpeedData.expect_True_speed = SystemData.SpeedData.MaxSpeed;
  if (SystemData.SpeedData.expect_True_speed < SystemData.SpeedData.MinSpeed)
    SystemData.SpeedData.expect_True_speed = SystemData.SpeedData.MinSpeed;

  if (ImageStatus.straight_acc)
    SystemData.SpeedData.expect_True_speed =
        SystemData.SpeedData.straight_speed;



  if (sector == 4) {
    if (rampnum == 1)
      SystemData.SpeedData.expect_True_speed = 65;

    if (ImageStatus.Road_type == Ramp)
      SystemData.SpeedData.expect_True_speed = 60;

    if (rampnum == 1&&ramptestflag==0)
      SystemData.SpeedData.expect_True_speed = 60;

//    if (ImageStatus.Barn_Flag == 2)
//      SystemData.SpeedData.expect_True_speed = 30;

    if (Fork_dowm == 1)
      SystemData.SpeedData.expect_True_speed = 65;
  }

  if (sector == 6) {
    if (rampnum == 1)
      SystemData.SpeedData.expect_True_speed = 70;

    if (ImageStatus.Road_type == Ramp)
      SystemData.SpeedData.expect_True_speed = 60;

    if (rampnum == 1&&ramptestflag==0)
      SystemData.SpeedData.expect_True_speed = 65;

//    if (ImageStatus.Barn_Flag == 2)
//      SystemData.SpeedData.expect_True_speed = 30;

    if (Fork_dowm == 1)
      SystemData.SpeedData.expect_True_speed = 65;

    if ((ImageStatus.Road_type == LeftCirque ||
        ImageStatus.Road_type == RightCirque)&&ImageStatus.CirqueOff == 'F') {

        SystemData.SpeedData.expect_True_speed = 68;
    }
  }

  if (SystemData.rounds > 9)
  SystemData.SpeedData.expect_True_speed +=
      SystemData.speed_per_round * (SystemData.rounds - 9);

  //  if(SystemData.SpeedData.nowspeed-SystemData.SpeedData.expect_True_speed>20)
  //      down_fast=1;
  //
  //  else
  //  if(SystemData.SpeedData.expect_True_speed-SystemData.SpeedData.nowspeed>0)
  //      down_fast=0;
  //
  //  if(down_fast==1)
  //      SystemData.SpeedData.expect_True_speed=0;
}

static uint32 max_v_offline;
static uint32 min_v_offline;
/**
 * @brief speed decision-making by offline
 * @param void
 * @return void
 */
void Control_Speed_1(void) {
  SystemData.SpeedData.expect_True_speed =
      (ImageStatus.OFFLine - max_v_offline) *
          (SystemData.SpeedData.MinSpeed - SystemData.SpeedData.MaxSpeed) /
          (min_v_offline - max_v_offline) +
      SystemData.SpeedData.MaxSpeed;
  if (SystemData.SpeedData.expect_True_speed > SystemData.SpeedData.MaxSpeed)
    SystemData.SpeedData.expect_True_speed = SystemData.SpeedData.MaxSpeed;
  if (SystemData.SpeedData.expect_True_speed < SystemData.SpeedData.MinSpeed)
    SystemData.SpeedData.expect_True_speed = SystemData.SpeedData.MinSpeed;
}

/**
 * @brief wireless debug
 * @param void
 * @return boolean
 */
boolean max_voffline_fun(pchar args, void* data, IfxStdIf_DPipe* io) {
  PRINTF("max= %d min= %d\n\r", max_v_offline, min_v_offline);
  Ifx_Shell_parseUInt32(&args, &max_v_offline, FALSE);
  Ifx_Shell_parseUInt32(&args, &min_v_offline, FALSE);
  return TRUE;
}

/**
 * @brief speed decision-making by Foresight different from algorithm of
 * Control_Speed
 * @param void
 * @return void
 */
void Control_Speed_2(void) {
  if (ImageStatus.Foresight >= 18)

    SystemData.SpeedData.expect_True_speed = SystemData.SpeedData.MinSpeed;

  else if (ImageStatus.Foresight >= 15) {
    SystemData.SpeedData.expect_True_speed = SystemData.SpeedData.MinSpeed;
  } else if (ImageStatus.Foresight >= 13) {
    SystemData.SpeedData.expect_True_speed = SystemData.SpeedData.MinSpeed;
  } else if (ImageStatus.Foresight >= 11) {
    SystemData.SpeedData.expect_True_speed = SystemData.SpeedData.MinSpeed;
  } else if (ImageStatus.Foresight >= 9) {
    SystemData.SpeedData.expect_True_speed = SystemData.SpeedData.MinSpeed;
  } else if (ImageStatus.Foresight >= 8) {
    SystemData.SpeedData.expect_True_speed = SystemData.SpeedData.MinSpeed;
  }
  if (ImageStatus.Foresight <= 7) {
    SystemData.SpeedData.expect_True_speed += 1;
  }

  if (SystemData.SpeedData.expect_True_speed > SystemData.SpeedData.MaxSpeed)
    SystemData.SpeedData.expect_True_speed = SystemData.SpeedData.MaxSpeed;
  if (SystemData.SpeedData.expect_True_speed < SystemData.SpeedData.MinSpeed)
    SystemData.SpeedData.expect_True_speed = SystemData.SpeedData.MinSpeed;

  if (ImageStatus.Barn_Flag == 2)
    SystemData.SpeedData.expect_True_speed = 30;
}
