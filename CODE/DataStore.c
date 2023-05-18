/*
 * DataStore.c
 *    数据初始化与存储
 *  Created on: 2021年2月3日
 *      Author: 周文奇
 */

#include "DataStore.h"
#include "C_H.h"
#include "Fuzzy.h"
#include "KEY.h"
#include "Steer.h"
#include "System.h"
#include "car.h"
#include "icm.h"
#include "zf_eeprom.h"

#pragma section all "cpu0_dsram"
//存储的整形变量
static sint32* const int_buf[] = {&ImageStatus.TowPoint,
                                  &SystemData.SpeedData.expectspeed,
                                  &SystemData.SpeedData.MinSpeed,
                                  &SystemData.SpeedData.MaxSpeed,
                                  &SystemData.OutCicle_line,
                                  &SystemData.Circleoff_offline,
                                  &SystemData.Model,
                                  &SystemData.CircleP,
                                  &SystemData.fork_start_lenth,
                                  &SystemData.fork_off_lenth,
                                  &SystemData.ramp_lenth_start,
                                  &steer_left,
                                  &steer_middle,
                                  &steer_right,
                                  &SystemData.circles_pl,
                                  &SystemData.circles_pr,
                                  &SystemData.circles_off_lenth,
                                  &SystemData.SpeedData.straight_speed,
                                  &ImageScanInterval,
                                  &ImageScanInterval_Cross,
                                  &ImageStatus.variance,
                                  &ImageStatus.Threshold_static,
                                  &SystemData.straighet_towpoint,
                                  &SystemData.circlem_pl,
                                  &SystemData.circlem_pr,
                                  &SystemData.circlem_off_lenth,
                                  &SystemData.circlel_pl,
                                  &SystemData.circlel_pr,
                                  &SystemData.circlel_off_lenth,
                                  &SystemData.clrcle_priority[0],
                                  &SystemData.clrcle_priority[1],
                                  &SystemData.clrcle_priority[2],
                                  &SystemData.debug_lenth,
                                  &SystemData.exp_time,
                                  &SystemData.mtv_lroffset,
                                  &SystemData.mtv_gain,
                                  &SystemData.circle_kin,
                                  &SystemData.circle_max_ang,
                                  &Car.config.magnetErrorGain,
                                  &SystemData.ramp_lenth_start,
                                  &SystemData.fork_lenth_start,
                                  &SystemData.barn_lenth,
                                  &SystemData.outbent_acc,
                                  &ImageStatus.variance_acc,
                                  &SystemData.speed_per_round,
                                  NULL};
//存储的float型变量
static float32* const float_buf[] = {&SteerPIDdata.P,
                                     &SteerPIDdata.D,
                                     &MotorPID_Data.P,
                                     &MotorPID_Data.I,
                                     &MotorPID_Data.D,
                                     &SteerPIDdata.Kdin,
                                     &SteerPIDdata.Kdout,
                                     &Steer_d,
                                     &MHstruct.f_DuoJiP_TableL[0],
                                     &MHstruct.f_DuoJiP_TableL[1],
                                     &MHstruct.f_DuoJiP_TableL[2],
                                     &MHstruct.f_DuoJiP_TableL[3],
                                     &MHstruct.f_DuoJiP_TableL[4],
                                     &MHstruct.f_DuoJiP_TableL[5],
                                     &MHstruct.f_DuoJiP_TableL[6],
                                     &steer_k_r,
                                     &SystemData.straight_p,
                                     &SystemData.straight_d,
                                     &steer_k_l,
                                     &MHstruct.f_DuoJiP_TableR[0],
                                     &MHstruct.f_DuoJiP_TableR[1],
                                     &MHstruct.f_DuoJiP_TableR[2],
                                     &MHstruct.f_DuoJiP_TableR[3],
                                     &MHstruct.f_DuoJiP_TableR[4],
                                     &MHstruct.f_DuoJiP_TableR[5],
                                     &MHstruct.f_DuoJiP_TableR[6],
                                     &SystemData.circle_kout,

                                     NULL};

static uint32 int_sector = 1, float_sector = 2;
uint32 sector;
IFX_INLINE static void dial_encode(void) {
  uint8 bit0 = dial_read(0), bit1 = dial_read(1);
  sector = ((bit1 << 1) | bit0) * 2;
  int_sector = sector + 1;
  float_sector = sector + 2;
  PRINTF("sector: int=%d float=%d\n\r", int_sector, float_sector);
}
//数据存储在flash中
void data_store(void) {
  //  dial_encode();
  eeprom_erase_sector(int_sector);
  eeprom_erase_sector(float_sector);

  for (int i = 0; int_buf[i]; ++i)
    eeprom_page_program(int_sector, i, int_buf[i]);
  for (int i = 0; float_buf[i]; ++i)
    eeprom_page_program(float_sector, i, (uint32*)float_buf[i]);
}
//数据读取到变量中
void data_load(void) {
  dial_encode();
  if (!flash_check(int_sector, 0) || !flash_check(float_sector, 0))
    return;
  for (int i = 0; float_buf[i]; ++i)
    *float_buf[i] = flash_read(float_sector, i, float);
  for (int i = 0; int_buf[i]; ++i)
    *int_buf[i] = flash_read(int_sector, i, int);
  PRINTF("flash loaded!\n\r");
}

#pragma section all restore

void Data_Settings(void) {
  //图像参数
  extern uint8 Fork_in_1, Fork_in_2;
  Fork_in_1 = Fork_in_2 = 'F';
  ImageStatus.MiddleLine = 40;
  ImageStatus.TowPoint_Gain = 0.2;
  ImageStatus.TowPoint_Offset_Max = 5;
  ImageStatus.TowPoint_Offset_Min = -2;
  ImageStatus.TowPointAdjust_v = 160;
  ImageStatus.Det_all_k = 0.7;  //待定
  ImageStatus.CirquePass = 'F';
  ImageStatus.IsCinqueOutIn = 'F';
  ImageStatus.CirqueOut = 'F';
  ImageStatus.CirqueOff = 'F';
  ImageStatus.Barn_Flag = 0;
  ImageStatus.straight_acc = 0;

  //舵机参数
  SteerPIDdata.P = 1.0;
  SteerPIDdata.I = 0.0;
  SteerPIDdata.D = 0.0;

  //二次P
  Steer_pl = 50.0;
  Steer_ph = 1.0;
  Steer_d = 0.0;

  //分段P
  Po_PID.Straight.P = 1.0;
  Po_PID.Bend_Large.P = 1.0;
  Po_PID.Bend_Small.P = 1.0;

  //速度参数

  SystemData.SpeedData.expectspeed = 50;

  /**************************/
  /**************************/
  /********调试参数**********/
  /**************************/
  /**************************/

  //控制部分

  //舵机系数
  steer_k_r = 1.000;  //   舵机偏置系数
  steer_k_l = 1.000;
  steer_middle = 513;  //舵机中值440
  steer_left = 435;    //左转   372
  steer_right = 594;   //右转   505

  //模糊P速度92
  MHstruct.f_DuoJiP_TableL[0] = 1.549;
  MHstruct.f_DuoJiP_TableL[1] = 1.559;
  MHstruct.f_DuoJiP_TableL[2] = 1.789;
  MHstruct.f_DuoJiP_TableL[3] = 1.859;
  MHstruct.f_DuoJiP_TableL[4] = 1.909;
  MHstruct.f_DuoJiP_TableL[5] = 1.939;
  MHstruct.f_DuoJiP_TableL[6] = 1.999;

  MHstruct.f_DuoJiP_TableR[0] = 1.549;
  MHstruct.f_DuoJiP_TableR[1] = 1.559;
  MHstruct.f_DuoJiP_TableR[2] = 1.789;
  MHstruct.f_DuoJiP_TableR[3] = 1.859;
  MHstruct.f_DuoJiP_TableR[4] = 1.909;
  MHstruct.f_DuoJiP_TableR[5] = 1.939;
  MHstruct.f_DuoJiP_TableR[6] = 1.999;

  //小圆环
  SystemData.circles_pl = 226;
  SystemData.circles_pr = 226;
  SystemData.circles_off_lenth = 0;  //小圆环出环防误判距离

  //中圆环
  SystemData.circlem_pl = 226;
  SystemData.circlem_pr = 226;
  SystemData.circlem_off_lenth = 0;  //小圆环出环防误判距离

  //大圆环
  SystemData.circlel_pl = 226;
  SystemData.circlel_pr = 226;
  SystemData.circlel_off_lenth = 0;  //小圆环出环防误判距离
  SystemData.clrcle_priority[3] = 0;
  SystemData.clrcle_num = 0;
  //直道
  SystemData.straight_p = 2.26;
  SystemData.straight_d = 2.54;
  SystemData.straighet_towpoint = 22;

  //入弯出弯D
  SteerPIDdata.Kdin = 2.47;
  SteerPIDdata.Kdout = 3.01;

  //电机参数
  MotorPID_Data.P = 9.00;
  MotorPID_Data.I = 0.63;
  MotorPID_Data.D = 0.0;

  //图像参数
  ImageStatus.TowPoint = 22;           //前瞻
  ImageStatus.Threshold_static = 95;   //静态阈值
  ImageStatus.Threshold_detach = 200;  //阳光算法  亮斑分离
  ImageScanInterval = 5;  //扫边范围    上一行的边界+-ImageScanInterval
  ImageScanInterval_Cross = 2;  //十字扫线范围
  ImageStatus.variance = 26;    //直道方差阈值

  //速度参数
  SystemData.SpeedData.MaxSpeed = 90;         //最大速度
  SystemData.SpeedData.MinSpeed = 94;         //最小速度
  SystemData.SpeedData.straight_speed = 100;  //最小速度
}
