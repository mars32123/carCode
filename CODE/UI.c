/*
 * UI.c
 *
 *  Created on: 2021年2月2日
 *      Author: 周文奇
 */

#include <key.h>
#include "C_H.h"
#include "DataStore.h"
#include "Fuzzy.h"
#include "Key.h"
#include "SEEKFREE_IPS114_SPI.h"
#include "Steer.h"
#include "System.h"
#include "buzzer.h"
#include "car.h"
#include "icm.h"
#include "led.h"
#include "magnet.h"
#include "shell.h"
#include "zf_stm_systick.h"
#define Main_menuL 22
#define Main_menuH 6  //主菜单个数

#define DH 60
#define DL 30

int Cursor = 0;      //光标所在行，由上到下递增
int NowPage = 0;     //嵌套页
int Menustatus = 0;  //主菜单状态，0表示处于主菜单
float DataInterval = 0;
int Option_bar = 0;  //主菜单选项栏
unsigned count = 1;

static const char* Main_menu[Main_menuH] = {
    "Camera View", "Parameters", "Inductor", "Let's Go!", "Gray View", "Mortor",
};

static const char* DATA_menu[] = {"left P 0",
                                  "left P 1",
                                  "left P 2",
                                  "left P 3",
                                  "left P 4",
                                  "left P 5",
                                  "left P 6",
                                  "left P all",
                                  "SteerP",
                                  "SteerD",
                                  "MotorP",
                                  "MotorI",
                                  "MotorD",
                                  "Kd in",
                                  "Kd out",
                                  "Steer Kd",
                                  "Out Cicle line",
                                  "L T R W",
                                  "Circle offLine",
                                  "debug mode",
                                  "steer middle",
                                  "steer left",
                                  "steer right",
                                  "steer k r",
                                  "ScanInterval",
                                  "ScanItvCross",
                                  "straightVarianc",
                                  "straight Kp",
                                  "straight Kd",
                                  "straight tow",
                                  "steer k l",
                                  "circle S L P",
                                  "circle S R P",
                                  "circle S offLen",
                                  "circle M L P",
                                  "circle M R P",
                                  "circle M offLen",
                                  "circle L L P",
                                  "circle L R P",
                                  "circle L offLen",
                                  "circle 1",
                                  "circle 2",
                                  "circle 3",
                                  "right P 0",
                                  "right P 1",
                                  "right P 2",
                                  "right P 3",
                                  "right P 4",
                                  "right P 5",
                                  "right P 6",
                                  "right P all",
                                  "exposure time",
                                  "horizon offset",
                                  "image gain",
                                  "circle k in",
                                  "circle k out",
                                  "circle max angle",
                                  "magnet gain",
                                  "str acc variance",
                                  "slow down/round",
                                  "",
                                  "",
                                  "",
                                  "7"};

static const char* ADC_menu[] = {"AD_L", "AD_M", "AD_R", "AD_ALL"};

/********此处用于参数添加*************/
void Data_Chang_Event(void) {  //用于数据更改事件添加
  switch (Cursor) {
    case 8:
      SteerPIDdata.P = SteerPIDdata.P + 5 * DataInterval / 10;
      break;
    case 9:
      SteerPIDdata.D = SteerPIDdata.D + 5 * DataInterval / 100;
      break;
    case 10:
      MotorPID_Data.P = MotorPID_Data.P + 1 * DataInterval;
      break;
    case 11:
      MotorPID_Data.I = MotorPID_Data.I + 1 * DataInterval / 100;
      break;
    case 12:
      MotorPID_Data.D = MotorPID_Data.D + 1 * DataInterval / 20;
      break;
      //                  case 2:Po_PID.Straight.P =
      //                  Po_PID.Straight.P+2*DataInterval/100; break; case
      //                  3:Po_PID.Bend_Large.P=Po_PID.Bend_Large.P+2*DataInterval/100;
      //                  break;
      //                  case
      //                  4:Po_PID.Bend_Small.P=Po_PID.Bend_Small.P+2*DataInterval/100;
      //                  break;

    case 13:
      SteerPIDdata.Kdin = SteerPIDdata.Kdin + 5 * DataInterval / 100;
      break;
    case 14:
      SteerPIDdata.Kdout = SteerPIDdata.Kdout + 5 * DataInterval / 100;
      break;
    case 15:
      Steer_d = Steer_d + 1 * DataInterval / 100;
      break;

    case 0:
      MHstruct.f_DuoJiP_TableL[0] =
          MHstruct.f_DuoJiP_TableL[0] + 1 * DataInterval / 100;
      break;
    case 1:
      MHstruct.f_DuoJiP_TableL[1] =
          MHstruct.f_DuoJiP_TableL[1] + 1 * DataInterval / 100;
      break;
    case 2:
      MHstruct.f_DuoJiP_TableL[2] =
          MHstruct.f_DuoJiP_TableL[2] + 1 * DataInterval / 100;
      break;
    case 3:
      MHstruct.f_DuoJiP_TableL[3] =
          MHstruct.f_DuoJiP_TableL[3] + 1 * DataInterval / 100;
      break;
    case 4:
      MHstruct.f_DuoJiP_TableL[4] =
          MHstruct.f_DuoJiP_TableL[4] + 1 * DataInterval / 100;
      break;
    case 5:
      MHstruct.f_DuoJiP_TableL[5] =
          MHstruct.f_DuoJiP_TableL[5] + 1 * DataInterval / 100;
      break;
    case 6:
      MHstruct.f_DuoJiP_TableL[6] =
          MHstruct.f_DuoJiP_TableL[6] + 1 * DataInterval / 100;
      break;
    case 7:
      for (int i = 0; i < 7; i++) {
        MHstruct.f_DuoJiP_TableL[i] =
            MHstruct.f_DuoJiP_TableL[i] + 1 * DataInterval / 100;
      }
      break;

    case 16:
      SystemData.OutCicle_line = SystemData.OutCicle_line + DataInterval;
      break;
    case 17:
      SystemData.L_T_R_W = SystemData.L_T_R_W + DataInterval;
      break;
    case 18:
      SystemData.Circleoff_offline =
          SystemData.Circleoff_offline + DataInterval;
      break;
    case 19:
      SystemData.Model = SystemData.Model + DataInterval;
      break;
    case 20: {
      steer_middle = steer_middle + 1 * DataInterval;
      pwm_duty(ATOM0_CH1_P33_9, steer_middle);
    } break;
    case 21: {
      steer_left = steer_left + 1 * DataInterval;
      pwm_duty(ATOM0_CH1_P33_9, steer_left);
    } break;
    case 22: {
      steer_right = steer_right + 1 * DataInterval;
      pwm_duty(ATOM0_CH1_P33_9, steer_right);
    } break;

    case 23:
      steer_k_r = steer_k_r + 5 * DataInterval / 1000;
      break;
    case 24:
      ImageScanInterval = ImageScanInterval + DataInterval;
      break;
    case 25:
      ImageScanInterval_Cross = ImageScanInterval_Cross + DataInterval;
      break;
    case 26:
      ImageStatus.variance = ImageStatus.variance + 2 * DataInterval;
      break;
    case 27:
      SystemData.straight_p = SystemData.straight_p + 2 * DataInterval / 100;
      break;
    case 28:
      SystemData.straight_d = SystemData.straight_d + 2 * DataInterval / 100;
      break;
    case 29:
      SystemData.straighet_towpoint =
          SystemData.straighet_towpoint + DataInterval;
      break;
    case 30:
      steer_k_l = steer_k_l + 5 * DataInterval / 1000;
      break;
    case 31:
      SystemData.circles_pl = SystemData.circles_pl + 2 * DataInterval;
      break;
    case 32:
      SystemData.circles_pr = SystemData.circles_pr + 2 * DataInterval;
      break;
    case 33:
      SystemData.circles_off_lenth =
          SystemData.circles_off_lenth + 20 * DataInterval;
      break;
    case 34:
      SystemData.circlem_pl = SystemData.circlem_pl + 2 * DataInterval;
      break;
    case 35:
      SystemData.circlem_pl = SystemData.circlem_pl + 2 * DataInterval;
      break;
    case 36:
      SystemData.circlem_off_lenth =
          SystemData.circlem_off_lenth + 20 * DataInterval;
      break;
    case 37:
      SystemData.circlel_pl = SystemData.circlel_pl + 2 * DataInterval;
      break;
    case 38:
      SystemData.circlel_pr = SystemData.circlel_pr + 2 * DataInterval;
      break;
    case 39:
      SystemData.circlel_off_lenth =
          SystemData.circlel_off_lenth + 20 * DataInterval;
      break;
    case 40:
      SystemData.clrcle_priority[0] =
          SystemData.clrcle_priority[0] + DataInterval;
      break;
    case 41:
      SystemData.clrcle_priority[1] =
          SystemData.clrcle_priority[1] + DataInterval;
      break;
    case 42:
      SystemData.clrcle_priority[2] =
          SystemData.clrcle_priority[2] + DataInterval;
      break;

    case 43:
      MHstruct.f_DuoJiP_TableR[0] =
          MHstruct.f_DuoJiP_TableR[0] + 1 * DataInterval / 100;
      break;
    case 44:
      MHstruct.f_DuoJiP_TableR[1] =
          MHstruct.f_DuoJiP_TableR[1] + 1 * DataInterval / 100;
      break;
    case 45:
      MHstruct.f_DuoJiP_TableR[2] =
          MHstruct.f_DuoJiP_TableR[2] + 1 * DataInterval / 100;
      break;
    case 46:
      MHstruct.f_DuoJiP_TableR[3] =
          MHstruct.f_DuoJiP_TableR[3] + 1 * DataInterval / 100;
      break;
    case 47:
      MHstruct.f_DuoJiP_TableR[4] =
          MHstruct.f_DuoJiP_TableR[4] + 1 * DataInterval / 100;
      break;
    case 48:
      MHstruct.f_DuoJiP_TableR[5] =
          MHstruct.f_DuoJiP_TableR[5] + 1 * DataInterval / 100;
      break;
    case 49:
      MHstruct.f_DuoJiP_TableR[6] =
          MHstruct.f_DuoJiP_TableR[6] + 1 * DataInterval / 100;
      break;
    case 50:
      for (int i = 0; i < 7; i++) {
        MHstruct.f_DuoJiP_TableR[i] =
            MHstruct.f_DuoJiP_TableR[i] + 1 * DataInterval / 100;
      }
      break;

    case 51:
      SystemData.exp_time = SystemData.exp_time + 10 * DataInterval;
      break;
    case 52:
      SystemData.mtv_lroffset = SystemData.mtv_lroffset + DataInterval;
      break;
    case 53:
      SystemData.mtv_gain = SystemData.mtv_gain + 2 * DataInterval;
      break;
    case 54:
      SystemData.circle_kin = SystemData.circle_kin + 1 * DataInterval;
      break;
    case 55:
      SystemData.circle_kout = SystemData.circle_kout + DataInterval / 100;
      break;
    case 56:
      SystemData.circle_max_ang = SystemData.circle_max_ang + DataInterval;
      break;
    case 57:
      Car.config.magnetErrorGain = Car.config.magnetErrorGain + DataInterval;
      break;
    case 58:
        ImageStatus.variance_acc = ImageStatus.variance_acc + DataInterval;
      break;
    case 59:
        SystemData.speed_per_round += DataInterval;
        break;
    default:
      break;
  }
}

void Key_Event(void) {
  //调整数据变化
  if (Option_bar == 1 && NowPage == 2) {
    Data_Chang_Event();
    if (DataInterval != 0)
      data_store();
    DataInterval = 0;
  }

  // Camera
  if (Option_bar == 0 && NowPage >= 1) {
    SystemData.SteerOK = 1;  //打开舵机
    if (count)               //防止持续开中断
    {
      SystemData.OldCameraOK = 0;
      SystemData.CameraOK = 1;
      count = 0;
    }

    // Camera参数调整
    if (NowPage == 2) {
      switch (Cursor) {
        case 0:
          ImageStatus.TowPoint = ImageStatus.TowPoint + DataInterval;
          break;
        case 1:
          ImageStatus.Threshold_static =
              ImageStatus.Threshold_static + DataInterval;
          break;
        default:
          break;
      }
      if (DataInterval != 0)
        data_store();
      DataInterval = 0;
    }
  } else if (Option_bar == 2 && NowPage >= 1) {  //电感采集
    if (count) {
      SystemData.OldCameraOK = 0;  //关闭图像传输，开中断
      SystemData.CameraOK = 0;
      count = 0;
    }
  } else if (Option_bar == 4 && NowPage >= 1) {  //原始灰度图像
    SystemData.SteerOK = 1;
    if (count) {
      SystemData.CameraOK = 0;
      SystemData.OldCameraOK = 1;
      count = 0;
    }
  } else {
    SystemData.CameraOK = 0;
    SystemData.OldCameraOK = 0;
    SystemData.SteerOK = 0;
    count = 1;
  }

  //电机
  if (Option_bar == 5 && NowPage == 2) {
    switch (Cursor) {
      case 0:
        SystemData.SpeedData.expectspeed += (50 * DataInterval);
        break;
      case 1:
        SystemData.SpeedData.MinSpeed += 1 * (DataInterval);
        break;
      case 2:
        SystemData.SpeedData.MaxSpeed += 1 * (DataInterval);
        break;
      case 3:
        SystemData.SpeedData.straight_speed += (1* DataInterval);
        break;
      case 4:
        SystemData.debug_lenth += (100 * DataInterval);
        break;
      case 5:
        SystemData.ramp_lenth_start += (100 * DataInterval);
        break;
      case 6:
        SystemData.fork_lenth_start += (100 * DataInterval);
        break;
      case 7:
        SystemData.barn_lenth += (100 * DataInterval);
        break;
      case 10:
        SystemData.outbent_acc += (DataInterval);
        break;
      default:
        break;
    }
    // pwm_duty(ATOM0_CH1_P33_9,
    // SystemData.SpeedData.expectspeed);//测试舵机暂用 MotorPWM_output(M_PWM);
    MotorPWM_output(SystemData.SpeedData.expectspeed);
    if (DataInterval != 0)
      data_store();
    DataInterval = 0;
  }

  if (NowPage == 0) {
    SystemData.Stop = 0;
    SystemData.CameraOK = 0;
    SystemData.SteerOK = 0;
    SystemData.GO_OK = 0;
  }
  //发车
  if (Option_bar == 3 && NowPage == 1) 
    car_launch();
}

void ui_key_up_click(void* btn) {
  if (!Menustatus) {  //如果此时处于主菜单
    ips114_clear(BLACK);
    Cursor--;
    Option_bar = Cursor;
    if (Cursor < 0)
      Cursor = Main_menuH - 1;
  } else if (NowPage == 1) {
    ips114_clear(BLACK);
    if (dial_read(3))
      Cursor--;
    else
      Cursor = Cursor - 8;

    if (Cursor < 0)
      Cursor = DH - 1;
  } else if (NowPage == 2) {
    if (dial_read(3))
      DataInterval = 1;
    else
      DataInterval = 10;
  }
  Key_Event();
}

void ui_key_down_click(void* btn) {
  if (!Menustatus) {
    ips114_clear(BLACK);

    Cursor++;

    Option_bar = Cursor;
    if (Cursor >= Main_menuH)
      Cursor = 0;
  } else if (NowPage == 1) {
    ips114_clear(BLACK);
    if (dial_read(3))
      Cursor++;
    else
      Cursor += 8;
    if (Cursor >= DH)
      Cursor = 0;

  } else if (NowPage == 2) {
    if (dial_read(3))
      DataInterval = -1;
    else
      DataInterval = -10;
  }
  Key_Event();
}

void ui_key_yes_click(void* btn) {
  if (!Menustatus) {
    ips114_clear(BLACK);
    NowPage++;
    Menustatus = 1;
    Cursor = 0;
  } else {
    NowPage++;
    NowPage = Limit(NowPage, 2, 0);
  }
  Key_Event();
}

void ui_key_no_click(void* btn) {
  if (!Menustatus) {
    ips114_clear(BLACK);
  } else if (NowPage >= 1) {
    ips114_clear(BLACK);
    NowPage--;
    if (NowPage <= 0) {
      Menustatus = 0;
      Cursor = 0;
      Option_bar = Cursor;
      NowPage = Limit(NowPage, 2, 0);
    }
  }
  Key_Event();
}

void Camera_menu(void) {
  uint8 page = 0;
  uint8 cursor;
  page = Cursor / 8;
  cursor = Cursor % 8;
  if (page == 0) {
    ips114_showstr(161, 0, "Tow", IPS114_PENCOLOR, IPS114_BGCOLOR);
    ips114_showint32(200, 0, ImageStatus.TowPoint, 2);

    ips114_showstr(161, 1, "TreS", IPS114_PENCOLOR, IPS114_BGCOLOR);
    ips114_showint32(200, 1, ImageStatus.Threshold_static, 3);

    ips114_showstr(161, 2, "OFF", IPS114_PENCOLOR, IPS114_BGCOLOR);
    ips114_showint32(200, 2, ImageStatus.OFFLine, 2);

    ips114_showstr(161, 3, "Det", IPS114_PENCOLOR, IPS114_BGCOLOR);
    ips114_showint32(200, 3, ImageStatus.Det_True, 2);

    ips114_showstr(161, 4, "DaT", IPS114_PENCOLOR, IPS114_BGCOLOR);
    ips114_showint32(200, 4, icmdata.Yaw, 3);

    ips114_showstr(161, 5, "P", IPS114_PENCOLOR, IPS114_BGCOLOR);
    ips114_showfloat(190, 5, ImageStatus.MU_P, 2, 3);

    ips114_showstr(161, 6, "towt", IPS114_PENCOLOR, IPS114_BGCOLOR);
    ips114_showint32(190, 6, ImageStatus.TowPoint_True, 4);

    ips114_showstr(161, 7, "Tre", IPS114_PENCOLOR, IPS114_BGCOLOR);
    ips114_showint32(200, 7, ImageStatus.Threshold, 3);
  }

  if (page == 1) {
    ips114_showstr(161, 0, "Speed", IPS114_PENCOLOR, IPS114_BGCOLOR);
    ips114_showfloat(200, 0, SystemData.SpeedData.nowspeed, 2, 3);

    ips114_showstr(161, 1, "Sight", IPS114_PENCOLOR, IPS114_BGCOLOR);
    ips114_showint32(200, 1, ImageStatus.Foresight, 4);

    ips114_showstr(161, 2, "expV", IPS114_PENCOLOR, IPS114_BGCOLOR);
    ips114_showfloat(200, 2, SystemData.SpeedData.expect_True_speed, 4, 3);

    ips114_showstr(161, 3, "Leth", IPS114_PENCOLOR, IPS114_BGCOLOR);
    ips114_showfloat(190, 3, SystemData.SpeedData.Length * OX, 6, 3);

    ips114_showstr(161, 4, "EC", IPS114_PENCOLOR, IPS114_BGCOLOR);
    ips114_showint32(190, 4, SteerPIDdata.EC, 2);

    ips114_showstr(161, 5, "van", IPS114_PENCOLOR, IPS114_BGCOLOR);
    ips114_showfloat(190, 5, variance, 2, 3);

    ips114_showstr(161, 6, "clnum", IPS114_PENCOLOR, IPS114_BGCOLOR);
    ips114_showint32(200, 6, SystemData.clrcle_num, 2);

    ips114_showstr(161, 7, "vacc", IPS114_PENCOLOR, IPS114_BGCOLOR);
    ips114_showint32(200, 7, variance_acc, 2);
  }

  ips114_showstr(228, cursor, "-", IPS114_PENCOLOR, IPS114_BGCOLOR);
  ips114_fill(0, uLCDH, uLCDW, uLCDH + 1, PINK);


}

void Data_menu(void) {
  for (int i = 0; i < 8; i++)
    if (NowPage == 2 && Cursor % 8 == i)
      ips114_showstr(0, i, DATA_menu[Cursor / 8 * 8 + i], PINK, GRAY);
    else
      ips114_showstr(0, i, DATA_menu[Cursor / 8 * 8 + i], IPS114_PENCOLOR,
                     IPS114_BGCOLOR);
  if (NowPage == 2)
    ips114_showstr(120, Cursor % 8, "<===>", PINK, GRAY);
  else
    ips114_showstr(120, Cursor % 8, "<===>", IPS114_PENCOLOR, IPS114_BGCOLOR);

  switch (Cursor / 8) {
    case 0:
      for (int i = 0; i < 7; i++)
        ips114_showfloat(160, i, MHstruct.f_DuoJiP_TableL[i], 2, 2);
      break;
    case 1:
      ips114_showfloat(160, 0, SteerPIDdata.P, 2, 2);
      ips114_showfloat(160, 1, SteerPIDdata.D, 2, 2);
      ips114_showfloat(160, 2, MotorPID_Data.P, 2, 2);
      ips114_showfloat(160, 3, MotorPID_Data.I, 2, 2);
      ips114_showfloat(160, 4, MotorPID_Data.D, 2, 2);
      ips114_showfloat(160, 5, SteerPIDdata.Kdin, 3, 2);
      ips114_showfloat(160, 6, SteerPIDdata.Kdout, 2, 2);
      ips114_showfloat(160, 7, Steer_d, 2, 2);
      break;
    case 2:
      ips114_showint32(160, 0, SystemData.OutCicle_line, 4);
      ips114_showint32(160, 1, SystemData.L_T_R_W, 4);
      ips114_showint32(160, 2, SystemData.Circleoff_offline, 4);
      ips114_showint32(160, 3, SystemData.Model, 4);
      ips114_showint32(160, 4, steer_middle, 4);
      ips114_showint32(160, 5, steer_left, 4);
      ips114_showint32(160, 6, steer_right, 4);
      ips114_showfloat(160, 7, steer_k_r, 1, 3);

      break;
    case 3:
      ips114_showint32(160, 0, ImageScanInterval, 4);
      ips114_showint32(160, 1, ImageScanInterval_Cross, 4);
      ips114_showint32(160, 2, ImageStatus.variance, 4);
      ips114_showfloat(160, 3, SystemData.straight_p, 2, 2);
      ips114_showfloat(160, 4, SystemData.straight_d, 2, 2);
      ips114_showfloat(160, 5, SystemData.straighet_towpoint, 2, 2);
      ips114_showfloat(160, 6, steer_k_l, 1, 3);
      ips114_showint32(160, 7, SystemData.circles_pl, 3);
      break;
    case 4:
      ips114_showint32(160, 0, SystemData.circles_pr, 3);
      ips114_showint32(160, 1, SystemData.circles_off_lenth, 3);
      ips114_showint32(160, 2, SystemData.circlem_pl, 3);
      ips114_showint32(160, 3, SystemData.circlem_pr, 3);
      ips114_showint32(160, 4, SystemData.circlem_off_lenth, 3);
      ips114_showint32(160, 5, SystemData.circlel_pl, 3);
      ips114_showint32(160, 6, SystemData.circlel_pr, 3);
      ips114_showint32(160, 7, SystemData.circlel_off_lenth, 3);
      break;
    case 5:
      ips114_showint32(160, 0, SystemData.clrcle_priority[0], 3);
      ips114_showint32(160, 1, SystemData.clrcle_priority[1], 3);
      ips114_showint32(160, 2, SystemData.clrcle_priority[2], 3);
      ips114_showfloat(160, 3, MHstruct.f_DuoJiP_TableR[0], 2, 2);
      ips114_showfloat(160, 4, MHstruct.f_DuoJiP_TableR[1], 2, 2);
      ips114_showfloat(160, 5, MHstruct.f_DuoJiP_TableR[2], 2, 2);
      ips114_showfloat(160, 6, MHstruct.f_DuoJiP_TableR[3], 2, 2);
      ips114_showfloat(160, 7, MHstruct.f_DuoJiP_TableR[4], 2, 2);
      break;
    case 6:
      ips114_showfloat(160, 0, MHstruct.f_DuoJiP_TableR[5], 2, 2);
      ips114_showfloat(160, 1, MHstruct.f_DuoJiP_TableR[6], 2, 2);
      ips114_showint32(160, 3, SystemData.exp_time, 3);
      ips114_showint32(160, 4, SystemData.mtv_lroffset, 3);
      ips114_showint32(160, 5, SystemData.mtv_gain, 3);
      ips114_showint32(160, 6, SystemData.circle_kin, 3);
      ips114_showfloat(160, 7, SystemData.circle_kout, 2, 2);
      break;
    case 7:
      ips114_showint32(160, 0, SystemData.circle_max_ang, 3);
      ips114_showint32(160, 1, Car.config.magnetErrorGain, 3);
      ips114_showint32(160, 2, ImageStatus.variance_acc, 3);
      ips114_showint32(160, 3, SystemData.speed_per_round, 3);
      break;
    default:
      break;
  }
}

void Adc_0_menu(void) {
  for (int8 i = 0; i < 4; i++) {
    ips114_showstr(0, i, ADC_menu[i], IPS114_PENCOLOR, IPS114_BGCOLOR);
  }
  magnet_record_max = NowPage == 2;
  ips114_showint32(80, 0, Car.status.magnetSensors.left.raw, 4);
  ips114_showint32(80, 1, Car.status.magnetSensors.middle.raw, 4);
  ips114_showint32(80, 2, Car.status.magnetSensors.right.raw, 4);
  ips114_showint32(80, 3, Car.status.magnetSensors.rawSum, 4);
  ips114_showint32(80, 4, Car.status.magnetSensors.error, 4);
}

void Motor_menu(void) {
  uint8 page = Cursor / 8, cursor = Cursor % 8;
  static const char* menu_labels[] = {"speed",
                                      "min speed",
                                      "max speed",
                                      "straight speed",
                                      "debug distance",
                                      "ramp distance",
                                      "fork distance",
                                      "barn distance",
                                      "real speed(cm/s",
                                      "distance (m)",
                                      "out bent acc",
                                      "",
                                      "",
                                      "",
                                      "",
                                      ""};
  for (int i = 0; i < 8; i++)
    if (NowPage == 2 && Cursor % 8 == i)
      ips114_showstr(0, i, menu_labels[page * 8 + i], PINK, GRAY);
    else
      ips114_showstr(0, i, menu_labels[page * 8 + i], IPS114_PENCOLOR,
                     IPS114_BGCOLOR);
  if (NowPage == 2)
    ips114_showstr(120, cursor, "<===>", PINK, GRAY);
  else
    ips114_showstr(120, cursor, "<===>", IPS114_PENCOLOR, IPS114_BGCOLOR);

  switch (page) {
    case 0:
      ips114_showint32(160, 0, SystemData.SpeedData.expectspeed, 4);
      ips114_showint32(160, 1, SystemData.SpeedData.MinSpeed, 4);
      ips114_showint32(160, 2, SystemData.SpeedData.MaxSpeed, 4);
      ips114_showint32(160, 3, SystemData.SpeedData.straight_speed, 4);
      ips114_showint32(160, 4, SystemData.debug_lenth, 5);
      ips114_showint32(160, 5, SystemData.ramp_lenth_start, 5);
      ips114_showint32(160, 6, SystemData.fork_lenth_start, 5);
      ips114_showint32(160, 7, SystemData.barn_lenth, 5);
      break;
    case 1:
      ips114_showfloat(160, 0, get_speed_convert(SystemData.SpeedData.nowspeed),
                       3, 3);
      ips114_showfloat(
          160, 1, get_speed_convert(SystemData.SpeedData.Length) / 20000, 5, 3);
      ips114_showint32(160, 2, SystemData.outbent_acc, 5);

      break;
    default:
      break;
  }
}

void menu(void) {  //主界面
  if (!Menustatus && NowPage == 0) {
    for (int8 i = 0; i < Main_menuH; i++) {
      ips114_showstr(0, i, Main_menu[i], IPS114_PENCOLOR, IPS114_BGCOLOR);
    }
    ips114_showstr(100, Cursor, "<--", IPS114_PENCOLOR, IPS114_BGCOLOR);
  } else if (NowPage >= 1) {  //子界面
    switch (Option_bar) {
      case 0:  //摄像头界面
        Camera_menu();
        break;
      case 1:  //调参数界面
        Data_menu();
        break;
      case 2:  // ADC采集界面
        Adc_0_menu();
        break;
      case 4:
        ips114_fill(0, uLCDH, uLCDW, uLCDH + 1, PINK);
        break;
      case 5:
        Motor_menu();
        break;
      default:
        break;
    }
  }
}
