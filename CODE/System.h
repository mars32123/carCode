/*
 * System.h
 *
 *  Created on: 2021年1月30日
 *      Author: 周文奇
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
  uint8 SteerOK;      //舵机启动标志
  uint8 CameraOK;     //摄像头启动显示标志
  uint8 OldCameraOK;  //灰度传输标志
  uint8 MotorOK;      //电机开关
  uint8 Point;        //赛道实际中心
  uint8 UpdataOK;     //数据更新
  uint8 Stop;         //停止标志
  uint8 GO_OK;        //冲冲冲
  int Model;          //车辆模式

  //图像参数
  int OutCicle_line;  //出环判断图像截止行  越大出环判断越严格越晚
  int L_T_R_W;  // 出环左有边右无边数量  越大出环结束越严格
  int Circleoff_offline;  //看到远处的距离判断出环结束  越小越严格
  int CircleP;            //环内P


//防误判距离
  int fork_start_lenth;  //岔路口检测起始
  int fork_off_lenth;//岔路口检测截止

  int circle_off_lenth;//出环防误判距离

  //小圆环
  int circles_pl;
  int circles_pr;
  int circles_off_lenth;//小圆环出环防误判距离

   //中圆环
  int circlem_pl;
  int circlem_pr;
  int circlem_off_lenth;//小圆环出环防误判距离

   //大圆环
  int circlel_pl;
  int circlel_pr;
  int circlel_off_lenth;//小圆环出环防误判距离
  int clrcle_priority[3];//圆环类型
  int clrcle_num;// 在第几个圆环


  int circle_kin;//入环补线半径
  float circle_kout;//出环补线斜率
  int circle_max_ang;//环内大叫限幅


  //直道
  float straight_p;//直道P
  float straight_d;//直道D 
  int   straighet_towpoint;//直道前瞻


  int debug_lenth;//调试距离

  //摄像头配置
  int exp_time; //曝光时间
  int mtv_lroffset;//摄像头左右偏置
  int mtv_gain;//摄像头增益

  int ramp_lenth_start;//坡道距离
  int fork_lenth_start;//三叉距离
  int barn_lenth;//圆环距离

  int outbent_acc;//出弯加速


  int rounds; // 圈数
  int speed_per_round; // 每圈减速多少

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
