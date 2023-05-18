/*
 * icm.c
 *
 *  Created on: 2021年6月1日
 *      Author: 周文奇
 */

#include "icm.h"
#include "C_H.h"
#include "MOTOR.h"
#include "System.h"
#include "buzzer.h"
#include "headfile.h"
#include "car.h"
ICMDatatypedef icmdata;

int ramp_pitch;
float dt = 0.02;
extern int icm_start_test_cross;

void ICM_OneOrderFilter(void) {
  get_icm20602_accdata_spi();
  get_icm20602_gyro_spi();
  //获取加速度
  icmdata.ACCEL_X = (float)icm_acc_x / 4096.0;
  icmdata.ACCEL_Z = (float)icm_acc_z / 4096.0;
  //获取角速度
  if(car_which==0)
      icmdata.YawVelocity =(float)icm_gyro_z / 16.4 + 0.36;
  else if(car_which==1)
      icmdata.YawVelocity =(float)icm_gyro_z / 16.4 + 0.36-1.33;

//  icmdata.YawVelocity =(float)icm_gyro_z / 16.4 + 0.36;//-1.33;  //
  //    //偏航角速度偏置（校正）
  //    if(SystemData.SpeedData.Length*OX<=1)
  //    icmdata.YawVelocity_offset=icmdata.YawVelocity;
  //俯仰角度
  icmdata.Pitch = atan(icmdata.ACCEL_Z / icmdata.ACCEL_X) * 57.3;
  if (icmdata.Pitch > 0)
    icmdata.Pitch = 90 - icmdata.Pitch;
  else
    icmdata.Pitch = -90 - icmdata.Pitch;

  //偏航角度
  if (  ImageStatus.Road_type == LeftCirque
      ||ImageStatus.Road_type == RightCirque
      ||icm_start_test_cross == 1
      ||ImageStatus.Barn_Flag == 2
      ||SystemData.SpeedData.Length * OX < 100) {
    icmdata.Yaw = (icmdata.YawVelocity) * dt + icmdata.Yaw;
    //        buzzer_off();
  } else {
    icmdata.Yaw = 0;

    //        buzzer_on();
  }
//
//  if (ImageStatus.Road_type == Ramp)
//    ramp_pitch += icm_gyro_y;
//  else
//    ramp_pitch = 0;
}
