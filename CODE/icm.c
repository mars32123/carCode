/*
 * icm.c
 *
 *  Created on: 2021��6��1��
 *      Author: ������
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
  //��ȡ���ٶ�
  icmdata.ACCEL_X = (float)icm_acc_x / 4096.0;
  icmdata.ACCEL_Z = (float)icm_acc_z / 4096.0;
  //��ȡ���ٶ�
  if(car_which==0)
      icmdata.YawVelocity =(float)icm_gyro_z / 16.4 + 0.36;
  else if(car_which==1)
      icmdata.YawVelocity =(float)icm_gyro_z / 16.4 + 0.36-1.33;

//  icmdata.YawVelocity =(float)icm_gyro_z / 16.4 + 0.36;//-1.33;  //
  //    //ƫ�����ٶ�ƫ�ã�У����
  //    if(SystemData.SpeedData.Length*OX<=1)
  //    icmdata.YawVelocity_offset=icmdata.YawVelocity;
  //�����Ƕ�
  icmdata.Pitch = atan(icmdata.ACCEL_Z / icmdata.ACCEL_X) * 57.3;
  if (icmdata.Pitch > 0)
    icmdata.Pitch = 90 - icmdata.Pitch;
  else
    icmdata.Pitch = -90 - icmdata.Pitch;

  //ƫ���Ƕ�
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
