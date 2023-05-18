/*
 * icm.h
 *
 *  Created on: 2021年6月1日
 *      Author: 周文奇
 */

#ifndef CODE_ICM_H_
#define CODE_ICM_H_
#include "zf_stm_systick.h"
void ICM_OneOrderFilter(void);



//ICM数据
typedef struct{

    float ACCEL_X;
    float ACCEL_Y;
    float ACCEL_Z;
    float GYRO_X;
    float GYRO_Y;
    float GYRO_Z;
    //角度
    float Pitch;
    float Yaw;
    float Roll;
    //角速度
    float PitchVelocity;
    float RollVelocity;
    float YawVelocity;
    float YawVelocity_offset;//偏航角速度偏置
}ICMDatatypedef;

extern ICMDatatypedef icmdata;
extern int ramp_pitch;

#endif /* CODE_ICM_H_ */
