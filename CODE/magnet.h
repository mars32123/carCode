/*
 * magnet.h
 *
 *  Created on: 2021年6月28日
 *      Author: hlK
 */

#ifndef CODE_MAGNET_H_
#define CODE_MAGNET_H_

#include "Platform_Types.h"

typedef struct { 
    //! @brief ADC采集的原始值, 保存的最大值
    uint16 raw, maximum; 
    
    //! @brief 归一化后的浮点值
    float32 normalized;
} magnet_sensor_t;

typedef struct {
    magnet_sensor_t left, middle, right;

    //! @brief 和和差比和或其他给pid的偏差
    float32 sum, error;
    int rawSum;
} magnet_t;

//! @brief 开启后记录每个电感的最大值
extern boolean magnet_record_max;
extern int magnet_max;
void magnet_init(void);
void magnet_routine(void);
boolean magnet_is_derail(void);

#endif /* CODE_MAGNET_H_ */
