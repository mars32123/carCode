/*
 * magnet.h
 *
 *  Created on: 2021��6��28��
 *      Author: hlK
 */

#ifndef CODE_MAGNET_H_
#define CODE_MAGNET_H_

#include "Platform_Types.h"

typedef struct { 
    //! @brief ADC�ɼ���ԭʼֵ, ��������ֵ
    uint16 raw, maximum; 
    
    //! @brief ��һ����ĸ���ֵ
    float32 normalized;
} magnet_sensor_t;

typedef struct {
    magnet_sensor_t left, middle, right;

    //! @brief �ͺͲ�Ⱥͻ�������pid��ƫ��
    float32 sum, error;
    int rawSum;
} magnet_t;

//! @brief �������¼ÿ����е����ֵ
extern boolean magnet_record_max;
extern int magnet_max;
void magnet_init(void);
void magnet_routine(void);
boolean magnet_is_derail(void);

#endif /* CODE_MAGNET_H_ */
