/*
 * magnet.c
 *
 *  Created on: 2021年6月28日
 *      Author: hlK
 */

#include "magnet.h"
#include "car.h"
#include "zf_vadc.h"
#include "System.h"
#include "magnet.h"
#pragma section all "cpu0_dsram"

boolean magnet_record_max;

IFX_INLINE static void magnet_sample(void) {

    if(car_which==0){
  Car.status.magnetSensors.left.raw =
      adc_mean_filter(ADC_0, ADC0_CH2_A2, ADC_12BIT, 5);
  Car.status.magnetSensors.middle.raw =
      adc_mean_filter(ADC_0, ADC0_CH1_A1, ADC_12BIT, 5);
  Car.status.magnetSensors.right.raw =
      adc_mean_filter(ADC_0, ADC0_CH0_A0, ADC_12BIT, 5);
    }else if(car_which==1){

        Car.status.magnetSensors.left.raw =
            adc_mean_filter(ADC_0, ADC0_CH0_A0, ADC_12BIT, 5);
        Car.status.magnetSensors.middle.raw =
            adc_mean_filter(ADC_0, ADC0_CH1_A1, ADC_12BIT, 5);
        Car.status.magnetSensors.right.raw =
            adc_mean_filter(ADC_0, ADC0_CH2_A2, ADC_12BIT, 5);

    }
  Car.status.magnetSensors.rawSum = Car.status.magnetSensors.left.raw +
                                     Car.status.magnetSensors.middle.raw;
//                                     +Car.status.magnetSensors.right.raw
}

//! @brief 归一化
IFX_INLINE static void magnet_normalize(magnet_sensor_t* sensor) {
  if (!magnet_record_max)
    sensor->normalized = (float)sensor->raw * 100 / sensor->maximum;
  else if (sensor->raw > sensor->maximum)
    sensor->maximum = sensor->raw;
}

//! @brief 计算偏差给pid用
IFX_INLINE static void magnet_get_error(void) {
  magnet_t* const p = &Car.status.magnetSensors;
  p->error = Car.config.magnetErrorGain *
             (p->left.raw - p->right.raw) / p->rawSum;
}

//! @brief 出轨保护判定
boolean magnet_is_derail(void) {
  const magnet_t* const p = &Car.status.magnetSensors;
  return p->rawSum < MAGNET_ZERO;
}

int magnet_max=0;
void magnet_get_max(void){
    if(       SystemData.SpeedData.Length * OX<30
            &&SystemData.SpeedData.Length * OX>10
            ){
        if(Car.status.magnetSensors.rawSum>magnet_max)
            magnet_max=Car.status.magnetSensors.rawSum;

    }


}
void magnet_routine(void) {
  magnet_sample();
  // magnet_normalize(&Car.status.magnetSensors.left);
  // magnet_normalize(&Car.status.magnetSensors.middle);
  // magnet_normalize(&Car.status.magnetSensors.right);
//  magnet_get_max();
  magnet_get_error();
}

void magnet_init(void) {
  adc_init(ADC_0, ADC0_CH3_A3);
  adc_init(ADC_0, ADC0_CH2_A2);
  adc_init(ADC_0, ADC0_CH1_A1);
  adc_init(ADC_0, ADC0_CH0_A0);
}

#pragma section all restore
