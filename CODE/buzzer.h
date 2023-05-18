/*
 * buzzer.h
 *
 *  Created on: 2021年6月23日
 *      Author: 周文奇
 */

#ifndef CODE_BUZZER_H_
#define CODE_BUZZER_H_

#include "zf_gtm_pwm.h"

#define BUZZER_PIN (ATOM2_CH0_P33_10)

static const uint16 buzzer_note[7] = {
  330, 370, 415, 440, 494, 554, 622
};

IFX_INLINE static void buzzer_freq(uint32 freq) {
  gtm_pwm_init(BUZZER_PIN, freq, 0);  //蜂鸣器初始化
}

IFX_INLINE static void buzzer_on(void) {
  pwm_duty(BUZZER_PIN, 500);
}

IFX_INLINE static void buzzer_off(void) {
  pwm_duty(BUZZER_PIN, 0);
}

void buzzer_event(void);
#endif /* CODE_BUZZER_H_ */
