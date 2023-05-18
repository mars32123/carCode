/*
 * led.h
 *
 *  Created on: 2021Äê6ÔÂ23ÈÕ
 *      Author: hlK
 */

#ifndef CODE_LED_H_
#define CODE_LED_H_

#include "zf_gpio.h"

#define CORE_LED0_PIN (P21_4)
#define CORE_LED1_PIN (P21_5)
#define CORE_LED2_PIN (P20_8)
#define CORE_LED3_PIN (P20_9)

#define LED_TOGGLE(id) (gpio_toggle(CORE_LED ## id ## _PIN))

void led_init(void);
void led_run0(void);

#endif /* CODE_LED_H_ */
