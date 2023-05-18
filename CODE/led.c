/*
 * led.c
 *
 *  Created on: 2021Äê6ÔÂ23ÈÕ
 *      Author: hlK
 */

#include "led.h"

void led_init(void) {
  gpio_init(CORE_LED0_PIN, GPO, 1, PUSHPULL); 
  gpio_init(CORE_LED1_PIN, GPO, 1, PUSHPULL);
  gpio_init(CORE_LED2_PIN, GPO, 1, PUSHPULL);
  gpio_init(CORE_LED3_PIN, GPO, 1, PUSHPULL);
}

void led_run0(void) {
  LED_TOGGLE(0);
}
