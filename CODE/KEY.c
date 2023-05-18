/*
 * KEY.c
 *
 *  Created on: 2021年2月2日
 *      Author: 周文奇
 */

#include "key.h"
#include "UI.h"
#include "multi_button.h"
#include "shell.h"
#include "zf_gpio.h"

static const PIN_enum key_pin[4] = {P22_3, P22_2, P22_1, P22_0},
                      dial_pin[4] = {P33_4, P33_5,P33_13 , P33_12};
static Button buttons[4];

IFX_INLINE static void assignments(void) {
  button_attach(&buttons[1], PRESS_DOWN, ui_key_down_click);
  button_attach(&buttons[0], PRESS_DOWN, ui_key_up_click);
  button_attach(&buttons[0], LONG_PRESS_HOLD, ui_key_up_click);
  button_attach(&buttons[1], LONG_PRESS_HOLD, ui_key_down_click);
  button_attach(&buttons[2], PRESS_DOWN, ui_key_yes_click);
  button_attach(&buttons[3], PRESS_DOWN, ui_key_no_click);
}

void get_dial(void) {
  PRINTF("%d %d %d %d\n\r", gpio_get(dial_pin[0]), gpio_get(dial_pin[1]),
         gpio_get(dial_pin[2]), gpio_get(dial_pin[3]));
}

uint8_t key0_read(void) {
  return gpio_get(key_pin[0]);
}

uint8_t key1_read(void) {
  return gpio_get(key_pin[1]);
}

uint8_t key2_read(void) {
  return gpio_get(key_pin[2]);
}

uint8_t key3_read(void) {
  return gpio_get(key_pin[3]);
}

uint8_t dial_read(uint8_t id) {
  return gpio_get(dial_pin[id]);
}

void key_init(void) {
  button_init(&buttons[0], key0_read, 0);
  button_init(&buttons[1], key1_read, 0);
  button_init(&buttons[2], key2_read, 0);
  button_init(&buttons[3], key3_read, 0);
  assignments();
  for (int i = 0; i < 4; ++i) {
    gpio_init(key_pin[i], GPI, 0, NO_PULL);
    gpio_init(dial_pin[i], GPI, 0, NO_PULL);
    button_start(&buttons[i]);
  }
}
