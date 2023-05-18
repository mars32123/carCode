/*
 * scheduler.c
 *
 *  Created on: 2021年6月24日
 *      Author: hlK
 */

#include "scheduler.h"
#include "C_H.h"
#include "Key.h"
#include "UI.h"
#include "buzzer.h"
#include "car.h"
#include "icm.h"
#include "led.h"
#include "multi_button.h"
#include "shell.h"
#include "System.h"

#define EVENT_LIST_END \
  { 0, 0, NULL }

typedef void (*event_handler_t)(void);

typedef struct {
  volatile boolean flag;
  const unsigned period;
  const event_handler_t event;
} event_item_t;

static void stop_watch(void) {
  ++Car.status.seconds;
  if (Car.status.seconds > 124)
    SystemData.Stop = 1;
}

//! @brief flag都是0，周期单位10毫秒，函数名别加括号
static event_item_t event_list[] = {//event是函数名 period*10是函数运行周期
    {.flag = 0, .period = 10, .event = cpu_usage_refresh},
    {.flag = 0, .period = 12, .event = menu},
    {.flag = 0, .period = 7,  .event = led_run0},
    {.flag = 0, .period = 10, .event = car_statusbar},
    {.flag = 0, .period = 100,.event = stop_watch},
    {.flag = 0, .period = 1,  .event = button_ticks},
    {.flag = 0, .period = 10, .event = buzzer_event},
    {.flag = 0, .period = 31, .event = car_stall_protect},
    EVENT_LIST_END};

static unsigned scheduler_counter;
volatile static boolean counter_flag;

//! @brief 每次中断后调用，遍历列表，到时间就将标志位置1
static IFX_INLINE void update_flags(void) {
  for (unsigned i = 0; event_list[i].period && event_list[i].event; ++i)
    if (scheduler_counter % event_list[i].period == 0)
      event_list[i].flag = TRUE;
}

void scheduler_tick(void) {
  counter_flag = TRUE;
}

void scheduler_run(void) {
  CR_BEGIN
  if (counter_flag) {
    counter_flag = FALSE;
    ++scheduler_counter;
    update_flags();
  }
  CR_END
  for (unsigned i = 0; event_list[i].period && event_list[i].event; ++i)
    if (event_list[i].flag) {
      // 清标志位，触发事件
      event_list[i].flag = FALSE;
      event_list[i].event();
    }
}

static void wireless_debug(void) {
  int8 buff;
  buff = (char)(ImageStatus.Det_True - ImageStatus.MiddleLine);
  PRINTF("%c", 0x03);
  PRINTF("%c", 0xFC);
  PRINTF("%c", buff);
  PRINTF("%c", 0xFC);
  PRINTF("%c", 0x03);
}
