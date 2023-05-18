#include "Platform_Types.h"
#include "System.h"

typedef enum {
  fsm_normal,          // 无任何特征
  fsm_cross,           // 十字路口
  fsm_fork,            // 三岔路口
  fsm_ramp,            // 坡道
  fsm_straight_short,  // 短直道变参
  fsm_straight_long,   // 长直道冲刺
  fsm_cirque_left,     // 左环岛
  fsm_cirque_right,    // 右环岛
  fsm_barn_in,         // 入库
  fsm_barn_out,        // 出库
  fsm_launching,       // 起步
  fsm_stop             // 停车
} fsm_state_t;

static fsm_state_t current_state;

boolean straight_test_week(void) {
  return TRUE;
}

boolean straight_test_strict(void) {
  return FALSE;
}

boolean stall_test(void) {
  return TRUE;
}

void straight_handle(void) {}

void fsm_handle(void) {
  switch (current_state) {
    case fsm_normal:

      break;

    case fsm_straight_short:
      straight_handle();
      break;

    case fsm_launching:
      SystemData.MotorOK = SystemData.SteerOK = TRUE;
      break;

    case fsm_stop:
      SystemData.MotorOK = SystemData.SteerOK = FALSE;
      break;

    default:
      break;
  }
}

void fsm_transfer(void) {
  switch (current_state) {
    case fsm_normal:
      if (straight_test_week())
        current_state = fsm_straight_short;
      break;

    case fsm_straight_short:
      if (straight_test_strict())
        current_state = fsm_straight_long;
      else if (!straight_test_week())
        current_state = fsm_normal;
      break;

    case fsm_launching:
      if (SystemData.SpeedData.nowspeed >
          SystemData.SpeedData.expect_True_speed - 20)
        current_state = fsm_normal;

    default:
      break;
  }
  if (current_state < fsm_launching && stall_test())
    current_state = fsm_stop;
}
