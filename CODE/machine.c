#include "Platform_Types.h"
#include "System.h"

typedef enum {
  fsm_normal,          // ���κ�����
  fsm_cross,           // ʮ��·��
  fsm_fork,            // ����·��
  fsm_ramp,            // �µ�
  fsm_straight_short,  // ��ֱ�����
  fsm_straight_long,   // ��ֱ�����
  fsm_cirque_left,     // �󻷵�
  fsm_cirque_right,    // �һ���
  fsm_barn_in,         // ���
  fsm_barn_out,        // ����
  fsm_launching,       // ��
  fsm_stop             // ͣ��
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
