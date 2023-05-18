#ifndef CODE_CAR_H_
#define CODE_CAR_H_

#include "Platform_Types.h"
#include "SysSe/Bsp/Bsp.h"
#include "magnet.h"

#define MAGNET_LEFT_PIN (ADC0_CH2_A2)
#define MAGNET_MIDDLE_PIN (ADC0_CH1_A1)
#define MAGNET_RIGHT_PIN (ADC0_CH0_A0)

#define MOTOR_PWM_N (ATOM0_CH4_P02_4) /* 正转 */
#define MOTOR_PWM_P (ATOM0_CH5_P02_5) /* 反转 */
#define STEER_PIN (ATOM0_CH1_P33_9)

#define STEER_PERIOD_MS (20)
#define SPEED_PERIOD_MS (5)
#define SCHED_PERIOD_MS (10)
#define car_which (1)//0是老车  1是新车
//! @brief 每厘米120个脉冲（1024线编码器）
#define PAUSE_PER_CM (60)

//! @brief 没有电磁（出轨阈值）
#define MAGNET_ZERO (7000)

#define CR_BEGIN \
  {              \
    boolean interrupt_state = disableInterrupts();
#define CR_END                        \
  restoreInterrupts(interrupt_state); \
  }

typedef struct {
  //! @brief 差比和前面乘的系数
  uint16 magnetErrorGain;
} ox_config_t;

typedef struct {
  uint32 cpu0_usage, cpu1_usage;
  float battery;

  uint32 seconds;
  //! @brief 电磁传感器
  magnet_t magnetSensors;
} ox_status_t;

typedef struct {
  ox_config_t config;
  ox_status_t status;
} byd_ox_t;

extern volatile byd_ox_t Car;  //!< unique global var!

//! @brief 在主循环前调用
void car_init(void);

//! @brief 发车
void car_launch(void);

//! @brief 在主循环中调用
void car_backstage(void);

//! @brief 在终端刷新一个状态条
void car_statusbar(void);

//! @brief 堵转保护
void car_stall_protect(void);

//! @brief 测量电池电压
void car_battery_sample(void);

#endif /* CODE_CAR_H_ */
