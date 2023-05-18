#ifndef CODE_CAR_H_
#define CODE_CAR_H_

#include "Platform_Types.h"
#include "SysSe/Bsp/Bsp.h"
#include "magnet.h"

#define MAGNET_LEFT_PIN (ADC0_CH2_A2)
#define MAGNET_MIDDLE_PIN (ADC0_CH1_A1)
#define MAGNET_RIGHT_PIN (ADC0_CH0_A0)

#define MOTOR_PWM_N (ATOM0_CH4_P02_4) /* ��ת */
#define MOTOR_PWM_P (ATOM0_CH5_P02_5) /* ��ת */
#define STEER_PIN (ATOM0_CH1_P33_9)

#define STEER_PERIOD_MS (20)
#define SPEED_PERIOD_MS (5)
#define SCHED_PERIOD_MS (10)
#define car_which (1)//0���ϳ�  1���³�
//! @brief ÿ����120�����壨1024�߱�������
#define PAUSE_PER_CM (60)

//! @brief û�е�ţ�������ֵ��
#define MAGNET_ZERO (7000)

#define CR_BEGIN \
  {              \
    boolean interrupt_state = disableInterrupts();
#define CR_END                        \
  restoreInterrupts(interrupt_state); \
  }

typedef struct {
  //! @brief ��Ⱥ�ǰ��˵�ϵ��
  uint16 magnetErrorGain;
} ox_config_t;

typedef struct {
  uint32 cpu0_usage, cpu1_usage;
  float battery;

  uint32 seconds;
  //! @brief ��Ŵ�����
  magnet_t magnetSensors;
} ox_status_t;

typedef struct {
  ox_config_t config;
  ox_status_t status;
} byd_ox_t;

extern volatile byd_ox_t Car;  //!< unique global var!

//! @brief ����ѭ��ǰ����
void car_init(void);

//! @brief ����
void car_launch(void);

//! @brief ����ѭ���е���
void car_backstage(void);

//! @brief ���ն�ˢ��һ��״̬��
void car_statusbar(void);

//! @brief ��ת����
void car_stall_protect(void);

//! @brief ������ص�ѹ
void car_battery_sample(void);

#endif /* CODE_CAR_H_ */
