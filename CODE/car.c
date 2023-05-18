#include "car.h"
#include "C_H.h"
#include "DataStore.h"
#include "Fuzzy.h"
#include "MOTOR.h"
#include "SEEKFREE_ICM20602.h"
#include "SEEKFREE_MT9V03X.h"
#include "Steer.h"
#include "System.h"
#include "buzzer.h"
#include "icm.h"
#include "key.h"
#include "led.h"
#include "magnet.h"
#include "scheduler.h"
#include "shell.h"
#include "vt100.h"
#include "zf_ccu6_pit.h"
#include "zf_gpio.h"
#include "zf_gtm_pwm.h"
#include "zf_stm_systick.h"
#include "zf_vadc.h"
#include "csu.h"

#pragma section all "cpu0_dsram"



volatile byd_ox_t Car = {
    .config = {.magnetErrorGain = 100},
    .status = {.magnetSensors = {.left = {.maximum = 1},
                                 .middle = {.maximum = 1},
                                 .right = {.maximum = 1}}}};

void car_backstage(void) {
  shell_run();      //shell通讯
  scheduler_run();  //后台函数
}

static uint8 cnt;
 
void car_launch(void) {   //发车初始化
  SteerControl(steer_middle);
  ips114_display_image_full_screen(gImage_1);//显示校徽
  buzzer_on();
  systick_delay_ms(STM0, 800);
  SystemData.CameraOK = 0;
  SystemData.GO_OK = 1;
  SystemData.MotorOK = 1;
  SystemData.SteerOK = 1;
  buzzer_off();
  Car.status.seconds = 0;   
  SystemData.SpeedData.Length = 0;
  cnt = 0;
  SystemData.Stop = 0;
}

void car_stall_protect(void) {//电机堵转保护
  if (SystemData.GO_OK) {
    ++cnt;
    if (cnt > 2 && SystemData.SpeedData.nowspeed < 30)
      SystemData.Stop = 1;
    if (SystemData.SpeedData.nowspeed > 30)
      cnt = 0;
  }
}

void car_battery_sample(void) {//电池ADC采样
  Car.status.battery =(float)adc_mean_filter(ADC_0, ADC0_CH2_A2, ADC_12BIT, 25) / 4096 * 3.3 /0.39;
}

void car_statusbar(void) {//向串口助手传输数据
  vt_store_cursor();
  vt_move_to(0, 30);
  vt_set_bg_color(VT_B_YELLOW);
  vt_set_font_color(VT_F_BLACK);
  vt_clear_line();
  PRINTF("cpu0 usage: 0x%x", Car.status.cpu0_usage);
  vt_move_to(1, 30);
  vt_clear_line();
  PRINTF("speed: %.3fcm/s", get_speed_convert(SystemData.SpeedData.nowspeed));

  vt_move_to(2, 30);
  vt_clear_line();
  PRINTF("road%d", ImageStatus.Road_type);

  vt_move_to(3, 30);
  vt_clear_line();
  PRINTF("YawV: %f", icmdata.YawVelocity);

  vt_move_to(4, 30);
  vt_clear_line();
  PRINTF("time: %d", Car.status.seconds);

  vt_move_to(5, 30);
  vt_clear_line();
  PRINTF("yawsum: %d", icmdata.Yaw);

  vt_move_to(6, 30);
  vt_clear_line();
  PRINTF("forklenth: %d", forklenth);

  vt_move_to(7, 30);
  vt_clear_line();
  PRINTF("barnlenth: %d", barnlenth);

  vt_move_to(8, 30);
  vt_clear_line();
  PRINTF("ramplenth: %d", ramplenth);

  vt_move_to(9, 30);
  vt_clear_line();
  PRINTF("Fork: %d", ForkLinePointy);

  vt_move_to(10, 30);
  vt_clear_line();
  PRINTF("magnetmax: %d", magnet_max);
  //       vt_move_to(5, 30);
  //       vt_clear_line();
  //       PRINTF("rawsum:%d", Car.status.magnetSensors.rawSum);
  vt_move_to(11, 30);
  vt_clear_line();
  PRINTF("rampnum: %d", rampnum);

  vt_move_to(12, 30);
   vt_clear_line();
   PRINTF("rounds: %d", SystemData.rounds);

  vt_clear_attr();
  vt_restore_cursor();

//  if (ImageStatus.Road_type == Ramp)
//    PRINTF("pitch = %d\n\r", ramp_pitch);
  // PRINTF("%d\t", sterr);
}

void car_init(void) {
  disableInterrupts();
  get_clk();        // 获取时钟频率  务必保留
  shell_init();
  InitMH();         // 模糊控制器初始化
  Data_Settings();  // 数据初始化
  data_load();      // 数据读取
  led_init();
  key_init();           // 按键初始化
  ips114_init();        // 初始化IPS屏幕
  mt9v03x_init();       // 初始化摄像头
  magnet_init();        // 电感初始化
  icm20602_init_spi();  // 陀螺仪初始
  MOTOR_init();         // 电机初始化
  Encoder_init();       // 编码器初始化
  Steer_init();         // 舵机初始化

  pit_interrupt_ms(CCU6_0, PIT_CH0, STEER_PERIOD_MS);  // 舵机中断
  pit_interrupt_ms(CCU6_0, PIT_CH1, SPEED_PERIOD_MS);  // 速度采样及其控制中断
  pit_interrupt_ms(CCU6_1, PIT_CH1, SCHED_PERIOD_MS);  // scheduler

  IfxCpu_emitEvent(&g_cpuSyncEvent);
  IfxCpu_waitEvent(&g_cpuSyncEvent, -1);
  PRINTF("cpu %d starting!\n\r", IfxCpu_getCoreIndex());
  pwm_duty(STEER_PIN, steer_middle);  // 舵机回正.
  buzzer_freq(600);
  enableInterrupts();
  //  icmdata.YawVelocity_offset=(float)icm_gyro_z/16.4;
}

#pragma section all restore
