#include "car.h"

#pragma section all "cpu0_dsram"

int core0_main(void) {
    car_init();//车辆初始化

  while (TRUE) {
    car_backstage();//后台程序
  }
}//227

#pragma section all restore
