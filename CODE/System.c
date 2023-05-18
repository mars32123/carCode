/*
 * System.c
 *
 *  Created on: 2021��1��30��
 *      Author: ������
 */

#include "System.h"
#include "car.h"

SystemDatatypdef SystemData;

int Limit(int num, int numH, int numL) {
  if (num > numH)
    num = numH;
  if (num < numL)
    num = numL;
  return num;
}

float get_speed_convert(float val) {
  return val / SPEED_PERIOD_MS * 1000 / PAUSE_PER_CM;
}

int set_speed_convert(float speed) {
  return speed * PAUSE_PER_CM / 1000 * SPEED_PERIOD_MS;
}
