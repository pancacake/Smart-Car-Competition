#ifndef __BALANCE_CAR_CONTROL_H__
#define __BALANCE_CAR_CONTROL_H__

#include "zf_common_headfile.h"

#define MIN(a,b)    ((a) < (b) ? (a) : (b))

extern int16 left_motor_duty;
extern int16 right_motor_duty;

extern uint8 run_flag;
extern int16 car_speed;
extern int16 target_speed;
extern float single_brigde_distance;
extern int16 target_speed_normal;
extern int16 target_speed_low;

void motor_speed_control(void);

#endif
