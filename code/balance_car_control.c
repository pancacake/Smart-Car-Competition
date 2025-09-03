#include "balance_car_control.h"
#include "first_order_filter.h"
#include "pid.h"
#include "small_driver_uart_control.h"
#include "camera.h"

uint8 run_flag = 1;
int16 car_speed = 0;
int16 left_motor_duty;
int16 right_motor_duty;
int16 target_speed;
int16 target_speed_normal = 450;
int16 target_speed_low = 450;
//int16 target_speed_normal = 400;
//int16 target_speed_low = 350;

float single_brigde_distance = 0.8;

void motor_speed_control(void)
{

    if(run_flag)
    {
        if(balance_casade.filtering_angle>1800 || balance_casade.filtering_angle<-1800)//Çã½£±£»¤
           run_flag=0;
       // left_motor_duty  = func_limit_ab(balance_control.ang_v_control.output , -10000, 10000);
       // right_motor_duty = func_limit_ab(balance_control.ang_v_control.output ,-10000, 10000);
        left_motor_duty  = func_limit_ab(balance_control.ang_v_control.output + imu660ra_gyro_z / 2 - pid_turn_angle.output , -10000, 10000);
        right_motor_duty = func_limit_ab(balance_control.ang_v_control.output - imu660ra_gyro_z / 2 + pid_turn_angle.output ,-10000, 10000);
        small_driver_set_duty(-left_motor_duty, right_motor_duty);
    }
    else
        small_driver_set_duty(0,0);
}
