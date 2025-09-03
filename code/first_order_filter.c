#include "first_order_filter.h"
#include "camera.h"
#include "pid.h"

uint32 system_count = 0;
float origin_roll_gyro=0 ;
casade_common_value_struct balance_casade, steer_balance_casade;

void balance_casade_init(void)
{
    balance_casade.gyro_raw_data = &imu660ra_gyro_y;
    balance_casade.acc_raw_data = &imu660ra_acc_x;
    balance_casade.gyro_ration = 9.0f*0.97f;
    balance_casade.acc_ration = 200.0f*0.03f;//200.0f;
    balance_casade.call_cycle = 0.005f;
    balance_casade.mechanical_zero = 0;//-650; //-up -> back

    balance_casade.filtering_angle = -balance_casade.mechanical_zero;
    balance_casade.angle_temp = -balance_casade.mechanical_zero;

    pid_init(&balance_control.ang_v_control, 0.7f, 0.05f, 0.0f, 10000, 10000);
    pid_init(&balance_control.angle_control, 3.000f, 0.00f, 0.6f, 10000, 20000);
    pid_init(&balance_control.speed_control, 3.2f, 0.04f, 1.55f, 1000, 1300);

    //pid_init(&pid_turn_angle, 0.084f, 0.000f, 0.048f, 5000, 8000);
    //pid_init(&pid_turn_angle, 0.096, 0.000f, 0.0540f, 5000, 10000);
    pid_init(&pid_turn_angle, 0.088, 0.000f, 0.0580f, 5000, 10000);

    steer_balance_casade.gyro_raw_data = &imu660ra_gyro_x;
    steer_balance_casade.acc_raw_data = &imu660ra_acc_y;
    steer_balance_casade.gyro_ration = 9.0f*0.985;//0.992f;
    steer_balance_casade.acc_ration = 200.0f*0.015f;//200.0f;
    steer_balance_casade.call_cycle = 0.005f;
    steer_balance_casade.mechanical_zero = 0; //-up -> back

    steer_balance_casade.filtering_angle = -steer_balance_casade.mechanical_zero;
    steer_balance_casade.angle_temp = -steer_balance_casade.mechanical_zero;

    pid_init(&steer_balance_control.ang_v_control, 0.0f, 0.0f, 0.0f, 10000, 10000);
    pid_init(&steer_balance_control.angle_control, -0.000f, -0.006f, -0.0032f, 240, 500);
    pid_init(&steer_balance_control.speed_control, 0.0f, 0.00f, 0.0f, 1000, 10000);
}

void first_order_filter_refresh(casade_common_value_struct *filter_value, int16 gyro_raw_data, int16 acc_raw_data)
{
    float gyro_temp;
    float acc_temp;

    gyro_temp = -gyro_raw_data * filter_value -> gyro_ration;
    acc_temp = (acc_raw_data - filter_value->angle_temp) * filter_value->acc_ration;

    filter_value->angle_temp += ((gyro_temp + acc_temp) * filter_value->call_cycle);
    filter_value->filtering_angle = filter_value->angle_temp + filter_value->mechanical_zero;
}
