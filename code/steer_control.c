#include "steer_control.h"

steer_control_struct steer_1;
steer_control_struct steer_2;
steer_control_struct steer_3;
steer_control_struct steer_4;

void steer_control_init(void)
{
    steer_1.pwm_pin             = STEER_1_PWM;
    steer_1.control_frequency   = STEER_1_FRE;
    steer_1.steer_dir           = STEER_1_DIR;
    steer_1.center_num          = STEER_1_CENTER;
    steer_1.duty_min            = STEER_1_DUTY_MIN;
    steer_1.duty_max            = STEER_1_DUTY_MAX;

    steer_2.pwm_pin             = STEER_2_PWM;
    steer_2.control_frequency   = STEER_2_FRE;
    steer_2.steer_dir           = STEER_2_DIR;
    steer_2.center_num          = STEER_2_CENTER;
    steer_2.duty_min            = STEER_2_DUTY_MIN;
    steer_2.duty_max            = STEER_2_DUTY_MAX;

    steer_3.pwm_pin             = STEER_3_PWM;
    steer_3.control_frequency   = STEER_3_FRE;
    steer_3.steer_dir           = STEER_3_DIR;
    steer_3.center_num          = STEER_3_CENTER;
    steer_3.duty_min            = STEER_3_DUTY_MIN;
    steer_3.duty_max            = STEER_3_DUTY_MAX;

    steer_4.pwm_pin             = STEER_4_PWM;
    steer_4.control_frequency   = STEER_4_FRE;
    steer_4.steer_dir           = STEER_4_DIR;
    steer_4.center_num          = STEER_4_CENTER;
    steer_4.duty_min            = STEER_4_DUTY_MIN;
    steer_4.duty_max            = STEER_4_DUTY_MAX;

    steer_1.center_num = steer_1.center_num - 1000 * steer_1.steer_dir;
    steer_2.center_num = steer_2.center_num - 1000 * steer_2.steer_dir;
    steer_3.center_num = steer_3.center_num - 1000 * steer_3.steer_dir;
    steer_4.center_num = steer_4.center_num - 1000 * steer_4.steer_dir;

    steer_1.now_location = steer_1.center_num;
    steer_2.now_location = steer_2.center_num;
    steer_3.now_location = steer_3.center_num;
    steer_4.now_location = steer_4.center_num;

    pwm_init(steer_1.pwm_pin, steer_1.control_frequency, steer_1.now_location);
    pwm_init(steer_2.pwm_pin, steer_2.control_frequency, steer_2.now_location);
    pwm_init(steer_3.pwm_pin, steer_3.control_frequency, steer_3.now_location);
    pwm_init(steer_4.pwm_pin, steer_4.control_frequency, steer_4.now_location);
}

void steer_duty_set(steer_control_struct *control_data, int16 duty)
{
    control_data->now_location = func_limit_ab(duty, control_data->duty_min, control_data->duty_max);
    pwm_set_duty(control_data->pwm_pin, duty);
}

void steer_control(steer_control_struct *control_data, int16 move_num)
{
    control_data->now_location = control_data->now_location + (control_data->steer_dir == 1 ? move_num : -move_num);
    control_data->now_location = func_limit_ab(control_data->now_location, control_data->duty_min, control_data->duty_max);
    pwm_set_duty(control_data->pwm_pin, control_data->now_location);
}
