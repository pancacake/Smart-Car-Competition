#ifndef __STEER_CONTROL_H__
#define __STEER_CONTROL_H__

#include "zf_common_headfile.h"

#define STEER_1_PWM         (ATOM0_CH0_P21_2)//右上
#define STEER_1_FRE         (300)
#define STEER_1_DIR         (1)
#define STEER_1_CENTER      (5900) //        3000-8400
#define STEER_1_DUTY_MIN    (3300)
#define STEER_1_DUTY_MAX    (8100)

#define STEER_2_PWM         (ATOM0_CH2_P21_4)//左上
#define STEER_2_FRE         (300)
#define STEER_2_DIR         (-1)
#define STEER_2_CENTER      (3500) //        6400-1000
#define STEER_2_DUTY_MIN    (1300)
#define STEER_2_DUTY_MAX    (6100)

#define STEER_3_PWM         (ATOM0_CH3_P21_5)//右下
#define STEER_3_FRE         (300)
#define STEER_3_DIR         (-1)
#define STEER_3_CENTER      (3950) //        6900-1400
#define STEER_3_DUTY_MIN    (1700)
#define STEER_3_DUTY_MAX    (6600)


#define STEER_4_PWM         (ATOM0_CH1_P21_3)//左下
#define STEER_4_FRE         (300)
#define STEER_4_DIR         (1)
#define STEER_4_CENTER      (5250) //        2100-7800
#define STEER_4_DUTY_MIN    (2400)
#define STEER_4_DUTY_MAX    (7500)

typedef struct 
{
    pwm_channel_enum pwm_pin;
    int16 control_frequency;
    int16 steer_dir;
    int16 center_num;
    int16 duty_min;
    int16 duty_max;
    int16 now_location;
}steer_control_struct;

extern steer_control_struct steer_1;
extern steer_control_struct steer_2;
extern steer_control_struct steer_3;
extern steer_control_struct steer_4;

void steer_control_init(void);
void steer_duty_set(steer_control_struct *control_data, int16 duty);
void steer_control(steer_control_struct *control_data, int16 move_num);

#endif
