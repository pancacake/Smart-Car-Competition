#include "jump.h"
#include "steer_control.h"
#include "leg_control.h"
#include "balance_car_control.h"

int jump_time=0;
//int barrier_flag = 0 ;

void jump_step_a(int step_num)
{
    switch(step_num)
    {
        case 1:
        {
            steer_duty_set(&steer_1, steer_1.center_num - 2500);
            steer_duty_set(&steer_2, steer_2.center_num + 2500);
            steer_duty_set(&steer_3, steer_3.center_num + 2500);
            steer_duty_set(&steer_4, steer_4.center_num - 2500);
        }break;
        case 2:
        {
            steer_duty_set(&steer_1, steer_1.center_num);
            steer_duty_set(&steer_2, steer_2.center_num);
            steer_duty_set(&steer_3, steer_3.center_num);
            steer_duty_set(&steer_4, steer_4.center_num);
        }break;
        case 3:
        {
            steer_duty_set(&steer_1, steer_1.center_num + 800);
            steer_duty_set(&steer_2, steer_2.center_num - 800);
            steer_duty_set(&steer_3, steer_3.center_num - 800);
            steer_duty_set(&steer_4, steer_4.center_num + 800);
        }break;
        case 4:
        {
            steer_duty_set(&steer_1, -14);
            steer_duty_set(&steer_2, -14);
            steer_duty_set(&steer_3, -14);
            steer_duty_set(&steer_4, -14);
        }break;
        default: break;
    }
}

const jump_control_struct jump_control_config[]=
{
    {0, 21, jump_step_a},
    {21, 50, jump_step_a},
    {50, 66, jump_step_a},
    {66, 86, jump_step_a},
};

const uint8 jump_step = sizeof(jump_control_config) / sizeof(jump_control_struct);

void dynamic_jump_control(void)
{
    if(jump_flag && run_flag)
    {
        jump_time++;

        if(jump_time < jump_control_config[jump_step - 1].max)
        {
            for(int i=0; i< jump_step; i++)
            {
                if(jump_time >= jump_control_config[i].min && jump_time <= jump_control_config[i].max)
                {
                    jump_control_config[i].handler(i);
                    break;
                }
            }
        }
        else
        {
            jump_flag = 0;
            jump_time = 0;
        }
    }
}
