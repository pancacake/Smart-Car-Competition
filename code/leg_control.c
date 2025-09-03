#include "leg_control.h"
#include "balance_car_control.h"
#include "pid.h"

int jump_flag = 0;

void dynamic_steer_control(void)
{
    int16 steer_output_duty = 0;
    int16 steer_location_offset[4] = {0};
    int16 steer_target_offset[4] = {0};

    static int16 steer_balance_angle = 0;
    static float steer_output_duty_filter = 0;

    steer_output_duty = -func_limit_ab( -balance_control.speed_control.output / 7, -350, 350) * 6;

    steer_output_duty_filter = (steer_output_duty_filter * 19 + (float)steer_output_duty) / 20.0f;

    if(jump_flag == 0)
    {
        steer_balance_angle = func_limit_ab(steer_balance_control.angle_control.output, -500, 500)*6;
    }
    else
    {
        steer_balance_angle = 0;
    }
    
    steer_location_offset[0] = (steer_1.now_location - steer_1.center_num) * steer_1.steer_dir;
    steer_location_offset[1] = (steer_2.now_location - steer_2.center_num) * steer_2.steer_dir;
    steer_location_offset[2] = (steer_3.now_location - steer_3.center_num) * steer_3.steer_dir;
    steer_location_offset[3] = (steer_4.now_location - steer_4.center_num) * steer_4.steer_dir;

    steer_target_offset[0] =  steer_output_duty_filter - (steer_balance_angle > 0 ? 0: steer_balance_angle );
    steer_target_offset[1] =  steer_output_duty_filter + (steer_balance_angle < 0 ? 0: steer_balance_angle );
    steer_target_offset[2] = -steer_output_duty_filter - (steer_balance_angle > 0 ? 0: steer_balance_angle );
    steer_target_offset[3] = -steer_output_duty_filter + (steer_balance_angle < 0 ? 0: steer_balance_angle );

    if(run_flag == 1)
    {
        if(jump_flag == 0)
        {
            steer_control(&steer_1, func_limit_ab(steer_target_offset[0] - steer_location_offset[0], -10, 10));
            steer_control(&steer_2, func_limit_ab(steer_target_offset[1] - steer_location_offset[1], -10, 10));
            steer_control(&steer_3, func_limit_ab(steer_target_offset[2] - steer_location_offset[2], -10, 10));
            steer_control(&steer_4, func_limit_ab(steer_target_offset[3] - steer_location_offset[3], -10, 10));
        }
    }
    else
    {
        steer_control(&steer_1, func_limit_ab(steer_target_offset[0] - steer_1.now_location, -1, 1) * steer_1.steer_dir);
        steer_control(&steer_2, func_limit_ab(steer_target_offset[1] - steer_2.now_location, -1, 1) * steer_2.steer_dir);
        steer_control(&steer_3, func_limit_ab(steer_target_offset[2] - steer_3.now_location, -1, 1) * steer_3.steer_dir);
        steer_control(&steer_4, func_limit_ab(steer_target_offset[3] - steer_4.now_location, -1, 1) * steer_4.steer_dir);
    }

}
