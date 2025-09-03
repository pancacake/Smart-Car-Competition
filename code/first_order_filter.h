#ifndef __FIRST_ORDER_FILTER_H__
#define __FIRST_ORDER_FILTER_H__

#include "zf_common_headfile.h"

typedef struct
{
    int16 *gyro_raw_data;
    int16 *acc_raw_data;
    float gyro_ration;
    float angle_temp;
    float acc_ration;
    float call_cycle;
    float filtering_angle;
    float mechanical_zero;
}casade_common_value_struct;


extern casade_common_value_struct balance_casade;
extern casade_common_value_struct steer_balance_casade;
extern uint32 system_count;
extern float origin_roll_gyro;

void first_order_filter_refresh(casade_common_value_struct *filter_value, int16 gyro_raw_data, int16 acc_raw_data);
void balance_casade_init(void);

#endif
