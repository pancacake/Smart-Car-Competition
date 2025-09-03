#ifndef _PID_H
#define _PID_H

#define LIMIT_MIN_MAX(amt,low,high) ((amt)<(low)?(low):((amt)>(high)?(high):(amt)))

#include "zf_common_headfile.h"

typedef struct
{
    float kp;
    float ki;
    float kd;
    float in_max;
    float out_max;
	float in_min;
    float out_min;
    float ref;
    float fdb;
    float err[2];
    float p_out;
    float i_out;
    float d_out;
    int16 output;
}pid_struct_t;

typedef struct
{
    pid_struct_t speed_control;
    pid_struct_t angle_control;
    pid_struct_t ang_v_control;
}pid_casade_t;

extern pid_casade_t balance_control;
extern pid_struct_t speed_control;
extern pid_struct_t angle_control;
extern pid_struct_t ang_v_control;
extern pid_casade_t steer_balance_control;

void pid_init(pid_struct_t *pid,float kp,float ki,float kd, float in_max, float out_max);
float pid_calc(pid_struct_t *pid, float ref, float fdb);
float pid_calc_in(pid_struct_t *pid, float ref, float fdb);



#endif
