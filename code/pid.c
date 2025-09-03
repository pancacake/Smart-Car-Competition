#include "pid.h"

pid_casade_t balance_control;
pid_casade_t steer_balance_control;

void pid_init(pid_struct_t *pid, float kp, float ki, float kd, float in_max, float out_max)
{
  pid->kp      = kp;
  pid->ki      = ki;
  pid->kd      = kd;
  pid->in_max  = in_max;
  pid->out_max = out_max;
  pid->in_min  = -in_max;
  pid->out_min = -out_max;
}

float pid_calc(pid_struct_t *pid, float ref, float fdb)
{
    pid->ref = ref;
    pid->fdb = fdb;
    pid->err[1] = pid->err[0];
    pid->err[0] = pid->ref - pid->fdb;

    pid->p_out  = pid->kp * pid->err[0];
    pid->i_out += pid->ki * pid->err[0];
    pid->d_out  = pid->kd * (pid->err[0] - pid->err[1]);
    pid->i_out=LIMIT_MIN_MAX(pid->i_out, pid->in_min, pid->in_max);

    pid->output = pid->p_out + pid->i_out + pid->d_out;
    pid->output=LIMIT_MIN_MAX(pid->output, pid->out_min, pid->out_max);
    return pid->output;
}

float pid_calc_in(pid_struct_t *pid, float ref, float fdb)
{
    float error_prev = pid->err[0];     // 保存上一次误差
    float error_prev2 = pid->err[1];    // 保存上上次误差

    pid->ref = ref;
    pid->fdb = fdb;
    pid->err[1] = error_prev;           // 更新上上次误差
    pid->err[0] = pid->ref - pid->fdb;  // 计算当前误差

    // 增量式PID计算
    float delta_output = pid->kp * (pid->err[0] - error_prev) +
                         pid->ki * pid->err[0] +
                         pid->kd * (pid->err[0] - 2 * error_prev + error_prev2);

    // 限制输出增量范围（可选）
    delta_output = LIMIT_MIN_MAX(delta_output, pid->in_min, pid->in_max);

    // 更新总输出
    pid->output += delta_output;

    // 限制最终输出范围
    pid->output = LIMIT_MIN_MAX(pid->output, pid->out_min, pid->out_max);

    return pid->output;
}
