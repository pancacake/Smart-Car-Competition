/*
 * camera.c
 * Modified by: 二旬小灯拄拐漂移
 * Modified on:2025/07 @ BJTU Weihai Campus
 */

#ifndef CAMERA_H_
#define CAMERA_H_

#include "zf_common_typedef.h"
#include "zf_device_mt9v03x.h"
//#include "math_plus.h"
#include "pid.h"
#define  WHITE_COL_START_Y      (110)
#define  WHITE_COL_END_Y        (30)
#define  WHITE_COL_DIF_START     (8)
#define BUZZER_PIN              (P33_10)
#define MAX_CORNERS 10 // 最多记录10个拐点
#define LED1                    (P20_9)

extern bool inter_flag;

extern float turn_angle;
extern pid_struct_t pid_turn_angle;
extern int16 shift;

extern uint8 stop_flag;
extern uint8 x_write_col;
extern uint8 y_write_col;
extern uint8 dead_zone;
extern uint8 leftline[MT9V03X_H];
extern uint8 rightline[MT9V03X_H];
extern uint8 midline[MT9V03X_H];
extern int16 sar_thre;//差比和阈值
extern uint8 island_flag;
extern uint8 island_state;
extern float island_radius;
extern uint8 left_loss;
extern uint8 right_loss;
//直线
extern uint8 is_right_straight;
extern uint8 is_left_straight;
//十字
extern uint8 is_crossroad;
extern uint8 left_outer;
extern uint8 right_outer;
//extern uint8 traverse;

extern uint8 right_corner;
extern uint8 left_corner;
extern uint8 left_corner_in;//断点检测
extern uint8 left_corner_out;
extern uint8 max_left_corner_val;
extern float circle_count;
extern float speed_integration;
extern float z_delta;
extern IFX_ALIGN(4) uint8  original_image[MT9V03X_H][MT9V03X_W];
extern uint8 stand_up;
extern uint8 is_barrier;
extern uint8 is_thin_lane;
extern uint8 is_single_bridge;
extern uint8 diff_lane;
extern uint8 crossline_time;
extern uint8 crossline_state;
extern float speed_integration_stop;
extern uint8 is_barrier;
extern uint8 higher_y;
extern uint8 barrier_state;

void camera_mt9v03x_init(void);
void save_image(void);
void init(void);
void set_para(void);
void image_boundary_process(void);
bool find_crossline(void);
void detect_crossroad(void);
void detect_track_elements(void);
void reset_corner_flags(void);
void check_straight_lines(void);
static uint8 is_line_straight_mse(const uint8* line, uint8 is_left_line);
void is_left_corner(uint8 high_thresh);
void is_right_corner(uint8 high_thresh);
void difsum_left(uint8 y,uint8 x);
void difsum_right(uint8 y,uint8 x);
void get_longest_write_col(uint8 white_col_start_y,uint8 white_col_end_y,uint8 find_white_col_mov);
void show_line(void);

#endif /* CODE_CAMERA_H_ */
