/*
 * camera.c
 * Modified by: 二旬小灯拄拐漂移
 * Modified on:2025/07 @ BJTU Weihai Campus
 */
#include "zf_device_mt9v03x.h"
#include "zf_common_debug.h"
#include "zf_common_font.h"
#include "zf_driver_gpio.h"
#include "camera.h"
#include "balance_car_control.h"
#include "spi_wifi.h"
#include "first_order_filter.h"
#include "jump.h"
#include "leg_control.h"

//
bool inter_flag = 0;
float turn_angle = 0;
uint8 stop_flag = 0;
uint8 x_write_col;
uint8 y_write_col;
uint8 dead_zone = 2;//死区
uint8 leftline[MT9V03X_H];
uint8 rightline[MT9V03X_H];
uint8 midline[MT9V03X_H];
IFX_ALIGN(4) uint8  original_image[MT9V03X_H][MT9V03X_W];
pid_struct_t pid_turn_angle;
int16 shift=0;
typedef enum
{
    CORNER_TYPE_NONE,
    CORNER_TYPE_OUTER, // 外拐点
    CORNER_TYPE_INNER // 内拐点
} CornerType; //拐点类型

typedef struct
{
    uint8 x;
    uint8 y;
    CornerType type;
} CornerPoint; //拐点结构

CornerPoint g_left_corners[MAX_CORNERS]; //左侧拐点数组
CornerPoint g_right_corners[MAX_CORNERS];//右侧拐点数组
uint8 g_left_corner_count = 0;//左拐点数量
uint8 g_right_corner_count = 0;//右拐点数量

CornerType corner_type = CORNER_TYPE_NONE; //初始化
uint8 left_inner =0;//四个点的数量
uint8 left_outer = 0;
uint8 right_inner = 0;
uint8 right_outer = 0;

//巡线
int16 sar_thre = 28;//差比和阈值
uint8 general_vis = 75;
//停车
uint8 crossline_time = 0; //检测过斑马线的次数，第二次停车
uint8 crossline_state = 0;
float speed_integration_stop = 0;
//直线
uint8 long_of_midline = 0;
uint8 is_left_straight = 0;
uint8 is_right_straight = 0;
//十字
uint8 is_crossroad = 0;
float cross_dis = 0.3f; //过十字防止误判其他元素
uint8 crossroad_vis = 40;
//丢线
uint8 left_loss = 0;
uint8 right_loss = 0;
//环岛
uint8 bound = MT9V03X_W /2; //约束case2的直道部分的误判
uint8 island_flag =0; //island_flag = 1是左环岛，=2是右环岛
float speed_integration = 0;
float circle_count = 0.0;
float z_delta =0.0;
uint8 island_state = 0;
uint8 left_corner = 0;
uint8 right_corner = 0;
uint8 case1_shift = 46;
uint8 case1_vis = 80;
uint8 case2_shift = 35;
uint8 case2_vis = 95;
float case2_straight_part = 0.2f;
uint8 case2_straight_shift = 35;
uint8 case3_shift = 35;
uint8 case3_vis = 75;
//环岛2状态临界值
float island_in_dis = 0.66f; //进环岛固定值
float island_out_dis = 0.65f; //出环硬直
float island_radius = 0.95f;
//拐点
uint8 corner_low = 95;//不考虑95行以后的拐点
uint8 single_bridge_detect_thresh = 30;//单边桥提前判
uint8 normal_detect_thresh = 65; //不是单边桥，正常判
uint8 step = 1; //隔几个点判断
uint8 threshold = 35; //两个diff加起来需要超过多少距离，才判断为拐点
uint8 X_MIN_BOUNDARY = 20;
uint8 X_MAX_BOUNDARY = MT9V03X_W - 20; // 168
uint8 CENTER_LINE_X = MT9V03X_W / 2; // 94
uint8 X_RELATIVE_DIST_MAX = 30; // 同侧拐点最大水平距离
uint8 Y_RELATIVE_DIST_MAX = 30; // 同侧拐点最大垂直距离
//单边桥
uint8 is_single_bridge = 0;//直接判断是否是单边桥
uint8 stand_up = 0;//抬腿flag
uint8 single_bridge_dis = 1.5f; //单边桥距离积分
//横断
uint8 no_barrier = 0;
uint8 is_barrier = 0;
uint8 higher_y = 0;
uint8 barrier_state = 0;

//停车
uint8 stop_edge = 20; //停车线检测的横向范围
uint8 stop_thre = 30; //停车线突变的差比和阈值
uint8 stop_count = 14; //突变点临界值个数

//结束变量定义
void camera_mt9v03x_init(void)
{
    zf_log(!mt9v03x_init(), "mt9v03x init error");
}


void save_image(void)
{
    memcpy(original_image[0], mt9v03x_image[0], MT9V03X_IMAGE_SIZE);
}


void init(void)
{

    //基本巡线
    long_of_midline = 0;
    //turn_angle = 0;
    left_loss = 0;
    right_loss = 0;
    left_corner = 0;
    right_corner = 0;
    is_left_straight = 0;
    is_right_straight = 0;
}

void image_boundary_process(void)
{
    init();//参数归零
    uint8 row;//行
    uint8 start_col_L;
    uint8 start_col_R;//各行起点的列坐标,默认为MT9V03X_W / 2
    uint8 vis; //控制巡线距离
    float num_of_outpoint = 0;
    //两侧线、中线的长度分配
    memset(leftline,0,sizeof(leftline));
    memset(midline,0,sizeof(rightline));
    memset(rightline,188,sizeof(rightline));
    long_of_midline = WHITE_COL_START_Y - y_write_col;
    //set_para();
    if (long_of_midline>=5)
    {
        if (stop_flag != 2)
        stop_flag =0;
        start_col_L = x_write_col;
        start_col_R = x_write_col;
        //找出边界
        for(row = WHITE_COL_START_Y; row >= y_write_col; row--)
        {
           difsum_left(row,LIMIT_MIN_MAX(start_col_L, dead_zone, MT9V03X_W-1-dead_zone));
           difsum_right(row,LIMIT_MIN_MAX(start_col_R, dead_zone, MT9V03X_W-1-dead_zone));
           midline[row] = (leftline[row]+rightline[row])/2;
           if (leftline[row]<5 || rightline[row] >MT9V03X_W-5)
               num_of_outpoint++;
           if (leftline[row]<5 && rightline[row] >MT9V03X_W-5)
               num_of_outpoint-=5;
        }

        //判定部分
        //调用元素识别的函数，修改相应的flag。后面会根据flag进入不同的条件
        //find_crossline();
        check_straight_lines();
        detect_crossroad();
        detect_track_elements();
        vis = general_vis;
//        if (crossline_time == 1)
//        {
//            gpio_set_level(BUZZER_PIN, GPIO_HIGH);
//            speed_integration = 0;
//        }
        //
        //单边桥
        if(is_single_bridge == 1 && island_flag == 0 && is_barrier == 0)
        {
            gpio_set_level(BUZZER_PIN, GPIO_HIGH);
            stand_up = 1;
        }
//        //十字
//        else if(detect_crossroad())
//        {
//            gpio_set_level(BUZZER_PIN, GPIO_HIGH);
//            is_crossroad = 1;
//            speed_integration = 0;
//        }
        //判断环岛：现在非环岛；只有一边有拐角；只有一边是直线
        else if (island_flag == 0 && left_corner == 1 && right_corner == 0
                && is_right_straight == 1 && is_left_straight == 0 && is_single_bridge == 0)
        {
            gpio_set_level(BUZZER_PIN, GPIO_HIGH);
            island_flag = 1;
            island_state = 1;
            speed_integration = 0;
        }
        else if (island_flag ==0 && right_outer == 1 && left_outer == 0 && left_inner == 0 && right_inner == 0
                && is_left_straight == 1 && is_right_straight == 0 && is_single_bridge == 0)
        {
            gpio_set_level(BUZZER_PIN, GPIO_HIGH);
            island_flag = 2;
            island_state = 1;
            speed_integration = 0;
        }
        ////实施部分
//        if (stand_up == 1)
//        {
//            if (speed_integration >= single_brigde_distance)
//            {
//                gpio_set_level(BUZZER_PIN,GPIO_LOW);
//                stand_up = 0;
//                speed_integration = 0;
//            }
//        }
        //停车
/*      if (crossline_time == 0)
        {
            switch(crossline_state)
            {
                case 0:
                {
                    if (find_crossline())
                    {
                        crossline_state = 1;
                        speed_integration = 0;
                    }
                    break;
                }
                case 1:
                {
                    if (speed_integration > 1.4f)
                    {
                        speed_integration = 0;
                        crossline_state = 2;
                    }
                    break;
                }
                case 2:
                {
                    if (find_crossline())
                    {
                        if (speed_integration < 0.5f)
                            gpio_set_level(BUZZER_PIN, GPIO_HIGH);
                        else
                        {
                            run_flag = 0;
                            gpio_set_level(BUZZER_PIN, GPIO_LOW);
                        }
                    }
                    break;
                }
            }
        }*/
//        if (crossline_time == 0)
//        {
//           if (find_crossline())
//           {
//              crossline_time = 1;
//           }
//        }
//        else
//        {
//           if (find_crossline() && crossline_state ==0 && system_count >= 20000 )
//           {
//               crossline_time = 2;
//               crossline_state = 1;
//               speed_integration_stop = 0;
//           }
//
//
//
//
//         if (speed_integration_stop > 0.3f )
//          {
//               run_flag = 0;
//               gpio_set_level(BUZZER_PIN, GPIO_HIGH);
//                              gpio_set_level(LED1, GPIO_HIGH);
//           }
//        }

        if (is_crossroad == 1 && is_barrier == 0)
        {
            uint8 a, count;
            count = 0;
            float sum_midline = 0.0f;
            // Find the two outer corners (assuming they exist since is_crossroad==1)
            CornerPoint left_outer_pt = {0, 0, CORNER_TYPE_NONE};
            for (uint8 i = 0; i < g_left_corner_count; i++) {
                if (g_left_corners[i].type == CORNER_TYPE_OUTER) {
                    left_outer_pt = g_left_corners[i];
                    break;
                }
            }
            CornerPoint right_outer_pt = {0, 0, CORNER_TYPE_NONE};
            for (uint8 i = 0; i < g_right_corner_count; i++) {
                if (g_right_corners[i].type == CORNER_TYPE_OUTER) {
                    right_outer_pt = g_right_corners[i];
                    break;
                }
            }
            // Determine the max y (a)
            if (left_outer_pt.y >= right_outer_pt.y) a = left_outer_pt.y;
            else a = right_outer_pt.y;
            //对拐点以下的中线求和
            for (uint8 row = WHITE_COL_START_Y; row > a; row--) {
                sum_midline += (float)midline[row];
                count++;
            }
            float avg_midline = (count > 0) ? (sum_midline / (float)count) : (float)(MT9V03X_W / 2);
            //给拐点以上的中线赋值
            for (uint8 row = a; row >= WHITE_COL_END_Y; row--) {
                midline[row] = (uint8)avg_midline;
            }
            //vis = crossroad_vis;
            //退出条件：距离
            if (speed_integration >= cross_dis)
            {
                is_crossroad = 0;
                speed_integration = 0;
                gpio_set_level(BUZZER_PIN, GPIO_LOW);
            }
        }
        //左环岛
        else if (island_flag == 1 && is_barrier == 0)
        {
            switch(island_state)
            {
                case 1://入环僵直
                {
                    for(row = WHITE_COL_START_Y; row >= y_write_col; row--)
                        midline[row] = rightline[row] - case1_shift;
                    vis = case1_vis;
                    //状态转移判定
                    if (speed_integration >= island_in_dis + 0.12)
                    {
                        island_state = 2;
                        speed_integration = 0;
                        circle_count = 0;
                    }
                break;
                }
                case 2: //寻左线，陀螺仪积分
                {
                    //gpio_set_level(BUZZER_PIN, GPIO_HIGH);
                    for(row = WHITE_COL_START_Y; row >= 5; row--)
                    {
                        if (leftline[row] <= (MT9V03X_W /2))
                        {
                            midline[row] = leftline[row] + case2_shift;
                            if (midline[row] < bound)
                                bound = midline[row]; //找到最左的中线去巡线
                        }
                        else
                            midline[row] = bound;
                        if (speed_integration <= case2_straight_part && midline[row] >MT9V03X_W/2 ) //刚入环的时候巡线不稳，再寻一段边线
                        {
                            midline[row] = rightline[row] - case2_straight_shift;
                        }
                    }

                    vis = case2_vis;
                    //出环判定
                    if (circle_count >= 361)
                    {
                        island_state = 3;
                        speed_integration = 0;
                        //gpio_set_level(BUZZER_PIN, GPIO_LOW);
                    }
                    else if (speed_integration > 1 && speed_integration / (circle_count / 180 * 3.14159265) > island_radius)
                    {
                        island_state = 0;
                        island_flag = 0;
                        speed_integration = 0;
                        //gpio_set_level(BUZZER_PIN, GPIO_LOW);
                        bound = 0;
                        gpio_set_level(BUZZER_PIN, GPIO_LOW);
                    }
                break;
                }
                case 3:
                {
                    //gpio_set_level(BUZZER_PIN, GPIO_HIGH);
                    //出环僵直
                    for(row = WHITE_COL_START_Y; row >= y_write_col; row--)
                       midline[row] = rightline[row] - case3_shift;
                    vis = 75;
                    if (speed_integration >= island_out_dis)
                    {
                        island_state = 0;
                        island_flag = 0;
                        speed_integration = 0;
                        //gpio_set_level(BUZZER_PIN, GPIO_LOW);
                        bound = 0;
                        gpio_set_level(BUZZER_PIN, GPIO_LOW);
                    }
                break;
                }
            }
        }
        else if (island_flag == 2 && is_barrier == 0)
        {
            switch(island_state)
            {
                case 1://入环僵直，寻左线
                {
                    for(row = WHITE_COL_START_Y; row >= y_write_col; row--)
                        midline[row] = leftline[row] + case1_shift;
                    vis = case1_vis;
                    //状态转移判定
                    if (speed_integration >= island_in_dis)
                    {
                        island_state = 2;
                        speed_integration = 0;
                        circle_count = 0;
                    }
                break;
                }
                case 2: //寻右线，陀螺仪积分
                {
                    for(row = WHITE_COL_START_Y; row >= y_write_col; row--)
                    {

                        if (rightline[row] >= (MT9V03X_W /2))
                        {
                            midline[row] = rightline[row] - case2_shift;
                            if (midline[row] > bound)
                                bound = midline[row]; //找到最左的中线
                        }
                        else
                            midline[row] = bound;
//                        if (speed_integration <= case2_straight_part && midline[row] >MT9V03X_W/2 )
//                        {
//                            midline[row] = leftline[row]+ case2_straight_shift;
//                        }

                    }
                    vis = case2_vis;
                    if (circle_count <= -361)
                    {
                        island_state = 3;
                        speed_integration = 0;
                    }
                    else if (speed_integration > 0.8 && speed_integration / (circle_count / 180 * 3.14159265) < -island_radius)
                    {
                        island_state = 0;
                        island_flag = 0;
                        speed_integration = 0;
                        //gpio_set_level(BUZZER_PIN, GPIO_LOW);
                        bound = 0;
                        gpio_set_level(BUZZER_PIN, GPIO_LOW);
                    }
                break;
                }
                case 3:
                {
                    //出环僵直，寻左线
                    for(row = WHITE_COL_START_Y; row >= y_write_col; row--)
                       midline[row] = leftline[row] + case3_shift;
                    vis = case3_vis;
                    if (speed_integration >= island_out_dis)
                    {
                        island_state = 0;
                        island_flag = 0;
                        gpio_set_level(BUZZER_PIN, GPIO_LOW);
                        speed_integration = 0;
                    }
                break;
                }
            }//switch的括号
        }//else if的括号
        //结束环岛

        //巡线
        if (y_write_col > vis)
        {
            for (row = y_write_col; row < y_write_col+5; row++)
                turn_angle += midline[row];
            turn_angle = turn_angle/5;
            turn_angle -= MT9V03X_W/2;
        }
        else
        {
            for (row = vis; row <vis+5; row++)
                turn_angle += midline[row];
            turn_angle = turn_angle/5;
            turn_angle -= MT9V03X_W/2;
        }
        if(num_of_outpoint >= 20) num_of_outpoint = 20;
        if(num_of_outpoint <= 0) num_of_outpoint = 0;
        turn_angle = turn_angle * (1.0f + 0.007*num_of_outpoint);

        //}

    }
    //如果是横断在这限制turn_angle
    if (is_barrier == 1)
    {
        /*jump_flag = 1;
        if(inter_flag == 0)
        {
            speed_integration = 0;
            inter_flag = 1;
        }
        if(speed_integration >= (0.25 - (car_speed * 0.21 / 60) * 0.25) )
        {
            dynamic_jump_control();
            inter_flag = 0;
        }*/
        if(gpio_get_level(P33_11))
        {
        switch (barrier_state)
        {
            case 1://右拐，出赛道
            {
                turn_angle = 120;
                if (circle_count < -65)
                {
                    circle_count = 0;
                    barrier_state = 2;
                    speed_integration = 0;
                }
                break;
            }
            case 2:
            {
                turn_angle = -65; //姿态和赛道
                if (circle_count > 70)
                {
                    circle_count = 0;
                    speed_integration = 0;
                    //barrier_state = 4;
                    is_barrier = 0;
                    barrier_state = 0;
                    //这里可以直接提速
                    target_speed = target_speed_normal ;
                }
                break;
            }
        }}
    }
    pid_calc(&pid_turn_angle, 0.0f, turn_angle);
}

///*
// * 停车函数
// */
//void find_crossline(void)
//{
//
//    float sum,dif,sar;//和，差，比
//    uint8 col;//列
//    uint8 change_count = 0;
//    uint8 mov = 1;//每次作差后的移动量,默认为2，可以根据画面分辨率调整
//    //计算第x行的右边界
//    for(col = 94 - stop_edge; col <= 94 + stop_edge ; col += mov)
//    {
//        dif = (float)((original_image[col][70] - original_image[col + mov + 1][70])<<8);//左移8位即乘256，可避免浮点数乘，加快速度
//        sum = (float)((original_image[col][70] + original_image[col + mov + 1][70]));
//        sar = fabs(dif / sum);//求取差比和
//        if(sar > stop_thre)
//        {//差比和大于阈值代表深浅色突变
//            change_count++;
//            if(change_count >= stop_count && crossline_time == 0)
//            {
//                crossline_time = 1;
//                //run_flag = 0;
//            }
//            else if (change_count >= stop_count && crossline_time == 1)
//                crossline_time = 2;
//        }
//    }
//}

// 新的斑马线检测函数，替换原有find_crossline
bool find_crossline(void)
{
    // 参数区间，可根据实际情况调整
    uint8 low_stop = 60;     // y区间下界
    uint8 high_stop = 80;   // y区间上界
    uint8 x_left = 50;  // 检测起始x
    uint8 x_right = MT9V03X_W - 50; // 检测终止x
    uint8 mov = 1;   // 检测步长
    float sar_thre_local = 25; // 差比和阈值
    uint8 row_thresh = 10;   // 至少多少行满足条件
    uint8 col_thresh = 10;   // 单行突变点个数阈值
    uint8 row_count = 0;
    for(uint8 y = low_stop; y <= high_stop; y++)
    {
        uint8 change_count = 0;
        for(uint8 col = x_left; col < x_right; col += mov)
        {
            float sum = (float)(original_image[y][col] + original_image[y][col + mov]);
            float dif = (float)((original_image[y][col] - original_image[y][col + mov]) << 8);
            float sar = fabs(dif / sum);
            if(sar > sar_thre_local)
            {
                change_count++;
            }
        }
        if(change_count >= col_thresh)
        {
            row_count++;
        }
    }
    if(row_count >= row_thresh)
    {
        return true;
//       if (crossline_time == 0)
//       {
//           crossline_time = 1;
//       }
//       else if (crossline_time == 1)
//       {
//           if (speed_integration > 1.0f)
//               crossline_time = 2;
//       }
//       else if (crossline_time == 2)
//           run_flag = 0;
    }
    else
        return false;
}

// New function to detect crossroads with additional conditions
void detect_crossroad(void) {

    if (island_flag != 0 || is_single_bridge != 0 || left_outer != 1 || right_outer != 1 ||
        left_inner != 0 || right_inner != 0 || is_barrier != 0) {
        //is_barrier = 0;
        is_crossroad = 0;
    }
    else
    {
        // Find left outer corner
        CornerPoint left_outer_pt = {0, 0, CORNER_TYPE_NONE};
        for (uint8 i = 0; i < g_left_corner_count; i++) {
            if (g_left_corners[i].type == CORNER_TYPE_OUTER) {
                left_outer_pt = g_left_corners[i];
                break;
            }
        }
        // Find right outer corner
        CornerPoint right_outer_pt = {0, 0, CORNER_TYPE_NONE};
        for (uint8 i = 0; i < g_right_corner_count; i++) {
            if (g_right_corners[i].type == CORNER_TYPE_OUTER) {
                right_outer_pt = g_right_corners[i];
                break;
            }
        }
        //获得一个y值
        if (left_outer_pt.y < right_outer_pt.y) higher_y = left_outer_pt.y;
        else higher_y = right_outer_pt.y;
        bool crossroad = false;
        for (int row = higher_y=10; row>= WHITE_COL_END_Y; row--)
        {
            if (midline[row] > 20 && midline[row] < 160)
                crossroad = true;
        }
        if (crossroad == true)
        {
            is_crossroad = 1;
            //is_barrier = 0;
        }
        else
        {
            is_barrier = 1;
            barrier_state = 1;
            is_crossroad = 0;
            circle_count = 0;
        }
    }
}

//bool detect_crossroad(void) {
//    uint8 higher_y;
//    if (island_flag != 0 || is_single_bridge != 0 || left_outer != 1 || right_outer != 1 ||
//        left_inner != 0 || right_inner != 0) {
//        no_barrier = 1;
//        return false;
//    }
//
//    // Find left outer corner
//    CornerPoint left_outer_pt = {0, 0, CORNER_TYPE_NONE};
//    for (uint8 i = 0; i < g_left_corner_count; i++) {
//        if (g_left_corners[i].type == CORNER_TYPE_OUTER) {
//            left_outer_pt = g_left_corners[i];
//            break;
//        }
//    }
//
//    // Find right outer corner
//    CornerPoint right_outer_pt = {0, 0, CORNER_TYPE_NONE};
//    for (uint8 i = 0; i < g_right_corner_count; i++) {
//        if (g_right_corners[i].type == CORNER_TYPE_OUTER) {
//            right_outer_pt = g_right_corners[i];
//            break;
//        }
//    }
//    //find_barrier
//    if (left_outer_pt.y < right_outer_pt.y) higher_y = left_outer_pt.y;
//    else higher_y = right_outer_pt.y;
//    for (int row = higher_y; row>= WHITE_COL_END_Y; row--)
//    {
//        if (midline[row] > 10 && midline[row] < 178)
//            no_barrier = 1;
//    }
//    if (no_barrier == 0)
//    {
//        barrier_flag = 1;
//        return false;
//    }
//
//    // Check y coordinates > 60
//    if (left_outer_pt.y <= 60 || right_outer_pt.y <= 60) {
//        return false;
//    }
//
//    // Check x difference between 50 and 70
//    int16 x_diff = (int16)right_outer_pt.x - (int16)left_outer_pt.x;
//    if (x_diff < 40 || x_diff > 80) {
//        return false;
//    }
//    return true;
//}

/**
 * 判断是否单边桥，如果是否则按原先逻辑判断
 */
void detect_track_elements(void)
{
    // --- 第一轮检测：使用灵敏阈值尝试识别单边桥 ---
    reset_corner_flags();
    is_left_corner(single_bridge_detect_thresh);
    is_right_corner(single_bridge_detect_thresh);
    // 检查单边桥的特定拐点组合
    // --- 全新的、更严格的单边桥判断逻辑 ---
    // 1. 定义几何与相对位置约束


    //左侧单边桥
    if (left_inner == 1 && left_outer == 1 && right_inner == 1 &&
            g_left_corner_count == 2 && g_right_corner_count == 1)
 //   if (left_inner == 1 && left_outer == 1 )
    {
        // 提取对应的三个拐点
        CornerPoint left_inner_pt, left_outer_pt;
        CornerPoint right_inner_pt = g_right_corners[0]; // 右侧只有一个，直接获取
        // 遍历找到左侧的内外拐点
        for (uint8 i = 0; i < g_left_corner_count; i++)
        {
            if (g_left_corners[i].type == CORNER_TYPE_INNER) left_inner_pt = g_left_corners[i];
            else if (g_left_corners[i].type == CORNER_TYPE_OUTER) left_outer_pt = g_left_corners[i];
        }

        // 检查绝对位置
        uint8 abs_pos_ok = (left_inner_pt.x > X_MIN_BOUNDARY && left_inner_pt.x < CENTER_LINE_X) &&
                           (left_outer_pt.x > X_MIN_BOUNDARY && left_outer_pt.x < CENTER_LINE_X) &&
                           (right_inner_pt.x > CENTER_LINE_X && right_inner_pt.x < X_MAX_BOUNDARY);

        // 检查左侧两个拐点的相对位置
        uint8 rel_pos_ok = (abs((int16)left_inner_pt.x - (int16)left_outer_pt.x) < X_RELATIVE_DIST_MAX) &&
                (abs((int16)left_inner_pt.y - (int16)left_outer_pt.y) < Y_RELATIVE_DIST_MAX);

        if (abs_pos_ok && rel_pos_ok)
        {
            is_single_bridge = 1;
            //return; // 所有条件满足，确认是单边桥并返回
        }
    }
//    //右侧单边桥
//    else if (left_outer == 1 && right_inner == 1 && right_outer == 1 &&
//            g_left_corner_count == 1 && g_right_corner_count == 2)
    else if (right_inner == 1 && right_outer == 1)
    {
        // 提取对应的三个拐点
        CornerPoint right_inner_pt, right_outer_pt;
        CornerPoint left_inner_pt = g_left_corners[0]; // 左侧只有一个，直接获取
        // 遍历找到右侧的内外拐点
        for (uint8 i = 0; i < g_right_corner_count; i++)
        {
            if (g_right_corners[i].type == CORNER_TYPE_INNER) right_inner_pt = g_right_corners[i];
            else if (g_right_corners[i].type == CORNER_TYPE_OUTER) right_outer_pt = g_right_corners[i];
        }

        // 检查绝对位置
        uint8 abs_pos_ok = (left_inner_pt.x > X_MIN_BOUNDARY && left_inner_pt.x < CENTER_LINE_X) &&
                           (right_inner_pt.x > CENTER_LINE_X && right_inner_pt.x < X_MAX_BOUNDARY) &&
                           (right_outer_pt.x > CENTER_LINE_X && right_outer_pt.x < X_MAX_BOUNDARY);

        // 检查右侧两个拐点的相对位置
        uint8 rel_pos_ok = (abs((int16)right_inner_pt.x - (int16)right_outer_pt.x) < X_RELATIVE_DIST_MAX) &&
                (abs((int16)right_inner_pt.y - (int16)right_outer_pt.y) < Y_RELATIVE_DIST_MAX);

        if (abs_pos_ok && rel_pos_ok)
        {
            is_single_bridge = 1;
            //return; // 所有条件满足，确认是单边桥并返回
        }
    }
    // --- 第二轮检测：若未识别到单边桥，则使用常规阈值重新检测 ---
    else
    {
        //reset_corner_flags();
        left_inner = 0;
        left_outer = 0;
        right_inner = 0;
        right_outer = 0;
        is_left_corner(normal_detect_thresh);
        is_right_corner(normal_detect_thresh);
        // 明确未检测到单边桥
        is_single_bridge = 0;
    }

}

/**
 * @brief  重置所有与拐点相关的标志位和计数器
 * @param  None
 * @retval None
 * @note   在每次新的元素识别周期开始时调用，确保检测状态干净。
 */
void reset_corner_flags(void)
{
    g_left_corner_count = 0;
    g_right_corner_count = 0;
    left_corner = 0;
    right_corner = 0;
    left_inner = 0;
    left_outer = 0;
    right_inner = 0;
    right_outer = 0;
    is_single_bridge = 0; // Also reset single bridge flag
}


/**
 * @brief  检查左右边界是否为直线。
 * @param  None
 * @retval None
 * @note   该函数调用通用的直线检测算法来更新 is_left_straight 和 is_right_straight 两个全局变量。
 */
void check_straight_lines(void)
{
    // 检查前重置标志位
    is_left_straight = 0;
    is_right_straight = 0;

    // 分别调用通用函数来判断左右边界
    is_right_straight = is_line_straight_mse(rightline, 0);
    is_left_straight = is_line_straight_mse(leftline, 1);
}
/**
 * @brief  使用MSE算法判断单条边界线是否为直线
 * @param  line          [in]  输入的边界线数组 (例如 leftline 或 rightline)
 * @param  is_left_line  [in]  标志位，1表示是左边界线，0表示是右边界线
 * @return uint8         1 表示是直线, 0 表示不是直线
 * @note   这是一个通用的直线检测函数，被 check_straight_lines 调用。
 */
static uint8 is_line_straight_mse(const uint8* line, uint8 is_left_line)
{
    // --- 参数定义 ---
    uint8 row_start = WHITE_COL_START_Y;
    uint8 row_end = WHITE_COL_END_Y;
    float mse_threshold = 5.0f;
    int min_valid_points = (WHITE_COL_START_Y - WHITE_COL_END_Y) * 0.8;
    uint8 ref_row_1 = (WHITE_COL_START_Y + WHITE_COL_END_Y) / 2 + 10;
    uint8 ref_row_2 = (WHITE_COL_START_Y + WHITE_COL_END_Y) / 2 - 10;
    // 1. 检查参考点有效性
    uint8 ref1_valid, ref2_valid;
    if (is_left_line)
    {
        ref1_valid = (leftline[ref_row_1] > dead_zone);
        ref2_valid = (leftline[ref_row_2] > dead_zone);
    }
    else
    {
        ref1_valid = (rightline[ref_row_1] < MT9V03X_W - 1 - dead_zone);
        ref2_valid = (rightline[ref_row_2] < MT9V03X_W - 1 - dead_zone);
    }
    if (ref1_valid && ref2_valid)
    {
        // 2. 定义基准线
        float x1 = (float)line[ref_row_1];
        float y1 = (float)ref_row_1;
        float x2 = (float)line[ref_row_2];
        float y2 = (float)ref_row_2;

        float A = y2 - y1;
        float B = x1 - x2;
        float C = -A * x1 - B * y1;
        float den_sq = A * A + B * B;

        if (den_sq > 1e-6f)
        {
            // 3. 计算均方误差(MSE)
            float sum_sq_dist = 0.0f;
            int valid_points_count = 0;
            for (uint8 row = row_end; row <= row_start; row++)
            {
                uint8 point_valid;
                if (is_left_line)
                    point_valid = (line[row] > dead_zone);
                else
                    point_valid = (line[row] < MT9V03X_W - 1 - dead_zone);

                if (point_valid)
                {
                    float x0 = (float)line[row];
                    float y0 = (float)row;
                    float dist_sq = (A * x0 + B * y0 + C) * (A * x0 + B * y0 + C) / den_sq;
                    sum_sq_dist += dist_sq;
                    valid_points_count++;
                }
            }

            // 4. 根据MSE做出判断
            if (valid_points_count >= min_valid_points)
            {
                float mse = sum_sq_dist / valid_points_count;
                if (mse <= mse_threshold)
                    return 1; // 是直线
            }
        }
    }
    return 0; // 不是直线
}

/**
  * @brief 判断左、右边界是否存在拐点。
  */
void is_left_corner(uint8 high_thresh)
{
    uint8 row;
    int16 diff1, diff2;
    uint8 high_mark; //定义扫描的上界 防止因未扫描全导致的误判
    uint8 low_mark; //定义扫描的下界，防止转弯时误判

    // 每次检查前，重置拐点计数器
    g_left_corner_count = 0;
    left_corner = 0; // 为兼容环岛逻辑，保留此标志位
    //确定扫描的上下界
    if (y_write_col > high_thresh) high_mark = y_write_col;
    else high_mark = high_thresh;
    if (WHITE_COL_START_Y < corner_low) low_mark = WHITE_COL_START_Y;
    else low_mark = corner_low;
    // 从图像底部向上遍历
    for (row = low_mark; row > high_mark; row--)
    {
        // 如果拐点数组已满，则停止搜索
        if (g_left_corner_count >= MAX_CORNERS)
        {
            break;
        }

        // 计算两段差值
        diff1 = leftline[row] - leftline[row - step];
        diff2 = leftline[row - step] - leftline[row - 2 * step];

        // 检查是否满足拐点条件
        //if (diff1 + diff2 > threshold && diff1 > 0 && diff2 > 0)
        if (abs(diff1) > 5 || abs(diff2) > 5)
        {
            if (diff1 + diff2 > threshold)
            {
                corner_type = CORNER_TYPE_OUTER;
                // 记录拐点信息
                g_left_corners[g_left_corner_count].x = leftline[row];
                g_left_corners[g_left_corner_count].y = row;
                g_left_corners[g_left_corner_count].type = corner_type;
                g_left_corner_count++;
                left_corner = 1; // 标记找到了拐点
                left_outer ++;
                row -=2;

            }
            else if (diff1 + diff2 < -threshold && diff1 <= 2 && diff2 <= 2)
            {
                corner_type = CORNER_TYPE_INNER;
                // 记录拐点信息
                g_left_corners[g_left_corner_count].x = leftline[row];
                g_left_corners[g_left_corner_count].y = row;
                g_left_corners[g_left_corner_count].type = corner_type;
                g_left_corner_count++;
                left_corner = 1; // 标记找到了拐点
                left_inner ++;
                row -= 2;
            }
            else
                corner_type = CORNER_TYPE_NONE;
        }
    }
}

void is_right_corner(uint8 high_thresh)
{
    uint8 row;
    int16 diff1, diff2;
    uint8 high_mark; //定义扫描的上界 防止因未扫描全导致的误判
    uint8 low_mark; //定义扫描的下界，防止转弯时误判

    // 每次检查前，重置拐点计数器
    g_right_corner_count = 0;
    right_corner = 0; // 为兼容环岛逻辑，保留此标志位
    //确定扫描的上下界
    if (y_write_col > high_thresh) high_mark = y_write_col;
    else high_mark = high_thresh;
    if (WHITE_COL_START_Y < corner_low) low_mark = WHITE_COL_START_Y;
    else low_mark = corner_low;
    // 从图像底部向上遍历
    for (row = low_mark; row > high_mark; row--)
    {
        // 如果拐点数组已满，则停止搜索
        if (g_right_corner_count >= MAX_CORNERS)
        {
            break;
        }
        // 计算两段差值
        diff1 = rightline[row] - rightline[row - step];
        diff2 = rightline[row - step] - rightline[row - 2 * step];
        // 检查是否满足拐点条件
        //if (diff1 + diff2 > threshold && diff1 > 0 && diff2 > 0)
        if (abs(diff1) > 5 || abs(diff2) > 5)
        {
            if (diff1 + diff2 > threshold)
            {
                corner_type = CORNER_TYPE_INNER;
                // 记录拐点信息
                g_right_corners[g_right_corner_count].x = rightline[row];
                g_right_corners[g_right_corner_count].y = row;
                g_right_corners[g_right_corner_count].type = corner_type;
                g_right_corner_count++;
                right_corner = 1; // 标记找到了拐点
                right_inner ++;
                row -=2;

            }
            else if (diff1 + diff2 < -threshold && diff1 <= 2 && diff2 <= 2)
            {
                corner_type = CORNER_TYPE_OUTER;
                // 记录拐点信息
                g_right_corners[g_right_corner_count].x = rightline[row];
                g_right_corners[g_right_corner_count].y = row;
                g_right_corners[g_right_corner_count].type = corner_type;
                g_right_corner_count++;
                right_corner = 1; // 标记找到了拐点
                right_outer ++;
                row -= 2;
            }
            else
                corner_type = CORNER_TYPE_NONE;
        }
    }
}

/*
 * 最基本的巡线函数，寻左右边线
 */
void difsum_left(uint8 y,uint8 x)
{
    float sum,dif,sar;//和，差，比
    uint8 col;//列
    uint8 mov = 1;//每次作差后的移动量,默认为2，可以根据画面分辨率调整
    //计算第x行的左边界
    //leftline[y] = dead_zone;//未找到左边界时输出为0
    uint8 found_edge =0;
    for(col = x; col >= dead_zone+mov; col -= mov)
        {
        dif = (float)((original_image[y][col] - original_image[y][col - mov - 1])<<8);//左移8位即乘256，可避免浮点数乘，加快速度
        sum = (float)((original_image[y][col] + original_image[y][col - mov - 1]));
        sar = fabs(dif / sum);//求取差比和
        if(sar > sar_thre)
            {//差比和大于阈值代表深浅色突变
            leftline[y] = (int16)(col - mov);
            found_edge = 1;
            break;//找到边界后退出
            }
        }
    if (!found_edge)
    {
        leftline[y] = dead_zone;
        left_loss++;
    }
}

void difsum_right(uint8 y,uint8 x)
{
    float sum,dif,sar;//和，差，比
    uint8 col;//列
    uint8 mov = 1;//每次作差后的移动量,默认为2，可以根据画面分辨率调整
    //计算第x行的右边界
    //rightline[y] = MT9V03X_W-1-dead_zone;//未找到左右边界时输出为0
    uint8 found_edge = 0;
    for(col = x; col <= MT9V03X_W-mov - 1-dead_zone; col += mov)
        {
        dif = (float)((original_image[y][col] - original_image[y][col + mov + 1])<<8);//左移8位即乘256，可避免浮点数乘，加快速度
        sum = (float)((original_image[y][col] + original_image[y][col + mov + 1]));
        sar = fabs(dif / sum);//求取差比和
        if(sar > sar_thre)
            {//差比和大于阈值代表深浅色突变
            rightline[y] = (int16)(col + mov);
            found_edge = 1;
            break;//找到边界后退出
            }
        }
    if (!found_edge)
    {
        right_loss++;
    }

}

/*
 * 寻找最长白列
 */
void get_longest_write_col(uint8 white_col_start_y,uint8 white_col_end_y,uint8 find_white_col_mov)
{
float sum,dif,gray_dif_sum;
x_write_col = MT9V03X_W/2;
y_write_col = white_col_start_y;
    for (int x=20;x<168;x++)
        {
        if(original_image[white_col_start_y][x]>=140)
        {
            int y = white_col_start_y ;
            for (y = white_col_start_y; y >= find_white_col_mov+1; y -= find_white_col_mov)
            {
                dif = (float)((original_image[y][x] - original_image[y - find_white_col_mov - 1][x])<<8);//左移8位即乘256，可避免浮点数乘，加快速度
                sum = (float)((original_image[y][x] + original_image[y - find_white_col_mov - 1][x]));
                gray_dif_sum = fabs(dif / sum);//求取差比和
                if (gray_dif_sum >= 25)
                {
                    if(y <= y_write_col)
                    {
                    y_write_col=(uint8)y;
                    x_write_col=(uint8)x;
                    }
                    break;
                }
            }
            //y += find_white_col_mov;
            if (y <= white_col_end_y || y+1-find_white_col_mov < find_white_col_mov+1)
            {
            y_write_col=white_col_end_y;
            x_write_col=(uint8)x;
            break;
            }
        }
        }
}

//显示和元素有关的内容
void show_line(void)
{
    for(int8 i = MT9V03X_H-1; i > y_write_col; i --)
    {
        ips200_draw_point((uint16)leftline[i]+shift, i, RGB565_RED);//红色左线
        ips200_draw_point((uint16)rightline[i]+shift, i, RGB565_BLUE);//蓝色右线
        ips200_draw_point(LIMIT_MIN_MAX((int16)midline[i]+shift,shift,shift+MT9V03X_W-1), i, RGB565_PURPLE);//紫色中线
    }
    //拐点
    // 绘制所有记录的左拐点
    for (uint8 i = 0; i < g_left_corner_count; i++)
    {
        uint16 color = RGB565_RED; // 默认颜色为红色
        if (g_left_corners[i].type == CORNER_TYPE_OUTER)
        {
            color = RGB565_GREEN; // 外拐点为绿色
        }
        else if (g_left_corners[i].type == CORNER_TYPE_INNER)
        {
            color = RGB565_CYAN; // 内拐点为青色
        }
        if (g_left_corners[i].x < 186 && g_left_corners[i].y < 118
                && g_left_corners[i].x > 2 && g_left_corners[i].y > 2)
        {
            ips200_draw_point(g_left_corners[i].x , g_left_corners[i].y, color);
            ips200_draw_point(g_left_corners[i].x + 1, g_left_corners[i].y, color);
            ips200_draw_point(g_left_corners[i].x - 1, g_left_corners[i].y, color);
            ips200_draw_point(g_left_corners[i].x + shift, g_left_corners[i].y + 1, color);
            ips200_draw_point(g_left_corners[i].x + shift, g_left_corners[i].y - 1, color);
            ips200_draw_point(g_left_corners[i].x + 2, g_left_corners[i].y, color);
            ips200_draw_point(g_left_corners[i].x - 2, g_left_corners[i].y, color);
            ips200_draw_point(g_left_corners[i].x + shift, g_left_corners[i].y + 2, color);
            ips200_draw_point(g_left_corners[i].x + shift, g_left_corners[i].y - 2, color);
            ips200_draw_point(g_left_corners[i].x + 1, g_left_corners[i].y + 1, color);
            ips200_draw_point(g_left_corners[i].x + 1, g_left_corners[i].y - 1, color);
            ips200_draw_point(g_left_corners[i].x - 1, g_left_corners[i].y + 1, color);
            ips200_draw_point(g_left_corners[i].x - 1, g_left_corners[i].y - 1, color);
        }

    }
    // 绘制所有记录的右拐点
    for (uint8 i = 0; i < g_right_corner_count; i++)
    {
        uint16 color = RGB565_RED; // 默认颜色为红色
        if (g_right_corners[i].type == CORNER_TYPE_OUTER)
        {
            color = RGB565_BLUE; // 外拐点为蓝色
        }
        else if (g_right_corners[i].type == CORNER_TYPE_INNER)
        {
            color = RGB565_YELLOW; // 内拐点为黄色
        }
        if (g_right_corners[i].x < 186 && g_right_corners[i].y < 118
                && g_right_corners[i].x > 2 && g_right_corners[i].y > 2)
        {
            ips200_draw_point(g_right_corners[i].x , g_right_corners[i].y, color);
            ips200_draw_point(g_right_corners[i].x + 1, g_right_corners[i].y, color);
            ips200_draw_point(g_right_corners[i].x - 1, g_right_corners[i].y, color);
            ips200_draw_point(g_right_corners[i].x, g_right_corners[i].y + 1, color);
            ips200_draw_point(g_right_corners[i].x, g_right_corners[i].y - 1, color);
            ips200_draw_point(g_right_corners[i].x + 2, g_right_corners[i].y, color);
            ips200_draw_point(g_right_corners[i].x - 2, g_right_corners[i].y, color);
            ips200_draw_point(g_right_corners[i].x, g_right_corners[i].y + 2, color);
            ips200_draw_point(g_right_corners[i].x, g_right_corners[i].y - 2, color);
            ips200_draw_point(g_right_corners[i].x + 1, g_right_corners[i].y + 1, color);
            ips200_draw_point(g_right_corners[i].x + 1, g_right_corners[i].y - 1, color);
            ips200_draw_point(g_right_corners[i].x - 1, g_right_corners[i].y + 1, color);
            ips200_draw_point(g_right_corners[i].x - 1, g_right_corners[i].y - 1, color);
        }

    }
}

//void set_para(void)
//{
//    dead_zone = 2;//死区
//    sar_thre = 40;//差比和阈值
//    cross_dis = 0.25f; //过十字防止误判其他元素
//    island_in_dis = 0.68f; //进环岛固定值
//    island_out_dis = 0.50f; //出环硬直
//    //拐点
//    corner_low = 95;//不考虑95行以后的拐点
//    single_bridge_detect_thresh = 30;//单边桥提前判
//    normal_detect_thresh = 65; //不是单边桥，正常判
//    step = 1; //隔几个点判断
//    threshold = 25; //两个diff加起来需要超过多少距离，才判断为拐点
//    single_bridge_dis = 1.5f; //单边桥距离积分
//    //环岛部分
//    case1_shift = 40;
//    case1_vis = 80;
//    case2_shift = 45;
//    case2_vis = 95;
//    case2_straight_part = 0.2f;
//    case2_straight_shift = 35;
//    case3_shift = 35;
//    case3_vis = 75;
//    //停车
//    stop_edge = 20; //停车线检测的横向范围
//    stop_thre = 30; //停车线突变的差比和阈值
//    stop_count = 14; //突变点临界值个数
//    //单边桥拐点距离约束
//    X_MIN_BOUNDARY = 20;
//    X_MAX_BOUNDARY = MT9V03X_W - 20; // 168
//    X_RELATIVE_DIST_MAX = 30; // 同侧拐点最大水平距离
//    Y_RELATIVE_DIST_MAX = 30; // 同侧拐点最大垂直距离
//}
