///*
// * camera.c
// *
// *  Created on: 2024年6月6日
// *      Author: Merlin_yang
// */
//
//#include "zf_device_mt9v03x.h"
//#include "zf_common_debug.h"
//#include "zf_common_font.h"
//#include "zf_driver_gpio.h"
//#include "camera.h"
//#include "math.h"
//#include "stdlib.h"
//#include "spi_wifi.h"
//
//float turn_angle = 0;
//uint8 stop_flag = 2;
//uint8 x_write_col;
////uint8 run_flag;
//uint8 y_write_col;
//uint8 dead_zone = 5;
//uint8 leftline[MT9V03X_H];
//uint8 rightline[MT9V03X_H];
//uint8 midline[MT9V03X_H];
//int16 sar_thre = 50;//差比和阈值
//IFX_ALIGN(4) uint8  original_image[MT9V03X_H][MT9V03X_W];
//pid_struct_t pid_turn_angle;
//int16 shift=0;
//uint8 x_write_col_L;
//uint8 x_write_col_R;
////uint8 search_stop_line = 0;
//uint8 longest_len_L = 0;
////uint8 l_col = 0;
//uint8 longest_len_R = 0;
////uint8 r_col = 0;
//// 全局变量，用于存储检测到的拐点行号，0表示未找到
//int left_down_corner_row = 0;
//int right_down_corner_row = 0;
//int left_up_corner_row = 0;
//int right_up_corner_row = 0;
//int16 flag = 0; //flag = 1,表示识别到十字路口;
//
//// --- Roundabout State Machine ---
//int island_state = 0; // 环岛状态机: 0:关闭, 1:进入, 2:环内, 3:退出
//int right_island_flag = 0; // 1 表示当前识别到的是右环岛
//#define STANDARD_ROAD_WIDTH 80 // 用于半边补线的标准赛道宽度
//
//// --- Lost Line Analysis ---
//uint8 left_lost_flag[MT9V03X_H];
//uint8 right_lost_flag[MT9V03X_H];
//int left_lost_time = 0;
//int right_lost_time = 0;
//int both_lost_time = 0;
//int boundary_start_left = 0;
//int boundary_start_right = 0;
//
//void camera_mt9v03x_init(void)
//{
//    zf_log(!mt9v03x_init(), "mt9v03x init error");
//}
//
//
//void save_image(void)
//{
//    memcpy(original_image[0], mt9v03x_image[0], MT9V03X_IMAGE_SIZE);
//}
//
// void image_boundary_process(void)
//{
//    uint8 row;//行
//    int start_col_L = 0;
//    int start_col_R = 0;//各行起点的列坐标,默认为MT9V03X_W / 2
//    memset(leftline,0,sizeof(leftline));
//    memset(midline,0,sizeof(midline)); // 这里应该是 sizeof(midline)
//    memset(rightline,0,sizeof(rightline));
//    uint8 long_of_midline = 0;
//    float num_of_outpoint = 0;
//    // 使用有效区域底部80作为基准
//    long_of_midline = 90 - y_write_col;
//
//    if (long_of_midline>=15)
//    {
//        if (stop_flag != 2)
//        stop_flag =0;
//        start_col_L = x_write_col_L;
//        start_col_R = x_write_col_R;
//        //get_longest_write_col();
//        // =================================================================
//        // 步骤 1: 基础的边线搜寻
//        // =================================================================
//        // 从有效区域底部80开始搜索
////        for(row = 80; row >= y_write_col; row--)
////        {
////           difsum_left(row,LIMIT_MIN_MAX(start_col_L, dead_zone, MT9V03X_W-1-dead_zone));
////           difsum_right(row,LIMIT_MIN_MAX(start_col_R, dead_zone, MT9V03X_W-1-dead_zone));
////        }
//        // =================================================================
//        // 步骤 3: 基于（可能被修正过的）边线，计算中线和转向误差
//        // =================================================================
//        for(row = 80; row >= y_write_col; row--)
//        {
//           // 【关键】重新计算中线，确保使用的是最新的边线数据
//           // 如果是直道，这里用的就是原始边线；如果是十字，用的就是被修正过的直线
//           if(leftline[row] != 0 && rightline[row] != 0) // 确保左右边线都有效
//           {
//               midline[row] = (leftline[row]+rightline[row])/2;
//           }
//           else midline[row] = midline[row+1];// 如果有丢线，用上一行的中线值来填充
//
//
//           if (leftline[row]<7 || rightline[row] >MT9V03X_W-7)
//           {
//            num_of_outpoint++;
//           }
//
//           if (leftline[row]<7 && rightline[row] >MT9V03X_W-7)
//           {
//            num_of_outpoint--;
//           }
//        }
//
//        turn_angle = 0;
//
//        // 首先判断，我们能看到理想的瞄准点(第27行)吗？
//        if (y_write_col > 27)
//        {
//            // 这个分支意味着：看不远。
//            // 我们的视线在到达第27行之前就中断了（比如y_write_col是40）。
//            // 这是“非理想状况”，我们没得选，只能看我们能看到的最远的那一小段路。
//            // 所以，从 y_write_col 这一行开始，往近处看5行来计算角度。
//            for (row = y_write_col; row < y_write_col+5; row++)
//            {
//                turn_angle += midline[row];
//            }
//            turn_angle = turn_angle/5;
//            turn_angle -= MT9V03X_W/2;
//        }
//        else
//        {
//            // 这个分支是“理想状况”。
//            // 我们的视线至少能达到第27行（比如y_write_col是20）。
//            // 既然能看到理想的瞄准点，那我们就用固定的第27行开始的5行路况来计算。
//            // 这样做能保证每次决策的参考标准统一，让车开得更稳。
//            for (row = 27; row < 27+5; row++)
//            {
//                turn_angle += midline[row];
//            }
//            turn_angle = turn_angle/5;
//            turn_angle -= MT9V03X_W/2;
//        }
//        if(num_of_outpoint >= 25) num_of_outpoint = 20;
//        if(num_of_outpoint <= 0) num_of_outpoint = 0;
//        turn_angle = turn_angle * (1.0f + 0.007*num_of_outpoint);
//        pid_calc(&pid_turn_angle, 0.0f, turn_angle);
//
//    }
//    else
//    {
//        if (stop_flag != 2)stop_flag =1;
//        turn_angle = 0;
//    }
//}
//
//
//
//void difsum_left(uint8 y,uint8 x)
//{
//    float sum,dif,sar;//和，差，比
//    uint8 col;//列
//    uint8 mov = 1;//每次作差后的移动量,默认为2，可以根据画面分辨率调整
//    //计算第x行的左边界
//    leftline[y] = dead_zone;//未找到左边界时输出为0
//    for(col = x; col >= dead_zone; col -= mov)
//        {
//        dif = (float)((original_image[y][col] - original_image[y][col - mov - 1])<<8);//左移8位即乘256，可避免浮点数乘，加快速度
//        sum = (float)((original_image[y][col] + original_image[y][col - mov - 1]));
//        sar = fabs(dif / sum);//求取差比和
//        if(sar > sar_thre)
//            {//差比和大于阈值代表深浅色突变
//            leftline[y] = (int16)(col - mov);
//            break;//找到边界后退出
//            }
//        }
//}
//
//void difsum_right(uint8 y,uint8 x)
//{
//    float sum,dif,sar;//和，差，比
//    uint8 col;//列
//    uint8 mov = 2;//每次作差后的移动量,默认为2，可以根据画面分辨率调整
//    //计算第x行的右边界
//    rightline[y] = MT9V03X_W-1-dead_zone;//未找到左右边界时输出为0
//    for(col = x; col <= MT9V03X_W-mov - 1-dead_zone; col += mov)
//        {
//        dif = (float)((original_image[y][col] - original_image[y][col + mov + 1])<<8);//左移8位即乘256，可避免浮点数乘，加快速度
//        sum = (float)((original_image[y][col] + original_image[y][col + mov + 1]));
//        sar = fabs(dif / sum);//求取差比和
//        if(sar > sar_thre)
//            {//差比和大于阈值代表深浅色突变
//            rightline[y] = (int16)(col + mov);
//            break;//找到边界后退出
//            }
//        }
//
//}
//
//
////-------------------------------------------------------------------------------------------------------------------
//// 函数简介     双最长白线巡线法
//// 修改参数     x_write_col,y_write_col
//// 使用示例     get_dual_longest_white_cols();
////-------------------------------------------------------------------------------------------------------------------
//
//
//void get_longest_write_col(void)
//{
//    uint16 i, j;
//    uint8 white_column[MT9V03X_W] = {0};
//    const uint8 start_column = 5;
//    const uint8 end_column = MT9V03X_W - 5;
//    const uint8 white_threshold = 120;
//    int left_border = 0;
//    int right_border = 0;
//    int l_col = 0;
//    int r_col = 0;
//    int search_stop_line = 0;
//
//    // 根据用户提示，有效y坐标范围为 [20, 80]
//    // 从有效区域底部(y=80)向上搜索至顶部(y=20)
//    for (j = start_column; j <= end_column; j++)
//    {
//        for (i = 100; i >= 0; i--)
//        {
//            if (original_image[j][i] < white_threshold)
//            {
//                break;
//            }
//            white_column[j]++;
//        }
//    }
//
//    for (j = start_column; j <= end_column; j++)
//    {
//        if (longest_len_L < white_column[j])
//        {
//            longest_len_L = white_column[j];
//            l_col = j;
//        }
//    }
//
//    for (j = end_column; j >= start_column; j--)
//    {
//        if (longest_len_R < white_column[j])
//        {
//            longest_len_R = white_column[j];
//            r_col = j;
//        }
//    }
//
//    x_write_col_L = l_col;
//    x_write_col_R = r_col;
//
//    uint8 max_len = (longest_len_L > longest_len_R) ? longest_len_L : longest_len_R;
//
//    search_stop_line = l_col;
//    for (int j = 90 ; j >= search_stop_line; j--) //常规巡线
//    {
//        for (int i = r_col; i <= MT9V03X_W - 1; i++)
//        {
//            if (original_image[i][j]> white_threshold && original_image[i+1][j] <= white_threshold && original_image[i+2][j] <= white_threshold)
//            {
//                right_border = i;
//                right_lost_flag[j] = 0;
//                break;
//            }
//            else
//            {
//                right_border = i;
//                right_lost_flag[j] = 1;
//                break;
//            }
//        }
//        for (int i = l_col; i >= 2; i--)
//        {
//           if (original_image[i][j]> white_threshold && original_image[i-1][j]<= white_threshold && original_image[i-2][j] <= white_threshold)
//           {
//               left_border = i;
//               left_lost_flag[j] = 0;
//               break;
//           }
//           else
//           {
//               left_border = i;
//               left_lost_flag[j] = 1;
//               break;
//           }
//        }
//        leftline[j] = left_border;
//        rightline[j] = right_border;
//    }
//
//    if (max_len > 5) // 减少阈值，因为搜索范围变小
//    {
//        // y_write_col 是最长白列的顶端行号
//        y_write_col = 80 - (max_len > 0 ? max_len - 1 : 0);
//    }
//    else
//    {
//        // 如果没找到，则从有效区域底部开始搜索
//        y_write_col = 80;
//    }
//
//    x_write_col = (l_col + r_col) / 2;
//}
//
//
//
//
//void show_line(void)
//{
//    for(int8 i = MT9V03X_H-1; i > y_write_col; i --)
//    {
//        ips200_draw_point((uint16)leftline[i]+shift, i, RGB565_RED);//红色左线
//        ips200_draw_point((uint16)rightline[i]+shift, i, RGB565_BLUE);//蓝色右线
//        ips200_draw_point(LIMIT_MIN_MAX((int16)midline[i]+shift,shift,shift+MT9V03X_W-1), i, RGB565_PURPLE);//紫色中线
//        // ips200_showint16(0,0, x_write_col_L);//【0】是左白列长度
//        // ips200_showint16(0,1, x_write_col_R);//【1】是右白列长度
//        // ips200_showint16(0,2, x_write_col);//【2】是中线
//        // ips200_showint16(0,3, y_write_col);//【3】是行号
//        // //对最长白线的终点可视化
//        // ips200_draw_point(x_write_col_L, y_write_col, RGB565_ORANGE);
//        // ips200_draw_point(x_write_col_R, y_write_col, RGB565_ORANGE);
//    }
//
//    // 显示四个拐点（如果找到）
//    // 左下拐点
//    // if(left_down_corner_row > 0 && left_down_corner_row < MT9V03X_H)
//    // {
//    //     ips200_draw_point((uint16)leftline[left_down_corner_row]+shift, left_down_corner_row, RGB565_YELLOW);
//    // }
//    // // 右下拐点
//    // if(right_down_corner_row > 0 && right_down_corner_row < MT9V03X_H)
//    // {
//    //     ips200_draw_point((uint16)rightline[right_down_corner_row]+shift, right_down_corner_row, RGB565_CYAN);
//    // }
//    // // 左上拐点
//    // if(left_up_corner_row > 0 && left_up_corner_row < MT9V03X_H)
//    // {
//    //     ips200_draw_point((uint16)leftline[left_up_corner_row]+shift, left_up_corner_row, RGB565_YELLOW);
//    // }
//    // // 右上拐点
//    // if(right_up_corner_row > 0 && right_up_corner_row < MT9V03X_H)
//    // {
//    //     ips200_draw_point((uint16)rightline[right_up_corner_row]+shift, right_up_corner_row, RGB565_CYAN);
//    // }
//    // // 显示flag
//    // ips200_showint16(0, 4, flag); // 【4】显示flag
//}
//
////====================================================================================================================
//// 元素识别基础
//// 左下、右下、左上、右上拐点识别
////
////====================================================================================================================
//
///**
// * @brief   寻找左下角点 (已重构)
// * @param   start_row   搜索起始行
// * @param   end_row     搜索结束行
// * @return  void        结果存储在全局变量 left_down_corner_row
// * @note    根据多点斜率判断，寻找下方平直、上方急转的角点。
// */
//static void find_left_down_corner(int start_row, int end_row)
//{
//    left_down_corner_row = 0; // 每次搜索前重置
//    // 调整搜索范围，确保安全访问数组
//    if (start_row >= 80 - 4) start_row = 80 - 5;
//    if (end_row < 20) end_row = 20;
//
//    for (int i = start_row; i >= end_row; i--)
//    {
//        // 检查点周围的边线数据是否有效 (检查 i-4 到 i+3 的范围)
//        if (leftline[i] == 0 || leftline[i+1] == 0 || leftline[i+2] == 0 || leftline[i+3] == 0 ||
//            leftline[i-1] == 0 || leftline[i-2] == 0 || leftline[i-3] == 0 || leftline[i-4] == 0)
//        {
//            continue;
//        }
//
//        // 角点特征: 下方平直，上方急转 (向左上)
//        if ( (abs(leftline[i] - leftline[i+1]) <= 5) &&
//             (abs(leftline[i+1] - leftline[i+2]) <= 5) &&
//             (abs(leftline[i+2] - leftline[i+3]) <= 5) &&
//             ((leftline[i] - leftline[i-2]) >= 8) &&
//             ((leftline[i] - leftline[i-3]) >= 15) &&
//             ((leftline[i] - leftline[i-4]) >= 15) )
//        {
//            left_down_corner_row = i; // 找到第一个符合条件的点，更新全局变量
//            return;                   // 立刻退出函数
//        }
//    }
//}
//
///**
// * @brief   寻找右下角点 (已重构)
// * @return  void        结果存储在全局变量 right_down_corner_row
// * @note    根据多点斜率判断，寻找下方平直、上方急转的角点。
// */
//static void find_right_down_corner(int start_row, int end_row)
//{
//    right_down_corner_row = 0; // 每次搜索前重置
//    // 调整搜索范围，确保安全访问数组
//    if (start_row >= 80 - 4) start_row = 80 - 5;
//    if (end_row < 20) end_row = 20;
//
//    for (int i = start_row; i >= end_row; i--)
//    {
//        // 检查点周围的边线数据是否有效
//        if (rightline[i] == MT9V03X_W-1 || rightline[i+1] == MT9V03X_W-1 || rightline[i+2] == MT9V03X_W-1 || rightline[i+3] == MT9V03X_W-1 ||
//            rightline[i-1] == MT9V03X_W-1 || rightline[i-2] == MT9V03X_W-1 || rightline[i-3] == MT9V03X_W-1 || rightline[i-4] == MT9V03X_W-1)
//        {
//            continue;
//        }
//
//        // 角点特征: 下方平直，上方急转 (向右上)
//        if ( (abs(rightline[i] - rightline[i+1]) <= 5) &&
//             (abs(rightline[i+1] - rightline[i+2]) <= 5) &&
//             (abs(rightline[i+2] - rightline[i+3]) <= 5) &&
//             ((rightline[i] - rightline[i-2]) <= -8) &&
//             ((rightline[i] - rightline[i-3]) <= -15) &&
//             ((rightline[i] - rightline[i-4]) <= -15) )
//        {
//            right_down_corner_row = i;
//            return;
//        }
//    }
//}
//
///**
// * @brief   寻找左上角点 (已重构)
// * @return  void        结果存储在全局变量 left_up_corner_row
// * @note    根据多点斜率判断，寻找上方平直、下方急转的角点。
// */
//static void find_left_up_corner(int start_row, int end_row)
//{
//    left_up_corner_row = 0; // 每次搜索前重置
//    // 调整搜索范围，确保安全访问数组
//    if (start_row >= 80 - 5) start_row = 80 - 6;
//    if (end_row < 20) end_row = 20;
//
//    for (int i = start_row; i >= end_row; i--)
//    {
//        // 检查点周围的边线数据是否有效 (i-3 to i+4)
//        if (leftline[i] == 0 || leftline[i+1] == 0 || leftline[i+2] == 0 || leftline[i+3] == 0 || leftline[i+4] == 0 ||
//            leftline[i-1] == 0 || leftline[i-2] == 0 || leftline[i-3] == 0)
//        {
//            continue;
//        }
//
//        // 角点特征: 上方平直, 下方急转 (向左下)
//        if ( (abs(leftline[i] - leftline[i-1]) <= 5) &&
//             (abs(leftline[i-1] - leftline[i-2]) <= 5) &&
//             (abs(leftline[i-2] - leftline[i-3]) <= 5) &&
//             ((leftline[i] - leftline[i+2]) >= 8) &&
//             ((leftline[i] - leftline[i+3]) >= 15) &&
//             ((leftline[i] - leftline[i+4]) >= 15) )
//        {
//            left_up_corner_row = i;
//            return;
//        }
//    }
//}
//
///**
// * @brief   寻找右上角点 (已重构)
// * @return  void        结果存储在全局变量 right_up_corner_row
// * @note    根据多点斜率判断，寻找上方平直、下方急转的角点。
// */
//static void find_right_up_corner(int start_row, int end_row)
//{
//    right_up_corner_row = 0; // 每次搜索前重置
//    // 调整搜索范围，确保安全访问数组
//    if (start_row >= 80 - 5) start_row = 80 - 6;
//    if (end_row < 20) end_row = 20;
//
//    for (int i = start_row; i >= end_row; i--)
//    {
//        // 检查点周围的边线数据是否有效
//        if (rightline[i] == MT9V03X_W-1 || rightline[i+1] == MT9V03X_W-1 || rightline[i+2] == MT9V03X_W-1 || rightline[i+3] == MT9V03X_W-1 || rightline[i+4] == MT9V03X_W-1 ||
//            rightline[i-1] == MT9V03X_W-1 || rightline[i-2] == MT9V03X_W-1 || rightline[i-3] == MT9V03X_W-1)
//        {
//            continue;
//        }
//
//        // 角点特征: 上方平直, 下方急转 (向右下)
//        if ( (abs(rightline[i] - rightline[i-1]) <= 5) &&
//             (abs(rightline[i-1] - rightline[i-2]) <= 5) &&
//             (abs(rightline[i-2] - rightline[i-3]) <= 5) &&
//             ((rightline[i] - rightline[i+2]) <= -8) &&
//             ((rightline[i] - rightline[i+3]) <= -15) &&
//             ((rightline[i] - rightline[i+4]) <= -15) )
//        {
//            right_up_corner_row = i;
//            return;
//        }
//    }
//}
//
//
///**
// * @brief   通用补线函数（两点法绘制直线）
// * @param   line_array  要操作的边线数组 (leftline 或 rightline)
// * @param   y1, x1      点1坐标 (行, 列)
// * @param   y2, x2      点2坐标 (行, 列)
// */
//static void patch_line(uint8* line_array, int y1, int x1, int y2, int x2)
//{
//    //y1= 100; y2 = 50; x1 = 30; x2 = 30;
//    if (y1 > y2) // 保证 y1 是较小的行号，方便从上往下补线
//    {
//        int temp_y = y1; y1 = y2; y2 = temp_y;
//        int temp_x = x1; x1 = x2; x2 = temp_x;
//    }
//
//    if (y1 == y2) return; // 避免除以0
//
//    for (int i = y1; i <= y2; i++)
//    {
//        if (i < 0 || i >= MT9V03X_H) continue; // 边界检查
//        // 线性插值公式，用整数运算避免浮点数
//        int hx = x1 + ((x2 - x1) * (i - y1)) / (y2 - y1);
//        if (hx >= MT9V03X_W) hx = MT9V03X_W - 1;
//        if (hx < 0) hx = 0;
//        line_array[i] = (uint8)hx;
//    }
//}
//
//
///**
// * @brief   斜率补线（外插法）
// * @param   line_array      要操作的边线数组
// * @param   y1, x1          点1坐标
// * @param   y2, x2          点2坐标 (y1,y2,x1,x2用于确定斜率)
// * @param   patch_from_row  开始补线的行
// * @param   patch_to_row    结束补线的行
// */
//static void extrapolate_line(uint8* line_array, int y1, int x1, int y2, int x2, int patch_from_row, int patch_to_row)
//{
//    if (y1 == y2) return; // 避免除以0
//
//    if (y1 > y2) // 保证 y1 < y2
//    {
//        int temp_y = y1; y1 = y2; y2 = temp_y;
//        int temp_x = x1; x1 = x2; x2 = temp_x;
//    }
//
//    int start_row = (patch_from_row < patch_to_row) ? patch_from_row : patch_to_row;
//    int end_row = (patch_from_row > patch_to_row) ? patch_from_row : patch_to_row;
//
//    for (int i = start_row; i <= end_row; i++)
//    {
//        if (i < 0 || i >= MT9V03X_H) continue; // 边界检查
//        // 线性外插公式
//        int hx = x1 + ((long)(x2 - x1) * (i - y1)) / (y2 - y1);
//        if (hx >= MT9V03X_W) hx = MT9V03X_W - 1;
//        if (hx < 0) hx = 0;
//        line_array[i] = (uint8)hx;
//    }
//}
//
///**
// * @brief   十字路口识别与处理主函数 (硬编码版)
// * @param   void
// * @return  int   返回识别到的元素类型 (1 为十字路口, 0 为直道)
// * @note    该函数在搜线后、计算中线前调用。
// */
//int crossroad_recognition_and_processing(void)
//{
//    // 步骤 0: 计算双边丢线行数，作为十字识别的先决条件
//    uint8 both_lost_count = 0;
//    for(int i = y_write_col; i < 80; i++)
//    {
//        if(leftline[i] <= dead_zone && rightline[i] >= (MT9V03X_W - 1 - dead_zone))
//        {
//            both_lost_count++;
//        }
//    }
//
//    // 步骤 1: 如果双边丢线不严重，则不可能是十字，直接退出
//    if(both_lost_count < 10)
//    {
//        return 0; // 不是十字
//    }
//
//    // 步骤 2: 寻找上拐点
//    find_left_up_corner(80, 20);
//    find_right_up_corner(80, 20);
//
//    // 步骤 3: 如果没找到两个上拐点，肯定不是十字，直接退出
//    if(left_up_corner_row == 0 || right_up_corner_row == 0)
//    {
//        return 0;
//    }
//
//    // 步骤 4: 找到了两个上拐点，以此为基础寻找下拐点
//    // 用两个上拐点中更靠下的点（行号大）作为下拐点搜索范围的上限
//    int down_search_start_row = (left_up_corner_row > right_up_corner_row) ? left_up_corner_row : right_up_corner_row;
//    find_left_down_corner(80 - 5, down_search_start_row + 2);
//    find_right_down_corner(80 - 5, down_search_start_row + 2);
//
//    // 步骤 5: 合理性检查，下拐点不能在上拐点的上方
//    if(left_down_corner_row != 0 && left_down_corner_row <= left_up_corner_row)
//    {
//        left_down_corner_row = 0; // 视为无效
//    }
//    if(right_down_corner_row != 0 && right_down_corner_row <= right_up_corner_row)
//    {
//        right_down_corner_row = 0; // 视为无效
//    }
//
//    // 步骤 6: 根据找到的拐点数量，执行不同的补线策略
//    if(left_down_corner_row > 0 && right_down_corner_row > 0)
//    {
//        // 四个点都在：直接连接上下对应的拐点
//        patch_line(leftline, left_up_corner_row, leftline[left_up_corner_row], left_down_corner_row, leftline[left_down_corner_row]);
//        patch_line(rightline, right_up_corner_row, rightline[right_up_corner_row], right_down_corner_row, rightline[right_down_corner_row]);
//    }
//    else if(left_down_corner_row == 0 && right_down_corner_row > 0)
//    {
//        // 缺左下点：右边连线，左边根据斜率延长
//        patch_line(rightline, right_up_corner_row, rightline[right_up_corner_row], right_down_corner_row, rightline[right_down_corner_row]);
//        extrapolate_line(leftline,
//                         left_up_corner_row - 2, leftline[left_up_corner_row - 2],
//                         left_up_corner_row - 1, leftline[left_up_corner_row - 1],
//                         left_up_corner_row, 80);
//    }
//    else if(left_down_corner_row > 0 && right_down_corner_row == 0)
//    {
//        // 缺右下点：左边连线，右边根据斜率延长
//        patch_line(leftline, left_up_corner_row, leftline[left_up_corner_row], left_down_corner_row, leftline[left_down_corner_row]);
//        extrapolate_line(rightline,
//                         right_up_corner_row - 2, rightline[right_up_corner_row - 2],
//                         right_up_corner_row - 1, rightline[right_up_corner_row - 1],
//                         right_up_corner_row, 80);
//    }
//    else // 只找到了两个上拐点
//    {
//        // 左右两边都根据斜率延长
//        extrapolate_line(leftline,
//                         left_up_corner_row - 2, leftline[left_up_corner_row - 2],
//                         left_up_corner_row - 1, leftline[left_up_corner_row - 1],
//                         left_up_corner_row, 80);
//        extrapolate_line(rightline,
//                         right_up_corner_row - 2, rightline[right_up_corner_row - 2],
//                         right_up_corner_row - 1, rightline[right_up_corner_row - 1],
//                         right_up_corner_row, 80);
//    }
//
//    flag = 1; // 调试用，标记已识别到十字
//    return 1; // 返回 1, 代表识别到十字路口并已处理
//}
//
