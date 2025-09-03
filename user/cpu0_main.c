/*********************************************************************************************************************
* TC264 Opensourec Library 即（TC264 开源库）是一个基于官方 SDK 接口的第三方开源库
* Copyright (c) 2022 SEEKFREE 逐飞科技
*
* 本文件是 TC264 开源库的一部分
*
* TC264 开源库 是免费软件
* 您可以根据自由软件基金会发布的 GPL（GNU General Public License，即 GNU通用公共许可证）的条款
* 即 GPL 的第3版（即 GPL3.0）或（您选择的）任何后来的版本，重新发布和/或修改它
*
* 本开源库的发布是希望它能发挥作用，但并未对其作任何的保证
* 甚至没有隐含的适销性或适合特定用途的保证
* 更多细节请参见 GPL
*
* 您应该在收到本开源库的同时收到一份 GPL 的副本
* 如果没有，请参阅<https://www.gnu.org/licenses/>
*
* 额外注明：
* 本开源库使用 GPL3.0 开源许可证协议 以上许可申明为译文版本
* 许可申明英文版在 libraries/doc 文件夹下的 GPL3_permission_statement.txt 文件中
* 许可证副本在 libraries 文件夹下 即该文件夹下的 LICENSE 文件
* 欢迎各位使用并传播本程序 但修改内容时必须保留逐飞科技的版权声明（即本声明）
*
* 文件名称          cpu0_main
* 公司名称          成都逐飞科技有限公司
* 版本信息          查看 libraries/doc 文件夹内 version 文件 版本说明
* 开发环境          ADS v1.9.4
* 适用平台          TC264D
* 店铺链接          https://seekfree.taobao.com/
*
* 修改记录
* 日期              作者                备注
* 2022-09-15       pudding            first version
********************************************************************************************************************/
#include "zf_common_headfile.h"
#include "spi_wifi.h"
#include "small_driver_uart_control.h"
#include "pid.h"
#include "first_order_filter.h"
#include "balance_car_control.h"
#include "camera.h"
#include "steer_control.h"
#include "jump.h"

#pragma section all "cpu0_dsram"
// 将本语句与#pragma section all restore语句之间的全局变量都放在CPU0的RAM中

// *************************** 例程硬件连接说明 ***************************
// 使用逐飞科技 tc264 V2.6主板 按照下述方式进行接线
//      模块引脚    单片机引脚
//      RX          查看 small_driver_uart_control.h 中 SMALL_DRIVER_TX  宏定义 默认 P15_7
//      TX          查看 small_driver_uart_control.h 中 SMALL_DRIVER_RX  宏定义 默认 P15_6
//      GND         GND


// *************************** 例程测试说明 ***************************
// 1.核心板烧录完成本例程 主板电池供电 连接 CYT2BL3 FOC 双驱
// 2.如果初次使用 请先点击双驱上的MODE按键 以矫正零点位置 矫正时 电机会发出音乐
// 3.可以在逐飞助手上位机上看到如下串口信息：
//      left speed:xxxx, right speed:xxxx
// 如果发现现象与说明严重不符 请参照本文件最下方 例程常见问题说明 进行排查

// **************************** 代码区域 ****************************
/*#define SERVO_MOTOR_PWM1             (ATOM0_CH0_P21_2)                           // 定义主板上舵机对应引脚
#define SERVO_MOTOR_PWM2             (ATOM0_CH2_P21_4)
#define SERVO_MOTOR_PWM3             (ATOM0_CH1_P21_3)
#define SERVO_MOTOR_PWM4             (ATOM0_CH1_P21_3)
#define SERVO_MOTOR_FREQ            (200 )                                       // 定义主板上舵机频率  请务必注意范围 50-300

#define SERVO_MOTOR_L_MAX           (0 )                                       // 定义主板上舵机活动范围 角度
#define SERVO_MOTOR_R_MAX           (180)                                       // 定义主板上舵机活动范围 角度
#define MAX_DUTY            (30)                                                // 最大 测试 占空比

#define SERVO_MOTOR_DUTY(x)         ((float)PWM_DUTY_MAX/(1000.0/(float)SERVO_MOTOR_FREQ)*(0.5+(float)(x)/90.0))

#if (SERVO_MOTOR_FREQ<50 || SERVO_MOTOR_FREQ>300)
    #error "SERVO_MOTOR_FREQ ERROE!"
#endif
*/


#define IPS200_TYPE     (IPS200_TYPE_SPI)

float servo_motor_duty = 180;                                                  // 舵机动作角度
float servo_motor_dir = 1;                                                      // 舵机动作状态

int8 duty = 0;
bool dir = true;


int core0_main(void)
{
    clock_init();                   // 获取时钟频率<务必保留>
    debug_init();                   // 初始化默认调试串口
    // 此处编写用户代码 例如外设初始化代码等
    system_delay_ms(100);
    small_driver_uart_init();		// 初始化驱动通讯功能
    system_delay_ms(100);
    steer_control_init();
    gpio_init(LED1, GPO, GPIO_HIGH, GPO_PUSH_PULL);                             // 初始化 LED1 输出 默认高电平 推挽输出模式
    gpio_init(BUZZER_PIN, GPO, GPIO_LOW, GPO_PUSH_PULL);
    gpio_init(P33_11, GPI, GPIO_HIGH, GPO_PUSH_PULL);
    gpio_init(P33_12, GPI, GPIO_HIGH, GPO_PUSH_PULL);
    gpio_init(P20_6, GPI, GPIO_HIGH, GPI_PULL_UP);
    gpio_init(P20_7, GPI, GPIO_HIGH, GPI_PULL_UP);
    gpio_init(P11_2, GPI, GPIO_HIGH, GPI_PULL_UP);
    gpio_init(P11_3, GPI, GPIO_HIGH, GPI_PULL_UP);
    //wireless_device_init();

    ips200_init(IPS200_TYPE);
    ips200_show_string(0, 0, "mt9v03x init.");

    while(1)
    {
        if(mt9v03x_init())
        {
            ips200_show_string(0, 80, "mt9v03x reinit.");
        }
        else
            break;                                                  // 短延时快速闪灯表示异常
    }
    ips200_show_string(0, 16, "init success.");
    system_delay_ms(100);
    while(1)
        {
          if(imu660ra_init())
             printf("\r\n IMU660RA init error.");                                 // IMU660RA 初始化失败
          else
             break;
          gpio_toggle_level(LED1);                                                 // 翻转 LED 引脚输出电平 控制 LED 亮灭 初始化出错这个灯会闪的很慢
        }
        balance_casade_init();
    for(int i=0; i<10 ;i++)
    {
        imu660ra_get_gyro();
        z_delta += imu660ra_gyro_z / 0.050 * 0.005;
        system_delay_ms(50);
    }
    z_delta = z_delta / 10;
    pit_ms_init(CCU60_CH0, 5);
    /*if(!gpio_get_level(P33_11))
    {
        target_speed_normal = 300;
        target_speed_low = 300;
    }*/
    // 此处编写用户代码 例如外设初始化代码等
    cpu_wait_event_ready();         // 等待所有核心初始化完毕
    circle_count = 0.0;
    target_speed = target_speed_normal;
    while (TRUE)
    {

        // 此处编写需要循环执行的代码
        /*pwm_set_duty(SERVO_MOTOR_PWM1, (uint32)SERVO_MOTOR_DUTY(90));
        pwm_set_duty(SERVO_MOTOR_PWM4, (uint32)SERVO_MOTOR_DUTY(90));
        pwm_set_duty(SERVO_MOTOR_PWM3, (uint32)SERVO_MOTOR_DUTY(90));
        pwm_set_duty(SERVO_MOTOR_PWM2, (uint32)SERVO_MOTOR_DUTY(90));*/


            /*if(mt9v03x_finish_flag)
                    {
                        ips200_displayimage03x((const uint8 *)mt9v03x_image, MT9V03X_W, MT9V03X_H);
                        save_image();
                        mt9v03x_finish_flag = 0;
                        get_longest_write_col(WHITE_COL_START_Y,WHITE_COL_END_Y,4);
                        image_boundary_process();
                        find_crossline();

                    }*/

        /*if(mt9v03x_finish_flag)
        {
            ips200_displayimage03x((const uint8 *)mt9v03x_image, MT9V03X_W, MT9V03X_H);                       // 显示原始图像
            //ips200_show_gray_image(0, 0, (const uint8 *)mt9v03x_image, MT9V03X_W, MT9V03X_H, 240, 180, 64);     // 显示二值化图像
            mt9v03x_finish_flag = 0;
        }*/

        //printf("%f, %d,%d\r\n", balance_casade.filtering_angle, right_motor_duty, left_motor_duty);
        //printf("%f,%d\r\n", balance_control.ang_v_control.output, motor_value.receive_right_speed_data);
        //printf("%f,%d,%d,%d\r\n", balance_casade.filtering_angle,balance_control.ang_v_control.output,balance_control.angle_control.output,imu660ra_gyro_y);
        //printf("%d,%d,%d,%d,%d,%d\r\n", imu660ra_gyro_x,imu660ra_gyro_y,imu660ra_gyro_z, imu660ra_acc_x, imu660ra_acc_y, imu660ra_acc_z);
        //printf("%d,%d,%f,%d,%d,%d \r\n", car_speed,-balance_control.speed_control.output,balance_casade.filtering_angle, -balance_control.angle_control.output, *balance_casade.gyro_raw_data,balance_control.ang_v_control.output);
        ips200_displayimage03x((const uint8 *)mt9v03x_image, MT9V03X_W, MT9V03X_H);
        show_line();


        //ips200_show_float(  0 , 190,   balance_casade.filtering_angle,     5, 3);
        //ips200_show_int(0, 210, balance_control.ang_v_control.output, 6);


        //ips200_show_int(80, 210, pid_turn_angle.output, 6);
        //ips200_show_int(120, 210, imu660ra_gyro_z, 6);
        //wireless_device_send();
        ips200_show_float(  200 , 25,   speed_integration, 2, 3);
        ips200_show_int(200, 75, crossline_time , 1 );
        ips200_show_int(200, 100, crossline_state , 1 );
        //几组调试参数
        //0：转角、速度、陀螺仪积分、距离积分
        ips200_show_float(  0 , 150,   turn_angle, 5, 5);
        ips200_show_int( 80, 150, car_speed, 6);
        ips200_show_int(140, 150, target_speed, 3);
        ips200_show_int(200, 150, motor_value.receive_left_speed_data, 6);
        //ips200_show_float(80 , 150,   circle_count,  5, 5);
        //ips200_show_float(  120 , 150,   speed_integration, 5, 5);
        //1：指定行右线与左线的坐标差、单边桥状态、是否瘦线
        //2：左拐点、右拐点、左直线、右直线
        ips200_show_int(20, 170, left_outer, 6);
        ips200_show_int(80, 170, right_outer, 6);
        ips200_show_int(140, 170, crossline_time, 6);
        ips200_show_int(200, 170, is_left_straight, 6);
        //ips200_show_int(200, 170, traverse, 6);
        //3：十字、环岛、环岛状态、单边桥
        ips200_show_int(0, 210, is_barrier, 1);
        ips200_show_float(40, 210, circle_count, 5, 5);
        ips200_show_int(80, 210, barrier_state, 6);
        //ips200_show_int(40, 210, is_crossroad, 6);
        //ips200_show_int(80, 210, island_state, 6);
        //ips200_show_int(120, 210, island_flag, 6);
        //ips200_show_int(160, 210, is_single_bridge, 6);

        // 此处编写需要循环执行的代码
    }
}


#pragma section all restore
// **************************** 代码区域 ****************************
// *************************** 例程常见问题说明 ***************************
// 遇到问题时请按照以下问题检查列表检查
// 问题1：串口没有数据
//      查看逐飞助手上位机打开的是否是正确的串口，检查打开的 COM 口是否对应的是调试下载器或者 USB-TTL 模块的 COM 口
//      如果是使用逐飞科技 英飞凌TriCore 调试下载器连接，那么检查下载器线是否松动，检查核心板串口跳线是否已经焊接，串口跳线查看核心板原理图即可找到
//      如果是使用 USB-TTL 模块连接，那么检查连线是否正常是否松动，模块 TX 是否连接的核心板的 RX，模块 RX 是否连接的核心板的 TX
// 问题2：串口数据乱码
//      查看逐飞助手上位机设置的波特率是否与程序设置一致，程序中 zf_common_debug.h 文件中 DEBUG_UART_BAUDRATE 宏定义为 debug uart 使用的串口波特率
// 问题3：无刷电机无反应
//      检查Rx信号引脚是否接对，信号线是否松动
// 问题4：无刷电机转动但转速显示无速度
//      检查Tx信号引脚是否接对，信号线是否松动
