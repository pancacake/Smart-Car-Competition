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
* 文件名称          isr
* 公司名称          成都逐飞科技有限公司
* 版本信息          查看 libraries/doc 文件夹内 version 文件 版本说明
* 开发环境          ADS v1.9.20
* 适用平台          TC264D
* 店铺链接          https://seekfree.taobao.com/
*
* 修改记录
* 日期              作者                备注
* 2022-09-15       pudding            first version
********************************************************************************************************************/

#include <camera_new.h>
#include "camera.h"
#include "isr_config.h"
#include "isr.h"
#include "spi_wifi.h"
#include "small_driver_uart_control.h"
#include "pid.h"
#include "first_order_filter.h"
#include "balance_car_control.h"
#include "steer_control.h"
#include "leg_control.h"
#include "jump.h"

// 对于TC系列默认是不支持中断嵌套的，希望支持中断嵌套需要在中断内使用 interrupt_global_enable(0); 来开启中断嵌套
// 简单点说实际上进入中断后TC系列的硬件自动调用了 interrupt_global_disable(); 来拒绝响应任何的中断，因此需要我们自己手动调用 interrupt_global_enable(0); 来开启中断的响应。

// **************************** PIT中断函数 ****************************
IFX_INTERRUPT(cc60_pit_ch0_isr, 0, CCU6_0_CH0_ISR_PRIORITY)
{
    interrupt_global_enable(0);                     // 开启中断嵌套
    pit_clear_flag(CCU60_CH0);

    imu660ra_get_acc();                                                         // 获取 IMU660RA 的加速度测量数值
    imu660ra_get_gyro();                                                        // 获取 IMU660RA 的角速度测量数值

    imu660ra_gyro_x+=1;
    imu660ra_gyro_y+=9;
    imu660ra_gyro_z+=1;

    if(func_abs(imu660ra_gyro_x)<=5)
        imu660ra_gyro_x=0;
    if(func_abs(imu660ra_gyro_y)<=5)
        imu660ra_gyro_y=0;
    if(func_abs(imu660ra_gyro_z)<=5)
        imu660ra_gyro_z=0;

    system_count+=5;
    //if(target_speed < 300)
      //  target_speed+=20;
    if(!gpio_get_level(P20_6))
        target_speed+=5;
    if(!gpio_get_level(P20_7))
        target_speed-=5;

    if (system_count % 20 == 0)
    {
        if( is_single_bridge ==1)// || abs( motor_value.receive_right_speed_data + motor_value.receive_left_speed_data ) > 20)
            car_speed = MIN(motor_value.receive_right_speed_data,  -motor_value.receive_left_speed_data);
        /*else if (abs( motor_value.receive_right_speed_data + motor_value.receive_left_speed_data ) > 20)
            car_speed = (int16)((MIN(motor_value.receive_right_speed_data,  -motor_value.receive_left_speed_data)
                        + ( imu660ra_gyro_z  / 16.384 * 0.02 / 180 * 3.14159265 ) * 0.1 * 60 / 0.020 / 0.21 ));*/
        else
            car_speed = (-motor_value.receive_left_speed_data + motor_value.receive_right_speed_data) /2;

        pid_calc(&balance_control.speed_control, car_speed, target_speed * run_flag);
    }

    if (system_count % 10 == 0)
    {
        first_order_filter_refresh(&balance_casade, *balance_casade.gyro_raw_data, *balance_casade.acc_raw_data);
        pid_calc(&balance_control.angle_control, 0 , balance_casade.filtering_angle);
        //pid_calc(&balance_control.angle_control, -balance_control.speed_control.output , balance_casade.filtering_angle);
        first_order_filter_refresh(&steer_balance_casade, *steer_balance_casade.gyro_raw_data, *steer_balance_casade.acc_raw_data);
        //pid_calc(&steer_balance_control.angle_control, 0 , steer_balance_casade.filtering_angle);
        if (stand_up == 1)
        {
            pid_calc(&steer_balance_control.angle_control, 0 , steer_balance_casade.filtering_angle);
            //target_speed = target_speed_low;
            speed_integration += car_speed / 60 * 0.010 * 0.21;//0.21 转每秒
            /*
            steer_duty_set(&steer_1, STEER_1_CENTER); //+
            steer_duty_set(&steer_2, STEER_2_CENTER); //-
            steer_duty_set(&steer_3, STEER_3_CENTER); //-
            steer_duty_set(&steer_4, STEER_4_CENTER); //+
            */
        }
        if (speed_integration >= single_brigde_distance && stand_up == 1)
        {
            gpio_set_level(BUZZER_PIN,GPIO_LOW);
            steer_duty_set(&steer_1, STEER_1_CENTER);
            steer_duty_set(&steer_2, STEER_2_CENTER);
            steer_duty_set(&steer_3, STEER_3_CENTER);
            steer_duty_set(&steer_4, STEER_4_CENTER);
            //steer_control_init();
            stand_up = 0;
            speed_integration = 0;
            steer_balance_control.angle_control.output = 0;
            target_speed = target_speed_low;
        }

        /*else if( abs(origin_roll_gyro - steer_balance_casade.filtering_angle) <= 50 && abs(turn_angle) <5)
            steer_balance_control.angle_control.output = 0;*/
    }

    if (system_count < 50)
        origin_roll_gyro += steer_balance_casade.filtering_angle;
    else if (system_count == 50 )
        origin_roll_gyro *= 0.1;

    //pid_calc(&balance_control.ang_v_control, 0 , *balance_casade.gyro_raw_data);
    pid_calc(&balance_control.ang_v_control, -balance_control.angle_control.output , *balance_casade.gyro_raw_data);
    pid_calc(&pid_turn_angle, 0.0f ,turn_angle*car_speed);

    //if(run_flag == 1)
    //{
        dynamic_steer_control();
    //}

    if(island_flag == 1 || island_flag == 2 || is_crossroad == 1 || is_barrier == 1)
    {
       circle_count += (imu660ra_gyro_z ) / 16.384 * 0.005; //16.384
       speed_integration += car_speed / 60 * 0.005 * 0.21;//0.21 转每秒
    }
    else
        circle_count = 0.0;
    if(crossline_time == 2)
        speed_integration_stop += car_speed / 60 * 0.005 * 0.21;

    if(mt9v03x_finish_flag)
    {
       //ips200_displayimage03x((const uint8 *)mt9v03x_image, MT9V03X_W, MT9V03X_H);
       save_image();
       mt9v03x_finish_flag = 0;
       get_longest_write_col(WHITE_COL_START_Y,WHITE_COL_END_Y,2);
       //get_longest_write_col();
       image_boundary_process();
       find_crossline();
     }
    if(!gpio_get_level(P33_11))
    {
     if (is_barrier == 1)
     {
         jump_flag = 1;
         gpio_toggle_level(P20_9);
                 /*if(inter_flag == 0)
                 {
                     speed_integration = 0;
                     inter_flag = 1;
                 }
         //balance_control.angle_control.kp = balance_control.angle_control.kp * 0.5f;
         //balance_control.speed_control.kp = balance_control.angle_control.kp * 0.5f;

         //balance_control.angle_control.kp = balance_control.angle_control.kp * 2;
         //balance_control.speed_control.kp = balance_control.angle_control.kp * 2;
                 if(speed_integration >= (0.25 - (car_speed * 0.21 / 60) * 0.25) )
                              {*/
                                  dynamic_jump_control();
                           //       inter_flag = 0;
                            //  }
                                  is_barrier =0;

     }

     //dynamic_jump_control();
    }
        motor_speed_control();

}


IFX_INTERRUPT(cc60_pit_ch1_isr, 0, CCU6_0_CH1_ISR_PRIORITY)
{
    interrupt_global_enable(0);                     // 开启中断嵌套
    pit_clear_flag(CCU60_CH1);




}

IFX_INTERRUPT(cc61_pit_ch0_isr, 0, CCU6_1_CH0_ISR_PRIORITY)
{
    interrupt_global_enable(0);                     // 开启中断嵌套
    pit_clear_flag(CCU61_CH0);




}

IFX_INTERRUPT(cc61_pit_ch1_isr, 0, CCU6_1_CH1_ISR_PRIORITY)
{
    interrupt_global_enable(0);                     // 开启中断嵌套
    pit_clear_flag(CCU61_CH1);




}
// **************************** PIT中断函数 ****************************


// **************************** 外部中断函数 ****************************
IFX_INTERRUPT(exti_ch0_ch4_isr, 0, EXTI_CH0_CH4_INT_PRIO)
{
    interrupt_global_enable(0);                     // 开启中断嵌套
    if(exti_flag_get(ERU_CH0_REQ0_P15_4))           // 通道0中断
    {
        exti_flag_clear(ERU_CH0_REQ0_P15_4);

    }

    if(exti_flag_get(ERU_CH4_REQ13_P15_5))          // 通道4中断
    {
        exti_flag_clear(ERU_CH4_REQ13_P15_5);
    }
}

IFX_INTERRUPT(exti_ch1_ch5_isr, 0, EXTI_CH1_CH5_INT_PRIO)
{
    interrupt_global_enable(0);                     // 开启中断嵌套

    if(exti_flag_get(ERU_CH1_REQ10_P14_3))          // 通道1中断
    {
        exti_flag_clear(ERU_CH1_REQ10_P14_3);

        tof_module_exti_handler();                  // ToF 模块 INT 更新中断

    }

    if(exti_flag_get(ERU_CH5_REQ1_P15_8))           // 通道5中断
    {
        exti_flag_clear(ERU_CH5_REQ1_P15_8);


    }
}

// 由于摄像头pclk引脚默认占用了 2通道，用于触发DMA，因此这里不再定义中断函数
// IFX_INTERRUPT(exti_ch2_ch6_isr, 0, EXTI_CH2_CH6_INT_PRIO)
// {
//  interrupt_global_enable(0);                     // 开启中断嵌套
//  if(exti_flag_get(ERU_CH2_REQ7_P00_4))           // 通道2中断
//  {
//      exti_flag_clear(ERU_CH2_REQ7_P00_4);
//  }
//  if(exti_flag_get(ERU_CH6_REQ9_P20_0))           // 通道6中断
//  {
//      exti_flag_clear(ERU_CH6_REQ9_P20_0);
//  }
// }
IFX_INTERRUPT(exti_ch3_ch7_isr, 0, EXTI_CH3_CH7_INT_PRIO)
{
    interrupt_global_enable(0);                     // 开启中断嵌套
    if(exti_flag_get(ERU_CH3_REQ6_P02_0))           // 通道3中断
    {
        exti_flag_clear(ERU_CH3_REQ6_P02_0);
        camera_vsync_handler();                     // 摄像头触发采集统一回调函数
    }
    if(exti_flag_get(ERU_CH7_REQ16_P15_1))          // 通道7中断
    {
        exti_flag_clear(ERU_CH7_REQ16_P15_1);




    }
}
// **************************** 外部中断函数 ****************************


// **************************** DMA中断函数 ****************************
IFX_INTERRUPT(dma_ch5_isr, 0, DMA_INT_PRIO)
{
    interrupt_global_enable(0);                     // 开启中断嵌套
    camera_dma_handler();                           // 摄像头采集完成统一回调函数
}
// **************************** DMA中断函数 ****************************


// **************************** 串口中断函数 ****************************
// 串口0默认作为调试串口
IFX_INTERRUPT(uart0_tx_isr, 0, UART0_TX_INT_PRIO)
{
    interrupt_global_enable(0);                     // 开启中断嵌套



}
IFX_INTERRUPT(uart0_rx_isr, 0, UART0_RX_INT_PRIO)
{
    interrupt_global_enable(0);                     // 开启中断嵌套

#if DEBUG_UART_USE_INTERRUPT                        // 如果开启 debug 串口中断
        debug_interrupr_handler();                  // 调用 debug 串口接收处理函数 数据会被 debug 环形缓冲区读取
#endif                                              // 如果修改了 DEBUG_UART_INDEX 那这段代码需要放到对应的串口中断去
}


// 串口1默认连接到摄像头配置串口
IFX_INTERRUPT(uart1_tx_isr, 0, UART1_TX_INT_PRIO)
{
    interrupt_global_enable(0);                     // 开启中断嵌套




}
IFX_INTERRUPT(uart1_rx_isr, 0, UART1_RX_INT_PRIO)
{
    interrupt_global_enable(0);                     // 开启中断嵌套
    camera_uart_handler();                          // 摄像头参数配置统一回调函数
}

// 串口2默认连接到无线转串口模块
IFX_INTERRUPT(uart2_tx_isr, 0, UART2_TX_INT_PRIO)
{
    interrupt_global_enable(0);                     // 开启中断嵌套



}

IFX_INTERRUPT(uart2_rx_isr, 0, UART2_RX_INT_PRIO)
{
    interrupt_global_enable(0);                     // 开启中断嵌套
    wireless_module_uart_handler();                 // 无线模块统一回调函数



}
// 串口3默认连接到GPS定位模块
IFX_INTERRUPT(uart3_tx_isr, 0, UART3_TX_INT_PRIO)
{
    interrupt_global_enable(0);                     // 开启中断嵌套



}

IFX_INTERRUPT(uart3_rx_isr, 0, UART3_RX_INT_PRIO)
{
    interrupt_global_enable(0);                     // 开启中断嵌套

//    gnss_uart_callback();                           // GNSS串口回调函数
    uart_control_callback();


}

// 串口通讯错误中断
IFX_INTERRUPT(uart0_er_isr, 0, UART0_ER_INT_PRIO)
{
    interrupt_global_enable(0);                     // 开启中断嵌套
    IfxAsclin_Asc_isrError(&uart0_handle);
}
IFX_INTERRUPT(uart1_er_isr, 0, UART1_ER_INT_PRIO)
{
    interrupt_global_enable(0);                     // 开启中断嵌套
    IfxAsclin_Asc_isrError(&uart1_handle);
}
IFX_INTERRUPT(uart2_er_isr, 0, UART2_ER_INT_PRIO)
{
    interrupt_global_enable(0);                     // 开启中断嵌套
    IfxAsclin_Asc_isrError(&uart2_handle);
}
IFX_INTERRUPT(uart3_er_isr, 0, UART3_ER_INT_PRIO)
{
    interrupt_global_enable(0);                     // 开启中断嵌套
    IfxAsclin_Asc_isrError(&uart3_handle);
}
