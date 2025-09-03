/*********************************************************************************************************************
* TC264 Opensourec Library ����TC264 ��Դ�⣩��һ�����ڹٷ� SDK �ӿڵĵ�������Դ��
* Copyright (c) 2022 SEEKFREE ��ɿƼ�
*
* ���ļ��� TC264 ��Դ���һ����
*
* TC264 ��Դ�� ��������
* �����Ը��������������ᷢ���� GPL��GNU General Public License���� GNUͨ�ù������֤��������
* �� GPL �ĵ�3�棨�� GPL3.0������ѡ��ģ��κκ����İ汾�����·�����/���޸���
*
* ����Դ��ķ�����ϣ�����ܷ������ã�����δ�������κεı�֤
* ����û�������������Ի��ʺ��ض���;�ı�֤
* ����ϸ����μ� GPL
*
* ��Ӧ�����յ�����Դ���ͬʱ�յ�һ�� GPL �ĸ���
* ���û�У������<https://www.gnu.org/licenses/>
*
* ����ע����
* ����Դ��ʹ�� GPL3.0 ��Դ���֤Э�� �����������Ϊ���İ汾
* �������Ӣ�İ��� libraries/doc �ļ����µ� GPL3_permission_statement.txt �ļ���
* ���֤������ libraries �ļ����� �����ļ����µ� LICENSE �ļ�
* ��ӭ��λʹ�ò����������� ���޸�����ʱ���뱣����ɿƼ��İ�Ȩ����������������
*
* �ļ�����          isr
* ��˾����          �ɶ���ɿƼ����޹�˾
* �汾��Ϣ          �鿴 libraries/doc �ļ����� version �ļ� �汾˵��
* ��������          ADS v1.9.20
* ����ƽ̨          TC264D
* ��������          https://seekfree.taobao.com/
*
* �޸ļ�¼
* ����              ����                ��ע
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

// ����TCϵ��Ĭ���ǲ�֧���ж�Ƕ�׵ģ�ϣ��֧���ж�Ƕ����Ҫ���ж���ʹ�� interrupt_global_enable(0); �������ж�Ƕ��
// �򵥵�˵ʵ���Ͻ����жϺ�TCϵ�е�Ӳ���Զ������� interrupt_global_disable(); ���ܾ���Ӧ�κε��жϣ������Ҫ�����Լ��ֶ����� interrupt_global_enable(0); �������жϵ���Ӧ��

// **************************** PIT�жϺ��� ****************************
IFX_INTERRUPT(cc60_pit_ch0_isr, 0, CCU6_0_CH0_ISR_PRIORITY)
{
    interrupt_global_enable(0);                     // �����ж�Ƕ��
    pit_clear_flag(CCU60_CH0);

    imu660ra_get_acc();                                                         // ��ȡ IMU660RA �ļ��ٶȲ�����ֵ
    imu660ra_get_gyro();                                                        // ��ȡ IMU660RA �Ľ��ٶȲ�����ֵ

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
            speed_integration += car_speed / 60 * 0.010 * 0.21;//0.21 תÿ��
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
       speed_integration += car_speed / 60 * 0.005 * 0.21;//0.21 תÿ��
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
    interrupt_global_enable(0);                     // �����ж�Ƕ��
    pit_clear_flag(CCU60_CH1);




}

IFX_INTERRUPT(cc61_pit_ch0_isr, 0, CCU6_1_CH0_ISR_PRIORITY)
{
    interrupt_global_enable(0);                     // �����ж�Ƕ��
    pit_clear_flag(CCU61_CH0);




}

IFX_INTERRUPT(cc61_pit_ch1_isr, 0, CCU6_1_CH1_ISR_PRIORITY)
{
    interrupt_global_enable(0);                     // �����ж�Ƕ��
    pit_clear_flag(CCU61_CH1);




}
// **************************** PIT�жϺ��� ****************************


// **************************** �ⲿ�жϺ��� ****************************
IFX_INTERRUPT(exti_ch0_ch4_isr, 0, EXTI_CH0_CH4_INT_PRIO)
{
    interrupt_global_enable(0);                     // �����ж�Ƕ��
    if(exti_flag_get(ERU_CH0_REQ0_P15_4))           // ͨ��0�ж�
    {
        exti_flag_clear(ERU_CH0_REQ0_P15_4);

    }

    if(exti_flag_get(ERU_CH4_REQ13_P15_5))          // ͨ��4�ж�
    {
        exti_flag_clear(ERU_CH4_REQ13_P15_5);
    }
}

IFX_INTERRUPT(exti_ch1_ch5_isr, 0, EXTI_CH1_CH5_INT_PRIO)
{
    interrupt_global_enable(0);                     // �����ж�Ƕ��

    if(exti_flag_get(ERU_CH1_REQ10_P14_3))          // ͨ��1�ж�
    {
        exti_flag_clear(ERU_CH1_REQ10_P14_3);

        tof_module_exti_handler();                  // ToF ģ�� INT �����ж�

    }

    if(exti_flag_get(ERU_CH5_REQ1_P15_8))           // ͨ��5�ж�
    {
        exti_flag_clear(ERU_CH5_REQ1_P15_8);


    }
}

// ��������ͷpclk����Ĭ��ռ���� 2ͨ�������ڴ���DMA��������ﲻ�ٶ����жϺ���
// IFX_INTERRUPT(exti_ch2_ch6_isr, 0, EXTI_CH2_CH6_INT_PRIO)
// {
//  interrupt_global_enable(0);                     // �����ж�Ƕ��
//  if(exti_flag_get(ERU_CH2_REQ7_P00_4))           // ͨ��2�ж�
//  {
//      exti_flag_clear(ERU_CH2_REQ7_P00_4);
//  }
//  if(exti_flag_get(ERU_CH6_REQ9_P20_0))           // ͨ��6�ж�
//  {
//      exti_flag_clear(ERU_CH6_REQ9_P20_0);
//  }
// }
IFX_INTERRUPT(exti_ch3_ch7_isr, 0, EXTI_CH3_CH7_INT_PRIO)
{
    interrupt_global_enable(0);                     // �����ж�Ƕ��
    if(exti_flag_get(ERU_CH3_REQ6_P02_0))           // ͨ��3�ж�
    {
        exti_flag_clear(ERU_CH3_REQ6_P02_0);
        camera_vsync_handler();                     // ����ͷ�����ɼ�ͳһ�ص�����
    }
    if(exti_flag_get(ERU_CH7_REQ16_P15_1))          // ͨ��7�ж�
    {
        exti_flag_clear(ERU_CH7_REQ16_P15_1);




    }
}
// **************************** �ⲿ�жϺ��� ****************************


// **************************** DMA�жϺ��� ****************************
IFX_INTERRUPT(dma_ch5_isr, 0, DMA_INT_PRIO)
{
    interrupt_global_enable(0);                     // �����ж�Ƕ��
    camera_dma_handler();                           // ����ͷ�ɼ����ͳһ�ص�����
}
// **************************** DMA�жϺ��� ****************************


// **************************** �����жϺ��� ****************************
// ����0Ĭ����Ϊ���Դ���
IFX_INTERRUPT(uart0_tx_isr, 0, UART0_TX_INT_PRIO)
{
    interrupt_global_enable(0);                     // �����ж�Ƕ��



}
IFX_INTERRUPT(uart0_rx_isr, 0, UART0_RX_INT_PRIO)
{
    interrupt_global_enable(0);                     // �����ж�Ƕ��

#if DEBUG_UART_USE_INTERRUPT                        // ������� debug �����ж�
        debug_interrupr_handler();                  // ���� debug ���ڽ��մ����� ���ݻᱻ debug ���λ�������ȡ
#endif                                              // ����޸��� DEBUG_UART_INDEX ����δ�����Ҫ�ŵ���Ӧ�Ĵ����ж�ȥ
}


// ����1Ĭ�����ӵ�����ͷ���ô���
IFX_INTERRUPT(uart1_tx_isr, 0, UART1_TX_INT_PRIO)
{
    interrupt_global_enable(0);                     // �����ж�Ƕ��




}
IFX_INTERRUPT(uart1_rx_isr, 0, UART1_RX_INT_PRIO)
{
    interrupt_global_enable(0);                     // �����ж�Ƕ��
    camera_uart_handler();                          // ����ͷ��������ͳһ�ص�����
}

// ����2Ĭ�����ӵ�����ת����ģ��
IFX_INTERRUPT(uart2_tx_isr, 0, UART2_TX_INT_PRIO)
{
    interrupt_global_enable(0);                     // �����ж�Ƕ��



}

IFX_INTERRUPT(uart2_rx_isr, 0, UART2_RX_INT_PRIO)
{
    interrupt_global_enable(0);                     // �����ж�Ƕ��
    wireless_module_uart_handler();                 // ����ģ��ͳһ�ص�����



}
// ����3Ĭ�����ӵ�GPS��λģ��
IFX_INTERRUPT(uart3_tx_isr, 0, UART3_TX_INT_PRIO)
{
    interrupt_global_enable(0);                     // �����ж�Ƕ��



}

IFX_INTERRUPT(uart3_rx_isr, 0, UART3_RX_INT_PRIO)
{
    interrupt_global_enable(0);                     // �����ж�Ƕ��

//    gnss_uart_callback();                           // GNSS���ڻص�����
    uart_control_callback();


}

// ����ͨѶ�����ж�
IFX_INTERRUPT(uart0_er_isr, 0, UART0_ER_INT_PRIO)
{
    interrupt_global_enable(0);                     // �����ж�Ƕ��
    IfxAsclin_Asc_isrError(&uart0_handle);
}
IFX_INTERRUPT(uart1_er_isr, 0, UART1_ER_INT_PRIO)
{
    interrupt_global_enable(0);                     // �����ж�Ƕ��
    IfxAsclin_Asc_isrError(&uart1_handle);
}
IFX_INTERRUPT(uart2_er_isr, 0, UART2_ER_INT_PRIO)
{
    interrupt_global_enable(0);                     // �����ж�Ƕ��
    IfxAsclin_Asc_isrError(&uart2_handle);
}
IFX_INTERRUPT(uart3_er_isr, 0, UART3_ER_INT_PRIO)
{
    interrupt_global_enable(0);                     // �����ж�Ƕ��
    IfxAsclin_Asc_isrError(&uart3_handle);
}
