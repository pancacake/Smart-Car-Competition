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
* �ļ�����          cpu0_main
* ��˾����          �ɶ���ɿƼ����޹�˾
* �汾��Ϣ          �鿴 libraries/doc �ļ����� version �ļ� �汾˵��
* ��������          ADS v1.9.4
* ����ƽ̨          TC264D
* ��������          https://seekfree.taobao.com/
*
* �޸ļ�¼
* ����              ����                ��ע
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
// ���������#pragma section all restore���֮���ȫ�ֱ���������CPU0��RAM��

// *************************** ����Ӳ������˵�� ***************************
// ʹ����ɿƼ� tc264 V2.6���� ����������ʽ���н���
//      ģ������    ��Ƭ������
//      RX          �鿴 small_driver_uart_control.h �� SMALL_DRIVER_TX  �궨�� Ĭ�� P15_7
//      TX          �鿴 small_driver_uart_control.h �� SMALL_DRIVER_RX  �궨�� Ĭ�� P15_6
//      GND         GND


// *************************** ���̲���˵�� ***************************
// 1.���İ���¼��ɱ����� �����ع��� ���� CYT2BL3 FOC ˫��
// 2.�������ʹ�� ���ȵ��˫���ϵ�MODE���� �Խ������λ�� ����ʱ ����ᷢ������
// 3.���������������λ���Ͽ������´�����Ϣ��
//      left speed:xxxx, right speed:xxxx
// �������������˵�����ز��� ����ձ��ļ����·� ���̳�������˵�� �����Ų�

// **************************** �������� ****************************
/*#define SERVO_MOTOR_PWM1             (ATOM0_CH0_P21_2)                           // ���������϶����Ӧ����
#define SERVO_MOTOR_PWM2             (ATOM0_CH2_P21_4)
#define SERVO_MOTOR_PWM3             (ATOM0_CH1_P21_3)
#define SERVO_MOTOR_PWM4             (ATOM0_CH1_P21_3)
#define SERVO_MOTOR_FREQ            (200 )                                       // ���������϶��Ƶ��  �����ע�ⷶΧ 50-300

#define SERVO_MOTOR_L_MAX           (0 )                                       // ���������϶�����Χ �Ƕ�
#define SERVO_MOTOR_R_MAX           (180)                                       // ���������϶�����Χ �Ƕ�
#define MAX_DUTY            (30)                                                // ��� ���� ռ�ձ�

#define SERVO_MOTOR_DUTY(x)         ((float)PWM_DUTY_MAX/(1000.0/(float)SERVO_MOTOR_FREQ)*(0.5+(float)(x)/90.0))

#if (SERVO_MOTOR_FREQ<50 || SERVO_MOTOR_FREQ>300)
    #error "SERVO_MOTOR_FREQ ERROE!"
#endif
*/


#define IPS200_TYPE     (IPS200_TYPE_SPI)

float servo_motor_duty = 180;                                                  // ��������Ƕ�
float servo_motor_dir = 1;                                                      // �������״̬

int8 duty = 0;
bool dir = true;


int core0_main(void)
{
    clock_init();                   // ��ȡʱ��Ƶ��<��ر���>
    debug_init();                   // ��ʼ��Ĭ�ϵ��Դ���
    // �˴���д�û����� ���������ʼ�������
    system_delay_ms(100);
    small_driver_uart_init();		// ��ʼ������ͨѶ����
    system_delay_ms(100);
    steer_control_init();
    gpio_init(LED1, GPO, GPIO_HIGH, GPO_PUSH_PULL);                             // ��ʼ�� LED1 ��� Ĭ�ϸߵ�ƽ �������ģʽ
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
            break;                                                  // ����ʱ�������Ʊ�ʾ�쳣
    }
    ips200_show_string(0, 16, "init success.");
    system_delay_ms(100);
    while(1)
        {
          if(imu660ra_init())
             printf("\r\n IMU660RA init error.");                                 // IMU660RA ��ʼ��ʧ��
          else
             break;
          gpio_toggle_level(LED1);                                                 // ��ת LED ���������ƽ ���� LED ���� ��ʼ����������ƻ����ĺ���
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
    // �˴���д�û����� ���������ʼ�������
    cpu_wait_event_ready();         // �ȴ����к��ĳ�ʼ�����
    circle_count = 0.0;
    target_speed = target_speed_normal;
    while (TRUE)
    {

        // �˴���д��Ҫѭ��ִ�еĴ���
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
            ips200_displayimage03x((const uint8 *)mt9v03x_image, MT9V03X_W, MT9V03X_H);                       // ��ʾԭʼͼ��
            //ips200_show_gray_image(0, 0, (const uint8 *)mt9v03x_image, MT9V03X_W, MT9V03X_H, 240, 180, 64);     // ��ʾ��ֵ��ͼ��
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
        //������Բ���
        //0��ת�ǡ��ٶȡ������ǻ��֡��������
        ips200_show_float(  0 , 150,   turn_angle, 5, 5);
        ips200_show_int( 80, 150, car_speed, 6);
        ips200_show_int(140, 150, target_speed, 3);
        ips200_show_int(200, 150, motor_value.receive_left_speed_data, 6);
        //ips200_show_float(80 , 150,   circle_count,  5, 5);
        //ips200_show_float(  120 , 150,   speed_integration, 5, 5);
        //1��ָ�������������ߵ�����������״̬���Ƿ�����
        //2����յ㡢�ҹյ㡢��ֱ�ߡ���ֱ��
        ips200_show_int(20, 170, left_outer, 6);
        ips200_show_int(80, 170, right_outer, 6);
        ips200_show_int(140, 170, crossline_time, 6);
        ips200_show_int(200, 170, is_left_straight, 6);
        //ips200_show_int(200, 170, traverse, 6);
        //3��ʮ�֡�����������״̬��������
        ips200_show_int(0, 210, is_barrier, 1);
        ips200_show_float(40, 210, circle_count, 5, 5);
        ips200_show_int(80, 210, barrier_state, 6);
        //ips200_show_int(40, 210, is_crossroad, 6);
        //ips200_show_int(80, 210, island_state, 6);
        //ips200_show_int(120, 210, island_flag, 6);
        //ips200_show_int(160, 210, is_single_bridge, 6);

        // �˴���д��Ҫѭ��ִ�еĴ���
    }
}


#pragma section all restore
// **************************** �������� ****************************
// *************************** ���̳�������˵�� ***************************
// ��������ʱ�밴�������������б���
// ����1������û������
//      �鿴���������λ���򿪵��Ƿ�����ȷ�Ĵ��ڣ����򿪵� COM ���Ƿ��Ӧ���ǵ������������� USB-TTL ģ��� COM ��
//      �����ʹ����ɿƼ� Ӣ����TriCore �������������ӣ���ô������������Ƿ��ɶ��������İ崮�������Ƿ��Ѿ����ӣ��������߲鿴���İ�ԭ��ͼ�����ҵ�
//      �����ʹ�� USB-TTL ģ�����ӣ���ô��������Ƿ������Ƿ��ɶ���ģ�� TX �Ƿ����ӵĺ��İ�� RX��ģ�� RX �Ƿ����ӵĺ��İ�� TX
// ����2��������������
//      �鿴���������λ�����õĲ������Ƿ����������һ�£������� zf_common_debug.h �ļ��� DEBUG_UART_BAUDRATE �궨��Ϊ debug uart ʹ�õĴ��ڲ�����
// ����3����ˢ����޷�Ӧ
//      ���Rx�ź������Ƿ�Ӷԣ��ź����Ƿ��ɶ�
// ����4����ˢ���ת����ת����ʾ���ٶ�
//      ���Tx�ź������Ƿ�Ӷԣ��ź����Ƿ��ɶ�
