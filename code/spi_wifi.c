/*
 * wireless.c
 *
 *  Created on: 2024��6��13��
 *      Author: Merlin_yang
 */
#include "spi_wifi.h"
#include "zf_driver_delay.h"
#include "camera.h"
#include "zf_device_mt9v03x.h"
#include "zf_device_wireless_uart.h"
#include "zf_device_wifi_spi.h"
#include "seekfree_assistant.h"
#include "seekfree_assistant_interface.h"
#include "zf_common_fifo.h"



#define WIFI_SSID          "SMARTCAR_WIFI"
#define WIFI_PASSWORD      "SMARTCAR_WIFI"  // �����Ҫ���ӵ�WIFI û����������Ҫ�� ���� �滻Ϊ NULL
#define TCP_TARGET_IP           "192.168.31.12"             // ����Ŀ��� IP
#define TCP_TARGET_PORT         "8086"                      // ����Ŀ��Ķ˿�
#define WIFI_LOCAL_PORT         "6666"                      // �����Ķ˿� 0�����  �����÷�Χ2048-65535  Ĭ�� 6666
float data[VOFA_DATA_SIZE]={0};
#if(0 == SEND_TYPE)//0������ͼ��ͱ߽����������

// �߽�ĵ�����Զ����ͼ��߶ȣ����ڱ����������
#define BOUNDARY_NUM            (MT9V03X_H * 2)

// ֻ��X�߽�
uint8 xy_x1_boundary[BOUNDARY_NUM], xy_x2_boundary[BOUNDARY_NUM], xy_x3_boundary[BOUNDARY_NUM];
// ֻ��Y�߽�
uint8 xy_y1_boundary[BOUNDARY_NUM], xy_y2_boundary[BOUNDARY_NUM], xy_y3_boundary[BOUNDARY_NUM];

// X Y�߽綼�ǵ���ָ����
uint8 x1_boundary[MT9V03X_H], x2_boundary[MT9V03X_H], x3_boundary[MT9V03X_H];
uint8 y1_boundary[MT9V03X_W], y2_boundary[MT9V03X_W], y3_boundary[MT9V03X_W];
// ͼ�񱸷����飬�ڷ���ǰ��ͼ�񱸷��ٽ��з��ͣ��������Ա���ͼ�����˺�ѵ�����
uint8 image_copy[MT9V03X_H][MT9V03X_W];

#elif(1 == SEND_TYPE)//1������data[8]��vofa

#define VOFA_BUFFER_SIZE              ( 0x80 )
unsigned char copy_vofa_data[VOFA_DATA_SIZE*4+4] = {0};

static uint8        vofa_buffer[VOFA_BUFFER_SIZE];    
static fifo_struct  vofa_fifo =                                                                       // FIFO�ṹ��
{   
    .buffer    = vofa_buffer, 
    .execution = FIFO_IDLE, 
    .type      = FIFO_DATA_8BIT,    
    .head      = 0, 
    .end       = 0, 
    .size      = VOFA_BUFFER_SIZE,    
    .max       = VOFA_BUFFER_SIZE,    
};  

vofa_parameter_struct vofa_parameter;

static uint8 vofa_sum (uint8 *buffer, uint32 length)
{
    uint8 temp_sum = 0;

    while(length--)
    {
        temp_sum += *buffer++;
    }

    return temp_sum;
}

#endif


void wireless_device_init(void)
{

    while(wifi_spi_init(WIFI_SSID, WIFI_PASSWORD))
    {
        printf("\r\n connect wifi failed. \r\n");
        system_delay_ms(100);                                                   // ��ʼ��ʧ�� �ȴ� 100ms
    }

    printf("\r\n module version:%s",wifi_spi_version);                          // ģ��̼��汾
    printf("\r\n module mac    :%s",wifi_spi_mac_addr);                         // ģ�� MAC ��Ϣ
    printf("\r\n module ip     :%s",wifi_spi_ip_addr_port);                     // ģ�� IP ��ַ

    // zf_device_wifi_spi.h �ļ��ڵĺ궨����Ը���ģ������(����) WIFI ֮���Ƿ��Զ����� TCP ������������ UDP ����
    if(1 != WIFI_SPI_AUTO_CONNECT)                                              // ���û�п����Զ����� ����Ҫ�ֶ�����Ŀ�� IP
    {
        while(wifi_spi_socket_connect(                                          // ��ָ��Ŀ�� IP �Ķ˿ڽ��� TCP ����
            "TCP",                                                              // ָ��ʹ��TCP��ʽͨѶ
            TCP_TARGET_IP,                                                      // ָ��Զ�˵�IP��ַ����д��λ����IP��ַ
            TCP_TARGET_PORT,                                                    // ָ��Զ�˵Ķ˿ںţ���д��λ���Ķ˿ںţ�ͨ����λ��Ĭ����8080
            WIFI_LOCAL_PORT))                                                   // ָ�������Ķ˿ں�
        {
            // ���һֱ����ʧ�� ����һ���ǲ���û�н�Ӳ����λ
            printf("\r\n Connect TCP Servers error, try again.");
            system_delay_ms(100);                                               // ��������ʧ�� �ȴ� 100ms
        }
    }

    #if(0 == SEND_TYPE)//0��ʹ��������ַ���

    //������ֳ�ʼ�� ���ݴ���ʹ�ø���WIFI SPI
    // seekfree_assistant_interface_init(SEEKFREE_ASSISTANT_WIFI_SPI);
    // seekfree_assistant_camera_information_config(SEEKFREE_ASSISTANT_MT9V03X, image_copy[0], MT9V03X_W, MT9V03X_H);// ���������ͼ����Ϣ(���Ұ��������߽���Ϣ���߽���Ϣ���к���������)
    // seekfree_assistant_camera_boundary_config(XY_BOUNDARY, BOUNDARY_NUM, xy_x1_boundary, xy_x2_boundary, xy_x3_boundary, xy_y1_boundary, xy_y2_boundary, xy_y3_boundary);

    seekfree_assistant_interface_init(SEEKFREE_ASSISTANT_WIFI_SPI);
    seekfree_assistant_camera_information_config(SEEKFREE_ASSISTANT_MT9V03X, image_copy[0], MT9V03X_W, MT9V03X_H);
    seekfree_assistant_camera_boundary_config(X_BOUNDARY, MT9V03X_H, x1_boundary, x2_boundary, x3_boundary, NULL, NULL ,NULL);
    #elif(1 == SEND_TYPE)//1��ʹ��VOFA����
    // ���������ͼ����Ϣ(���Ұ��������߽���Ϣ���߽���Ϣֻ���к������꣬����������ͼ��߶ȵõ�����ζ��ÿ���߽���һ����ֻ����һ����)
    // �Ա߽�����д������

    #endif
}



void wireless_device_send(void)
    {
    #if(0 == SEND_TYPE)

    memcpy(image_copy[0], mt9v03x_image[0], MT9V03X_IMAGE_SIZE);
    for (int i=119;i>=0;i--)
    {
        //x1_boundary[i] = (uint8)leftline[i];
        //x2_boundary[i] = (uint8)midline[i];
        //x3_boundary[i] = (uint8)rightline[i];
    }
    // ����ͼ��
    seekfree_assistant_camera_send();

    #elif(1 == SEND_TYPE)
    memcpy(copy_vofa_data, (unsigned char *)data, sizeof(data));
    copy_vofa_data[VOFA_DATA_SIZE*4+2] = 0x80;
    copy_vofa_data[VOFA_DATA_SIZE*4+3] = 0x7f;
    wifi_spi_send_buffer(copy_vofa_data, VOFA_DATA_SIZE*4+4);
    #endif
    }

void wireless_device_receive(void)
{
    #if(0 == SEND_TYPE)
    seekfree_assistant_data_analysis ();
    #elif(1 == SEND_TYPE)
    uint8  temp_sum;
    uint32 read_length;
    vofa_parameter_struct *receive_packet;

    uint32  temp_buffer[VOFA_BUFFER_SIZE / 4];

    read_length = wifi_spi_read_buffer((uint8 *)temp_buffer, VOFA_BUFFER_SIZE);


    if(read_length)
    {
        // ����ȡ��������д��FIFO
        fifo_write_buffer(&vofa_fifo, (uint8 *)temp_buffer, read_length);
    }

    while(sizeof(vofa_parameter_struct) <= fifo_used(&vofa_fifo))
    {
        read_length = sizeof(vofa_parameter_struct);
        fifo_read_buffer(&vofa_fifo, (uint8 *)temp_buffer, &read_length, FIFO_READ_ONLY);

        if(VOFA_RECEIVE_HEAD != ((uint8 *)temp_buffer)[0])
        {
            // û��֡ͷ���FIFO��ȥ����һ������
            read_length = 1;
        }
        else
        {
            // �ҵ�֡ͷ
            receive_packet = (vofa_parameter_struct *)temp_buffer;
            temp_sum = receive_packet->check_sum;
            receive_packet->check_sum = 0;
            if(temp_sum == vofa_sum((uint8 *)temp_buffer, sizeof(vofa_parameter_struct)))
            {
                // ��У��ɹ���������
                receive_packet->check_sum = temp_sum;
                memcpy((uint8*)&vofa_parameter, (uint8*)receive_packet, sizeof(vofa_parameter_struct));
            }
            else
            {
                read_length = 1;
            }
        }

        // ��������ʹ�õ�����
        fifo_read_buffer(&vofa_fifo, (uint8 *)temp_buffer, &read_length, FIFO_READ_AND_CLEAN);
    }
    #endif
}


    


