/*
 * wireless.c
 *
 *  Created on: 2024年6月13日
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
#define WIFI_PASSWORD      "SMARTCAR_WIFI"  // 如果需要连接的WIFI 没有密码则需要将 这里 替换为 NULL
#define TCP_TARGET_IP           "192.168.31.12"             // 连接目标的 IP
#define TCP_TARGET_PORT         "8086"                      // 连接目标的端口
#define WIFI_LOCAL_PORT         "6666"                      // 本机的端口 0：随机  可设置范围2048-65535  默认 6666
float data[VOFA_DATA_SIZE]={0};
#if(0 == SEND_TYPE)//0：发送图像和边界至逐飞助手

// 边界的点数量远大于图像高度，便于保存回弯的情况
#define BOUNDARY_NUM            (MT9V03X_H * 2)

// 只有X边界
uint8 xy_x1_boundary[BOUNDARY_NUM], xy_x2_boundary[BOUNDARY_NUM], xy_x3_boundary[BOUNDARY_NUM];
// 只有Y边界
uint8 xy_y1_boundary[BOUNDARY_NUM], xy_y2_boundary[BOUNDARY_NUM], xy_y3_boundary[BOUNDARY_NUM];

// X Y边界都是单独指定的
uint8 x1_boundary[MT9V03X_H], x2_boundary[MT9V03X_H], x3_boundary[MT9V03X_H];
uint8 y1_boundary[MT9V03X_W], y2_boundary[MT9V03X_W], y3_boundary[MT9V03X_W];
// 图像备份数组，在发送前将图像备份再进行发送，这样可以避免图像出现撕裂的问题
uint8 image_copy[MT9V03X_H][MT9V03X_W];

#elif(1 == SEND_TYPE)//1：发送data[8]至vofa

#define VOFA_BUFFER_SIZE              ( 0x80 )
unsigned char copy_vofa_data[VOFA_DATA_SIZE*4+4] = {0};

static uint8        vofa_buffer[VOFA_BUFFER_SIZE];    
static fifo_struct  vofa_fifo =                                                                       // FIFO结构体
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
        system_delay_ms(100);                                                   // 初始化失败 等待 100ms
    }

    printf("\r\n module version:%s",wifi_spi_version);                          // 模块固件版本
    printf("\r\n module mac    :%s",wifi_spi_mac_addr);                         // 模块 MAC 信息
    printf("\r\n module ip     :%s",wifi_spi_ip_addr_port);                     // 模块 IP 地址

    // zf_device_wifi_spi.h 文件内的宏定义可以更改模块连接(建立) WIFI 之后，是否自动连接 TCP 服务器、创建 UDP 连接
    if(1 != WIFI_SPI_AUTO_CONNECT)                                              // 如果没有开启自动连接 就需要手动连接目标 IP
    {
        while(wifi_spi_socket_connect(                                          // 向指定目标 IP 的端口建立 TCP 连接
            "TCP",                                                              // 指定使用TCP方式通讯
            TCP_TARGET_IP,                                                      // 指定远端的IP地址，填写上位机的IP地址
            TCP_TARGET_PORT,                                                    // 指定远端的端口号，填写上位机的端口号，通常上位机默认是8080
            WIFI_LOCAL_PORT))                                                   // 指定本机的端口号
        {
            // 如果一直建立失败 考虑一下是不是没有接硬件复位
            printf("\r\n Connect TCP Servers error, try again.");
            system_delay_ms(100);                                               // 建立连接失败 等待 100ms
        }
    }

    #if(0 == SEND_TYPE)//0：使用逐飞助手发送

    //逐飞助手初始化 数据传输使用高速WIFI SPI
    // seekfree_assistant_interface_init(SEEKFREE_ASSISTANT_WIFI_SPI);
    // seekfree_assistant_camera_information_config(SEEKFREE_ASSISTANT_MT9V03X, image_copy[0], MT9V03X_W, MT9V03X_H);// 发送总钻风图像信息(并且包含三条边界信息，边界信息含有横纵轴坐标)
    // seekfree_assistant_camera_boundary_config(XY_BOUNDARY, BOUNDARY_NUM, xy_x1_boundary, xy_x2_boundary, xy_x3_boundary, xy_y1_boundary, xy_y2_boundary, xy_y3_boundary);

    seekfree_assistant_interface_init(SEEKFREE_ASSISTANT_WIFI_SPI);
    seekfree_assistant_camera_information_config(SEEKFREE_ASSISTANT_MT9V03X, image_copy[0], MT9V03X_W, MT9V03X_H);
    seekfree_assistant_camera_boundary_config(X_BOUNDARY, MT9V03X_H, x1_boundary, x2_boundary, x3_boundary, NULL, NULL ,NULL);
    #elif(1 == SEND_TYPE)//1：使用VOFA发送
    // 发送总钻风图像信息(并且包含三条边界信息，边界信息只含有横轴坐标，纵轴坐标由图像高度得到，意味着每个边界在一行中只会有一个点)
    // 对边界数组写入数据

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
    // 发送图像
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
        // 将读取到的数据写入FIFO
        fifo_write_buffer(&vofa_fifo, (uint8 *)temp_buffer, read_length);
    }

    while(sizeof(vofa_parameter_struct) <= fifo_used(&vofa_fifo))
    {
        read_length = sizeof(vofa_parameter_struct);
        fifo_read_buffer(&vofa_fifo, (uint8 *)temp_buffer, &read_length, FIFO_READ_ONLY);

        if(VOFA_RECEIVE_HEAD != ((uint8 *)temp_buffer)[0])
        {
            // 没有帧头则从FIFO中去掉第一个数据
            read_length = 1;
        }
        else
        {
            // 找到帧头
            receive_packet = (vofa_parameter_struct *)temp_buffer;
            temp_sum = receive_packet->check_sum;
            receive_packet->check_sum = 0;
            if(temp_sum == vofa_sum((uint8 *)temp_buffer, sizeof(vofa_parameter_struct)))
            {
                // 和校验成功保存数据
                receive_packet->check_sum = temp_sum;
                memcpy((uint8*)&vofa_parameter, (uint8*)receive_packet, sizeof(vofa_parameter_struct));
            }
            else
            {
                read_length = 1;
            }
        }

        // 丢弃无需使用的数据
        fifo_read_buffer(&vofa_fifo, (uint8 *)temp_buffer, &read_length, FIFO_READ_AND_CLEAN);
    }
    #endif
}


    


