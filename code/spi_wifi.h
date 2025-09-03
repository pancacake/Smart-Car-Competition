#ifndef SPI_WIFI_H_
#define SPI_WIFI_H_

//include

#include "zf_common_typedef.h"

//define
#define SEND_TYPE   0
//0：发送图像和边界至逐飞助手
//1：发送data[8]至vofa
#if(1 == SEND_TYPE)
//全局变量


#define VOFA_RECEIVE_HEAD             ( 0x17 )


typedef struct
{
    uint8 head;                                                 // 帧头
    uint8 function;                                             // 功能字
    uint8 channel;                                              // 通道
    uint8 check_sum;                                            // 和校验
    float data;                                                 // 数据
}vofa_parameter_struct;

extern vofa_parameter_struct vofa_parameter;
#endif

#define VOFA_DATA_SIZE    (15)
extern float data[VOFA_DATA_SIZE];
//声明函数

void wireless_device_send(void);//发送数据
void wireless_device_init(void);//初始化
void wireless_device_receive(void);//收数据
#endif
