#ifndef SPI_WIFI_H_
#define SPI_WIFI_H_

//include

#include "zf_common_typedef.h"

//define
#define SEND_TYPE   0
//0������ͼ��ͱ߽����������
//1������data[8]��vofa
#if(1 == SEND_TYPE)
//ȫ�ֱ���


#define VOFA_RECEIVE_HEAD             ( 0x17 )


typedef struct
{
    uint8 head;                                                 // ֡ͷ
    uint8 function;                                             // ������
    uint8 channel;                                              // ͨ��
    uint8 check_sum;                                            // ��У��
    float data;                                                 // ����
}vofa_parameter_struct;

extern vofa_parameter_struct vofa_parameter;
#endif

#define VOFA_DATA_SIZE    (15)
extern float data[VOFA_DATA_SIZE];
//��������

void wireless_device_send(void);//��������
void wireless_device_init(void);//��ʼ��
void wireless_device_receive(void);//������
#endif
