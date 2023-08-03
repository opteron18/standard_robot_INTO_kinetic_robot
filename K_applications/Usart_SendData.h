#ifndef __USART_SEND_H
#define __USART_SEND_H
#include "sys.h"
#include "usart.h"

extern int SpinTop_Flag_cnt;
extern int Vision_Flag_cnt;
extern int Fire_Mode_cnt;
extern int Chassis_Mode_cnt;
extern int Stronghold_Flag_cnt;

void graphic_1t(u8 Variable);
void number_t(u8 Variable);
//***************************************************************�Զ���*********************************************************************************
/* ֡ͷ */
typedef __packed struct
{
	uint8_t  SOF;
	uint16_t DataLength;
	uint8_t  Seq;
	uint8_t  CRC8;
} FrameHeader_struct;


/* �������� */
typedef __packed struct
{
  uint16_t cmd_ID;//���ݶε����� ID
  uint16_t send_ID;//�����ߵ� ID  �� ��ҪУ�鷢���ߵ� ID ��ȷ�ԣ������ 1 ���͸��� 5��������ҪУ��� 1
  uint16_t receiver_ID;//�����ߵ� ID �� ��ҪУ������ߵ� ID ��ȷ�ԣ����粻�ܷ��͵��жԻ����˵�ID
} ID_struct;

/***************************���ݴ���*******************************/
/******** - ͼ�� -*************/
typedef __packed struct
{
	uint8_t  graphic_name[3]; 
	uint32_t operate_tpye:3;//ͼ�β���
	uint32_t graphic_tpye:3;//ͼ������
	uint32_t layer:4;//ͼ�β���
	uint32_t color:4;//ͼ����ɫ
	uint32_t start_angle:9;//��ʼ�Ƕ�
	uint32_t end_angle:9;//�����Ƕ�
	uint32_t width:10;//�߿�
	uint32_t start_x:11;//��ʼx����
	uint32_t start_y:11;//��ʼy����
	uint32_t radius:10;//�����С��뾶
	uint32_t end_x:11;//����x����
	uint32_t end_y:11;//����y����
} graphic_data_struct;


/************** - ������ - - ������ -***************/
typedef __packed struct
{
	uint8_t  number_name[3]; 
	uint32_t operate_tpye:3;//ͼ�β���
	uint32_t graphic_tpye:3;//ͼ������
	uint32_t layer:4;//ͼ�β���
	uint32_t color:4;//ͼ����ɫ
	uint32_t start_angle:9;//��ʼ�Ƕ�
	uint32_t end_angle:9;//�����Ƕ�
	uint32_t width:10;//�߿�
	uint32_t start_x:11;//��ʼx����
	uint32_t start_y:11;//��ʼy����
	int32_t  data;//����
} number_data_struct;


/************** - �ַ� -***************/

typedef __packed struct
{
	uint8_t  character_name[3]; 
	uint32_t operate_tpye:3;//ͼ�β���
	uint32_t graphic_tpye:3;//ͼ������
	uint32_t layer:4;//ͼ�β���
	uint32_t color:4;//ͼ����ɫ
	uint32_t start_angle:9;//��ʼ�Ƕ�
	uint32_t end_angle:9;//�����Ƕ�
	uint32_t width:10;//�߿�
	uint32_t start_x:11;//��ʼx����
	uint32_t start_y:11;//��ʼy����
	uint32_t radius:10;//�����С��뾶
	uint32_t end_x:11;//����x����
	uint32_t end_y:11;//����y����
  unsigned char     data[30];
} character_data_struct;
/*******************************************************************************/	

/************** - ���ͽṹ�� -***************/
typedef __packed struct
{
	FrameHeader_struct  	 Frame;
	uint16_t 							 CmdID;//������
	ID_struct           	 ID;
	graphic_data_struct  	 data[7];
	uint16_t  						 CRC16;
} graphic_struct;

typedef __packed struct
{
	FrameHeader_struct  	 Frame;
	uint16_t 							 CmdID;//������
	ID_struct           	 ID;
	number_data_struct 	 	 data[2];
	uint16_t  						 CRC16;
} number_struct;

typedef __packed struct
{
	FrameHeader_struct  	 Frame;
	uint16_t 							 CmdID;//������
	ID_struct            	 ID;
	character_data_struct  data;
	uint16_t  						 CRC16;
} character_struct;

void FRT_USART_DATA_Ctrl(void *pvParameters);

#endif
