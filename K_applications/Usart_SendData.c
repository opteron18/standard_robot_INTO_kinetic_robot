#include "Usart_SendData.h"
#include "referee.h"
#include "CAN_receive.h"
#include "CRC.h"
#include "string.h"
#include "stdbool.h"
//#include "delay.h"
#include "remote_control.h"
#include "Configuration.h"
#include "Higher_Class.h"
#include "User_Api.h"
#include "FreeRTOS.h"					
#include "task.h"
#include "User_Api.h"
#include "Ctrl_shoot.h"

u8 ex1 = 0;
u8 ex2 = 0;

/******************************************ID�ж�*******************************************************/
uint8_t Judge_Self_ID;                             //��ǰ�����˵�ID
uint16_t Judge_SelfClient_ID;                      //�����߻����˶�Ӧ�Ŀͻ���ID
/**
  * @brief  �ж��Լ�������
  * @param  void
  * @retval RED   BLUE
  * @attention  ���ݴ��,�����ɺ�ͨ�����ڷ��͵�����ϵͳ
  */
#define BLUE  0
#define RED   1
bool Color;
bool is_red_or_blue(void)
{
	Judge_Self_ID = Game_robot_state.robot_id;//��ȡ��ǰ������ID
	
	if(Game_robot_state.robot_id > 10)
	{
		return BLUE;
	}
	else 
	{
		return RED;
	}
}


/**
  * @brief  �ж�����ID��ѡ��ͻ���ID
  * @param  void
  * @retval RED   BLUE
  * @attention  ���ݴ��,�����ɺ�ͨ�����ڷ��͵�����ϵͳ
  */
void determine_ID(void)
{
	Color = is_red_or_blue();
	if(Color == BLUE)
	{
		Judge_SelfClient_ID = 0x0100 + Judge_Self_ID;//����ͻ���ID
	}
	else if(Color == RED)
	{
		Judge_SelfClient_ID = 0x0100 + Judge_Self_ID;//����ͻ���ID
	}	
}


/******************************************ID�ж�*******************************************************/
/******************�ṹ�弰����**************************/
#define send_max_len    200
unsigned char CliendTxBuffer_graphic[send_max_len];//���������
unsigned char CliendTxBuffer_number[send_max_len];//���������
unsigned char CliendTxBuffer_character[send_max_len];//���������
graphic_struct 		graphic;
number_struct     number;
character_struct  character;
/*******************************************���ݱ�д********************************************************/
/**********************ͼ������**********************/
//void graphic_1t(u8 Variable)
//{
//	determine_ID();//�жϷ�����ID�����Ӧ�Ŀͻ���ID
//	/*- ֡ͷ -*/	
//	graphic.Frame.SOF = 0xA5;
//	graphic.Frame.Seq = 0;	
//	graphic.Frame.DataLength = sizeof(graphic.ID) + sizeof(graphic.data);
//	/*- ID���� -*/
//  graphic.CmdID = 0x0301;//������
//	graphic.ID.cmd_ID = 0x0104;//��������ID 
//	//ID�Ѿ����Զ���ȡ����
//	graphic.ID.send_ID  = Judge_Self_ID;//�����ߵ�ID
//	graphic.ID.receiver_ID = Judge_SelfClient_ID;//�ͻ��˵�ID��ֻ��Ϊ�����߻����˶�Ӧ�Ŀͻ���
//	/*- �Զ�������һ -*/	
//	graphic.data[0].graphic_name[0] = 0;
//	graphic.data[0].graphic_name[1] = 0;
//	graphic.data[0].graphic_name[2] = 1;
//	graphic.data[0].operate_tpye = Variable;
//	graphic.data[0].graphic_tpye = 0;
//	graphic.data[0].layer = 1;
//	graphic.data[0].color = 2;
//	graphic.data[0].start_angle = 0;
//	graphic.data[0].end_angle = 0;
//	graphic.data[0].width = 1;
//	graphic.data[0].start_x = 940;//�е�x��960 y��540
//	graphic.data[0].start_y = 300;
//	graphic.data[0].radius = 0; 
//	graphic.data[0].end_x = 980;
//	graphic.data[0].end_y = 300;
//	/*- �Զ������ݶ� -*/	
//	graphic.data[1].graphic_name[0] = 0;
//	graphic.data[1].graphic_name[1] = 0;
//	graphic.data[1].graphic_name[2] = 2;
//	graphic.data[1].operate_tpye = Variable;
//	graphic.data[1].graphic_tpye = 0;
//	graphic.data[1].layer = 1;
//	graphic.data[1].color = 2;
//	graphic.data[1].start_angle = 0;
//	graphic.data[1].end_angle = 0;
//	graphic.data[1].width = 1;
//	graphic.data[1].start_x = 945;//�е�x��960 y��540
//	graphic.data[1].start_y = 500;
//	graphic.data[1].radius = 0; 
//	graphic.data[1].end_x = 980;
//	graphic.data[1].end_y = 500;	
//  /*- �Զ��������� -*/	
//	graphic.data[2].graphic_name[0] = 0;
//	graphic.data[2].graphic_name[1] = 0;
//	graphic.data[2].graphic_name[2] = 3;
//	graphic.data[2].operate_tpye = Variable;
//	graphic.data[2].graphic_tpye = 0;
//	graphic.data[2].layer = 1;
//	graphic.data[2].color = 2;
//	graphic.data[2].start_angle = 0;
//	graphic.data[2].end_angle = 0;
//	graphic.data[2].width = 1;
//	graphic.data[2].start_x = 950;//�е�x��960 y��540
//	graphic.data[2].start_y = 530;
//	graphic.data[2].radius = 0; 
//	graphic.data[2].end_x = 970;
//	graphic.data[2].end_y = 530;
//	/*- �Զ��������� -*/	
//	graphic.data[3].graphic_name[0] = 0;
//	graphic.data[3].graphic_name[1] = 0;
//	graphic.data[3].graphic_name[2] = 4;
//	graphic.data[3].operate_tpye = Variable;
//	graphic.data[3].graphic_tpye = 0;
//	graphic.data[3].layer = 1;
//	graphic.data[3].color = 2;
//	graphic.data[3].start_angle = 0;
//	graphic.data[3].end_angle = 0;
//	graphic.data[3].width = 1;
//	graphic.data[3].start_x = 950;//�е�x��960 y��540
//	graphic.data[3].start_y = 540;
//	graphic.data[3].radius = 0; 
//	graphic.data[3].end_x = 970;
//	graphic.data[3].end_y = 540;
//	/*- �Զ��������� -*/	
//	graphic.data[4].graphic_name[0] = 0;
//	graphic.data[4].graphic_name[1] = 0;
//	graphic.data[4].graphic_name[2] = 5;
//	graphic.data[4].operate_tpye = Variable;
//	graphic.data[4].graphic_tpye = 0;
//	graphic.data[4].layer = 1;
//	graphic.data[4].color = 2;
//	graphic.data[4].start_angle = 0;
//	graphic.data[4].end_angle = 0;
//	graphic.data[4].width = 1;
//	graphic.data[4].start_x = 950;//�е�x��960 y��540
//	graphic.data[4].start_y = 550;
//	graphic.data[4].radius = 0; 
//	graphic.data[4].end_x = 970;
//	graphic.data[4].end_y = 550;
//	/*- �Զ��������� -*/	
//	graphic.data[5].graphic_name[0] = 0;
//	graphic.data[5].graphic_name[1] = 0;
//	graphic.data[5].graphic_name[2] = 6;
//	graphic.data[5].operate_tpye = Variable;
//	graphic.data[5].graphic_tpye = 0;
//	graphic.data[5].layer = 1;
//	graphic.data[5].color = 2;
//	graphic.data[5].start_angle = 0;
//	graphic.data[5].end_angle = 0;
//	graphic.data[5].width = 1;
//	graphic.data[5].start_x = 950;//�е�x��960 y��540
//	graphic.data[5].start_y = 560;
//	graphic.data[5].radius = 0; 
//	graphic.data[5].end_x = 970;
//	graphic.data[5].end_y = 560;
//	/*- �Զ��������� -*/	
//	graphic.data[6].graphic_name[0] = 0;
//	graphic.data[6].graphic_name[1] = 0;
//	graphic.data[6].graphic_name[2] = 7;
//	graphic.data[6].operate_tpye = Variable;
//	graphic.data[6].graphic_tpye = 0;
//	graphic.data[6].layer = 1;
//	graphic.data[6].color = 4;
//	graphic.data[6].start_angle = 0;
//	graphic.data[6].end_angle = 0;
//	graphic.data[6].width = 5;
//	graphic.data[6].start_x = 0;//�е�x��960 y��540
//	graphic.data[6].start_y = 100;
//	graphic.data[6].radius = 0; 
//	graphic.data[6].end_x = 1560;
//	graphic.data[6].end_y = 100;
//	memcpy(CliendTxBuffer_graphic,&graphic,sizeof(graphic));
//	Append_CRC8_Check_Sum(CliendTxBuffer_graphic,sizeof(graphic.Frame));//******
//	Append_CRC16_Check_Sum(CliendTxBuffer_graphic,sizeof(graphic));//*******
//	
//	//  /*- ���д�뷢�� -*/
//	for(int i = 0;i < sizeof(CliendTxBuffer_graphic);i++)
//	{
//		USART_SendData(USART6,CliendTxBuffer_graphic[i]);
//		while(USART_GetFlagStatus(USART6,USART_FLAG_TXE)==RESET);
//	}
//	while(USART_GetFlagStatus(USART6,USART_FLAG_TC)==RESET);
//}


//void graphic_2t(u8 Variable)
//{
//	determine_ID();//�жϷ�����ID�����Ӧ�Ŀͻ���ID
//	/*- ֡ͷ -*/	
//	graphic.Frame.SOF = 0xA5;
//	graphic.Frame.Seq = 0;	
//	graphic.Frame.DataLength = sizeof(graphic.ID) + sizeof(graphic.data);
//	/*- ID���� -*/
//  graphic.CmdID = 0x0301;//������
//	graphic.ID.cmd_ID = 0x0104;//��������ID 
//	//ID�Ѿ����Զ���ȡ����
//	graphic.ID.send_ID  = Judge_Self_ID;//�����ߵ�ID
//	graphic.ID.receiver_ID = Judge_SelfClient_ID;//�ͻ��˵�ID��ֻ��Ϊ�����߻����˶�Ӧ�Ŀͻ���

//	/*- �Զ�������һ -*/	
//	graphic.data[0].graphic_name[0] = 0;
//	graphic.data[0].graphic_name[1] = 0;
//	graphic.data[0].graphic_name[2] = 8;
//	graphic.data[0].operate_tpye = Variable;
//	graphic.data[0].graphic_tpye = 0;
//	graphic.data[0].layer = 1;
//	graphic.data[0].color = 2;
//	graphic.data[0].start_angle = 0;
//	graphic.data[0].end_angle = 0;
//	graphic.data[0].width = 1;
//	graphic.data[0].start_x = 960;//�е�x��960 y��540
//	graphic.data[0].start_y = 560;
//	graphic.data[0].radius = 0; 
//	graphic.data[0].end_x = 960;
//	graphic.data[0].end_y = 440;
//	/*- �Զ������ݶ� -*/	
//	graphic.data[1].graphic_name[0] = 0;
//	graphic.data[1].graphic_name[1] = 0;
//	graphic.data[1].graphic_name[2] = 9;
//	graphic.data[1].operate_tpye = Variable;
//	graphic.data[1].graphic_tpye = 0;
//	graphic.data[1].layer = 1;
//	graphic.data[1].color = 2;
//	graphic.data[1].start_angle = 0;
//	graphic.data[1].end_angle = 0;
//	graphic.data[1].width = 1;
//	graphic.data[1].start_x = 960;//�е�x��960 y��540
//	graphic.data[1].start_y = 360;
//	graphic.data[1].radius = 0; 
//	graphic.data[1].end_x = 960;
//	graphic.data[1].end_y = 270;	
//  /*- �Զ��������� -*/	
//	graphic.data[2].graphic_name[0] = 0;
//	graphic.data[2].graphic_name[1] = 0;
//	graphic.data[2].graphic_name[2] = 10;
//	graphic.data[2].operate_tpye = Variable;
//	graphic.data[2].graphic_tpye = 0;
//	graphic.data[2].layer = 1;
//	graphic.data[2].color = 2;
//	graphic.data[2].start_angle = 0;
//	graphic.data[2].end_angle = 0;
//	graphic.data[2].width = 1;
//	graphic.data[2].start_x = 950;//�е�x��960 y��540
//	graphic.data[2].start_y = 430;
//	graphic.data[2].radius = 0; 
//	graphic.data[2].end_x = 950;
//	graphic.data[2].end_y = 370;
//	/*- �Զ��������� -*/	
//	graphic.data[3].graphic_name[0] = 0;
//	graphic.data[3].graphic_name[1] = 0;
//	graphic.data[3].graphic_name[2] = 11;
//	graphic.data[3].operate_tpye = Variable;
//	graphic.data[3].graphic_tpye = 0;
//	graphic.data[3].layer = 1;
//	graphic.data[3].color = 2;
//	graphic.data[3].start_angle = 0;
//	graphic.data[3].end_angle = 0;
//	graphic.data[3].width = 1;
//	graphic.data[3].start_x = 970;//�е�x��960 y��540
//	graphic.data[3].start_y = 430;
//	graphic.data[3].radius = 0; 
//	graphic.data[3].end_x = 970;
//	graphic.data[3].end_y = 370;
//	/*- �Զ��������� -*/	
//	graphic.data[4].graphic_name[0] = 0;
//	graphic.data[4].graphic_name[1] = 0;
//	graphic.data[4].graphic_name[2] = 12;
//	graphic.data[4].operate_tpye = Variable;
//	graphic.data[4].graphic_tpye = 0;
//	graphic.data[4].layer = 1;
//	graphic.data[4].color = 2;
//	graphic.data[4].start_angle = 0;
//	graphic.data[4].end_angle = 0;
//	graphic.data[4].width = 1;
//	graphic.data[4].start_x = 930;//�е�x��960 y��540
//	graphic.data[4].start_y = 470;
//	graphic.data[4].radius = 0; 
//	graphic.data[4].end_x = 990;
//	graphic.data[4].end_y = 470;
//	/*- �Զ��������� -*/	
//	graphic.data[5].graphic_name[0] = 0;
//	graphic.data[5].graphic_name[1] = 0;
//	graphic.data[5].graphic_name[2] = 13;
//	graphic.data[5].operate_tpye = Variable;
//	graphic.data[5].graphic_tpye = 0;
//	graphic.data[5].layer = 1;
//	graphic.data[5].color = 2;
//	graphic.data[5].start_angle = 0;
//	graphic.data[5].end_angle = 0;
//	graphic.data[5].width = 1;
//	graphic.data[5].start_x = 920;//�е�x��960 y��540
//	graphic.data[5].start_y = 440;
//	graphic.data[5].radius = 0; 
//	graphic.data[5].end_x = 1000;
//	graphic.data[5].end_y = 440;
//	/*- �Զ��������� -*/	
//	graphic.data[6].graphic_name[0] = 0;
//	graphic.data[6].graphic_name[1] = 0;
//	graphic.data[6].graphic_name[2] = 14;
//	graphic.data[6].operate_tpye = Variable;
//	graphic.data[6].graphic_tpye = 0;
//	graphic.data[6].layer = 1;
//	graphic.data[6].color = 2;
//	graphic.data[6].start_angle = 0;
//	graphic.data[6].end_angle = 0;
//	graphic.data[6].width = 1;
//	graphic.data[6].start_x = 910;//�е�x��960 y��540
//	graphic.data[6].start_y = 420;
//	graphic.data[6].radius = 0; 
//	graphic.data[6].end_x = 1010;
//	graphic.data[6].end_y = 420;
//	memcpy(CliendTxBuffer_graphic,&graphic,sizeof(graphic));
//	Append_CRC8_Check_Sum(CliendTxBuffer_graphic,sizeof(graphic.Frame));//******
//	Append_CRC16_Check_Sum(CliendTxBuffer_graphic,sizeof(graphic));//*******
//	
//	//  /*- ���д�뷢�� -*/
//	for(int i = 0;i < sizeof(CliendTxBuffer_graphic);i++)
//	{
//		USART_SendData(USART6,CliendTxBuffer_graphic[i]);
//		while(USART_GetFlagStatus(USART6,USART_FLAG_TXE)==RESET);
//	}
//	while(USART_GetFlagStatus(USART6,USART_FLAG_TC)==RESET);
//}


//void graphic_3t(u8 Variable)
//{
//	determine_ID();//�жϷ�����ID�����Ӧ�Ŀͻ���ID
//	/*- ֡ͷ -*/	
//	graphic.Frame.SOF = 0xA5;
//	graphic.Frame.Seq = 0;	
//	graphic.Frame.DataLength = sizeof(graphic.ID) + sizeof(graphic.data);
//	/*- ID���� -*/
//  graphic.CmdID = 0x0301;//������
//	graphic.ID.cmd_ID = 0x0104;//��������ID 
//	//ID�Ѿ����Զ���ȡ����
//	graphic.ID.send_ID  = Judge_Self_ID;//�����ߵ�ID
//	graphic.ID.receiver_ID = Judge_SelfClient_ID;//�ͻ��˵�ID��ֻ��Ϊ�����߻����˶�Ӧ�Ŀͻ���

//	/*- �Զ�������һ -*/	
//	graphic.data[0].graphic_name[0] = 0;
//	graphic.data[0].graphic_name[1] = 0;
//	graphic.data[0].graphic_name[2] = 15;
//	graphic.data[0].operate_tpye = Variable;
//	graphic.data[0].graphic_tpye = 2;
//	graphic.data[0].layer = 1;
//	graphic.data[0].color = 3;
//	graphic.data[0].start_angle = 0;
//	graphic.data[0].end_angle = 0;
//	graphic.data[0].width = 5;
//	graphic.data[0].start_x = 960;//�е�x��960 y��540
//	graphic.data[0].start_y = 400;
//	graphic.data[0].radius = 5; 
//	graphic.data[0].end_x = 0;
//	graphic.data[0].end_y = 0;
//	/*- �Զ������ݶ� -*/	
//	graphic.data[1].graphic_name[0] = 0;
//	graphic.data[1].graphic_name[1] = 0;
//	graphic.data[1].graphic_name[2] = 16;
//	graphic.data[1].operate_tpye = Variable;
//	graphic.data[1].graphic_tpye = 0;
//	graphic.data[1].layer = 1;
//	graphic.data[1].color = 2;
//	graphic.data[1].start_angle = 0;
//	graphic.data[1].end_angle = 0;
//	graphic.data[1].width = 1;
//	graphic.data[1].start_x = 910;//�е�x��960 y��540
//	graphic.data[1].start_y = 420;
//	graphic.data[1].radius = 0; 
//	graphic.data[1].end_x = 1010;
//	graphic.data[1].end_y = 420;	
//  /*- �Զ��������� -*/	
//	graphic.data[2].graphic_name[0] = 0;
//	graphic.data[2].graphic_name[1] = 0;
//	graphic.data[2].graphic_name[2] = 17;
//	graphic.data[2].operate_tpye = Variable;
//	graphic.data[2].graphic_tpye = 0;
//	graphic.data[2].layer = 1;
//	graphic.data[2].color = 2;
//	graphic.data[2].start_angle = 0;
//	graphic.data[2].end_angle = 0;
//	graphic.data[2].width = 1;
//	graphic.data[2].start_x = 910;//�е�x��960 y��540
//	graphic.data[2].start_y = 380;
//	graphic.data[2].radius = 0; 
//	graphic.data[2].end_x = 1010;
//	graphic.data[2].end_y = 380;
//	/*- �Զ��������� -*/	
//	graphic.data[3].graphic_name[0] = 0;
//	graphic.data[3].graphic_name[1] = 0;
//	graphic.data[3].graphic_name[2] = 18;
//	graphic.data[3].operate_tpye = Variable;
//	graphic.data[3].graphic_tpye = 0;
//	graphic.data[3].layer = 1;
//	graphic.data[3].color = 2;
//	graphic.data[3].start_angle = 0;
//	graphic.data[3].end_angle = 0;
//	graphic.data[3].width = 1;
//	graphic.data[3].start_x = 920;//�е�x��960 y��540
//	graphic.data[3].start_y = 360;
//	graphic.data[3].radius = 0; 
//	graphic.data[3].end_x = 1000;
//	graphic.data[3].end_y = 360;
//	/*- �Զ��������� -*/	
//	graphic.data[4].graphic_name[0] = 0;
//	graphic.data[4].graphic_name[1] = 0;
//	graphic.data[4].graphic_name[2] = 19;
//	graphic.data[4].operate_tpye = Variable;
//	graphic.data[4].graphic_tpye = 0;
//	graphic.data[4].layer = 1;
//	graphic.data[4].color = 2;
//	graphic.data[4].start_angle = 0;
//	graphic.data[4].end_angle = 0;
//	graphic.data[4].width = 1;
//	graphic.data[4].start_x = 930;//�е�x��960 y��540
//	graphic.data[4].start_y = 330;
//	graphic.data[4].radius = 0; 
//	graphic.data[4].end_x = 990;
//	graphic.data[4].end_y = 330;
//	/*- �Զ��������� -*/	
//	graphic.data[5].graphic_name[0] = 0;
//	graphic.data[5].graphic_name[1] = 0;
//	graphic.data[5].graphic_name[2] = 20;
//	graphic.data[5].operate_tpye = Variable;
//	graphic.data[5].graphic_tpye = 0;
//	graphic.data[5].layer = 1;
//	graphic.data[5].color = 2;
//	graphic.data[5].start_angle = 0;
//	graphic.data[5].end_angle = 0;
//	graphic.data[5].width = 1;
//	graphic.data[5].start_x = 930;//�е�x��961 y��540
//	graphic.data[5].start_y = 410;
//	graphic.data[5].radius = 0; 
//	graphic.data[5].end_x = 930;
//	graphic.data[5].end_y = 390;
//	/*- �Զ��������� -*/	
//	graphic.data[6].graphic_name[0] = 0;
//	graphic.data[6].graphic_name[1] = 0;
//	graphic.data[6].graphic_name[2] = 21;
//	graphic.data[6].operate_tpye = Variable;
//	graphic.data[6].graphic_tpye = 0;
//	graphic.data[6].layer = 1;
//	graphic.data[6].color = 2;
//	graphic.data[6].start_angle = 0;
//	graphic.data[6].end_angle = 0;
//	graphic.data[6].width = 1;
//	graphic.data[6].start_x = 990;//�е�x��960 y��540
//	graphic.data[6].start_y = 410;
//	graphic.data[6].radius = 0; 
//	graphic.data[6].end_x = 990;
//	graphic.data[6].end_y = 390;
//	memcpy(CliendTxBuffer_graphic,&graphic,sizeof(graphic));
//	Append_CRC8_Check_Sum(CliendTxBuffer_graphic,sizeof(graphic.Frame));//******
//	Append_CRC16_Check_Sum(CliendTxBuffer_graphic,sizeof(graphic));//*******
//	
//	//  /*- ���д�뷢�� -*/
//	for(int i = 0;i < sizeof(CliendTxBuffer_graphic);i++)
//	{
//		USART_SendData(USART6,CliendTxBuffer_graphic[i]);
//		while(USART_GetFlagStatus(USART6,USART_FLAG_TXE)==RESET);
//	}
//	while(USART_GetFlagStatus(USART6,USART_FLAG_TC)==RESET);
//}


///**********************ͼ������**********************/
///**********��������**********/
//void number_t(u8 Variable)
//{
//	determine_ID();//�жϷ�����ID�����Ӧ�Ŀͻ���ID
//	/*- ֡ͷ -*/	
//	number.Frame.SOF = 0xA5;
//	number.Frame.Seq = 0;	
//	number.Frame.DataLength = sizeof(number.ID) + sizeof(number.data);
//	/*- ID���� -*/
//  number.CmdID = 0x0301;//������
//	number.ID.cmd_ID = 0x0102;//��������ID 
//	//ID�Ѿ����Զ���ȡ����
//	number.ID.send_ID  = Judge_Self_ID;//�����ߵ�ID
//	number.ID.receiver_ID = Judge_SelfClient_ID;//�ͻ��˵�ID��ֻ��Ϊ�����߻����˶�Ӧ�Ŀͻ���
//	/*- �Զ�������һ -*/	
//	number.data[0].number_name[0] = 0;
//	number.data[0].number_name[1] = 1;
//	number.data[0].number_name[2] = 0;
//	number.data[0].operate_tpye = Variable;
//	number.data[0].graphic_tpye = 6;
//	number.data[0].layer = 1;
//	number.data[0].color = 4;
//	number.data[0].start_angle = 20;
//	number.data[0].end_angle = 0;
//	number.data[0].width = 2;//�����С���߿�10��1
//	number.data[0].start_x = 1080;
//	number.data[0].start_y = 610;
//	number.data[0].data = PowerData[1]*1000;
//	/*- �Զ������ݶ� -*/	
//	number.data[1].number_name[0] = 0;
//	number.data[1].number_name[1] = 2;
//	number.data[1].number_name[2] = 0;
//	number.data[1].operate_tpye = Variable;
//	number.data[1].graphic_tpye = 6;
//	number.data[1].layer = 1;
//	number.data[1].color = 8;
//	number.data[1].start_angle = 20;
//	number.data[1].end_angle = 0;
//	number.data[1].width = 2;//�����С���߿�10��1
//	number.data[1].start_x = 1080;
//	number.data[1].start_y = 550;
//	number.data[1].data = Control_data.Pitch_angle*1000;

//	
//	memcpy(CliendTxBuffer_number,&number,sizeof(number));
//	Append_CRC8_Check_Sum(CliendTxBuffer_number,sizeof(number.Frame));//******
//	Append_CRC16_Check_Sum(CliendTxBuffer_number,sizeof(number));//*******
//	
//	//  /*- ���д�뷢�� -*/
//	for(int i = 0;i < sizeof(CliendTxBuffer_number);i++)
//	{
//		USART_SendData(USART6,CliendTxBuffer_number[i]);
//		while(USART_GetFlagStatus(USART6,USART_FLAG_TXE)==RESET);
//	}
//	while(USART_GetFlagStatus(USART6,USART_FLAG_TC)==RESET);
//}


///**********��������**********/
///**********�ַ�����**********/
//void character_Cap_v(u8 Variable)
//{
//	determine_ID();//�жϷ�����ID�����Ӧ�Ŀͻ���ID
//	unsigned char character_demo[30]="Cap_v";
//	/*- ֡ͷ -*/	
//	character.Frame.SOF = 0xA5;
//	character.Frame.Seq = 0;	
//	character.Frame.DataLength = sizeof(character.ID) + sizeof(character.data);
//	/*- ID���� -*/
//  character.CmdID = 0x0301;//������
//	character.ID.cmd_ID = 0x0110;//��������ID 
//	//ID�Ѿ����Զ���ȡ����
//	character.ID.send_ID  = Judge_Self_ID;//�����ߵ�ID
//	character.ID.receiver_ID = Judge_SelfClient_ID;//�ͻ��˵�ID��ֻ��Ϊ�����߻����˶�Ӧ�Ŀͻ���
//	/*- �Զ�������һ -*/	
//	character.data.character_name[0] = 1;
//	character.data.character_name[1] = 0;
//	character.data.character_name[2] = 0;
//	character.data.operate_tpye = Variable;
//	character.data.graphic_tpye = 7;
//	character.data.layer = 2;
//	character.data.color = 3;
//	character.data.start_angle = 20;
//	character.data.end_angle = 5;
//	character.data.width = 2;
//	character.data.start_x = 1060;
//	character.data.start_y = 640;
//  memcpy(character.data.data, &character_demo, sizeof(character_demo));
//	/*- �Զ�������һ -*/	
//	
//	memcpy(CliendTxBuffer_character,&character,sizeof(character));
//	Append_CRC8_Check_Sum(CliendTxBuffer_character,sizeof(character.Frame));//******
//	Append_CRC16_Check_Sum(CliendTxBuffer_character,sizeof(character));//*******
//	
//	//  /*- ���д�뷢�� -*/
//	for(int i = 0;i < sizeof(CliendTxBuffer_character);i++)
//	{
//		USART_SendData(USART6,CliendTxBuffer_character[i]);
//		while(USART_GetFlagStatus(USART6,USART_FLAG_TXE)==RESET);
//	}
//	while(USART_GetFlagStatus(USART6,USART_FLAG_TC)==RESET);
//}


//void character_Pitch_angle(u8 Variable)
//{
//	determine_ID();//�жϷ�����ID�����Ӧ�Ŀͻ���ID
//	unsigned char character_demo[30]="Pitch";
//	/*- ֡ͷ -*/	
//	character.Frame.SOF = 0xA5;
//	character.Frame.Seq = 0;	
//	character.Frame.DataLength = sizeof(character.ID) + sizeof(character.data);
//	/*- ID���� -*/
//  character.CmdID = 0x0301;//������
//	character.ID.cmd_ID = 0x0110;//��������ID 
//	//ID�Ѿ����Զ���ȡ����
//	character.ID.send_ID  = Judge_Self_ID;//�����ߵ�ID
//	character.ID.receiver_ID = Judge_SelfClient_ID;//�ͻ��˵�ID��ֻ��Ϊ�����߻����˶�Ӧ�Ŀͻ���
//	/*- �Զ�������һ -*/	
//	character.data.character_name[0] = 2;
//	character.data.character_name[1] = 0;
//	character.data.character_name[2] = 0;
//	
//	character.data.operate_tpye = Variable;
//	character.data.graphic_tpye = 7;
//	character.data.layer = 2;
//	character.data.color = 6;
//	character.data.start_angle = 20;
//	character.data.end_angle = 5;
//	character.data.width = 2;
//	character.data.start_x = 1060;
//	character.data.start_y = 580;
//  memcpy(character.data.data, &character_demo, sizeof(character_demo));
//	/*- �Զ�������һ -*/	
//	
//	memcpy(CliendTxBuffer_character,&character,sizeof(character));
//	Append_CRC8_Check_Sum(CliendTxBuffer_character,sizeof(character.Frame));//******
//	Append_CRC16_Check_Sum(CliendTxBuffer_character,sizeof(character));//*******
//	
//	//  /*- ���д�뷢�� -*/
//	for(int i = 0;i < sizeof(CliendTxBuffer_character);i++)
//	{
//		USART_SendData(USART6,CliendTxBuffer_character[i]);
//		while(USART_GetFlagStatus(USART6,USART_FLAG_TXE)==RESET);
//	}
//	while(USART_GetFlagStatus(USART6,USART_FLAG_TC)==RESET);
//}


///**********�ַ�����**********/
//void character_SpinTop(u8 Variable)
//{
//	determine_ID();//�жϷ�����ID�����Ӧ�Ŀͻ���ID
//	unsigned char character_demo[30]="SpinTop";
//	/*- ֡ͷ -*/	
//	character.Frame.SOF = 0xA5;
//	character.Frame.Seq = 0;	
//	character.Frame.DataLength = sizeof(character.ID) + sizeof(character.data);
//	/*- ID���� -*/
//  character.CmdID = 0x0301;//������
//	character.ID.cmd_ID = 0x0110;//��������ID 
//	//ID�Ѿ����Զ���ȡ����
//	character.ID.send_ID  = Judge_Self_ID;//�����ߵ�ID
//	character.ID.receiver_ID = Judge_SelfClient_ID;//�ͻ��˵�ID��ֻ��Ϊ�����߻����˶�Ӧ�Ŀͻ���
//	/*- �Զ�������һ -*/	
//	character.data.character_name[0] = 3;
//	character.data.character_name[1] = 0;
//	character.data.character_name[2] = 0;
//	character.data.operate_tpye = Variable;
//	character.data.graphic_tpye = 7;
//	character.data.layer = 2;
//	character.data.color = 5;
//	character.data.start_angle = 20;
//	character.data.end_angle = 5;
//	character.data.width = 2;
//	character.data.start_x = 850;
//	character.data.start_y = 840;
//  memcpy(character.data.data, &character_demo, sizeof(character_demo));
//	/*- �Զ�������һ -*/	
//	
//	memcpy(CliendTxBuffer_character,&character,sizeof(character));
//	Append_CRC8_Check_Sum(CliendTxBuffer_character,sizeof(character.Frame));//******
//	Append_CRC16_Check_Sum(CliendTxBuffer_character,sizeof(character));//*******
//	
//	//  /*- ���д�뷢�� -*/
//	for(int i = 0;i < sizeof(CliendTxBuffer_character);i++)
//	{
//		USART_SendData(USART6,CliendTxBuffer_character[i]);
//		while(USART_GetFlagStatus(USART6,USART_FLAG_TXE)==RESET);
//	}
//	while(USART_GetFlagStatus(USART6,USART_FLAG_TC)==RESET);
//}


///**********�ַ�����**********/
//void character_Vision(u8 Variable)
//{
//	determine_ID();//�жϷ�����ID�����Ӧ�Ŀͻ���ID
//	unsigned char character_demo[30]="Vision";
//	/*- ֡ͷ -*/	
//	character.Frame.SOF = 0xA5;
//	character.Frame.Seq = 0;	
//	character.Frame.DataLength = sizeof(character.ID) + sizeof(character.data);
//	/*- ID���� -*/
//  character.CmdID = 0x0301;//������
//	character.ID.cmd_ID = 0x0110;//��������ID 
//	//ID�Ѿ����Զ���ȡ����
//	character.ID.send_ID  = Judge_Self_ID;//�����ߵ�ID
//	character.ID.receiver_ID = Judge_SelfClient_ID;//�ͻ��˵�ID��ֻ��Ϊ�����߻����˶�Ӧ�Ŀͻ���
//	/*- �Զ�������һ -*/	
//	character.data.character_name[0] = 4;
//	character.data.character_name[1] = 0;
//	character.data.character_name[2] = 0;
//	character.data.operate_tpye = Variable;
//	character.data.graphic_tpye = 7;
//	character.data.layer = 2;
//	character.data.color = 5;
//	character.data.start_angle = 20;
//	character.data.end_angle = 5;
//	character.data.width = 2;
//	character.data.start_x = 850;
//	character.data.start_y = 800;
//  memcpy(character.data.data, &character_demo, sizeof(character_demo));
//	/*- �Զ�������һ -*/	
//	
//	memcpy(CliendTxBuffer_character,&character,sizeof(character));
//	Append_CRC8_Check_Sum(CliendTxBuffer_character,sizeof(character.Frame));//******
//	Append_CRC16_Check_Sum(CliendTxBuffer_character,sizeof(character));//*******
//	
//	//  /*- ���д�뷢�� -*/
//	for(int i = 0;i < sizeof(CliendTxBuffer_character);i++)
//	{
//		USART_SendData(USART6,CliendTxBuffer_character[i]);
//		while(USART_GetFlagStatus(USART6,USART_FLAG_TXE)==RESET);
//	}
//	while(USART_GetFlagStatus(USART6,USART_FLAG_TC)==RESET);
//}


//void character_fire_mode1(u8 Variable)
//{
//	determine_ID();//�жϷ�����ID�����Ӧ�Ŀͻ���ID
//	unsigned char character_demo[30]="Booming_fire";
//	/*- ֡ͷ -*/	
//	character.Frame.SOF = 0xA5;
//	character.Frame.Seq = 0;	
//	character.Frame.DataLength = sizeof(character.ID) + sizeof(character.data);
//	/*- ID���� -*/
//  character.CmdID = 0x0301;//������
//	character.ID.cmd_ID = 0x0110;//��������ID 
//	//ID�Ѿ����Զ���ȡ����
//	character.ID.send_ID  = Judge_Self_ID;//�����ߵ�ID
//	character.ID.receiver_ID = Judge_SelfClient_ID;//�ͻ��˵�ID��ֻ��Ϊ�����߻����˶�Ӧ�Ŀͻ���
//	/*- �Զ�������һ -*/	
//	character.data.character_name[0] = 5;
//	character.data.character_name[1] = 0;
//	character.data.character_name[2] = 0;
//	character.data.operate_tpye = Variable;
//	character.data.graphic_tpye = 7;
//	character.data.layer = 2;
//	character.data.color = 5;
//	character.data.start_angle = 20;
//	character.data.end_angle = 5;
//	character.data.width = 2;
//	character.data.start_x = 30;
//	character.data.start_y = 800;
//  memcpy(character.data.data, &character_demo, sizeof(character_demo));
//	/*- �Զ�������һ -*/	
//	
//	memcpy(CliendTxBuffer_character,&character,sizeof(character));
//	Append_CRC8_Check_Sum(CliendTxBuffer_character,sizeof(character.Frame));//******
//	Append_CRC16_Check_Sum(CliendTxBuffer_character,sizeof(character));//*******
//	
//	//  /*- ���д�뷢�� -*/
//	for(int i = 0;i < sizeof(CliendTxBuffer_character);i++)
//	{
//		USART_SendData(USART6,CliendTxBuffer_character[i]);
//		while(USART_GetFlagStatus(USART6,USART_FLAG_TXE)==RESET);
//	}
//	while(USART_GetFlagStatus(USART6,USART_FLAG_TC)==RESET);
//}


//void character_fire_mode2(u8 Variable)
//{
//	determine_ID();//�жϷ�����ID�����Ӧ�Ŀͻ���ID
//	unsigned char character_demo[30]="Rerolling_fire";
//	/*- ֡ͷ -*/	
//	character.Frame.SOF = 0xA5;
//	character.Frame.Seq = 0;	
//	character.Frame.DataLength = sizeof(character.ID) + sizeof(character.data);
//	/*- ID���� -*/
//  character.CmdID = 0x0301;//������
//	character.ID.cmd_ID = 0x0110;//��������ID 
//	//ID�Ѿ����Զ���ȡ����
//	character.ID.send_ID  = Judge_Self_ID;//�����ߵ�ID
//	character.ID.receiver_ID = Judge_SelfClient_ID;//�ͻ��˵�ID��ֻ��Ϊ�����߻����˶�Ӧ�Ŀͻ���
//	/*- �Զ�������һ -*/	
//	character.data.character_name[0] = 6;
//	character.data.character_name[1] = 0;
//	character.data.character_name[2] = 0;
//	character.data.operate_tpye = Variable;
//	character.data.graphic_tpye = 7;
//	character.data.layer = 2;
//	character.data.color = 5;
//	character.data.start_angle = 20;
//	character.data.end_angle = 5;
//	character.data.width = 2;
//	character.data.start_x = 30;
//	character.data.start_y = 800;
//  memcpy(character.data.data, &character_demo, sizeof(character_demo));
//	/*- �Զ�������һ -*/	
//	
//	memcpy(CliendTxBuffer_character,&character,sizeof(character));
//	Append_CRC8_Check_Sum(CliendTxBuffer_character,sizeof(character.Frame));//******
//	Append_CRC16_Check_Sum(CliendTxBuffer_character,sizeof(character));//*******
//	
//	//  /*- ���д�뷢�� -*/
//	for(int i = 0;i < sizeof(CliendTxBuffer_character);i++)
//	{
//		USART_SendData(USART6,CliendTxBuffer_character[i]);
//		while(USART_GetFlagStatus(USART6,USART_FLAG_TXE)==RESET);
//	}
//	while(USART_GetFlagStatus(USART6,USART_FLAG_TC)==RESET);
//}


//void character_chassis_mode1(u8 Variable)
//{
//	determine_ID();//�жϷ�����ID�����Ӧ�Ŀͻ���ID
//	unsigned char character_demo[30]="POWER_Chassis";
//	/*- ֡ͷ -*/	
//	character.Frame.SOF = 0xA5;
//	character.Frame.Seq = 0;	
//	character.Frame.DataLength = sizeof(character.ID) + sizeof(character.data);
//	/*- ID���� -*/
//  character.CmdID = 0x0301;//������
//	character.ID.cmd_ID = 0x0110;//��������ID 
//	//ID�Ѿ����Զ���ȡ����
//	character.ID.send_ID  = Judge_Self_ID;//�����ߵ�ID
//	character.ID.receiver_ID = Judge_SelfClient_ID;//�ͻ��˵�ID��ֻ��Ϊ�����߻����˶�Ӧ�Ŀͻ���
//	/*- �Զ�������һ -*/	
//	character.data.character_name[0] = 7;
//	character.data.character_name[1] = 0;
//	character.data.character_name[2] = 0;
//	character.data.operate_tpye = Variable;
//	character.data.graphic_tpye = 7;
//	character.data.layer = 2;
//	character.data.color = 5;
//	character.data.start_angle = 20;
//	character.data.end_angle = 5;
//	character.data.width = 2;
//	character.data.start_x = 30;
//	character.data.start_y = 760;
//  memcpy(character.data.data, &character_demo, sizeof(character_demo));
//	/*- �Զ�������һ -*/	
//	
//	memcpy(CliendTxBuffer_character,&character,sizeof(character));
//	Append_CRC8_Check_Sum(CliendTxBuffer_character,sizeof(character.Frame));//******
//	Append_CRC16_Check_Sum(CliendTxBuffer_character,sizeof(character));//*******
//	
//	//  /*- ���д�뷢�� -*/
//	for(int i = 0;i < sizeof(CliendTxBuffer_character);i++)
//	{
//		USART_SendData(USART6,CliendTxBuffer_character[i]);
//		while(USART_GetFlagStatus(USART6,USART_FLAG_TXE)==RESET);
//	}
//	while(USART_GetFlagStatus(USART6,USART_FLAG_TC)==RESET);
//}


//void character_chassis_mode2(u8 Variable)
//{
//	determine_ID();//�жϷ�����ID�����Ӧ�Ŀͻ���ID
//	unsigned char character_demo[30]="HP_Chassis";
//	/*- ֡ͷ -*/	
//	character.Frame.SOF = 0xA5;
//	character.Frame.Seq = 0;	
//	character.Frame.DataLength = sizeof(character.ID) + sizeof(character.data);
//	/*- ID���� -*/
//  character.CmdID = 0x0301;//������
//	character.ID.cmd_ID = 0x0110;//��������ID 
//	//ID�Ѿ����Զ���ȡ����
//	character.ID.send_ID  = Judge_Self_ID;//�����ߵ�ID
//	character.ID.receiver_ID = Judge_SelfClient_ID;//�ͻ��˵�ID��ֻ��Ϊ�����߻����˶�Ӧ�Ŀͻ���
//	/*- �Զ�������һ -*/	
//	character.data.character_name[0] = 8;
//	character.data.character_name[1] = 0;
//	character.data.character_name[2] = 0;
//	character.data.operate_tpye = Variable;
//	character.data.graphic_tpye = 7;
//	character.data.layer = 2;
//	character.data.color = 5;
//	character.data.start_angle = 20;
//	character.data.end_angle = 5;
//	character.data.width = 2;
//	character.data.start_x = 30;
//	character.data.start_y = 760;
//  memcpy(character.data.data, &character_demo, sizeof(character_demo));
//	/*- �Զ�������һ -*/	
//	
//	memcpy(CliendTxBuffer_character,&character,sizeof(character));
//	Append_CRC8_Check_Sum(CliendTxBuffer_character,sizeof(character.Frame));//******
//	Append_CRC16_Check_Sum(CliendTxBuffer_character,sizeof(character));//*******
//	
//	//  /*- ���д�뷢�� -*/
//	for(int i = 0;i < sizeof(CliendTxBuffer_character);i++)
//	{
//		USART_SendData(USART6,CliendTxBuffer_character[i]);
//		while(USART_GetFlagStatus(USART6,USART_FLAG_TXE)==RESET);
//	}
//	while(USART_GetFlagStatus(USART6,USART_FLAG_TC)==RESET);
//}


//void character_Stronghold(u8 Variable)
//{
//	determine_ID();//�жϷ�����ID�����Ӧ�Ŀͻ���ID
//	unsigned char character_demo[30]="Stronghold";
//	/*- ֡ͷ -*/	
//	character.Frame.SOF = 0xA5;
//	character.Frame.Seq = 0;	
//	character.Frame.DataLength = sizeof(character.ID) + sizeof(character.data);
//	/*- ID���� -*/
//  character.CmdID = 0x0301;//������
//	character.ID.cmd_ID = 0x0110;//��������ID 
//	//ID�Ѿ����Զ���ȡ����
//	character.ID.send_ID  = Judge_Self_ID;//�����ߵ�ID
//	character.ID.receiver_ID = Judge_SelfClient_ID;//�ͻ��˵�ID��ֻ��Ϊ�����߻����˶�Ӧ�Ŀͻ���
//	/*- �Զ�������һ -*/	
//	character.data.character_name[0] = 9;
//	character.data.character_name[1] = 0;
//	character.data.character_name[2] = 0;
//	character.data.operate_tpye = Variable;
//	character.data.graphic_tpye = 7;
//	character.data.layer = 2;
//	character.data.color = 2;
//	character.data.start_angle = 20;
//	character.data.end_angle = 5;
//	character.data.width = 2;
//	character.data.start_x = 30;
//	character.data.start_y = 720;
//  memcpy(character.data.data, &character_demo, sizeof(character_demo));
//	/*- �Զ�������һ -*/	
//	
//	memcpy(CliendTxBuffer_character,&character,sizeof(character));
//	Append_CRC8_Check_Sum(CliendTxBuffer_character,sizeof(character.Frame));//******
//	Append_CRC16_Check_Sum(CliendTxBuffer_character,sizeof(character));//*******
//	
//	//  /*- ���д�뷢�� -*/
//	for(int i = 0;i < sizeof(CliendTxBuffer_character);i++)
//	{
//		USART_SendData(USART6,CliendTxBuffer_character[i]);
//		while(USART_GetFlagStatus(USART6,USART_FLAG_TXE)==RESET);
//	}
//	while(USART_GetFlagStatus(USART6,USART_FLAG_TC)==RESET);
//}


/**********�ַ�����**********/
/*******************************************���ݱ�д********************************************************/
int start_cnt = 0;
int SpinTop_Flag_cnt = 0;
int Vision_Flag_cnt = 0;
int Fire_Mode_cnt = 0;
int Chassis_Mode_cnt = 0;
int Stronghold_Flag_cnt = 0;
///***********************************ģʽ***********************************/
//void send_mode(void)
//{
//	if(start_cnt != 100)
//	{
//		graphic_2t(1);
//		graphic_3t(1);
//		number_t(1);
//		character_Cap_v(1);
//		character_Pitch_angle(1);
//		start_cnt++;
//		if(Fire_mode == 1)
//		{
//			character_fire_mode1(1);
//			character_fire_mode2(3);
//		}
//		if(Fire_mode == 0)
//		{
//			character_fire_mode2(1);
//			character_fire_mode1(3);
//		}
//		if(Chassis_mode == 1)
//		{
//			character_chassis_mode1(1);
//			character_chassis_mode2(3);
//		}
//		if(Chassis_mode == 0)
//		{
//			character_chassis_mode2(1);
//			character_chassis_mode1(3);
//		}
//		if(!Stronghold_flag)
//		{
//			character_Stronghold(1);
//		}
//		if(Stronghold_flag)
//		{
//			character_Stronghold(3);
//		}
//	}
//	else 
//	{
//		number_t(2);
//		if(SpinTop_Flag && SpinTop_Flag_cnt !=10)
//		{
//			character_SpinTop(1);
//			SpinTop_Flag_cnt++;
//		}
//		if(!SpinTop_Flag && SpinTop_Flag_cnt != 10)
//		{
//			character_SpinTop(3);
//			SpinTop_Flag_cnt++;
//		}
//		if(Vision_Flag && Vision_Flag_cnt != 10)
//		{
//			character_Vision(1);
//			Vision_Flag_cnt++;
//		}
//		if(!Vision_Flag && Vision_Flag_cnt != 10)
//		{
//			character_Vision(3);
//			Vision_Flag_cnt++;
//		}
//		if(Fire_mode == 1 && Fire_Mode_cnt != 10)
//		{
//			character_fire_mode2(3);
//			character_fire_mode1(1);
//			Fire_Mode_cnt++;
//		}
//		if(Fire_mode == 0 && Fire_Mode_cnt != 10)
//		{
//			character_fire_mode1(3);
//			character_fire_mode2(1);
//			Fire_Mode_cnt++;
//		}
//		if(Chassis_mode == 1 && Chassis_Mode_cnt != 10)
//		{
//			character_chassis_mode2(3);
//			character_chassis_mode1(1);
//			Chassis_Mode_cnt++;
//		}
//		if(Chassis_mode == 0 && Chassis_Mode_cnt != 10)
//		{
//			character_chassis_mode1(3);
//			character_chassis_mode2(1);
//			Chassis_Mode_cnt++;
//		}
//	  if(Stronghold_flag && Stronghold_Flag_cnt != 10)
//		{
//			character_Stronghold(3);
//			Stronghold_Flag_cnt++;
//		}		
//		if(!Stronghold_flag && Stronghold_Flag_cnt != 10)
//		{
//			character_Stronghold(1);
//			Stronghold_Flag_cnt++;
//		}
//	}
//}

//char tx2_data[3];
//void tx2_send()
//{
//	//  /*- ���д�뷢�� -*/
//	Color = is_red_or_blue();
//	tx2_data[0] = 0xA6;
//	if(Color == RED)
//		tx2_data[1] = 1;
//	else if(Color == BLUE)
//		tx2_data[1] = 2;
//	else tx2_data[1] = 0;
//	if(Rx_Visual[13] == tx2_data[1] && Rx_Visual[13] != 0)
//			tx2_data[2] = 1;
//	else tx2_data[2] = 0;
//	for(int i = 0;i < sizeof(tx2_data);i++)
//	{
//		USART_SendData(UART7,tx2_data[i]);
//		while(USART_GetFlagStatus(UART7,USART_FLAG_TXE)==RESET);
//	}
//	while(USART_GetFlagStatus(UART7,USART_FLAG_TC)==RESET);
//}

void FRT_USART_DATA_Ctrl(void *pvParameters)
{
	vTaskDelay(426);
	while(1)
	{
//		send_mode();
//		tx2_send();
//	  vTaskDelay(300);//������������		
	}
}
