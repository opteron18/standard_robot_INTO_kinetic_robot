#include "Ctrl_chassis.h"
#include "usart.h"
//#include "delay.h"
//#include "led.h"
#include "Configuration.h"
#include "PID.h"
#include "CAN_receive.h"
#include <math.h>
#include <stdlib.h>
#include "Higher_Class.h"
#include "remote_control.h"
#include "FreeRTOS.h"
#include "task.h"
#include "User_Api.h"
#include "referee.h"
#include "Ctrl_gimbal.h"
//#include "laser.h"
#include "math.h"
//#include "Usart_SendData.h"

#define CHASSIS_MAX_SPEED ((LIM_3510_SPEED/60)*2*PI*WHEEL_R/MOTOR_P)//��������ٶȣ�mm/s
SpinTop_t SpinTop={0};

/*********************************************************************
 *ʵ�ֺ�����void virtual_encoders(int16_t aim[4])
 *��    �ܣ�������������
 *��    �룺������Ŀ��ֵ
 *˵    ����
 ********************************************************************/
int16_t Encoders[4];		//��������ֵ
void virtual_encoders(int16_t aim[4])
{
	uint8_t i;
	for(i=0; i<4; i++)
	{
		Encoders[i] += aim[i];
		if (Encoders[i]>8191) Encoders[i] -= 8192;
		else if (Encoders[i]<0) Encoders[i] += 8192;
	}
}


/*********************************************************************
 *ʵ�ֺ�����int encoders_err(int8_t i,int error)
 *��    �ܣ�����ʵ��Ŀ������ �� Ŀ���������̲� ֵ
 *��    �룺���ʶ��������ڲ�ֵ
 *��    �أ�ʵ��Ŀ������ �� Ŀ���������̲� ֵ
 *˵    ����
 ********************************************************************/
int encoders_err(int8_t i,int error)
{
	int temp;
	
	temp = motor_data[i].NowAngle + error;
	temp = temp%8192;	
	if (temp<0) temp += 8192;						//ʵ��Ŀ������ֵ
	temp = Encoders[i] - temp;
	if (temp<-3000) temp += 8192;
	else if (temp>3000) temp -= 8192;
	
	return temp;
}


/***************************
 *���̵���ٶȿ���
 *���룺4�������Ŀ���ٶ�
 ***************************/
static void motor_speed_ctrl(int16_t M1_speed,int16_t M2_speed,int16_t M3_speed,int16_t M4_speed)
{
	u8 i;
	int16_t target_speed[4];
	
	target_speed[0]=M1_speed;	target_speed[1]=M2_speed;
	target_speed[2]=M3_speed;	target_speed[3]=M4_speed;
	
	for(i=0;i<4;i++)
		PID_Control(&motor_speed_pid[i],target_speed[i],motor_data[i].ActualSpeed);
}


/***************************
 *���̵����ֵ���ƣ�λ�ñջ�v2.0
 *���룺4�������ÿ����Ŀ���ٶ�
 *�������̲�ֵ������Ŀ���ٶ��ϣ�����������̲�ֵ��pid�����ַ�����Ӧ���ܸ��ø�ƽ��
 ***************************/
static M_pid angle_pid[4];		//���pid�ṹ��
void motor_angle_ctrl(int16_t M1_speed,int16_t M2_speed,int16_t M3_speed,int16_t M4_speed)
{
	u8 i;
	float P,D;
	int error,temp;
	
	int16_t target_speed[4];
	int16_t target_angle[4];
	
	target_speed[0]=M1_speed;	target_speed[1]=M2_speed;
	target_speed[2]=M3_speed;	target_speed[3]=M4_speed;
	
	for(i=0;i<4;i++)//ת��Ϊÿ����Ŀ����ֵ
		target_angle[i] = (((float)target_speed[i]/ 60) * 8192)/PID_Hz;
	
	if(Encoders[0]==0)
	{
		if(Encoders[1]==0)
		{
			Encoders[0] = motor_data[0].NowAngle;
			Encoders[1] = motor_data[1].NowAngle;
			Encoders[2] = motor_data[2].NowAngle;
			Encoders[3] = motor_data[3].NowAngle;
		}
	}
	virtual_encoders(target_angle);	//������������ֵ
	
	for (i=0; i<4; i++)
	{
		P=0; D=0;									//�м�ֵ����
		error = angle_pid[i].old_aim - motor_data[i].D_Angle;	//��һ��δ��ɲ�ֵ
		error += encoders_err(i,error);					//�ۼ�ʵ������ֵ����������ֵ�Ĳ�
		angle_pid[i].old_aim = error + target_angle[i];					//���¾�Ŀ��ֵ
		
		//**********P��**************
		P = PID_MOTOR_ANGLE_KP * error;
		//**********D��**************
		D = (error - angle_pid[i].old_err) * PID_MOTOR_ANGLE_KD;
		angle_pid[i].old_err = error;
		temp = P - D;
		if (temp > LIM_3508_SPEED) angle_pid[i].output = LIM_3508_SPEED;			//Ŀ��ת�ٲ�������޷�
		else if(temp < -LIM_3508_SPEED) angle_pid[i].output = -LIM_3508_SPEED;
		else angle_pid[i].output = temp;
		if ((angle_pid[i].output<70) && (angle_pid[i].output>-70)) angle_pid[i].output = 0;
		
		//���뵽�ٶȻ�
		temp = angle_pid[i].output + target_speed[i];	//�������Ŀ���ٶ�
		PID_Control(&motor_speed_pid[i],temp,motor_data[i].ActualSpeed);
	}
}


/***************************
 *���˶�ѧ����
 *�Ƶ��̵�ˮƽ�ٶȡ����ٶ�
 *����Ϊ���̵��˶�Ŀ�꣬�ֱ�ΪVx��Vy��Wv����λmm/s,rad/s,ǰ��ΪVx
 *������ǰ��Ϊw0����ʱ��˳��
 ***************************/
#ifdef CHASSIS_POSITION_CRTL
//#define ANGLE_CTRL		//ʹ��λ�û�
#endif

//void Spintop_handle(void)
//{
//	SpinTop.Angle  = (motor_data[4].NowAngle - Yaw_Mid_encode + 8192) % 8192 / 22.7555556f;
//		if(SpinTop.Angle > 360)
//			SpinTop.Angle = (SpinTop.Angle - 360) * 0.0174532f;
//		else
//			SpinTop.Angle *= 0.0174532f;
//	

//		SpinTop.Vx = Control_data.Vx * cos(SpinTop.Angle) + Control_data.Vy * sin(SpinTop.Angle);
//		SpinTop.Vy = -Control_data.Vx * sin(SpinTop.Angle) + Control_data.Vy * cos(SpinTop.Angle);
//	
//}


static void Inverse_Kinematic_Ctrl(float Vx,float Vy,float Wz)
{
	float w[4];		//�ĸ����ӵ�ʵ��ת��rad/s
	int16_t n[4];	//ת��Ϊ�����ٶȵ��ĸ������ת��
	uint8_t i=0;
	
	SpinTop.Angle  = (motor_data[4].NowAngle - Yaw_Mid_encode + 8192) % 8192 / 22.7555556f;
	if(SpinTop.Angle > 360)
		SpinTop.Angle = (SpinTop.Angle - 360) * 0.0174532f;
	else
		SpinTop.Angle *= 0.0174532f;
	
	SpinTop.Vx = Control_data.Vx * cos(SpinTop.Angle) + Control_data.Vy * sin(SpinTop.Angle);
	SpinTop.Vy = -Control_data.Vx * sin(SpinTop.Angle) + Control_data.Vy * cos(SpinTop.Angle);
		
	w[0] = (SpinTop.Vy - SpinTop.Vx +CHASSIS_K*Wz)/WHEEL_R*speed_zoom;    //speed_zoom ��������ϵ��  ��ǰ��
	w[1] = (SpinTop.Vy + SpinTop.Vx + CHASSIS_K*Wz)/WHEEL_R*speed_zoom;   //��ǰ
	w[2] = (-SpinTop.Vy + SpinTop.Vx + CHASSIS_K*Wz)/WHEEL_R*speed_zoom;  //���
	w[3] = (-SpinTop.Vy - SpinTop.Vx + CHASSIS_K*Wz)/WHEEL_R*speed_zoom;  //�Һ�

	for(i=0;i<4;i++)
	  n[i] = ((float)w[i]*MOTOR_P / (2*PI)) * 60;	//ת��Ϊ��������ٶ�

	//����б���ߵ��ٶ�
	for(i=0;i<4;i++)
	{
		if(n[i] > LIM_3508_SPEED)
		{
			uint8_t j=0;
			float temp = (float)n[i] / LIM_3508_SPEED;	//����
			
			for(j=0;j<4;j++)	n[j] = (float)n[j]/temp;		//�ȱ�����С
		}
		else if(n[i] < -LIM_3508_SPEED) 
		{
			uint8_t j=0;
			float temp = -(float)n[i] / LIM_3508_SPEED;	//����
			
			for(j=0;j<4;j++)	n[j] = (float)n[j]/temp;		//�ȱ�����С
		}
	}
	motor_speed_ctrl(n[0],n[1],n[2],n[3]);
	CAN1_SendCommand_chassis(motor_speed_pid[0].Control_OutPut,motor_speed_pid[1].Control_OutPut,
	                         motor_speed_pid[2].Control_OutPut,motor_speed_pid[3].Control_OutPut);
}


void FRT_Inverse_Kinematic_Ctrl(void const *pvParameters)//���̿�������
{
	vTaskDelay(357);
	while(1)
	{ 
		if (RC_Ctl.rc.s2 == RC_SW_DOWN)
		{
	    Power_off_function();		
		}
		else
		{
		  Chassis_Power_Limit(); //ͨ���޸�б������б�ʣ��Լ���������ϵ�����ﵽЧ��
	    chassis_control_acquisition();//���̿�������ȡ��״̬������
		  chassis_set_contorl();//���̿���������
		  Inverse_Kinematic_Ctrl(Control_data.Vx,Control_data.Vy,Control_data.Chassis_Wz);//���̿��ƽӿ�
		}
	  vTaskDelay(2);
	}
}

