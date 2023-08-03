///*************************************************************************
// *�߼�����ʵ��
// *���ƹ���
// *Ť��ģʽ
// *��̨����ģʽ
// *�Ӿ��Զ���׼ģʽ
// *
// *************************************************************************/
#include "Higher_Class.h"
#include "User_Api.h"
#include "Ctrl_gimbal.h"
#include "IMU.h"
#include "INS_task.h"
#include "Configuration.h"
//#include "led.h"
//#include "buzzer.h"
#include "remote_control.h"
#include "CAN_receive.h"
//#include "can2.h"
#include "Ctrl_chassis.h"
#include "referee.h"
#include <math.h>
#include <stdlib.h>
#include "PID.h"
#include "Kalman.h"
#include "Ctrl_shoot.h"
//#include "pwm.h"
//#include "laser.h"
#include "FreeRTOS.h"
#include "task.h"

int yawcnt = 0;  //���ڵ����˶�����
float imu_yaw1 = 0;  //���ڵ����˶�����

float Twist_buff;  //Ť����ű���
float SpinTop_buff;  //С���ݴ�ű���
float SpinTop_timecount=0;  //С���������ֱ���ʱ��ֹ������

//*************�������Ʊ�����*************
u8 PowerLimit_Flag = 0;
int16_t Can_Msg_Power;
float speed_zoom_double = 0;
float speed_zoom_init=0.8;  //35w��ʱ�������ʼ�ٶ�ϵ��Ϊ0.8
float speed_zoom_initmin = 0.27f;
int power_send_flag = 0;
int power_send_flag0 = 0;
int power_send_flag1 = 0;
int power_send_flag2 = 0;
int power_send_flag3 = 0;
int hp_send_flag = 0;
int hp_send_flag1 = 0;
int hp_send_flag2 = 0;
int hp_send_flag3 = 0;
int Chassis_Maxspeed_Level = 1;
int Chassis_Maxspeed_Level_last = 1;
int Chassis_Maxspeed_Warning_Flag1 = 0;
int Chassis_Maxspeed_Warning_Flag2 = 0;
int Chassis_Maxspeed_Warning_Flag3 = 0;
float MAX_VX = Vx_MAX_init;
float MAX_VY = Vy_MAX_init;
float MAX_WZ = Wz_MAX_init;

//******************�Ӿ�����******************
float vision_yaw_angle_target = 0;
float vision_yaw_angle_target_last = 0;
float vision_pitch_angle_target = 0;
float vision_pitch_angle_target_last = 0;
float vision_run_time = 0;
float vision_run_fps = 0;
float armor_speed = 0;
float armor_speed_last = 0;
float armor_angle = 0;
float armor_angle_forcast = 0;
float armor_angle_forcast1 = 0;

/**
  * @brief  ��ȡ��ֵ�ľ���ֵ
  * @param  value
  * @retval ����ֵ
  */
float Func_Abs(float value)
{
	if(value >= 0)
		return value;
	else 
		return -value;	
}


void chassis_set_contorl(void)//���̿���������
{  
	if (SpinTop_Flag)
	{	
		SpinTop_timecount += 0.0016f;  //����С���ݻ���Ĺ���
		VAL_LIMIT(SpinTop_timecount, 0, 1);  // ��������1  
		SpinTop_buff =  SpinTop_SPEED * SpinTop_timecount * SpinTop_Direction;  //���Ի�һ��С���ݵķ��� ʵ�ڲ��а�С���ݵ����ȼ����ɲ������ȼ�
		Control_data.Chassis_Wz = SpinTop_buff ;
		speed_zoom_double = 2.0f;
	} 
	else 
	{  
		SpinTop_timecount = 0;
    speed_zoom_double = 2.0f;
		
//		if(Twist_Flag)//Ť��
//		{
//			Encoder_angle_Handle();//���̽Ƕ�ת��
//			if(Yaw_Encode_Angle>=10)	//��̨�ڵ����ұ�
//			{
//				Twist_buff = -TWIST_SPEED;
//			}
//			else if(Yaw_Encode_Angle<=-10)	//��̨�ڵ������
//			{
//				Twist_buff = TWIST_SPEED;
//			}
//			Control_data.Chassis_Wz = Twist_buff;
//		}
		
    if (Gimbal_180_flag)  //һ��180
	    Control_data.Chassis_Wz = -8.3f;  //-8.3f
			
	  if (Chassismode_flag)//������̨������ģʽ���̲���
	  {
			if(imu.yaw > 170 | imu.yaw <-170)
			{ 
				yawcnt = 0;
				return;
			}
			if (!(RC_Ctl.rc.ch2-1024) && !RC_Ctl.mouse.x)	
			{
				if(!yawcnt)
				{
					imu_yaw1 =  imu.yaw;
					yawcnt++;
				}		
					Control_data.Chassis_Wz = Control_data.Chassis_Wz*0.8f + ( imu.yaw - imu_yaw1)*0.1f;  //0.2  0.1
			}
	    else
	      yawcnt = 0;	
	  }			
		else//��̨���̷���
		{
			if (Fire_Loading_Flag || Fire_Loading_End_Flag)
			{
				Control_data.Vx = 0;
				Control_data.Vy = 0;
				Control_data.Chassis_Wz = 0;
			}
			else
			{
				if (RC_Ctl.rc.s2 == RC_SW_MID)//ң����ģʽ���̿�������
				{	
					Control_data.Chassis_Wz = Control_data.Chassis_Wz*0.1f + (-Yaw_Encode_Angle) * 0.15f;  //0.2  0.1   0.27   0.1
				}
				if (RC_Ctl.rc.s2 == RC_SW_UP)//����ģʽ���̿�������
				{
					if (Vision_Flag)
						Control_data.Chassis_Wz = Control_data.Chassis_Wz*0.1f + (-Yaw_Encode_Angle) * 0.06f; 
					else
						Control_data.Chassis_Wz = Control_data.Chassis_Wz*0.1f + (-Yaw_Encode_Angle) * 0.15f;   
				}   
		  }			
		}
  }
} 


void gimbal_set_contorl(void)
{		
	if (Vision_Flag)
	{
		if (!Vision_Data.target_lose)
		{
			armor_speed = -gryoYaw + (vision_yaw_angle_target - vision_yaw_angle_target_last) * 1000 / (vision_run_fps + 0.001f);
			armor_speed = KalmanFilter(&Vision_kalman_x, armor_speed);
			armor_speed = KalmanFilter(&Vision_kalman_x1, armor_speed);		
	
			armor_angle_forcast1 = vision_yaw_angle_target + angleYaw + armor_speed * vision_run_time
                       			+ 0.5f * (armor_speed - armor_speed_last) * vision_run_time * vision_run_time;
			armor_angle_forcast = KalmanFilter2(&Vision_kalman2_x1, armor_angle_forcast1, armor_speed);
			armor_speed_last = armor_speed;
		}
		else
		{
			Kalman_Reset(&Vision_kalman_x);
			Kalman_Reset(&Vision_kalman_x1);
			Kalman_Reset2(&Vision_kalman2_x1);	
			Kalman_Reset2(&Vision_kalman2_x);
      armor_speed = 0;	
		}
	}

	else if (Gimbal_180_flag)
	  Control_data.Gimbal_Wz = -0.45f;  //-0.25f
	
	else if (Fire_Loading_Flag && !Fire_Loading_End_Flag)
	{
		Control_data.Gimbal_Wz = -0.35f;
		if (motor_data[4].NowAngle > 2650)
			Control_data.Gimbal_Wz = 0;
	}
	
	else if (Fire_Loading_End_Flag)
	{
		Control_data.Gimbal_Wz = 0.35f;
		if (motor_data[4].NowAngle < 750)
		{
			Fire_Loading_End_Flag = 0;
			Fire_Loading_Flag = 0;
		}
	}
			
	else//���̸�����̨
	{
		if (RC_Ctl.rc.s2 == RC_SW_MID)//ң����ģʽ��̨��������
		{
			if ((20 < Yaw_Encode_Angle && Control_data.Gimbal_Wz < 0)| (Yaw_Encode_Angle < -20 && Control_data.Gimbal_Wz >0))
				Control_data.Gimbal_Wz = Control_data.Gimbal_Wz*0.6f;  //0.1f  0.6
			else 
				Control_data.Gimbal_Wz = Control_data.Gimbal_Wz*0.6f;	   //2   0.7
		}			
	  else if (RC_Ctl.rc.s2 == RC_SW_UP)//����ģʽ��̨��������  
		{
			if( (50 < Yaw_Encode_Angle && Control_data.Gimbal_Wz < 0)| (Yaw_Encode_Angle < -50 && Control_data.Gimbal_Wz >0))
			  Control_data.Gimbal_Wz = Control_data.Gimbal_Wz*0.7f;    //0.1f   //1.7    //1.2
			else 
			  Control_data.Gimbal_Wz = Control_data.Gimbal_Wz*0.7f;       //2    //1.2
    }		
		else
		  Control_data.Gimbal_Wz = 0.015f*RC_Ctl.mouse.x;    //С���ݿ���ʱ����׼�ٶ�    //0.027
	}
	
	if(Control_data.Gimbal_Wz > 0.5f)//��̨�������޷�   //0.45f     0.6
		Control_data.Gimbal_Wz = 0.5f;                     //0.45f
	if(Control_data.Gimbal_Wz < -0.5f)                  //0.45f
		Control_data.Gimbal_Wz = -0.5f;                    //0.45f
}


void Power_off_function(void)//�ϵ籣��
{
	CAN1_SendCommand_chassis(0,0,0,0);	
	CAN1_Send_Msg_gimbal(0,0,0);
	CAN2_Send_Msg_gimbal(0);
	speed_17mm_level_Start_Flag = 0;
	friction_wheel_ramp_function();
	Control_data.Vx = 0; 
	Control_data.Vy = 0; 
	Control_data.Chassis_Wz = 0;
	Control_data.Pitch_angle = 0;
	Control_data.Gimbal_Wz = 0;
	speed_17mm_level_Start_Flag = 0;
	Twist_Flag = 0;
	Vision_Flag = 0;
	Gimbal_180_flag = 0;
	SafeHeatflag = 1;
	block_flag = 0;
	Stronghold_flag = 0;
	offline_flag = 0;
	error = 0;
	Yaw_AnglePid_i = 0;
	T_yaw = 0;
	P1 = 0;
	I1 = 0;
	D1 = 0;
	angle_output1 = 0;
	Control_data.Gimbal_Wz = 0;
	E_yaw = imu.yaw;
	Chassismode_flag = 0;
//	PWM_Write(PWM2_CH3,0);
	laser_off();			
}


void Cap_limit_Decision(void)
{
	if(Chassis_mode)
	{	
		if (PowerData[1] > 21)
			speed_zoom_initmin = 0.52f;
		else if (PowerData[1] > 19 && PowerData[1] < 21)
			speed_zoom_initmin = 0.43f;
		else
			speed_zoom_initmin = 0.37f;
  }
	else
  {
		if (PowerData[1] > 21)
		  speed_zoom_initmin = 0.39f;
		else if (PowerData[1] > 19 && PowerData[1] < 21)
			speed_zoom_initmin = 0.37f;
		else
			speed_zoom_initmin = 0.32f;
  }
}


//�ú���������ѹ���ʵ����ó�ʼ���ٶ�ϵ����������Ч��ֹ�������͵���ǰ�ڵ��ݵ����ĵ�̫��
//���ͬʱ��ֹcanͨ���жϵ��µ��̲��������ƶ������ݰ���MCUͨ��ʧ��ǿ�������ٶ�
void Powerlimit_Decision(void)
{
	//////////////////Ѫ������///////////////////////////////////////////////////
	if(PowerData[3] == Hp_Level_1)//�����ӵ�45w��ѹ
	{
		if(SpinTop_Flag)
			speed_zoom_init = 0.35f;
		else
		  speed_zoom_init = 0.41f;
	}
	else if(PowerData[3] == Hp_Level_2)//�����ӵ�50w��ѹ
	{
		if(SpinTop_Flag)
			speed_zoom_init = 0.37f;
		else
		  speed_zoom_init = 0.45f;
	}
	else if(PowerData[3] == Hp_Level_3)//55w����ͳһ��ʼϵ��1��������Ҫ���Ϳ����ټӸ��ж�
	{
		if(SpinTop_Flag)
			speed_zoom_init = 0.37f;
		else
		  speed_zoom_init = 0.53f;
	}
	///////////////////��������//////////////////////////////////////////////////
	else if(PowerData[3] == P_Level_1)//
	{
		if(SpinTop_Flag)
			speed_zoom_init = 0.35f;
		else
		  speed_zoom_init = 0.43f;
	}	
	else if(PowerData[3] == P_Level_2)//
	{
		if(SpinTop_Flag)
			speed_zoom_init = 0.38f;
		else
		  speed_zoom_init = 0.55f;
	}
	else if(PowerData[3] == P_Level_3)//
	{
		if(SpinTop_Flag)
			speed_zoom_init = 0.42f;
		else
		  speed_zoom_init =0.60f;
	}
	else  //canͨ��ʧ��ǿ���޳�ʼ�ٶ�,�����ǿ���ͨ��shift���ټ�������ջ�
	  speed_zoom_init = 0.42f;
}


//ͨ����ͬ�ȼ���ͬ��ѹ������޸�������ٶ�
//ǿ�����ٱ�֤���ݵ�ѹ��������ٶ�ϵ���޹�
void Chassis_Maxspeed_ctrl(void)
{
	if (Chassis_mode)
	{	
		if (PowerData[1] > 22 && !Chassis_Maxspeed_Warning_Flag1)
		{
		  MAX_VX = 5200;
		  MAX_VY = 4700;
			Chassis_Maxspeed_Level = 1;
		}
		else if ((PowerData[1] > 22 && Chassis_Maxspeed_Warning_Flag1) 
			    || (PowerData[1] > 19 && PowerData[1] <= 22 && !Chassis_Maxspeed_Warning_Flag2))
		{
		  MAX_VX = 5000;
		  MAX_VY = 4500;
			Chassis_Maxspeed_Level = 2;
		}
		else if ((PowerData[1] > 19 && PowerData[1] <= 22 && Chassis_Maxspeed_Warning_Flag2) 
			    || (PowerData[1] > 17 && PowerData[1] <= 19 && !Chassis_Maxspeed_Warning_Flag3))
		{
		  MAX_VX = 4500;
		  MAX_VY = 4000;
			Chassis_Maxspeed_Level = 3;
		}
		else if ((PowerData[1] > 17 && PowerData[1] <= 19 && Chassis_Maxspeed_Warning_Flag3) 
			    || (PowerData[1] <= 17))
		{
      MAX_VX = 4000;
		  MAX_VY = 3500;
		}
    
    if (Chassis_Maxspeed_Level == 1 && Chassis_Maxspeed_Level_last == 2)
			Chassis_Maxspeed_Warning_Flag1 = 1;
		if (PowerData[1] > 22.5f)
			Chassis_Maxspeed_Warning_Flag1 = 0;
		
	  if (Chassis_Maxspeed_Level == 2 && Chassis_Maxspeed_Level_last == 3)
			Chassis_Maxspeed_Warning_Flag2 = 1;
		if (PowerData[1] > 20.5f)
			Chassis_Maxspeed_Warning_Flag2 = 0;

	  if (Chassis_Maxspeed_Level == 3 && Chassis_Maxspeed_Level_last == 4)
			Chassis_Maxspeed_Warning_Flag3 = 1;
		if (PowerData[1] > 18.0f)
			Chassis_Maxspeed_Warning_Flag3 = 0;
		
		Chassis_Maxspeed_Level_last = Chassis_Maxspeed_Level;
  }
	else //Ѫ������
  {
		if (PowerData[1] > 22)
		{
		  MAX_VX = 5500;
		  MAX_VY = 5500;
		}
		else if (PowerData[1] > 19 && PowerData[1] < 22)
		{
		  MAX_VX = 5000;
		  MAX_VY = 5000;
		}
		else if (PowerData[1] > 17 && PowerData[1] < 19)
		{
	  	MAX_VX = 4500;
		  MAX_VY = 4500;
		}
		else
		{
      MAX_VX = 4000;
		  MAX_VY = 4000;
		}			
  }
}


////���ʱջ�ʡ������2021
//void Chassis_Power_Limit(void)
//{
//  Powerlimit_Decision(); //���޸�Ramp_K�������������ٶȣ���б��б��
//	Power_Limit_pid.Control_OutPut = PID_Control(&Power_Limit_pid, 20, PowerData[1]);  //�Ե��ݵ�ѹֵ���������ϵ��
//	if(PowerLimit_Flag == 1) //���ʱջ��ֶ������
//	{  
//		if(PowerData[1] <= 16.5f)
//			speed_zoom = 0.70f;
//		else
//			speed_zoom = 1.5f;//shift�����٣�ʵ���Ǹ����ٶ��޷����ֵ
//	}
//	else 
//	{
//		speed_zoom = speed_zoom_init - Power_Limit_pid.Control_OutPut * speed_zoom_double;  //����޷�Ϊ5		
//		VAL_LIMIT(speed_zoom, 0.4f, speed_zoom_init);
//	}
//	speed_zoom = 0.5f;
//}


//�ɹ���ģ�鷽��2021
void Chassis_Power_Limit(void)
{
	Chassis_Maxspeed_ctrl();
	Powerlimit_Decision();  //���޸�Ramp_K�������������ٶȣ���б��б��
	Cap_limit_Decision();
	Power_limit_PIDloop(&Power_Limit_pid, 22, PowerData[1]);  //�Ե��ݵ�ѹֵ���������ϵ��
	if(PowerLimit_Flag == 1)  //���ʱջ��ֶ������(PowerData[1] > 19)
	{  
		speed_zoom = speed_zoom_initmin + 0.55f;//shift�����٣�ʵ���Ǹ����ٶ��޷����ֵ
	}
	else 
	{
		speed_zoom =speed_zoom_init-Power_Limit_pid.Control_OutPut * 1.5f;  //����޷�Ϊ5	 
		VAL_LIMIT(speed_zoom,speed_zoom_initmin,speed_zoom_init);
	}
}


////���ʼ�ģ��ջ�����2021
//void Chassis_Power_Limit(void)
//{
//	Powerlimit_Decision(); //���޸�Ramp_K�������������ٶȣ���б��б��
//	Power_limit_PIDloop(&Power_Limit_pid, PowerData[3] - 20, Real_power);  //�Ե��ݵ�ѹֵ���������ϵ��
//	if(PowerLimit_Flag == 1) //���ʱջ��ֶ������(PowerData[1] > 19)
//	{  
//		speed_zoom = 1.2f;//shift�����٣�ʵ���Ǹ����ٶ��޷����ֵ
//	}
//	else 
//	{
//		speed_zoom = speed_zoom_init + Power_Limit_pid.Control_OutPut ;  //����޷�Ϊ5	 
//		VAL_LIMIT(speed_zoom,0.43f,speed_zoom_init);//���Ʋ�ͬ���ʵȼ�������ٶ��Լ���ʼ�ٶ�
//	}
//}


//��ϳ������ݰ�
//���룺�趨��ѹ���ʣ�35w����130w��
//�����ú���ʱ���ݰ��ϵ�����Ĭ��35w
void Powerlimit_Ctrl(int16_t Target_power)
{			
	if (Chassis_mode)
	{
		hp_send_flag = 0;
		hp_send_flag1 = 0;
		hp_send_flag2 = 0;
		hp_send_flag3 = 0;
	}
	else
	{
		power_send_flag = 0;
		power_send_flag1 = 0;
		power_send_flag2 = 0;
		power_send_flag3 = 0;		
	}
	
	if(Game_robot_state.robot_level == 3 && Chassis_mode)
	{
		power_send_flag3++;
		if(power_send_flag3 < 5)
		{
			Can_Msg_Power = Target_power * 100;
			CAN1_SendCommand_Powerlimit(Can_Msg_Power);
		}
	  else
		  power_send_flag3 = 5;
	}
	else if(Game_robot_state.robot_level == 2 && Chassis_mode)
	{
		power_send_flag2++;
		if(power_send_flag2 < 5)
		{
			Can_Msg_Power = Target_power * 100;
			CAN1_SendCommand_Powerlimit(Can_Msg_Power);
		}
	  else
		  power_send_flag2 = 5;
	}
	else if(Game_robot_state.robot_level == 1 && Chassis_mode)
	{
		power_send_flag1++;
		if(power_send_flag1 < 5)
		{
			Can_Msg_Power = Target_power * 100;
			CAN1_SendCommand_Powerlimit(Can_Msg_Power);
		}
	  else
		  power_send_flag1 = 5;
	}		
	else if(offline_flag == 2000 && Chassis_mode)
	{
		power_send_flag++;
		if(power_send_flag < 5)
		{
			Can_Msg_Power = 44 * 100;
			CAN1_SendCommand_Powerlimit(Can_Msg_Power);
		}
	  else
		  power_send_flag = 5;
	}
	if(Game_robot_state.robot_level == 3 && !Chassis_mode)
	{
		hp_send_flag3++;
		if (hp_send_flag3 < 5)
		{
			Can_Msg_Power = Target_power * 100;
			CAN1_SendCommand_Powerlimit(Can_Msg_Power);
		}
	  else
		  hp_send_flag3 = 5;
	}
	else if(Game_robot_state.robot_level == 2 && !Chassis_mode)
	{
		hp_send_flag2++;
		if(hp_send_flag2 < 5)
		{
			Can_Msg_Power = Target_power * 100;
			CAN1_SendCommand_Powerlimit(Can_Msg_Power);
		}
	  else
		  hp_send_flag2 = 5;
	}
	else if(Game_robot_state.robot_level == 1 && !Chassis_mode)
	{
		hp_send_flag1++;
		if(hp_send_flag1 < 5)
		{
			Can_Msg_Power = Target_power * 100;
			CAN1_SendCommand_Powerlimit(Can_Msg_Power);
		}
	  else
		  hp_send_flag1 = 5;
	}		
	else if(offline_flag == 2000 && !Chassis_mode)
	{
		hp_send_flag++;
		if(hp_send_flag < 5)
		{
			Can_Msg_Power = 44 * 100;
			CAN1_SendCommand_Powerlimit(Can_Msg_Power);
		}
	  else
		  hp_send_flag = 5;
	}
	else 
	{
		power_send_flag0++;
		if(power_send_flag0 < 5)
		{
			Can_Msg_Power = Target_power * 100;
			CAN1_SendCommand_Powerlimit(Can_Msg_Power);
		}
	  else
		  power_send_flag0 = 5;
	}		
}
