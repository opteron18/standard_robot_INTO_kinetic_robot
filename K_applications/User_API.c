#include "Kalman.h"
#include "User_Api.h"
#include "Ctrl_gimbal.h"
#include "Ctrl_chassis.h"
#include "IMU.h"
#include "INS_task.h"
#include "Configuration.h"
//#include "led.h"
//#include "buzzer.h"
#include "remote_control.h"
#include "referee.h"
#include "Higher_Class.h"
#include "CAN_receive.h"
#include "PID.h"
//#include "pwm.h"
#include "shoot.h"
#include "Ctrl_shoot.h"
#include "FreeRTOS.h"
#include "task.h"
#include "math.h"
#include "Usart_SendData.h"
#include "bsp_laser.h"

#define DITHERING_TIMNE 150 

//״̬��
u8 SpinTop_Flag =0;             //С���ݱ�־
u8 Twist_Flag = 0;			        //Ť����־
u8 Vision_Flag = 0;			        //�Ӿ������־
u8 Gimbal_180_flag = 0;         //180�ȱ�־
u8 Stronghold_flag = 0;         //�������Ʊ�־
u8 Inverse_flag = 0;            //���ַ�ת��־
u8 speed_17mm_level_Start_Flag = 0;  //Ħ���ֿ�����־
u8 Fire_Loading_Flag = 0;  //������־λ
u8 Fire_Loading_End_Flag = 0;   //����������־λ
int Chassismode_flag = 0;       //����ģʽ��־λ
int Gimbalmode_flag = 0;        //��̨ģʽ��־λ

//����
u8 TwistCnt,Inverse_cnt,Stronghold_resist_cnt,SpinTop_cnt,SpinTop_close_cnt;
u8 speed_17mm_level_Close_Cnt,speed_17mm_level_Start_Cnt;
u8 Shoot_mode_cnt,Chassis_mode_cnt,Fire_Loading_Cnt,Fire_Loading_End_Cnt;

//�������
int16_t Target_chassis_power = 0;
int32_t VerticalCnt = 0;
int32_t HorizontalCnt = 0;
float Ramp_K = 1.5f;  
float Wz_increasing = 0;
float PC_Speed_Vx =0;
float PC_Speed_Vy =0;
float speed_zoom = 1.0f ; //ң�������Ƶ��ٶ�
u8 Chassis_mode = Chassis_mode_init;

//�������
int Heatmax = 0;  //�������
int Shootnumber = 0;  //�ɷ����ӵ�����
int SafeHeatflag = 1;  //��ȫ������־
int Shootnumber_fired = 0;  //�ѷ����ӵ���
int cycle_time = 0;  //һ����������ʱ��
float Shootnumber_limit = 4;

ControlDATA_TypeDef Control_data;//��������


void chassis_control_acquisition(void)
{
	if (RC_Ctl.rc.s2 == RC_SW_MID)//ң�ؿ���
	{
		Control_data.Vx = (RC_Ctl.rc.ch1 - 1024) * 6.2;
		Control_data.Vy = (RC_Ctl.rc.ch0 - 1024) * 6.2;
		Control_data.Chassis_Wz = (RC_Ctl.rc.ch2 - 1024) * 0.035f;    
		Chassismode_flag = 0;
		offline_flag++;//����ϵͳ���߼���
		if(offline_flag >= 2000)
			offline_flag = 2000;
	}
	else //���Կ���
	{  
		//����ֵ����б��
		if(RC_Ctl.key.Vertical)
		{ 
			VerticalCnt++;
			PC_Speed_Vx = Ramp_K * sqrt(5000 * VerticalCnt);  //�������������ٶ��𽥼�С�ļ���б��
		}  
		else 
		{
			PC_Speed_Vx =0;
			VerticalCnt=0;
		}
		if (RC_Ctl.key.Horizontal)
		{   
			 HorizontalCnt++;
			 PC_Speed_Vy = Ramp_K*sqrt(4000*HorizontalCnt);//б�������й�  //3000
		}	 
		else 
		{
			 PC_Speed_Vy=0;
			 HorizontalCnt=0;
		}
		Control_data.Vx = RC_Ctl.key.Vertical * PC_Speed_Vx;
		Control_data.Vy = RC_Ctl.key.Horizontal * PC_Speed_Vy;
	 
		//��תб��
		if(RC_Ctl.mouse.x != 0) 
			Wz_increasing = 0.015f;    
		else
			Wz_increasing = 0;
	 
		if(Wz_increasing >= 2.3f)   
			Wz_increasing = 2.3f;       
		Control_data.Chassis_Wz = Wz_increasing * RC_Ctl.mouse.x;

		//������ٶ��޷�
		VAL_LIMIT(Control_data.Vx, -MAX_VX, MAX_VX);  
		VAL_LIMIT(Control_data.Vy, -MAX_VY, MAX_VY);
		VAL_LIMIT(Control_data.Chassis_Wz, -MAX_WZ, MAX_WZ);
		
		if(RC_Ctl.rc.s2 == RC_SW_UP && RC_Ctl.rc.s1 == RC_SW_UP)//��̨���̷���Ͳ�����ģʽ�л�
		{
			Chassismode_flag = 1;//��̨���̲�����ģʽ	
		}
		else
		{
			yawcnt = 0;
			Chassismode_flag = 0;//��̨���̷���ģʽ
		}
  }
}


void gimbal_control_acquisition(void)
{
	if (RC_Ctl.rc.s2 == RC_SW_MID)//ң�ؿ���
	{
		Control_data.Gimbal_Wz = (RC_Ctl.rc.ch2 - 1024) * 0.001f;  
		Control_data.Pitch_angle += (RC_Ctl.rc.ch3 - 1024) * 0.0003f;
	
		Control_on_off_friction_wheel();//Ħ���ֿ��غ���
		friction_wheel_ramp_function();//Ħ���ֿ��ƺ���
	
		if (RC_Ctl.rc.ch4 !=1024)
		{
			SpinTop_Flag =1;
			Control_data.Gimbal_Wz = 	(RC_Ctl.rc.ch2-1024)*0.001f;
		}else SpinTop_Flag =0;		
		
		if (RC_Ctl.rc.s1 != RC_SW_DOWN)
			shoot_ready_control();
		else
			shoot_task();
		
		if (speed_17mm_level_Start_Flag)
			laser_on();
		if (!speed_17mm_level_Start_Flag)
			laser_off();
		
	  Gimbalmode_flag = 0; 
	}
	else//���Կ���
	{
		Control_data.Gimbal_Wz = 0.008f * RC_Ctl.mouse.x;  
		VAL_LIMIT(Control_data.Gimbal_Wz, -0.696f, 0.696f);	
		Control_data.Pitch_angle = Control_data.Pitch_angle -RC_Ctl.mouse.y * 0.003f;  
			
		//����
	  if (RC_Ctl.mouse.press_r)
		{
		  Vision_Flag = 1;
		}	
		else
		{
		  Vision_Flag = 0;
		}
	

		//����
		if (Fire_Loading_Cnt> DITHERING_TIMNE)
		{
			if (RC_Ctl.key.R && RC_Ctl.key.Ctrl && !Fire_Loading_Flag)
			{	
				Fire_Loading_Flag = !Fire_Loading_Flag;
				Fire_Loading_Cnt = 0;
			}
		}
		else Fire_Loading_Cnt++;		
		
		
		//��������
		if (Fire_Loading_End_Cnt> DITHERING_TIMNE)
		{
			if (RC_Ctl.key.Q && Fire_Loading_Flag)
			{	
				Fire_Loading_End_Flag = !Fire_Loading_End_Flag;
				Fire_Loading_End_Cnt = 0;
			}
		}
		else Fire_Loading_End_Cnt++;			
		
		
		//С����
		if (SpinTop_cnt>DITHERING_TIMNE)
		{
			if (RC_Ctl.key.G && Gimbal_180_flag==0)
			{	
				SpinTop_Flag = 1;
				SpinTop_cnt = 0;
				SpinTop_Flag_cnt = 0;
			}
		}
		else SpinTop_cnt++;
		
	
		//С���ݹر�
		if (SpinTop_close_cnt>DITHERING_TIMNE)
		{
			if (RC_Ctl.key.F && Gimbal_180_flag==0)
			{	
				SpinTop_Flag_cnt = 0;
				SpinTop_close_cnt = 0;
				SpinTop_Flag = 0;
			}
		}
		else SpinTop_close_cnt++;
		
		
		//CTRL+C ��������/��ȴ���� ģʽ�л�
		if (Shoot_mode_cnt > 200)
		{
			if (RC_Ctl.key.C && RC_Ctl.key.Ctrl)
			{	
				Fire_Mode_cnt = 0;//��UI��
				Shoot_mode_cnt = 0;
				Fire_mode = !Fire_mode;	
			}
		}
		else Shoot_mode_cnt++;
		

	  //CTRL+Z ��������/Ѫ������ ģʽ�л�
		if (Chassis_mode_cnt>200)
		{
			if (RC_Ctl.key.Z)
			{	
				Chassis_Mode_cnt = 0; //��UI��
				Chassis_mode_cnt = 0;
				Chassis_mode = !Chassis_mode;	
			}
		}
		else Chassis_mode_cnt++;
	

    //17mm���ٵ��ٵ����ٵ��л�(��Ħ����Ĭ������ٶ�)
		if (speed_17mm_level_Start_Cnt > DITHERING_TIMNE)
		{
			if (RC_Ctl.key.E)//��Ħ���֣�Ĭ������ٶ�
			{
				speed_17mm_level_Start_Flag = 1;
				speed_17mm_level_Start_Cnt = 0;
			}
		}speed_17mm_level_Start_Cnt++;
		
		
	  //B���ر�
		if (speed_17mm_level_Close_Cnt > DITHERING_TIMNE)  //��Ħ����
		{
			if (RC_Ctl.key.B)
			{
				speed_17mm_level_Start_Flag = 0;  			
				speed_17mm_level_Close_Cnt = 0;
			}
		}speed_17mm_level_Close_Cnt++;
	

    //CTRL+v ��ը����ģʽ ��������ջ�
		if (Stronghold_resist_cnt > DITHERING_TIMNE)
	  {
			if (RC_Ctl.key.Ctrl&&RC_Ctl.key.V)
			{
				Stronghold_flag = !Stronghold_flag;  
				Stronghold_resist_cnt = 0;
				Stronghold_Flag_cnt = 0;
			}
	  }else Stronghold_resist_cnt++;
		
	
		if (Chassis_mode) //��������
		{
			if (Game_robot_state.robot_level == 3)
			{
				Target_chassis_power = P_Level_3;
			}
			else if (Game_robot_state.robot_level == 2)
			{
			 Target_chassis_power = P_Level_2;
			}
			else 
			{
				Target_chassis_power = P_Level_1; //�������̹������� 55w
			}
		}
		else  //Ѫ������
		{
			if (Game_robot_state.robot_level == 3)
			{
				Target_chassis_power = Hp_Level_3;
			}
			else if (Game_robot_state.robot_level == 2)
			{
			 Target_chassis_power = Hp_Level_2;
			}
			else 
			{
				Target_chassis_power = Hp_Level_1; //�������̹������� 55w
			}
		}
    Powerlimit_Ctrl(Target_chassis_power);


		if (RC_Ctl.key.Shift)
		{
			PowerLimit_Flag = 1; //������ʱջ����ĵ��ݵĵ�
		}else PowerLimit_Flag =0;
			
		
		if (Inverse_cnt > DITHERING_TIMNE)  //�����ֶ���ת
		{
			if (RC_Ctl.key.X == 1)
			{
				Inverse_flag = 1;
				Inverse_cnt = 0;
			}
			else Inverse_flag = 0;
		}else Inverse_cnt++;

	
		offline_flag++;  //����ϵͳ���߼���
		if (offline_flag >= 2000)
		{
			offline_flag = 2000;
			Stronghold_flag = 1; //����ϵͳ���߽�������ջ���Ȼ���䲻���ӵ�
		}
	
	
	  friction_wheel_ramp_function();  //Ħ���ֿ���
    ShooterHeat_Ctrl();	
		if (!RC_Ctl.mouse.press_l)  //�������
		{
			cycle_time++;
			if(cycle_time >= 3000)
			{
				cycle_time = 3000;
				Shootnumber_fired = 0;
			}
			shoot_ready_control();
		}
		else
		{
			cycle_time = 0;
	    if(speed_17mm_level_Start_Flag && SafeHeatflag)  //Ħ���ִ�ʱ���ֲ�������
		    shoot_task1();  //���ֿ���			
	    if(!speed_17mm_level_Start_Flag)
		    trigger_moto_current = 0;						
		}
		
		if(speed_17mm_level_Start_Flag && !Vision_Flag)
			laser_on();
		else if(!speed_17mm_level_Start_Flag || Vision_Flag)
			laser_off();
	
		if(Inverse_flag)  //�����ֶ���ת
		{
			trigger_moto_speed_ref2 = -5000;
			trigger_moto_current = PID_Control(&Fire_speed_pid, trigger_moto_speed_ref2, (motor_data[6].ActualSpeed));
		}				

		if(RC_Ctl.rc.s2 == RC_SW_UP && RC_Ctl.rc.s1 == RC_SW_UP && !Vision_Flag  && !Gimbal_180_flag)//��̨���̷���򲻷���ģʽ�л�
			Gimbalmode_flag = 1; //��̨���̲�����
		else
			Gimbalmode_flag = 0;  //��̨���̷���
  }		
}


//��������
static void ShooterHeat_Ctrl(void)
{		
	//��������
	if (Fire_mode == Booming_fire)
	{
		Shootnumber_limit=4;
		if (Game_robot_state.robot_level == 1)  //�����˵ȼ���Ӧ����������
			Heatmax = 150;
		else if (Game_robot_state.robot_level == 2)
			Heatmax = 280;
		else if (Game_robot_state.robot_level == 3)
			Heatmax = 400;
  }
	else //��ȴ����
	{
		Shootnumber_limit=4;
		if (Game_robot_state.robot_level == 1)//�����˵ȼ���Ӧ����������
			Heatmax = 50;
		else if (Game_robot_state.robot_level == 2)
			Heatmax = 100;
		else if (Game_robot_state.robot_level == 3)
			Heatmax = 150;
	}
	
	Shootnumber = (Heatmax - Umpire_PowerHeat.shooter_id1_17mm_cooling_heat)/10;//�ɷ����ӵ�����

  //������������4�ŵ��裬��ȴ��������
	if (Shootnumber < Shootnumber_limit)  //�ѷ����ӵ������ڵ��ڿɷ����ӵ���ʱ����ͣת
	{  
	  SafeHeatflag = 0;
	  trigger_moto_current = 0;
	  Shootnumber_fired = 0;
	}
	else
	{
	  SafeHeatflag	= 1;
	}

	if (Stronghold_flag)  //����ϵͳ���߽�������ջ���Ȼ���䲻���ӵ�
	  SafeHeatflag = 1;
	
}

