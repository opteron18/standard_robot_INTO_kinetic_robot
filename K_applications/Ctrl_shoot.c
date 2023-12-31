//#include "Ctrl_shoot.h"
#include "shoot.h"
#include "Ctrl_gimbal.h"
#include "INS_task.h"
#include "IMU.h"
#include "PID.h"
#include "CAN_receive.h"
#include "Configuration.h"
#include "User_Api.h"
#include <math.h>
#include <stdlib.h>
#include "remote_control.h"
//#include "delay.h"
//#include "pwm.h"
#include "Higher_Class.h"
#include "User_Api.h"
#include "FreeRTOS.h"
#include "task.h"
//#include "laser.h"
#include "CAN_receive.h"

#define Butten_Trig_Pin HAL_GPIO_ReadPin(BUTTON_TRIG_GPIO_Port, BUTTON_TRIG_Pin)
#define FireMotor_Direction 1 //发射拨盘的转动方向

u8 Fire_mode = Fire_mode_init;
int16_t trigger_moto_speed_ref2;  //目标速度 
int16_t trigger_moto_current;  //传给电调的值
int32_t trigger_moto_position_ref2;  //目标角度

int block_flag = 0;//堵转标志
int block_time = 0;//堵转时长
int blockruntime = 0;//堵转计数

int block_flag1 = 0;//堵转标志
int block_time1 = 0;//堵转时长
int blockruntime1 = 0;//堵转计数 

int time = 0;  //连发计数
int single_flag = 0;
int continue_flag = 0;
int trigger_motor_key_time = 0;
int Butten_Trig_Pin_flag = 0;

float degree_60_to_encode_ramp = 73728/128;//25941/1024;
float degree_60_to_encode = 0;//25941;//25941;
int degree_60_to_encode_cnt = 0;


void Encode_C(M_Data*ptr)
{
	if (abs(ptr->NowAngle - ptr->OldAngle) > 4095)
  {
    if (ptr->NowAngle < ptr->OldAngle)
		{
		   ptr->total_encode += 8191 - ptr->OldAngle + ptr->NowAngle;
		}
		else
		{
			ptr->total_encode -= 8191 - ptr->NowAngle + ptr->OldAngle;
		}
  }
  else 
	{
    ptr->total_encode += ptr->NowAngle - ptr->OldAngle;
	}		
}


float RAMP_float( float final, float now, float ramp )
{
	  float buffer = 0;
	  buffer = final - now;
	
		if (buffer > 0)
		{
			if (buffer > ramp)
			{  
				now += ramp;
			}   
			else
			{
				now += buffer;
			}
		}
		else
		{
			if (buffer < -ramp)
			{
				now += -ramp;
			}
			else
			{
				now += buffer;
			}
		}
		return now;
}


void shoot_ready_control(void)
{
	block_time = 0;
	blockruntime = 0;
	RC_Ctl.rc.s1_last = RC_Ctl.rc.s1;
	RC_Ctl.mouse.press_l_last = RC_Ctl.mouse.press_l;
	if(!Butten_Trig_Pin)  //判断子弹到达微动开关处  
	{
		trigger_moto_current = PID_Control(&Fire_speed_pid, 0, motor_data[6].ActualSpeed);
		trigger_motor_key_time = 0;
		Butten_Trig_Pin_flag = 1;
	}
	else if(Butten_Trig_Pin && trigger_motor_key_time < 1000)  //判断无子弹一段时间  //触碰开关与单发模式下的结合
	{  
		Butten_Trig_Pin_flag = 2;
		if(single_flag)  //单发模式转到大约期望角度停转
		{
			if(trigger_motor_key_time > 20)  //单发模式下强制计数停止//虽然不知道意义何在
			trigger_moto_current = PID_Control(&Fire_speed_pid, 0, motor_data[6].ActualSpeed);  //PID速度环//	注意这里trigger_moto_current不能直接赋0
		}
		if (continue_flag)//连发模式直接停转//这里加这个处理可以做到单发连发切换效果，切换中间停顿一下
		  trigger_moto_current = PID_Control(&Fire_speed_pid, 0, motor_data[6].ActualSpeed);//PID速度环	
		trigger_motor_key_time++;
	}
	else if(Butten_Trig_Pin && trigger_motor_key_time == 1000)  //微动开关一段时间没有子弹，进入拨弹
	{
		Butten_Trig_Pin_flag = 2;
		if(!block_flag1)
		{
		  trigger_moto_speed_ref2 = 900 * FireMotor_Direction;  //微动开关没检测到子弹，波轮自动转
		  trigger_moto_current = PID_Control(&Fire_speed_pid, trigger_moto_speed_ref2, (motor_data[6].ActualSpeed));
		}		
		if (motor_data[6].Intensity > 8000)
		   blockruntime1++;
		if(blockruntime1 > 45) //开始反转的时间 决定反转的频率 100
		{
		  trigger_moto_speed_ref2 = -FireMotor_Direction*7500;	//决定反转的力度 5000
		  block_flag1 = 1;
		}
		if(block_flag1)
		{
			block_time1++;
			if(block_time1 > 55)//反转的时间 决定反转的幅度 40
			{
				block_flag1 = 0;
				block_time1 = 0;
				blockruntime1 = 0;
			}	
		}
		if(block_flag1)
		  trigger_moto_current = PID_Control(&Fire_speed_pid, trigger_moto_speed_ref2, (motor_data[6].ActualSpeed));	
	}
}


//遥控器模式控制拨轮
void shoot_task(void)
{
	block_flag1 = 0;
  blockruntime1 = 0;
	trigger_motor_key_time = 0;
	Encode_C(&motor_data[6]);
	if(!block_flag)
	{
		if(RC_Ctl.rc.s1 == RC_SW_DOWN && RC_Ctl.rc.s1_last != RC_SW_DOWN)
		{
			time = xTaskGetTickCount();
			single_flag = 1;
			continue_flag = 0;			
			trigger_moto_position_ref2 = motor_data[6].total_encode + FireMotor_Direction*DEGREE_60_TO_ENCODER;
		}
			
		trigger_moto_speed_ref2 = PID_Control(&Fire_pid, trigger_moto_position_ref2, motor_data[6].total_encode);
		
		if(RC_Ctl.rc.s1 == RC_SW_DOWN && xTaskGetTickCount() - time > 160)  //左拨杆一直朝下连发模式  清理子弹用
		{
			trigger_moto_speed_ref2 = FireMotor_Direction*5000;
			single_flag = 0;
			continue_flag = 1;		
		}

		PID_Control(&Fire_speed_pid, trigger_moto_speed_ref2, (motor_data[6].ActualSpeed));	

		trigger_moto_current = Fire_speed_pid.Control_OutPut;
		
		RC_Ctl.rc.s1_last = RC_Ctl.rc.s1;
  }
  
	block_bullet_handle();
	//电流值强制清零
	if((RC_Ctl.rc.s1 != RC_SW_DOWN))
	{
	  trigger_moto_current = 0;
    blockruntime = 0;	
	}
	
}


//电脑模式控制拨轮
void shoot_task1()
{
	block_flag1 = 0;
  blockruntime1 = 0;
	trigger_motor_key_time = 0;
	Encode_C(&motor_data[6]);
	if(!block_flag)
	{
		if(RC_Ctl.mouse.press_l && RC_Ctl.mouse.press_l != RC_Ctl.mouse.press_l_last)
		{
			time = xTaskGetTickCount();
			single_flag = 1;
			continue_flag = 0;			
			trigger_moto_position_ref2 = motor_data[6].total_encode + FireMotor_Direction*DEGREE_60_TO_ENCODER;
		}
		
		trigger_moto_speed_ref2 = PID_Control(&Fire_pid, trigger_moto_position_ref2, motor_data[6].total_encode);
		
		if(RC_Ctl.mouse.press_l && xTaskGetTickCount() - time > 160)//鼠标左键长按连发模式
		{
			single_flag = 0;
			continue_flag = 1;		
			if(Stronghold_flag)
			{
				trigger_moto_speed_ref2 = 5000;  
			}		
			else
			{
				trigger_moto_speed_ref2 = 5000;
			}

		}	
		
		PID_Control(&Fire_speed_pid, trigger_moto_speed_ref2, (motor_data[6].ActualSpeed));	

		trigger_moto_current = Fire_speed_pid.Control_OutPut;

		RC_Ctl.mouse.press_l_last = RC_Ctl.mouse.press_l;//记录上次鼠标左键情况
  }
	
  block_bullet_handle();
	
  if(!RC_Ctl.mouse.press_l)//鼠标左键松开
	{
    blockruntime = 0;	
	}		
}


//卡弹自动反转处理
//此处反转与触碰开关处不冲突即触碰开关任务的反转与这里的反转任务是独立的
void block_bullet_handle()//卡弹自动反转60度
{
	if ((motor_data[6].Intensity > 8000 && RC_Ctl.rc.s1 == RC_SW_DOWN) || (motor_data[6].Intensity > 8000 && RC_Ctl.mouse.press_l))
		blockruntime++;
	
	if(blockruntime > 45) //100
	{
	  trigger_moto_speed_ref2 = -7500;  //8500
	  block_flag = 1;
	}
	
	if(block_flag)
	{
		block_time++;
		if(block_time > 55)  //150
		{
			blockruntime = 0;
			block_flag  = 0;
			block_time = 0; 		
		}	
  }
	
	if(block_flag)
    trigger_moto_current = PID_Control(&Fire_speed_pid, trigger_moto_speed_ref2, (motor_data[6].ActualSpeed));
}

