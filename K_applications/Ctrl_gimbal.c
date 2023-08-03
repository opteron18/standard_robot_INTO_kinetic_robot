#include "Ctrl_gimbal.h"
#include "Kalman.h"
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
#include "bsp_fric.h"
#include "Higher_Class.h"
#include "User_Api.h"
#include "FreeRTOS.h"
#include "task.h"
#include "referee.h"
//#include "laser.h"
#include "Ctrl_shoot.h"
#include "CAN_receive.h"
//#include "Usart_SendData.h"

#define DPI 0.043945f	 //码值角度分辨率，360/8192
#define PITCH_IMU_CTRL  //pitch轴反馈控制方式
#define SHOOT_FRIC_PWM_ADD_VALUE 40  //摩擦轮信号递增值
#define fri_min_value 1000  //摩擦轮信号最小输出值

float fri_out = 0;  //摩擦轮信号递增输出值
float fri_out1=0;  //摩擦轮信号递减输出值
float fri_max_value = 1560;  //摩擦轮信号最大输出值
float speed_17mm_level = 0;

//用于Yaw轴控制的变量
float error = 0; 
float P1 = 0;
float I1 = 0;
float D1 = 0;
float angleYaw;
float E_yaw;  //未校准的yaw角度值，需要在启动云台控制前赋值E_yaw=imu.yaw
float T_yaw;  //目标角度
float Yaw_AnglePid_i;  //角度环积分项
float angle_output1 = 0;
float gryoYaw = 0;//角速度
float gryoPITCH = 0;
float Yaw_Encode_Angle = 0; 
float Pitch_Encode_Angle = 0;  //Yaw，Pitch角度

int16_t pitch_moto_current_final;
int16_t Yaw_PID_current;
int16_t yaw_moto_current;
int16_t yaw_moto_current_final;

//Jscope变量
float Yaw_Encode_Angle_jscope;
float gryoYaw_jscope;
float Control_data_Pitch_angle;
float Pitch_angle_pid_Control_OutPut;

static void Pitch_pid(float Target_angle)//pitch轴串级PID控制
{	
	if(Control_data.Pitch_angle < -15)  //20
	  Control_data.Pitch_angle = -15;  //20
	if(Control_data.Pitch_angle > 26)  //25
	  Control_data.Pitch_angle = 26;  //25
	
	gryoPITCH = -0.3f * (imu.gyro[0] - imu.gyroOffset[0]) * 57.2957795f;
  Target_angle = Control_data.Pitch_angle;
	Control_data_Pitch_angle = Control_data.Pitch_angle;
	Pitch_angle_pid_Control_OutPut = PID_Control(&Pitch_angle_pid, Target_angle, Pitch_Encode_Angle);
  PID_Control(&Pitch_speed_pid, Pitch_angle_pid.Control_OutPut, -0.3f*(imu.gyro[0]-imu.gyroOffset[0])*57.2957795f);
}


static void Yaw_angle_pid(float Targrt_d_angle)//yaw轴串级PID控制
{ 
	if(Gimbalmode_flag) //Gimbalmode_flag =1 云台底盘不分离
	{
		error = 0;
		Yaw_AnglePid_i = 0;
		T_yaw = 0;
		P1 = 0;
		I1 = 0;
		D1 = 0;
		angle_output1 = 0;
		E_yaw = imu.yaw;
			
		PID_Control(&Yaw_pid, Targrt_d_angle * 25, -Yaw_Encode_Angle);
		PID_Control(&Yaw_speed_pid, Yaw_pid.Control_OutPut, -motor_data[4].ActualSpeed);
	}	 
	else
	{
		angleYaw = -(imu.yaw - E_yaw);  //测量角度，需用上电时的角度校准imu测量角度零点
		gryoYaw = (imu.gyro[2] - imu.gyroOffset[2]) * 57.2957795f;  //角速度  
	
		T_yaw += Targrt_d_angle;	//计算目标角度
		
		if (Vision_Flag)
		{
			if (!Vision_Data.target_lose)
			  T_yaw = armor_angle_forcast;
			else
				T_yaw = angleYaw;		
		}
		
		if (T_yaw > 180)
		{
			T_yaw = T_yaw - 360;
		}
		else if (T_yaw < -180)
		{
			T_yaw = T_yaw + 360;
		}		

		error = T_yaw - angleYaw;
		if (error < -170)
		{
			error = error + 360;
		}
		else if (error > 170)
		{
			error = error - 360;
		}	

		if (Vision_Flag)
		{
			if (T_yaw < -170)
			{
				T_yaw = T_yaw + 360;
			}
			else if (T_yaw > 170)
			{
				T_yaw = T_yaw - 360;
			}	
			Yaw_Encode_Angle_jscope = angleYaw + vision_yaw_angle_target;	
			angle_output1 = PID_Control(&vision_yaw_angle_pid, T_yaw, angleYaw);
			PID_Control(&vision_yaw_speed_pid,angle_output1,-gryoYaw);
			Yaw_PID_current = vision_yaw_speed_pid.Control_OutPut;		
//			P1 = error * VISION_PID_YAW_ANGLE_KP;
//			
//			if((error > 8) || (error < 8))
//				Yaw_AnglePid_i += error;
//			else
//				Yaw_AnglePid_i = 0;
//			if(Yaw_AnglePid_i > 50)	Yaw_AnglePid_i = 50;
//			if(Yaw_AnglePid_i < -50)	Yaw_AnglePid_i = -50;
//			I1 = Yaw_AnglePid_i * VISION_PID_YAW_ANGLE_KI;
//			
//			D1 = VISION_PID_YAW_ANGLE_KD * (error - error_last);
//			angle_output1 = P1 + I1 - D1;	
//			
//			PID_Control(&vision_yaw_speed_pid, angle_output1, -gryoYaw);
//			Yaw_PID_current = vision_yaw_speed_pid.Control_OutPut;
//			error_last = error;
		}
		else 
		{			
			P1 = error * PID_YAW_ANGLE_KP;
			
			if((error < 10) && (error > -10))
				Yaw_AnglePid_i += error;
			if(Yaw_AnglePid_i > 50)	
				Yaw_AnglePid_i = 50;
			if(Yaw_AnglePid_i < -50)	
				Yaw_AnglePid_i = -50;
			if((error < 0.2f) && (error > -0.2f)) 
				Yaw_AnglePid_i = 0;		
			I1 = Yaw_AnglePid_i * PID_YAW_ANGLE_KI;
			
			D1 = PID_YAW_ANGLE_KD * gryoYaw;
			angle_output1 = P1 + I1 - D1;	
			gryoYaw_jscope = -gryoYaw;
			PID_Control(&Yaw_speed_pid, angle_output1, -gryoYaw);
			Yaw_PID_current = Yaw_speed_pid.Control_OutPut;
		}
  } 
}


void Encoder_angle_Handle(void)//云台电机旋转角度函数
{
	float Yaw_encode = motor_data[4].NowAngle;
	float Pitch_encode = motor2_data[0].NowAngle;

  Yaw_Encode_Angle = (((Yaw_encode - Yaw_Mid_encode) * DPI) * Yaw_Direction);
  Pitch_Encode_Angle =((Pitch_encode - Pitch_Mid_encode) * DPI) * Pitch_Direction;	
	
  //YAW轴过0处理
  if(Yaw_Encode_Angle > 180)
	{
	  Yaw_Encode_Angle -= 360;
	}
	if(Yaw_Encode_Angle < -180)
	{
		Yaw_Encode_Angle += 360;
	}
}


static void ramp_calc0(void)//摩擦轮pwm值递增
{
	fri_out += (SHOOT_FRIC_PWM_ADD_VALUE * 0.01);
	
	if(fri_out > speed_17mm_level)
	{
		fri_out = speed_17mm_level;
	}
	else if(fri_out < fri_min_value)
	{
		fri_out = fri_min_value;
	}
}	


static void ramp_calc1(void)//摩擦轮pwm值递减
{ 
	
	fri_out1 = fri_out-1;
	fri_out--;
	
	if(fri_out1 > speed_17mm_level)
	{
		fri_out1 = speed_17mm_level;
	}
	else if(fri_out1 < fri_min_value)
	{
		fri_out1 = fri_min_value;
	}
}


void friction_wheel_ramp_function(void)//摩擦轮控制函数
{
	if(speed_17mm_level_Start_Flag)//摩擦轮开启
	{
		if(Fire_mode==0 && (Game_robot_state.robot_level > 1))//冷却优先高等级射速
		  speed_17mm_level = 1620;  // 18m/s
		else
		  speed_17mm_level = 1537;//1537;  //1500// 15m/s
		
		ramp_calc0();
//		PWM_Write(PWM2_CH1,fri_out);
//    PWM_Write(PWM2_CH2,fri_out);	
		fric1_on(fri_out);
		fric2_on(fri_out);
	} 
	
	//关闭摩擦轮
	if(!speed_17mm_level_Start_Flag)
	{
		ramp_calc1();
		fric1_on(fri_out);
		fric2_on(fri_out);
	}
}


//摩擦轮开关函数
void Control_on_off_friction_wheel(void)  
{
	if (RC_Ctl.rc.s1 == RC_SW_UP && RC_Ctl.rc.s1_last != RC_SW_UP)
	{
		speed_17mm_level_Start_Flag = !speed_17mm_level_Start_Flag;
	}
}


//云台整体控制
void Gimbal_Ctrl(float pitch,float yaw_rate)
{
  Encoder_angle_Handle();  //云台电机旋转角度函数
	#ifdef PITCH_IMU_CTRL	
	/*if(imu.roll > 0 && imu.roll > 150)
		imu.roll =(imu.roll - 179)-179;
	if(imu.roll < 0)
		imu.roll += 179;
	if(imu.roll > 120)
		imu.roll = -25;
	if(imu.roll >15)
		imu.roll = 15;*/
	Pitch_pid(pitch);  //pitch轴串级PID控制
	#else	
	Pitch_pid(pitch);
	#endif
	Yaw_angle_pid(yaw_rate);//yaw轴串级PID控制
	
	yaw_moto_current = Yaw_PID_current;
	yaw_moto_current_final = -yaw_moto_current;
	pitch_moto_current_final = Pitch_Direction * Pitch_speed_pid.Control_OutPut;
  CAN1_Send_Msg_gimbal(yaw_moto_current_final, 200, trigger_moto_current);//can发送云台控制量
	CAN2_Send_Msg_gimbal(pitch_moto_current_final);
}


//云台控制任务
void FRT_Gimbal_Ctrl(void *pvParameters)
{
	vTaskDelay(201);
	while(1)
	{
		if (RC_Ctl.rc.s2 == RC_SW_DOWN)
			Power_off_function();
		else
		{	
			gimbal_control_acquisition();  //云台控制量获取，状态机设置
			gimbal_set_contorl();  //云台控制量设置
			Gimbal_Ctrl(Control_data.Pitch_angle,Control_data.Gimbal_Wz);  //云台，射击的控制接口	
		}
	  vTaskDelay(1);  //任务运行周期
	}
}

