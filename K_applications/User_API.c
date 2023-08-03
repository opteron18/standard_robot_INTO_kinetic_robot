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

//状态机
u8 SpinTop_Flag =0;             //小陀螺标志
u8 Twist_Flag = 0;			        //扭腰标志
u8 Vision_Flag = 0;			        //视觉自瞄标志
u8 Gimbal_180_flag = 0;         //180度标志
u8 Stronghold_flag = 0;         //热量限制标志
u8 Inverse_flag = 0;            //拨轮反转标志
u8 speed_17mm_level_Start_Flag = 0;  //摩擦轮开启标志
u8 Fire_Loading_Flag = 0;  //补弹标志位
u8 Fire_Loading_End_Flag = 0;   //补弹结束标志位
int Chassismode_flag = 0;       //底盘模式标志位
int Gimbalmode_flag = 0;        //云台模式标志位

//计数
u8 TwistCnt,Inverse_cnt,Stronghold_resist_cnt,SpinTop_cnt,SpinTop_close_cnt;
u8 speed_17mm_level_Close_Cnt,speed_17mm_level_Start_Cnt;
u8 Shoot_mode_cnt,Chassis_mode_cnt,Fire_Loading_Cnt,Fire_Loading_End_Cnt;

//底盘相关
int16_t Target_chassis_power = 0;
int32_t VerticalCnt = 0;
int32_t HorizontalCnt = 0;
float Ramp_K = 1.5f;  
float Wz_increasing = 0;
float PC_Speed_Vx =0;
float PC_Speed_Vy =0;
float speed_zoom = 1.0f ; //遥控器控制的速度
u8 Chassis_mode = Chassis_mode_init;

//发射相关
int Heatmax = 0;  //最大热量
int Shootnumber = 0;  //可发射子弹数量
int SafeHeatflag = 1;  //安全热量标志
int Shootnumber_fired = 0;  //已发射子弹数
int cycle_time = 0;  //一个发射周期时间
float Shootnumber_limit = 4;

ControlDATA_TypeDef Control_data;//控制数据


void chassis_control_acquisition(void)
{
	if (RC_Ctl.rc.s2 == RC_SW_MID)//遥控控制
	{
		Control_data.Vx = (RC_Ctl.rc.ch1 - 1024) * 6.2;
		Control_data.Vy = (RC_Ctl.rc.ch0 - 1024) * 6.2;
		Control_data.Chassis_Wz = (RC_Ctl.rc.ch2 - 1024) * 0.035f;    
		Chassismode_flag = 0;
		offline_flag++;//裁判系统离线计数
		if(offline_flag >= 2000)
			offline_flag = 2000;
	}
	else //电脑控制
	{  
		//输入值给定斜坡
		if(RC_Ctl.key.Vertical)
		{ 
			VerticalCnt++;
			PC_Speed_Vx = Ramp_K * sqrt(5000 * VerticalCnt);  //开方根函数加速度逐渐减小的加速斜坡
		}  
		else 
		{
			PC_Speed_Vx =0;
			VerticalCnt=0;
		}
		if (RC_Ctl.key.Horizontal)
		{   
			 HorizontalCnt++;
			 PC_Speed_Vy = Ramp_K*sqrt(4000*HorizontalCnt);//斜坡启动有关  //3000
		}	 
		else 
		{
			 PC_Speed_Vy=0;
			 HorizontalCnt=0;
		}
		Control_data.Vx = RC_Ctl.key.Vertical * PC_Speed_Vx;
		Control_data.Vy = RC_Ctl.key.Horizontal * PC_Speed_Vy;
	 
		//旋转斜坡
		if(RC_Ctl.mouse.x != 0) 
			Wz_increasing = 0.015f;    
		else
			Wz_increasing = 0;
	 
		if(Wz_increasing >= 2.3f)   
			Wz_increasing = 2.3f;       
		Control_data.Chassis_Wz = Wz_increasing * RC_Ctl.mouse.x;

		//总输出速度限幅
		VAL_LIMIT(Control_data.Vx, -MAX_VX, MAX_VX);  
		VAL_LIMIT(Control_data.Vy, -MAX_VY, MAX_VY);
		VAL_LIMIT(Control_data.Chassis_Wz, -MAX_WZ, MAX_WZ);
		
		if(RC_Ctl.rc.s2 == RC_SW_UP && RC_Ctl.rc.s1 == RC_SW_UP)//云台底盘分离和不分离模式切换
		{
			Chassismode_flag = 1;//云台底盘不分离模式	
		}
		else
		{
			yawcnt = 0;
			Chassismode_flag = 0;//云台底盘分离模式
		}
  }
}


void gimbal_control_acquisition(void)
{
	if (RC_Ctl.rc.s2 == RC_SW_MID)//遥控控制
	{
		Control_data.Gimbal_Wz = (RC_Ctl.rc.ch2 - 1024) * 0.001f;  
		Control_data.Pitch_angle += (RC_Ctl.rc.ch3 - 1024) * 0.0003f;
	
		Control_on_off_friction_wheel();//摩擦轮开关函数
		friction_wheel_ramp_function();//摩擦轮控制函数
	
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
	else//电脑控制
	{
		Control_data.Gimbal_Wz = 0.008f * RC_Ctl.mouse.x;  
		VAL_LIMIT(Control_data.Gimbal_Wz, -0.696f, 0.696f);	
		Control_data.Pitch_angle = Control_data.Pitch_angle -RC_Ctl.mouse.y * 0.003f;  
			
		//自瞄
	  if (RC_Ctl.mouse.press_r)
		{
		  Vision_Flag = 1;
		}	
		else
		{
		  Vision_Flag = 0;
		}
	

		//补弹
		if (Fire_Loading_Cnt> DITHERING_TIMNE)
		{
			if (RC_Ctl.key.R && RC_Ctl.key.Ctrl && !Fire_Loading_Flag)
			{	
				Fire_Loading_Flag = !Fire_Loading_Flag;
				Fire_Loading_Cnt = 0;
			}
		}
		else Fire_Loading_Cnt++;		
		
		
		//补弹结束
		if (Fire_Loading_End_Cnt> DITHERING_TIMNE)
		{
			if (RC_Ctl.key.Q && Fire_Loading_Flag)
			{	
				Fire_Loading_End_Flag = !Fire_Loading_End_Flag;
				Fire_Loading_End_Cnt = 0;
			}
		}
		else Fire_Loading_End_Cnt++;			
		
		
		//小陀螺
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
		
	
		//小陀螺关闭
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
		
		
		//CTRL+C 爆发优先/冷却优先 模式切换
		if (Shoot_mode_cnt > 200)
		{
			if (RC_Ctl.key.C && RC_Ctl.key.Ctrl)
			{	
				Fire_Mode_cnt = 0;//画UI用
				Shoot_mode_cnt = 0;
				Fire_mode = !Fire_mode;	
			}
		}
		else Shoot_mode_cnt++;
		

	  //CTRL+Z 功率优先/血量优先 模式切换
		if (Chassis_mode_cnt>200)
		{
			if (RC_Ctl.key.Z)
			{	
				Chassis_Mode_cnt = 0; //画UI用
				Chassis_mode_cnt = 0;
				Chassis_mode = !Chassis_mode;	
			}
		}
		else Chassis_mode_cnt++;
	

    //17mm射速低速档高速档切换(开摩擦轮默认最低速度)
		if (speed_17mm_level_Start_Cnt > DITHERING_TIMNE)
		{
			if (RC_Ctl.key.E)//开摩擦轮，默认最低速度
			{
				speed_17mm_level_Start_Flag = 1;
				speed_17mm_level_Start_Cnt = 0;
			}
		}speed_17mm_level_Start_Cnt++;
		
		
	  //B键关闭
		if (speed_17mm_level_Close_Cnt > DITHERING_TIMNE)  //关摩擦轮
		{
			if (RC_Ctl.key.B)
			{
				speed_17mm_level_Start_Flag = 0;  			
				speed_17mm_level_Close_Cnt = 0;
			}
		}speed_17mm_level_Close_Cnt++;
	

    //CTRL+v 爆炸开火模式 解除热量闭环
		if (Stronghold_resist_cnt > DITHERING_TIMNE)
	  {
			if (RC_Ctl.key.Ctrl&&RC_Ctl.key.V)
			{
				Stronghold_flag = !Stronghold_flag;  
				Stronghold_resist_cnt = 0;
				Stronghold_Flag_cnt = 0;
			}
	  }else Stronghold_resist_cnt++;
		
	
		if (Chassis_mode) //功率优先
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
				Target_chassis_power = P_Level_1; //步兵底盘功率三级 55w
			}
		}
		else  //血量优先
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
				Target_chassis_power = Hp_Level_1; //步兵底盘功率三级 55w
			}
		}
    Powerlimit_Ctrl(Target_chassis_power);


		if (RC_Ctl.key.Shift)
		{
			PowerLimit_Flag = 1; //解除功率闭环消耗电容的电
		}else PowerLimit_Flag =0;
			
		
		if (Inverse_cnt > DITHERING_TIMNE)  //拨轮手动反转
		{
			if (RC_Ctl.key.X == 1)
			{
				Inverse_flag = 1;
				Inverse_cnt = 0;
			}
			else Inverse_flag = 0;
		}else Inverse_cnt++;

	
		offline_flag++;  //裁判系统离线计数
		if (offline_flag >= 2000)
		{
			offline_flag = 2000;
			Stronghold_flag = 1; //裁判系统离线解除热量闭环不然发射不了子弹
		}
	
	
	  friction_wheel_ramp_function();  //摩擦轮控制
    ShooterHeat_Ctrl();	
		if (!RC_Ctl.mouse.press_l)  //射击策略
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
	    if(speed_17mm_level_Start_Flag && SafeHeatflag)  //摩擦轮打开时拨轮才能启动
		    shoot_task1();  //拨轮控制			
	    if(!speed_17mm_level_Start_Flag)
		    trigger_moto_current = 0;						
		}
		
		if(speed_17mm_level_Start_Flag && !Vision_Flag)
			laser_on();
		else if(!speed_17mm_level_Start_Flag || Vision_Flag)
			laser_off();
	
		if(Inverse_flag)  //拨轮手动反转
		{
			trigger_moto_speed_ref2 = -5000;
			trigger_moto_current = PID_Control(&Fire_speed_pid, trigger_moto_speed_ref2, (motor_data[6].ActualSpeed));
		}				

		if(RC_Ctl.rc.s2 == RC_SW_UP && RC_Ctl.rc.s1 == RC_SW_UP && !Vision_Flag  && !Gimbal_180_flag)//云台底盘分离或不分离模式切换
			Gimbalmode_flag = 1; //云台底盘不分离
		else
			Gimbalmode_flag = 0;  //云台底盘分离
  }		
}


//热量限制
static void ShooterHeat_Ctrl(void)
{		
	//爆发优先
	if (Fire_mode == Booming_fire)
	{
		Shootnumber_limit=4;
		if (Game_robot_state.robot_level == 1)  //机器人等级对应的热量上限
			Heatmax = 150;
		else if (Game_robot_state.robot_level == 2)
			Heatmax = 280;
		else if (Game_robot_state.robot_level == 3)
			Heatmax = 400;
  }
	else //冷却优先
	{
		Shootnumber_limit=4;
		if (Game_robot_state.robot_level == 1)//机器人等级对应的热量上限
			Heatmax = 50;
		else if (Game_robot_state.robot_level == 2)
			Heatmax = 100;
		else if (Game_robot_state.robot_level == 3)
			Heatmax = 150;
	}
	
	Shootnumber = (Heatmax - Umpire_PowerHeat.shooter_id1_17mm_cooling_heat)/10;//可发射子弹数量

  //爆发优先限制4颗弹丸，冷却优先限制
	if (Shootnumber < Shootnumber_limit)  //已发射子弹数大于等于可发射子弹数时拨轮停转
	{  
	  SafeHeatflag = 0;
	  trigger_moto_current = 0;
	  Shootnumber_fired = 0;
	}
	else
	{
	  SafeHeatflag	= 1;
	}

	if (Stronghold_flag)  //裁判系统离线解除热量闭环不然发射不了子弹
	  SafeHeatflag = 1;
	
}

