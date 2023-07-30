#include "Start_Task.h"
#include "sys.h"
#include "delay.h"
#include "usart.h"
#include "led.h"
#include "FreeRTOS.h"
#include "task.h"
#include "timer.h"
#include "Power.h"
#include "pwm.h"
#include "can1.h"
#include "Comm_umpire.h"
#include "RemoteControl.h"
#include "spi.h"
#include "IMU.h"
#include "PID.h"
#include "can2.h"
#include "laser.h"
#include "Configuration.h"
#include "Ctrl_gimbal.h"
#include "Ctrl_chassis.h"
#include "User_Api.h"
#include "Higher_Class.h"
#include <math.h>
#include <stdlib.h>
#include "Ctrl_shoot.h"
#include "Usart_SendData.h"

//�������ȼ�
#define START_TASK_PRIO		1
//�����ջ��С	
#define START_STK_SIZE 		128  
//������
TaskHandle_t StartTask_Handler;
//������
void start_task(void *pvParameters);

//�������ȼ�
#define FRT_USART_DATA_Ctrl_TASK_PRIO      2
//�����ջ��С
#define FRT_USART_DATA_Ctrl_STK_SIZE       512
//������
TaskHandle_t  FRT_USART_DATA_CtrlTask_Handler;  

//�������ȼ�
#define FRT_Inverse_Kinematic_Ctrl_TASK_PRIO		3
//�����ջ��С	
#define FRT_Inverse_Kinematic_Ctrl_STK_SIZE 		512  
//������
TaskHandle_t FRT_Inverse_Kinematic_CtrlTask_Handler;

//�������ȼ�
#define FRT_Gimbal_Ctrl_TASK_PRIO		4
//�����ջ��С	
#define FRT_Gimbal_Ctrl_STK_SIZE 		512  
//������
TaskHandle_t FRT_Gimbal_CtrlTask_Handler;

//�������ȼ�
#define FRT_IMU_task_PRIO           5
//�����ջ��С
#define FRT_IMU_task_STK_SIZE       512
//������
TaskHandle_t  FRT_IMU_task_Handler;   

void FRT_IMU(void *pvParameters);



void start_task(void *pvParameters)
{
	  taskENTER_CRITICAL();           //�����ٽ���
    xTaskCreate((TaskFunction_t )FRT_IMU,//IMU��̬��������      
                (const char*    )"FRT_IMU",   
                (uint16_t       )FRT_IMU_task_STK_SIZE, 
                (void*          )NULL,
                (UBaseType_t    )FRT_IMU_task_PRIO,
                (TaskHandle_t*  )&FRT_IMU_task_Handler);						
	  xTaskCreate((TaskFunction_t )FRT_Gimbal_Ctrl,//��̨��������     
                (const char*    )"FRT_Gimbal_Ctrl",   
                (uint16_t       )FRT_Gimbal_Ctrl_STK_SIZE, 
                (void*          )NULL, 
                (UBaseType_t    )FRT_Gimbal_Ctrl_TASK_PRIO,
                (TaskHandle_t*  )&FRT_Gimbal_CtrlTask_Handler);
    xTaskCreate((TaskFunction_t )FRT_Inverse_Kinematic_Ctrl,//���̿�������     	
                (const char*    )"FRT_Inverse_Kinematic_Ctrl",   	
                (uint16_t       )FRT_Inverse_Kinematic_Ctrl_STK_SIZE, 
                (void*          )NULL,				 
                (UBaseType_t    )FRT_Inverse_Kinematic_Ctrl_TASK_PRIO,	
                (TaskHandle_t*  )&FRT_Inverse_Kinematic_CtrlTask_Handler);
    xTaskCreate((TaskFunction_t )FRT_USART_DATA_Ctrl,//ͨ�Ŵ�������     	
                (const char*    )"FRT_USART_DATA_Ctrl",   	
                (uint16_t       )FRT_USART_DATA_Ctrl_STK_SIZE, 
                (void*          )NULL,				 
                (UBaseType_t    )FRT_USART_DATA_Ctrl_TASK_PRIO,	
                (TaskHandle_t*  )&FRT_USART_DATA_CtrlTask_Handler);								
    vTaskDelete(StartTask_Handler); //ɾ����ʼ����
    taskEXIT_CRITICAL();            //�˳��ٽ���
}
void startTast(void)
{
	    xTaskCreate((TaskFunction_t )start_task,          //������
                (const char*    )"start_task",          //��������
                (uint16_t       )START_STK_SIZE,        //�����ջ��С
                (void*          )NULL,                  //���ݸ��������Ĳ���
                (UBaseType_t    )START_TASK_PRIO,       //�������ȼ�
                (TaskHandle_t*  )&StartTask_Handler);   //������ 
}
