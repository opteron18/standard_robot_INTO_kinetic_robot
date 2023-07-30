#ifndef IMUSO3_H
#define IMUSO3_H

#include "sys.h"
#include "INS_task.h"


extern uint8_t bFilterInit;	//��������������־λ
extern float imu_last;
extern float pitch,roll,yaw; 		//ŷ����
extern short aacx,aacy,aacz;		//���ٶȴ�����ԭʼ����
extern short gyrox,gyroy,gyroz;	//������ԭʼ����
//У׼ʱ��
#define ACC_CALC_TIME  3000//ms
#define GYRO_CALC_TIME   3000000l	//us

/* Function prototypes */
static float invSqrt(float number);
static void NonlinearSO3AHRSinit(float ax, float ay, float az, float mx, float my, float mz);
static void NonlinearSO3AHRSupdate(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz, float twoKp, float twoKi, float dt);

static void IMUSO3Thread(void);		//imu��ȡ����̬�����ܽӿ�
static void IMU_Temp_Ctrl(void);		//imu�¶ȱջ�
void IMU_Calibration(void);	//imuУ׼
static void IMUSO3Threadout(void);
void Attitude_Calculation(float gx,float gy,float gz,float ax,float ay,float az);

#endif

