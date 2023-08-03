/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       can_receive.c/h
  * @brief      there is CAN interrupt function  to receive motor data,
  *             and CAN send function to send motor current to control motor.
  *             ������CAN�жϽ��պ��������յ������,CAN���ͺ������͵���������Ƶ��.
  * @note       
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Dec-26-2018     RM              1. done
  *  V1.1.0     Nov-11-2019     RM              1. support hal lib
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2019 DJI****************************
  */

#include "CAN_receive.h"

#include "cmsis_os.h"

#include "main.h"
#include "bsp_rng.h"
#include "Configuration.h"

#include "detect_task.h"

extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;
//motor data read
#define get_motor_measure(ptr, data)                                    \
    {                                                                   \
        (ptr)->last_ecd = (ptr)->ecd;                                   \
        (ptr)->ecd = (uint16_t)((data)[0] << 8 | (data)[1]);            \
        (ptr)->speed_rpm = (uint16_t)((data)[2] << 8 | (data)[3]);      \
        (ptr)->given_current = (uint16_t)((data)[4] << 8 | (data)[5]);  \
        (ptr)->temperate = (data)[6];                                   \
    }
/*
motor data,  0:chassis motor1 3508;1:chassis motor3 3508;2:chassis motor3 3508;3:chassis motor4 3508;
4:yaw gimbal motor 6020;5:pitch gimbal motor 6020;6:trigger motor 2006;
�������, 0:���̵��1 3508���,  1:���̵��2 3508���,2:���̵��3 3508���,3:���̵��4 3508���;
4:yaw��̨��� 6020���; 5:pitch��̨��� 6020���; 6:������� 2006���*/
static motor_measure_t motor_chassis[7];

static CAN_TxHeaderTypeDef  gimbal_tx_message;
static uint8_t              gimbal_can_send_data[8];
static CAN_TxHeaderTypeDef  chassis_tx_message;
static uint8_t              chassis_can_send_data[8];
static CAN_TxHeaderTypeDef  CAN2_tx_message;
static uint8_t              CAN2_can_send_data[8];
static CAN_TxHeaderTypeDef  Powerlimit_tx_message;
static uint8_t              Powerlimit_can_send_data[8];


/*********�˴���ɹ���********
 *���������ݽṹ��
 *��ʼ��can
 *���忨�����˲�����
 *���յ�����ݲ������˲�
 *���巢�����ݺ���
 *����һ���ڻ�е�ǲ���
 ****************************/

M_Data motor_data[7];			//���������ݽṹ��
uint8_t cnt[5]={0,0,0,0,0};	//������
int16_t old_angle[4]={0};	//�ɽǶ�ֵ
float PowerData[4];   //���ݰ����������ݽṹ��
float Real_power = 0;
float Real_voltage = 0;
float Real_current = 0;

float vision_error = 0;
M_Data motor2_data[3];			//���������ݽṹ��
Vision_InitTypeDef Vision_Data={0};

/**
  * @brief          hal CAN fifo call back, receive motor data
  * @param[in]      hcan, the point to CAN handle
  * @retval         none
  */
/**
  * @brief          hal��CAN�ص�����,���յ������
  * @param[in]      hcan:CAN���ָ��
  * @retval         none
  */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    CAN_RxHeaderTypeDef rx_header;
    uint8_t rx_data[8];

    HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, rx_data);

    switch (rx_header.StdId)
    {
//        case CAN_3508_M1_ID:
//        case CAN_3508_M2_ID:
//        case CAN_3508_M3_ID:
//        case CAN_3508_M4_ID:
//        case CAN_YAW_MOTOR_ID:
//        case CAN_PIT_MOTOR_ID:
//        case CAN_TRIGGER_MOTOR_ID:
//        {
//            static uint8_t i = 0;
//            //get motor id
//            i = rx_header.StdId - CAN_3508_M1_ID;
//            get_motor_measure(&motor_chassis[i], rx_data);
//            detect_hook(CHASSIS_MOTOR1_TOE + i);
//            break;
//        }

//        default:
//        {
//            break;
//        }
				case 0x201:	//���̵��
			{
				motor_data[0].ActualSpeed = (uint16_t)((rx_data[2]<<8)+rx_data[3]);
				motor_data[0].ActualSpeed = kalman(motor_data[0].ActualSpeed,0+4);//������ƽ������
				
				cnt[0]++;
				if (cnt[0]==Hz_Flag)//���ÿ������Ϣn��������е��һ��
				{
					cnt[0] = 0;
					motor_data[0].OldAngle = motor_data[0].NowAngle;
					motor_data[0].NowAngle = (uint16_t)((rx_data[0]<<8)+rx_data[1]);
					
					Angle_deal(0);												//��е��Խ�紦��
					motor_data[0].D_Angle = kalman(motor_data[0].D_Angle,0);	//��е���˲�
				}
				break;
			}
			case 0x202:	//���̵��
			{
				motor_data[1].ActualSpeed=(uint16_t)((rx_data[2]<<8)+rx_data[3]);
				motor_data[1].ActualSpeed = kalman(motor_data[1].ActualSpeed,1+4);//������ƽ������
				
				cnt[1]++;
				if (cnt[1]==Hz_Flag)//���ÿ������Ϣn��������е��һ��
				{
					cnt[1] = 0;
					motor_data[1].OldAngle = motor_data[1].NowAngle;
					motor_data[1].NowAngle=(uint16_t)((rx_data[0]<<8)+rx_data[1]);
					Angle_deal(1);												//��е��Խ�紦��
					motor_data[1].D_Angle = kalman(motor_data[1].D_Angle,1);	//��е���˲�
				}
				break;
			}
			case 0x203:	//���̵��
			{
				motor_data[2].ActualSpeed=(uint16_t)((rx_data[2]<<8)+rx_data[3]);
				motor_data[2].ActualSpeed = kalman(motor_data[2].ActualSpeed,2+4);//������ƽ������
				
				cnt[2]++;
				if (cnt[2]==Hz_Flag)//���ÿ������Ϣn��������е��һ��
				{
					cnt[2] = 0;
					motor_data[2].OldAngle = motor_data[2].NowAngle;
					motor_data[2].NowAngle=(uint16_t)((rx_data[0]<<8)+rx_data[1]);
					Angle_deal(2);												//��е��Խ�紦��
					motor_data[2].D_Angle = kalman(motor_data[2].D_Angle,2);	//��е���˲�
				}
				break;
			}
			case 0x204:	//���̵��
			{
				motor_data[3].ActualSpeed=(uint16_t)((rx_data[2]<<8)+rx_data[3]);
				motor_data[3].ActualSpeed = kalman(motor_data[3].ActualSpeed,3+4);//������ƽ������
				
				cnt[3]++;
				if (cnt[3]==Hz_Flag)//���ÿ������Ϣn��������е��һ��
				{
					cnt[3] = 0;
					motor_data[3].OldAngle = motor_data[3].NowAngle;
					motor_data[3].NowAngle=(uint16_t)((rx_data[0]<<8)+rx_data[1]);
					Angle_deal(3);												//��е��Խ�紦��
					motor_data[3].D_Angle = kalman(motor_data[3].D_Angle,3);	//��е���˲�
				}
				break;
			}
			case 0x209://��̨���yaw��
			{
				motor_data[4].OldAngle = motor_data[4].NowAngle;
				motor_data[4].NowAngle = (uint16_t)((rx_data[0]<<8)+rx_data[1]);
				motor_data[4].ActualSpeed = (uint16_t)((rx_data[2]<<8)| rx_data[3]);
				motor_data[4].Intensity = (uint16_t)((rx_data[4]<<8)+rx_data[5]);
				break;
			}			
			case 0x205://���ָǵ��
			{
				motor_data[5].NowAngle = (uint16_t)((rx_data[0]<<8)+rx_data[1]);
				motor_data[5].ActualSpeed = (uint16_t)((rx_data[2]<<8)|rx_data[3]);
				motor_data[5].Intensity = (uint16_t)((rx_data[4]<<8)+rx_data[5]);
				break;
			}
			case 0x206://�������
			{
				motor_data[6].OldAngle = motor_data[6].NowAngle;
				motor_data[6].NowAngle=(uint16_t)((rx_data[0]<<8)+rx_data[1]);
				motor_data[6].ActualSpeed=(uint16_t)((rx_data[2]<<8)+rx_data[3]);
				motor_data[6].Intensity=(uint16_t)((rx_data[4]<<8)+rx_data[5]);
				break;
			}
	    case 0x211://�����������ذ���M0��дĬ�ϵ�λ��ǰ
			{		
				PowerData[0]=(uint16_t)((rx_data[1]<<8)+rx_data[0])/ 100.f ; //�����ѹ
				PowerData[1]=(uint16_t)((rx_data[3]<<8)+rx_data[2])/ 100.f ;  //���ݵ�ѹ //����������ϵͳ��customdata
				PowerData[2]=(uint16_t)((rx_data[5]<<8)+rx_data[4])/ 100.f ;  //�������
				PowerData[3]=(uint16_t)((rx_data[7]<<8)+rx_data[6])/ 100.f ;  //�趨����
			break;
			}
			case 0x212://���ʼ�
			{			
				Real_voltage=(uint16_t)((rx_data[1]<<8)+rx_data[0])/ 100.f ; //��ѹ
				Real_current=(uint16_t)((rx_data[3]<<8)+rx_data[2])/ 100.f ;  //����
				Real_power = Real_voltage*Real_current;
			break;
			}
			default:
			break;
    }
}



/**
  * @brief          send control current of motor (0x205, 0x206, 0x207, 0x208)
  * @param[in]      yaw: (0x205) 6020 motor control current, range [-30000,30000] 
  * @param[in]      pitch: (0x206) 6020 motor control current, range [-30000,30000]
  * @param[in]      shoot: (0x207) 2006 motor control current, range [-10000,10000]
  * @param[in]      rev: (0x208) reserve motor control current
  * @retval         none
  */
/**
  * @brief          ���͵�����Ƶ���(0x205,0x206,0x207,0x208)
  * @param[in]      yaw: (0x205) 6020������Ƶ���, ��Χ [-30000,30000]
  * @param[in]      pitch: (0x206) 6020������Ƶ���, ��Χ [-30000,30000]
  * @param[in]      shoot: (0x207) 2006������Ƶ���, ��Χ [-10000,10000]
  * @param[in]      rev: (0x208) ������������Ƶ���
  * @retval         none
  */
void CAN_cmd_gimbal(int16_t yaw, int16_t pitch, int16_t shoot, int16_t rev)
{
    uint32_t send_mail_box;
    gimbal_tx_message.StdId = CAN_GIMBAL_ALL_ID;
    gimbal_tx_message.IDE = CAN_ID_STD;
    gimbal_tx_message.RTR = CAN_RTR_DATA;
    gimbal_tx_message.DLC = 0x08;
    gimbal_can_send_data[0] = (yaw >> 8);
    gimbal_can_send_data[1] = yaw;
    gimbal_can_send_data[2] = (pitch >> 8);
    gimbal_can_send_data[3] = pitch;
    gimbal_can_send_data[4] = (shoot >> 8);
    gimbal_can_send_data[5] = shoot;
    gimbal_can_send_data[6] = (rev >> 8);
    gimbal_can_send_data[7] = rev;
    HAL_CAN_AddTxMessage(&GIMBAL_CAN, &gimbal_tx_message, gimbal_can_send_data, &send_mail_box);
}

/**
  * @brief          send CAN packet of ID 0x700, it will set chassis motor 3508 to quick ID setting
  * @param[in]      none
  * @retval         none
  */
/**
  * @brief          ����IDΪ0x700��CAN��,��������3508��������������ID
  * @param[in]      none
  * @retval         none
  */
void CAN_cmd_chassis_reset_ID(void)
{
    uint32_t send_mail_box;
    chassis_tx_message.StdId = 0x700;
    chassis_tx_message.IDE = CAN_ID_STD;
    chassis_tx_message.RTR = CAN_RTR_DATA;
    chassis_tx_message.DLC = 0x08;
    chassis_can_send_data[0] = 0;
    chassis_can_send_data[1] = 0;
    chassis_can_send_data[2] = 0;
    chassis_can_send_data[3] = 0;
    chassis_can_send_data[4] = 0;
    chassis_can_send_data[5] = 0;
    chassis_can_send_data[6] = 0;
    chassis_can_send_data[7] = 0;

    HAL_CAN_AddTxMessage(&CHASSIS_CAN, &chassis_tx_message, chassis_can_send_data, &send_mail_box);
}


/**
  * @brief          send control current of motor (0x201, 0x202, 0x203, 0x204)
  * @param[in]      motor1: (0x201) 3508 motor control current, range [-16384,16384] 
  * @param[in]      motor2: (0x202) 3508 motor control current, range [-16384,16384] 
  * @param[in]      motor3: (0x203) 3508 motor control current, range [-16384,16384] 
  * @param[in]      motor4: (0x204) 3508 motor control current, range [-16384,16384] 
  * @retval         none
  */
/**
  * @brief          ���͵�����Ƶ���(0x201,0x202,0x203,0x204)
  * @param[in]      motor1: (0x201) 3508������Ƶ���, ��Χ [-16384,16384]
  * @param[in]      motor2: (0x202) 3508������Ƶ���, ��Χ [-16384,16384]
  * @param[in]      motor3: (0x203) 3508������Ƶ���, ��Χ [-16384,16384]
  * @param[in]      motor4: (0x204) 3508������Ƶ���, ��Χ [-16384,16384]
  * @retval         none
  */
void CAN_cmd_chassis(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4)
{
    uint32_t send_mail_box;
    chassis_tx_message.StdId = CAN_CHASSIS_ALL_ID;
    chassis_tx_message.IDE = CAN_ID_STD;
    chassis_tx_message.RTR = CAN_RTR_DATA;
    chassis_tx_message.DLC = 0x08;
    chassis_can_send_data[0] = motor1 >> 8;
    chassis_can_send_data[1] = motor1;
    chassis_can_send_data[2] = motor2 >> 8;
    chassis_can_send_data[3] = motor2;
    chassis_can_send_data[4] = motor3 >> 8;
    chassis_can_send_data[5] = motor3;
    chassis_can_send_data[6] = motor4 >> 8;
    chassis_can_send_data[7] = motor4;

    HAL_CAN_AddTxMessage(&CHASSIS_CAN, &chassis_tx_message, chassis_can_send_data, &send_mail_box);
}

void CAN2_Send_Msg_gimbal(int16_t control2_20A)  //ID 6
{	
	uint32_t send_mail_box;
  CAN2_tx_message.StdId=0x2ff;	 // ��׼��ʶ��Ϊ0
  CAN2_tx_message.IDE = CAN_ID_STD;			//ָ����Ҫ�������Ϣ�ı�ʶ��������
  CAN2_tx_message.RTR = CAN_RTR_DATA;		//ָ����֡�����������Ϣ������   ����֡��Զ��֡
  CAN2_tx_message.DLC = 8;			//ָ�����ݵĳ���
  CAN2_can_send_data[2] = control2_20A>>8;
	CAN2_can_send_data[3] = control2_20A;
	
	HAL_CAN_AddTxMessage(&hcan2, &CAN2_tx_message, CAN2_can_send_data, &send_mail_box);
}

//���͵��̵������
void CAN1_SendCommand_chassis(signed long ESC_201,signed long ESC_202,signed long ESC_203,signed long ESC_204)
{
	uint32_t send_mail_box;
	
	chassis_tx_message.StdId = 0x200;				//����c620���ñ�ʶ��
	chassis_tx_message.IDE = CAN_ID_STD;				//ָ����Ҫ�������Ϣ�ı�ʶ��������
	chassis_tx_message.RTR = CAN_RTR_DATA;			//ָ����֡�����������Ϣ������   ����֡��Զ��֡
	chassis_tx_message.DLC = 8;						//ָ�����ݵĳ���
	chassis_can_send_data[0] = (unsigned char)(ESC_201>>8);
	chassis_can_send_data[1] = (unsigned char)(ESC_201);
	chassis_can_send_data[2] = (unsigned char)(ESC_202>>8);
	chassis_can_send_data[3] = (unsigned char)(ESC_202);
	chassis_can_send_data[4] = (unsigned char)(ESC_203>>8);
	chassis_can_send_data[5] = (unsigned char)(ESC_203);
	chassis_can_send_data[6] = (unsigned char)(ESC_204>>8);
	chassis_can_send_data[7] = (unsigned char)(ESC_204);
	
	HAL_CAN_AddTxMessage(&hcan1, &chassis_tx_message, chassis_can_send_data, &send_mail_box);
}


//���ͳ����������ذ���Ϣ�����з��͵�ʵ��ֵ����Ҫ��100  ���趨��Χ��35w����130w����Ӧ���͵�ֵΪ3500����13000
void CAN1_SendCommand_Powerlimit(uint16_t Target_power)
{
	uint32_t send_mail_box;
	
	Powerlimit_tx_message.StdId = 0x210;				//���ݵ������ذ����ñ�ʶ��
	Powerlimit_tx_message.IDE = CAN_ID_STD;				//ָ����Ҫ�������Ϣ�ı�ʶ��������
	Powerlimit_tx_message.RTR = CAN_RTR_DATA;			//ָ����֡�����������Ϣ������   ����֡��Զ��֡
	Powerlimit_tx_message.DLC = 8;						//ָ�����ݵĳ���
	Powerlimit_can_send_data[0] = (Target_power>>8);  //Ŀ�깦��/�趨����
	Powerlimit_can_send_data[1] = (Target_power);

	HAL_CAN_AddTxMessage(&hcan1, &Powerlimit_tx_message, Powerlimit_can_send_data, &send_mail_box);
}


//can���� control_209 Y��ID7  control_20A P��  control_205 ID5 ���ָǵ��  control_206 ID 6 �������
void CAN1_Send_Msg_gimbal(int16_t control_209,int16_t control_205,int16_t control_206)
{	
	uint32_t send_mail_box;
	
  gimbal_tx_message.StdId=0x2ff;	 									// ��׼��ʶ��Ϊ0
  gimbal_tx_message.IDE = CAN_ID_STD;								//ָ����Ҫ�������Ϣ�ı�ʶ��������
  gimbal_tx_message.RTR = CAN_RTR_DATA;							//ָ����֡�����������Ϣ������   ����֡��Զ��֡
  gimbal_tx_message.DLC = 4;												//ָ�����ݵĳ���
	gimbal_can_send_data[0] = control_209>>8;
	gimbal_can_send_data[1] = control_209;
	
  HAL_CAN_AddTxMessage(&hcan1, &gimbal_tx_message, gimbal_can_send_data, &send_mail_box);
}

////////////////////////////////////////////////////////////////////////
/////////////////////////����Ϊ����Ԥ������////////////////////////////
////////////////////////////////////////////////////////////////////////


//������
float Q=600;					//Ԥ�ⷽ��
float R=5000;					//��������
float variance_kalman[8]={100,100,100,100,100,100,100,100};	//����������ֵ�����ʼ��

float old_kalman[8];				//�ɿ���������ֵ

static int16_t kalman(int16_t x,uint8_t i)			//�������˲�����x�����ݱ�ʶ
{
	//Ԥ��
	float state_pre = old_kalman[i];					//Ԥ��ֵΪ��һ������ֵ
	float variance_pre = variance_kalman[i] + Q;	//Ԥ��ֵ����Ϊ��һ������ֵ����+Ԥ�ⷽ��
	
	//���㿨��������
	float K = variance_pre / (variance_pre + R);
	
	//��ϲ���ֵ��Ԥ��ֵ����У��
	float state_kalman = state_pre + (K *(x-state_pre));	//����������ֵ����
	
	if(state_pre<0) state_pre = -state_pre;
	variance_kalman[i] = (1 - K) * state_pre;				//����������ֵ�������
	
	old_kalman[i] = state_kalman;							//���¾�����ֵ
	
	return state_kalman;								//�������������ֵ
}


//��е��Խ�紦��
static void Angle_deal(int i)		//����iΪ�����ʶ��
{
	motor_data[i].D_Angle = motor_data[i].NowAngle - motor_data[i].OldAngle;
	
	if ((motor_data[i].ActualSpeed<5) && (motor_data[i].ActualSpeed>-5)) motor_data[i].ActualSpeed=0;
	
	if ((motor_data[i].ActualSpeed>=0) && (motor_data[i].D_Angle<-10))
	{
		motor_data[i].D_Angle += 8192;
	}
	else if ((motor_data[i].ActualSpeed<0) && (motor_data[i].D_Angle>10))
	{
		motor_data[i].D_Angle -= 8192; 
	}
	
	//�쳣����
	if ((motor_data[i].D_Angle - old_angle[i])>4000)
		motor_data[i].D_Angle -= 8192;
	else if ((motor_data[i].D_Angle - old_angle[i])<-4000)
		motor_data[i].D_Angle += 8192;
	else
		old_angle[i] = motor_data[i].D_Angle;
}

/**
  * @brief          return the yaw 6020 motor data point
  * @param[in]      none
  * @retval         motor data point
  */
/**
  * @brief          ����yaw 6020�������ָ��
  * @param[in]      none
  * @retval         �������ָ��
  */
const motor_measure_t *get_yaw_gimbal_motor_measure_point(void)
{
    return &motor_chassis[4];
}

/**
  * @brief          return the pitch 6020 motor data point
  * @param[in]      none
  * @retval         motor data point
  */
/**
  * @brief          ����pitch 6020�������ָ��
  * @param[in]      none
  * @retval         �������ָ��
  */
const motor_measure_t *get_pitch_gimbal_motor_measure_point(void)
{
    return &motor_chassis[5];
}


/**
  * @brief          return the trigger 2006 motor data point
  * @param[in]      none
  * @retval         motor data point
  */
/**
  * @brief          ���ز������ 2006�������ָ��
  * @param[in]      none
  * @retval         �������ָ��
  */
const motor_measure_t *get_trigger_motor_measure_point(void)
{
    return &motor_chassis[6];
}


/**
  * @brief          return the chassis 3508 motor data point
  * @param[in]      i: motor number,range [0,3]
  * @retval         motor data point
  */
/**
  * @brief          ���ص��̵�� 3508�������ָ��
  * @param[in]      i: ������,��Χ[0,3]
  * @retval         �������ָ��
  */
const motor_measure_t *get_chassis_motor_measure_point(uint8_t i)
{
    return &motor_chassis[(i & 0x03)];
}
