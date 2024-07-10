#ifndef _PID_H_
#define _PID_H_



#include "Filter.h"

typedef struct
{
 float p;
 float i;
 float d;
}Vector3f_pid;


typedef struct
{
    uint8 Err_Limit_Flag :1;//ƫ���޷���־
    uint8 Integrate_Limit_Flag :1;//�����޷���־
    uint8 Integrate_Separation_Flag :1;//���ַ����־
    float Expect;//����
    float FeedBack;//����ֵ
    float Err;//ƫ��
    float Last_Err;//�ϴ�ƫ��
    float Err_Max;//ƫ���޷�ֵ
    float Integrate_Separation_Err;//���ַ���ƫ��ֵ
    float Integrate;//����ֵ
    float Integrate_Max;//�����޷�ֵ
    float Kp;//���Ʋ���Kp
    float Ki;//���Ʋ���Ki
    float Kd;//���Ʋ���Kd
    float Control_OutPut,Raw_Control_OutPut;//�����������
    float Last_Control_OutPut;//�ϴο����������
    float Control_OutPut_Limit;//����޷�
    /***************************************/
    float Pre_Last_Err;//���ϴ�ƫ��
    float Adaptable_Kd;//����Ӧ΢�ֲ���
    float Last_FeedBack;//�ϴη���ֵ
    float Dis_Err;//΢����
    float Dis_Error_History[5];//��ʷ΢����
    float Err_LPF;
    float Last_Err_LPF;
    float Dis_Err_LPF;
    float Last_Dis_Err_LPF;
    float Pre_Last_Dis_Err_LPF;
    lpf_buf _lpf_buf;//��������ͨ�����������
		systime Systime_t;	
		//
		float d_lpf_alpha;
		float derivative,last_derivative;//�ϴ�΢����	
		
		float Last_Expect;
		float Expect_Div;		//������΢��
		float Feedback_Div;	//�����źŵ�΢��
		float Combine_Div;	//���΢��
}PID_Controler;

typedef struct
{
     PID_Controler Pitch_Angle_Control;
     PID_Controler Pitch_Gyro_Control;
     PID_Controler Roll_Angle_Control;
     PID_Controler Roll_Gyro_Control;
     PID_Controler Yaw_Angle_Control;
     PID_Controler Yaw_Gyro_Control;
     PID_Controler Height_Position_Control;
     PID_Controler Height_Speed_Control;
     PID_Controler Longitude_Position_Control;
     PID_Controler Longitude_Speed_Control;
     PID_Controler Latitude_Position_Control;
     PID_Controler Latitude_Speed_Control;
     /*************���ٶȿ��������¼�****************/
     PID_Controler Height_Acce_Control;
     PID_Controler Reserved_User1_Control;
     PID_Controler Reserved_User2_Control;
     /*************�������������¼�****************/
     PID_Controler Optical_Position_Control;
     PID_Controler Optical_Speed_Control;
     PID_Controler SDK_Roll_Position_Control;
     PID_Controler SDK_Pitch_Position_Control;
		 PID_Controler IMU_Temperature_Control;
}AllControler;

typedef enum
{
     Pitch_Angle_Controler=0,
     Pitch_Gyro_Controler=1,
     Roll_Angle_Controler=2,
     Roll_Gyro_Controler=3,
     Yaw_Angle_Controler=4,
     Yaw_Gyro_Controler=5,
     Height_Position_Controler=6,
     Height_Speed_Controler=7,
		 
     Longitude_Position_Controler=8,
     Longitude_Speed_Controler=9,
     Latitude_Position_Controler=10,
     Latitude_Speed_Controler=11,
     Height_Acce_Controler=12,
     Reserved_User1_Controler=13,
     Reserved_User2_Controler=14,
     Optical_Position_Controler=15,
     Optical_Speed_Controler=16,
     SDK_Roll_Position_Controler=17,
     SDK_Pitch_Position_Controler=18,
     IMU_Temperature_Controler=19	
}Controler_Label;


typedef enum 
{
	direct_diff=0,		//ֱ��΢��
	interval_diff=1,	//����΢��
	incomplete_diff=2,//΢������
}diff_mode;

typedef enum 
{
	noneed_lpf=0,		    //΢�������ͨ
	first_order_lpf=1,	//΢��һ�׵�ͨ
	second_order_lpf=2, //΢�ֶ��׵�ͨ
}lpf_mode;


extern AllControler Total_Controller;


void PID_Paramter_Init_With_Flash(void);
void  Total_PID_Init(void);
void  PID_Init(PID_Controler *Controler,Controler_Label Label);

float PID_Control(PID_Controler *Controler,float period_second);
float PID_Control_Yaw(PID_Controler *Controler,float period_second);
float PID_Control_Div_LPF(PID_Controler *Controler,float period_second);
float PID_Control_Err_LPF(PID_Controler *Controler,float period_second);
float PID_Control_SDK_Err_LPF(PID_Controler *Controler,uint8_t Differential_Enable_Flag,float period_second);
float Differential_Forward_PID_Control_Div_LPF(PID_Controler *Controler,float period_second);
float PID_Control_Div_LPF_For_Gyro(PID_Controler *Controler,float period_second);

float pid_ctrl_rpy_gyro_maple(PID_Controler *ctrl,float period_second,diff_mode _diff_mode,lpf_mode _lpf_mode);;


void  PID_LPF_Reset(PID_Controler *Controler,Controler_Label Label);
void  PID_Integrate_Reset(PID_Controler *Controler);
void  Take_Off_Reset(void);
void East_North_Ctrl_Reset(void);

void Throttle_Control_Reset(void);
void Save_Or_Reset_PID_Parameter(void);
void Save_PID_Parameter(void);
extern uint8_t Sort_PID_Flag;
#endif


