/* Copyright (c)  2019-2030 Wuhan Nameless Innovation Technology Co.,Ltd. All rights reserved.*/
/*----------------------------------------------------------------------------------------------------------------------/
																									��Դ�����������
																									��Դ�����������
																									��Դ�����������
																									��Ҫ������˵����
								�����ߵ���ʷ�Ѿ�֤�����ڵ�ǰ�����Ը���+��ƽ+�ھ�Ĵ󻷾��£����ں�������Ŀ�Դ��Ŀ����������ɿذ����ߡ�
								�����黳ʽ���Է����������ȥ���뿪Դ��Ŀ�ķ�ʽ�в�ͨ���õĿ�Դ��Ŀ��Ҫ��רְ��Ա�����ۺ�����������
								�ֲ����Ƶ�̳�Ҫ�����������ŵ����׽׶Σ�ʹ�ù����ж��û�����������������ͳ�ơ���ʵ������ɶԲ�Ʒ��һ
								�δ����������������
-----------------------------------------------------------------------------------------------------------------------
*                                                 Ϊʲôѡ���������£�
*                                         �ж����ļ۸�������׵Ŀ�Դ�ɿأ�
*                                         ����ҵ������֮������µ��ۺ����
*                                         ׷�����û����飬��Ч����ѧϰ֮·��
*                                         ���²��ٹµ�������������տ�Դ�߶ȣ�
*                                         ��Ӧ���ҷ�ƶ���٣��ٽ��������ƹ�ƽ��
*                                         ��ʱ���ܶ�����ʣ����������˹�ͬ�塣 
-----------------------------------------------------------------------------------------------------------------------
*               ������Ϣ���ܶ���ֹ��ǰ�����������˳���������
*               ��Դ���ף���ѧ����ϧ��ף������Ϯ�����׳ɹ�������
*               ѧϰ�����ߣ��������Ƽ���DJI��ZEROTECH��XAG��AEE��GDU��AUTEL��EWATT��HIGH GREAT�ȹ�˾��ҵ
*               ��ְ�����뷢�ͣ�15671678205@163.com���豸ע��ְ����λ����λ��������
*               �������¿�Դ�ɿ�QQȺ��2��Ⱥ465082224��1��Ⱥ540707961
*               CSDN���ͣ�http://blog.csdn.net/u011992534
*               Bվ��ѧ��Ƶ��https://space.bilibili.com/67803559/#/video				�ſ�ID��NamelessCotrun����С��
*               �������¹����׿�TI��Դ�ɿ���Ƴ��ԡ�֪��ר��:https://zhuanlan.zhihu.com/p/54471146
*               TI�������˻�Ʒ�ʹ�Ӧ�̣���Դ-��ѧ-����-����,�̹� TI MCUϵͳ�� NController�๦�ܿ�����https://item.taobao.com/item.htm?spm=a21n57.1.0.0.7200523c4JP61D&id=697442280363&ns=1&abbucket=19#detail 
*               �Ա����̣�https://shop348646912.taobao.com/?spm=2013.1.1000126.2.5ce78a88ht1sO2
*               ��˾����:www.nameless.tech
*               �޸�����:2024/01/20                  
*               �汾����Ӯ��PRO_V3����CarryPilot_V6.0.5
*               ��Ȩ���У�����ؾ���
*               Copyright(C) 2019-2030 �人�������¿Ƽ����޹�˾ 
*               All rights reserved
-----------------------------------------------------------------------------------------------------------------------
*               ��Ҫ��ʾ��
*               �����Ա�����ת�ֵķɿء��������ѡ�����ѧ�ܵĶ����Խ��ۺ�Ⱥѧϰ������
*               �������������������������ϣ���˾���Ŵ������������Ȩ������Ȩ�����˲��ý�
*               ���ϴ��봫���Ϲ��������أ�������ı��ΪĿȥ�������ϴ��룬�����д�������ߣ�
*               ��˾����ǰ��֪����1���ڼ�ʱ�������������ȨΥ����Ϊ�ᱻ�����ڶ�����
*               ����ͷ�����ټҺš���˾������΢�Ź���ƽ̨���������͡�֪����ƽ̨���Թ�ʾ�ع�
*               ������Ȩ��Ϊ���Ϊ���������۵㣬Ӱ����ѧ���ҹ���������������ܿ�ͺ������˻����������������ء�
*               �����Ϊ����˾����ش���ʧ�ߣ����Է���;���������л���ĺ�����лл������
----------------------------------------------------------------------------------------------------------------------*/
#include "Headfile.h"
#include "Subtask_Demo.h"


uint16_t flight_subtask_cnt[FLIGHT_SUBTASK_NUM]={0};//�����������̼߳��������������ڿ���ÿ���������̵߳�ִ��
uint32_t flight_global_cnt[FLIGHT_SUBTASK_NUM]={0}; //������������ȫ�ּ����������Խ��λ��ƫ�������ж��жϺ����Ƿ񵽴�
uint32_t execute_time_ms[FLIGHT_SUBTASK_NUM]={0};		//������������ִ��ʱ�䣬������������ĳ�����̵߳�ִ��ʱ��
uint32_t flight_global_cnt2[FLIGHT_SUBTASK_NUM]={0};//������������ȫ�ּ�����2
#define flight_subtask_delta 5//5ms
Vector3f base_position;//���ڼ�¼������׼ԭ��λ��

void flight_subtask_reset(void)
{
	for(uint16_t i=0;i<FLIGHT_SUBTASK_NUM;i++)
	{
		flight_subtask_cnt[i]=0;
		execute_time_ms[i]=0;
		flight_global_cnt[i]=0;
		flight_global_cnt2[i]=0;
	}
}


//�������б��ϱر��Ӻ����������ߡ�����
void basic_auto_flight_support(void)
{
	OpticalFlow_Control_Pure(0);//SLAM�������
	Flight.yaw_ctrl_mode=ROTATE;//ƫ������Ϊ�ֶ�ģʽ
	Flight.yaw_outer_control_output  =RC_Data.rc_rpyt[RC_YAW];
	Flight_Alt_Hold_Control(ALTHOLD_MANUAL_CTRL,NUL,NUL);//�߶ȿ���
}



//�������б��ϱر��Ӻ����������ߡ����㡢ͬʱ������ƫ��ָ�����
void ros_flight_support(void)
{
	Flight.yaw_ctrl_mode=ROTATE;//ƫ������Ϊ�ֶ�ģʽ
	if(RC_Data.rc_rpyt[RC_YAW]==0)//��ˮƽң��������)
	{
		if(ngs_nav_ctrl.cmd_vel_update==1)	Flight.yaw_outer_control_output  =ngs_nav_ctrl.cmd_vel_angular_z;
		else Flight.yaw_outer_control_output  =RC_Data.rc_rpyt[RC_YAW];//�����ڹ��󣬿���Ȩ�޽���ң����
	}
	else
	{
		Flight.yaw_outer_control_output  =RC_Data.rc_rpyt[RC_YAW];
		ngs_nav_ctrl.cmd_vel_update=0;//ֻҪ����ң������˲�����ǿ�ƽ��������ٶȿ���
		ngs_nav_ctrl.cmd_vel_during_cnt=0;		
	}		
	OpticalFlow_Control_Pure(0);//SLAM�������
	Flight_Alt_Hold_Control(ALTHOLD_MANUAL_CTRL,NUL,NUL);//�߶ȿ���
}


/***********************************************************************************************************/
//˳ʱ��ת��90�ȣ���ɺ���
void flight_subtask_1(void)
{
	static uint8_t n=0;
	if(flight_subtask_cnt[n]==0)
	{
		Flight.yaw_ctrl_mode=CLOCKWISE;
		Flight.yaw_ctrl_start=1;
		Flight.yaw_outer_control_output  =90;//˳ʱ��90��	
		OpticalFlow_Control_Pure(0);	
		Flight_Alt_Hold_Control(ALTHOLD_MANUAL_CTRL,NUL,NUL);//�߶ȿ���
		flight_subtask_cnt[n]=1;		
	}
	else if(flight_subtask_cnt[n]==1)
	{
		Flight.yaw_ctrl_mode=CLOCKWISE;
		Flight.yaw_outer_control_output  =0;
		OpticalFlow_Control_Pure(0);	
		Flight_Alt_Hold_Control(ALTHOLD_MANUAL_CTRL,NUL,NUL);//�߶ȿ���
		
		if(Flight.yaw_ctrl_end==1)  flight_subtask_cnt[n]=2;//ִ����Ϻ��л�����һ�׶�	
	}
	else if(flight_subtask_cnt[n]==2)
	{
		
		Flight.yaw_ctrl_mode=ROTATE;
		Flight.yaw_outer_control_output  =RC_Data.rc_rpyt[RC_YAW];
		OpticalFlow_Control_Pure(0);
		Flight_Alt_Hold_Control(ALTHOLD_AUTO_VEL_CTRL,NUL,-30);//�߶ȿ���
	}
	else
	{
		basic_auto_flight_support();//��������֧�����
	}
}





/***********************************************************************************************************/
//��ʱ��ת��90�ȣ���ɺ���
void flight_subtask_2(void)
{
	static uint8_t n=1;
	if(flight_subtask_cnt[n]==0)
	{
		Flight.yaw_ctrl_mode=ANTI_CLOCKWISE;
		Flight.yaw_ctrl_start=1;
		Flight.yaw_outer_control_output  =90;//��ʱ��90��	
		OpticalFlow_Control_Pure(0);	
		Flight_Alt_Hold_Control(ALTHOLD_MANUAL_CTRL,NUL,NUL);//�߶ȿ���
		flight_subtask_cnt[n]=1;		
	}
	else if(flight_subtask_cnt[n]==1)
	{
		Flight.yaw_ctrl_mode=ANTI_CLOCKWISE;
		Flight.yaw_outer_control_output  =0;
		OpticalFlow_Control_Pure(0);	
		Flight_Alt_Hold_Control(ALTHOLD_MANUAL_CTRL,NUL,NUL);//�߶ȿ���		
		if(Flight.yaw_ctrl_end==1)  flight_subtask_cnt[n]=2;//ִ����Ϻ��л�����һ�׶�		
	}
	else if(flight_subtask_cnt[n]==2)
	{
		Flight.yaw_ctrl_mode=ROTATE;
		Flight.yaw_outer_control_output  =RC_Data.rc_rpyt[RC_YAW];
		OpticalFlow_Control_Pure(0);
		Flight_Alt_Hold_Control(ALTHOLD_AUTO_VEL_CTRL,NUL,-30);//�߶ȿ���
	}
	else
	{
		basic_auto_flight_support();//��������֧�����
	}
}





/***********************************************************************************************************/
//��10deg/s�Ľ��ٶ�˳ʱ��ת��10000ms����ɺ���
void flight_subtask_3(void)
{
	static uint8_t n=2;
	if(flight_subtask_cnt[n]==0)
	{
		
		Flight.yaw_ctrl_mode=CLOCKWISE_TURN;
		Flight.yaw_ctrl_start=1;
		Flight.yaw_outer_control_output  =10;//��10deg/s�Ľ��ٶ�˳ʱ��ת��10000ms
		Flight.execution_time_ms=10000;//ִ��ʱ��
		OpticalFlow_Control_Pure(0);	
		Flight_Alt_Hold_Control(ALTHOLD_MANUAL_CTRL,NUL,NUL);//�߶ȿ���
		flight_subtask_cnt[n]=1;		
	}
	else if(flight_subtask_cnt[n]==1)
	{
		Flight.yaw_ctrl_mode=CLOCKWISE_TURN;
		Flight.yaw_outer_control_output  =0;
		OpticalFlow_Control_Pure(0);	
		Flight_Alt_Hold_Control(ALTHOLD_MANUAL_CTRL,NUL,NUL);//�߶ȿ���
		
		if(Flight.yaw_ctrl_end==1)  flight_subtask_cnt[n]=2;//ִ����Ϻ��л�����һ�׶�		
	}
	else if(flight_subtask_cnt[n]==2)
	{
		
		Flight.yaw_ctrl_mode=ROTATE;
		Flight.yaw_outer_control_output  =RC_Data.rc_rpyt[RC_YAW];

		OpticalFlow_Control_Pure(0);
		Flight_Alt_Hold_Control(ALTHOLD_AUTO_VEL_CTRL,NUL,-30);//�߶ȿ���
	}
	else
	{
		basic_auto_flight_support();//��������֧�����
	}
}





/***********************************************************************************************************/
//��10deg/s�Ľ��ٶ���ʱ��ת��10000ms����ɺ���
void flight_subtask_4(void)
{
	static uint8_t n=3;
	if(flight_subtask_cnt[n]==0)
	{	
		Flight.yaw_ctrl_mode=ANTI_CLOCKWISE_TURN;
		Flight.yaw_ctrl_start=1;
		Flight.yaw_outer_control_output  =10;//��10deg/s�Ľ��ٶ�˳ʱ��ת��10000ms
		Flight.execution_time_ms=10000;//ִ��ʱ��
		OpticalFlow_Control_Pure(0);	
		Flight_Alt_Hold_Control(ALTHOLD_MANUAL_CTRL,NUL,NUL);//�߶ȿ���
		flight_subtask_cnt[n]=1;		
	}
	else if(flight_subtask_cnt[n]==1)
	{
		Flight.yaw_ctrl_mode=ANTI_CLOCKWISE_TURN;
		Flight.yaw_outer_control_output  =0;
		OpticalFlow_Control_Pure(0);	
		Flight_Alt_Hold_Control(ALTHOLD_MANUAL_CTRL,NUL,NUL);//�߶ȿ���
		
		if(Flight.yaw_ctrl_end==1)  flight_subtask_cnt[n]=2;//ִ����Ϻ��л�����һ�׶�		
	}
	else if(flight_subtask_cnt[n]==2)
	{
		Flight.yaw_ctrl_mode=ROTATE;
		Flight.yaw_outer_control_output  =RC_Data.rc_rpyt[RC_YAW];

		OpticalFlow_Control_Pure(0);
		Flight_Alt_Hold_Control(ALTHOLD_AUTO_VEL_CTRL,NUL,-30);//�߶ȿ���
	}
	else
	{
		basic_auto_flight_support();//��������֧�����
	}
}






//��demo�����ڼ����״�SLAM��λ�����£���ͨ������LC307��LC302����λ��������Ч
//��������ϵ�����λ��
//��ǰ�Ϸֱ��ӦXYZ������
void flight_subtask_5(void)
{		
	static uint8_t n=4;
	if(flight_subtask_cnt[n]==0)
	{
		basic_auto_flight_support();//��������֧�����
		flight_subtask_cnt[n]=1;
		execute_time_ms[n]=10000/flight_subtask_delta;//������ִ��ʱ��
		//��ǰ�ƶ�100cm
		Horizontal_Navigation(0,100,0,RELATIVE_MODE,BODY_FRAME);
	}
	else if(flight_subtask_cnt[n]==1)
	{
		basic_auto_flight_support();//��������֧�����
		if(execute_time_ms[n]>0) execute_time_ms[n]--;
		if(execute_time_ms[n]==0) 
		{
			flight_subtask_cnt[n]=2;
			execute_time_ms[n]=10000/flight_subtask_delta;//������ִ��ʱ��
			//�����ƶ�100cm
			Horizontal_Navigation(100,0,0,RELATIVE_MODE,BODY_FRAME);
		}
	}
	else if(flight_subtask_cnt[n]==2)
	{
		basic_auto_flight_support();//��������֧�����
		if(execute_time_ms[n]>0) execute_time_ms[n]--;
		if(execute_time_ms[n]==0) 
		{
			flight_subtask_cnt[n]=3;
			execute_time_ms[n]=10000/flight_subtask_delta;//������ִ��ʱ��
			//����ƶ�100cm
			Horizontal_Navigation(0,-100,0,RELATIVE_MODE,BODY_FRAME);
		}
	}
	else if(flight_subtask_cnt[n]==3)
	{
		basic_auto_flight_support();//��������֧�����
		if(execute_time_ms[n]>0) execute_time_ms[n]--;
		if(execute_time_ms[n]==0) 
		{
			flight_subtask_cnt[n]=4;
			execute_time_ms[n]=10000/flight_subtask_delta;//������ִ��ʱ��
			//�����ƶ�100cm
			Horizontal_Navigation(-100,0,0,RELATIVE_MODE,BODY_FRAME);
		}
	}
	else if(flight_subtask_cnt[n]==4)
	{
		basic_auto_flight_support();//��������֧�����
		if(execute_time_ms[n]>0) execute_time_ms[n]--;
		if(execute_time_ms[n]==0) 
		{
			flight_subtask_cnt[n]=5;
			execute_time_ms[n]=10000/flight_subtask_delta;//������ִ��ʱ��
		  
			//���»����Լ������������񣬱��������ƶ�50cm
			//Horizontal_Navigation(0,0,50,RELATIVE_FRAME,MAP_FRAME);
		}
	}
	else
	{
		basic_auto_flight_support();//��������֧�����
	}
}

//��������ϵ�����λ��
//������ֱ��ӦXYZ������
void flight_subtask_6(void)
{		
	static uint8_t n=5;
	if(flight_subtask_cnt[n]==0)
	{
		basic_auto_flight_support();//��������֧�����
		flight_subtask_cnt[n]=1;
		execute_time_ms[n]=10000/flight_subtask_delta;//������ִ��ʱ��
		//��ǰ�ƶ�100cm
		Horizontal_Navigation(0,100,0,RELATIVE_MODE,MAP_FRAME);
	}
	else if(flight_subtask_cnt[n]==1)
	{
		basic_auto_flight_support();//��������֧�����
		if(execute_time_ms[n]>0) execute_time_ms[n]--;
		if(execute_time_ms[n]==0) 
		{
			flight_subtask_cnt[n]=2;
			execute_time_ms[n]=10000/flight_subtask_delta;//������ִ��ʱ��
			//�����ƶ�100cm
			Horizontal_Navigation(100,0,0,RELATIVE_MODE,MAP_FRAME);
		}
	}
	else if(flight_subtask_cnt[n]==2)
	{
		basic_auto_flight_support();//��������֧�����
		if(execute_time_ms[n]>0) execute_time_ms[n]--;
		if(execute_time_ms[n]==0) 
		{
			flight_subtask_cnt[n]=3;
			execute_time_ms[n]=10000/flight_subtask_delta;//������ִ��ʱ��
			//����ƶ�100cm
			Horizontal_Navigation(0,-100,0,RELATIVE_MODE,MAP_FRAME);
		}
	}
	else if(flight_subtask_cnt[n]==3)
	{
		basic_auto_flight_support();//��������֧�����
		if(execute_time_ms[n]>0) execute_time_ms[n]--;
		if(execute_time_ms[n]==0) 
		{
			flight_subtask_cnt[n]=4;
			execute_time_ms[n]=10000/flight_subtask_delta;//������ִ��ʱ��
			//�����ƶ�100cm
			Horizontal_Navigation(-100,0,0,RELATIVE_MODE,MAP_FRAME);
		}
	}
	else if(flight_subtask_cnt[n]==4)
	{
		basic_auto_flight_support();//��������֧�����
		if(execute_time_ms[n]>0) execute_time_ms[n]--;
		if(execute_time_ms[n]==0) 
		{
			flight_subtask_cnt[n]=5;
			execute_time_ms[n]=10000/flight_subtask_delta;//������ִ��ʱ��
		  
			//���»����Լ������������񣬱��������ƶ�50cm
			//Horizontal_Navigation(0,0,50,RELATIVE_FRAME,MAP_FRAME);
		}
	}
	else
	{
		basic_auto_flight_support();//��������֧�����
	}
}


//��������ϵ�£����ڳ�ʼ��ľ�������λ��
void flight_subtask_7(void)
{		
	static uint8_t n=6;
	Vector3f target_position;
	float x=0,y=0,z=0;
	if(flight_subtask_cnt[n]==0)
	{
		basic_auto_flight_support();//��������֧�����
		//��¼�³�ʼ���λ�ã�ʵ����Ŀ�п�����Ϊĳһ��׼ԭ��
		base_position.x=VIO_SINS.Position[_EAST];
		base_position.y=VIO_SINS.Position[_NORTH];
		base_position.z=NamelessQuad.Position[_UP];
		
		flight_subtask_cnt[n]=1;
		execute_time_ms[n]=10000/flight_subtask_delta;//������ִ��ʱ��
		//��ǰ�ƶ�100cm
		x=0;y=100;z=0;
		target_position.x=base_position.x+x;
		target_position.y=base_position.y+y;
		target_position.z=base_position.z+z;
		Horizontal_Navigation(target_position.x,
												  target_position.y,
												  target_position.z,
												  GLOBAL_MODE,
												  MAP_FRAME);
	}
	else if(flight_subtask_cnt[n]==1)
	{
		basic_auto_flight_support();//��������֧�����
		if(execute_time_ms[n]>0) execute_time_ms[n]--;
		if(execute_time_ms[n]==0) 
		{
			flight_subtask_cnt[n]=2;
			execute_time_ms[n]=10000/flight_subtask_delta;//������ִ��ʱ��
			//�����ƶ�100cm
			x=100;y=100;z=0;
			target_position.x=base_position.x+x;
			target_position.y=base_position.y+y;
			target_position.z=base_position.z+z;
			Horizontal_Navigation(target_position.x,
														target_position.y,
														target_position.z,
														GLOBAL_MODE,
														MAP_FRAME);
		}
	}
	else if(flight_subtask_cnt[n]==2)
	{
		basic_auto_flight_support();//��������֧�����
		if(execute_time_ms[n]>0) execute_time_ms[n]--;
		if(execute_time_ms[n]==0) 
		{
			flight_subtask_cnt[n]=3;
			execute_time_ms[n]=10000/flight_subtask_delta;//������ִ��ʱ��
			//����ƶ�100cm
			x=100;y=0;z=0;
			target_position.x=base_position.x+x;
			target_position.y=base_position.y+y;
			target_position.z=base_position.z+z;
			Horizontal_Navigation(target_position.x,
														target_position.y,
														target_position.z,
														GLOBAL_MODE,
														MAP_FRAME);
		}
	}
	else if(flight_subtask_cnt[n]==3)
	{
		basic_auto_flight_support();//��������֧�����
		if(execute_time_ms[n]>0) execute_time_ms[n]--;
		if(execute_time_ms[n]==0) 
		{
			flight_subtask_cnt[n]=4;
			execute_time_ms[n]=10000/flight_subtask_delta;//������ִ��ʱ��
			//�����ƶ�100cm
			x=0;y=0;z=0;
			target_position.x=base_position.x+x;
			target_position.y=base_position.y+y;
			target_position.z=base_position.z+z;
			Horizontal_Navigation(target_position.x,
														target_position.y,
														target_position.z,
														GLOBAL_MODE,
														MAP_FRAME);
		}
	}
	else if(flight_subtask_cnt[n]==4)
	{
		basic_auto_flight_support();//��������֧�����
		if(execute_time_ms[n]>0) execute_time_ms[n]--;
		if(execute_time_ms[n]==0) 
		{
			flight_subtask_cnt[n]=5;
			execute_time_ms[n]=10000/flight_subtask_delta;//������ִ��ʱ��
		  
			//���»����Լ������������񣬱��������ƶ�50cm
			//Horizontal_Navigation(0,0,50,RELATIVE_FRAME,MAP_FRAME);
		}
	}
	else
	{
		basic_auto_flight_support();//��������֧�����
	}
}


//�켣Բ
#define Waypoint_Radius_CM     50 									//�켣��뾶����λΪcm   100
#define Waypoint_Num           60				 						//�켣������						 60
//�ı�켣Բ������һ�������ֻ��Ҫ�����뾶Waypoint_Radius_CM����
#define Waypoint_Angle_Delta   (360/Waypoint_Num)		//����Ƕ���������λΪdeg
#define Waypoint_Ideal_Speed	 20										//�����Ѳ���ٶȣ��������㺽�㷢��ʱ�䣬��λΪcm/s
#define Waypoint_Time_Gap			 1000*(2*3.14*Waypoint_Radius_CM/Waypoint_Ideal_Speed)/Waypoint_Num		//���㷢��ʱ��������λΪms
float waypoint_x[Waypoint_Num+1],waypoint_y[Waypoint_Num+1];
void Circle_Waypoint_Generate(uint16_t num,float radius)
{
	for(uint16_t i=0;i<num+1;i++)
	{
		waypoint_x[i]=radius-radius*cos(Waypoint_Angle_Delta*i*DEG2RAD);
		waypoint_y[i]=radius*sin(Waypoint_Angle_Delta*i*DEG2RAD);
	}
}


void flight_subtask_8(void)
{		
	static uint8_t n=7;
	Vector3f target_position;
	basic_auto_flight_support();//��������֧�����
	if(flight_subtask_cnt[n]==0)
	{
		Circle_Waypoint_Generate(Waypoint_Num,Waypoint_Radius_CM);
		//��¼�³�ʼ���λ�ã�ʵ����Ŀ�п�����Ϊĳһ��׼ԭ��
		base_position.x=VIO_SINS.Position[_EAST];
		base_position.y=VIO_SINS.Position[_NORTH];
		base_position.z=NamelessQuad.Position[_UP];
		
		execute_time_ms[n]=Waypoint_Time_Gap/flight_subtask_delta;//������ִ��ʱ��
		target_position.x=base_position.x+waypoint_x[0];
		target_position.y=base_position.y+waypoint_y[0];
		target_position.z=base_position.z;
		Horizontal_Navigation(target_position.x,
													target_position.y,
													target_position.z,
													GLOBAL_MODE,
													MAP_FRAME);
		flight_subtask_cnt[n]=1;
		laser_light1.reset=1;
		laser_light1.times=50000;//����1��
		laser_light1.period=40;
		laser_light1.light_on_percent=0.5;
	}
	else if(flight_subtask_cnt[n]<Waypoint_Num+1)
	{
		if(execute_time_ms[n]>0) execute_time_ms[n]--;
		if(execute_time_ms[n]==0) 
		{
			execute_time_ms[n]=Waypoint_Time_Gap/flight_subtask_delta;//������ִ��ʱ��
			
			target_position.x=base_position.x+waypoint_x[flight_subtask_cnt[n]];
			target_position.y=base_position.y+waypoint_y[flight_subtask_cnt[n]];
			target_position.z=base_position.z;
			Horizontal_Navigation(target_position.x,
														target_position.y,
														target_position.z,
														GLOBAL_MODE,
														MAP_FRAME);
			flight_subtask_cnt[n]++;
		}
	}
	else//һֱ�ظ�Բ�ι켣��ֻ����һ�ο���ע�͵��·���ֵ��
	{
		flight_subtask_cnt[n]=1;
	}
}

/*****************************************************************************************/
const int16_t work_waypoints_table[2][32]={
{0,1,1,2,2,3,3,4,5,6,7,7,6,5,4,4,5,6,7,7,6,5,4,4,5,6,7,7,6,5,4,0},
{0,4,5,5,4,4,5,5,5,5,5,4,4,4,4,3,3,3,3,2,2,2,2,1,1,1,1,0,0,0,0,0}
};
float work_waypoints[2][32]={0};
uint16_t work_time_gap[32]={
3000,
10000,
3000,3000,3000,3000,3000,3000,3000,3000,3000,
3000,3000,3000,3000,3000,3000,3000,3000,3000,3000,
3000,3000,3000,3000,3000,3000,3000,3000,3000,3000,
10000
};
#define Scale_Param  50.0f
void work_waypoint_generate(void)
{
	for(uint16_t i=0;i<32;i++)
	{
		work_waypoints[0][i]=Scale_Param*work_waypoints_table[0][i];
		work_waypoints[1][i]=Scale_Param*work_waypoints_table[1][i];
	}

}


void Agriculture_UAV_Basic(void)
{		
	static uint8_t n=9;
	Vector3f target_position;	
	if(flight_subtask_cnt[n]==0)//��ɵ���Ϊ��һ����ͣ��
	{
		basic_auto_flight_support();//��������֧�����
		work_waypoint_generate();
		//��¼�³�ʼ���λ�ã�ʵ����Ŀ�п�����Ϊĳһ��׼ԭ��
		base_position.x=VIO_SINS.Position[_EAST];
		base_position.y=VIO_SINS.Position[_NORTH];
		base_position.z=NamelessQuad.Position[_UP];
		
		execute_time_ms[n]=work_time_gap[0]/flight_subtask_delta;//������ִ��ʱ��
		target_position.x=base_position.x+work_waypoints[0][0];
		target_position.y=base_position.y+work_waypoints[1][0];
		target_position.z=base_position.z;
		Horizontal_Navigation(target_position.x,
													target_position.y,
													target_position.z,
													GLOBAL_MODE,
													MAP_FRAME);
		flight_subtask_cnt[n]=1;		
	}
	else if(flight_subtask_cnt[n]==1)//��ɵ���ͣ��Ϻ󣬷���Ŀ��A������21����ҵ����
	{
		basic_auto_flight_support();//��������֧�����
		if(execute_time_ms[n]>0) execute_time_ms[n]--;
		if(execute_time_ms[n]==0) 
		{
			execute_time_ms[n]=work_time_gap[1]/flight_subtask_delta;//������ִ��ʱ��
			
			target_position.x=base_position.x+work_waypoints[0][1];
			target_position.y=base_position.y+work_waypoints[1][1];
			target_position.z=base_position.z;
			Horizontal_Navigation(target_position.x,
														target_position.y,
														target_position.z,
														GLOBAL_MODE,
														MAP_FRAME);
			flight_subtask_cnt[n]++;
		}	
	}	
	else if(flight_subtask_cnt[n]<31)//����A��������ҵ��󣬱������к��㲢����Ƿ���
	{
		basic_auto_flight_support();//��������֧�����
		if(execute_time_ms[n]>0) execute_time_ms[n]--;
		if(execute_time_ms[n]==0) 
		{
			if(Opv_Top_View_Target.trust_flag==1)//����Ŀ�꺽����жϵײ��Ƿ�Ϊũ����
			{
				laser_light1.reset=1;
				laser_light1.times=1;//��������
			}
				
			execute_time_ms[n]=work_time_gap[flight_subtask_cnt[n]]/flight_subtask_delta;//������ִ��ʱ��		
			target_position.x=base_position.x+work_waypoints[0][flight_subtask_cnt[n]];
			target_position.y=base_position.y+work_waypoints[1][flight_subtask_cnt[n]];
			target_position.z=base_position.z;
			Horizontal_Navigation(target_position.x,
														target_position.y,
														target_position.z,
														GLOBAL_MODE,
														MAP_FRAME);
			flight_subtask_cnt[n]++;
		}
	}
	else if(flight_subtask_cnt[n]==31)//�������һ����ҵ�㣬����Ƿ��㣬��󷵻���ɵ�
	{
		basic_auto_flight_support();//��������֧�����
		if(execute_time_ms[n]>0) execute_time_ms[n]--;
		if(execute_time_ms[n]==0) 
		{
			if(Opv_Top_View_Target.trust_flag==1)//����Ŀ�꺽����жϵײ��Ƿ�Ϊũ����
			{
				laser_light1.reset=1;
				laser_light1.times=1;//����1��
			}	
			execute_time_ms[n]=work_time_gap[31]/flight_subtask_delta;//������ִ��ʱ��
			target_position.x=base_position.x+work_waypoints[0][31];
			target_position.y=base_position.y+work_waypoints[1][31];
			target_position.z=base_position.z;
			Horizontal_Navigation(target_position.x,
														target_position.y,
														target_position.z,
														GLOBAL_MODE,
														MAP_FRAME);
			flight_subtask_cnt[n]++;
		}	
	}
	else if(flight_subtask_cnt[n]==32)//������ɵ�
	{
		basic_auto_flight_support();//��������֧�����
		if(execute_time_ms[n]>0) execute_time_ms[n]--;
		if(execute_time_ms[n]==0) 
		{
			execute_time_ms[n]=2000/flight_subtask_delta;//������ִ��ʱ��
			
			flight_subtask_cnt[n]++;
		}			
	}
	else if(flight_subtask_cnt[n]==33)//ԭ�ؽ���
	{
		
		Flight.yaw_ctrl_mode=ROTATE;
		Flight.yaw_outer_control_output  =RC_Data.rc_rpyt[RC_YAW];
		OpticalFlow_Control_Pure(0);
		Flight_Alt_Hold_Control(ALTHOLD_AUTO_VEL_CTRL,NUL,-30);//�߶ȿ���
	}
	else
	{
		basic_auto_flight_support();//��������֧�����
	}
}



#define Waypoint_Fix_Cnt1   5 //5
#define Waypoint_Fix_CM1    5//5
//λ��kp=400
void Agriculture_UAV_Closeloop(void)
{		
	static uint8_t n=10;
	Vector3f target_position;	
	if(flight_subtask_cnt[n]==0)//��ɵ���Ϊ��һ����ͣ��
	{
		basic_auto_flight_support();//��������֧�����
		work_waypoint_generate();
		//��¼�³�ʼ���λ�ã�ʵ����Ŀ�п�����Ϊĳһ��׼ԭ��
		//base_position.x=VIO_SINS.Position[_EAST];
		//base_position.y=VIO_SINS.Position[_NORTH];
		base_position.z=NamelessQuad.Position[_UP];
		
		execute_time_ms[n]=work_time_gap[0]/flight_subtask_delta;//������ִ��ʱ��
		target_position.x=base_position.x+work_waypoints[0][0];
		target_position.y=base_position.y+work_waypoints[1][0];
		target_position.z=base_position.z;
		Horizontal_Navigation(target_position.x,
													target_position.y,
													target_position.z,
													GLOBAL_MODE,
													MAP_FRAME);
		flight_subtask_cnt[n]=1;
		flight_global_cnt[n]=0;
	}
	else if(flight_subtask_cnt[n]==1)//�����ɵ���ͣ��Ϻ󣬷���Ŀ��A������21����ҵ����
	{
		basic_auto_flight_support();//��������֧�����
		
		//�ж��Ƿ񵽴�Ŀ�꺽��λ��
		if(flight_global_cnt[n]<Waypoint_Fix_Cnt1)//����4*5ms=0.05s����
		{
			float dis_cm=pythagorous2(OpticalFlow_Pos_Ctrl_Err.x,OpticalFlow_Pos_Ctrl_Err.y);
			if(dis_cm<=Waypoint_Fix_CM1)	flight_global_cnt[n]++;
			else flight_global_cnt[n]/=2;
		}
		else//����4*5ms���㣬��ʾ����Ŀ�꺽��λ��
		{
			target_position.x=base_position.x+work_waypoints[0][1];
			target_position.y=base_position.y+work_waypoints[1][1];
			target_position.z=base_position.z;
			Horizontal_Navigation(target_position.x,
														target_position.y,
														target_position.z,
														GLOBAL_MODE,
														MAP_FRAME);
			flight_subtask_cnt[n]++;
			flight_global_cnt[n]=0;		
			
			//flight_subtask_cnt[n]=30;//�����û������ý׶ε����ã���ʡʱ��			
		}
	}	
	else if(flight_subtask_cnt[n]<31)//����A��������ҵ��󣬱������к��㲢����Ƿ���
	{
		basic_auto_flight_support();//��������֧�����
		//�ж��Ƿ񵽴�Ŀ�꺽��λ��
		if(flight_global_cnt[n]<Waypoint_Fix_Cnt1)//����4*5ms=0.05s����
		{
			float dis_cm=pythagorous2(OpticalFlow_Pos_Ctrl_Err.x,OpticalFlow_Pos_Ctrl_Err.y);
			if(dis_cm<=Waypoint_Fix_CM1)	flight_global_cnt[n]++;
			else flight_global_cnt[n]/=2;
		}
		else//����4*5ms���㣬��ʾ����Ŀ�꺽��λ��
		{
			execute_time_ms[n]=work_time_gap[flight_subtask_cnt[n]]/flight_subtask_delta;//������ִ��ʱ��		
			target_position.x=base_position.x+work_waypoints[0][flight_subtask_cnt[n]];
			target_position.y=base_position.y+work_waypoints[1][flight_subtask_cnt[n]];
			target_position.z=base_position.z;
			Horizontal_Navigation(target_position.x,
														target_position.y,
														target_position.z,
														GLOBAL_MODE,
														MAP_FRAME);
			flight_subtask_cnt[n]++;
			flight_global_cnt[n]=0;				
			
			if(Opv_Top_View_Target.trust_flag==1)//����Ŀ�꺽����жϵײ��Ƿ�Ϊũ����
			{
				laser_light1.reset=1;
				laser_light1.times=1;//��˸1��
			}		
		}
	}
	else if(flight_subtask_cnt[n]==31)//�������һ�����㵥������
	{
		basic_auto_flight_support();//��������֧�����
		//�ж��Ƿ񵽴�Ŀ�꺽��λ��
		if(flight_global_cnt[n]<Waypoint_Fix_Cnt1)//����4*5ms=0.05s����
		{
			float dis_cm=pythagorous2(OpticalFlow_Pos_Ctrl_Err.x,OpticalFlow_Pos_Ctrl_Err.y);
			if(dis_cm<=Waypoint_Fix_CM1)	flight_global_cnt[n]++;
			else flight_global_cnt[n]/=2;
		}
		else//����4*5ms���㣬��ʾ����Ŀ�꺽��λ��
		{
			execute_time_ms[n]=1000/flight_subtask_delta;//������ִ��ʱ��
			flight_subtask_cnt[n]++;
			flight_global_cnt[n]=0;		
			if(Opv_Top_View_Target.trust_flag==1)//����Ŀ�꺽����жϵײ��Ƿ�Ϊũ����
			{
				laser_light1.reset=1;
				laser_light1.times=1;//��˸1��
			}			
		}			
	}
	else if(flight_subtask_cnt[n]==32)//��󷵻���ɵ�
	{
		basic_auto_flight_support();//��������֧�����
		
		if(execute_time_ms[n]>0) execute_time_ms[n]--;
		if(execute_time_ms[n]==0) 
		{
			execute_time_ms[n]=work_time_gap[31]/flight_subtask_delta;//������ִ��ʱ��
			target_position.x=base_position.x+work_waypoints[0][31];
			target_position.y=base_position.y+work_waypoints[1][31];
			target_position.z=base_position.z;
			Horizontal_Navigation(target_position.x,
														target_position.y,
														target_position.z,
														GLOBAL_MODE,
														MAP_FRAME);
			flight_subtask_cnt[n]++;
		}	
	}
	else if(flight_subtask_cnt[n]==33)//������ɵ�
	{
		basic_auto_flight_support();//��������֧�����
		//�ж��Ƿ񵽴�Ŀ�꺽��λ��
		if(flight_global_cnt[n]<Waypoint_Fix_Cnt1)//����4*5ms=0.05s����
		{
			float dis_cm=pythagorous2(OpticalFlow_Pos_Ctrl_Err.x,OpticalFlow_Pos_Ctrl_Err.y);
			if(dis_cm<=Waypoint_Fix_CM1)	flight_global_cnt[n]++;
			else flight_global_cnt[n]/=2;
		}
		else
		{
			flight_subtask_cnt[n]++;
			flight_global_cnt[n]=0;
			
			//���¸�λ���������ã����е����治ͬ�߶ȣ������ֲ�����仯��ɵ����
			//��ͬ�߶��ݶ��ϳ���仯����ʱ���������⴦���ֿ���ȥ��
			//���⴦��ʼ
//			send_check_back=4;//����slam
//			VIO_SINS.Position[_EAST] = 0;
//			VIO_SINS.Position[_NORTH]= 0;
//		  OpticalFlow_Pos_Ctrl_Expect.x=0;
//		  OpticalFlow_Pos_Ctrl_Expect.y=0;
			//���⴦�����
		}		
	}
	else if(flight_subtask_cnt[n]==34)//ԭ�ؽ���
	{	
		Flight.yaw_ctrl_mode=ROTATE;
		Flight.yaw_outer_control_output  =RC_Data.rc_rpyt[RC_YAW];
		OpticalFlow_Control_Pure(0);
		Flight_Alt_Hold_Control(ALTHOLD_AUTO_VEL_CTRL,NUL,-30);//�߶ȿ���
		//��Ѳ���߶����ڻ�������滷������ƫ��ʱ������ˮƽ����ֻʹ���ٶȿ��ƽ��䣬����λ�ù۲������ɵĽ����ƫ��
		//OpticalFlow_X_Vel_Control(0);
		//OpticalFlow_Y_Vel_Control(0);
	}
	else
	{
		basic_auto_flight_support();//��������֧�����
	}
}



	



#define Barcode_Height_CM   135//���������ڸ߶�
#define Waypoint_Fix_Cnt2   5
#define Waypoint_Fix_CM2    5
float min_dis_cm=0,min_dis_angle=0;
float min_dis_cm_backups[Laser_Min_Info_Num],min_dis_angle_backups[Laser_Min_Info_Num];
float target_yaw_err=0;
uint8_t header_fix_flag=1;//��ͷ��׼���˱�־λ
uint16_t barcode_id=0;//�����ʾ��������Ϣ
uint8_t barcode_flag=0;//����ʶ��ɹ���־λ
void Agriculture_UAV_Innovation(void)//���Ӳ���
{		
	if(ABS(min_dis_cm_backups[0]-min_dis_cm_backups[1])>100.0f)//���־������ͻ�䣬�����ϴμ����Ϣ
	{
		min_dis_cm_backups[0]		=min_dis_cm_backups[1];//������ǰ�ľ���ͽǶ���Ϣ
		min_dis_angle_backups[0]=min_dis_angle_backups[1];//������ǰ�ľ���ͽǶ���Ϣ
	}
	min_dis_cm	 =min_dis_cm_backups[0];
	min_dis_angle=min_dis_angle_backups[0];	
	
	if(min_dis_angle>180) min_dis_angle=min_dis_angle-360;
	target_yaw_err=min_dis_angle;
	
	static uint8_t n=9;
	Vector3f target_position;	
	if(flight_subtask_cnt[n]==0)//��ɵ���Ϊ��һ����ͣ��
	{
		basic_auto_flight_support();//��������֧�����
		work_waypoint_generate();
		//��¼�³�ʼ���λ�ã�ʵ����Ŀ�п�����Ϊĳһ��׼ԭ��
		base_position.x=VIO_SINS.Position[_EAST];
		base_position.y=VIO_SINS.Position[_NORTH];
		base_position.z=NamelessQuad.Position[_UP];
		
		execute_time_ms[n]=work_time_gap[0]/flight_subtask_delta;//������ִ��ʱ��
		target_position.x=base_position.x+work_waypoints[0][0];
		target_position.y=base_position.y+work_waypoints[1][0];
		target_position.z=base_position.z;
		Horizontal_Navigation(target_position.x,
													target_position.y,
													target_position.z,
													GLOBAL_MODE,
													MAP_FRAME);
		flight_subtask_cnt[n]=1;
		flight_global_cnt[n]=0;
		
		//flight_subtask_cnt[n]=30;//�����û������ý׶ε����ã���ʡʱ��
	}
	else if(flight_subtask_cnt[n]==1)//�����ɵ���ͣ��Ϻ󣬷���Ŀ��A������21����ҵ����
	{
		basic_auto_flight_support();//��������֧�����
		
		//�ж��Ƿ񵽴�Ŀ�꺽��λ��
		if(flight_global_cnt[n]<Waypoint_Fix_Cnt2)//����10*5ms=0.05s����
		{
			float dis_cm=pythagorous2(OpticalFlow_Pos_Ctrl_Err.x,OpticalFlow_Pos_Ctrl_Err.y);
			if(dis_cm<=Waypoint_Fix_CM2)	flight_global_cnt[n]++;
			else flight_global_cnt[n]/=2;
		}
		else//����10*5ms���㣬��ʾ����Ŀ�꺽��λ��
		{
			target_position.x=base_position.x+work_waypoints[0][1];
			target_position.y=base_position.y+work_waypoints[1][1];
			target_position.z=base_position.z;
			Horizontal_Navigation(target_position.x,
														target_position.y,
														target_position.z,
														GLOBAL_MODE,
														MAP_FRAME);
			flight_subtask_cnt[n]++;
			flight_global_cnt[n]=0;
			//flight_subtask_cnt[n]=30;//��������Ѱ�ˡ�������ʶ������ʱ�����ڴ˸�ֵ��������㲽��
		}
		
	}	
	else if(flight_subtask_cnt[n]<31)//����A��������ҵ��󣬱������к��㲢����Ƿ���
	{
		basic_auto_flight_support();//��������֧�����
		
		//�ж��Ƿ񵽴�Ŀ�꺽��λ��
		if(flight_global_cnt[n]<Waypoint_Fix_Cnt2)//����10*5ms=0.05s����
		{
			float dis_cm=pythagorous2(OpticalFlow_Pos_Ctrl_Err.x,OpticalFlow_Pos_Ctrl_Err.y);
			if(dis_cm<=Waypoint_Fix_CM2)	flight_global_cnt[n]++;
			else flight_global_cnt[n]/=2;
		}
		else//����10*5ms���㣬��ʾ����Ŀ�꺽��λ��
		{
			execute_time_ms[n]=work_time_gap[flight_subtask_cnt[n]]/flight_subtask_delta;//������ִ��ʱ��		
			target_position.x=base_position.x+work_waypoints[0][flight_subtask_cnt[n]];
			target_position.y=base_position.y+work_waypoints[1][flight_subtask_cnt[n]];
			target_position.z=base_position.z;
			Horizontal_Navigation(target_position.x,
														target_position.y,
														target_position.z,
														GLOBAL_MODE,
														MAP_FRAME);
			flight_subtask_cnt[n]++;
			flight_global_cnt[n]=0;				
			
			if(Opv_Top_View_Target.trust_flag==1)//����Ŀ�꺽����жϵײ��Ƿ�Ϊũ����
			{
				laser_light1.reset=1;
				laser_light1.times=1;//��˸1��
			}		
		}
	}
	else if(flight_subtask_cnt[n]==31)//�������һ�����㵥������
	{
		basic_auto_flight_support();//��������֧�����
		//�ж��Ƿ񵽴�Ŀ�꺽��λ��
		if(flight_global_cnt[n]<Waypoint_Fix_Cnt2)//����4*5ms=0.05s����
		{
			float dis_cm=pythagorous2(OpticalFlow_Pos_Ctrl_Err.x,OpticalFlow_Pos_Ctrl_Err.y);
			if(dis_cm<=Waypoint_Fix_CM2)	flight_global_cnt[n]++;
			else flight_global_cnt[n]/=2;
		}
		else//����4*5ms���㣬��ʾ����Ŀ�꺽��λ��
		{
			execute_time_ms[n]=1000/flight_subtask_delta;//������ִ��ʱ��
			flight_subtask_cnt[n]++;
			flight_global_cnt[n]=0;		
			if(Opv_Top_View_Target.trust_flag==1)//����Ŀ�꺽����жϵײ��Ƿ�Ϊũ����
			{
				laser_light1.reset=1;
				laser_light1.times=1;//��˸1��
			}			
		}	
	}
	else if(flight_subtask_cnt[n]==32)//���Ƚ��ͷ��и߶ȵ�130cm
	{
		basic_auto_flight_support();//��������֧�����
		target_position.x=base_position.x+work_waypoints[0][30];
		target_position.y=base_position.y+work_waypoints[1][30];
		target_position.z=Barcode_Height_CM;//���������ڸ߶�
		Horizontal_Navigation(target_position.x,
													target_position.y,
													target_position.z,
													GLOBAL_MODE,
													MAP_FRAME);
		flight_subtask_cnt[n]++;
		flight_global_cnt[n]=0;		
	}
	else if(flight_subtask_cnt[n]==33)//�ж��Ƿ񵽴�Ŀ��߶�
	{
		basic_auto_flight_support();//��������֧�����
		//�ж��Ƿ񵽴�Ŀ��߶�
		if(flight_global_cnt[n]<100)//����100*5ms����
		{
			if(ABS(Total_Controller.Height_Position_Control.Err)<=10.0f)	flight_global_cnt[n]++;
			else flight_global_cnt[n]/=2;
		}
		else//����100*5ms���㣬��ʾ����Ŀ��߶�
		{
			flight_subtask_cnt[n]++;
			flight_global_cnt[n]=0;
		}
	}
	else if(flight_subtask_cnt[n]==34)//�ɵ���ͼ�м�14��λ�ã������ֳ���Ավ��̫���������״�ɨ������Ϊ�Ǿ�������ϰ�
	{
		basic_auto_flight_support();//��������֧�����
		target_position.x=base_position.x+4*Scale_Param;
		target_position.y=base_position.y+3*Scale_Param;
		target_position.z=Barcode_Height_CM;
		Horizontal_Navigation(target_position.x,
													target_position.y,
													target_position.z,
													GLOBAL_MODE,
													MAP_FRAME);
		//�ж��Ƿ񵽴�Ŀ�꺽��λ��
		if(flight_global_cnt[n]<Waypoint_Fix_Cnt2)//����10*5ms=0.05s����
		{
			float dis_cm=pythagorous2(OpticalFlow_Pos_Ctrl_Err.x,OpticalFlow_Pos_Ctrl_Err.y);
			if(dis_cm<Waypoint_Fix_CM2)	flight_global_cnt[n]++;
			else flight_global_cnt[n]/=2;
		}
		else
		{
			flight_subtask_cnt[n]++;
			flight_global_cnt[n]=0;		
		}
	}
	else if(flight_subtask_cnt[n]==35)//���ȿ���ƫ���˶���ʹ�÷ɻ�ͷ����׼����
	{
		float expect_yaw_gyro=Total_Controller.Yaw_Angle_Control.Kp*target_yaw_err;//����ƫ�����ٶ�	
		
		Flight.yaw_ctrl_mode=ROTATE;
		Flight.yaw_outer_control_output=constrain_float(expect_yaw_gyro,-10,10);//��10deg/s�Ľ��ٶ�˳ʱ��ת��10000ms
		if(ABS(target_yaw_err)<=5.0f)//ֻ�е�ƫ���ǶȱȽ�Сʱ���ſ�����
		{
			float dis_err=min_dis_cm-50;//�趨�˵����˻����ĵľ��룬������ͷ���˾��룬�ɸ���ʵ����ࡢ�Ӿ�����ģ�鰲װλ���Լ�ʶ������ʵ�����
			dis_err=constrain_float(dis_err,-20,20);
			OpticalFlow_Y_Vel_Control(Total_Controller.Optical_Position_Control.Kp*dis_err);
			OpticalFlow_X_Vel_Control(0);
			header_fix_flag=1;
		}
		else//����ԭ����ͣ
		{
			OpticalFlow_Control_Pure(header_fix_flag);
			header_fix_flag=0;
		}			
		Flight_Alt_Hold_Control(ALTHOLD_MANUAL_CTRL,NUL,NUL);//�߶ȿ���
	
		if(barcode_flag==1)//ֻҪʶ���˶�ά�룬��ǰ������׼���ˡ���������
		{
			flight_subtask_cnt[n]=36;
			flight_global_cnt[n]=0;
			
			//ʶ������������»��˵�14���飬����ֱ�ӷ���������ײ����
			target_position.x=base_position.x+4*Scale_Param;
			target_position.y=base_position.y+3*Scale_Param;
			target_position.z=Barcode_Height_CM;
			Horizontal_Navigation(target_position.x,
														target_position.y,
														target_position.z,
														GLOBAL_MODE,
														MAP_FRAME);
		}
	}
	else if(flight_subtask_cnt[n]==36)
	{
		basic_auto_flight_support();//��������֧�����
		laser_light1.reset=1;
		laser_light1.times=barcode_id;//��˸barcode_id��
		execute_time_ms[n]=(barcode_id*1000+3000)/flight_subtask_delta;
		flight_subtask_cnt[n]++;
		flight_global_cnt[n]=0;
		
		//����������ᷢ���������������
		if(execute_time_ms[n]>=20000/flight_subtask_delta)   execute_time_ms[n]=20000/flight_subtask_delta;//��󲻳���20S,����id��Ϣʶ�������ɳ�ʱ�䵢��
	}
	else if(flight_subtask_cnt[n]==37)//���3s���ٴ���˸
	{
		basic_auto_flight_support();//��������֧�����
		if(execute_time_ms[n]>0) execute_time_ms[n]--;
		if(execute_time_ms[n]==0) 
		{
			laser_light1.reset=1;
			laser_light1.times=barcode_id;//��˸barcode_id��
			flight_subtask_cnt[n]++;
			flight_global_cnt[n]=0;
		}
	}
	else if(flight_subtask_cnt[n]==38)//�˻ص�4�ŵ��·�����
	{
		basic_auto_flight_support();//��������֧�����
		target_position.x=base_position.x+4*Scale_Param;
		target_position.y=base_position.y-1*Scale_Param;
		target_position.z=Barcode_Height_CM;
		Horizontal_Navigation(target_position.x,
													target_position.y,
													target_position.z,
													GLOBAL_MODE,
													MAP_FRAME);
		//�ж��Ƿ񵽴�Ŀ�꺽��λ��
		if(flight_global_cnt[n]<Waypoint_Fix_Cnt2)//����10*5ms=0.05s����
		{
			float dis_cm=pythagorous2(OpticalFlow_Pos_Ctrl_Err.x,OpticalFlow_Pos_Ctrl_Err.y);
			if(dis_cm<=Waypoint_Fix_CM2)	flight_global_cnt[n]++;
			else flight_global_cnt[n]/=2;
		}
		else
		{
			flight_subtask_cnt[n]++;
			flight_global_cnt[n]=0;				
		}
	}
	else if(flight_subtask_cnt[n]==39)//�˻ص���ɵ���·�����
	{
		basic_auto_flight_support();//��������֧�����
		target_position.x=base_position.x;
		target_position.y=base_position.y-1*Scale_Param;
		target_position.z=Barcode_Height_CM;
		Horizontal_Navigation(target_position.x,
													target_position.y,
													target_position.z,
													GLOBAL_MODE,
													MAP_FRAME);
		//�ж��Ƿ񵽴�Ŀ�꺽��λ��
		if(flight_global_cnt[n]<Waypoint_Fix_Cnt2)//����10*5ms=0.05s����
		{
			float dis_cm=pythagorous2(OpticalFlow_Pos_Ctrl_Err.x,OpticalFlow_Pos_Ctrl_Err.y);
			if(dis_cm<=Waypoint_Fix_CM2)	flight_global_cnt[n]++;
			else flight_global_cnt[n]/=2;
		}
		else
		{
			flight_subtask_cnt[n]++;
			flight_global_cnt[n]=0;				
		}	
	}
	else if(flight_subtask_cnt[n]==40)//��ʼ��׼��ͷ����ִֵֻ��һ��
	{
		Flight.yaw_ctrl_mode=AZIMUTH;				//ֱ�ӿ��ƺ����ģʽ
		Flight.yaw_ctrl_start=1;						//��ʼ����ʹ��
		Flight.yaw_outer_control_output  =0;//Ŀ��Ƕȣ���Ч�����ͳ�ʼ��ͷ����һ��
		OpticalFlow_Control_Pure(0);//SLAM�������	
		Flight_Alt_Hold_Control(ALTHOLD_MANUAL_CTRL,NUL,NUL);//�߶ȿ���
		flight_subtask_cnt[n]=41;		
	}
	else if(flight_subtask_cnt[n]==41)//ִ�ж�׼��ͷ���̣����ж��Ƿ��׼���
	{
		Flight.yaw_ctrl_mode=AZIMUTH;
		Flight.yaw_outer_control_output  =0;
		OpticalFlow_Control_Pure(0);	
		Flight_Alt_Hold_Control(ALTHOLD_MANUAL_CTRL,NUL,NUL);//�߶ȿ���
		
		if(Flight.yaw_ctrl_end==1)  flight_subtask_cnt[n]=42;//ִ����Ϻ��л�����һ�׶�	
	}	
	else if(flight_subtask_cnt[n]==42)//�˻ص���ɵ����·�barcode_id*10����ͣ
	{
		basic_auto_flight_support();//��������֧�����
		target_position.x=base_position.x;
		target_position.y=base_position.y-10*barcode_id;
		target_position.z=Barcode_Height_CM;
		Horizontal_Navigation(target_position.x,
													target_position.y,
													target_position.z,
													GLOBAL_MODE,
													MAP_FRAME);
		//�ж��Ƿ񵽴�Ŀ�꺽��λ��
		if(flight_global_cnt[n]<Waypoint_Fix_Cnt2)//����10*5ms=0.05s����
		{
			float dis_cm=pythagorous2(OpticalFlow_Pos_Ctrl_Err.x,OpticalFlow_Pos_Ctrl_Err.y);
			if(dis_cm<=Waypoint_Fix_CM2)	flight_global_cnt[n]++;
			else flight_global_cnt[n]/=2;
		}
		else
		{
			flight_subtask_cnt[n]++;
			flight_global_cnt[n]=0;

			//���¸�λ���������ã����е����治ͬ�߶ȣ������ֲ�����仯��ɵ����
			//��ͬ�߶��ݶ��ϳ���仯����ʱ���������⴦���ֿ���ȥ��
			//���⴦��ʼ
//			send_check_back=4;//����slam
//			VIO_SINS.Position[_EAST] = 0;
//			VIO_SINS.Position[_NORTH]= 0;
//		  OpticalFlow_Pos_Ctrl_Expect.x=0;
//		  OpticalFlow_Pos_Ctrl_Expect.y=0;
			//���⴦�����
		}	
	}	
	else if(flight_subtask_cnt[n]==43)//ԭ�ؽ���
	{
		Flight.yaw_ctrl_mode=ROTATE;
		Flight.yaw_outer_control_output  =RC_Data.rc_rpyt[RC_YAW];
		OpticalFlow_Control_Pure(0);
		Flight_Alt_Hold_Control(ALTHOLD_AUTO_VEL_CTRL,NUL,-30);//�߶ȿ���
	}
	else
	{
		basic_auto_flight_support();//��������֧�����
	}
}





//2022��TI��������7�·�ʡ�������ͻ����˻�
#define Deliver_Scale_Param   25
#define First_Working_Height  150
#define Second_Working_Height	80
#define Fixed_CM     5.0f
#define Times_Fixed  5

#define deliver_down_time 6000//��λms�������������½�ʱ��
#define deliver_up_time   6100//��λms����������������ʱ��

#define deliver_down_pwm_us 2000
#define deliver_stop_pwm_us 1500
#define deliver_up_pwm_us 	1000


const int16_t deliver_work_waypoints_table[4][12]={
{2 , 8, 11, 14, 14, 11, 5, 5, 2,  8, 8 , 14},//������X
{11, 5, 8 , -1, 11, 2 , 2, 8, 5, -1, 11,  5} //������Y
};

const uint16_t deliver_work_feature_table[2][12]={
{1, 1, 2, 2, 1, 1, 2, 2, 1, 1, 2, 2},//��ɫ��1��ɫ��2��ɫ
{3, 3, 3, 3, 1, 1, 1, 1, 2, 2, 2, 2} //��״��1Բ�Ρ�2���Ρ�3������
};

uint16_t template_feature[2]={0,0};//��������ɫ+��״
float xtarget[2],ytarget[2];

//�����ܹ���6�飺
//��ɫ�����Ρ���ɫ�����Ρ���ɫԲ��
//��ɫԲ��  ����ɫ����  ����ɫ����
void deliver_work_waypoint_generate(void)
{
	for(uint16_t i=0;i<12;i++)
	{
		work_waypoints[0][i]=Deliver_Scale_Param*deliver_work_waypoints_table[0][i];
		work_waypoints[1][i]=Deliver_Scale_Param*deliver_work_waypoints_table[1][i];
	}

}


//��һ���֡����ֶ���������Ŀ��ص���ҵ
void Deliver_UAV_Basic(void)
{
	static uint8_t n=12;
	Vector3f target_position;
	float x=0,y=0,z=0;	
	if(flight_subtask_cnt[n]==0)//��ɵ���Ϊ��һ����ͣ��
	{
		basic_auto_flight_support();//��������֧�����
		deliver_work_waypoint_generate();//����
			
		//��¼�³�ʼ���λ�ã�ʵ����Ŀ�п�����Ϊĳһ��׼ԭ��
		//base_position.x=VIO_SINS.Position[_EAST];
		//base_position.y=VIO_SINS.Position[_NORTH];
		base_position.z=First_Working_Height;//��һ��ҵ�߶�
		
		x=base_position.x;
		y=base_position.y;
		z=First_Working_Height;
		target_position.x=x;
		target_position.y=y;
		target_position.z=z;
		Horizontal_Navigation(target_position.x,
													target_position.y,
													target_position.z,
													GLOBAL_MODE,
													MAP_FRAME);
		flight_subtask_cnt[n]=1;
		flight_global_cnt[n]=0;
		execute_time_ms[n]=5000/flight_subtask_delta;//������ִ��ʱ��
		
		laser_light1.reset=1;
		laser_light1.times=50000;//����50000��
		laser_light1.period=200;
		laser_light1.light_on_percent=0.98;
	}
	else if(flight_subtask_cnt[n]==1)//���֮��ԭ����ͣ5����ִ�к�������
	{
		basic_auto_flight_support();//��������֧�����
		if(execute_time_ms[n]>0) execute_time_ms[n]--;
		if(execute_time_ms[n]==0)//��ͣʱ����������㣬��ͣ����ִ����� 
		{
			x=param_value[0]*param_value[1];
			y=param_value[0]*param_value[2];
			z=First_Working_Height;
			target_position.x=base_position.x+x;
			target_position.y=base_position.y+y;
			target_position.z=z;
			Horizontal_Navigation(target_position.x,
														target_position.y,
														target_position.z,
														GLOBAL_MODE,
														MAP_FRAME);
			
			flight_subtask_cnt[n]++;
			flight_global_cnt[n]=0;
			execute_time_ms[n]=0;
		}		
	}
	else if(flight_subtask_cnt[n]==2)//�����ɵ���ͣ��Ϻ󣬷����һ��Ŀ���
	{
		basic_auto_flight_support();//��������֧�����
		
		//�ж��Ƿ񵽴�Ŀ�꺽��λ��
		if(flight_global_cnt[n]<Times_Fixed)//����10*5ms=0.05s����
		{
			float dis_cm=pythagorous2(OpticalFlow_Pos_Ctrl_Err.x,OpticalFlow_Pos_Ctrl_Err.y);
			if(dis_cm<=Fixed_CM)	flight_global_cnt[n]++;
			else flight_global_cnt[n]/=2;
		}
		else//����10*5ms���㣬��ʾ����Ŀ�꺽��λ�ã��󽵵�Ŀ��߶�
		{
			x=param_value[0]*param_value[1];
			y=param_value[0]*param_value[2];
			z=Second_Working_Height;
			target_position.x=base_position.x+x;
			target_position.y=base_position.y+y;
			target_position.z=z;
			Horizontal_Navigation(target_position.x,
														target_position.y,
														target_position.z,
														GLOBAL_MODE,
														MAP_FRAME);
			flight_subtask_cnt[n]++;
			flight_global_cnt[n]=0;
		}
	}
	else if(flight_subtask_cnt[n]==3)
	{
		basic_auto_flight_support();//��������֧�����		
		//�ж��Ƿ񵽴�ڶ���ҵ�߶�
		if(flight_global_cnt[n]<400)//����400*5ms����
		{
			if(ABS(Total_Controller.Height_Position_Control.Err)<=10.0f)	flight_global_cnt[n]++;
			else flight_global_cnt[n]/=2;
		}		
		else
		{
			flight_subtask_cnt[n]++;
			flight_global_cnt[n]=0;
			execute_time_ms[n]=deliver_down_time/flight_subtask_delta;
		}
	}
	else if(flight_subtask_cnt[n]==4)//ִ�������·�����
	{
		//basic_auto_flight_support();//��������֧�����
		Color_Block_Control_Pilot();//����OPENMV�Ӿ�ˮƽ׷��		
		Flight.yaw_ctrl_mode=ROTATE;
		Flight.yaw_outer_control_output =RC_Data.rc_rpyt[RC_YAW];			
		Flight_Alt_Hold_Control(ALTHOLD_MANUAL_CTRL,NUL,NUL);//�߶ȿ���
		
		if(execute_time_ms[n]>0) 
		{
			execute_time_ms[n]--;
			Reserved_PWM1_Output(deliver_down_pwm_us);//��������
		}
		
		if(execute_time_ms[n]==0)//��ͣʱ����������㣬��ͣ����ִ����� 
		{
			Reserved_PWM1_Output(deliver_stop_pwm_us);//ֹͣת��
			//����Ҫˢ��λ�����������ϸ��̱߳���һ��
			flight_subtask_cnt[n]++;
			flight_global_cnt[n]=0;
			execute_time_ms[n]=5000/flight_subtask_delta;
			
			//������������ʾ
			laser_light2.period=200;//200*5ms
			laser_light2.light_on_percent=0.5f;
			laser_light2.reset=1;
			laser_light2.times=4;//��˸4��	
		}	
	}
	else if(flight_subtask_cnt[n]==5)//ԭ����ͣ5s����ָ�����һѲ���߶�
	{
		//basic_auto_flight_support();//��������֧�����
		Color_Block_Control_Pilot();//����OPENMV�Ӿ�ˮƽ׷��		
		Flight.yaw_ctrl_mode=ROTATE;
		Flight.yaw_outer_control_output =RC_Data.rc_rpyt[RC_YAW];			
		Flight_Alt_Hold_Control(ALTHOLD_MANUAL_CTRL,NUL,NUL);//�߶ȿ���
		
		if(execute_time_ms[n]>0) execute_time_ms[n]--;
		if(execute_time_ms[n]==0)//��ͣʱ����������㣬��ͣ����ִ����� 
		{
			OpticalFlow_Control_Pure(1);//ǿ��ˢ����ͣλ��
			flight_subtask_cnt[n]++;
			flight_global_cnt[n]=0;
			execute_time_ms[n]=deliver_up_time/flight_subtask_delta;;
		}		
	}
	else if(flight_subtask_cnt[n]==6)//��������
	{
		basic_auto_flight_support();//��������֧�����
		if(execute_time_ms[n]>0) 
		{
			execute_time_ms[n]--;
			Reserved_PWM1_Output(deliver_up_pwm_us);//��������
		}
		
		if(execute_time_ms[n]==0)//��ͣʱ����������㣬��ͣ����ִ����� 
		{
			Reserved_PWM1_Output(deliver_stop_pwm_us);//ֹͣת��
			
			x=param_value[0]*param_value[1];
			y=param_value[0]*param_value[2];
			z=First_Working_Height;
			target_position.x=base_position.x+x;
			target_position.y=base_position.y+y;
			target_position.z=z;
			Horizontal_Navigation(target_position.x,
														target_position.y,
														target_position.z,
														GLOBAL_MODE,
														MAP_FRAME);
			
			flight_subtask_cnt[n]++;
			flight_global_cnt[n]=0;
			execute_time_ms[n]=5000/flight_subtask_delta;;
		}			
	}
	else if(flight_subtask_cnt[n]==7)//��һ������ִ����Ϻ󣬻ָ�����һ���и߶�
	{
		basic_auto_flight_support();//��������֧�����
		//�ж��Ƿ񵽴��һ��ҵ�߶�
		if(flight_global_cnt[n]<200)//����200*5ms����
		{
			if(ABS(Total_Controller.Height_Position_Control.Err)<=10.0f)	flight_global_cnt[n]++;
			else flight_global_cnt[n]/=2;
		}		
		else
		{
			//�ﵽ��һ���и߶Ⱥ󣬼���ִ�еڶ�����������
			x=param_value[0]*param_value[3];
			y=param_value[0]*param_value[4];
			z=First_Working_Height;
			target_position.x=base_position.x+x;
			target_position.y=base_position.y+y;
			target_position.z=z;
			Horizontal_Navigation(target_position.x,
														target_position.y,
														target_position.z,
														GLOBAL_MODE,
														MAP_FRAME);
			
			flight_subtask_cnt[n]++;
			flight_global_cnt[n]=0;
			execute_time_ms[n]=0;
		}	
	}		
	else if(flight_subtask_cnt[n]==8)//�����ɵ���ͣ��Ϻ󣬷���ڶ���Ŀ���
	{
		basic_auto_flight_support();//��������֧�����
		
		//�ж��Ƿ񵽴�Ŀ�꺽��λ��
		if(flight_global_cnt[n]<Times_Fixed)//����10*5ms=0.05s����
		{
			float dis_cm=pythagorous2(OpticalFlow_Pos_Ctrl_Err.x,OpticalFlow_Pos_Ctrl_Err.y);
			if(dis_cm<=Fixed_CM)	flight_global_cnt[n]++;
			else flight_global_cnt[n]/=2;
		}
		else//����10*5ms���㣬��ʾ����Ŀ�꺽��λ�ã��󽵵�Ŀ��߶�
		{
			x=param_value[0]*param_value[3];
			y=param_value[0]*param_value[4];
			z=Second_Working_Height;
			target_position.x=base_position.x+x;
			target_position.y=base_position.y+y;
			target_position.z=z;
			Horizontal_Navigation(target_position.x,
														target_position.y,
														target_position.z,
														GLOBAL_MODE,
														MAP_FRAME);
			flight_subtask_cnt[n]++;
			flight_global_cnt[n]=0;
		}
	}
	else if(flight_subtask_cnt[n]==9)
	{
		basic_auto_flight_support();//��������֧�����
		
		//�ж��Ƿ񵽴�ڶ���ҵ�߶�
		if(flight_global_cnt[n]<200)//����200*5ms����
		{
			if(ABS(Total_Controller.Height_Position_Control.Err)<=10.0f)	flight_global_cnt[n]++;
			else flight_global_cnt[n]/=2;
		}		
		else//�ų�����
		{	
			flight_subtask_cnt[n]++;
			flight_global_cnt[n]=0;
			execute_time_ms[n]=deliver_down_time/flight_subtask_delta;
		}
	}
	else if(flight_subtask_cnt[n]==10)//ִ�������·�����
	{
		//basic_auto_flight_support();//��������֧�����
		Color_Block_Control_Pilot();//����OPENMV�Ӿ�ˮƽ׷��		
		Flight.yaw_ctrl_mode=ROTATE;
		Flight.yaw_outer_control_output =RC_Data.rc_rpyt[RC_YAW];			
		Flight_Alt_Hold_Control(ALTHOLD_MANUAL_CTRL,NUL,NUL);//�߶ȿ���
		
		if(execute_time_ms[n]>0) 
		{
			execute_time_ms[n]--;
			Reserved_PWM1_Output(deliver_down_pwm_us);//��������
		}
			
		if(execute_time_ms[n]==0)//��ͣʱ����������㣬��ͣ����ִ����� 
		{
			Reserved_PWM1_Output(deliver_stop_pwm_us);//ֹͣת��

			flight_subtask_cnt[n]++;
			flight_global_cnt[n]=0;
			execute_time_ms[n]=5000/flight_subtask_delta;
			
			//������������ʾ
			laser_light2.period=200;//200*5ms
			laser_light2.light_on_percent=0.5f;
			laser_light2.reset=1;
			laser_light2.times=4;//��˸4��		
		}	
	}
	else if(flight_subtask_cnt[n]==11)//ԭ����ͣ5s����ָ�����һѲ���߶�
	{
		//basic_auto_flight_support();//��������֧�����
		Color_Block_Control_Pilot();//����OPENMV�Ӿ�ˮƽ׷��		
		Flight.yaw_ctrl_mode=ROTATE;
		Flight.yaw_outer_control_output =RC_Data.rc_rpyt[RC_YAW];			
		Flight_Alt_Hold_Control(ALTHOLD_MANUAL_CTRL,NUL,NUL);//�߶ȿ���
		
		if(execute_time_ms[n]>0) execute_time_ms[n]--;
		if(execute_time_ms[n]==0)//��ͣʱ����������㣬��ͣ����ִ����� 
		{
			OpticalFlow_Control_Pure(1);//ǿ��ˢ����ͣλ��
			flight_subtask_cnt[n]++;
			flight_global_cnt[n]=0;
			execute_time_ms[n]=deliver_up_time/flight_subtask_delta;
		}		
	}
	else if(flight_subtask_cnt[n]==12)//��������
	{
		basic_auto_flight_support();//��������֧�����
		if(execute_time_ms[n]>0) 
		{
			execute_time_ms[n]--;
			Reserved_PWM1_Output(deliver_up_pwm_us);//��������
		}
		
		if(execute_time_ms[n]==0)//��ͣʱ����������㣬��ͣ����ִ����� 
		{
			Reserved_PWM1_Output(deliver_stop_pwm_us);//ֹͣת��
			
			x=param_value[0]*param_value[3];
			y=param_value[0]*param_value[4];
			z=First_Working_Height;
			target_position.x=base_position.x+x;
			target_position.y=base_position.y+y;
			target_position.z=z;
			Horizontal_Navigation(target_position.x,
														target_position.y,
														target_position.z,
														GLOBAL_MODE,
														MAP_FRAME);
			
			flight_subtask_cnt[n]++;
			flight_global_cnt[n]=0;
			execute_time_ms[n]=0;
		}			
	}
	else if(flight_subtask_cnt[n]==13)//�ڶ�������ִ����Ϻ󣬻ָ�����һ���и߶�
	{
		basic_auto_flight_support();//��������֧�����
		//�ж��Ƿ񵽴��һ��ҵ�߶�
		if(flight_global_cnt[n]<200)//����400*5ms����
		{
			if(ABS(Total_Controller.Height_Position_Control.Err)<=10.0f)	flight_global_cnt[n]++;
			else flight_global_cnt[n]/=2;
		}		
		else
		{
			//�ﵽ��һ���и߶Ⱥ󣬼���ִ�з������񺽵�����
			target_position.x=base_position.x;
			target_position.y=base_position.y;
			target_position.z=First_Working_Height;
			Horizontal_Navigation(target_position.x,
														target_position.y,
														target_position.z,
														GLOBAL_MODE,
														MAP_FRAME);
			
			flight_subtask_cnt[n]++;
			flight_global_cnt[n]=0;
			execute_time_ms[n]=0;
		}	
	}	
	else if(flight_subtask_cnt[n]==14)//������ɵ����Ϸ�
	{
		basic_auto_flight_support();//��������֧�����
		
		//�ж��Ƿ񵽴�Ŀ�꺽��λ��
		if(flight_global_cnt[n]<Times_Fixed)//����10*5ms=0.05s����
		{
			float dis_cm=pythagorous2(OpticalFlow_Pos_Ctrl_Err.x,OpticalFlow_Pos_Ctrl_Err.y);
			if(dis_cm<=Fixed_CM)	flight_global_cnt[n]++;
			else flight_global_cnt[n]/=2;
		}
		else//����10*5ms���㣬��ʾ����Ŀ�꺽��λ��
		{
			flight_subtask_cnt[n]++;
			flight_global_cnt[n]=0;
			execute_time_ms[n]=0;
			/*___________________________________________________________________
			���¸�λ���������ã����е����治ͬ�߶ȣ������ֲ�����仯��ɵ����
			��ͬ�߶��ݶ��ϳ���仯����ʱ���������⴦���ֿ���ȥ��
			___________________________________________________________________*/
			//���⴦��ʼ
			send_check_back=4;//����slam
			VIO_SINS.Position[_EAST] = 0;
			VIO_SINS.Position[_NORTH]= 0;
			OpticalFlow_Pos_Ctrl_Expect.x=0;
			OpticalFlow_Pos_Ctrl_Expect.y=0;
			//���⴦�����	
		}
	}	
	else if(flight_subtask_cnt[n]==15)//ԭ���½�
	{
		Flight.yaw_ctrl_mode=ROTATE;
		Flight.yaw_outer_control_output  =RC_Data.rc_rpyt[RC_YAW];
		OpticalFlow_Control_Pure(0);
		Flight_Alt_Hold_Control(ALTHOLD_AUTO_VEL_CTRL,NUL,-30);//�߶ȿ���		
	}
	else
	{
		basic_auto_flight_support();//��������֧�����
	}
}

/************************************************************************************************/
//�ڶ����֡���ѧϰĳһ�������ڷ������ҵ�����Ŀ��ص���ҵ
void Deliver_UAV_Innovation(void)
{
	static uint8_t n=12;
	Vector3f target_position;
	float x=0,y=0,z=0;
	if(flight_subtask_cnt[n]==0)//��ɵ���Ϊ��һ����ͣ��
	{
		deliver_work_waypoint_generate();//����
		template_feature[0]=param_value[7];//����ģ����ɫ����������ɫ��1��ɫ��2��ɫ
		template_feature[1]=param_value[8];//����ģ����״����������״��1Բ�Ρ�2���Ρ�3������
		uint16_t k=0;
		for(uint16_t i=0;i<12;i++)
		{
			if(template_feature[0]==deliver_work_feature_table[0][i]
			 &&template_feature[1]==deliver_work_feature_table[1][i])
			{
				//��Ŀ���������ڵ�λ����Ϣ���ص�Ŀ������
				xtarget[k]=work_waypoints[0][i];
				ytarget[k]=work_waypoints[1][i];
				k++;
			}
			if(k>=2) break;//����������Ϣˢ����Ϻ�,��ǰ��ֹ
		}
			
		basic_auto_flight_support();//��������֧�����
		//��¼�³�ʼ���λ�ã�ʵ����Ŀ�п�����Ϊĳһ��׼ԭ��
		//base_position.x=VIO_SINS.Position[_EAST];
		//base_position.y=VIO_SINS.Position[_NORTH];
		base_position.z=First_Working_Height;//��һ��ҵ�߶�
		
		x=base_position.x;
		y=base_position.y;
		z=First_Working_Height;
		target_position.x=x;
		target_position.y=y;
		target_position.z=z;
		Horizontal_Navigation(target_position.x,
													target_position.y,
													target_position.z,
													GLOBAL_MODE,
													MAP_FRAME);
		flight_subtask_cnt[n]=1;
		flight_global_cnt[n]=0;
		execute_time_ms[n]=2000/flight_subtask_delta;//������ִ��ʱ��
		
		laser_light1.reset=1;
		laser_light1.times=50000;//����50000��
		laser_light1.period=200;
		laser_light1.light_on_percent=0.75;
	}
	else if(flight_subtask_cnt[n]==1)//���֮��ԭ����ͣ5����ִ�к�������
	{
		basic_auto_flight_support();//��������֧�����
		if(execute_time_ms[n]>0) execute_time_ms[n]--;
		if(execute_time_ms[n]==0)//��ͣʱ����������㣬��ͣ����ִ����� 
		{
			x=xtarget[0];
			y=ytarget[0];
			z=First_Working_Height;
			target_position.x=base_position.x+x;
			target_position.y=base_position.y+y;
			target_position.z=z;
			Horizontal_Navigation(target_position.x,
														target_position.y,
														target_position.z,
														GLOBAL_MODE,
														MAP_FRAME);
			
			flight_subtask_cnt[n]++;
			flight_global_cnt[n]=0;
			execute_time_ms[n]=0;
		}		
	}
	else if(flight_subtask_cnt[n]==2)//�����ɵ���ͣ��Ϻ󣬷����һ��Ŀ���
	{
		basic_auto_flight_support();//��������֧�����
		
		//�ж��Ƿ񵽴�Ŀ�꺽��λ��
		if(flight_global_cnt[n]<Times_Fixed)//����10*5ms=0.05s����
		{
			float dis_cm=pythagorous2(OpticalFlow_Pos_Ctrl_Err.x,OpticalFlow_Pos_Ctrl_Err.y);
			if(dis_cm<=Fixed_CM)	flight_global_cnt[n]++;
			else flight_global_cnt[n]/=2;
		}
		else//����10*5ms���㣬��ʾ����Ŀ�꺽��λ�ã��󽵵�Ŀ��߶�
		{
			x=xtarget[0];
			y=ytarget[0];
			z=Second_Working_Height;
			target_position.x=base_position.x+x;
			target_position.y=base_position.y+y;
			target_position.z=z;
			Horizontal_Navigation(target_position.x,
														target_position.y,
														target_position.z,
														GLOBAL_MODE,
														MAP_FRAME);
			flight_subtask_cnt[n]++;
			flight_global_cnt[n]=0;
		}
	}
	else if(flight_subtask_cnt[n]==3)
	{
		basic_auto_flight_support();//��������֧�����
		
		//�ж��Ƿ񵽴�ڶ���ҵ�߶�
		if(flight_global_cnt[n]<400)//����400*5ms����
		{
			if(ABS(Total_Controller.Height_Position_Control.Err)<=10.0f)	flight_global_cnt[n]++;
			else flight_global_cnt[n]/=2;
		}		
		else
		{
			flight_subtask_cnt[n]++;
			flight_global_cnt[n]=0;
			execute_time_ms[n]=deliver_down_time/flight_subtask_delta;
		}
	}
	else if(flight_subtask_cnt[n]==4)//ִ�������·�����
	{
		//basic_auto_flight_support();//��������֧�����
		Color_Block_Control_Pilot();//����OPENMV�Ӿ�ˮƽ׷��		
		Flight.yaw_ctrl_mode=ROTATE;
		Flight.yaw_outer_control_output =RC_Data.rc_rpyt[RC_YAW];			
		Flight_Alt_Hold_Control(ALTHOLD_MANUAL_CTRL,NUL,NUL);//�߶ȿ���
		
		
		if(execute_time_ms[n]>0) 
		{
			execute_time_ms[n]--;
			Reserved_PWM1_Output(deliver_down_pwm_us);//��������
		}
		
		if(execute_time_ms[n]==0)//��ͣʱ����������㣬��ͣ����ִ����� 
		{
			Reserved_PWM1_Output(deliver_stop_pwm_us);//ֹͣת��
			//����Ҫˢ��λ�����������ϸ��̱߳���һ��
			flight_subtask_cnt[n]++;
			flight_global_cnt[n]=0;
			execute_time_ms[n]=5000/flight_subtask_delta;
			
			//������������ʾ
			laser_light2.period=200;//200*5ms
			laser_light2.light_on_percent=0.5f;
			laser_light2.reset=1;
			laser_light2.times=4;//��˸4��	
		}	
	}
	else if(flight_subtask_cnt[n]==5)//ԭ����ͣ5s����ָ�����һѲ���߶�
	{
		//basic_auto_flight_support();//��������֧�����
		Color_Block_Control_Pilot();//����OPENMV�Ӿ�ˮƽ׷��		
		Flight.yaw_ctrl_mode=ROTATE;
		Flight.yaw_outer_control_output =RC_Data.rc_rpyt[RC_YAW];			
		Flight_Alt_Hold_Control(ALTHOLD_MANUAL_CTRL,NUL,NUL);//�߶ȿ���

		
		if(execute_time_ms[n]>0) execute_time_ms[n]--;
		if(execute_time_ms[n]==0)//��ͣʱ����������㣬��ͣ����ִ����� 
		{		
			OpticalFlow_Control_Pure(1);//ǿ��ˢ����ͣλ��
			flight_subtask_cnt[n]++;
			flight_global_cnt[n]=0;
			execute_time_ms[n]=deliver_up_time/flight_subtask_delta;;
		}		
	}
	else if(flight_subtask_cnt[n]==6)//��������
	{
		basic_auto_flight_support();//��������֧�����
		if(execute_time_ms[n]>0) 
		{
			execute_time_ms[n]--;
			Reserved_PWM1_Output(deliver_up_pwm_us);//��������
		}
		
		if(execute_time_ms[n]==0)//��ͣʱ����������㣬��ͣ����ִ����� 
		{
			Reserved_PWM1_Output(deliver_stop_pwm_us);//ֹͣת��
			
			x=xtarget[0];
			y=ytarget[0];
			z=First_Working_Height;
			target_position.x=base_position.x+x;
			target_position.y=base_position.y+y;
			target_position.z=z;
			Horizontal_Navigation(target_position.x,
														target_position.y,
														target_position.z,
														GLOBAL_MODE,
														MAP_FRAME);
			
			flight_subtask_cnt[n]++;
			flight_global_cnt[n]=0;
			execute_time_ms[n]=5000/flight_subtask_delta;;
		}			
	}
	else if(flight_subtask_cnt[n]==7)//��һ������ִ����Ϻ󣬻ָ�����һ���и߶�
	{
		basic_auto_flight_support();//��������֧�����
		//�ж��Ƿ񵽴��һ��ҵ�߶�
		if(flight_global_cnt[n]<200)//����200*5ms����
		{
			if(ABS(Total_Controller.Height_Position_Control.Err)<=10.0f)	flight_global_cnt[n]++;
			else flight_global_cnt[n]/=2;
		}		
		else
		{
			//�ﵽ��һ���и߶Ⱥ󣬼���ִ�еڶ�����������
			x=xtarget[1];
			y=ytarget[1];
			z=First_Working_Height;
			target_position.x=base_position.x+x;
			target_position.y=base_position.y+y;
			target_position.z=z;
			Horizontal_Navigation(target_position.x,
														target_position.y,
														target_position.z,
														GLOBAL_MODE,
														MAP_FRAME);
			
			flight_subtask_cnt[n]++;
			flight_global_cnt[n]=0;
			execute_time_ms[n]=0;
		}	
	}		
	else if(flight_subtask_cnt[n]==8)//�����ɵ���ͣ��Ϻ󣬷���ڶ���Ŀ���
	{
		basic_auto_flight_support();//��������֧�����
		
		//�ж��Ƿ񵽴�Ŀ�꺽��λ��
		if(flight_global_cnt[n]<Times_Fixed)//����10*5ms=0.05s����
		{
			float dis_cm=pythagorous2(OpticalFlow_Pos_Ctrl_Err.x,OpticalFlow_Pos_Ctrl_Err.y);
			if(dis_cm<=Fixed_CM)	flight_global_cnt[n]++;
			else flight_global_cnt[n]/=2;
		}
		else//����10*5ms���㣬��ʾ����Ŀ�꺽��λ�ã��󽵵�Ŀ��߶�
		{
			x=xtarget[1];
			y=ytarget[1];
			z=Second_Working_Height;
			target_position.x=base_position.x+x;
			target_position.y=base_position.y+y;
			target_position.z=z;
			Horizontal_Navigation(target_position.x,
														target_position.y,
														target_position.z,
														GLOBAL_MODE,
														MAP_FRAME);
			flight_subtask_cnt[n]++;
			flight_global_cnt[n]=0;
		}
	}
	else if(flight_subtask_cnt[n]==9)
	{
		basic_auto_flight_support();//��������֧�����
		
		//�ж��Ƿ񵽴�ڶ���ҵ�߶�
		if(flight_global_cnt[n]<200)//����200*5ms����
		{
			if(ABS(Total_Controller.Height_Position_Control.Err)<=10.0f)	flight_global_cnt[n]++;
			else flight_global_cnt[n]/=2;
		}		
		else//�ų�����
		{	
			flight_subtask_cnt[n]++;
			flight_global_cnt[n]=0;
			execute_time_ms[n]=deliver_down_time/flight_subtask_delta;
		}
	}
	else if(flight_subtask_cnt[n]==10)//ִ�������·�����
	{
		//basic_auto_flight_support();//��������֧�����
		Color_Block_Control_Pilot();//����OPENMV�Ӿ�ˮƽ׷��		
		Flight.yaw_ctrl_mode=ROTATE;
		Flight.yaw_outer_control_output =RC_Data.rc_rpyt[RC_YAW];			
		Flight_Alt_Hold_Control(ALTHOLD_MANUAL_CTRL,NUL,NUL);//�߶ȿ���

		if(execute_time_ms[n]>0) 
		{
			execute_time_ms[n]--;
			Reserved_PWM1_Output(deliver_down_pwm_us);//��������
		}
			
		if(execute_time_ms[n]==0)//��ͣʱ����������㣬��ͣ����ִ����� 
		{
			Reserved_PWM1_Output(deliver_stop_pwm_us);//ֹͣת��

			flight_subtask_cnt[n]++;
			flight_global_cnt[n]=0;
			execute_time_ms[n]=5000/flight_subtask_delta;
			
			//������������ʾ
			laser_light2.period=200;//200*5ms
			laser_light2.light_on_percent=0.5f;
			laser_light2.reset=1;
			laser_light2.times=4;//��˸4��		
		}	
	}
	else if(flight_subtask_cnt[n]==11)//ԭ����ͣ5s����ָ�����һѲ���߶�
	{
		//basic_auto_flight_support();//��������֧�����
		Color_Block_Control_Pilot();//����OPENMV�Ӿ�ˮƽ׷��		
		Flight.yaw_ctrl_mode=ROTATE;
		Flight.yaw_outer_control_output =RC_Data.rc_rpyt[RC_YAW];			
		Flight_Alt_Hold_Control(ALTHOLD_MANUAL_CTRL,NUL,NUL);//�߶ȿ���
	
		if(execute_time_ms[n]>0) execute_time_ms[n]--;
		if(execute_time_ms[n]==0)//��ͣʱ����������㣬��ͣ����ִ����� 
		{
			OpticalFlow_Control_Pure(1);//ǿ��ˢ����ͣλ��
			flight_subtask_cnt[n]++;
			flight_global_cnt[n]=0;
			execute_time_ms[n]=deliver_up_time/flight_subtask_delta;
		}		
	}
	else if(flight_subtask_cnt[n]==12)//��������
	{
		basic_auto_flight_support();//��������֧�����
		if(execute_time_ms[n]>0) 
		{
			execute_time_ms[n]--;
			Reserved_PWM1_Output(deliver_up_pwm_us);//��������
		}
		
		if(execute_time_ms[n]==0)//��ͣʱ����������㣬��ͣ����ִ����� 
		{
			Reserved_PWM1_Output(deliver_stop_pwm_us);//ֹͣת��
			
			x=xtarget[1];
			y=ytarget[1];
			z=First_Working_Height;
			target_position.x=base_position.x+x;
			target_position.y=base_position.y+y;
			target_position.z=z;
			Horizontal_Navigation(target_position.x,
														target_position.y,
														target_position.z,
														GLOBAL_MODE,
														MAP_FRAME);
			
			flight_subtask_cnt[n]++;
			flight_global_cnt[n]=0;
			execute_time_ms[n]=0;
		}			
	}
	else if(flight_subtask_cnt[n]==13)//�ڶ�������ִ����Ϻ󣬻ָ�����һ���и߶�
	{
		basic_auto_flight_support();//��������֧�����
		//�ж��Ƿ񵽴��һ��ҵ�߶�
		if(flight_global_cnt[n]<200)//����400*5ms����
		{
			if(ABS(Total_Controller.Height_Position_Control.Err)<=10.0f)	flight_global_cnt[n]++;
			else flight_global_cnt[n]/=2;
		}		
		else
		{
			//�ﵽ��һ���и߶Ⱥ󣬼���ִ�з������񺽵�����
			target_position.x=base_position.x;
			target_position.y=base_position.y;
			target_position.z=First_Working_Height;
			Horizontal_Navigation(target_position.x,
														target_position.y,
														target_position.z,
														GLOBAL_MODE,
														MAP_FRAME);
			
			flight_subtask_cnt[n]++;
			flight_global_cnt[n]=0;
			execute_time_ms[n]=0;
		}	
	}	
	else if(flight_subtask_cnt[n]==14)//������ɵ����Ϸ�
	{
		basic_auto_flight_support();//��������֧�����
		
		//�ж��Ƿ񵽴�Ŀ�꺽��λ��
		if(flight_global_cnt[n]<Times_Fixed)//����10*5ms=0.05s����
		{
			float dis_cm=pythagorous2(OpticalFlow_Pos_Ctrl_Err.x,OpticalFlow_Pos_Ctrl_Err.y);
			if(dis_cm<=Fixed_CM)	flight_global_cnt[n]++;
			else flight_global_cnt[n]/=2;
		}
		else//����10*5ms���㣬��ʾ����Ŀ�꺽��λ��
		{
			flight_subtask_cnt[n]++;
			flight_global_cnt[n]=0;
			execute_time_ms[n]=0;
		}
	}	
	else if(flight_subtask_cnt[n]==15)//ԭ���½�
	{
		Flight.yaw_ctrl_mode=ROTATE;
		Flight.yaw_outer_control_output  =RC_Data.rc_rpyt[RC_YAW];
		OpticalFlow_Control_Pure(0);
		Flight_Alt_Hold_Control(ALTHOLD_AUTO_VEL_CTRL,NUL,-30);//�߶ȿ���		
	}
	else
	{
		basic_auto_flight_support();//��������֧�����
	}
}

/************************************************************************************************/
//�������֡�����ԽԲ��
void Deliver_UAV_Hulahoop(void)
{
	static uint8_t n=12;
	static float loop_x_cm=0,loop_y_cm=0,loop_angle=0;
	static float temp_x[2],temp_y[2];
	Vector3f target_position;
	float x=0,y=0,z=0;
	if(flight_subtask_cnt[n]==0)//��ɵ���Ϊ��һ����ͣ��
	{
		deliver_work_waypoint_generate();//����
		loop_x_cm=param_value[9];
		loop_y_cm=param_value[10];
		loop_angle=param_value[11];

		xtarget[0]=loop_x_cm+110*sinf(DEG2RAD*loop_angle);
		ytarget[0]=loop_y_cm-110*cosf(DEG2RAD*loop_angle);
		xtarget[1]=loop_x_cm-110*sinf(DEG2RAD*loop_angle);
		ytarget[1]=loop_y_cm+110*cosf(DEG2RAD*loop_angle);
		//����Բ������X��н���������Ѱ��׼��
		if(loop_angle<=90)
		{
			temp_x[0]=0;
			temp_y[0]=-50;
			temp_x[1]=350;
			temp_y[1]=-50;
		}
		else
		{
			temp_x[0]=0;
			temp_y[0]=300;
			temp_x[1]=350;
			temp_y[1]=300;		
		}
			
		basic_auto_flight_support();//��������֧�����
		//��¼�³�ʼ���λ�ã�ʵ����Ŀ�п�����Ϊĳһ��׼ԭ��
		//base_position.x=VIO_SINS.Position[_EAST];
		//base_position.y=VIO_SINS.Position[_NORTH];
		base_position.z=First_Working_Height;//��һ��ҵ�߶�
		
		x=base_position.x;
		y=base_position.y;
		z=First_Working_Height;
		target_position.x=x;
		target_position.y=y;
		target_position.z=z;
		Horizontal_Navigation(target_position.x,
													target_position.y,
													target_position.z,
													GLOBAL_MODE,
													MAP_FRAME);
		flight_subtask_cnt[n]=1;
		flight_global_cnt[n]=0;
		execute_time_ms[n]=2000/flight_subtask_delta;//������ִ��ʱ��	
	}
	else if(flight_subtask_cnt[n]==1)//���֮��ԭ����ͣ5����ִ�к�������
	{
		basic_auto_flight_support();//��������֧�����
		if(execute_time_ms[n]>0) execute_time_ms[n]--;
		if(execute_time_ms[n]==0)//��ͣʱ����������㣬��ͣ����ִ����� 
		{
			x=temp_x[0];
			y=temp_y[0];
			z=First_Working_Height;
			target_position.x=base_position.x+x;
			target_position.y=base_position.y+y;
			target_position.z=z;
			Horizontal_Navigation(target_position.x,
														target_position.y,
														target_position.z,
														GLOBAL_MODE,
														MAP_FRAME);
			
			flight_subtask_cnt[n]++;
			flight_global_cnt[n]=0;
			execute_time_ms[n]=0;
		}		
	}
	else if(flight_subtask_cnt[n]==2)//�����ɵ���ͣ��Ϻ󣬷����һ��Ŀ���
	{
		basic_auto_flight_support();//��������֧�����
		
		//�ж��Ƿ񵽴�Ŀ�꺽��λ��
		if(flight_global_cnt[n]<Times_Fixed)//����10*5ms=0.05s����
		{
			float dis_cm=pythagorous2(OpticalFlow_Pos_Ctrl_Err.x,OpticalFlow_Pos_Ctrl_Err.y);
			if(dis_cm<=Fixed_CM)	flight_global_cnt[n]++;
			else flight_global_cnt[n]/=2;
		}
		else//����10*5ms���㣬��ʾ����Ŀ�꺽��λ�ã��󽵵�Ŀ��߶�
		{
			x=temp_x[1];
			y=temp_y[1];
			z=First_Working_Height;
			target_position.x=base_position.x+x;
			target_position.y=base_position.y+y;
			target_position.z=z;
			Horizontal_Navigation(target_position.x,
														target_position.y,
														target_position.z,
														GLOBAL_MODE,
														MAP_FRAME);
			flight_subtask_cnt[n]++;
			flight_global_cnt[n]=0;
		}
	}
	else if(flight_subtask_cnt[n]==3)//�����ɵ���ͣ��Ϻ󣬷���ڶ���Ŀ���
	{
		basic_auto_flight_support();//��������֧�����
		//�ж��Ƿ񵽴�Ŀ�꺽��λ��
		if(flight_global_cnt[n]<Times_Fixed)//����10*5ms=0.05s����
		{
			float dis_cm=pythagorous2(OpticalFlow_Pos_Ctrl_Err.x,OpticalFlow_Pos_Ctrl_Err.y);
			if(dis_cm<=Fixed_CM)	flight_global_cnt[n]++;
			else flight_global_cnt[n]/=2;
		}
		else//����10*5ms���㣬��ʾ����Ŀ�꺽��λ�ã��󽵵�Ŀ��߶�
		{
			flight_subtask_cnt[n]++;
			flight_global_cnt[n]=0;
		}
	}
	else if(flight_subtask_cnt[n]==4)//��ʼ��׼��ͷ����ִֵֻ��һ��
	{
		Flight.yaw_ctrl_mode=AZIMUTH;				//ֱ�ӿ��ƺ����ģʽ
		Flight.yaw_ctrl_start=1;						//��ʼ����ʹ��
		Flight.yaw_outer_control_output  =loop_angle;//Ŀ��Ƕȣ���Ч�����ͳ�ʼ��ͷ����һ��
		
		OpticalFlow_Control_Pure(0);//SLAM�������	
		Flight_Alt_Hold_Control(ALTHOLD_MANUAL_CTRL,NUL,NUL);//�߶ȿ���
		flight_subtask_cnt[n]++;		
	}
	else if(flight_subtask_cnt[n]==5)//ִ�ж�׼��ͷ���̣����ж��Ƿ��׼���
	{
		Flight.yaw_ctrl_mode=AZIMUTH;
		Flight.yaw_outer_control_output  =loop_angle;
		OpticalFlow_Control_Pure(0);	
		Flight_Alt_Hold_Control(ALTHOLD_MANUAL_CTRL,NUL,NUL);//�߶ȿ���
		
		if(Flight.yaw_ctrl_end==1)  flight_subtask_cnt[n]++;//ִ����Ϻ��л�����һ�׶�	
	}
	else if(flight_subtask_cnt[n]==6)
	{
		basic_auto_flight_support();//��������֧�����
		
		//����Բ����Խ������B
		x=xtarget[0];
		y=ytarget[0];
		z=First_Working_Height;
		target_position.x=base_position.x+x;
		target_position.y=base_position.y+y;
		target_position.z=z;
		Horizontal_Navigation(target_position.x,
													target_position.y,
													target_position.z,
													GLOBAL_MODE,
													MAP_FRAME);
		flight_subtask_cnt[n]++;
		flight_global_cnt[n]=0;
	}	
	else if(flight_subtask_cnt[n]==7)//�����ɵ���ͣ��Ϻ󣬷���ڶ���Ŀ���
	{
		basic_auto_flight_support();//��������֧�����
		
		//�ж��Ƿ񵽴�Ŀ�꺽��λ��
		if(flight_global_cnt[n]<Times_Fixed)//����10*5ms=0.05s����
		{
			float dis_cm=pythagorous2(OpticalFlow_Pos_Ctrl_Err.x,OpticalFlow_Pos_Ctrl_Err.y);
			if(dis_cm<=Fixed_CM)	flight_global_cnt[n]++;
			else flight_global_cnt[n]/=2;
		}
		else//����10*5ms���㣬��ʾ����Ŀ�꺽��λ�ã��󽵵�Ŀ��߶�
		{
			//����Բ����Խ������C
			x=xtarget[1];
			y=ytarget[1];
			z=First_Working_Height;
			target_position.x=base_position.x+x;
			target_position.y=base_position.y+y;
			target_position.z=z;
			Horizontal_Navigation(target_position.x,
														target_position.y,
														target_position.z,
														GLOBAL_MODE,
														MAP_FRAME);
			flight_subtask_cnt[n]++;
			flight_global_cnt[n]=0;
		}
	}
	else if(flight_subtask_cnt[n]==8)//�����ɵ���ͣ��Ϻ󣬷���ڶ���Ŀ���
	{
		basic_auto_flight_support();//��������֧�����
		//�ж��Ƿ񵽴�Ŀ�꺽��λ��
		if(flight_global_cnt[n]<Times_Fixed)//����10*5ms=0.05s����
		{
			float dis_cm=pythagorous2(OpticalFlow_Pos_Ctrl_Err.x,OpticalFlow_Pos_Ctrl_Err.y);
			if(dis_cm<=Fixed_CM)	flight_global_cnt[n]++;
			else flight_global_cnt[n]/=2;
		}
		else//����10*5ms���㣬��ʾ����Ŀ�꺽��λ�ã��󽵵�Ŀ��߶�
		{
			flight_subtask_cnt[n]++;
			flight_global_cnt[n]=0;
		}
	} 
	else if(flight_subtask_cnt[n]==9)//��ʼ��׼��ͷ����ִֵֻ��һ��
	{
		Flight.yaw_ctrl_mode=AZIMUTH;				//ֱ�ӿ��ƺ����ģʽ
		Flight.yaw_ctrl_start=1;						//��ʼ����ʹ��
		Flight.yaw_outer_control_output  =0;//Ŀ��Ƕȣ���Ч�����ͳ�ʼ��ͷ����һ��
		OpticalFlow_Control_Pure(0);//SLAM�������	
		Flight_Alt_Hold_Control(ALTHOLD_MANUAL_CTRL,NUL,NUL);//�߶ȿ���
		flight_subtask_cnt[n]++;	
	}
	else if(flight_subtask_cnt[n]==10)//ִ�ж�׼��ͷ���̣����ж��Ƿ��׼���
	{
		Flight.yaw_ctrl_mode=AZIMUTH;
		Flight.yaw_outer_control_output  =0;
		OpticalFlow_Control_Pure(0);	
		Flight_Alt_Hold_Control(ALTHOLD_MANUAL_CTRL,NUL,NUL);//�߶ȿ���
		
		if(Flight.yaw_ctrl_end==1)  
		{
			flight_subtask_cnt[n]++;//ִ����Ϻ��л�����һ�׶�
			flight_global_cnt[n]=0;
			
			target_position.x=base_position.x;
			target_position.y=base_position.y;
			target_position.z=First_Working_Height;
			Horizontal_Navigation(target_position.x,
														target_position.y,
														target_position.z,
														GLOBAL_MODE,
														MAP_FRAME);		
		}			
	}
	else if(flight_subtask_cnt[n]==11)//���ص����
	{
		basic_auto_flight_support();//��������֧�����
		
		//�ж��Ƿ񵽴�Ŀ�꺽��λ��
		if(flight_global_cnt[n]<Times_Fixed)//����10*5ms=0.05s����
		{
			float dis_cm=pythagorous2(OpticalFlow_Pos_Ctrl_Err.x,OpticalFlow_Pos_Ctrl_Err.y);
			if(dis_cm<=Fixed_CM)	flight_global_cnt[n]++;
			else flight_global_cnt[n]/=2;
		}
		else//����10*5ms���㣬��ʾ����Ŀ�꺽��λ�ã��󽵵�Ŀ��߶�
		{
			flight_subtask_cnt[n]++;
			flight_global_cnt[n]=0;
			
			//���¸�λ���������ã����е����治ͬ�߶ȣ������ֲ�����仯��ɵ����
			//��ͬ�߶��ݶ��ϳ���仯����ʱ���������⴦���ֿ���ȥ��
			//���⴦��ʼ
			send_check_back=4;//����slam
			VIO_SINS.Position[_EAST] = 0;
			VIO_SINS.Position[_NORTH]= 0;
		  OpticalFlow_Pos_Ctrl_Expect.x=0;
		  OpticalFlow_Pos_Ctrl_Expect.y=0;
			//���⴦�����			
		}
	}
	else if(flight_subtask_cnt[n]==12)//ԭ���½�
	{
		Flight.yaw_ctrl_mode=ROTATE;
		Flight.yaw_outer_control_output  =RC_Data.rc_rpyt[RC_YAW];
		OpticalFlow_Control_Pure(0);
		Flight_Alt_Hold_Control(ALTHOLD_AUTO_VEL_CTRL,NUL,-30);//�߶ȿ���		
	}	
	else
	{
		basic_auto_flight_support();//��������֧�����
	}
}




//�Զ���ɵ�ĳһ�߶�
uint8_t Auto_Takeoff(float target)
{
	static uint8_t n=11;
	Vector3f target_position;
	basic_auto_flight_support();//��������֧�����	
	if(flight_subtask_cnt[n]==0)
	{
		//���Ӵ��д��룬������ȫ�����������¶����󣬷ɻ�����Զ����䵽���治���Զ�����
		Unwanted_Lock_Flag=0;//����ɻ��Զ�������ԭ����ֶ������������
			
		//��¼�³�ʼ���λ�ã�ʵ����Ŀ�п�����Ϊĳһ��׼ԭ��
		base_position.x=VIO_SINS.Position[_EAST];
		base_position.y=VIO_SINS.Position[_NORTH];
		base_position.z=NamelessQuad.Position[_UP];
		
		//execute_time_ms[n]=10000/flight_subtask_delta;//������ִ��ʱ��
		target_position.x=base_position.x;
		target_position.y=base_position.y;
		target_position.z=base_position.z+target;
		Horizontal_Navigation(target_position.x,
													target_position.y,
													target_position.z,
													GLOBAL_MODE,
													MAP_FRAME);
		flight_subtask_cnt[n]=1;
	  return	0;		
	}
	else if(flight_subtask_cnt[n]==1)
	{
		//�ж��Ƿ���ɵ�Ŀ��߶�
		if(flight_global_cnt[n]<400)//����400*5ms����
		{
			if(ABS(Total_Controller.Height_Position_Control.Err)<=10.0f)	flight_global_cnt[n]++;
			else flight_global_cnt[n]/=2;
			return	0;
		}
		else//����200*5ms���㣬��ʾ����Ŀ��߶�
		{
			//flight_global_cnt[n]=0;
			//flight_subtask_cnt[n]=0;
			return	1;
		}
	}
	return	0;	
}



/********************************************************************************************************************/
#define nav_param_base (51-1)  //Ԥ�������������ڴ�ź�����Ϣ����ʼ��
#define user_setpoint_max 28   //��󺽵������ɸ���ʵ����Ŀ��Ҫ�Լ�����:��ʾ����������ҳ��Ϊ4ҳ��ÿҳ��7�����㣬��28��
#define user_setpoint_fixed_2d_cm  5.0f //5cm
#define user_setpoint_fixed_3d_cm  10.0f//10cm
#define user_setpoint_fixed_times  5//�������
int32_t nav_setpoint[user_setpoint_max][3]={0,0,0};//������������ 0x200066D8 
uint8_t user_setpoint_valid_flag[user_setpoint_max]={0};//������Ч��־λ
void user_setpoint_generate(void)
{
	memset(user_setpoint_valid_flag,0,sizeof(char)*user_setpoint_max);
	for(uint16_t i=0;i<user_setpoint_max;i++)
	{
		if((param_value[nav_param_base+3*i+0]==0
		  &&param_value[nav_param_base+3*i+1]==0
		  &&param_value[nav_param_base+3*i+2]==0)!=1)//ͨ���жϲ����������ݾ���0�����ж����������Ƿ���Ч
		{
			user_setpoint_valid_flag[i]=true;
			nav_setpoint[i][0]=param_value[nav_param_base+3*i+0];
			nav_setpoint[i][1]=param_value[nav_param_base+3*i+1];
			nav_setpoint[i][2]=param_value[nav_param_base+3*i+2];
		}
		else user_setpoint_valid_flag[i]=false;
	}
}



//�û�ͨ�������Զ���������ά�ĺ���λ�ã����˻����α����������㣬���֧��28������
void Navigation_User_Setpoint(void)
{
	static uint8_t n=13;
	Vector3f target_position;
	float x=0,y=0,z=0;	
	if(flight_subtask_cnt[n]==0)//��ɵ���Ϊ��һ����ͣ��
	{
		basic_auto_flight_support();//��������֧�����
		user_setpoint_generate();//���ɺ���
			
		//��¼�³�ʼ���λ�ã�ʵ����Ŀ�п�����Ϊĳһ��׼ԭ��
		//base_position.x=VIO_SINS.Position[_EAST];
		//base_position.y=VIO_SINS.Position[_NORTH];
		base_position.z=First_Working_Height;//��һ��ҵ�߶�
		
		x=base_position.x;
		y=base_position.y;
		z=First_Working_Height;
		target_position.x=x;
		target_position.y=y;
		target_position.z=z;
		Horizontal_Navigation(target_position.x,
													target_position.y,
													target_position.z,
													GLOBAL_MODE,
													MAP_FRAME);
		flight_subtask_cnt[n]=1;
		flight_global_cnt[n]=0;
		flight_global_cnt2[n]=0;
		execute_time_ms[n]=1000/flight_subtask_delta;//������ִ��ʱ��
	}
	else if(flight_subtask_cnt[n]==1)//���֮��ԭ����ͣ1S����ִ�к�������
	{
		basic_auto_flight_support();//��������֧�����
		if(execute_time_ms[n]>0) execute_time_ms[n]--;
		if(execute_time_ms[n]==0)//��ͣʱ����������㣬��ͣ����ִ����� 
		{
			//�ж����к����ִ����ϣ�ִ����һ����
			if(flight_global_cnt2[n]>=user_setpoint_max)
			{
				flight_subtask_cnt[n]=3;//�����������
				flight_global_cnt[n]=0;
				execute_time_ms[n]=0;
				//�����������0
				flight_global_cnt2[n]=0;
				return ;				
			}
				
			uint16_t current_num=constrain_int32(flight_global_cnt2[n],0,user_setpoint_max-1);//�޷������
			if(user_setpoint_valid_flag[current_num]==true)//�����ǰ�ĺ�����Ч��������Ŀ�꺽�������һ�߳�
			{
				x=nav_setpoint[current_num][0];
				y=nav_setpoint[current_num][1];
				z=nav_setpoint[current_num][2];
				target_position.x=base_position.x+x;//ˮƽλ������Ϊ��ɺ��׼λ��+λ��Xƫ��
				target_position.y=base_position.y+y;//ˮƽλ������Ϊ��ɺ��׼λ��+λ��Yƫ��
				target_position.z=z;
				Horizontal_Navigation(target_position.x,
															target_position.y,
															target_position.z,
															GLOBAL_MODE,
															MAP_FRAME);
				flight_subtask_cnt[n]=2;
				flight_global_cnt[n]=0;
				execute_time_ms[n]=0;				
			}
			else//�����ǰ�ĺ�����Ч��������ǰ����,����������һ����
			{
				//����������Լ�
				flight_global_cnt2[n]++;			
			}
		}		
	}
	else if(flight_subtask_cnt[n]==2)//�����ɵ���ͣ��Ϻ󣬷�����һ��Ŀ���
	{
		basic_auto_flight_support();//��������֧�����		
		//�ж��Ƿ񵽴�Ŀ�꺽��λ��
		if(flight_global_cnt[n]<user_setpoint_fixed_times)//����10*5ms=0.05s����
		{
			float dis_cm=pythagorous3(OpticalFlow_Pos_Ctrl_Err.x,OpticalFlow_Pos_Ctrl_Err.y,Total_Controller.Height_Position_Control.Err);
			if(dis_cm<=user_setpoint_fixed_3d_cm)	flight_global_cnt[n]++;
			else flight_global_cnt[n]/=2;
		}
		else//����10*5ms���㣬��ʾ����Ŀ�꺽��λ��
		{
			flight_subtask_cnt[n]=1;
			flight_global_cnt[n]=0;
			execute_time_ms[n]=0;	
			//����������Լ�
			flight_global_cnt2[n]++;
		}
		//�ж����к����ִ����ϣ�ִ����һ����		
		if(flight_global_cnt2[n]>=user_setpoint_max)
		{
			flight_subtask_cnt[n]=3;
			flight_global_cnt[n]=0;
			execute_time_ms[n]=0;
			//�����������0
			flight_global_cnt2[n]=0;			
		}
	}
	else if(flight_subtask_cnt[n]==3)//ִ�к�����Ϻ�ԭ���½������Ը���ʵ����Ҫ����д����ķ�������
	{
		Flight.yaw_ctrl_mode=ROTATE;
		Flight.yaw_outer_control_output  =RC_Data.rc_rpyt[RC_YAW];
		OpticalFlow_Control_Pure(0);
		Flight_Alt_Hold_Control(ALTHOLD_AUTO_VEL_CTRL,NUL,-30);//�߶ȿ���		
	}
	else
	{
		basic_auto_flight_support();//��������֧�����
	}
}



//2023��TI����G��Ŀ�����յ�Эͬ��������ϵͳ
/************************************************
	D  E  F
	A  B  C
U	G	
************************************************/
#define block_navpoint_num_basic 6//�������ֺ�������
const int16_t block_center_coordinate[block_navpoint_num_basic][2]={
	{100,155},//A
	{255,155},//B
	{410,125},//C
	{410,305},//F
	{270,305},//E
	{115,305},//D
};
const int16_t firetruck_home_center_coordinate[2]={135,25};//G
const int16_t uav_home_center_coordinate[2]={35,35};//U
#define Patrol_Height         180//Ѳ�߸߶�180cm
#define patrol_fixed_3d_30cm  30.0f//30cm
#define patrol_fixed_3d_20cm  20.0f//20cm
#define patrol_fixed_3d_10cm  10.0f//10cm
#define patrol_fixed_3d_5cm   5.0f//5cm
#define patrol_fixed_times    3//�������
#define patrol_fixed_2d_5cm   5.0f//5cm
#define patrol_fixed_2d_times 5
void Air_Ground_Extinguish_Fire_System_Basic(void)
{
	static uint8_t n=14;
	Vector3f target_position;
	float x=0,y=0,z=0;	
	if(flight_subtask_cnt[n]==0)//��ɵ���Ϊ��һ����ͣ��
	{
		basic_auto_flight_support();//��������֧�����
		//����ʵ���
		laser_light1.period=100;//200*5ms
		laser_light1.light_on_percent=1.0f;
		laser_light1.reset=1;
		laser_light1.times=10000;
		
		//��¼�³�ʼ���λ�ã�ʵ����Ŀ�п�����Ϊĳһ��׼ԭ��
		//base_position.x=VIO_SINS.Position[_EAST];
		//base_position.y=VIO_SINS.Position[_NORTH];
		base_position.z=Patrol_Height;//Ѳ�߸߶�
		
		x=base_position.x;
		y=base_position.y;
		z=Patrol_Height;
		target_position.x=x;
		target_position.y=y;
		target_position.z=z;
		Horizontal_Navigation(target_position.x,target_position.y,target_position.z,GLOBAL_MODE,MAP_FRAME);
		
		flight_subtask_cnt[n]=1;
		flight_global_cnt[n]=0;
		flight_global_cnt2[n]=0;
		execute_time_ms[n]=1000/flight_subtask_delta;//������ִ��ʱ��
	}
	else if(flight_subtask_cnt[n]==1)//���֮��ԭ����ͣ1S����ִ�к�������
	{
		basic_auto_flight_support();//��������֧�����
		
		//�ж����к����ִ����ϣ�ִ����һ����
		if(flight_global_cnt2[n]>=block_navpoint_num_basic)
		{
			flight_subtask_cnt[n]=3;//�����������
			flight_global_cnt[n]=0;
			execute_time_ms[n]=0;
			//�����������0
			flight_global_cnt2[n]=0;

						//����ִ�з������񺽵�����
			target_position.x=base_position.x;
			target_position.y=base_position.y;
			target_position.z=Patrol_Height;
			Horizontal_Navigation(target_position.x,target_position.y,target_position.z,GLOBAL_MODE,MAP_FRAME);
			
			return ;				
		}
		
		uint16_t current_num=constrain_int32(flight_global_cnt2[n],0,block_navpoint_num_basic-1);//�޷������		
		if(execute_time_ms[n]>0) execute_time_ms[n]--;
		if(execute_time_ms[n]==0)//��ͣʱ����������㣬��ͣ����ִ����� 
		{				
			//����A��������
			x=block_center_coordinate[current_num][0]-uav_home_center_coordinate[0];
			y=block_center_coordinate[current_num][1]-uav_home_center_coordinate[1];
			z=Patrol_Height;
			target_position.x=base_position.x+x;//ˮƽλ������Ϊ��ɺ��׼λ��+λ��Xƫ��
			target_position.y=base_position.y+y;//ˮƽλ������Ϊ��ɺ��׼λ��+λ��Yƫ��
			target_position.z=z;
			Horizontal_Navigation(target_position.x,target_position.y,target_position.z,GLOBAL_MODE,MAP_FRAME);
			
			flight_subtask_cnt[n]=2;
			flight_global_cnt[n]=0;
			execute_time_ms[n]=0;				
		}//��һ����ֻ�����˺��㣬��һ��������������õĺ���
	}
	else if(flight_subtask_cnt[n]==2)//�����ɵ���ͣ��Ϻ󣬷�����һ��Ŀ���
	{
		basic_auto_flight_support();//��������֧�����		
		//�ж��Ƿ񵽴�Ŀ�꺽��λ��
		if(flight_global_cnt[n]<patrol_fixed_times)//����10*5ms=0.05s����
		{
			float dis_cm=pythagorous3(OpticalFlow_Pos_Ctrl_Err.x,OpticalFlow_Pos_Ctrl_Err.y,Total_Controller.Height_Position_Control.Err);
			//�������Լ��20cm����Ի���������ԼӴ�˴���ֵ���Ӷ�ʵ�ָ����ٱ���
			if(dis_cm<=patrol_fixed_3d_20cm)	flight_global_cnt[n]++;
			else flight_global_cnt[n]/=2;
		}
		else//����10*5ms���㣬��ʾ����Ŀ�꺽��λ��
		{
			flight_subtask_cnt[n]=1;
			flight_global_cnt[n]=0;
			execute_time_ms[n]=500/flight_subtask_delta;;//���Ĵ�ֵ���Զ�ÿ���������ͣʱ���������	
			//����������Լ�
			flight_global_cnt2[n]++;
		}
		
		//�ж����к����ִ����ϣ�ִ����һ����
		if(flight_global_cnt2[n]>=block_navpoint_num_basic)
		{
			flight_subtask_cnt[n]=3;//�����������
			flight_global_cnt[n]=0;
			execute_time_ms[n]=0;
			//�����������0
			flight_global_cnt2[n]=0;
			
			//����ִ�з������񺽵�����
			target_position.x=base_position.x;
			target_position.y=base_position.y;
			target_position.z=Patrol_Height;
			Horizontal_Navigation(target_position.x,target_position.y,target_position.z,GLOBAL_MODE,MAP_FRAME);
			return ;				
		}
	}
	else if(flight_subtask_cnt[n]==3)//������ɵ����Ϸ�
	{
		basic_auto_flight_support();//��������֧�����
		//�ж��Ƿ񵽴�Ŀ�꺽��λ��
		if(flight_global_cnt[n]<patrol_fixed_2d_times)//����10*5ms=0.05s����
		{
			float dis_cm=pythagorous2(OpticalFlow_Pos_Ctrl_Err.x,OpticalFlow_Pos_Ctrl_Err.y);
			if(dis_cm<=patrol_fixed_2d_5cm)	flight_global_cnt[n]++;
			else flight_global_cnt[n]/=2;
		}
		else//����10*5ms���㣬��ʾ����Ŀ�꺽��λ��
		{
			flight_subtask_cnt[n]++;
			flight_global_cnt[n]=0;
			execute_time_ms[n]=0;
			
			//���¸�λ���������ã����е����治ͬ�߶ȣ������ֲ�����仯��ɵ����
			//��ͬ�߶��ݶ��ϳ���仯����ʱ���������⴦���ֿ���ȥ��
			//���⴦��ʼ
			/*
			send_check_back=4;//����slam
			VIO_SINS.Position[_EAST] = 0;
			VIO_SINS.Position[_NORTH]= 0;
		  OpticalFlow_Pos_Ctrl_Expect.x=0;
		  OpticalFlow_Pos_Ctrl_Expect.y=0;
			*/
			//���⴦�����	
		}
	}	
	else if(flight_subtask_cnt[n]==4)//ԭ���½�
	{
		Flight.yaw_ctrl_mode=ROTATE;
		Flight.yaw_outer_control_output=RC_Data.rc_rpyt[RC_YAW];
		OpticalFlow_Control_Pure(0);
		Flight_Alt_Hold_Control(ALTHOLD_AUTO_VEL_CTRL,NUL,-30);//�߶ȿ���		
	}
	else
	{
		basic_auto_flight_support();//��������֧�����
	}
}


/*******************************************************************************/
uint8_t fire_source_check(void)
{
	if(Opv_Top_View_Target.trust_flag==1)	return 1;
	else return 0;
}


#define block_navpoint_num_innovation 7//�������ֺ�������
const int16_t block_innovation_coordinate[block_navpoint_num_innovation][2]={
	{100,155},//A
	{255,155},//B
	{410,100},//C1
	{410,150},//C2
	{410,305},//F
	{270,305},//E
	{115,305},//D
};
#define Patrol_Work_Height  100//Ѳ�߸߶�100cm
uint16_t release_pwm_us=1500;//�ͷ��������pwm
uint16_t pinch_pwm_us=1500;//��ס�������pwm,��Ҫ�ڳ�ʼ����ʱ��͸��������pwmֵ
int16_t fire_x=0,fire_y=0;//��Դλ��cm
uint8_t fire_flag=0;//��Դ��־

void substask_params_init(void)
{
	float tmp_pinch_pwm_us=0,tmp_release_pwm_us=0;
	ReadFlashParameterOne(RES_SERVO_PINCH_PWM_US  ,&tmp_pinch_pwm_us);
	ReadFlashParameterOne(RES_SERVO_RELEASE_PWM_US,&tmp_release_pwm_us);
	
  if(isnan(tmp_pinch_pwm_us)==0)   pinch_pwm_us=tmp_pinch_pwm_us;
	else pinch_pwm_us=1500;
  if(isnan(tmp_release_pwm_us)==0)   release_pwm_us=tmp_release_pwm_us;
	else release_pwm_us=1500;
}


void Air_Ground_Extinguish_Fire_System_Innovation(void)
{
	static uint8_t n=15;
	static float fx=0,fy=0;
	static uint16_t current_nav_cnt=0;
	Vector3f target_position;
	float x=0,y=0,z=0;	
	if(flight_subtask_cnt[n]==0)//��ɵ���Ϊ��һ����ͣ��
	{
		basic_auto_flight_support();//��������֧�����
		//����ʵ���
		laser_light1.period=100;//200*5ms
		laser_light1.light_on_percent=1.0f;
		laser_light1.reset=1;
		laser_light1.times=10000;
		
		//��¼�³�ʼ���λ�ã�ʵ����Ŀ�п�����Ϊĳһ��׼ԭ��
		//base_position.x=VIO_SINS.Position[_EAST];
		//base_position.y=VIO_SINS.Position[_NORTH];
		base_position.z=Patrol_Height;//Ѳ�߸߶�
		
		x=base_position.x;
		y=base_position.y;
		z=Patrol_Height;
		target_position.x=x;
		target_position.y=y;
		target_position.z=z;
		Horizontal_Navigation(target_position.x,target_position.y,target_position.z,GLOBAL_MODE,MAP_FRAME);
		
		flight_subtask_cnt[n]=1;
		flight_global_cnt[n]=0;
		flight_global_cnt2[n]=0;
		execute_time_ms[n]=1000/flight_subtask_delta;//������ִ��ʱ��
		
		//����Ԥ��PWMͨ��2�Զ������
		Reserved_PWM2_Output(pinch_pwm_us);//��ס����,����ֵ��Ҫ����ʵ�ʻ�е���Լ�����
	}
	else if(flight_subtask_cnt[n]==1)//���֮��ԭ����ͣ1S����ִ�к�������
	{
		basic_auto_flight_support();//��������֧�����
		//�ж����к����ִ����ϣ�ִ����һ����
		if(flight_global_cnt2[n]>=block_navpoint_num_innovation)
		{
			flight_subtask_cnt[n]=3;//�����������,׼����������
			flight_global_cnt[n]=0;
			execute_time_ms[n]=0;
			//�����������0
			flight_global_cnt2[n]=0;

			//��Ŀ�꺽������Ϊhome��,����ִ�з������񺽵�����
			target_position.x=base_position.x;
			target_position.y=base_position.y;
			target_position.z=Patrol_Height;
			Horizontal_Navigation(target_position.x,target_position.y,target_position.z,GLOBAL_MODE,MAP_FRAME);
			return ;				
		}
		
		uint16_t current_num=constrain_int32(flight_global_cnt2[n],0,block_navpoint_num_innovation-1);//�޷������		
		if(execute_time_ms[n]>0) execute_time_ms[n]--;
		if(execute_time_ms[n]==0)//��ͣʱ����������㣬��ͣ����ִ����� 
		{				
			//����A��������
			x=block_innovation_coordinate[current_num][0]-uav_home_center_coordinate[0];
			y=block_innovation_coordinate[current_num][1]-uav_home_center_coordinate[1];
			z=Patrol_Height;
			target_position.x=base_position.x+x;//ˮƽλ������Ϊ��ɺ��׼λ��+λ��Xƫ��
			target_position.y=base_position.y+y;//ˮƽλ������Ϊ��ɺ��׼λ��+λ��Yƫ��
			target_position.z=z;
			Horizontal_Navigation(target_position.x,target_position.y,target_position.z,GLOBAL_MODE,MAP_FRAME);
			flight_subtask_cnt[n]=2;
			flight_global_cnt[n]=0;
			execute_time_ms[n]=0;				
		}		
	}
	else if(flight_subtask_cnt[n]==2)//�����ɵ���ͣ��Ϻ󣬷�����һ��Ŀ���
	{
		basic_auto_flight_support();//��������֧�����		
		//�ж��Ƿ񵽴�Ŀ�꺽��λ��
		if(flight_global_cnt[n]<patrol_fixed_times)//����10*5ms=0.05s����
		{
			float dis_cm=pythagorous3(OpticalFlow_Pos_Ctrl_Err.x,OpticalFlow_Pos_Ctrl_Err.y,Total_Controller.Height_Position_Control.Err);
			if(dis_cm<=patrol_fixed_3d_10cm)	flight_global_cnt[n]++;
			else flight_global_cnt[n]/=2;
		}
		else//����10*5ms���㣬��ʾ����Ŀ�꺽��λ��
		{
			flight_subtask_cnt[n]=1;
			flight_global_cnt[n]=0;
			execute_time_ms[n]=2000/flight_subtask_delta;//���Ĵ�ֵ���Զ�ÿ���������ͣʱ���������	
			//����������Լ�
			flight_global_cnt2[n]++;
		}
		
		//�ж����к����ִ����ϣ�ִ����һ����
		if(flight_global_cnt2[n]>=block_navpoint_num_innovation)
		{
			flight_subtask_cnt[n]=3;//�����������
			flight_global_cnt[n]=0;
			execute_time_ms[n]=0;
			//�����������0
			flight_global_cnt2[n]=0;
			//����ִ�з������񺽵�����
			target_position.x=base_position.x;
			target_position.y=base_position.y;
			target_position.z=Patrol_Height;
			Horizontal_Navigation(target_position.x,target_position.y,target_position.z,GLOBAL_MODE,MAP_FRAME);
			return ;				
		}
		
		/**************************************************************************/
		if(fire_flag==1) return;//�����Դ��Ϣ�Ѿ�����������Ѳ�߲��ٶԻ�������жϣ������Ǻ����������
		//�жϵײ���Դ���
		if(flight_global_cnt2[n]>0)//�ɵ���һ������A֮��,�������жϻ���
		{
			if(Opv_Top_View_Target.trust_flag==1)//����Ӿ���⵽�˻���
			{
				flight_subtask_cnt[n]=5;//������ǰ�������������ִ��ɫ���׼
				flight_global_cnt[n]=0;
				execute_time_ms[n]=10000/flight_subtask_delta;//����ɫ���׼����ִ��ʱ��Ϊ10000ms
				//��¼��ǰ���������,
				current_nav_cnt=flight_global_cnt2[n];//��¼ִ�з������񺽵�����,���ڵֽ���Դ�����»ָ�Ѳ��
				//���ֻ����LEDָʾ��ʾ��
				laser_light2.period=200;
				laser_light2.light_on_percent=0.5f;
				laser_light2.reset=1;
				laser_light2.times=10;				
			}
		}
		/**************************************************************************/
	}
	else if(flight_subtask_cnt[n]==3)//������ɵ����Ϸ�
	{
		basic_auto_flight_support();//��������֧�����
		//�ж��Ƿ񵽴�Ŀ�꺽��λ��
		if(flight_global_cnt[n]<patrol_fixed_2d_times)//����10*5ms=0.05s����
		{
			float dis_cm=pythagorous2(OpticalFlow_Pos_Ctrl_Err.x,OpticalFlow_Pos_Ctrl_Err.y);
			if(dis_cm<=patrol_fixed_2d_5cm)	flight_global_cnt[n]++;
			else flight_global_cnt[n]/=2;
		}
		else//����10*5ms���㣬��ʾ����Ŀ�꺽��λ��
		{
			flight_subtask_cnt[n]++;
			flight_global_cnt[n]=0;
			execute_time_ms[n]=0;
			
			//���¸�λ���������ã����е����治ͬ�߶ȣ������ֲ�����仯��ɵ����
			//��ͬ�߶��ݶ��ϳ���仯����ʱ���������⴦���ֿ���ȥ��
			//���⴦��ʼ
			/*
			send_check_back=4;//����slam
			VIO_SINS.Position[_EAST] = 0;
			VIO_SINS.Position[_NORTH]= 0;
		  OpticalFlow_Pos_Ctrl_Expect.x=0;
		  OpticalFlow_Pos_Ctrl_Expect.y=0;
			*/
			//���⴦�����	
		}
	}	
	else if(flight_subtask_cnt[n]==4)//ԭ���½�
	{
		Flight.yaw_ctrl_mode=ROTATE;
		Flight.yaw_outer_control_output=RC_Data.rc_rpyt[RC_YAW];
		OpticalFlow_Control_Pure(0);
		Flight_Alt_Hold_Control(ALTHOLD_AUTO_VEL_CTRL,NUL,-30);//�߶ȿ���		
	}
	else if(flight_subtask_cnt[n]==5)//ִ��ɫ���׼��ʵ�����˻�������Դ�Ϸ��Ķ���
	{
		Color_Block_Control_Pilot();//����OPENMV�Ӿ�ˮƽ׷��
		//��ɫ���׼���Ƶ������������޷���ʹ��̬���ڹ��̾�����ƽ��
		Flight.roll_outer_control_output =constrain_float(Flight.roll_outer_control_output ,-10.0f,10.0f);
		Flight.pitch_outer_control_output=constrain_float(Flight.pitch_outer_control_output,-10.0f,10.0f);
		//ƫ����߶ȿ���
		Flight.yaw_ctrl_mode=ROTATE;
		Flight.yaw_outer_control_output =RC_Data.rc_rpyt[RC_YAW];			
		Flight_Alt_Hold_Control(ALTHOLD_MANUAL_CTRL,NUL,NUL);//�߶ȿ���
		
		//�������Լ������1
		//ɫ���׼ִ��100000ms��Ĭ������ɶ�׼
		if(execute_time_ms[n]>0) execute_time_ms[n]--;
		if(execute_time_ms[n]==0)//��ͣʱ����������㣬��ͣ����ִ����� 
		{
			OpticalFlow_Control_Pure(1);//ǿ��ˢ����ͣλ��
			//
			x=VIO_SINS.Position[_EAST];
			y=VIO_SINS.Position[_NORTH];
			z=Patrol_Work_Height;
			target_position.x=x;
			target_position.y=y;
			target_position.z=z;
			Horizontal_Navigation(target_position.x,target_position.y,target_position.z,GLOBAL_MODE,MAP_FRAME);
			
			flight_subtask_cnt[n]++;
			flight_global_cnt[n]=0;
			execute_time_ms[n]=0;
			
			//ɫ���׼ִ����Ϻ󣬽���ǰλ�ü�¼����
			fx=VIO_SINS.Position[_EAST]; //��Դλ��cm
			fy=VIO_SINS.Position[_NORTH];//��Դλ��cm
		}
		
		//�������Լ������2
		//�ж��Ƿ񵽴�Ŀ�����Ϸ�
		if(flight_global_cnt[n]<patrol_fixed_times)//����10*5ms=0.05s����
		{
			float dis_cm=pythagorous2(Opv_Top_View_Target.sdk_target.x,Opv_Top_View_Target.sdk_target.y);
			if(dis_cm<=patrol_fixed_3d_5cm)	flight_global_cnt[n]++;
			else flight_global_cnt[n]/=2;
		}
		else//����10*5ms���㣬��ʾ����Ŀ�꺽��λ��
		{
			OpticalFlow_Control_Pure(1);//ǿ��ˢ����ͣλ��
			//
			x=VIO_SINS.Position[_EAST];
			y=VIO_SINS.Position[_NORTH];
			z=Patrol_Work_Height;
			target_position.x=x;
			target_position.y=y;
			target_position.z=z;
			Horizontal_Navigation(target_position.x,target_position.y,target_position.z,GLOBAL_MODE,MAP_FRAME);
			
			flight_subtask_cnt[n]++;
			flight_global_cnt[n]=0;
			execute_time_ms[n]=0;
			
			//ɫ���׼ִ����Ϻ󣬽���ǰλ�ü�¼����
			fx=VIO_SINS.Position[_EAST]; //��Դλ��cm
			fy=VIO_SINS.Position[_NORTH];//��Դλ��cm
		}
	}
	else if(flight_subtask_cnt[n]==6)//�½��߶���100cm
	{
		basic_auto_flight_support();//��������֧�����	
		//�ж��Ƿ񵽴���ҵ�߶�
		if(flight_global_cnt[n]<200)//����200*5ms����
		{
			if(ABS(Total_Controller.Height_Position_Control.Err)<=10.0f)	flight_global_cnt[n]++;
			else flight_global_cnt[n]/=2;
		}		
		else//�ų�����
		{	
			flight_subtask_cnt[n]++;
			flight_global_cnt[n]=0;
			execute_time_ms[n]=3000/flight_subtask_delta;
		}	
	}
	else if(flight_subtask_cnt[n]==7)//��ͣ3S��,��������
	{
		basic_auto_flight_support();//��������֧�����
		
		if(execute_time_ms[n]>0) execute_time_ms[n]--;
		if(execute_time_ms[n]==0)//��ͣʱ����������㣬��ͣ����ִ����� 
		{
			//����Ԥ��PWMͨ��2�Զ������
			Reserved_PWM2_Output(release_pwm_us);//�ͷŶ���,����ֵ��Ҫ����ʵ�ʻ�е���Լ�����

			//���������Ҫһ��ʱ�䣬���������ʱ��2000ms
			flight_subtask_cnt[n]++;
			flight_global_cnt[n]=0;
			execute_time_ms[n]=2000/flight_subtask_delta;
		}			
	}
	else if(flight_subtask_cnt[n]==8)//�ͷ�����������2000ms
	{
		basic_auto_flight_support();//��������֧�����
		if(execute_time_ms[n]>0) execute_time_ms[n]--;
		if(execute_time_ms[n]==0)//��ͣʱ����������㣬��ͣ����ִ����� 
		{
			//���»�Դ��Ϣ,��������ͨ�����ڷ��͵�С��
			fire_flag=1;
			fire_x=(fx-base_position.x)+uav_home_center_coordinate[0];//ȥ����ʼƫ�ú��������
			fire_y=(fy-base_position.y)+uav_home_center_coordinate[1];//ȥ����ʼƫ�ú��������
			
			target_position.x=VIO_SINS.Position[_EAST];
			target_position.y=VIO_SINS.Position[_NORTH];
			target_position.z=Patrol_Height;
			Horizontal_Navigation(target_position.x,target_position.y,target_position.z,GLOBAL_MODE,MAP_FRAME);
			
			flight_subtask_cnt[n]++;
			flight_global_cnt[n]=0;
			execute_time_ms[n]=0;
		}			
	}
	else if(flight_subtask_cnt[n]==9)//�ָ���180cmѲ�߸߶�
	{
		basic_auto_flight_support();//��������֧�����	
		//�ж��Ƿ񵽴���ҵ�߶�
		if(flight_global_cnt[n]<200)//����200*5ms����
		{
			if(ABS(Total_Controller.Height_Position_Control.Err)<=10.0f)	flight_global_cnt[n]++;
			else flight_global_cnt[n]/=2;
		}		
		else//׼����������
		{	
			flight_subtask_cnt[n]=1;//1:�ֽ��鿴�����,����Ѳ��ʣ�µ�����
															//3:��쵽�����,��ǰ�����������,����ִ���߳�3������+ԭ�ؽ���
			flight_global_cnt[n]=0;
			execute_time_ms[n]=0;
			//�����������1,����ִ��ʣ��ĺ���
			flight_global_cnt2[n]=current_nav_cnt+1;
			
			//����ִ�з������񺽵�����
			target_position.x=base_position.x;
			target_position.y=base_position.y;
			target_position.z=Patrol_Height;
			Horizontal_Navigation(target_position.x,target_position.y,target_position.z,GLOBAL_MODE,MAP_FRAME);
		}	
	}
	else
	{
		basic_auto_flight_support();//��������֧�����
	}
}

