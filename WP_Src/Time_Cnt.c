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
#include "Time_Cnt.h"


void Time0A_init(void)//ϵͳ���ȶ�ʱ����ʼ��
{
  SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER0);							//��ʱ��0ʹ��				
  TimerConfigure(TIMER0_BASE,TIMER_CFG_PERIODIC);						//32λ���ڶ�ʱ��				
  TimerLoadSet(TIMER0_BASE,TIMER_A,SysCtlClockGet()/200);		//�趨װ��ֵ,��80M/200��*1/80M=5ms				
  IntEnable(INT_TIMER0A);												//���ж�ʹ��				
  TimerIntEnable(TIMER0_BASE,TIMER_TIMA_TIMEOUT); 					//�ж����, ����ģʽ;			
  TimerIntRegister(TIMER0_BASE,TIMER_A,TIMER0A_Handler);		//�жϺ���ע��
//  IntMasterEnable();			
  TimerEnable(TIMER0_BASE,TIMER_A); 												//��ʱ��ʹ�ܿ�ʼ����
  IntPrioritySet(INT_TIMER0A,USER_INT6);
}


void Time1A_init(void)
{
  SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER1);//��ʱ��1ʹ��				
  TimerConfigure(TIMER1_BASE,TIMER_CFG_PERIODIC_UP);//32λ���ڶ�ʱ��
  TimerLoadSet(TIMER1_BASE,TIMER_A,SysCtlClockGet()/1000);//�趨װ��ֵ,��80M/1000*1/80M=1.0ms				
  IntEnable(INT_TIMER1A);//��ʱ��1�ж�ʹ��				
  TimerIntEnable(TIMER1_BASE,TIMER_TIMA_TIMEOUT); //�ж����, ����ģʽ;
  TimerIntRegister(TIMER1_BASE,TIMER_A,TIMER1A_Handler);//�жϺ���ע��
//  IntMasterEnable();			
  TimerEnable(TIMER1_BASE,TIMER_A); //��ʱ��ʹ�ܿ�ʼ����	
  //IntPriorityGroupingSet(0);	
  IntPrioritySet(INT_TIMER1A,USER_INT5);
}


void Time2A_init(void)//ϵͳ���ȶ�ʱ����ʼ��
{
  SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER2);							//��ʱ��2ʹ��				
  TimerConfigure(TIMER2_BASE,TIMER_CFG_PERIODIC);						//32λ���ڶ�ʱ��				
  TimerLoadSet(TIMER2_BASE,TIMER_A,SysCtlClockGet()/100);		//�趨װ��ֵ,��80M/100��*1/80M=10ms				
  IntEnable(INT_TIMER2A);																		//���ж�ʹ��				
  TimerIntEnable(TIMER2_BASE,TIMER_TIMA_TIMEOUT); 					//�ж����, ����ģʽ;			
  TimerIntRegister(TIMER2_BASE,TIMER_A,TIMER2A_Handler);		//�жϺ���ע��
//  IntMasterEnable();			
  TimerEnable(TIMER2_BASE,TIMER_A); 												//��ʱ��ʹ�ܿ�ʼ����
  IntPrioritySet(INT_TIMER2A,USER_INT7);
}

void Time3A_init(void)//ϵͳ���ȶ�ʱ����ʼ��
{
  SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER3);							//��ʱ��3ʹ��				
  TimerConfigure(TIMER3_BASE,TIMER_CFG_PERIODIC);						//32λ���ڶ�ʱ��				
  TimerLoadSet(TIMER3_BASE,TIMER_A,SysCtlClockGet()/1000);		//�趨װ��ֵ,��80M/1000��*1/80M=1.0ms				
  IntEnable(INT_TIMER3A);																		//���ж�ʹ��				
  TimerIntEnable(TIMER3_BASE,TIMER_TIMA_TIMEOUT); 					//�ж����, ����ģʽ;			
  TimerIntRegister(TIMER3_BASE,TIMER_A,TIMER3A_Handler);		//�жϺ���ע��
//  IntMasterEnable();			
  TimerEnable(TIMER3_BASE,TIMER_A); 												//��ʱ��ʹ�ܿ�ʼ����
  IntPrioritySet(INT_TIMER3A,USER_INT5);
}


/***********************************************************
@��������Time_init
@��ڲ�������
@���ڲ�������
@����������ϵͳ���ȶ�ʱ����ʼ��
@���ߣ�����С��
@���ڣ�2024/01/20
*************************************************************/
void Time_init(void)//ϵͳ���ȶ�ʱ����ʼ��
{
	IntMasterDisable();
  Time0A_init();//��ʱ��5ms
	Time1A_init();//��ʱ��1ms
	Time2A_init();//��ʱ��10ms
	Time3A_init();//��ʱ��1ms
	IntMasterEnable();	
}





Sensor WP_Sensor;
AHRS 	 WP_AHRS;
Testime Time0_Delta,Time0_Delta1;
float time0_max;
/***********************************************************
@��������TIMER0A_Handler
@��ڲ�������
@���ڲ�������
@����������ϵͳ���ȶ�ʱ���жϷ���������Ҫ����ң����������
�������ݲɼ��������˲�����̬���㡢���Ե��������Ƶȶ�������
�ϸ�Ҫ��ĺ���
@���ߣ�����С��
@���ڣ�2024/01/20
*************************************************************/
void TIMER0A_Handler(void)				//ϵͳ�����жϺ���   4.4ms
{  
	Test_Period(&Time0_Delta);
	Remote_Control();								  //ң�������ݽ���	
	OBS_Sensor_Update();						  //���������ݸ���	
	Get_Status_Feedback();					  //ˮƽ����ֱ����ߵ�����
	Optflow_Statemachine();					  //����״̬������ʼ��ʱ���ڹ�������
  GPS_Data_Prase();								  //GPS���ݽ���
  KalmanFilter_Horizontal();			  //ˮƽλ��GPS˫�۲���Kalman�˲��ں�
	Ground_Sensor_Statemachine();     //�Եؾ��봫����״̬�����£�������/tfmini_plus/VL53L1X
	CarryPilot_Control();	            //�ܿ�����
  Calibration_All_Check();					//У׼��ؼ��
	Temperature_State_Check();			  //�¿�ϵͳ���
  Bling_Working(Bling_Mode);			  //״ָ̬ʾ������
	ADC_Sample_Trigger();						  //ADC��ѹ�ɼ�
	Read_Button_State_All();				  //����״̬��ȡ
	ADC_Button_Scan();							  //�������ݴ���
	Battery_Voltage_Detection();			//��ص�ѹ��⡪������������
	Laser_Light_Work(&laser_light1); //�����/RGB��/������1����
	Laser_Light_Work(&laser_light2); //�����/RGB��/������2����
	Board_Buzzer_Work(&buzzer);       //��Դ�����������
	NCLink_Send_IMU_Feedback_PC();		//�������˻�״̬���ݸ����ؼ����������ݮ��4B������ܣ�https://item.taobao.com/item.htm?spm=a230r.1.14.6.1e7575f8M0CgLF&id=669633894747&ns=1&abbucket=12#detail
	Test_Period(&Time0_Delta1);
	float tmp=Time0_Delta1.Now_Time-Time0_Delta.Now_Time;
	if(time0_max<tmp) time0_max=tmp;
	TimerIntClear(TIMER0_BASE,TIMER_TIMA_TIMEOUT);
}


Testime Time1_Delta,Time1_Delta1;
float time1_max;
void TIMER1A_Handler(void)//�¿��жϺ���
{
	Test_Period(&Time1_Delta);
	Temperature_Ctrl();	
  Simulation_PWM_Output(Total_Controller.IMU_Temperature_Control.Control_OutPut);
	lsn10_data_prase();
	Test_Period(&Time1_Delta1);
	float tmp=Time1_Delta1.Now_Time-Time1_Delta.Now_Time;
	if(time1_max<tmp) time1_max=tmp;	
	TimerIntClear(TIMER1_BASE,TIMER_TIMA_TIMEOUT);  	
}



Testime Time2_Delta;
void TIMER2A_Handler(void)//����վ���ݷ����жϺ���
{
	Test_Period(&Time2_Delta);
	NCLink_SEND_StateMachine();			//�������¿�Դ����վ����������http://nameless.tech/download.html
	//Vcan_Send();
	laser_state_sort();							//����N10�����״����ݽ���

	speaker_notify_run(&speaker_mode);

	static uint16_t _cnt=0;
	_cnt++;
	if(_cnt>=20)
	{
		//ÿ20*10msִ��һ��
		NCLink_Send_To_Firetruck(VIO_SINS.Position[_EAST],
														 VIO_SINS.Position[_NORTH],
													   NamelessQuad.Position[_UP],
													   WP_Sensor.distance_3d_cm,
														 fire_x,fire_y,fire_flag);
		_cnt=0;
	}
	TimerIntClear(TIMER2_BASE,TIMER_TIMA_TIMEOUT);
}



systime Time3_Delta;
void TIMER3A_Handler(void)//����վ���ݷ����жϺ���
{
  Get_Systime(&Time3_Delta);
  INS_Sensor_Update();//��ȡ��̬����
	TimerIntClear(TIMER3_BASE,TIMER_TIMA_TIMEOUT);
}


