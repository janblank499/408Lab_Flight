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
#include "Temperature_Ctrl.h"

#define Temperature_Ctrl_Enable   1

#define Temperature_Setpoint   50
const float Temp_Control_Unit[20]={1  ,1 ,1 ,0 ,0 ,0 , 0 ,50    ,10  ,0 ,80,  8.00  ,0.75   ,125.0  ,0    ,0 ,100 , 1 ,  1 ,  1 };
/*
1ƫ���޷���־��  2�����޷���־��3���ַ����־��   4������
5����            6ƫ�        7�ϴ�ƫ�       8ƫ���޷�ֵ��
9���ַ���ƫ��ֵ��10����ֵ       11�����޷�ֵ��    12���Ʋ���Kp��
13���Ʋ���Ki��   14���Ʋ���Kd�� 15�������������  16�ϴο����������
17������޷��ȣ� 18����ֿ���ʱ�Ļ�������
*/		
void Temperature_Ctrl_Init(void)
{
  Total_Controller.IMU_Temperature_Control.Err_Limit_Flag=(uint8)(Temp_Control_Unit[0]);//1ƫ���޷���־
  Total_Controller.IMU_Temperature_Control.Integrate_Limit_Flag=(uint8)(Temp_Control_Unit[1]);//2�����޷���־
  Total_Controller.IMU_Temperature_Control.Integrate_Separation_Flag=(uint8)(Temp_Control_Unit[2]);//3���ַ����־
  Total_Controller.IMU_Temperature_Control.Expect=Temp_Control_Unit[3];//4����
  Total_Controller.IMU_Temperature_Control.FeedBack=Temp_Control_Unit[4];//5����ֵ
  Total_Controller.IMU_Temperature_Control.Err=Temp_Control_Unit[5];//6ƫ��
  Total_Controller.IMU_Temperature_Control.Last_Err=Temp_Control_Unit[6];//7�ϴ�ƫ��
  Total_Controller.IMU_Temperature_Control.Err_Max=Temp_Control_Unit[7];//8ƫ���޷�ֵ
  Total_Controller.IMU_Temperature_Control.Integrate_Separation_Err=Temp_Control_Unit[8];//9���ַ���ƫ��ֵ
  Total_Controller.IMU_Temperature_Control.Integrate=Temp_Control_Unit[9];//10����ֵ
  Total_Controller.IMU_Temperature_Control.Integrate_Max=Temp_Control_Unit[10];//11�����޷�ֵ
  Total_Controller.IMU_Temperature_Control.Kp=Temp_Control_Unit[11];//12���Ʋ���Kp
  Total_Controller.IMU_Temperature_Control.Ki=Temp_Control_Unit[12];//13���Ʋ���Ki
  Total_Controller.IMU_Temperature_Control.Kd=Temp_Control_Unit[13];//14���Ʋ���Ki
  Total_Controller.IMU_Temperature_Control.Control_OutPut=Temp_Control_Unit[14];//15�����������
  Total_Controller.IMU_Temperature_Control.Last_Control_OutPut=Temp_Control_Unit[15];//16�ϴο����������
  Total_Controller.IMU_Temperature_Control.Control_OutPut_Limit=Temp_Control_Unit[16];//17�ϴο����������
}

void Simulation_PWM_Init(void)
{
  SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOC);
  HWREG(GPIO_PORTC_BASE + GPIO_O_LOCK) = GPIO_LOCK_KEY; 
	HWREG(GPIO_PORTC_BASE + GPIO_O_AFSEL) |= GPIO_PIN_2; 
	HWREG(GPIO_PORTC_BASE + GPIO_O_LOCK) = 0x0;
  
	HWREG(GPIO_PORTC_BASE + GPIO_O_LOCK) = GPIO_LOCK_KEY; 
	HWREG(GPIO_PORTC_BASE + GPIO_O_AFSEL) |= GPIO_PIN_3; 
	HWREG(GPIO_PORTC_BASE + GPIO_O_LOCK) = 0x0;	
	
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);	
  GPIOPinTypeGPIOOutput(GPIO_PORTE_BASE, GPIO_PIN_2);
	
	GPIOPinWrite(GPIO_PORTE_BASE,GPIO_PIN_2,0);
}

#define Simulation_PWM_Period_MAX  100//100*1ms=0.1S
void Simulation_PWM_Output(uint16_t width)
{
	if(Sensor_Flag.Mpu_Health==0)
	{
		GPIOPinWrite(GPIO_PORTE_BASE,GPIO_PIN_2,0);
		return ;
	}
#if Temperature_Ctrl_Enable
	uint16_t static cnt=0;
	cnt++;
	if(cnt>=Simulation_PWM_Period_MAX)  cnt=0;
  if(cnt<=width) GPIOPinWrite(GPIO_PORTE_BASE,GPIO_PIN_2,GPIO_PIN_2);
	else GPIOPinWrite(GPIO_PORTE_BASE,GPIO_PIN_2,0);
#else
	GPIOPinWrite(GPIO_PORTE_BASE,GPIO_PIN_2,0);
#endif
	
}


void Temperature_Ctrl(void)
{
#if Temperature_Ctrl_Enable
 static uint16_t tmp_period_cnt=0;
 tmp_period_cnt++;
 Total_Controller.IMU_Temperature_Control.Expect=Temperature_Setpoint;
 Total_Controller.IMU_Temperature_Control.FeedBack=WP_Sensor._temperature;
 if(tmp_period_cnt>=20)  //8*2.5ms=20ms
 {
	 PID_Control_Div_LPF(&Total_Controller.IMU_Temperature_Control,0.02f);
	 Total_Controller.IMU_Temperature_Control.Control_OutPut=constrain_float(Total_Controller.IMU_Temperature_Control.Control_OutPut,0,Simulation_PWM_Period_MAX);
	 tmp_period_cnt=0;
 }
#else
	Total_Controller.IMU_Temperature_Control.Control_OutPut=0;
#endif
}

uint8_t Temperature_State_Get(void)
{
#if Temperature_Ctrl_Enable
  return (ABS(Total_Controller.IMU_Temperature_Control.Err))<=2.0f?1:0;
#else
	return 1;
#endif	
}

uint8_t Temperature_Stable_Flag=0;
void Temperature_State_Check(void)
{
	static uint16_t _cnt=0;
	static uint16_t temperature_crash_cnt=0;
	if(Temperature_State_Get()==1){
		_cnt++;
		if(_cnt>=400) Temperature_Stable_Flag=1;
	}
	else{
		_cnt/=2;
	}
	
	if(temperature_crash_cnt<400)
	{
		if(WP_Sensor.last_temperature==WP_Sensor.temperature)	temperature_crash_cnt++;
		else temperature_crash_cnt/=2;
		Sensor_Flag.Mpu_Health=1;		
	}
	else
	{
		Sensor_Flag.Mpu_Health=0;
		Controler_State=Lock_Controler;//����
	}	
}	
