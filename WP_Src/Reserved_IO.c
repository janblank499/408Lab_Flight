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
#include "Reserved_IO.h"



//LaunchPad�汾�ɿ�ʱ�����´����ӣ�https://www.bilibili.com/read/cv12740760
//TIVA LaunchPad�ɿ���չ��Ԥ����IO1����PD0��IO2����PD1����ֱ����
//��Ҫ�õ�IO1��IO2ʱ������ȡ��launchpad�ϵ�R9��R10����


#define RESERVED_IO_ENABLE 1
//ʹ��ǰ�����������ȡ��R9/R10������ɿ�PWM����᲻�������ᵼ��ը��

_laser_light laser_light1;//�����1
_laser_light laser_light2;//��Ӹ߷ֱ�������/�����2
_laser_light buzzer;//��չ��/��Դ�������
void Reserved_IO_Init(void)
{
#if RESERVED_IO_ENABLE
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOC);
	HWREG(GPIO_PORTC_BASE + GPIO_O_LOCK) = GPIO_LOCK_KEY;
  HWREG(GPIO_PORTC_BASE + GPIO_O_CR) |= GPIO_PIN_2;
  HWREG(GPIO_PORTC_BASE + GPIO_O_LOCK) = 0;
	
  GPIOPinTypeGPIOOutput(GPIO_PORTD_BASE, GPIO_PIN_0|GPIO_PIN_1);
  GPIOPadConfigSet(GPIO_PORTD_BASE,GPIO_PIN_0|GPIO_PIN_1,GPIO_STRENGTH_4MA,GPIO_PIN_TYPE_STD);
	
  //����������
	GPIODirModeSet(GPIO_PORTC_BASE, GPIO_PIN_2,GPIO_DIR_MODE_OUT);//BEEP
	GPIOPadConfigSet(GPIO_PORTC_BASE,GPIO_PIN_2,GPIO_STRENGTH_2MA,GPIO_PIN_TYPE_STD);
	
	GPIOPinWrite(GPIO_PORTD_BASE,GPIO_PIN_0,0);
	GPIOPinWrite(GPIO_PORTD_BASE,GPIO_PIN_1,0);
  GPIOPinWrite(GPIO_PORTC_BASE,GPIO_PIN_2,GPIO_PIN_2);
#endif
	
	laser_light1.port=GPIO_PORTD_BASE;
	laser_light1.pin=GPIO_PIN_0;
	laser_light1.period=200;//200*5ms
	laser_light1.light_on_percent=0.5f;
	laser_light1.reset=0;
	
	
	laser_light2.port=GPIO_PORTD_BASE;
	laser_light2.pin=GPIO_PIN_1;
	laser_light2.period=200;//200*5ms
	laser_light2.light_on_percent=0.5f;
	
	buzzer.port=GPIO_PORTC_BASE;
	buzzer.pin=GPIO_PIN_2;
	buzzer.period=200;//200*5ms
	buzzer.light_on_percent=0.2f;
	buzzer.reset=0;		
	
	//������ʼ����˸����֤Ӳ���������
	laser_light1.reset=1;
	laser_light1.times=5;
	laser_light2.reset=1;
	laser_light2.times=5;
}



void Laser_Light_Work(_laser_light *light)
{
	if(light->reset==1)
	{
		light->reset=0;
		light->cnt=0;
		light->times_cnt=0;//��������������
		light->end=0;
	}
	
	if(light->times_cnt==light->times)
	{
		light->end=1;
		return;
	}

	light->cnt++;
	if(light->cnt<=light->period*light->light_on_percent)
	{
		GPIOPinWrite(light->port,light->pin,light->pin);
	}
	else if(light->cnt<light->period)
	{
		GPIOPinWrite(light->port,light->pin,0);
	}
	else//��ɵ���һ��
	{
		GPIOPinWrite(light->port,light->pin,0);
		light->cnt=0;
		light->times_cnt++;
	}
}


void Board_Buzzer_Work(_laser_light *light)
{
	if(light->reset==1)
	{
		light->reset=0;
		light->cnt=0;
		light->times_cnt=0;//��������������
		light->end=0;
	}
	
	if(light->times_cnt==light->times)
	{
		light->end=1;
		return;
	}

	light->cnt++;
	if(light->cnt<light->period*light->light_on_percent)
	{
		GPIOPinWrite(light->port,light->pin,0);
	}
	else if(light->cnt<light->period)
	{
		GPIOPinWrite(light->port,light->pin,light->pin);
	}
	else//��ɵ���һ��
	{
		GPIOPinWrite(light->port,light->pin,light->pin);
		light->cnt=0;
		light->times_cnt++;
	}
}

void buzzer_setup(uint32_t _period,float _light_on_percent,uint16_t _times)
{
	buzzer.period=_period/5;//20*5ms
	buzzer.light_on_percent=_light_on_percent;
	buzzer.reset=1;	
	buzzer.times=_times;
}

