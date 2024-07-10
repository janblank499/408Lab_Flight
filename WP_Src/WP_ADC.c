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
#include "WP_ADC.h"
#include "hw_adc.h"
#include "adc.h"


uint32_t adc_value[8];
uint32_t wireless_adc_value[2];

void ADC0IntHandler(void);
/***********************************************************
@��������ADC_Init
@��ڲ�������
@���ڲ�������
������������ѹ�����˿�PE3����ΪADC0��ͨ��0��ʼ�������ж�ʽ�ɼ�
@���ߣ�����С��
@���ڣ�2024/01/20
*************************************************************/
void ADC_Init(void)//ADC��ʼ������   
{    
  SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC0);// Enable the ADC1 module.
  while(!SysCtlPeripheralReady(SYSCTL_PERIPH_ADC0));// Wait for the ADC1 module to be ready.	
  SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);    
  GPIOPinTypeADC(GPIO_PORTE_BASE, GPIO_PIN_3);//AIN0
	
  SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);    
//  GPIOPinTypeADC(GPIO_PORTD_BASE, GPIO_PIN_0);//AIN6	
//	GPIOPinTypeADC(GPIO_PORTD_BASE, GPIO_PIN_1);//AIN6
	GPIOPinTypeADC(GPIO_PORTD_BASE, GPIO_PIN_2);//AIN5
	GPIOPinTypeADC(GPIO_PORTD_BASE, GPIO_PIN_3);//AIN4
	
  //ADCReferenceSet(ADC0_BASE, ADC_REF_EXT_3V);
  //Enable the first sample sequencer to capture the value of channel 0 when
  //the processor trigger occurs.  
  ADCSequenceConfigure(ADC0_BASE,0,ADC_TRIGGER_PROCESSOR, 0); 
  //ADCHardwareOversampleConfigure(ADC0_BASE, 8);	
	ADCSequenceStepConfigure(ADC0_BASE, 0, 0, ADC_CTL_CH0);
	ADCSequenceStepConfigure(ADC0_BASE, 0, 1, ADC_CTL_CH4);
	ADCSequenceStepConfigure(ADC0_BASE, 0, 2, ADC_CTL_CH5|ADC_CTL_END | ADC_CTL_IE);
//	ADCSequenceStepConfigure(ADC0_BASE, 0, 3, ADC_CTL_CH6);
//	ADCSequenceStepConfigure(ADC0_BASE, 0, 4, ADC_CTL_CH7 |ADC_CTL_END | ADC_CTL_IE);
		
	ADCIntClear(ADC0_BASE, 0);
	ADCSequenceEnable(ADC0_BASE, 0);    
  ADCIntEnable(ADC0_BASE, 0); 
  //�жϴ�����ʽ����ADC_INT_SS0��ADC_INT_DMA_SS0��ADC_INT_DCON_SS0
	ADCIntEnableEx(ADC0_BASE,ADC_INT_SS0);//�ֱ������ͨ���д�����DMA���������ֱȽ�������
	IntEnable(INT_ADC0SS0); //ʹ��ADC���������ж�	
	ADCIntRegister(ADC0_BASE,0,ADC0IntHandler);		//�жϺ���ע��
	IntPrioritySet(INT_ADC0SS0, USER_INT7);	
} 


float Battery_Voltage;
/***********************************************************
@��������Get_Battery_Voltage
@��ڲ�������
@���ڲ�������
����������������ص�ѹ���ɿ�Ĭ�Ϸ�ѹ��Ϊ11���ʲ�����ѹ��Ҫ����
3.3V*11=36.6V����������ĵ�ѹ�����������Ϸ�ѹ������ֵ����
@���ߣ�����С��
@���ڣ�2024/01/20
*************************************************************/
void Get_Battery_Voltage(void)//ADC��ȡ   
{
//	ADCProcessorTrigger(ADC0_BASE, 0);   
//	while(!ADCIntStatus(ADC0_BASE, 0, false)) {;}
//	ADCIntClear(ADC0_BASE, 0);	
//	ADCSequenceDataGet(ADC0_BASE, 0, value);   
	//value[0] =  HWREG(ADC0_BASE+ ADC_SEQ + (ADC_SEQ_STEP*0) + ADC_SSFIFO);
	static float value_filter=0;
	value_filter=0.9f*value_filter+0.1f*adc_value[0]*36.3f/4095.0f;	
	Battery_Voltage=value_filter;	
}


void ADC_Sample_Trigger(void)
{
	static uint16_t cnt=0;
	cnt++;
	if(cnt<=2) return;
	cnt=0;
	ADCProcessorTrigger(ADC0_BASE, 0);
}

Testime ADC_Delta;
void ADC0IntHandler(void)
{
	Test_Period(&ADC_Delta);
	ADCIntClear(ADC0_BASE, 0);// ���ADC�жϱ�־��
	//while(!ADCIntStatus(ADC0_BASE, 0, false));//�ȴ��ɼ�����
	ADCSequenceDataGet(ADC0_BASE, 0, adc_value);// ��ȡADCֵ
}


static uint16_t low_vbat_cnt=0;
void Battery_Voltage_Detection(void)
{
	static uint16_t _cnt=0;
	_cnt++;
	if(_cnt>=200)//ÿ1S���һ��
	{
		_cnt=0;
		if(Battery_Voltage<Safe_Vbat/1000.0f&&Battery_Voltage>6.0f)	 low_vbat_cnt++;
		else low_vbat_cnt/=2;
		if(low_vbat_cnt>=10)//����10s����
		{
			low_vbat_cnt=0;//��0����һ���ڼ��
			if(Controler_State==Lock_Controler)//����״̬�²������������ѹ��������	
				buzzer_setup(1000,0.25f,5);//��˸5��			
		}			
	}
}

