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
#include "ADC_Button.h"

uint8_t adc_key1_ctrl_enable=0;//ʹ�ܰ�������ʱ����Ϊ1����֮����Ϊ0
uint8_t adc_key2_ctrl_enable=0;
uint8_t wireless_adc_key_ctrl_enable=0;
#define ADC_LONG_PRESS_LIMIT  1500//1500ms
#define ADC_IN_PRESS_LIMIT  	500//250ms


button_state ADC_Button1,ADC_Button2,ADC_Button3,ADC_Button4;
float Button_ADCResult[2];
uint8_t Key_Value[2]={0,0};
uint16_t ADC_PPM_Databuf[10]={0};

void ADC_Button_Init()
{
	ADC_Button1.port=GPIO_PORTD_BASE;
	ADC_Button1.pin=GPIO_PIN_3;
	ADC_Button1.value=1;
	ADC_Button1.last_value=1;

	ADC_Button2.port=GPIO_PORTD_BASE;
	ADC_Button2.pin=GPIO_PIN_3;
	ADC_Button2.value=1;
	ADC_Button2.last_value=1;

	ADC_Button3.port=GPIO_PORTD_BASE;
	ADC_Button3.pin=GPIO_PIN_3;
	ADC_Button3.value=1;
	ADC_Button3.last_value=1;

	ADC_Button4.port=GPIO_PORTD_BASE;
	ADC_Button4.pin=GPIO_PIN_3;
	ADC_Button4.value=1;
	ADC_Button4.last_value=1;
	
	float tmp_adckey_enable[3]={0};
	ReadFlashParameterOne(ADC_KEY1_ENABLE,&tmp_adckey_enable[0]);
	ReadFlashParameterOne(ADC_KEY2_ENABLE,&tmp_adckey_enable[1]);
	ReadFlashParameterOne(WIRELESS_ADC_KEY_ENABLE,&tmp_adckey_enable[2]);
	
	if(isnan(tmp_adckey_enable[0])==0)   adc_key1_ctrl_enable=tmp_adckey_enable[0];
	else adc_key1_ctrl_enable=0;

	if(isnan(tmp_adckey_enable[1])==0)   adc_key2_ctrl_enable=tmp_adckey_enable[1];
	else adc_key2_ctrl_enable=0;
	
	if(isnan(tmp_adckey_enable[2])==0)   wireless_adc_key_ctrl_enable=tmp_adckey_enable[2];
	else wireless_adc_key_ctrl_enable=0;
}





void ADC_Button_Read(void)
{	
	//��ԭʼ������12λADC������ת���ɵ�ѹ,��λV
	if(wireless_adc_key_ctrl_enable==1)//adc��ѹ��Դ�������������䣬��С��
	{
		Button_ADCResult[0]=wireless_adc_value[0]*3.3f/4095.0f;//PD3
		Button_ADCResult[1]=wireless_adc_value[1]*3.3f/4095.0f;//PD2
	}
	else//adc��ѹ��Դ�ڷɿ�����˿ڲɼ�
	{
		Button_ADCResult[0]=adc_value[1]*3.3f/4095.f;//PD3
		Button_ADCResult[1]=adc_value[2]*3.3f/4095.f;//PD2
	}

	
	if(Button_ADCResult[0]<=0.5f)  		   Key_Value[0]=0x00;//û�а������£�����5.1K������0V
	else if(Button_ADCResult[0]<=1.70f)  Key_Value[0]=0x04;//���䰴�����£�	 �Ͻ�5.1K������3.3V*5.1/10.2=1.65V
	else if(Button_ADCResult[0]<=2.10f)  Key_Value[0]=0x02;//�����������£�  �Ͻ�3.3K������3.3V*5.1/8.40=2.00V
	else if(Button_ADCResult[0]<=2.40f)  Key_Value[0]=0x03;//�����������£�  �Ͻ�2.2K������3.3V*5.1/7.30=2.31V
	else if(Button_ADCResult[0]<=3.00f)  Key_Value[0]=0x01;//�����������£�  �Ͻ�1.2K������3.3V*5.1/6.30=2.67V
	else Key_Value[0]=0x00;

  switch(Key_Value[0])
	{
		case 0://û�а�������
		{
			ADC_Button1.value=1;
			ADC_Button2.value=1;
			ADC_Button3.value=1;
			ADC_Button4.value=1;
		}
		break;
		case 1://������������
		{
			ADC_Button1.value=0;
			ADC_Button2.value=1;
			ADC_Button3.value=1;
			ADC_Button4.value=1;		
		}
		break;		
		case 2://������������
		{
			ADC_Button1.value=1;
			ADC_Button2.value=0;
			ADC_Button3.value=1;
			ADC_Button4.value=1;		
		}
		break;		
		case 3://������������
		{
			ADC_Button1.value=1;
			ADC_Button2.value=1;
			ADC_Button3.value=0;
			ADC_Button4.value=1;		
		}
		break;	
		case 4://���䰴������
		{
			ADC_Button1.value=1;
			ADC_Button2.value=1;
			ADC_Button3.value=1;
			ADC_Button4.value=0;		
		}
		break;	
	  default://û�а�������
		{
			ADC_Button1.value=1;
			ADC_Button2.value=1;
			ADC_Button3.value=1;
			ADC_Button4.value=1;				
		}
		break;	
	}
	
	
	//
	if(Button_ADCResult[1]<=0.5f)  		   Key_Value[1]=0x00;//û�а������£�����5.1K������0V
	else if(Button_ADCResult[1]<=1.35f)  Key_Value[1]=0x05;//�ϰ������£�	�Ͻ�10K������3.3V*5.1/15.1=1.11V
	else if(Button_ADCResult[1]<=1.70f)  Key_Value[1]=0x04;//�Ұ������£�	�Ͻ�5.1K������3.3V*5.1/10.2=1.65V
	else if(Button_ADCResult[1]<=2.10f)  Key_Value[1]=0x03;//�󰴼����£� �Ͻ�3.3K������3.3V*5.1/8.40=2.00V
	else if(Button_ADCResult[1]<=2.40f)  Key_Value[1]=0x02;//�а������£� �Ͻ�2.2K������3.3V*5.1/7.30=2.31V
	else if(Button_ADCResult[1]<=3.00f)  Key_Value[1]=0x01;//�°������£� �Ͻ�1.2K������3.3V*5.1/6.30=2.67V
	else Key_Value[1]=0x00;
	
	switch(Key_Value[1])
	{
		case 0://û�а�������
		{
			_button.state[UP_3D].value=1;
			_button.state[DN_3D].value=1;
			_button.state[LT_3D].value=1;
			_button.state[RT_3D].value=1;
			_button.state[ME_3D].value=1;
		}
		break;
		case 1://�°�������
		{
			_button.state[UP_3D].value=1;
			_button.state[DN_3D].value=0;
			_button.state[LT_3D].value=1;
			_button.state[RT_3D].value=1;
			_button.state[ME_3D].value=1;	
		}
		break;		
		case 2://�а�������
		{
			_button.state[UP_3D].value=1;
			_button.state[DN_3D].value=1;
			_button.state[LT_3D].value=1;
			_button.state[RT_3D].value=1;
			_button.state[ME_3D].value=0;	
		}
		break;		
		case 3://�󰴼�����
		{
			_button.state[UP_3D].value=1;
			_button.state[DN_3D].value=1;
			_button.state[LT_3D].value=0;
			_button.state[RT_3D].value=1;
			_button.state[ME_3D].value=1;	
		}
		break;	
		case 4://�Ұ�������
		{
			_button.state[UP_3D].value=1;
			_button.state[DN_3D].value=1;
			_button.state[LT_3D].value=1;
			_button.state[RT_3D].value=0;
			_button.state[ME_3D].value=1;	
		}
		break;
		case 5://�ϰ�������
		{
			_button.state[UP_3D].value=0;
			_button.state[DN_3D].value=1;
			_button.state[LT_3D].value=1;
			_button.state[RT_3D].value=1;
			_button.state[ME_3D].value=1;	
		}
		break;		
	  default://û�а�������
		{
			_button.state[UP_3D].value=1;
			_button.state[DN_3D].value=1;
			_button.state[LT_3D].value=1;
			_button.state[RT_3D].value=1;
			_button.state[ME_3D].value=1;			
		}
		break;	
	}
}



void Read_ADC_Button_State_One(button_state *button)
{
	if(button->value==0)
	{
		if(button->last_value!=0)//�״ΰ���
		{
			button->press_time=millis();//��¼���µ�ʱ���
			button->in_time=millis();//��¼���µ�ʱ���
			button->in_press_cnt=0;
		}
		else
		{
			if(millis()-button->in_time>ADC_IN_PRESS_LIMIT)//��������
			{
				button->in_time=millis();
				button->press=IN_PRESS;

				if(button->press==IN_PRESS)  button->in_press_cnt++;
			}
		}
	}
  else//�����ͷ�
	{
		if(button->last_value==0)//���º��ͷ�
		{
			button->release_time=millis();//��¼�ͷ�ʱ��ʱ��
			
			if(button->release_time-button->press_time>ADC_LONG_PRESS_LIMIT)
			{
			   button->press=LONG_PRESS;
				 button->state_lock_time=0;//5ms*300=1.5S
				 
				 buzzer_setup(1000,0.5f,1);
			}
			else
			{
			   button->press=SHORT_PRESS;
				 button->state_lock_time=0;//5ms*300=1.5S
				 buzzer_setup(100,0.5f,1);
			}
		}
	}
	button->last_value=button->value;
	
	
	if(button->press==LONG_PRESS
	 ||button->press==SHORT_PRESS)//�����ͷź󣬳����̨1.5S������Ӧ����λ����״̬
	{
	  button->state_lock_time++;
		if(button->state_lock_time>=300)
		{			
			 button->press=NO_PRESS;
			 button->state_lock_time=0;
		}
	}
}


void Read_ADC_Button1_State_All(void)
{
	if(adc_key1_ctrl_enable==0)  return ;//ADC����1δʹ��ʱ��ֱ�ӷ���
	
	Read_ADC_Button_State_One(&ADC_Button1);
	Read_ADC_Button_State_One(&ADC_Button2);
	Read_ADC_Button_State_One(&ADC_Button3);
	Read_ADC_Button_State_One(&ADC_Button4);
}

void Read_ADC_Button2_State_All(void)
{
	if(adc_key2_ctrl_enable==0)  return ;//ADC����2δʹ��ʱ��ֱ�ӷ���
	
	Read_ADC_Button_State_One(&_button.state[UP_3D]);
	Read_ADC_Button_State_One(&_button.state[DN_3D]);
	Read_ADC_Button_State_One(&_button.state[LT_3D]);
	Read_ADC_Button_State_One(&_button.state[RT_3D]);
	Read_ADC_Button_State_One(&_button.state[ME_3D]);
}



uint8_t adc_ppm_update_flag=0;

void ADC_Button_Scan(void)
{
	ADC_Button_Read();
	Read_ADC_Button1_State_All();
  Read_ADC_Button2_State_All();
	
	if(adc_key1_ctrl_enable==0)  return ;//ADC����1δʹ��ʱ��ֱ�ӷ���
	
	//����Ϊ���������¼������߼�
	if(ADC_Button1.press==IN_PRESS)
	{
		//����Ϊ�ð���ʵ�ֵ�����
		adc_ppm_update_flag=1;//����Ȩ�޸�ADC�������õ�PPM
		ADC_PPM_Databuf[0]=1500;//������λ����λ
		ADC_PPM_Databuf[1]=1500;//�����λ����λ
		ADC_PPM_Databuf[2]=1000;//���Ÿ�λ���²�
		ADC_PPM_Databuf[3]=2000;//ƫ����λ���Ҳ�
		
		ADC_PPM_Databuf[4]=2000;//����ͨ����λ�����ڶ���ģʽ
		ADC_PPM_Databuf[5]=1000;//����ͨ����λ
		ADC_PPM_Databuf[6]=1000;//����ͨ����λ
		ADC_PPM_Databuf[7]=1000;//�ڰ�ͨ����λ
		Bling_Set(&rgb_green,5000,1000,0.5,0,GPIO_PORTF_BASE,GPIO_PIN_3,1);
	}
	else if(ADC_Button1.press==LONG_PRESS)//������������
	{
		//����Ϊ�ð���ʵ�ֵ�����
		adc_ppm_update_flag=1;//����Ȩ�޸�ADC�������õ�PPM
		if(Controler_State==Unlock_Controler)//������ڽ���״̬��ң�������Ż���׼��ִ��SDK
		{
			ADC_PPM_Databuf[0]=1500;//������λ����λ
			ADC_PPM_Databuf[1]=1500;//�����λ����λ
			ADC_PPM_Databuf[2]=1000;//���Ÿ�λ���²�
			ADC_PPM_Databuf[3]=1500;//ƫ����λ����λ
			
			ADC_PPM_Databuf[4]=2000;//����ͨ����λ�����ڶ���ģʽ
			ADC_PPM_Databuf[5]=1000;//����ͨ����λ
			ADC_PPM_Databuf[6]=1000;//����ͨ����λ
			ADC_PPM_Databuf[7]=1000;//�ڰ�ͨ����λ
			Bling_Set(&rgb_green,5000,1000,0.5,0,GPIO_PORTF_BASE,GPIO_PIN_3,1);
		}
	}
	else if(ADC_Button1.press==SHORT_PRESS)//���������̰�
	{
		//����Ϊ�ð���ʵ�ֵ�����
		adc_ppm_update_flag=0;//����Ȩ�޸��ͷŸ�����������ȡ��PPM��PPM���ջ�������վ����ң����
													//�ʰ��������󣬴�ʱ�̰��°�����ң��������Ȩ��˲����ͷŸ����ջ�
													//ң�����ӹܺ󣬿����ֶ����Ʒɻ���ȫ����
		
		Bling_Set(&rgb_green,5000,1000,0.5,0,GPIO_PORTF_BASE,GPIO_PIN_3,0);
	}
	
	
	//����Ϊ���������¼������߼�
	if(ADC_Button2.press==IN_PRESS)
	{
		//����Ϊ�ð���ʵ�ֵ�����
	}
	else if(ADC_Button2.press==LONG_PRESS)//������������
	{
		//����Ϊ�ð���ʵ�ֵ�����

	}
	else if(ADC_Button2.press==SHORT_PRESS)//���������̰�
	{
		//����Ϊ�ð���ʵ�ֵ�����
		adc_ppm_update_flag=0;//����Ȩ�޸��ͷŸ�����������ȡ��PPM��PPM���ջ�������վ����ң����
													//�ʰ��������󣬴�ʱ�̰��°�����ң��������Ȩ��˲����ͷŸ����ջ�
													//ң�����ӹܺ󣬿����ֶ����Ʒɻ���ȫ����
		
		if(Controler_State==Unlock_Controler)//������ڽ���״̬
		{
			Controler_State=Lock_Controler;//ֱ������
			Bling_Set(&rgb_green,5000,1000,0.5,0,GPIO_PORTF_BASE,GPIO_PIN_3,0);
		}	
	}
	
	
	
	
	//����ΪSDK�����¼������߼�
	if(ADC_Button3.press==IN_PRESS)
	{
		//����Ϊ�ð���ʵ�ֵ�����
	}
	else if(ADC_Button3.press==LONG_PRESS)//SDK��������
	{
		//����Ϊ�ð���ʵ�ֵ�����
		adc_ppm_update_flag=1;//����Ȩ�޸�ADC�������õ�PPM
		if(Controler_State==Unlock_Controler)//������ڽ���״̬��ִ��SDK
		{		
			ADC_PPM_Databuf[0]=1500;//������λ����λ
			ADC_PPM_Databuf[1]=1500;//�����λ����λ
			ADC_PPM_Databuf[2]=1500;//���Ÿ�λ���в�
			ADC_PPM_Databuf[3]=1500;//ƫ����λ����λ

			ADC_PPM_Databuf[4]=2000;//����ͨ����λ�����ڶ���ģʽ
			ADC_PPM_Databuf[5]=2000;//����ͨ����λ������SDKģʽ
			ADC_PPM_Databuf[6]=1000;//����ͨ����λ
			ADC_PPM_Databuf[7]=2000;//�ڰ�ͨ����λ�����ڹ�������ģʽ
			
			Bling_Set(&rgb_green,5000,1000,0.5,0,GPIO_PORTF_BASE,GPIO_PIN_3,1);
		}
	}
	else if(ADC_Button3.press==SHORT_PRESS)//SDK�����̰�
	{
		//����Ϊ�ð���ʵ�ֵ�����
		adc_ppm_update_flag=1;//����Ȩ�޸�ADC�������õ�PPM
		if(Controler_State==Unlock_Controler)//������ڽ���״̬���˳�SDKģʽ����ԭ����ͣ
		{
			ADC_PPM_Databuf[0]=1500;//������λ����λ
			ADC_PPM_Databuf[1]=1500;//�����λ����λ
			ADC_PPM_Databuf[2]=1500;//���Ÿ�λ���в�
			ADC_PPM_Databuf[3]=1500;//ƫ����λ����λ

			ADC_PPM_Databuf[4]=2000;//����ͨ����λ�����ڶ���ģʽ
			ADC_PPM_Databuf[5]=1000;//����ͨ����λ���˳�SDKģʽ
			ADC_PPM_Databuf[6]=1000;//����ͨ����λ
			ADC_PPM_Databuf[7]=2000;//�ڰ�ͨ����λ�����ڹ�������ģʽ
			Bling_Set(&rgb_green,5000,1000,0.5,0,GPIO_PORTF_BASE,GPIO_PIN_3,1);
		}
	}
	
	
	
	//����Ϊһ�������¼������߼�
	if(ADC_Button4.press==IN_PRESS)
	{
		//����Ϊ�ð���ʵ�ֵ�����
	}
	else if(ADC_Button4.press==LONG_PRESS)//һ�����䰴������
	{
		//����Ϊ�ð���ʵ�ֵ�����

	}
	else if(ADC_Button4.press==SHORT_PRESS)//һ�����䰴���̰�
	{
		//����Ϊ�ð���ʵ�ֵ�����
		adc_ppm_update_flag=1;//����Ȩ�޸�ADC�������õ�PPM
		if(Controler_State==Unlock_Controler)//������ڽ���״̬���˳�SDKģʽ����ԭ����ͣ
		{
			ADC_PPM_Databuf[0]=1500;
			ADC_PPM_Databuf[1]=1500;
			ADC_PPM_Databuf[2]=1500;
			ADC_PPM_Databuf[3]=1500;//ǰ�ĸ�ͨ��������λ
			
			ADC_PPM_Databuf[4]=2000;//����ͨ����λ�����ڶ���ģʽ
			ADC_PPM_Databuf[5]=1000;//����ͨ����λ
			ADC_PPM_Databuf[6]=2000;//����ͨ����λ������һ������ģʽ
			ADC_PPM_Databuf[7]=2000;//�ڰ�ͨ����λ�����ڹ�������ģʽ
			Bling_Set(&rgb_green,5000,1000,0.5,0,GPIO_PORTF_BASE,GPIO_PIN_3,1);
		}
	}

}




