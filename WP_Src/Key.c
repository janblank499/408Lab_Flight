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
#include "Key.h"

rc_buttton _button;


/***********************************************************
@��������Key_Init
@��ڲ�������
@���ڲ�������
����������������ʼ��
@���ߣ�����С��
@���ڣ�2024/01/20
*************************************************************/
void Key_Init(void)
{
  SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
  //SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);
  // Unlock PF0 so we can change it to a GPIO input
  // Once we have enabled (unlocked) the commit register then re-lock it
  // to prevent further changes.  PF0 is muxed with NMI thus a special case.
  HWREG(GPIO_PORTF_BASE + GPIO_O_LOCK) = GPIO_LOCK_KEY;
  HWREG(GPIO_PORTF_BASE + GPIO_O_CR) |= 0x01;
  HWREG(GPIO_PORTF_BASE + GPIO_O_LOCK) = 0;
  GPIODirModeSet(GPIO_PORTF_BASE, GPIO_PIN_0,GPIO_DIR_MODE_IN);//SW1
  GPIOPadConfigSet(GPIO_PORTF_BASE,GPIO_PIN_0,GPIO_STRENGTH_2MA,GPIO_PIN_TYPE_STD_WPU);
  GPIODirModeSet(GPIO_PORTF_BASE, GPIO_PIN_4,GPIO_DIR_MODE_IN);//SW2
  GPIOPadConfigSet(GPIO_PORTF_BASE,GPIO_PIN_4,GPIO_STRENGTH_2MA,GPIO_PIN_TYPE_STD_WPU);

	_button.state[UP].port=GPIO_PORTF_BASE;
	_button.state[UP].pin=GPIO_PIN_0;
	_button.state[UP].value=1;
	_button.state[UP].last_value=1;

	_button.state[DOWN].port=GPIO_PORTF_BASE;
	_button.state[DOWN].pin=GPIO_PIN_4;
	_button.state[DOWN].value=1;
	_button.state[DOWN].last_value=1;	
	
	/******************************************************************************************/
	_button.state[UP_3D].value=1;
	_button.state[DN_3D].value=1;
	_button.state[LT_3D].value=1;
	_button.state[RT_3D].value=1;
	_button.state[ME_3D].value=1;
	
	_button.state[UP_3D].last_value=1;
	_button.state[DN_3D].last_value=1;
	_button.state[LT_3D].last_value=1;
	_button.state[RT_3D].last_value=1;
	_button.state[ME_3D].last_value=1;
	
	ADC_Button_Init();
}





void Read_Button_State_One(button_state *button)
{
  button->value=GPIOPinRead(button->port,button->pin);
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
			if(millis()-button->in_time>IN_PRESS_LIMIT)//��������
			{
				button->in_time=millis();
				button->press=IN_PRESS;
				//rc_beep.time=1;
				if(button->press==IN_PRESS)  button->in_press_cnt++;
			}
		}
	}
  else
	{
		if(button->last_value==0)//���º��ͷ�
		{
			button->release_time=millis();//��¼�ͷ�ʱ��ʱ��
			
			if(button->release_time-button->press_time>LONG_PRESS_LIMIT)
			{
			   button->press=LONG_PRESS;
				 button->state_lock_time=0;//5ms*300=1.5S
				 buzzer_setup(1000,0.5,1);
			}
			else
			{
			   button->press=SHORT_PRESS;
				 button->state_lock_time=0;//5ms*300=1.5S
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


void Read_Button_State_All(void)
{
  for(uint8_t i=0;i<2;i++)
	{
	  Read_Button_State_One(&_button.state[i]);
	}
}





void key_delay(int32_t period)
{
  while(period--)
  {
    delay_us(1);  
  }
}

#define LONG_PRESS_MAX  5000
#define Page_Number_Max 33-1
int16_t Page_Number=0;
uint8_t Key_Right_Release=0;
/***********************************************************
@��������Key_Scan
@��ڲ�����release
@���ڲ�����bool
��������������ɨ�裬��ڲ���release�����Ƿ񿪷Ű���ɨ��Ȩ��
����ɨ�践��TRUE����������ʱΪ�͵�ƽ���ͷź�IO���õ�������
����ģʽ������Ϊ�ߵ�ƽ
@���ߣ�����С��
@���ڣ�2024/01/20
*************************************************************/
bool Key_Scan(uint8_t release)
{
	uint16_t long_press_cnt=0;
  if(release==1)  return false;
	if(_button.state[UP].press==SHORT_PRESS)
	{
		Page_Number--;
		if(Page_Number<0) Page_Number=Page_Number_Max;
		_button.state[UP].press=NO_PRESS;
		LCD_CLS();
		buzzer_setup(100,0.5,1);
	}

	if(_button.state[DOWN].press==SHORT_PRESS)
	{
		Page_Number++;
		if(Page_Number>Page_Number_Max) Page_Number=0;
		_button.state[DOWN].press=NO_PRESS;
		LCD_CLS();
		buzzer_setup(100,0.5,1);
	}
	

		if(Controler_State==Lock_Controler
		  &&Throttle_Control<=(1000+Buttom_Safe_Deadband)
		  &&Yaw_Control>=Yaw_Max*Scale_Pecent_Max
			&&Roll_Control<-Pit_Rol_Max*Scale_Pecent_Max&&Pitch_Control==0)
		{
			while(Controler_State==Lock_Controler
					&&Throttle_Control<=(1000+Buttom_Safe_Deadband)
					&&Yaw_Control>=Yaw_Max*Scale_Pecent_Max
					&&Roll_Control<-Pit_Rol_Max*Scale_Pecent_Max&&Pitch_Control==0)
			{
				long_press_cnt++;
				key_delay(10);
				if(long_press_cnt>=LONG_PRESS_MAX)
				{
					Page_Number++;
					if(Page_Number>Page_Number_Max) Page_Number=0;
					LCD_CLS();
					buzzer_setup(100,0.5,1);
					return true;
				}
			}
		}

		
		if(Controler_State==Lock_Controler
		  &&Throttle_Control<=(1000+Buttom_Safe_Deadband)
			&&Yaw_Control>=Yaw_Max*Scale_Pecent_Max
		  &&Roll_Control>Pit_Rol_Max*Scale_Pecent_Max&&Pitch_Control==0)
		{
			while(Controler_State==Lock_Controler
					&&Throttle_Control<=(1000+Buttom_Safe_Deadband)
					&&Yaw_Control>=Yaw_Max*Scale_Pecent_Max
					&&Roll_Control>Pit_Rol_Max*Scale_Pecent_Max&&Pitch_Control==0) 
			{
				long_press_cnt++;
				key_delay(10);
				if(long_press_cnt>=LONG_PRESS_MAX)
				{
					Page_Number--;
					if(Page_Number<0) Page_Number=Page_Number_Max;
					LCD_CLS();
					buzzer_setup(100,0.5,1);
					return true;
				}
			}
		}	
  return true;
}


/***********************************************************
@��������QuadShow
@��ڲ�������
@���ڲ�������
������������ʾ��ˢ�£�ÿһҳÿһ�����ݽ��ʹ��˵�������ۿ���
��������ҳ���ϰ���ɨ����ʵ��
@���ߣ�����С��
@���ڣ�2024/01/20
*************************************************************/
void QuadShow(uint8_t show_enable)
{
  uint16_t i=0;
	switch(Page_Number)
	{
		case 0:
		{
				LCD_clear_L(0,0);  display_6_8_string(0,0,"Basic");           display_6_8_number(40,0,Time0_Delta.Time_Delta);  								  display_6_8_number(75,0,Battery_Voltage);display_6_8_number(115,0,Page_Number+1);
				LCD_clear_L(0,1);  display_6_8_string(0,1,"Y:");            
				display_6_8_number(15,1,WP_AHRS.Yaw); 
				if(WP_AHRS.Yaw_Gyro<0)	display_6_8_string(55,1,"-");
				else display_6_8_string(55,1,"+");
				display_6_8_number(63,1,ABS(WP_AHRS.Yaw_Gyro));		
				display_6_8_number(105,1,IMU_ID);


				LCD_clear_L(0,2);  display_6_8_string(0,2,"P:");          
				display_6_8_number(15,2,WP_AHRS.Pitch);                             
				if(WP_AHRS.Pitch_Gyro<0) display_6_8_string(55,2,"-");
				else display_6_8_string(55,2,"+");
				display_6_8_number(63,2,ABS(WP_AHRS.Pitch_Gyro));
				switch(IMU_ID)
				{
					case WHO_AM_I_MPU6050:display_6_8_string(105,2,"mpu"); break;
					case WHO_AM_I_ICM20689:display_6_8_string(105,2,"689"); break;
					case WHO_AM_I_ICM20608D:display_6_8_string(105,2,"08D"); break;
					case WHO_AM_I_ICM20608G:display_6_8_string(105,2,"08G"); break;
					case WHO_AM_I_ICM20602:display_6_8_string(105,2,"602"); break;
					default:display_6_8_string(105,2,"unk");		
				}
			
				LCD_clear_L(0,3);  display_6_8_string(0,3,"R:");           
				display_6_8_number(15,3,WP_AHRS.Roll);                            	
				if(WP_AHRS.Roll_Gyro<0)	display_6_8_string(55,3,"-");
				else display_6_8_string(55,3,"+");
				display_6_8_number(63,3,ABS(WP_AHRS.Roll_Gyro));
				
				LCD_clear_L(0,4);  display_6_8_string(0,4,"Baro:");           display_6_8_number(40,4,(uint32_t)(WP_Sensor.baro_pressure_raw));   display_6_8_number(90,4,WP_Sensor.baro_altitude);
				LCD_clear_L(0,5);  display_6_8_string(0,5,"RF:");             display_6_8_number(40,5,GD_Distance);               			          display_6_8_number(80,5,NamelessQuad.Position[_UP]);
				LCD_clear_L(0,6);  display_6_8_string(0,6,"AD:");             write_6_8_number_f1(26,6,Button_ADCResult[0]);
																																			write_6_8_number_f1(55,6,Button_ADCResult[1]);
																																			display_6_8_number(90,6,NamelessQuad.Speed[_UP]);
				LCD_clear_L(0,7);  display_6_8_number(0,7,SDK1_Mode_Setup);
													 display_6_8_number(15,7,Temperature_Stable_Flag);  
													 display_6_8_number(30,7,Gyro_Safety_Calibration_Flag);
													 display_6_8_number(45,7,WP_Sensor._temperature);         
													 display_6_8_number(85,7,NamelessQuad.Inertial_Acceleration[_UP]);
		}
		break;
		case 1:
		{
				LCD_clear_L(0,0);  LCD_P6x8Str(0,0,"Accel:");write_6_8_number(115,0,Page_Number+1);
				if(Accel_Simple_Mode==0)  LCD_P6x8Str(50,0,"6-Side");
				else LCD_P6x8Str(50,0,"Simple");
				
				if(Pitch_Control>=Scale_Pecent_Max*Pit_Rol_Max)
				{
					delay_ms(10);
					if(Pitch_Control>=Scale_Pecent_Max*Pit_Rol_Max)
					{
						while(Pitch_Control>=Scale_Pecent_Max*Pit_Rol_Max);
						if(Accel_Simple_Mode==0) Accel_Simple_Mode=1;
						else if(Accel_Simple_Mode==1) Accel_Simple_Mode=0;
						WriteFlashParameter(ACCEL_SIMPLE_MODE,Accel_Simple_Mode);
						Bling_Set(&rgb_blue,500,50,0.2,0,GPIO_PORTF_BASE,GPIO_PIN_2,0);//��ɫ
					}
				}
				
				if(_button.state[DOWN].press==LONG_PRESS)
				{
						_button.state[DOWN].press=NO_PRESS;
						if(Accel_Simple_Mode==0) Accel_Simple_Mode=1;
						else if(Accel_Simple_Mode==1) Accel_Simple_Mode=0;
						WriteFlashParameter(ACCEL_SIMPLE_MODE,Accel_Simple_Mode);
						Bling_Set(&rgb_blue,500,50,0.2,0,GPIO_PORTF_BASE,GPIO_PIN_2,0);//��ɫ				
				}		
				
				//������һҳУ׼����ˮƽ
				if(_button.state[UP].press==LONG_PRESS)
				{
						_button.state[UP].press=NO_PRESS;
						cal_flag=0x04;
						Bling_Set(&rgb_blue,500,50,0.2,0,GPIO_PORTF_BASE,GPIO_PIN_2,0);//��ɫ				
				}	

				LCD_clear_L(0,1);  LCD_P6x8Str(0,1,"X:");              write_6_8_number(12,1,K[0]);   write_6_8_number(55,1,B[0]);   write_6_8_number(95,1,Accel_Hor_Read.x);
				LCD_clear_L(0,2);  LCD_P6x8Str(0,2,"Y:");              write_6_8_number(12,2,K[1]);   write_6_8_number(55,2,B[1]);   write_6_8_number(95,2,Accel_Hor_Read.y);
				LCD_clear_L(0,3);  LCD_P6x8Str(0,3,"Z:");              write_6_8_number(12,3,K[2]);   write_6_8_number(55,3,B[2]);   write_6_8_number(95,3,Accel_Hor_Read.z);
				LCD_clear_L(0,4);  LCD_P6x8Str(0,4,"Mag/T:");    		   write_6_8_number(40,4,WP_Sensor.mag_intensity);
																															 LCD_P6x8Str(75,4,"GY/PR_Off:");
				LCD_clear_L(0,5);  LCD_P6x8Str(0,5,"X:");         		 write_6_8_number(12,5,Mag_Offset_Read.x); write_6_8_number(55,5,gyro_offset.x);  write_6_8_number(105,5,Pitch_Offset);
				LCD_clear_L(0,6);  LCD_P6x8Str(0,6,"Y:");         		 write_6_8_number(12,6,Mag_Offset_Read.y); write_6_8_number(55,6,gyro_offset.y);  write_6_8_number(105,6,Roll_Offset); 
				LCD_clear_L(0,7);  LCD_P6x8Str(0,7,"Z:");         		 write_6_8_number(12,7,Mag_Offset_Read.z); write_6_8_number(55,7,gyro_offset.z);                                   	
		}
		break;
		case 2:
		{
				LCD_clear_L(0,0);  display_6_8_string(0,0,"RC Data");         display_6_8_number(115,0,Page_Number+1);
				LCD_clear_L(0,1);  display_6_8_string(0,1,"Yaw:");            display_6_8_number(40,1,Yaw_Control);                     display_6_8_number(90,1,PPM_Databuf[3]);
				LCD_clear_L(0,2);  display_6_8_string(0,2,"Pitch:");          display_6_8_number(40,2,Pitch_Control);                   display_6_8_number(90,2,PPM_Databuf[1]);
				LCD_clear_L(0,3);  display_6_8_string(0,3,"Roll:");           display_6_8_number(40,3,Roll_Control);                    display_6_8_number(90,3,PPM_Databuf[0]);
				LCD_clear_L(0,4);  display_6_8_string(0,4,"Thr:");            display_6_8_number(40,4,Throttle_Control);                display_6_8_number(90,4,PPM_Databuf[2]);
				LCD_clear_L(0,5);  display_6_8_string(0,5,"ch5-6:");          display_6_8_number(40,5,PPM_Databuf[4]);                  display_6_8_number(90,5,PPM_Databuf[5]);
				LCD_clear_L(0,6);  display_6_8_string(0,6,"ch7-8:");          display_6_8_number(40,6,PPM_Databuf[6]);                  display_6_8_number(90,6,PPM_Databuf[7]);
				
				LCD_clear_L(0,7);  display_6_8_string(0,7,"mode");             
				if(RC_Data.thr_mode==HAND_THR)        display_6_8_string(30,7,"Manual_Ctrl_Alt");
				else if(RC_Data.thr_mode==AUTO_THR&&RC_Data.flight_mode==STABILIZE)   display_6_8_string(30,7,"Auto_Ctrl_Alt");
				else if(RC_Data.thr_mode==AUTO_THR&&RC_Data.flight_mode==POS_GUIDE)   display_6_8_string(30,7,"Auto_Pos_Guide");
				else if(RC_Data.thr_mode==AUTO_THR&&RC_Data.flight_mode==POS_HOLD)    display_6_8_string(30,7,"Auto_Ctrl_Hover");
				else display_6_8_string(30,7,"Manual_Ctrl_Alt"); 		
		}
		break;
		case 3:
		{
				LCD_clear_L(0,0);  display_6_8_string(0,0,"Ctrl Data P  I");  display_6_8_number(115,0,Page_Number+1);
				LCD_clear_L(0,1);  display_6_8_string(0,1,"Pit_Ang:");        display_6_8_number(50,1,Total_Controller.Pitch_Angle_Control.Kp);           display_6_8_number(90,1,Total_Controller.Pitch_Angle_Control.Ki); 
				LCD_clear_L(0,2);  display_6_8_string(0,2,"Rol_Ang:");        display_6_8_number(50,2,Total_Controller.Roll_Angle_Control.Kp);            display_6_8_number(90,2,Total_Controller.Roll_Angle_Control.Ki);    
				LCD_clear_L(0,3);  display_6_8_string(0,3,"Yaw_Ang:");        display_6_8_number(50,3,Total_Controller.Yaw_Angle_Control.Kp);             display_6_8_number(90,3,Total_Controller.Yaw_Angle_Control.Ki);    
				LCD_clear_L(0,4);  display_6_8_string(0,4,"Pit_Gyr:");        display_6_8_number(50,4,Total_Controller.Pitch_Gyro_Control.Kp);            display_6_8_number(90,4,Total_Controller.Pitch_Gyro_Control.Ki);    
				LCD_clear_L(0,5);  display_6_8_string(0,5,"Rol_Gyr:");        display_6_8_number(50,5,Total_Controller.Roll_Gyro_Control.Kp);             display_6_8_number(90,5,Total_Controller.Roll_Gyro_Control.Ki);    
				LCD_clear_L(0,6);  display_6_8_string(0,6,"Yaw_Gyr:");        display_6_8_number(50,6,Total_Controller.Yaw_Gyro_Control.Kp);              display_6_8_number(90,6,Total_Controller.Yaw_Gyro_Control.Ki);
				LCD_clear_L(0,7);  display_6_8_string(0,7,"Gyro_D:");         display_6_8_number(40,7,Total_Controller.Pitch_Gyro_Control.Kd);            display_6_8_number(70,7,Total_Controller.Roll_Gyro_Control.Kd);                  write_6_8_number(95,7,Total_Controller.Yaw_Gyro_Control.Kd);		
		}
		break;
		case 4:
		{
				if(Pitch_Control>=Scale_Pecent_Max*Pit_Rol_Max)//���������Ƹ˺����ʵ�ֶ�SDK1_Mode_Setup�ĵ���
				{
					delay_ms(10);
					if(Pitch_Control>=Scale_Pecent_Max*Pit_Rol_Max)
					{
						while(Pitch_Control>=Scale_Pecent_Max*Pit_Rol_Max);
						SDK1_Mode_Setup++;
						if(SDK1_Mode_Setup>SDK_MODE_VALID_MAX) SDK1_Mode_Setup=0;
						SDK_DT_Send_Check(SDK1_Mode_Setup,UART3_SDK);
						WriteFlashParameter(SDK1_MODE_DEFAULT,SDK1_Mode_Setup);
						Bling_Set(&rgb_blue,500,50,0.2,0,GPIO_PORTF_BASE,GPIO_PIN_2,0);//��ɫ
					}
				}
				
				if(_button.state[DOWN].press==LONG_PRESS)//��һҳ��������,ʵ�ֶ�SDK1_Mode_Setup�ĵ���
				{
						_button.state[DOWN].press=NO_PRESS;
						SDK1_Mode_Setup++;
						if(SDK1_Mode_Setup>SDK_MODE_VALID_MAX) SDK1_Mode_Setup=0;
						SDK_DT_Send_Check(SDK1_Mode_Setup,UART3_SDK);
						WriteFlashParameter(SDK1_MODE_DEFAULT,SDK1_Mode_Setup);	
						Bling_Set(&rgb_blue,500,50,0.2,0,GPIO_PORTF_BASE,GPIO_PIN_2,0);//��ɫ				
				}		
				/***********************************************************************************************************/
				if(Pitch_Control<=-Scale_Pecent_Max*Pit_Rol_Max)//���������Ƹ˺����ʵ�ֶ�task_select_cnt�ĵ���
				{
					delay_ms(10);
					if(Pitch_Control<=-Scale_Pecent_Max*Pit_Rol_Max)
					{
						while(Pitch_Control<=-Scale_Pecent_Max*Pit_Rol_Max);
						task_select_cnt++;
						if(task_select_cnt>6)	task_select_cnt=1;
						WriteFlashParameter(TASK_SELECT_AFTER_TAKEOFF,task_select_cnt);//����ѡ�������
						Bling_Set(&rgb_blue,500,50,0.2,0,GPIO_PORTF_BASE,GPIO_PIN_2,0);//��ɫ
					}
				}
				
				if(_button.state[UP].press==LONG_PRESS)//��һҳ��������,ʵ�ֶ�task_select_cnt�ĵ���
				{
						_button.state[UP].press=NO_PRESS;
						task_select_cnt++;
						if(task_select_cnt>6)	task_select_cnt=1;
						WriteFlashParameter(TASK_SELECT_AFTER_TAKEOFF,task_select_cnt);//����ѡ�������
						Bling_Set(&rgb_blue,500,50,0.2,0,GPIO_PORTF_BASE,GPIO_PIN_2,0);//��ɫ		
				}		
				
				LCD_clear_L(0,0);  LCD_P6x8Str(0,0,"Top_View_Openmv"); write_6_8_number(115,0,Page_Number+1);
				LCD_clear_L(0,1);  LCD_P6x8Str(0,1,"UART3_SDK_Mode:"); write_6_8_number(100,1,SDK1_Mode_Setup);
				LCD_clear_L(0,2); 			
				if(SDK1_Mode_Setup==0)     		  LCD_P6x8Str(0,2,"ncq_sdk_run");		        //Ĭ��ģʽ�������+ǰ��+����
				else if(SDK1_Mode_Setup==1)     LCD_P6x8Str(0,2,"color_block");		        //ɫ��׷��ģʽ
				else if(SDK1_Mode_Setup==2)     LCD_P6x8Str(0,2,"top apriltag");		      //AprilTag׷��ģʽ
				else if(SDK1_Mode_Setup==3)     LCD_P6x8Str(0,2,"track control");         //����ѭ��
				else if(SDK1_Mode_Setup==4)     LCD_P6x8Str(0,2,"front aprilTag");        //ǰ��׷��AprilTag
				else if(SDK1_Mode_Setup==5)     LCD_P6x8Str(0,2,"front surround pole");   //�������˷���
				else if(SDK1_Mode_Setup==6)     LCD_P6x8Str(0,2,"clockwise 90deg");       //˳ʱ��ת��90deg
				else if(SDK1_Mode_Setup==7)     LCD_P6x8Str(0,2,"clockwise gyro");        //��10deg/s�Ľ��ٶ�˳ʱ��ת��10000ms����ɺ���
				else if(SDK1_Mode_Setup==8)     LCD_P6x8Str(0,2,"rela motion track"); 		//��������ϵ�����λ��,�����ι켣
			  else if(SDK1_Mode_Setup==9)     LCD_P6x8Str(0,2,"abso motion track"); 		//��������ϵ�£����ڳ�ʼ��ľ�������λ��,�����ι켣
			  else if(SDK1_Mode_Setup==10)    LCD_P6x8Str(0,2,"circular track");        //������й켣Բ���뾶����������������
			  else if(SDK1_Mode_Setup==11)    LCD_P6x8Str(0,2,"auto takeoff 150");      //�Զ���ɵ�ĳһ�߶Ⱥ�ִ����������
			  else if(SDK1_Mode_Setup==12)    LCD_P6x8Str(0,2,"agriculture uav");       //ֲ�����˻���������
			  else if(SDK1_Mode_Setup==13)    LCD_P6x8Str(0,2,"ros control uav ");      //ROS��λ���������˻�
			  else if(SDK1_Mode_Setup==14)    LCD_P6x8Str(0,2,"ngs control uav ");      //����վ�������˻�
			  else if(SDK1_Mode_Setup==15)    LCD_P6x8Str(0,2,"delivery uav 1");      	//�ͻ����˻���������
				else if(SDK1_Mode_Setup==16)    LCD_P6x8Str(0,2,"delivery uav 2");      	//�ͻ����˻���������
				else if(SDK1_Mode_Setup==17)    LCD_P6x8Str(0,2,"delivery uav 3");      	//�ͻ����˻���������
				else if(SDK1_Mode_Setup==18)    LCD_P6x8Str(0,2,"auto takeoff 150");      //�Զ���ɵ�ĳһ�߶Ⱥ�ִ����������
				else if(SDK1_Mode_Setup==19)    LCD_P6x8Str(0,2,"user setpoint");         //�û��Զ��庽�����-������α�̣��Ϳ���ʵ��3ά�ռ��ڵ����ɺ����������
				else if(SDK1_Mode_Setup==20)    LCD_P6x8Str(0,2,"auto takeoff 180");      //�Զ���ɵ�ĳһ�߶Ⱥ�ִ����������
				else if(SDK1_Mode_Setup==21)    LCD_P6x8Str(0,2,"firefighting uav1");     //�յ�Эͬ��������ϵͳ������������
				else if(SDK1_Mode_Setup==22)    LCD_P6x8Str(0,2,"firefighting uav2");     //�յ�Эͬ��������ϵͳ�������Ӳ���
				else if(SDK1_Mode_Setup==23)    LCD_P6x8Str(0,2,"firefighting car");      //�յ�Эͬ��������ϵͳ������������С������
				else 
				{
					LCD_P6x8Str(0,2,"User SDK:");
					write_6_8_number(70,2,SDK1_Mode_Setup-23);
				}
				LCD_clear_L(0,3);LCD_P6x8Str(0,3,"Now_SDK1_Mode:");write_6_8_number(100,3,Opv_Top_View_Target.sdk_mode-0xA0);
				
				
				LCD_clear_L(0,5);LCD_P6x8Str(0,5,"task_select");	 write_6_8_number(100,5,task_select_cnt);
				int16_t tmp_sdk_duty=SDK1_Mode_Setup+task_select_cnt; 
				LCD_clear_L(0,6);LCD_P6x8Str(0,6,"work_duty_cnt"); write_6_8_number(100,6,tmp_sdk_duty);
				LCD_clear_L(0,7); 			
				if(tmp_sdk_duty==21)         LCD_P6x8Str(0,7,"firefighting uav1");     //�յ�Эͬ��������ϵͳ������������
				else if(tmp_sdk_duty==22)    LCD_P6x8Str(0,7,"firefighting uav2");     //�յ�Эͬ��������ϵͳ�������Ӳ���
				else 
				{
					LCD_P6x8Str(0,7,"invalid sdk:");
					write_6_8_number(80,7,tmp_sdk_duty-23);
				}
		}
		break;
		case 5:
		{
				LCD_clear_L(0,0);  LCD_P6x8Str(0,0,"Front_View_Openmv"); write_6_8_number(115,0,Page_Number+1);
				LCD_clear_L(0,1);  LCD_P6x8Str(0,1,"UART1_SDK_Mode:");   write_6_8_number(100,1,Forward_Vision_Mode_Setup);
				
				LCD_clear_L(0,6);LCD_P6x8Str(0,6,"Now_SDK2_Mode:");write_6_8_number(100,6,Opv_Front_View_Target.sdk_mode-0xA0);		

				
				if(Pitch_Control>=Scale_Pecent_Max*Pit_Rol_Max)
				{
					delay_ms(10);
					if(Pitch_Control>=Scale_Pecent_Max*Pit_Rol_Max)
					{
						while(Pitch_Control>=Scale_Pecent_Max*Pit_Rol_Max);
						Forward_Vision_Mode_Setup++;
						if(Forward_Vision_Mode_Setup>SDK_MODE_VALID_MAX) Forward_Vision_Mode_Setup=0;
						SDK_DT_Send_Check(Forward_Vision_Mode_Setup,UART0_SDK);
						WriteFlashParameter(FORWARD_VISION_MODE_DEFAULT,Forward_Vision_Mode_Setup);
						Bling_Set(&rgb_blue,500,50,0.2,0,GPIO_PORTF_BASE,GPIO_PIN_2,0);//��ɫ
					}
				}
				
				if(_button.state[DOWN].press==LONG_PRESS)
				{
						_button.state[DOWN].press=NO_PRESS;
						Forward_Vision_Mode_Setup++;
						if(Forward_Vision_Mode_Setup>SDK_MODE_VALID_MAX) Forward_Vision_Mode_Setup=0;
						SDK_DT_Send_Check(Forward_Vision_Mode_Setup,UART0_SDK);
						WriteFlashParameter(FORWARD_VISION_MODE_DEFAULT,Forward_Vision_Mode_Setup);
						Bling_Set(&rgb_blue,500,50,0.2,0,GPIO_PORTF_BASE,GPIO_PIN_2,0);//��ɫ				
				}			
		}
		break;
		case 6:
		{
				ssd1306_clear_display();
				ssd1306_draw_line(0,32,128,32,WHITE);
				ssd1306_draw_line(64,0,64,64,WHITE);			
				uint16_t x,y;
				y=32-32*constrain_float(0.5*Opv_Top_View_Target.height-Opv_Top_View_Target.y,-50,50)/50.0f;
				x=64-32*constrain_float(0.5*Opv_Top_View_Target.width-Opv_Top_View_Target.x,-50,50)/50.0f;
				if(Opv_Top_View_Target.flag==1)			ssd1306_fill_circle(x,y,5,WHITE);
				else ssd1306_draw_circle(x,y,5,WHITE);			
				
				ssd1306_display();
				display_6_8_string(0,0,"point_x:");  write_6_8_number(80,0,Opv_Top_View_Target.x);		write_6_8_number(105,0,Page_Number+1);
				display_6_8_string(0,1,"point_y::"); write_6_8_number(80,1,Opv_Top_View_Target.y);		write_6_8_number(105,1,Opv_Top_View_Target.sdk_mode-0xA0);
				display_6_8_string(0,2,"point_s:");  write_6_8_number(80,2,Opv_Top_View_Target.pixel);
				display_6_8_string(0,3,"point_f:");  write_6_8_number(80,3,Opv_Top_View_Target.flag);
				display_6_8_string(0,4,"Tar_x:");    write_6_8_number(80,4,Opv_Top_View_Target.sdk_target.x);
				display_6_8_string(0,5,"Tar_y:");    write_6_8_number(80,5,Opv_Top_View_Target.sdk_target.y); 
				display_6_8_string(0,6,"FPS:");      write_6_8_number(80,6,Opv_Top_View_Target.fps);
				display_6_8_string(0,7,"Dis:");      write_6_8_number(30,7,Opv_Top_View_Target.sdk_target.x);
																						 write_6_8_number(80,7,Opv_Top_View_Target.sdk_target.y);		
		}
		break;
		case 7:
		{
				ssd1306_clear_display();
				ssd1306_draw_line(0,32,128,32,WHITE);
				ssd1306_draw_line(64,0,64,64,WHITE);
				
				uint16_t x,y;
				y=32-32*constrain_float(0.5*Opv_Front_View_Target.height-Opv_Front_View_Target.y,-50,50)/50.0f;
				x=64-32*constrain_float(0.5*Opv_Front_View_Target.width-Opv_Front_View_Target.x,-50,50)/50.0f;
				if(Opv_Front_View_Target.flag==1)			ssd1306_fill_circle(x,y,5,WHITE);
				else ssd1306_draw_circle(x,y,5,WHITE);			
				
				ssd1306_display();
				display_6_8_string(0,0,"point_x:");  write_6_8_number(80,0,Opv_Front_View_Target.x);	write_6_8_number(105,0,Page_Number+1);
				display_6_8_string(0,1,"point_y::"); write_6_8_number(80,1,Opv_Front_View_Target.y);  write_6_8_number(105,1,Opv_Front_View_Target.sdk_mode-0xA0);
				display_6_8_string(0,2,"point_s:");  write_6_8_number(80,2,Opv_Front_View_Target.pixel);
				display_6_8_string(0,3,"point_f:");  write_6_8_number(80,3,Opv_Front_View_Target.flag);
				display_6_8_string(0,4,"Tar_x:");    write_6_8_number(80,4,Opv_Front_View_Target.sdk_target.x);
				display_6_8_string(0,5,"Tar_y:");    write_6_8_number(80,5,Opv_Front_View_Target.sdk_target.y); 
				display_6_8_string(0,6,"FPS:");      write_6_8_number(80,6,Opv_Front_View_Target.fps);
				display_6_8_string(0,7,"Dis:");      write_6_8_number(25,7,Opv_Front_View_Target.apriltag_distance);
																						 //write_6_8_number(75,7,Opv_Front_View_Target.apriltag_id);				
																						 write_6_8_number(120,7,Opv_Front_View_Target.apriltag_id);			
		}
		break;
		case 8:
		{
				LCD_clear_L(0,0);  					 			display_6_8_string(0,0,"OptType");  		
				if(Optical_Type_Present==1)  			display_6_8_string(45,0,"LC307");
				else if(Optical_Type_Present==2)  display_6_8_string(45,0,"LC302");
				else if(Optical_Type_Present==3)  display_6_8_string(45,0,"LIDAR_T265");
				else if(Optical_Type_Present==4)  display_6_8_string(45,0,"3DLIDAR_UNI");
				display_6_8_number(115,0,Page_Number+1);
				
				switch(Optical_Type_Present)
				{
					case 1:
					case 2:
					{
						LCD_clear_L(0,1);  display_6_8_string(0,1,"flow_x:");        display_6_8_number(50,1,opt_origin_data.pixel_flow_x_integral);     display_6_8_number(90,1,opt_gyro_data.x); 
						LCD_clear_L(0,2);  display_6_8_string(0,2,"flow_y:");        display_6_8_number(50,2,opt_origin_data.pixel_flow_y_integral);     display_6_8_number(90,2,opt_gyro_data.y);    
						LCD_clear_L(0,3);  display_6_8_string(0,3,"time:");       	display_6_8_number(50,3,opt_origin_data.integration_timespan);       display_6_8_number(90,3,opt_data.dt);    
						LCD_clear_L(0,4);  display_6_8_string(0,4,"qual:");      		 display_6_8_number(50,4,opt_origin_data.qual);            									  
						LCD_clear_L(0,5);  display_6_8_string(0,5,"gyro_x:");        display_6_8_number(50,5,opt_gyro_data.x);              
						LCD_clear_L(0,6);  display_6_8_string(0,6,"gyro_y:");        display_6_8_number(50,6,opt_gyro_data.y);           
						LCD_clear_L(0,7);  display_6_8_string(0,7,"opt_flow:");      display_6_8_number(40,7,opt_gyro_filter_data.x);            display_6_8_number(70,7,opt_gyro_filter_data.y);      
					}break;
					case 3:
					case 4:
					{
						LCD_clear_L(0,1);  display_6_8_string(0,1,"p_v_x:");    display_6_8_number(45,1,current_state.position_x); display_6_8_number(100,1,current_state.velocity_x); 
						LCD_clear_L(0,2);  display_6_8_string(0,2,"p_v_y:");    display_6_8_number(45,2,current_state.position_y); display_6_8_number(100,2,current_state.velocity_y);         
						LCD_clear_L(0,3);  display_6_8_string(0,3,"p_v_z:");    display_6_8_number(45,3,current_state.position_z); display_6_8_number(100,3,current_state.velocity_z);   	   
						LCD_clear_L(0,4);  display_6_8_string(0,4,"pitch:");    display_6_8_number(45,4,current_state.rpy[1]);	   display_6_8_number(100,4,min_dis_cm);   									  
						LCD_clear_L(0,5);  display_6_8_string(0,5,"roll:");     display_6_8_number(45,5,current_state.rpy[0]);		 display_6_8_number(100,5,min_dis_angle);
						LCD_clear_L(0,6);  display_6_8_string(0,6,"yaw:");      display_6_8_number(45,6,current_state.rpy[2]);  	 display_6_8_number(100,6,target_yaw_err); 
						LCD_clear_L(0,7);	 display_6_8_string(0,7,"qual_update:");
															 display_6_8_number(60,7,current_state.quality);
															 display_6_8_number(100,7,current_state.update_flag	);
					}break;
				}
				
				//����״̬�²���������
				if(Controler_State==Unlock_Controler) return;
				
				//����ң�����������������ö�λ��������
				if(Pitch_Control>=Scale_Pecent_Max*Pit_Rol_Max)
				{
					delay_ms(10);
					if(Pitch_Control>=Scale_Pecent_Max*Pit_Rol_Max)
					{
						while(Pitch_Control>=Scale_Pecent_Max*Pit_Rol_Max);
						Optical_Type_Present++;
						if(Optical_Type_Present>4) Optical_Type_Present=1;
						WriteFlashParameter(OPTICAL_TYPE,
																Optical_Type_Present);
					}
				}
				//������һҳ�����������ö�λ��������
				if(_button.state[DOWN].press==LONG_PRESS)
				{
						_button.state[DOWN].press=NO_PRESS;
						Optical_Type_Present++;
						if(Optical_Type_Present>4) Optical_Type_Present=1;
						WriteFlashParameter(OPTICAL_TYPE,
																Optical_Type_Present);
						Bling_Set(&rgb_blue,500,50,0.2,0,GPIO_PORTF_BASE,GPIO_PIN_2,0);//��ɫ				
				}	   
				
				//SLAM��ͼ��λ
				if(Throttle_Control<=(1000+Buttom_Safe_Deadband)  //���ŵ�λ
					 &&Yaw_Control>=Yaw_Max*Scale_Pecent_Max        //ƫ����λ
					 &&Pitch_Control<=-Scale_Pecent_Max*Pit_Rol_Max)//������λ
				{
					delay_ms(1000);
					if(Pitch_Control<=-Scale_Pecent_Max*Pit_Rol_Max)
					{
						while(Pitch_Control<=-Scale_Pecent_Max*Pit_Rol_Max);
						send_check_back=4;//����slam
					}
				}	

				//������һҳ������������slam
				if(_button.state[UP].press==LONG_PRESS)
				{
						_button.state[UP].press=NO_PRESS;
						send_check_back=4;//����slam				
				}	   						
		}
		break;
		case 9:
		{
				uint16_t x,y,x1,y1;
				ssd1306_clear_display();
				ssd1306_draw_line(31,31,96,31,WHITE);
				ssd1306_draw_line(63,0,63,63,WHITE);
				ssd1306_draw_circle(63,31,31,WHITE);
				x1=(uint16_t)(63-31*WP_AHRS.sin_rpy[_YAW]);
				y1=(uint16_t)(31-31*WP_AHRS.cos_rpy[_YAW]);
				ssd1306_draw_line(63,31,x1,y1,WHITE);
				y=(uint16_t)(31+31*constrain_float(WP_AHRS.Pitch,-90,90)/90);
				x=(uint16_t)(63+31*constrain_float(WP_AHRS.Roll,-90,90)/90);
				ssd1306_fill_circle(x,y,5,WHITE);
				ssd1306_display();
				display_6_8_number(115,0,Page_Number+1);
				
				display_6_8_string(60,0,"N");
				display_6_8_string(60,7,"S");
				display_6_8_string(0,1,"Yaw:");            display_6_8_number(90,1,WP_AHRS.Yaw);                             
				display_6_8_string(0,2,"Pitch:");          display_6_8_number(90,2,WP_AHRS.Pitch); 
				display_6_8_string(0,3,"Roll:");           display_6_8_number(90,3,WP_AHRS.Roll); 		
		}
		break;
		case 10:
		{
			LCD_clear_L(0,0);
			display_6_8_string(10,0,"Accel_Correct");
			display_6_8_number(100,0,flight_direction);display_6_8_number(115,0,Page_Number+1);
			LCD_clear_L(0,1);
			display_6_8_number(0,1,acce_sample[0].x);
			display_6_8_number(40,1,acce_sample[0].y);
			display_6_8_number(70,1,acce_sample[0].z);
			LCD_clear_L(0,2);
			display_6_8_number(0,2,acce_sample[1].x);
			display_6_8_number(40,2,acce_sample[1].y);
			display_6_8_number(70,2,acce_sample[1].z);
			LCD_clear_L(0,3);
			display_6_8_number(0,3,acce_sample[2].x);
			display_6_8_number(40,3,acce_sample[2].y);
			display_6_8_number(70,3,acce_sample[2].z);
			LCD_clear_L(0,4);
			display_6_8_number(0,4,acce_sample[3].x);
			display_6_8_number(40,4,acce_sample[3].y);
			display_6_8_number(70,4,acce_sample[3].z);
			LCD_clear_L(0,5);
			display_6_8_number(0,5,acce_sample[4].x);
			display_6_8_number(40,5,acce_sample[4].y);
			display_6_8_number(70,5,acce_sample[4].z);
			LCD_clear_L(0,6);
			display_6_8_number(0,6,acce_sample[5].x);
			display_6_8_number(40,6,acce_sample[5].y);
			display_6_8_number(70,6,acce_sample[5].z);		
		}
		break;
		case 11:
		{
				LCD_clear_L(0,0);
				display_6_8_string(10,0,"Mag_Correct");
				display_6_8_number(90,0,Mag_Calibration_Mode);display_6_8_number(115,0,Page_Number+1);
				LCD_clear_L(0,1);
				display_6_8_number(0,1, (int16_t)(WP_Sensor.accel_raw.x));
				display_6_8_number(40,1,(int16_t)(WP_Sensor.accel_raw.y));
				display_6_8_number(70,1,(int16_t)(WP_Sensor.accel_raw.z));
				LCD_clear_L(0,2);
				display_6_8_string(0,2,"0 To 360");
				display_6_8_number(70,2,Yaw_Correct);
				LCD_clear_L(0,3);
				display_6_8_string(0,3,"Mag Is Okay?");
				display_6_8_number(80,3,Mag_Is_Okay_Flag[0]);
				display_6_8_number(90,3,Mag_Is_Okay_Flag[1]);
				display_6_8_number(105,3,Mag_Is_Okay_Flag[2]);
				LCD_clear_L(0,4);
				for(i=0;i<12;i++)
				{
					display_6_8_number(10*i,4,Mag_360_Flag[0][3*i]);
				}
				LCD_clear_L(0,5);
				for(i=0;i<12;i++)
				{
					display_6_8_number(10*i,5,Mag_360_Flag[1][3*i]);
				}
				LCD_clear_L(0,6);
				for(i=0;i<12;i++)
				{
					display_6_8_number(10*i,6,Mag_360_Flag[2][3*i]);
				}
				LCD_clear_L(0,7);
				if(Mag_Calibration_Mode==0) 		 display_6_8_string(0,7,"Make Z+ Upside Sky");
				else if(Mag_Calibration_Mode==1) display_6_8_string(0,7,"Make Y+ Upside Sky");
				else if(Mag_Calibration_Mode==2) display_6_8_string(0,7,"Make X+ Upside Sky");
				else if(Mag_Calibration_Mode==3) display_6_8_string(0,7,"Start With Yaw Move"); 
				
				
				//������һҳУ׼����ˮƽ
				if(_button.state[UP].press==LONG_PRESS)
				{
					_button.state[UP].press=NO_PRESS;
					cal_flag=0x03;
					cal_step=0x01;
					Bling_Set(&rgb_blue,500,50,0.2,0,GPIO_PORTF_BASE,GPIO_PIN_2,0);//��ɫ				
				}		
		}
		break;
		case 12:
		{
				LCD_clear_L(0,0);
				display_6_8_string(10,0,"Sensor Data");   display_6_8_number(115,0,Page_Number+1);
				LCD_clear_L(0,2);display_6_8_string(0,2,"mag:");
				display_6_8_number(25,2,WP_Sensor.mag_raw.x);
				display_6_8_number(60,2,WP_Sensor.mag_raw.y);
				display_6_8_number(95,2,WP_Sensor.mag_raw.z);
				LCD_clear_L(0,3);display_6_8_string(0,3,"acc:");
				display_6_8_number(25,3,WP_Sensor.accel_raw.x);
				display_6_8_number(60,3,WP_Sensor.accel_raw.y);
				display_6_8_number(95,3,WP_Sensor.accel_raw.z);
				LCD_clear_L(0,4);display_6_8_string(0,4,"gyr:");
				display_6_8_number(25,4,WP_Sensor.gyro_raw.x);
				display_6_8_number(60,4,WP_Sensor.gyro_raw.y);
				display_6_8_number(95,4,WP_Sensor.gyro_raw.z);
				LCD_clear_L(0,5);display_6_8_string(0,5,"bar:");
				display_6_8_number(25,5,WP_Sensor.baro_pressure_raw);
				display_6_8_number(60,5,WP_Sensor.baro_temp_raw);
				LCD_clear_L(0,6);display_6_8_string(0,6,"opt:");
				display_6_8_number(25,6,opt_origin_data.pixel_flow_x_integral);
				display_6_8_number(60,6,opt_origin_data.pixel_flow_y_integral);
				LCD_clear_L(0,7);display_6_8_string(0,7,"tfm:");
				display_6_8_number(25,7,tfdata.distance);
				display_6_8_number(65,7,tfdata.strength);
				display_6_8_number(95,7,tfdata.temperature);	
		}
		break;
		case 13:
		{
				LCD_clear_L(0,0);display_6_8_number(115,0,Page_Number+1); 
				LCD_clear_L(0,1);  
				display_6_8_string(5,0,"Ground_Sensor");display_6_8_number(110,1,rangefinder_current);
				
				LCD_clear_L(0,2); LCD_clear_L(0,3);
				display_6_8_string(5,2,"Now_Mode");
				
				LCD_clear_L(0,4); LCD_clear_L(0,5);  
				if(rangefinder_current==0x01)         	display_6_8_string(5,4,"US100 Mode");
				else if(rangefinder_current==0x02)      display_6_8_string(5,4,"tfmini Mode");
				else if(rangefinder_current==0x03)      display_6_8_string(5,4,"tofsense Mode");
				else if(rangefinder_current==0x04)      display_6_8_string(5,4,"SMD15 Mode");
				else if(rangefinder_current==0x05)      display_6_8_string(5,4,"tofsense_m Mode");
			
				LCD_clear_L(0,6); LCD_clear_L(0,7); 
				if(Pitch_Control>=Scale_Pecent_Max*Pit_Rol_Max)
				{
					delay_ms(10);
					if(Pitch_Control>=Scale_Pecent_Max*Pit_Rol_Max)
					{
						while(Pitch_Control>=Scale_Pecent_Max*Pit_Rol_Max);
						rangefinder_current++;
						if(rangefinder_current>5) rangefinder_current=1;
						WriteFlashParameter(GROUND_DISTANCE_DEFAULT,
																rangefinder_current);
						tofsense_init();
					}
				}
				
				if(_button.state[DOWN].press==LONG_PRESS)
				{
						_button.state[DOWN].press=NO_PRESS;
						rangefinder_current++;
						if(rangefinder_current>5) rangefinder_current=1;
						WriteFlashParameter(GROUND_DISTANCE_DEFAULT,
																rangefinder_current);
						tofsense_init();
						Bling_Set(&rgb_blue,500,50,0.2,0,GPIO_PORTF_BASE,GPIO_PIN_2,0);//��ɫ				
				}				
				
				LCD_clear_L(0,6);display_6_8_string(0,6,"tf:");  							display_6_8_string(50,6,"us:");       display_6_8_string(90,6,"smd:");
				LCD_clear_L(0,7);display_6_8_number(0,7,tfdata.distance);     display_6_8_number(50,7,GD_Distance); display_6_8_number(90,7,0);	
		}
		break;
		case 14:
		{
				LCD_clear_L(0,0);  display_6_8_number(115,0,Page_Number+1);
													 display_6_8_string(0,0,"ch1:");display_6_8_number(25,0,PPM_Databuf[0]);display_6_8_number(55,0,RC_Data.cal[0].max);display_6_8_number(85,0,RC_Data.cal[0].min);
				LCD_clear_L(0,1);  display_6_8_string(0,1,"ch2:");display_6_8_number(25,1,PPM_Databuf[1]);display_6_8_number(55,1,RC_Data.cal[1].max);display_6_8_number(85,1,RC_Data.cal[1].min);
				LCD_clear_L(0,2);  display_6_8_string(0,2,"ch3:");display_6_8_number(25,2,PPM_Databuf[2]);display_6_8_number(55,2,RC_Data.cal[2].max);display_6_8_number(85,2,RC_Data.cal[2].min);
				LCD_clear_L(0,3);  display_6_8_string(0,3,"ch4:");display_6_8_number(25,3,PPM_Databuf[3]);display_6_8_number(55,3,RC_Data.cal[3].max);display_6_8_number(85,3,RC_Data.cal[3].min);
				LCD_clear_L(0,4);  display_6_8_string(0,4,"ch5:");display_6_8_number(25,4,PPM_Databuf[4]);display_6_8_number(55,4,RC_Data.cal[4].max);display_6_8_number(85,4,RC_Data.cal[4].min);
				LCD_clear_L(0,5);  display_6_8_string(0,5,"ch6:");display_6_8_number(25,5,PPM_Databuf[5]);display_6_8_number(55,5,RC_Data.cal[5].max);display_6_8_number(85,5,RC_Data.cal[5].min);
				LCD_clear_L(0,6);  display_6_8_string(0,6,"ch7:");display_6_8_number(25,6,PPM_Databuf[6]);display_6_8_number(55,6,RC_Data.cal[6].max);display_6_8_number(85,6,RC_Data.cal[6].min);
				LCD_clear_L(0,7);  display_6_8_string(0,7,"ch8:");display_6_8_number(25,7,PPM_Databuf[7]);display_6_8_number(55,7,RC_Data.cal[7].max);display_6_8_number(85,7,RC_Data.cal[7].min);
		}
		break;
		case 15:
		{
				LCD_clear_L(0,0);display_6_8_number(115,0,Page_Number+1);
				
				LCD_clear_L(0,1);display_6_8_string(0,0,"Please Move Thr");
				LCD_clear_L(0,2);LCD_clear_L(0,3);display_6_8_string(0,2,"Up And Pull Out");
				LCD_clear_L(0,4);LCD_clear_L(0,5);display_6_8_string(0,4,"The Power Supply");
				LCD_clear_L(0,6);LCD_clear_L(0,7);display_6_8_string(0,6,"ESC Calibration");	
		}
		break;
		case 16:
		{
				display_6_8_string(30,0,"pos:"); display_6_8_number(70,0,NamelessQuad.Position[_UP]);	display_6_8_number(115,0,Page_Number+1);
				display_6_8_string(30,2,"vel:"); display_6_8_number(70,2,NamelessQuad.Speed[_UP]);
				display_6_8_string(30,6,"acc:"); display_6_8_number(70,6,NamelessQuad.Acceleration[_UP]);
				draw_ins_wave();	
		}
		break;
		case 17:
		{
				LCD_clear_L(0,0);display_6_8_string(0,0,"tof_id");          
												 display_6_8_string(50,0,"dis");										  
												 display_6_8_string(90,0,"str");		
												 display_6_8_number(115,0,Page_Number+1);
				LCD_clear_L(0,1);display_6_8_number(0,1,tofdata[0].id);		  
												 display_6_8_number(15,1,tofdata[0].distance);				
												 display_6_8_number(65,1,tofdata[0].signal_strength);
												 display_6_8_number(90,1, tofdata[5].distance);
				
				LCD_clear_L(0,2);display_6_8_number(0,2,tofdata[1].id);		  
												 display_6_8_number(15,2,tofdata[1].distance);				
												 display_6_8_number(65,2,tofdata[1].signal_strength);
												 display_6_8_number(90,2, tofdata[6].distance);		
				
				LCD_clear_L(0,3);display_6_8_number(0,3,tofdata[2].id);		  
												 display_6_8_number(15,3,tofdata[2].distance);				
												 display_6_8_number(65,3,tofdata[2].signal_strength);
												 display_6_8_number(90,3, tofdata[7].distance);	
				
				LCD_clear_L(0,4);display_6_8_number(0,4,tofdata[3].id);		  
												 display_6_8_number(15,4,tofdata[3].distance);				
												 display_6_8_number(65,4,tofdata[3].signal_strength);
												 display_6_8_number(90,4, tofdata[8].distance);		
				
				LCD_clear_L(0,5);display_6_8_number(0,5,tofdata[4].id);		  
												 display_6_8_number(15,5,tofdata[4].distance);				
												 display_6_8_number(65,5,tofdata[4].signal_strength);			
												 display_6_8_number(90,5, tofdata[9].distance);			

				LCD_clear_L(0,6);display_6_8_string(0,6,"tof_min");		      
												 display_6_8_number(50,6,front_tofsense_distance);				
												 display_6_8_number(90,6,front_tofsense_distance_valid_cnt);	
				LCD_clear_L(0,7);display_6_8_string(0,7,"front_us");       
												 display_6_8_number(70,7,us100_front.distance);
		}
		break;
		case 18:
		{
				LCD_clear_L(0,0);
				display_6_8_string(0,0,"GPS_Date"); display_6_8_number(50,0,0);display_6_8_number(100,0,Page_Number+1);
				LCD_clear_L(0,1); 
				display_6_8_string(0,1,"Lo_deg:");
				display_6_8_number(50,1,Longitude_Deg);
				LCD_clear_L(0,2);
				display_6_8_string(0,2,"d/100w:");
				display_6_8_number(50,2,Longitude_Min);
				
				LCD_clear_L(0,3);
				display_6_8_string(0,3,"La_deg:");
				display_6_8_number(50,3,Latitude_Deg);
				LCD_clear_L(0,4);
				display_6_8_string(0,4,"d/100w:");
				display_6_8_number(50,4,Latitude_Min);
				
				LCD_clear_L(0,5);
				if(GPS_Fix_Flag[0]==0x01)
					display_6_8_string(0,5,"fix");
				else display_6_8_string(0,5,"nof");
				
				if(GPS_FixType==0x03)
					display_6_8_string(25,5,"3D");
				else if(GPS_FixType==0x02)
					display_6_8_string(25,5,"2D");
				else display_6_8_string(25,5,"0D");
				display_6_8_number(42,5,GPS_Sate_Num);
				display_6_8_number(60,5,TimeBeijing[0]);
				display_6_8_string(75,5,":");
				display_6_8_number(84,5,TimeBeijing[1]);
				display_6_8_string(100,5,":");
				display_6_8_number(105,5,TimeBeijing[2]);
				LCD_clear_L(0,6);
				display_6_8_string(0,6,"V:");
				display_6_8_number(10,6,GPS_Vel.N);
				display_6_8_number(65,6,GPS_Vel.E);
				LCD_clear_L(0,7);
				display_6_8_number(0,7,Horizontal_Acc_Est);
				display_6_8_number(80,7,GPS_Quality);
		}
		break;
		case 19:
		{
				draw_gps_wave();																						display_6_8_string(30,0,"fus:");   												display_6_8_string(95,0,"obs:");
				display_6_8_string(0,1,"P_E:");														  display_6_8_number(30,1,NamelessQuad.Position[_EAST]);    display_6_8_number(95,1,Earth_Frame_To_XYZ.E);
				display_6_8_string(0,2,"P_N:");														  display_6_8_number(30,2,NamelessQuad.Position[_NORTH]);   display_6_8_number(95,2,Earth_Frame_To_XYZ.N);
				display_6_8_string(0,3,"V_E:");														  display_6_8_number(30,3,NamelessQuad.Speed[_EAST]);    	  display_6_8_number(95,3,GPS_Speed_Resolve[1]);
				display_6_8_string(0,4,"V_N:");														  display_6_8_number(30,4,NamelessQuad.Speed[_NORTH]);      display_6_8_number(95,4,GPS_Speed_Resolve[0]);		
				display_6_8_string(0,5,"A_B:");	 														display_6_8_number(30,5,Acce_Bias[_EAST]);								display_6_8_number(95,4,GPS_Home_Set);  
				display_6_8_string(0,6,"GPS:"); 														display_6_8_number(30,6,GPS_Sate_Num);	  								display_6_8_number(100,6,GPS_Quality);
				display_6_8_string(0,7,"Default:");                         display_6_8_number(30,7,Filter_Defeated_Flag);			
		}
		break;
		case 20:
		{
				static int16_t dis_scale=2;
				static uint8_t show_mode=0x01;
				uint16_t x0,y0,x1,y1;
				ssd1306_clear_display();
				ssd1306_draw_line(31,31,96,31,WHITE);
				ssd1306_draw_line(63,0,63,63,WHITE);
				x0=63;y0=31;
				for(uint16_t i=0;i<450;i++)
				{
					float ang=360.0f*((i+1)/450.0f);
					float dis=_point_cloud[i].distance_mm/1000.0;//ת����m
					x1=x0+31*sinf(ang*DEG2RAD)*(dis/dis_scale);
					y1=y0-31*cosf(ang*DEG2RAD)*(dis/dis_scale);
					if(show_mode)	ssd1306_draw_line(x0,y0,x1,y1,WHITE);
					else ssd1306_draw_pixel(x1,y1,WHITE);
				}	
				ssd1306_display();
				display_6_8_number(0,0,dis_scale);
				
				display_6_8_number(80,1,_laser_state.dis_min/10.0f);
				display_6_8_number(80,2,_laser_state.angle_min);
				display_6_8_number(80,3,_laser_state.strength_min);
				display_6_8_number(80,4,_laser_state.dis_max/10.0f);
				display_6_8_number(80,5,_laser_state.angle_max);
				display_6_8_number(80,6,_laser_state.strength_max);
				
				//����״̬�²���������
				if(Controler_State==Unlock_Controler) return;
				//����ң�����������������ö�λ��������
				if(Pitch_Control>=Scale_Pecent_Max*Pit_Rol_Max)
				{
					delay_ms(10);
					if(Pitch_Control>=Scale_Pecent_Max*Pit_Rol_Max)
					{
						while(Pitch_Control>=Scale_Pecent_Max*Pit_Rol_Max);
						dis_scale++;
						if(dis_scale>12) dis_scale=2;
					}
				}
				
				if(Pitch_Control<=-Scale_Pecent_Max*Pit_Rol_Max)
				{
					delay_ms(10);
					if(Pitch_Control<=-Scale_Pecent_Max*Pit_Rol_Max)
					{
						while(Pitch_Control<=-Scale_Pecent_Max*Pit_Rol_Max);
						if(show_mode==0x00) show_mode=0x01;
						else show_mode=0x00;
					}
				}	
		}
		break;
		case 21://��������Ŀ��ص�
		{
			  static uint8_t ver_choose=1;
				LCD_clear_L(0,0); display_6_8_string(0,0,"reserved_params");	display_6_8_number(115,0,Page_Number+1);
				LCD_clear_L(0,1); display_6_8_string(0,1,"scale:"); display_6_8_number(50,1,param_value[0]);//�����߶Ȳ���A  
			  //���յ�������A*(x,y)ȷ��������25*(8,11)=(200,275)
			
				//��һ������x1��y1
			  LCD_clear_L(0,2); display_6_8_string(0,2,"x1:");	  display_6_8_number(50,2,param_value[1]);   display_6_8_number(90,2,param_value[1]*param_value[0]);
				LCD_clear_L(0,3); display_6_8_string(0,3,"y1:");    display_6_8_number(50,3,param_value[2]);   display_6_8_number(90,3,param_value[2]*param_value[0]);
				//�ڶ�������x2��y2																												 
				LCD_clear_L(0,4); display_6_8_string(0,4,"x2:");    display_6_8_number(50,4,param_value[3]);	 display_6_8_number(90,4,param_value[3]*param_value[0]);
				LCD_clear_L(0,5); display_6_8_string(0,5,"y2:");    display_6_8_number(50,5,param_value[4]);   display_6_8_number(90,5,param_value[4]*param_value[0]);
			 //����������x3��y3
				LCD_clear_L(0,6); display_6_8_string(0,6,"x3:");    display_6_8_number(50,6,param_value[5]);	 display_6_8_number(90,6,param_value[5]*param_value[0]);
				LCD_clear_L(0,7); display_6_8_string(0,7,"y3:");    display_6_8_number(50,7,param_value[6]);	 display_6_8_number(90,7,param_value[6]*param_value[0]);

				display_6_8_string(40,ver_choose,"*");				
				//ͨ��3D������ʵ�ֻ���ѡ�д��޸Ĳ���
				if(_button.state[UP_3D].press==SHORT_PRESS)
				{
						_button.state[UP_3D].press=NO_PRESS;
						ver_choose--;
						if(ver_choose<1) ver_choose=7;
						Bling_Set(&rgb_blue,500,50,0.2,0,GPIO_PORTF_BASE,GPIO_PIN_2,0);//��ɫ				
				}
				if(_button.state[DN_3D].press==SHORT_PRESS)
				{
						_button.state[DN_3D].press=NO_PRESS;
						ver_choose++;
						if(ver_choose>7) ver_choose=1;
						Bling_Set(&rgb_blue,500,50,0.2,0,GPIO_PORTF_BASE,GPIO_PIN_2,0);//��ɫ				
				}
				
				
				//ͨ��3D������ʵ�ֿ���ʵ��ѡ�еĲ����������ӵ���
				if(_button.state[RT_3D].press==SHORT_PRESS)
				{
						_button.state[RT_3D].press=NO_PRESS;
						Bling_Set(&rgb_blue,500,50,0.2,0,GPIO_PORTF_BASE,GPIO_PIN_2,0);//��ɫ	
						switch(ver_choose)
						{
							case 1://�޸ı���ϵ��
							{
								param_value[0]+=5;
								if(param_value[0]>100) param_value[0]=0;
							}
							break;
							case 2://x1
							{
								param_value[1]+=1;
								if(param_value[1]>20) param_value[1]=-20;
							}
							break;						
							case 3://y1
							{
								param_value[2]+=1;
								if(param_value[2]>20) param_value[2]=-20;
							}
							break;
							case 4://x2
							{
								param_value[3]+=1;
								if(param_value[3]>20) param_value[3]=-20;
							}
							break;						
							case 5://y2
							{
								param_value[4]+=1;
								if(param_value[4]>20) param_value[4]=-20;
							}
							break;	
							case 6://x3
							{
								param_value[5]+=1;
								if(param_value[5]>20) param_value[5]=-20;
							}
							break;						
							case 7://y3
							{
								param_value[6]+=1;
								if(param_value[6]>20) param_value[6]=-20;
							}
							break;								
						}
						//���º�Բ������б���
						WriteFlashParameter(RESERVED_PARAM,  param_value[0]);
						WriteFlashParameter(RESERVED_PARAM+1,param_value[1]);
						WriteFlashParameter(RESERVED_PARAM+2,param_value[2]);
						WriteFlashParameter(RESERVED_PARAM+3,param_value[3]);
						WriteFlashParameter(RESERVED_PARAM+4,param_value[4]);
						WriteFlashParameter(RESERVED_PARAM+5,param_value[5]);
						WriteFlashParameter(RESERVED_PARAM+6,param_value[6]);
				}	
				//ͨ��3D������ʵ�ֿ���ʵ��ѡ�еĲ����м�С����
				if(_button.state[LT_3D].press==SHORT_PRESS)
				{
						_button.state[LT_3D].press=NO_PRESS;
						Bling_Set(&rgb_blue,500,50,0.2,0,GPIO_PORTF_BASE,GPIO_PIN_2,0);//��ɫ	
						switch(ver_choose)
						{
							case 1://�޸ı���ϵ��
							{
								param_value[0]-=5;
								if(param_value[0]<0) param_value[0]=100;
							}
							break;
							case 2://x1
							{
								param_value[1]-=1;
								if(param_value[1]<-20) param_value[1]=20;
							}
							break;						
							case 3://y1
							{
								param_value[2]-=1;
								if(param_value[2]<-20) param_value[2]=20;
							}
							break;
							case 4://x2
							{
								param_value[3]-=1;
								if(param_value[3]<-20) param_value[3]=20;
							}
							break;						
							case 5://y2
							{
								param_value[4]-=1;
								if(param_value[4]<-20) param_value[4]=20;
							}
							break;	
							case 6://x3
							{
								param_value[5]-=1;
								if(param_value[5]<-20) param_value[5]=20;
							}
							break;						
							case 7://y3
							{
								param_value[6]-=1;
								if(param_value[6]<-20) param_value[6]=20;
							}
							break;								
						}
						//���º�Բ������б���
						WriteFlashParameter(RESERVED_PARAM,  param_value[0]);
						WriteFlashParameter(RESERVED_PARAM+1,param_value[1]);
						WriteFlashParameter(RESERVED_PARAM+2,param_value[2]);
						WriteFlashParameter(RESERVED_PARAM+3,param_value[3]);
						WriteFlashParameter(RESERVED_PARAM+4,param_value[4]);
						WriteFlashParameter(RESERVED_PARAM+5,param_value[5]);
						WriteFlashParameter(RESERVED_PARAM+6,param_value[6]);
				}				
		}
		break;
		case 22:
		{
				ssd1306_clear_display();
				ssd1306_draw_line(0,32,128,32,WHITE);
				ssd1306_draw_line(64,0,64,64,WHITE);			
				uint16_t x,y;
				y=32-32*constrain_float(0.5*Opv_Top_View_Target.height-Opv_Top_View_Target.y,-50,50)/50.0f;
				x=64-32*constrain_float(0.5*Opv_Top_View_Target.width-Opv_Top_View_Target.x,-50,50)/50.0f;
				if(Opv_Top_View_Target.flag==1)			ssd1306_fill_circle(x,y,2,WHITE);
				else ssd1306_draw_circle(x,y,2,WHITE);			
				
				display_6_8_string(0,0,"px:");  write_6_8_number(80,0,Opv_Top_View_Target.x);		write_6_8_number(105,0,Page_Number+1);
				display_6_8_string(0,1,"py:");  write_6_8_number(80,1,Opv_Top_View_Target.y);		write_6_8_number(105,1,Opv_Top_View_Target.sdk_mode-0xA0);
				display_6_8_string(0,2,"sq:");  write_6_8_number(80,2,Opv_Top_View_Target.pixel);
				display_6_8_string(0,3,"fg:");  write_6_8_number(80,3,Opv_Top_View_Target.flag);
				display_6_8_string(0,4,"cmx:"); write_6_8_number(60,4,Opv_Top_View_Target.sdk_target.x);	write_6_8_number(110,4,Opv_Top_View_Target.fps);
				display_6_8_string(0,5,"cmy:"); write_6_8_number(60,5,Opv_Top_View_Target.sdk_target.y); 
				display_6_8_string(0,6,"type:");
			
				if(Opv_Top_View_Target.reserved3==1) 			display_6_8_string(30,6,"red");
			  else if(Opv_Top_View_Target.reserved3==2) display_6_8_string(30,6,"blue");
				else display_6_8_string(30,6,"unk");
				
				if(Opv_Top_View_Target.reserved4==1) 			
				{
					display_6_8_string(65,6,"circular");
					ssd1306_draw_circle(64,32,16,WHITE);
				}
			  else if(Opv_Top_View_Target.reserved4==2) 
				{
					display_6_8_string(65,6,"rectangle");
					ssd1306_draw_rect(48,16,32,32,WHITE);	
				}
				else if(Opv_Top_View_Target.reserved4==3) 
				{
					display_6_8_string(65,6,"triangle");
					ssd1306_draw_triangle(48,48,80,48,64,21,WHITE);
				}
				else display_6_8_string(65,6,"unk");			
				ssd1306_display();
				
				display_6_8_string(0,7,"color");    
				display_6_8_number(80,7,param_value[7]);	 
				display_6_8_number(110,7,param_value[8]);
				
				static uint8_t hor_choose=1;
				display_6_8_string(40+30*(hor_choose-1),7,"*");
				//ͨ��3D������ʵ�ֻ���ѡ�д��޸Ĳ���
				if(_button.state[LT_3D].press==SHORT_PRESS)
				{
						_button.state[LT_3D].press=NO_PRESS;
						hor_choose--;
						if(hor_choose<1) hor_choose=3;
						Bling_Set(&rgb_blue,500,50,0.2,0,GPIO_PORTF_BASE,GPIO_PIN_2,0);//��ɫ				
				}
				if(_button.state[RT_3D].press==SHORT_PRESS)
				{
						_button.state[RT_3D].press=NO_PRESS;
						hor_choose++;
						if(hor_choose>3) hor_choose=1;
						Bling_Set(&rgb_blue,500,50,0.2,0,GPIO_PORTF_BASE,GPIO_PIN_2,0);//��ɫ				
				}
				
				
				
				//ͨ����һҳ������������ʵ��ѡ�еĲ����������ӵ���
				if(_button.state[DN_3D].press==SHORT_PRESS)
				{
						_button.state[DN_3D].press=NO_PRESS;
						Bling_Set(&rgb_blue,500,50,0.2,0,GPIO_PORTF_BASE,GPIO_PIN_2,0);//��ɫ	
						switch(hor_choose)
						{
							case 1://��OPENMVʶ�𵽵�ģ��������Ϊ�����Ŀ��
							{
								param_value[7]=Opv_Top_View_Target.reserved3;//��ɫ
								param_value[8]=Opv_Top_View_Target.reserved4;//��״
							}
							break;								
							case 2://�ɶ�ģ����ɫ����һ���޸�
							{
								param_value[7]+=1;
								if(param_value[7]>2) param_value[7]=1;
							}
							break;
							case 3://�ɶ�ģ����״����һ���޸�
							{
								param_value[8]+=1;
								if(param_value[8]>3) param_value[8]=1;
							}
							break;	
						}
						//���º�Բ������б���
						WriteFlashParameter(RESERVED_PARAM+7,param_value[7]);//��ɫ
						WriteFlashParameter(RESERVED_PARAM+8,param_value[8]);//��״
				}

				//ͨ����һҳ������������ʵ��ѡ�еĲ����������ӵ���
				if(_button.state[UP_3D].press==SHORT_PRESS)
				{
						_button.state[UP_3D].press=NO_PRESS;
						Bling_Set(&rgb_blue,500,50,0.2,0,GPIO_PORTF_BASE,GPIO_PIN_2,0);//��ɫ	
						switch(hor_choose)
						{
							case 1://��OPENMVʶ�𵽵�ģ��������Ϊ�����Ŀ��
							{
								param_value[7]=Opv_Top_View_Target.reserved3;//��ɫ
								param_value[8]=Opv_Top_View_Target.reserved4;//��״
							}
							break;								
							case 2://�ɶ�ģ����ɫ����һ���޸�
							{
								param_value[7]-=1;
								if(param_value[7]<1) param_value[7]=2;
							}
							break;
							case 3://�ɶ�ģ����״����һ���޸�
							{
								param_value[8]-=1;
								if(param_value[8]<1) param_value[8]=3;
							}
							break;	
						}
						//���º�Բ������б���
						WriteFlashParameter(RESERVED_PARAM+7,param_value[7]);//��ɫ
						WriteFlashParameter(RESERVED_PARAM+8,param_value[8]);//��״
				}		
		}
		break;
		case 23:
		{
			static uint8_t ver_item=1;
			static uint16_t step=1;
			LCD_clear_L(0,0);display_6_8_string(25,0,"loop_setup");	write_6_8_number(105,0,Page_Number+1);
			LCD_clear_L(0,1);display_6_8_string(0,1,"step");   		write_6_8_number(80,1,step);
			LCD_clear_L(0,2);display_6_8_string(0,2,"A_x");  			write_6_8_number(80,2,param_value[9]);
			LCD_clear_L(0,3);display_6_8_string(0,3,"A_y");  			write_6_8_number(80,3,param_value[10]);
			LCD_clear_L(0,4);display_6_8_string(0,4,"ang");				write_6_8_number(80,4,param_value[11]);
			LCD_clear_L(0,5);display_6_8_string(10,5,"save please long");			
			LCD_clear_L(0,6);display_6_8_string(10,6,"press middle key");
			
			display_6_8_string(25,ver_item,"*");
			//ͨ��3D������ʵ�ֻ���ѡ�д��޸Ĳ���
			if(_button.state[UP_3D].press==SHORT_PRESS)
			{
					_button.state[UP_3D].press=NO_PRESS;
					ver_item--;
					if(ver_item<1) ver_item=4;
					Bling_Set(&rgb_blue,500,50,0.2,0,GPIO_PORTF_BASE,GPIO_PIN_2,0);//��ɫ				
			}
			if(_button.state[DN_3D].press==SHORT_PRESS)
			{
					_button.state[DN_3D].press=NO_PRESS;
					ver_item++;
					if(ver_item>4) ver_item=1;
					Bling_Set(&rgb_blue,500,50,0.2,0,GPIO_PORTF_BASE,GPIO_PIN_2,0);//��ɫ				
			}
			

			//ͨ����һҳ����������������ʵ��ѡ�еĲ����������ӵ���
			if(_button.state[RT_3D].press==SHORT_PRESS)
			{
				_button.state[RT_3D].press=NO_PRESS;
				switch(ver_item)
				{
					case 1:
					{
						step+=1;
						if(step>100)	step=100;
					}
					break;
					case 2:
					{
						param_value[9]+=step;
						if(param_value[9]>500) param_value[9]=500;
					}
					break;
					case 3:
					{
						param_value[10]+=step;	
						if(param_value[10]>500) param_value[10]=500;
					}
					break;
					case 4:
					{
						param_value[11]+=step;
						if(param_value[11]>180) param_value[11]=180;
					}
					break;
				}
			}
			
			if(_button.state[LT_3D].press==SHORT_PRESS)
			{
				_button.state[LT_3D].press=NO_PRESS;
				switch(ver_item)
				{
					case 1:
					{
						step-=1;
						if(step<1)	step=1;
					}
					break;
					case 2:
					{
						param_value[9]-=step;
						if(param_value[9]<-500) param_value[9]=-500;
					}
					break;
					case 3:
					{
						param_value[10]-=step;	
						if(param_value[10]<-500) param_value[10]=-500;
					}
					break;
					case 4:
					{
						param_value[11]-=step;
						if(param_value[11]<0) param_value[11]=0;
					}
					break;
				}
			}
						
			//ͨ���м䰴����������ʵ�ִ�ҳ���õ����в�������
			if(_button.state[ME_3D].press==LONG_PRESS)
			{
				_button.state[ME_3D].press=NO_PRESS;
				//���º�Բ������б���
				WriteFlashParameter(RESERVED_PARAM+9 ,param_value[9]);
				WriteFlashParameter(RESERVED_PARAM+10,param_value[10]);
				WriteFlashParameter(RESERVED_PARAM+11,param_value[11]);
				Bling_Set(&rgb_blue,500,50,0.2,0,GPIO_PORTF_BASE,GPIO_PIN_2,0);//��ɫ	
			}		
		}
		break;	
		case 24:
		{
			uint16_t base_item=50;
			LCD_clear_L(0,0);		display_6_8_string(0,0,"nav_point:1/7 e-n-u");write_6_8_number(115,0,Page_Number+1);
			LCD_clear_L(0,1);		display_6_8_string(0,1,"p1");   		          write_6_8_number(25,1,param_value[base_item+0]);  write_6_8_number(60,1,param_value[base_item+1]);  write_6_8_number(95,1,param_value[base_item+2]);
			LCD_clear_L(0,2);		display_6_8_string(0,2,"p2");  			          write_6_8_number(25,2,param_value[base_item+3]);  write_6_8_number(60,2,param_value[base_item+4]);  write_6_8_number(95,2,param_value[base_item+5]);
			LCD_clear_L(0,3);		display_6_8_string(0,3,"p3");  			          write_6_8_number(25,3,param_value[base_item+6]);  write_6_8_number(60,3,param_value[base_item+7]);  write_6_8_number(95,3,param_value[base_item+8]);
			LCD_clear_L(0,4);		display_6_8_string(0,4,"p4");				          write_6_8_number(25,4,param_value[base_item+9]);  write_6_8_number(60,4,param_value[base_item+10]); write_6_8_number(95,4,param_value[base_item+11]);
			LCD_clear_L(0,5);		display_6_8_string(0,5,"p5");			            write_6_8_number(25,5,param_value[base_item+12]); write_6_8_number(60,5,param_value[base_item+13]); write_6_8_number(95,5,param_value[base_item+14]);
      LCD_clear_L(0,6);		display_6_8_string(0,6,"p6");				          write_6_8_number(25,6,param_value[base_item+15]); write_6_8_number(60,6,param_value[base_item+16]); write_6_8_number(95,6,param_value[base_item+17]);
      LCD_clear_L(0,7);		display_6_8_string(0,7,"p7");				          write_6_8_number(25,7,param_value[base_item+18]); write_6_8_number(60,7,param_value[base_item+19]);	write_6_8_number(95,7,param_value[base_item+20]);
			
			static uint16_t ver_item=1;	
			if(ver_item==1)	  display_6_8_string(18,1,"*"); else if(ver_item==2)	display_6_8_string(53,1,"*"); else if(ver_item==3)	display_6_8_string(88,1,"*");
			if(ver_item==4)  	display_6_8_string(18,2,"*"); else if(ver_item==5)	display_6_8_string(53,2,"*"); else if(ver_item==6)	display_6_8_string(88,2,"*");
			if(ver_item==7)	  display_6_8_string(18,3,"*"); else if(ver_item==8)	display_6_8_string(53,3,"*"); else if(ver_item==9)	display_6_8_string(88,3,"*");
			if(ver_item==10)	display_6_8_string(18,4,"*"); else if(ver_item==11)	display_6_8_string(53,4,"*"); else if(ver_item==12)	display_6_8_string(88,4,"*");
			if(ver_item==13)	display_6_8_string(18,5,"*"); else if(ver_item==14)	display_6_8_string(53,5,"*"); else if(ver_item==15)	display_6_8_string(88,5,"*");
			if(ver_item==16)	display_6_8_string(18,6,"*"); else if(ver_item==17)	display_6_8_string(53,6,"*"); else if(ver_item==18)	display_6_8_string(88,6,"*");
			if(ver_item==19)	display_6_8_string(18,7,"*"); else if(ver_item==20)	display_6_8_string(53,7,"*"); else if(ver_item==21)	display_6_8_string(88,7,"*");
			
			//ͨ��3D������ʵ�ֻ���ѡ�д��޸Ĳ���
			if(_button.state[UP_3D].press==SHORT_PRESS)
			{
					_button.state[UP_3D].press=NO_PRESS;
					ver_item--;
					if(ver_item<1) ver_item=21;
					Bling_Set(&rgb_blue,500,50,0.2,0,GPIO_PORTF_BASE,GPIO_PIN_2,0);//��ɫ				
			}
			if(_button.state[DN_3D].press==SHORT_PRESS)
			{
					_button.state[DN_3D].press=NO_PRESS;
					ver_item++;
					if(ver_item>21) ver_item=1;
					Bling_Set(&rgb_blue,500,50,0.2,0,GPIO_PORTF_BASE,GPIO_PIN_2,0);//��ɫ				
			}
			
			//ͨ���󰴼��̰�����ʵ��ѡ�еĲ������Լ�С1����
			if(_button.state[LT_3D].press==SHORT_PRESS)
			{
				_button.state[LT_3D].press=NO_PRESS;
			   param_value[base_item+ver_item-1]-=1;
			}
			
			//ͨ���Ұ����̰�����ʵ��ѡ�еĲ�����������1����
			if(_button.state[RT_3D].press==SHORT_PRESS)
			{
				_button.state[RT_3D].press=NO_PRESS;
			   param_value[base_item+ver_item-1]+=1;
			}			
			
			//ͨ���󰴼��̰�����ʵ��ѡ�еĲ������Լ�С50����
			if(_button.state[LT_3D].press==LONG_PRESS)
			{
				_button.state[LT_3D].press=NO_PRESS;
			   param_value[base_item+ver_item-1]-=50;
			}
			
			//ͨ���Ұ����̰�����ʵ��ѡ�еĲ�����������50����
			if(_button.state[RT_3D].press==LONG_PRESS)
			{
				_button.state[RT_3D].press=NO_PRESS;
			   param_value[base_item+ver_item-1]+=50;
			}		

			//ͨ���м䰴����������ʵ�ִ�ҳ���õ����в�������
			if(_button.state[ME_3D].press==LONG_PRESS)
			{
				_button.state[ME_3D].press=NO_PRESS;
				//���º�Բ������б���
				for(uint16_t i=0;i<21;i++)
				{
					WriteFlashParameter(RESERVED_PARAM+base_item+i,param_value[base_item+i]);
				}
			}				
		}
		break;
		case 25:
		{
			uint16_t base_item=50+21*1;
			LCD_clear_L(0,0);		display_6_8_string(0,0,"nav_point:8/14 e-n-u");write_6_8_number(115,0,Page_Number+1);
			LCD_clear_L(0,1);		display_6_8_string(0,1,"p8");   		           write_6_8_number(25,1,param_value[base_item+0]);  write_6_8_number(60,1,param_value[base_item+1]);  write_6_8_number(95,1,param_value[base_item+2]);
			LCD_clear_L(0,2);		display_6_8_string(0,2,"p9");  			           write_6_8_number(25,2,param_value[base_item+3]);  write_6_8_number(60,2,param_value[base_item+4]);  write_6_8_number(95,2,param_value[base_item+5]);
			LCD_clear_L(0,3);		display_6_8_string(0,3,"p10");  			         write_6_8_number(25,3,param_value[base_item+6]);  write_6_8_number(60,3,param_value[base_item+7]);  write_6_8_number(95,3,param_value[base_item+8]);
			LCD_clear_L(0,4);		display_6_8_string(0,4,"p11");				         write_6_8_number(25,4,param_value[base_item+9]);  write_6_8_number(60,4,param_value[base_item+10]); write_6_8_number(95,4,param_value[base_item+11]);
			LCD_clear_L(0,5);		display_6_8_string(0,5,"p12");			           write_6_8_number(25,5,param_value[base_item+12]); write_6_8_number(60,5,param_value[base_item+13]); write_6_8_number(95,5,param_value[base_item+14]);
      LCD_clear_L(0,6);		display_6_8_string(0,6,"p13");				         write_6_8_number(25,6,param_value[base_item+15]); write_6_8_number(60,6,param_value[base_item+16]); write_6_8_number(95,6,param_value[base_item+17]);
      LCD_clear_L(0,7);		display_6_8_string(0,7,"p14");				         write_6_8_number(25,7,param_value[base_item+18]); write_6_8_number(60,7,param_value[base_item+19]); write_6_8_number(95,7,param_value[base_item+20]);
			
			static uint16_t ver_item=1;	
			if(ver_item==1)	  display_6_8_string(18,1,"*"); else if(ver_item==2)	display_6_8_string(53,1,"*"); else if(ver_item==3)	display_6_8_string(88,1,"*");
			if(ver_item==4)  	display_6_8_string(18,2,"*"); else if(ver_item==5)	display_6_8_string(53,2,"*"); else if(ver_item==6)	display_6_8_string(88,2,"*");
			if(ver_item==7)	  display_6_8_string(18,3,"*"); else if(ver_item==8)	display_6_8_string(53,3,"*"); else if(ver_item==9)	display_6_8_string(88,3,"*");
			if(ver_item==10)	display_6_8_string(18,4,"*"); else if(ver_item==11)	display_6_8_string(53,4,"*"); else if(ver_item==12)	display_6_8_string(88,4,"*");
			if(ver_item==13)	display_6_8_string(18,5,"*"); else if(ver_item==14)	display_6_8_string(53,5,"*"); else if(ver_item==15)	display_6_8_string(88,5,"*");
			if(ver_item==16)	display_6_8_string(18,6,"*"); else if(ver_item==17)	display_6_8_string(53,6,"*"); else if(ver_item==18)	display_6_8_string(88,6,"*");
			if(ver_item==19)	display_6_8_string(18,7,"*"); else if(ver_item==20)	display_6_8_string(53,7,"*"); else if(ver_item==21)	display_6_8_string(88,7,"*");
			
			//ͨ��3D������ʵ�ֻ���ѡ�д��޸Ĳ���
			if(_button.state[UP_3D].press==SHORT_PRESS)
			{
					_button.state[UP_3D].press=NO_PRESS;
					ver_item--;
					if(ver_item<1) ver_item=21;
					Bling_Set(&rgb_blue,500,50,0.2,0,GPIO_PORTF_BASE,GPIO_PIN_2,0);//��ɫ				
			}
			if(_button.state[DN_3D].press==SHORT_PRESS)
			{
					_button.state[DN_3D].press=NO_PRESS;
					ver_item++;
					if(ver_item>21) ver_item=1;
					Bling_Set(&rgb_blue,500,50,0.2,0,GPIO_PORTF_BASE,GPIO_PIN_2,0);//��ɫ				
			}
			
			//ͨ���󰴼��̰�����ʵ��ѡ�еĲ������Լ�С1����
			if(_button.state[LT_3D].press==SHORT_PRESS)
			{
				_button.state[LT_3D].press=NO_PRESS;
			   param_value[base_item+ver_item-1]-=1;
			}
			
			//ͨ���Ұ����̰�����ʵ��ѡ�еĲ�����������1����
			if(_button.state[RT_3D].press==SHORT_PRESS)
			{
				_button.state[RT_3D].press=NO_PRESS;
			   param_value[base_item+ver_item-1]+=1;
			}			
			
			//ͨ���󰴼��̰�����ʵ��ѡ�еĲ������Լ�С50����
			if(_button.state[LT_3D].press==LONG_PRESS)
			{
				_button.state[LT_3D].press=NO_PRESS;
			   param_value[base_item+ver_item-1]-=50;
			}
			
			//ͨ���Ұ����̰�����ʵ��ѡ�еĲ�����������50����
			if(_button.state[RT_3D].press==LONG_PRESS)
			{
				_button.state[RT_3D].press=NO_PRESS;
			   param_value[base_item+ver_item-1]+=50;
			}		

			//ͨ���м䰴����������ʵ�ִ�ҳ���õ����в�������
			if(_button.state[ME_3D].press==LONG_PRESS)
			{
				_button.state[ME_3D].press=NO_PRESS;
				//���º�Բ������б���
				for(uint16_t i=0;i<21;i++)
				{
					WriteFlashParameter(RESERVED_PARAM+base_item+i,param_value[base_item+i]);
				}
			}				
		}
		break;
		case 26:
		{
			uint16_t base_item=50+21*2;
			LCD_clear_L(0,0);		display_6_8_string(0,0,"nav_point:15/21 e-n-u");write_6_8_number(115,0,Page_Number+1);
			LCD_clear_L(0,1);		display_6_8_string(0,1,"p15");   		           write_6_8_number(25,1,param_value[base_item+0]);  write_6_8_number(60,1,param_value[base_item+1]);  write_6_8_number(95,1,param_value[base_item+2]);
			LCD_clear_L(0,2);		display_6_8_string(0,2,"p16");  			         write_6_8_number(25,2,param_value[base_item+3]);  write_6_8_number(60,2,param_value[base_item+4]);  write_6_8_number(95,2,param_value[base_item+5]);
			LCD_clear_L(0,3);		display_6_8_string(0,3,"p17");  			         write_6_8_number(25,3,param_value[base_item+6]);  write_6_8_number(60,3,param_value[base_item+7]);  write_6_8_number(95,3,param_value[base_item+8]);
			LCD_clear_L(0,4);		display_6_8_string(0,4,"p18");				         write_6_8_number(25,4,param_value[base_item+9]);  write_6_8_number(60,4,param_value[base_item+10]); write_6_8_number(95,4,param_value[base_item+11]);
			LCD_clear_L(0,5);		display_6_8_string(0,5,"p19");			           write_6_8_number(25,5,param_value[base_item+12]); write_6_8_number(60,5,param_value[base_item+13]); write_6_8_number(95,5,param_value[base_item+14]);
      LCD_clear_L(0,6);		display_6_8_string(0,6,"p20");				         write_6_8_number(25,6,param_value[base_item+15]); write_6_8_number(60,6,param_value[base_item+16]); write_6_8_number(95,6,param_value[base_item+17]);
      LCD_clear_L(0,7);		display_6_8_string(0,7,"p21");				         write_6_8_number(25,7,param_value[base_item+18]); write_6_8_number(60,7,param_value[base_item+19]); write_6_8_number(95,7,param_value[base_item+20]);
			
			static uint16_t ver_item=1;	
			if(ver_item==1)	  display_6_8_string(18,1,"*"); else if(ver_item==2)	display_6_8_string(53,1,"*"); else if(ver_item==3)	display_6_8_string(88,1,"*");
			if(ver_item==4)  	display_6_8_string(18,2,"*"); else if(ver_item==5)	display_6_8_string(53,2,"*"); else if(ver_item==6)	display_6_8_string(88,2,"*");
			if(ver_item==7)	  display_6_8_string(18,3,"*"); else if(ver_item==8)	display_6_8_string(53,3,"*"); else if(ver_item==9)	display_6_8_string(88,3,"*");
			if(ver_item==10)	display_6_8_string(18,4,"*"); else if(ver_item==11)	display_6_8_string(53,4,"*"); else if(ver_item==12)	display_6_8_string(88,4,"*");
			if(ver_item==13)	display_6_8_string(18,5,"*"); else if(ver_item==14)	display_6_8_string(53,5,"*"); else if(ver_item==15)	display_6_8_string(88,5,"*");
			if(ver_item==16)	display_6_8_string(18,6,"*"); else if(ver_item==17)	display_6_8_string(53,6,"*"); else if(ver_item==18)	display_6_8_string(88,6,"*");
			if(ver_item==19)	display_6_8_string(18,7,"*"); else if(ver_item==20)	display_6_8_string(53,7,"*"); else if(ver_item==21)	display_6_8_string(88,7,"*");
			
			//ͨ��3D������ʵ�ֻ���ѡ�д��޸Ĳ���
			if(_button.state[UP_3D].press==SHORT_PRESS)
			{
					_button.state[UP_3D].press=NO_PRESS;
					ver_item--;
					if(ver_item<1) ver_item=21;
					Bling_Set(&rgb_blue,500,50,0.2,0,GPIO_PORTF_BASE,GPIO_PIN_2,0);//��ɫ				
			}
			if(_button.state[DN_3D].press==SHORT_PRESS)
			{
					_button.state[DN_3D].press=NO_PRESS;
					ver_item++;
					if(ver_item>21) ver_item=1;
					Bling_Set(&rgb_blue,500,50,0.2,0,GPIO_PORTF_BASE,GPIO_PIN_2,0);//��ɫ				
			}
			
			//ͨ���󰴼��̰�����ʵ��ѡ�еĲ������Լ�С1����
			if(_button.state[LT_3D].press==SHORT_PRESS)
			{
				_button.state[LT_3D].press=NO_PRESS;
			   param_value[base_item+ver_item-1]-=1;
			}
			
			//ͨ���Ұ����̰�����ʵ��ѡ�еĲ�����������1����
			if(_button.state[RT_3D].press==SHORT_PRESS)
			{
				_button.state[RT_3D].press=NO_PRESS;
			   param_value[base_item+ver_item-1]+=1;
			}			
			
			//ͨ���󰴼��̰�����ʵ��ѡ�еĲ������Լ�С50����
			if(_button.state[LT_3D].press==LONG_PRESS)
			{
				_button.state[LT_3D].press=NO_PRESS;
			   param_value[base_item+ver_item-1]-=50;
			}
			
			//ͨ���Ұ����̰�����ʵ��ѡ�еĲ�����������50����
			if(_button.state[RT_3D].press==LONG_PRESS)
			{
				_button.state[RT_3D].press=NO_PRESS;
			   param_value[base_item+ver_item-1]+=50;
			}		

			//ͨ���м䰴����������ʵ�ִ�ҳ���õ����в�������
			if(_button.state[ME_3D].press==LONG_PRESS)
			{
				_button.state[ME_3D].press=NO_PRESS;
				//���º�Բ������б���
				for(uint16_t i=0;i<21;i++)
				{
					WriteFlashParameter(RESERVED_PARAM+base_item+i,param_value[base_item+i]);
				}
			}				
		}
		break;
		case 27:
		{
			uint16_t base_item=50+21*3;
			LCD_clear_L(0,0);		display_6_8_string(0,0,"nav_point:22/28 e-n-u");write_6_8_number(115,0,Page_Number+1);
			LCD_clear_L(0,1);		display_6_8_string(0,1,"p22");   		           write_6_8_number(25,1,param_value[base_item+0]);  write_6_8_number(60,1,param_value[base_item+1]);  write_6_8_number(95,1,param_value[base_item+2]);
			LCD_clear_L(0,2);		display_6_8_string(0,2,"p23");  			         write_6_8_number(25,2,param_value[base_item+3]);  write_6_8_number(60,2,param_value[base_item+4]);  write_6_8_number(95,2,param_value[base_item+5]);
			LCD_clear_L(0,3);		display_6_8_string(0,3,"p24");  			         write_6_8_number(25,3,param_value[base_item+6]);  write_6_8_number(60,3,param_value[base_item+7]);  write_6_8_number(95,3,param_value[base_item+8]);
			LCD_clear_L(0,4);		display_6_8_string(0,4,"p25");				         write_6_8_number(25,4,param_value[base_item+9]);  write_6_8_number(60,4,param_value[base_item+10]); write_6_8_number(95,4,param_value[base_item+11]);
			LCD_clear_L(0,5);		display_6_8_string(0,5,"p26");			           write_6_8_number(25,5,param_value[base_item+12]); write_6_8_number(60,5,param_value[base_item+13]); write_6_8_number(95,5,param_value[base_item+14]);
      LCD_clear_L(0,6);		display_6_8_string(0,6,"p27");				         write_6_8_number(25,6,param_value[base_item+15]); write_6_8_number(60,6,param_value[base_item+16]); write_6_8_number(95,6,param_value[base_item+17]);
      LCD_clear_L(0,7);		display_6_8_string(0,7,"p28");				         write_6_8_number(25,7,param_value[base_item+18]); write_6_8_number(60,7,param_value[base_item+19]); write_6_8_number(95,7,param_value[base_item+20]);
			
			static uint16_t ver_item=1;	
			if(ver_item==1)	  display_6_8_string(18,1,"*"); else if(ver_item==2)	display_6_8_string(53,1,"*"); else if(ver_item==3)	display_6_8_string(88,1,"*");
			if(ver_item==4)  	display_6_8_string(18,2,"*"); else if(ver_item==5)	display_6_8_string(53,2,"*"); else if(ver_item==6)	display_6_8_string(88,2,"*");
			if(ver_item==7)	  display_6_8_string(18,3,"*"); else if(ver_item==8)	display_6_8_string(53,3,"*"); else if(ver_item==9)	display_6_8_string(88,3,"*");
			if(ver_item==10)	display_6_8_string(18,4,"*"); else if(ver_item==11)	display_6_8_string(53,4,"*"); else if(ver_item==12)	display_6_8_string(88,4,"*");
			if(ver_item==13)	display_6_8_string(18,5,"*"); else if(ver_item==14)	display_6_8_string(53,5,"*"); else if(ver_item==15)	display_6_8_string(88,5,"*");
			if(ver_item==16)	display_6_8_string(18,6,"*"); else if(ver_item==17)	display_6_8_string(53,6,"*"); else if(ver_item==18)	display_6_8_string(88,6,"*");
			if(ver_item==19)	display_6_8_string(18,7,"*"); else if(ver_item==20)	display_6_8_string(53,7,"*"); else if(ver_item==21)	display_6_8_string(88,7,"*");
			
			//ͨ��3D������ʵ�ֻ���ѡ�д��޸Ĳ���
			if(_button.state[UP_3D].press==SHORT_PRESS)
			{
					_button.state[UP_3D].press=NO_PRESS;
					ver_item--;
					if(ver_item<1) ver_item=21;
					Bling_Set(&rgb_blue,500,50,0.2,0,GPIO_PORTF_BASE,GPIO_PIN_2,0);//��ɫ				
			}
			if(_button.state[DN_3D].press==SHORT_PRESS)
			{
					_button.state[DN_3D].press=NO_PRESS;
					ver_item++;
					if(ver_item>21) ver_item=1;
					Bling_Set(&rgb_blue,500,50,0.2,0,GPIO_PORTF_BASE,GPIO_PIN_2,0);//��ɫ				
			}
			
			//ͨ���󰴼��̰�����ʵ��ѡ�еĲ������Լ�С1����
			if(_button.state[LT_3D].press==SHORT_PRESS)
			{
				_button.state[LT_3D].press=NO_PRESS;
			   param_value[base_item+ver_item-1]-=1;
			}
			
			//ͨ���Ұ����̰�����ʵ��ѡ�еĲ�����������1����
			if(_button.state[RT_3D].press==SHORT_PRESS)
			{
				_button.state[RT_3D].press=NO_PRESS;
			   param_value[base_item+ver_item-1]+=1;
			}			
			
			//ͨ���󰴼��̰�����ʵ��ѡ�еĲ������Լ�С50����
			if(_button.state[LT_3D].press==LONG_PRESS)
			{
				_button.state[LT_3D].press=NO_PRESS;
			   param_value[base_item+ver_item-1]-=50;
			}
			
			//ͨ���Ұ����̰�����ʵ��ѡ�еĲ�����������50����
			if(_button.state[RT_3D].press==LONG_PRESS)
			{
				_button.state[RT_3D].press=NO_PRESS;
			   param_value[base_item+ver_item-1]+=50;
			}		

			//ͨ���м䰴����������ʵ�ִ�ҳ���õ����в�������
			if(_button.state[ME_3D].press==LONG_PRESS)
			{
				_button.state[ME_3D].press=NO_PRESS;
				//���º�Բ������б���
				for(uint16_t i=0;i<21;i++)
				{
					WriteFlashParameter(RESERVED_PARAM+base_item+i,param_value[base_item+i]);
				}
			}	
		}
		break;
		case 28:
		{
			uint16_t base_item=50+21*4;
			LCD_clear_L(0,0);		display_6_8_string(0,0,"nav_point:29/35 e-n-u");write_6_8_number(115,0,Page_Number+1);
			LCD_clear_L(0,1);		display_6_8_string(0,1,"p29");   		           write_6_8_number(25,1,param_value[base_item+0]);  write_6_8_number(60,1,param_value[base_item+1]);  write_6_8_number(95,1,param_value[base_item+2]);
			LCD_clear_L(0,2);		display_6_8_string(0,2,"p30");  			         write_6_8_number(25,2,param_value[base_item+3]);  write_6_8_number(60,2,param_value[base_item+4]);  write_6_8_number(95,2,param_value[base_item+5]);
			LCD_clear_L(0,3);		display_6_8_string(0,3,"p31");  			         write_6_8_number(25,3,param_value[base_item+6]);  write_6_8_number(60,3,param_value[base_item+7]);  write_6_8_number(95,3,param_value[base_item+8]);
			LCD_clear_L(0,4);		display_6_8_string(0,4,"p32");				         write_6_8_number(25,4,param_value[base_item+9]);  write_6_8_number(60,4,param_value[base_item+10]); write_6_8_number(95,4,param_value[base_item+11]);
			LCD_clear_L(0,5);		display_6_8_string(0,5,"p33");			           write_6_8_number(25,5,param_value[base_item+12]); write_6_8_number(60,5,param_value[base_item+13]); write_6_8_number(95,5,param_value[base_item+14]);
      LCD_clear_L(0,6);		display_6_8_string(0,6,"p34");				         write_6_8_number(25,6,param_value[base_item+15]); write_6_8_number(60,6,param_value[base_item+16]); write_6_8_number(95,6,param_value[base_item+17]);
      LCD_clear_L(0,7);		display_6_8_string(0,7,"p35");				         write_6_8_number(25,7,param_value[base_item+18]); write_6_8_number(60,7,param_value[base_item+19]); write_6_8_number(95,7,param_value[base_item+20]);
			
			static uint16_t ver_item=1;	
			if(ver_item==1)	  display_6_8_string(18,1,"*"); else if(ver_item==2)	display_6_8_string(53,1,"*"); else if(ver_item==3)	display_6_8_string(88,1,"*");
			if(ver_item==4)  	display_6_8_string(18,2,"*"); else if(ver_item==5)	display_6_8_string(53,2,"*"); else if(ver_item==6)	display_6_8_string(88,2,"*");
			if(ver_item==7)	  display_6_8_string(18,3,"*"); else if(ver_item==8)	display_6_8_string(53,3,"*"); else if(ver_item==9)	display_6_8_string(88,3,"*");
			if(ver_item==10)	display_6_8_string(18,4,"*"); else if(ver_item==11)	display_6_8_string(53,4,"*"); else if(ver_item==12)	display_6_8_string(88,4,"*");
			if(ver_item==13)	display_6_8_string(18,5,"*"); else if(ver_item==14)	display_6_8_string(53,5,"*"); else if(ver_item==15)	display_6_8_string(88,5,"*");
			if(ver_item==16)	display_6_8_string(18,6,"*"); else if(ver_item==17)	display_6_8_string(53,6,"*"); else if(ver_item==18)	display_6_8_string(88,6,"*");
			if(ver_item==19)	display_6_8_string(18,7,"*"); else if(ver_item==20)	display_6_8_string(53,7,"*"); else if(ver_item==21)	display_6_8_string(88,7,"*");
			
			//ͨ��3D������ʵ�ֻ���ѡ�д��޸Ĳ���
			if(_button.state[UP_3D].press==SHORT_PRESS)
			{
					_button.state[UP_3D].press=NO_PRESS;
					ver_item--;
					if(ver_item<1) ver_item=21;
					Bling_Set(&rgb_blue,500,50,0.2,0,GPIO_PORTF_BASE,GPIO_PIN_2,0);//��ɫ				
			}
			if(_button.state[DN_3D].press==SHORT_PRESS)
			{
					_button.state[DN_3D].press=NO_PRESS;
					ver_item++;
					if(ver_item>21) ver_item=1;
					Bling_Set(&rgb_blue,500,50,0.2,0,GPIO_PORTF_BASE,GPIO_PIN_2,0);//��ɫ				
			}
			
			//ͨ���󰴼��̰�����ʵ��ѡ�еĲ������Լ�С1����
			if(_button.state[LT_3D].press==SHORT_PRESS)
			{
				_button.state[LT_3D].press=NO_PRESS;
			   param_value[base_item+ver_item-1]-=1;
			}
			
			//ͨ���Ұ����̰�����ʵ��ѡ�еĲ�����������1����
			if(_button.state[RT_3D].press==SHORT_PRESS)
			{
				_button.state[RT_3D].press=NO_PRESS;
			   param_value[base_item+ver_item-1]+=1;
			}			
			
			//ͨ���󰴼��̰�����ʵ��ѡ�еĲ������Լ�С50����
			if(_button.state[LT_3D].press==LONG_PRESS)
			{
				_button.state[LT_3D].press=NO_PRESS;
			   param_value[base_item+ver_item-1]-=50;
			}
			
			//ͨ���Ұ����̰�����ʵ��ѡ�еĲ�����������50����
			if(_button.state[RT_3D].press==LONG_PRESS)
			{
				_button.state[RT_3D].press=NO_PRESS;
			   param_value[base_item+ver_item-1]+=50;
			}		

			//ͨ���м䰴����������ʵ�ִ�ҳ���õ����в�������
			if(_button.state[ME_3D].press==LONG_PRESS)
			{
				_button.state[ME_3D].press=NO_PRESS;
				//���º�Բ������б���
				for(uint16_t i=0;i<21;i++)
				{
					WriteFlashParameter(RESERVED_PARAM+base_item+i,param_value[base_item+i]);
				}
			}	
		}
		break;
		case 29:
		{
			LCD_clear_L(0,0); display_6_8_string(0,0,"tofsense_m");       write_6_8_number_f1(60,0,GD_Distance*10);  write_6_8_number(115,0,Page_Number+1);
			LCD_clear_L(0,1);	write_6_8_number(0,1,tofsense_m[0].dis_mm); write_6_8_number(30,1,tofsense_m[1].dis_mm);  write_6_8_number(60,1,tofsense_m[2].dis_mm);  write_6_8_number(90,1,tofsense_m[3].dis_mm);   
			LCD_clear_L(0,2); write_6_8_number(0,2,tofsense_m[4].dis_mm); write_6_8_number(30,2,tofsense_m[5].dis_mm);  write_6_8_number(60,2,tofsense_m[6].dis_mm);  write_6_8_number(90,2,tofsense_m[8].dis_mm); 
			LCD_clear_L(0,3);	write_6_8_number(0,3,tofsense_m[8].dis_mm); write_6_8_number(30,3,tofsense_m[9].dis_mm);  write_6_8_number(60,3,tofsense_m[10].dis_mm); write_6_8_number(90,3,tofsense_m[12].dis_mm); 
			LCD_clear_L(0,4); write_6_8_number(0,4,tofsense_m[12].dis_mm);write_6_8_number(30,4,tofsense_m[13].dis_mm); write_6_8_number(60,4,tofsense_m[14].dis_mm); write_6_8_number(90,4,tofsense_m[15].dis_mm); 
		}
		break;
		case 30:
		{
			static uint8_t factory_reset_flag=0;
			LCD_clear_L(0,0);		display_6_8_string(25,0,"setup");								  write_6_8_number(70,0,PPM_Databuf[2]); write_6_8_number(105,0,Page_Number+1);
			LCD_clear_L(0,1);		display_6_8_string(0,1,"adckey1_ena");   		      write_6_8_number(90,1,adc_key1_ctrl_enable);
			LCD_clear_L(0,2);		display_6_8_string(0,2,"adckey2_ena");  			    write_6_8_number(90,2,adc_key2_ctrl_enable);
			LCD_clear_L(0,3);		display_6_8_string(0,3,"wireless_adc");  			    write_6_8_number(90,3,wireless_adc_key_ctrl_enable);
			LCD_clear_L(0,4);		display_6_8_string(0,4,"p2_pinch_us");				    write_6_8_number(90,4,pinch_pwm_us);
			LCD_clear_L(0,5);		display_6_8_string(0,5,"p2_release_us");			    write_6_8_number(90,5,release_pwm_us);
      LCD_clear_L(0,6);		display_6_8_string(0,6,"nouse");				          write_6_8_number(90,6,0);		
      LCD_clear_L(0,7);		
			if(factory_reset_flag==0) display_6_8_string(0,7,"factory reset");				  
			else                                               
			{
				static uint8_t _cnt=0;_cnt++;
				if(_cnt<50) display_6_8_string(0,7,"reset is ok");
				else if(_cnt<100)
				{
					display_6_8_string(0,7,"manual reboot");
				}
				else _cnt=0;
			}
			write_6_8_number(90,7,factory_reset_flag);		
			
			static uint16_t ver_item=1;
			display_6_8_string(80,ver_item,"*");
			if(ver_item==4||ver_item==5)
			{
				Reserved_PWM2_Output(PPM_Databuf[2]);
			}
			
			//ͨ��3D������ʵ�ֻ���ѡ�д��޸Ĳ���
			if(_button.state[UP].press==LONG_PRESS)
			{
					_button.state[UP].press=NO_PRESS;
					switch(ver_item)
					{
						case 1:
						{
							if(adc_key1_ctrl_enable==0) adc_key1_ctrl_enable=1;
							else adc_key1_ctrl_enable=0;
							//���º�Բ������б���
							WriteFlashParameter(ADC_KEY1_ENABLE,adc_key1_ctrl_enable);//����adc����1ʹ��
						}
						break;
						case 2:
						{
							if(adc_key2_ctrl_enable==0) adc_key2_ctrl_enable=1;
							else adc_key2_ctrl_enable=0;
							//���º�Բ������б���
							WriteFlashParameter(ADC_KEY2_ENABLE,adc_key2_ctrl_enable);//����adc����2ʹ��
						}
						break;
						case 3:
						{
							if(wireless_adc_key_ctrl_enable==0) wireless_adc_key_ctrl_enable=1;
							else wireless_adc_key_ctrl_enable=0;
							//���º�Բ������б���
							WriteFlashParameter(WIRELESS_ADC_KEY_ENABLE,wireless_adc_key_ctrl_enable);//����adc����2ʹ��
						}
						break;
						case 4:
						{
							pinch_pwm_us=PPM_Databuf[2];
							//���º�Բ������б���
							WriteFlashParameter(RES_SERVO_PINCH_PWM_US,pinch_pwm_us);//��ס����pwm
						}
						break;	
						case 5:
						{
							release_pwm_us=PPM_Databuf[2];
							//���º�Բ������б���
							WriteFlashParameter(RES_SERVO_RELEASE_PWM_US,release_pwm_us);//�ͷŻ���pwm
						}
						break;
						case 6:
						{
							;
						}
						break;
					  case 7://�ָ���������
						{
							factory_reset_flag=1;
							Resume_Factory_Setting();
							NCLink_Send_Check_Flag[11]=1;
						}
						break;						
					}
			}
			
			//������һҳʵ�ֻ���
			if(_button.state[DOWN].press==LONG_PRESS)
			{
					_button.state[DOWN].press=NO_PRESS;
					ver_item++;
					if(ver_item>7) ver_item=1;
					Bling_Set(&rgb_blue,500,50,0.2,0,GPIO_PORTF_BASE,GPIO_PIN_2,0);//��ɫ				
			}
			
		}
		break;	
		case 31:
		{
			LCD_clear_L(0,0);		display_6_8_string(0,0,"imu_filter_setup");		    write_6_8_number(105,0,Page_Number+1);
			LCD_clear_L(0,1);		display_6_8_string(0,1,"player:");   	      			write_6_8_number(105,1,WP_AHRS.player_level);
			if(WP_AHRS.player_level==0) 		 display_6_8_string(40,1,"general");//һ�����
			else if(WP_AHRS.player_level==1) display_6_8_string(40,1,"expert");//רҵ���
			else 														 display_6_8_string(40,1,"hardcore"); //Ӳ�����
			
			LCD_clear_L(0,2);		display_6_8_string(0,2,"ahrs_gyro_lpf:");   		  write_6_8_number(105,2,gyro_lpf_param.cf);
			LCD_clear_L(0,3);		display_6_8_string(0,3,"ahrs_accel_lpf:");  			write_6_8_number(105,3,accel_lpf_param.cf);
			LCD_clear_L(0,4);		display_6_8_string(0,4,"ins_accel_lpf:");  			  write_6_8_number(105,4,ins_lpf_param.cf);
			LCD_clear_L(0,5);		display_6_8_string(0,5,"fb_accel_lpf:");				  write_6_8_number(105,5,accel_fb_lpf_param.cf);
			
			static uint16_t ver_item=1;
			display_6_8_string(98,ver_item,"*");
			//ͨ��3D������ʵ�ֻ���ѡ�д��޸Ĳ���
			if(_button.state[UP_3D].press==SHORT_PRESS)
			{
					_button.state[UP_3D].press=NO_PRESS;
					ver_item--;
					if(ver_item<1) ver_item=5;
					Bling_Set(&Light_Blue,500,50,0.2,0,GPIO_PORTF_BASE,BLING_B_PIN,0);//��ɫ				
			} 
			if(_button.state[DN_3D].press==SHORT_PRESS)
			{
					_button.state[DN_3D].press=NO_PRESS;
					ver_item++;
					if(ver_item>5) ver_item=1;
					Bling_Set(&Light_Blue,500,50,0.2,0,GPIO_PORTF_BASE,BLING_B_PIN,0);//��ɫ				
			}

			//ͨ��3D�����а��������ָ�Ĭ�ϲ���
			if(_button.state[ME_3D].press==LONG_PRESS)
			{
					_button.state[ME_3D].press=NO_PRESS;
				  if(WP_AHRS.player_level==0)//ͨ�û���
					{
						gyro_lpf_param.cf=gyro_lpf_param_default1;
						accel_lpf_param.cf=accel_lpf_param_default1;
						ins_lpf_param.cf=ins_lpf_param_default1;
						accel_fb_lpf_param.cf=accel_fb_lpf_param_default1;
					}
					else if(WP_AHRS.player_level==1)
				  {
						gyro_lpf_param.cf=gyro_lpf_param_default2;
						accel_lpf_param.cf=accel_lpf_param_default2;
						ins_lpf_param.cf=ins_lpf_param_default2;
						accel_fb_lpf_param.cf=accel_fb_lpf_param_default2;
					}
					else
					{
						gyro_lpf_param.cf=gyro_lpf_param_default1;
						accel_lpf_param.cf=accel_lpf_param_default1;
						ins_lpf_param.cf=ins_lpf_param_default1;
						accel_fb_lpf_param.cf=accel_fb_lpf_param_default1;					
					}
					WriteFlashParameter(GYRO_LPF_CF ,gyro_lpf_param.cf);
					WriteFlashParameter(ACCEL_LPF_CF,accel_lpf_param.cf);
					WriteFlashParameter(INS_LPF_CF  ,ins_lpf_param.cf);
					WriteFlashParameter(FB_LPF_CF   ,accel_fb_lpf_param.cf);					
					Bling_Set(&Light_Blue,500,50,0.2,0,GPIO_PORTF_BASE,BLING_B_PIN,0);//��ɫBLING_B_PIN			
			}
			
			//ͨ���Ұ��������̰�����ʵ��ѡ�еĲ����������ӵ���
			if(_button.state[RT_3D].press==SHORT_PRESS)
			{
				_button.state[RT_3D].press=NO_PRESS;
				switch(ver_item)
				{
					case 1:
					{
						WP_AHRS.player_level++;
						if(WP_AHRS.player_level>4)	WP_AHRS.player_level=0;
						WriteFlashParameter(DRONE_PLAYER_LEVEL,WP_AHRS.player_level);
					}
					break;
					case 2:
					{
						gyro_lpf_param.cf+=1;
						if(gyro_lpf_param.cf>100)	gyro_lpf_param.cf=100;
						WriteFlashParameter(GYRO_LPF_CF ,gyro_lpf_param.cf);
					}
					break;
					case 3:
					{
						accel_lpf_param.cf+=1;
						if(accel_lpf_param.cf>100)	accel_lpf_param.cf=100;
						WriteFlashParameter(ACCEL_LPF_CF,accel_lpf_param.cf);
					}
					break;
					case 4:
					{
						ins_lpf_param.cf+=1;
						if(ins_lpf_param.cf>100)	ins_lpf_param.cf=100;
						WriteFlashParameter(INS_LPF_CF  ,ins_lpf_param.cf);
					}
					break;
					case 5:
					{
						accel_fb_lpf_param.cf+=1;
						if(accel_fb_lpf_param.cf>100)	accel_fb_lpf_param.cf=100;
						WriteFlashParameter(FB_LPF_CF   ,accel_fb_lpf_param.cf);
					}
					break;
				}	
			}			
			//ͨ���󰴼������̰�����ʵ��ѡ�еĲ������Լ�С����
			if(_button.state[LT_3D].press==SHORT_PRESS)
			{
				_button.state[LT_3D].press=NO_PRESS;
				switch(ver_item)
				{
					case 1:
					{
						WP_AHRS.player_level--;
						if(WP_AHRS.player_level<0)	WP_AHRS.player_level=4;
						WriteFlashParameter(DRONE_PLAYER_LEVEL,WP_AHRS.player_level);
					}
					break;
					case 2:
					{
						gyro_lpf_param.cf-=1;
						if(gyro_lpf_param.cf<0)	gyro_lpf_param.cf=0;
						WriteFlashParameter(GYRO_LPF_CF ,gyro_lpf_param.cf);
					}
					break;
					case 3:
					{
						accel_lpf_param.cf-=1;
						if(accel_lpf_param.cf<0)	accel_lpf_param.cf=0;
						WriteFlashParameter(ACCEL_LPF_CF,accel_lpf_param.cf);
					}
					break;
					case 4:
					{
						ins_lpf_param.cf-=1;
						if(ins_lpf_param.cf<0)	ins_lpf_param.cf=0;
						WriteFlashParameter(INS_LPF_CF  ,ins_lpf_param.cf);
					}
					break;
					case 5:
					{
						accel_fb_lpf_param.cf-=1;
						if(accel_fb_lpf_param.cf<0)	accel_fb_lpf_param.cf=0;
						WriteFlashParameter(FB_LPF_CF   ,accel_fb_lpf_param.cf);
					}
					break;
				}	
			}			
		}
		break;
		case 32://��̬���Ʋ�������
		{
			LCD_clear_L(0,0);		display_6_8_string(0,0,"pit_angle_kp:");	write_6_8_number(90,0,Total_Controller.Pitch_Angle_Control.Kp);	
			LCD_clear_L(0,1);		display_6_8_string(0,1,"pit_gyro_kp :");  write_6_8_number(90,1,Total_Controller.Pitch_Gyro_Control.Kp);
			LCD_clear_L(0,2);		display_6_8_string(0,2,"pit_gyro_ki :");  write_6_8_number(90,2,Total_Controller.Pitch_Gyro_Control.Ki);
			LCD_clear_L(0,3);		display_6_8_string(0,3,"pit_gyro_kd :");  write_6_8_number(90,3,Total_Controller.Pitch_Gyro_Control.Kd);
		  LCD_clear_L(0,4);		display_6_8_string(0,4,"yaw_angle_kp:");  write_6_8_number(90,4,Total_Controller.Yaw_Angle_Control.Kp);	
			LCD_clear_L(0,5);		display_6_8_string(0,5,"yaw_gyro_kp :");  write_6_8_number(90,5,Total_Controller.Yaw_Gyro_Control.Kp);
			LCD_clear_L(0,6);		display_6_8_string(0,6,"yaw_gyro_ki :");  write_6_8_number(90,6,Total_Controller.Yaw_Gyro_Control.Ki);
			LCD_clear_L(0,7);		display_6_8_string(0,7,"yaw_gyro_kd :");  write_6_8_number(90,7,Total_Controller.Yaw_Gyro_Control.Kd);
			
			static int16_t ver_item=0;
			display_6_8_string(82,ver_item,"*");
			//ͨ��3D������ʵ�ֻ���ѡ�д��޸Ĳ���
			if(_button.state[UP_3D].press==SHORT_PRESS)
			{
					_button.state[UP_3D].press=NO_PRESS;
					ver_item--;
					if(ver_item<0) ver_item=7;
					Bling_Set(&Light_Blue,500,50,0.2,0,GPIO_PORTF_BASE,BLING_B_PIN,0);//��ɫ				
			} 
			if(_button.state[DN_3D].press==SHORT_PRESS)
			{
					_button.state[DN_3D].press=NO_PRESS;
					ver_item++;
					if(ver_item>7) ver_item=0;
					Bling_Set(&Light_Blue,500,50,0.2,0,GPIO_PORTF_BASE,BLING_B_PIN,0);//��ɫ				
			}
			
			if(_button.state[ME_3D].press==LONG_PRESS)//�м䰴�������ָ�Ĭ�ϲ���
			{
					_button.state[ME_3D].press=NO_PRESS;
					Sort_PID_Flag=3;//����λPID��������д��Flash
					Bling_Set(&Light_Blue,500,50,0.2,0,GPIO_PORTF_BASE,BLING_B_PIN,0);//��ɫ				
			}	

			//ͨ���Ұ��������̰�����ʵ��ѡ�еĲ����������ӵ���
			if(_button.state[RT_3D].press==SHORT_PRESS)
			{
				_button.state[RT_3D].press=NO_PRESS;
				switch(ver_item)
				{
					case 0:
					{
						Total_Controller.Pitch_Angle_Control.Kp+=0.10f;
						Total_Controller.Pitch_Angle_Control.Kp=constrain_float(Total_Controller.Pitch_Angle_Control.Kp,0,100);
						Total_Controller.Roll_Angle_Control.Kp=Total_Controller.Pitch_Angle_Control.Kp;
						
						WriteFlashParameter(PID4_PARAMETER_KP,Total_Controller.Pitch_Angle_Control.Kp);
						WriteFlashParameter(PID5_PARAMETER_KP,Total_Controller.Roll_Angle_Control.Kp);
					}
					break;
					case 1:
					{
						Total_Controller.Pitch_Gyro_Control.Kp+=0.10f;
						Total_Controller.Pitch_Gyro_Control.Kp=constrain_float(Total_Controller.Pitch_Gyro_Control.Kp,0,100);
						Total_Controller.Roll_Gyro_Control.Kp=Total_Controller.Pitch_Gyro_Control.Kp;
						
						WriteFlashParameter(PID1_PARAMETER_KP,Total_Controller.Pitch_Gyro_Control.Kp);
						WriteFlashParameter(PID2_PARAMETER_KP,Total_Controller.Roll_Gyro_Control.Kp);
					}
					break;
					case 2:
					{
						Total_Controller.Pitch_Gyro_Control.Ki+=0.10f;
						Total_Controller.Pitch_Gyro_Control.Ki=constrain_float(Total_Controller.Pitch_Gyro_Control.Ki,0,100);
						Total_Controller.Roll_Gyro_Control.Ki=Total_Controller.Pitch_Gyro_Control.Ki;
						
						WriteFlashParameter(PID1_PARAMETER_KI,Total_Controller.Pitch_Gyro_Control.Ki);
						WriteFlashParameter(PID2_PARAMETER_KI,Total_Controller.Roll_Gyro_Control.Ki);
					}
					break;
					case 3:
					{
						Total_Controller.Pitch_Gyro_Control.Kd+=0.10f;
						Total_Controller.Pitch_Gyro_Control.Kd=constrain_float(Total_Controller.Pitch_Gyro_Control.Kd,0,100);
						Total_Controller.Roll_Gyro_Control.Kd=Total_Controller.Pitch_Gyro_Control.Kd;
						
						WriteFlashParameter(PID1_PARAMETER_KD,Total_Controller.Pitch_Gyro_Control.Kd);
						WriteFlashParameter(PID2_PARAMETER_KD,Total_Controller.Roll_Gyro_Control.Kd);
					}
					break;
					case 4:
					{
						Total_Controller.Yaw_Angle_Control.Kp+=0.10f;
						Total_Controller.Yaw_Angle_Control.Kp=constrain_float(Total_Controller.Yaw_Angle_Control.Kp,0,100);
						WriteFlashParameter(PID6_PARAMETER_KP,Total_Controller.Yaw_Angle_Control.Kp);
					}
					break;
					case 5:
					{
						Total_Controller.Yaw_Gyro_Control.Kp+=0.10f;
						Total_Controller.Yaw_Gyro_Control.Kp=constrain_float(Total_Controller.Yaw_Gyro_Control.Kp,0,100);
						WriteFlashParameter(PID3_PARAMETER_KP,Total_Controller.Yaw_Gyro_Control.Kp);
					}
					break;
					case 6:
					{
						Total_Controller.Yaw_Gyro_Control.Ki+=0.10f;
						Total_Controller.Yaw_Gyro_Control.Ki=constrain_float(Total_Controller.Yaw_Gyro_Control.Ki,0,100);
						WriteFlashParameter(PID3_PARAMETER_KI,Total_Controller.Yaw_Gyro_Control.Ki);
					}
					break;
					case 7:
					{
						Total_Controller.Yaw_Gyro_Control.Kd+=0.10f;
						Total_Controller.Yaw_Gyro_Control.Kd=constrain_float(Total_Controller.Yaw_Gyro_Control.Kd,0,100);
						WriteFlashParameter(PID3_PARAMETER_KD,Total_Controller.Yaw_Gyro_Control.Kd);
					}
					break;
				}	
			}	
			//ͨ���󰴼������̰�����ʵ��ѡ�еĲ������Լ�С����
			if(_button.state[LT_3D].press==SHORT_PRESS)
			{
				_button.state[LT_3D].press=NO_PRESS;
				switch(ver_item)
				{
					case 0:
					{
						Total_Controller.Pitch_Angle_Control.Kp-=0.10f;
						Total_Controller.Pitch_Angle_Control.Kp=constrain_float(Total_Controller.Pitch_Angle_Control.Kp,0,100);
						Total_Controller.Roll_Angle_Control.Kp=Total_Controller.Pitch_Angle_Control.Kp;
						
						WriteFlashParameter(PID4_PARAMETER_KP,Total_Controller.Pitch_Angle_Control.Kp);
						WriteFlashParameter(PID5_PARAMETER_KP,Total_Controller.Roll_Angle_Control.Kp);
					}
					break;
					case 1:
					{
						Total_Controller.Pitch_Gyro_Control.Kp-=0.10f;
						Total_Controller.Pitch_Gyro_Control.Kp=constrain_float(Total_Controller.Pitch_Gyro_Control.Kp,0,100);
						Total_Controller.Roll_Gyro_Control.Kp=Total_Controller.Pitch_Gyro_Control.Kp;
						
						WriteFlashParameter(PID1_PARAMETER_KP,Total_Controller.Pitch_Gyro_Control.Kp);
						WriteFlashParameter(PID2_PARAMETER_KP,Total_Controller.Roll_Gyro_Control.Kp);
					}
					break;
					case 2:
					{
						Total_Controller.Pitch_Gyro_Control.Ki-=0.10f;
						Total_Controller.Pitch_Gyro_Control.Ki=constrain_float(Total_Controller.Pitch_Gyro_Control.Ki,0,100);
						Total_Controller.Roll_Gyro_Control.Ki=Total_Controller.Pitch_Gyro_Control.Ki;
						
						WriteFlashParameter(PID1_PARAMETER_KI,Total_Controller.Pitch_Gyro_Control.Ki);
						WriteFlashParameter(PID2_PARAMETER_KI,Total_Controller.Roll_Gyro_Control.Ki);
					}
					break;
					case 3:
					{
						Total_Controller.Pitch_Gyro_Control.Kd-=0.10f;
						Total_Controller.Pitch_Gyro_Control.Kd=constrain_float(Total_Controller.Pitch_Gyro_Control.Kd,0,100);
						Total_Controller.Roll_Gyro_Control.Kd=Total_Controller.Pitch_Gyro_Control.Kd;
						
						WriteFlashParameter(PID1_PARAMETER_KD,Total_Controller.Pitch_Gyro_Control.Kd);
						WriteFlashParameter(PID2_PARAMETER_KD,Total_Controller.Roll_Gyro_Control.Kd);
					}
					break;
					case 4:
					{
						Total_Controller.Yaw_Angle_Control.Kp-=0.10f;
						Total_Controller.Yaw_Angle_Control.Kp=constrain_float(Total_Controller.Yaw_Angle_Control.Kp,0,100);
						WriteFlashParameter(PID6_PARAMETER_KP,Total_Controller.Yaw_Angle_Control.Kp);
					}
					break;
					case 5:
					{
						Total_Controller.Yaw_Gyro_Control.Kp-=0.10f;
						Total_Controller.Yaw_Gyro_Control.Kp=constrain_float(Total_Controller.Yaw_Gyro_Control.Kp,0,100);
						WriteFlashParameter(PID3_PARAMETER_KP,Total_Controller.Yaw_Gyro_Control.Kp);
					}
					break;
					case 6:
					{
						Total_Controller.Yaw_Gyro_Control.Ki-=0.10f;
						Total_Controller.Yaw_Gyro_Control.Ki=constrain_float(Total_Controller.Yaw_Gyro_Control.Ki,0,100);
						WriteFlashParameter(PID3_PARAMETER_KI,Total_Controller.Yaw_Gyro_Control.Ki);
					}
					break;
					case 7:
					{
						Total_Controller.Yaw_Gyro_Control.Kd-=0.10f;
						Total_Controller.Yaw_Gyro_Control.Kd=constrain_float(Total_Controller.Yaw_Gyro_Control.Kd,0,100);
						WriteFlashParameter(PID3_PARAMETER_KD,Total_Controller.Yaw_Gyro_Control.Kd);
					}
					break;
				}	
			}			
		}
		break;
		case 33://λ�ÿ��Ʋ�������
		{
			LCD_clear_L(0,0);		display_6_8_string(0,0,"opt_pos_kp:");	 write_6_8_number(90,0,Total_Controller.Optical_Position_Control.Kp);	
			LCD_clear_L(0,1);		display_6_8_string(0,1,"opt_vel_kp :");  write_6_8_number(90,1,Total_Controller.Optical_Speed_Control.Kp);
			LCD_clear_L(0,2);		display_6_8_string(0,2,"opt_vel_ki :");  write_6_8_number(90,2,Total_Controller.Optical_Speed_Control.Ki);
			LCD_clear_L(0,3);		display_6_8_string(0,3,"sdk_pos_kp :");  write_6_8_number(90,3,Total_Controller.SDK_Roll_Position_Control.Kp);
			LCD_clear_L(0,4);		display_6_8_string(0,4,"sdk_pos_kd :");  write_6_8_number(90,4,Total_Controller.SDK_Roll_Position_Control.Kd);
			LCD_clear_L(0,5);
			LCD_clear_L(0,6);
			LCD_clear_L(0,7);
			static int16_t ver_item=0;
			display_6_8_string(82,ver_item,"*");
			//ͨ��3D������ʵ�ֻ���ѡ�д��޸Ĳ���
			if(_button.state[UP_3D].press==SHORT_PRESS)
			{
					_button.state[UP_3D].press=NO_PRESS;
					ver_item--;
					if(ver_item<0) ver_item=4;
					Bling_Set(&Light_Blue,500,50,0.2,0,GPIO_PORTF_BASE,BLING_B_PIN,0);//��ɫ				
			} 
			if(_button.state[DN_3D].press==SHORT_PRESS)
			{
					_button.state[DN_3D].press=NO_PRESS;
					ver_item++;
					if(ver_item>4) ver_item=0;
					Bling_Set(&Light_Blue,500,50,0.2,0,GPIO_PORTF_BASE,BLING_B_PIN,0);//��ɫ				
			} 
			
			if(_button.state[ME_3D].press==LONG_PRESS)//�м䰴�������ָ�Ĭ�ϲ���
			{
					_button.state[ME_3D].press=NO_PRESS;
					Bling_Set(&Light_Blue,500,50,0.2,0,GPIO_PORTF_BASE,BLING_B_PIN,0);//��ɫ

					PID_Init(&Total_Controller.Optical_Position_Control,Optical_Position_Controler);
					PID_Init(&Total_Controller.Optical_Speed_Control,Optical_Speed_Controler);
					PID_Init(&Total_Controller.SDK_Roll_Position_Control,SDK_Roll_Position_Controler);
					PID_Init(&Total_Controller.SDK_Pitch_Position_Control,SDK_Roll_Position_Controler);	

					WriteFlashParameter(PID12_PARAMETER_KP,Total_Controller.Optical_Position_Control.Kp);
					WriteFlashParameter(PID13_PARAMETER_KP,Total_Controller.Optical_Speed_Control.Kp);
					WriteFlashParameter(PID13_PARAMETER_KI,Total_Controller.Optical_Speed_Control.Ki);
					WriteFlashParameter(PID14_PARAMETER_KP,Total_Controller.SDK_Roll_Position_Control.Kp);
					WriteFlashParameter(PID14_PARAMETER_KD,Total_Controller.SDK_Roll_Position_Control.Kd);				
			}
			
			//ͨ���Ұ��������̰�����ʵ��ѡ�еĲ����������ӵ���
			if(_button.state[RT_3D].press==SHORT_PRESS)
			{
				_button.state[RT_3D].press=NO_PRESS;
				switch(ver_item)
				{
					case 0:
					{
						Total_Controller.Optical_Position_Control.Kp+=0.01f;
						Total_Controller.Optical_Position_Control.Kp=constrain_float(Total_Controller.Optical_Position_Control.Kp,0,100);
						WriteFlashParameter(PID12_PARAMETER_KP,Total_Controller.Optical_Position_Control.Kp);
					}
					break;
					case 1:
					{
						Total_Controller.Optical_Speed_Control.Kp+=0.1f;
						Total_Controller.Optical_Speed_Control.Kp=constrain_float(Total_Controller.Optical_Speed_Control.Kp,0,100);
						WriteFlashParameter(PID13_PARAMETER_KP,Total_Controller.Optical_Speed_Control.Kp);
					}
					break;
					case 2:
					{
						Total_Controller.Optical_Speed_Control.Ki+=0.001f;
						Total_Controller.Optical_Speed_Control.Ki=constrain_float(Total_Controller.Optical_Speed_Control.Ki,0,100);
						WriteFlashParameter(PID13_PARAMETER_KI,Total_Controller.Optical_Speed_Control.Ki);
					}
					break;
					case 3:
					{
						Total_Controller.SDK_Roll_Position_Control.Kp+=0.1f;
						Total_Controller.SDK_Roll_Position_Control.Kp=constrain_float(Total_Controller.SDK_Roll_Position_Control.Kp,0,100);
						
						Total_Controller.SDK_Pitch_Position_Control.Kp=Total_Controller.SDK_Roll_Position_Control.Kp;
						WriteFlashParameter(PID14_PARAMETER_KP,Total_Controller.SDK_Roll_Position_Control.Kp);
					}
					break;
					case 4:
					{
						Total_Controller.SDK_Roll_Position_Control.Kd+=0.1f;
						Total_Controller.SDK_Roll_Position_Control.Kd=constrain_float(Total_Controller.SDK_Roll_Position_Control.Kd,0,100);
						
						Total_Controller.SDK_Pitch_Position_Control.Kd=Total_Controller.SDK_Roll_Position_Control.Kd;
						WriteFlashParameter(PID14_PARAMETER_KD,Total_Controller.SDK_Roll_Position_Control.Kd);
					}
					break;
				}	
			}	
			//ͨ���󰴼������̰�����ʵ��ѡ�еĲ������Լ�С����
			if(_button.state[LT_3D].press==SHORT_PRESS)
			{
				_button.state[LT_3D].press=NO_PRESS;
				switch(ver_item)
				{
					case 0:
					{
						Total_Controller.Optical_Position_Control.Kp-=0.01f;
						Total_Controller.Optical_Position_Control.Kp=constrain_float(Total_Controller.Optical_Position_Control.Kp,0,100);
						WriteFlashParameter(PID12_PARAMETER_KP,Total_Controller.Optical_Position_Control.Kp);
					}
					break;
					case 1:
					{
						Total_Controller.Optical_Speed_Control.Kp-=0.1f;
						Total_Controller.Optical_Speed_Control.Kp=constrain_float(Total_Controller.Optical_Speed_Control.Kp,0,100);
						WriteFlashParameter(PID13_PARAMETER_KP,Total_Controller.Optical_Speed_Control.Kp);
					}
					break;
					case 2:
					{
						Total_Controller.Optical_Speed_Control.Ki-=0.001f;
						Total_Controller.Optical_Speed_Control.Ki=constrain_float(Total_Controller.Optical_Speed_Control.Ki,0,100);
						WriteFlashParameter(PID13_PARAMETER_KI,Total_Controller.Optical_Speed_Control.Ki);
					}
					break;
					case 3:
					{
						Total_Controller.SDK_Roll_Position_Control.Kp-=0.1f;
						Total_Controller.SDK_Roll_Position_Control.Kp=constrain_float(Total_Controller.SDK_Roll_Position_Control.Kp,0,100);
						
						Total_Controller.SDK_Pitch_Position_Control.Kp=Total_Controller.SDK_Roll_Position_Control.Kp;
						WriteFlashParameter(PID14_PARAMETER_KP,Total_Controller.SDK_Roll_Position_Control.Kp);
					}
					break;
					case 4:
					{
						Total_Controller.SDK_Roll_Position_Control.Kd-=0.1f;
						Total_Controller.SDK_Roll_Position_Control.Kd=constrain_float(Total_Controller.SDK_Roll_Position_Control.Kd,0,100);
						
						Total_Controller.SDK_Pitch_Position_Control.Kd=Total_Controller.SDK_Roll_Position_Control.Kd;
						WriteFlashParameter(PID14_PARAMETER_KD,Total_Controller.SDK_Roll_Position_Control.Kd);
					}
					break;
				}	
			}			
		}
		break;
		default://�հ�ҳ����ʾ�������û�������
		{
			LCD_clear_L(0,0);		display_6_8_string(0,0,"Reserved Show Page:");write_6_8_number(115,0,Page_Number+1);
			LCD_clear_L(0,2);		display_6_8_string(0,2,"    Don't Worry ");
			LCD_clear_L(0,4);		display_6_8_string(0,4,"  Continue Press To");
			LCD_clear_L(0,6);		display_6_8_string(0,6,"    Page Up/Down");
		}
	}		
}









