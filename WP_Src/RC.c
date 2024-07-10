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
#include "RC.h"

uint16 Throttle_Control=0;
float Pitch_Control=0,Roll_Control=0,Yaw_Control=0;
float Target_Angle[2]={0};
uint16 Last_Throttle_Control,Pre_Last_Throttle_Control;
uint16 Throttle_Base=1000,TempThrottle_Control;
uint16_t Controler_State=0;
uint16_t Auto_ReLock_Cnt=0;//�Զ�����������
uint8_t Auto_Relock_Flag=0;
uint8_t Auto_Relock_Flag_Set=0;
int16 Temp_RC=0;
uint16_t Unlock_Makesure_Cnt=0,Lock_Makesure_Cnt=0;
uint16_t Unwanted_Lock_Flag=0;
uint16_t Forced_Lock_Makesure_Cnt=0;

uint16_t RC_Buttom=1000,RC_Top=2000,RC_Middle=1500,RC_Deadband=100;
uint16_t RC_Deadzone_Buttom=0,RC_Deadzone_Top=0;
lpf_buf RC_LPF_Buffer[8];
uint16_t PPM_LPF_Databuf[8];
rc RC_Data;
int16_t TRPY[4]={0};




int16_t constrain_int16_t(int16_t amt, int16_t low, int16_t high){
  return ((amt)<(low)?(low):((amt)>(high)?(high):(amt)));
}


/***********************************************************
@��������RC_Data_LPF_Filter
@��ڲ�������
@���ڲ�������
����������ң����ԭʼ�����˲�
@���ߣ�����С��
@���ڣ�2024/01/20
*************************************************************/
void RC_Data_LPF_Filter()
{
  uint16_t i=0;
  for(i=0;i<8;i++)//��ң����ǰ�ĸ�ͨ���˲�����
  {
    PPM_LPF_Databuf[i]=(uint16_t)(LPButterworth(PPM_Databuf[i],&RC_LPF_Buffer[i],&Butter_5HZ_Parameter_RC));
  }
}


int16_t Throttle_Rate=0;
/***********************************************************
@��������Get_Thr_Rate
@��ڲ�����float Thr
@���ڲ�������
�����������������ݱ仯��
@���ߣ�����С��
@���ڣ�2024/01/20
*************************************************************/
int16_t Get_Thr_Rate(float Thr)//���ű仯��
{
  static float Thr_Rec[20];
  uint16_t i=0;
  for(i=19;i>0;i--)
  {
    Thr_Rec[i]=Thr_Rec[i-1];
  }
  Thr_Rec[0]=Thr;
  return (int16_t)((Thr_Rec[0]-Thr_Rec[9])/1.0f);
}

/***********************************************************
@��������RC_Scale_Set
@��ڲ�����Vector_RC *rc_date
@���ڲ�������
����������ң����ͨ���г̻�ȡ
@���ߣ�����С��
@���ڣ�2024/01/20
*************************************************************/
void RC_Scale_Set(rc_calibration *rc_date)
{
  RC_Buttom=(uint16_t)(rc_date->min);
  RC_Top=(uint16_t)(rc_date->max);
  RC_Middle=(uint16_t)(rc_date->middle);
  RC_Deadband=(uint16_t)(rc_date->deadband);
  RC_Deadzone_Buttom=RC_Middle-RC_Deadband/2;
  RC_Deadzone_Top=RC_Middle+RC_Deadband/2;
}



uint8_t RC_Read_Switch(uint16_t ch)
{
		uint16_t pulsewidth = ch;
		if (pulsewidth <= 900 || pulsewidth >= 2200) return 255;// This is an error condition
		if (pulsewidth <= 1210) return 0;//��λ
		if (pulsewidth <= 1710) return 1;//��λ
		return 2;//��λ
}



float Remote_Data_Remap(rc *data,uint16_t ch,float max_down,float max_up,bool reverse_flag)
{
  float value=0;
  if(data->rcdata[ch]<=data->cal[ch].middle-0.5f*data->cal[ch].deadband)
  value=(data->cal[ch].middle-0.5f*data->cal[ch].deadband-data->rcdata[ch])*max_down
												  /data->cal[ch].scale;
	else if(data->rcdata[ch]>=data->cal[ch].middle+0.5f*data->cal[ch].deadband)
  value=(data->cal[ch].middle+0.5f*data->cal[ch].deadband-data->rcdata[ch])*max_up
												  /data->cal[ch].scale;	
  else value=0;
	
	if(reverse_flag)  value*=(-1);
  return 	value;
}


/***********************************************************
@��������Remote_Control
@��ڲ�������
@���ڲ�������
����������ң��������ת��
@���ߣ�����С��
@���ڣ�2024/01/20
*************************************************************/
void Remote_Control(void )
{
	static uint8_t _cnt=0;
	_cnt++;
	if(_cnt<4) return ;
	_cnt=0;
	if(adc_ppm_update_flag==1)
	{
		memcpy(PPM_Databuf,ADC_PPM_Databuf,10*sizeof(uint16));
	}
	else if(ppm_update_flag==1)//����ѡ���ⲿ���ջ������PPM�ź�
	{
	  memcpy(PPM_Databuf,Receiver_PPM_Databuf,10*sizeof(uint16));
	}
	else if(rc_update_flag==1)
	{
	  memcpy(PPM_Databuf,NCLINK_PPM_Databuf,10*sizeof(uint16));
	}
	
	
	if(ppm_update_flag==1||rc_update_flag==1||adc_ppm_update_flag==1)
	{
			memcpy(RC_Data.rcdata,PPM_Databuf,RC_CHL_MAX*sizeof(uint16_t));//����ÿ��ͨ��ң��������
		
			RC_Data.rc_rpyt[RC_ROLL]	=Remote_Data_Remap(&RC_Data ,RC_ROLL_CHANNEL	,Pit_Rol_Max,Pit_Rol_Max,true);
			RC_Data.rc_rpyt[RC_PITCH] =Remote_Data_Remap(&RC_Data ,RC_PITCH_CHANNEL ,Pit_Rol_Max,Pit_Rol_Max,true);
			RC_Data.rc_rpyt[RC_YAW]		=Remote_Data_Remap(&RC_Data ,RC_YAW_CHANNEL	  ,Yaw_Max		,Yaw_Max    ,false);
			RC_Data.rc_rpyt[RC_THR]		=Remote_Data_Remap(&RC_Data ,RC_THR_CHANNEL	  ,Max_Downvel,Max_Upvel,true);
		
		
			RC_Data.thr=RC_Data.rcdata[RC_THR_CHANNEL];
			RC_Data.aux[AUX1]=RC_Data.rcdata[RC_AUX1_CHANNEL];
			RC_Data.aux[AUX2]=RC_Data.rcdata[RC_AUX2_CHANNEL];
			RC_Data.aux[AUX3]=RC_Data.rcdata[RC_AUX3_CHANNEL];
			RC_Data.aux[AUX4]=RC_Data.rcdata[RC_AUX4_CHANNEL];

			RC_Data.last_thr_mode=RC_Data.thr_mode;			
			switch(RC_Read_Switch(RC_Data.aux[AUX1]))
			{
				case 0:RC_Data.thr_mode=HAND_THR;break;//�ֶ���������
				case 1:
				case 2:RC_Data.thr_mode=AUTO_THR;break;//�Զ���������
				default:RC_Data.thr_mode=HAND_THR;
			}
			

	
			rc_update_flag=0;
			ppm_update_flag=0;
			RC_Data_LPF_Filter();
			RC_Scale_Set(&RC_Data.cal[0]);
			if(PPM_Databuf[0]<=RC_Deadzone_Buttom)  Roll_Control=(RC_Deadzone_Buttom-PPM_LPF_Databuf[0])*Pit_Rol_Max/(RC_Deadzone_Buttom-RC_Buttom);//����г̿�����+-45
			else if(PPM_Databuf[0]>=RC_Deadzone_Top)  Roll_Control=(RC_Deadzone_Top-PPM_LPF_Databuf[0])*Pit_Rol_Max/(RC_Top-RC_Deadzone_Top);
			else Roll_Control=0;
			Roll_Control=constrain_float(Roll_Control,-Pit_Rol_Max,Pit_Rol_Max);
			
			RC_Scale_Set(&RC_Data.cal[1]);
			if(PPM_Databuf[1]<=RC_Deadzone_Buttom)  Pitch_Control=(RC_Deadzone_Buttom-PPM_LPF_Databuf[1])*Pit_Rol_Max/(RC_Deadzone_Buttom-RC_Buttom);//����г̿�����+-45
			else if(PPM_Databuf[1]>=RC_Deadzone_Top)  Pitch_Control=(RC_Deadzone_Top-PPM_LPF_Databuf[1])*Pit_Rol_Max/(RC_Top-RC_Deadzone_Top);
			else Pitch_Control=0;
			Pitch_Control=constrain_float(Pitch_Control,-Pit_Rol_Max,Pit_Rol_Max);
			
			Target_Angle[0]=-Pitch_Control;//����ʱ������������
			Target_Angle[1]=-Roll_Control;//����ʱ�����������
			
			RC_Scale_Set(&RC_Data.cal[2]);
			int16_t _thr=0;
			_thr=1000*(PPM_LPF_Databuf[2]-RC_Buttom)/(RC_Top-RC_Buttom);//Ϊ�˰�ȫ�����Ÿ˵�λ����ΪButtom_Safe_Deadband
			Throttle_Control=constrain_int16_t(_thr,0,1000);
			TRPY[0]=Throttle_Control;//ң������ԭʼ�г���
			
			
			RC_Scale_Set(&RC_Data.cal[3]);
			if(PPM_Databuf[3]<=RC_Deadzone_Buttom)  Yaw_Control=-(PPM_LPF_Databuf[3]-RC_Deadzone_Buttom)*Yaw_Max/(RC_Deadzone_Buttom-RC_Buttom);//ƫ������г̿�����+-150
			else if(PPM_Databuf[3]>=RC_Deadzone_Top)  Yaw_Control=-(PPM_LPF_Databuf[3]-RC_Deadzone_Top)*Yaw_Max/(RC_Top-RC_Deadzone_Top);
			else Yaw_Control=0;
			Yaw_Control=constrain_float(Yaw_Control,-Yaw_Max,Yaw_Max);
			
			
			Throttle_Rate=Get_Thr_Rate(Throttle_Control);
			Throttle_Control=Throttle_Base+Throttle_Control;
			/***************************************************************
			��������ʱ��ң�����������������ҡ�˴��ڵ�λ�����ڲ���
			��������ʱ��ң�����������������ҡ�˴��ڵ�λ���������
			***************************************************************/
			if(Throttle_Control<=(1000+Buttom_Safe_Deadband)
			 &&Yaw_Control>=Yaw_Max*Scale_Pecent_Max
			 &&Roll_Control==0&&Pitch_Control==0)
			{
				Unlock_Makesure_Cnt++;
				if(Forced_Lock_Makesure_Cnt<10000) Forced_Lock_Makesure_Cnt++;
			}
			else Unlock_Makesure_Cnt/=2;
			
			
			
			if((Throttle_Control<=(1000+Buttom_Safe_Deadband)
				&&Yaw_Control>=Yaw_Max*Scale_Pecent_Max
				&&Roll_Control==0&&Pitch_Control==0
				&&Unlock_Makesure_Cnt>50*2.0)//����2.0S
				||Forced_Lock_Makesure_Cnt>=50*5
				||unlock_flag==0)//����
			{
				Controler_State=Lock_Controler;
				Unlock_Makesure_Cnt=0;
				Lock_Makesure_Cnt=0;
				Forced_Lock_Makesure_Cnt=0;
				Bling_Set(&rgb_green,3000,200,0.5,0,GPIO_PORTF_BASE,GPIO_PIN_3,0);
				Bling_Mode=0;
				Page_Number=0;//OLED�ָ���ҳ
				Reset_Mag_Calibartion(1);
				Reset_Accel_Calibartion(1);
				Reset_RC_Calibartion(1);
			}
			
			if(Throttle_Control<=(1000+Buttom_Safe_Deadband)
				 &&Yaw_Control<=-Yaw_Max*Scale_Pecent_Max
					 &&Roll_Control==0
						 &&Pitch_Control==0)
				Lock_Makesure_Cnt++;
			else Lock_Makesure_Cnt/=2;
			
			if((Throttle_Control<=(1000+Buttom_Safe_Deadband)
				 &&Yaw_Control<=-Yaw_Max*Scale_Pecent_Max
					 &&Roll_Control==0
						 &&Pitch_Control==0
							 &&Lock_Makesure_Cnt>50*2.0//����2.0S
								 &&Temperature_Stable_Flag==1
									&&Gyro_Safety_Calibration_Flag==1
									&&Check_Calibration_Flag()==0x00)
			            ||unlock_flag==1)//����
			{
				Controler_State=Unlock_Controler;
				Lock_Makesure_Cnt=0;
				Unlock_Makesure_Cnt=0;
				Forced_Lock_Makesure_Cnt=0;
				Bling_Set(&rgb_green,5000,2000,0.1,0,GPIO_PORTF_BASE,GPIO_PIN_3,0);
				Bling_Mode=0;
				Page_Number=0;//OLED�ָ���ҳ
				Reset_Mag_Calibartion(1);
				Reset_Accel_Calibartion(1);
				Reset_RC_Calibartion(1);
				Auto_ReLock_Cnt=50*6;//����6S
				Auto_Relock_Flag_Set=0;
				if(RC_Data.thr_mode==AUTO_THR)//������ڶ���ģʽ�½��� 
				{  
					 Unwanted_Lock_Flag=1;   //����Ҫ�Զ���������ģʽ
					 Total_Controller.Height_Acce_Control.Integrate=-Total_Controller.Height_Acce_Control.Integrate_Max;
				}
				else Unwanted_Lock_Flag=0;
			}
			
		//����֮�����ŵ�λ+ƫ����λ����֮��
		if(Controler_State==Unlock_Controler
		 &&Auto_Relock_Flag_Set==0//�Զ�����λδ����
		 &&Throttle_Control<=(1000+Buttom_Safe_Deadband)
		 &&Pitch_Control==0&&Roll_Control==0&&Yaw_Control==0)//������ң�л���
		{
			Auto_Relock_Flag_Set=1;//���ν�����ֻ��λһ��
			if(Unwanted_Lock_Flag==0)
			{
				Auto_Relock_Flag=1;//������ʼʱ�����Զ�������־λ
			}
		}
					
		if(Auto_Relock_Flag==1)
		{
			if(Throttle_Control<=(1000+Buttom_Safe_Deadband)
			 &&Pitch_Control==0&&Roll_Control==0&&Yaw_Control==0
			 &&Controler_State==Unlock_Controler//�Զ�����������
			 &&Unwanted_Lock_Flag==0)
			{
				Auto_ReLock_Cnt--;
				if(Auto_ReLock_Cnt<=0)  Auto_ReLock_Cnt=0;
				if(Auto_ReLock_Cnt==0)
				{
					Controler_State=Lock_Controler;//�ٴ�����
					Bling_Set(&rgb_green,3000,200,0.5,0,GPIO_PORTF_BASE,GPIO_PIN_3,0);
					Bling_Mode=0;
				}
			}
			
			if((Throttle_Control>(1000+Buttom_Safe_Deadband)
				||Pitch_Control!=0||Roll_Control!=0||Yaw_Control!=0)
				&&Auto_ReLock_Cnt>0//�Զ�����������
				&&Unwanted_Lock_Flag==0)
			{
				Auto_Relock_Flag=0;//ֻҪң���ж��������ν������Զ������Ͳ�����
				Auto_ReLock_Cnt=0;
				Bling_Set(&rgb_green,5000,2000,0.3,0,GPIO_PORTF_BASE,GPIO_PIN_3,0);
				Bling_Mode=0;
			}
		}
	}
	
	if(shutdown_now_cal_flag==1)
	{
		Reset_Mag_Calibartion(1);
		Reset_Accel_Calibartion(1);
		Reset_RC_Calibartion(1);
		shutdown_now_cal_flag=0;
		Bling_Set(&rgb_green,0,1000,0.5,0,GPIO_PORTF_BASE,GPIO_PIN_3,0);
		Page_Number=0;//OLED�ָ���ҳ
	}
}
