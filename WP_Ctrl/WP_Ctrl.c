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
#include "WP_Ctrl.h"
#include "control_config.h"
/*****************ң�����г�����**********************/
int16_t M_PWM_1,M_PWM_2,M_PWM_3,M_PWM_4;//4��������PWM
int16_t _M_PWM_1,_M_PWM_2,_M_PWM_3,_M_PWM_4;//�ϴ�4��������PWM
uint8_t Controler_High_Mode=1,Last_Controler_High_Mode=1;
uint8_t Controler_Horizontal_Mode=1,Last_Controler_Horizontal_Mode=1;
uint8_t Controler_SDK1_Mode=0,Last_Controler_SDK1_Mode=0;
uint8_t Controler_Land_Mode=1,Last_Controler_Land_Mode=1;


uint8_t Controler_GPS_Mode=1,Last_Controler_GPS_Mode=1;
uint8_t Hover_Point_Set_Flag=0;


uint8_t Control_Mode_Change=0;
uint16_t High_Hold_Throttle=0;
uint16_t Throttle=0;
Controller_Output Flight;



typedef struct
{
	uint32_t switch_time;
	uint32_t release_time; 
	uint32_t switch_in_time;
}gps_switch_state;
gps_switch_state gps_switch;


/***************************************************
������: void Controler_Mode_Select(void)
˵��:	������ģʽѡ����
���:	��
����:	��
��ע:	�ж�������ȳ�������
****************************************************/
void Controler_Mode_Select()
{
  Last_Controler_High_Mode=Controler_High_Mode;//�ϴθ߶ȿ���ģʽ
  Last_Controler_Horizontal_Mode=Controler_Horizontal_Mode;//�ϴ�λ�ÿ���ģʽ
  Last_Controler_Land_Mode=Controler_Land_Mode;//�ϴη���ģʽ
	 
  if(PPM_Databuf[4]<=(RC_Data.cal[4].min+RC_Data.cal[4].deadband))  Controler_High_Mode=1;//��λ:����̬����
  else Controler_High_Mode=2;//�ǵ�λ:��ѹ�ơ�����������
		
  if(PPM_Databuf[5]>=(RC_Data.cal[5].max-RC_Data.cal[5].deadband))       
	{
		Controler_SDK1_Mode=1;//SDK1ģʽ
		Flight.roll_pitch_angle_limit_enable=1;
	}
	else//��SDK1ģʽ	
	{
		NCQ_SDK_Reset();
		flight_subtask_reset();
		Controler_SDK1_Mode=0;
		Flight.roll_pitch_angle_limit_enable=0;
	}
		
	uint8_t ch7_value=RC_Read_Switch(RC_Data.aux[AUX3]);
	uint8_t ch8_value=RC_Read_Switch(RC_Data.aux[AUX4]);

	current_state.last_fault=current_state.fault;
	switch(Optical_Type_Present)
	{
		case 1:
		case 2:
		{
				//����ģʽ�£�����ͨ��������λ������������1s�����GPS����ģʽ
				if(ch7_value==0)//ң�������ο��ش��ڵ�λ  
				{
					Controler_GPS_Mode=1;
					gps_switch.release_time=millis();//��λms
					//1������GPS����ģʽ����λ��������0�������㶨������ʱ����GPS��������ڣ��Զ�ˢ�µ�ǰλ��ΪĿ��λ��
					//2���г�GPS����ģʽ����λ��������0��Ϊ�´�����GPS����ģʽ��׼��
					Total_Controller.Latitude_Position_Control.Expect=0;
					Total_Controller.Longitude_Position_Control.Expect=0;
					East_North_Ctrl_Reset();
					Hover_Point_Set_Flag=0;
				}
				else if(ch7_value==1)//ң�������ο��ش�����λ     
				{
					 gps_switch.switch_time=millis();//��λms
					 gps_switch.switch_in_time=(gps_switch.switch_time-gps_switch.release_time);
					 if(gps_switch.switch_in_time>=1000)//��������1S�У�����Ϊ��������
					 {
						 Controler_GPS_Mode=2;
					 }
				} 
				else//��λ 
				{
					Controler_GPS_Mode=1;
					gps_switch.release_time=millis();//��λms
					//1������GPS����ģʽ����λ��������0�������㶨������ʱ����GPS��������ڣ��Զ�ˢ�µ�ǰλ��ΪĿ��λ��
					//2���г�GPS����ģʽ����λ��������0��Ϊ�´�����GPS����ģʽ��׼��
					Total_Controller.Latitude_Position_Control.Expect=0;
					Total_Controller.Longitude_Position_Control.Expect=0;		
					Hover_Point_Set_Flag=0;
				}
							
				if(ch7_value==2) 				  Controler_Land_Mode=2;//��λ��������ģʽ        
				else if(ch7_value==0)   	Controler_Land_Mode=1;//��λ�����Ƿ���ģʽ	
				Last_Controler_GPS_Mode=Controler_GPS_Mode;	
				
				//��Optical_Type_Present��Ϊ3ʱ����ʾ��ʹ��slamλ�ÿ��ƣ���8ͨ��ֱ��ȷ����������ģʽ������ͨ������ʲô״̬�޹���
				switch(ch8_value)
				{
					case 0:
					{
						Controler_Horizontal_Mode=1;//��̬���ȿ���
						//OpticalFlow_SINS_Reset();
						OpticalFlow_Ctrl_Reset();	
					}
					break;
					case 2:
					{
						Controler_Horizontal_Mode=2;//����ˮƽλ�ÿ���
					}
					break;
					default:
					{
						Controler_Horizontal_Mode=1;//��̬���ȿ���
						//OpticalFlow_SINS_Reset();
						OpticalFlow_Ctrl_Reset();						
					}
				}		
		}
		break;	
		case 3://��״̬�²������е�GPSģʽ
		case 4:	
		{
			switch(ch7_value)
			{
				case 0://����7���ڵ�λ
				{
					switch(ch8_value)
					{
						case 0:
						{
							Controler_Horizontal_Mode=1;//��̬���ȿ���
							//OpticalFlow_SINS_Reset();
							OpticalFlow_Ctrl_Reset();	
						}
						break;
						case 2:
						{
							Controler_Horizontal_Mode=2;//����ˮƽλ�ÿ���
						}
						break;
						default:
						{
							Controler_Horizontal_Mode=1;//��̬���ȿ���
							//OpticalFlow_SINS_Reset();
							OpticalFlow_Ctrl_Reset();						
						}
					}	
					Controler_Land_Mode=1;//�Ƿ���ģʽ 
				}
				break;
				case 1://����7������λ
				{
					current_state.fault=1;//��λ�쳣��ң�����ֶ������趨
					Controler_Horizontal_Mode=1;//ǿ����̬���ȿ���
					Controler_Land_Mode=1;//�Ƿ���ģʽ 
				}
				break;
				case 2:
				{
					Controler_Land_Mode=2;//��λ��������ģʽ 
				}
				break;
			}
		}
		break;
		default://����δʵ�����Σ�ͳһǿ���е�ˮƽ��̬����ģʽ
		{
			Controler_Horizontal_Mode=1;//��̬���ȿ���
		}		
	}
		
	
//*****************************************************************************************
  if(Unwanted_Lock_Flag==1)//����ģʽ���������κβ���
  {
    Thr_Push_Over_State=Thr_Push_Over_Deadband();
    if(Thr_Push_Over_State==2)//ֻҪ�����ƹ�����λ���������������Զ���������
    {
      Unwanted_Lock_Flag=0;
    }
    else
    {
      Take_Off_Reset();//�����
      //Throttle_Control_Reset();//�����
    }
  }
  
  if(Controler_Land_Mode!=Last_Controler_Land_Mode)
  {
		Total_Controller.Height_Position_Control.Expect=NamelessQuad.Position[_UP];//�����ز���˲��Ĺߵ��߶�����Ϊ�����߶�
    if(Controler_Land_Mode==1)   land_reset();//����ģʽ�л�����ģʽ 
  }
  
  if(Controler_High_Mode!=Last_Controler_High_Mode)
  {
    if(Controler_High_Mode==2)  {Control_Mode_Change=1;}//�����ж��ߣ����û�׼����ֵ����ͣ�߶�
    if(Controler_High_Mode==1)  {Control_Mode_Change=1;}//����������
  }
  
  if(Controler_Horizontal_Mode!=Last_Controler_Horizontal_Mode)	Control_Mode_Change=2;//λ��ͨ�����л�
  
  
  if(Control_Mode_Change==1)//���ڶ���ģʽ�л����߶�ֻ����һ��
  {
    if(Controler_High_Mode==High_Hold_Mode)//����Ϊ����ģʽ���������ж���
    {
      High_Hold_Throttle=Throttle_Control;//���浱ǰ����ֵ��ֻ��һ��
      /*******************����ǰ�ߵ���ֱλ�ù�����ΪĿ��߶�***************************/
      Total_Controller.Height_Position_Control.Expect=NamelessQuad.Position[_UP];//�����ز���˲��Ĺߵ��߶�����Ϊ�����߶�
    }
    else//����Ϊ����ģʽ��������������
    {
      //Throttle_Control_Reset();
    }
    Control_Mode_Change=0;//��ģʽ�л�λ��0,���ҽ�����һ��
  }
  else if(Control_Mode_Change==2)//���ڶ���ģʽ�л�����ͣλ��ֻ����һ��
  {
    Control_Mode_Change=0;//����Ӧ���ζ��㵵λ�л�
  }

//*****************************************************************************************
	if(Controler_GPS_Mode==2&&Hover_Point_Set_Flag==0)//GPS��λ�£���ͣ��δ����
	{
    if(GPS_ok()==TRUE)//�״��ж��㲻���㶨��������֮�������㶨������
    {
			/*******************����ǰ�ߵ�ˮƽλ�ù�����ΪĿ����ͣ��************************/
			Total_Controller.Latitude_Position_Control.Expect=NamelessQuad.Position[_NORTH];
			Total_Controller.Longitude_Position_Control.Expect=NamelessQuad.Position[_EAST];			
			East_North_Ctrl_Reset();
			Hover_Point_Set_Flag=1;
		}
		else //���㵵λ���ڶ���ģʽ����δ���㶨����������Pos_Hold_SetFlag��0���ȴ�����ʱ��������ͣ��
		{
			Total_Controller.Latitude_Position_Control.Expect=0;
			Total_Controller.Longitude_Position_Control.Expect=0;
			East_North_Ctrl_Reset();
			Hover_Point_Set_Flag=0;//�����㶨������ʱ����λλ��������־λ���ȴ����㶨������ʱ���ٴ�����		
		}			
	}  
}




void Angle_Control_Target(Controller_Output *_flight_output)//�ǶȻ���
{
	//��̬�Ƕ�����
	Total_Controller.Pitch_Angle_Control.Expect=_flight_output->pitch_outer_control_output;
	Total_Controller.Roll_Angle_Control.Expect=_flight_output->roll_outer_control_output;
	
//	if(_flight_output->roll_pitch_angle_limit_enable==1)//SDKģʽ������������ơ�������
//	{
//		Total_Controller.Pitch_Angle_Control.Expect=constrain_float(Total_Controller.Pitch_Angle_Control.Expect,-10,10);
//		Total_Controller.Roll_Angle_Control.Expect=constrain_float(Total_Controller.Roll_Angle_Control.Expect,-10,10);
//	}
	
  //��̬�Ƕȷ���
  Total_Controller.Roll_Angle_Control.FeedBack =(WP_AHRS.Roll-Roll_Offset);
	Total_Controller.Pitch_Angle_Control.FeedBack=(WP_AHRS.Pitch-Pitch_Offset);
	//����PID������
  PID_Control(&Total_Controller.Roll_Angle_Control,min_ctrl_dt);  
	PID_Control(&Total_Controller.Pitch_Angle_Control,min_ctrl_dt);
	/***************�ڻ����ٶ�����****************/
	Total_Controller.Pitch_Gyro_Control.Expect=Total_Controller.Pitch_Angle_Control.Control_OutPut;
	Total_Controller.Roll_Gyro_Control.Expect=Total_Controller.Roll_Angle_Control.Control_OutPut;
  switch(_flight_output->yaw_ctrl_mode)
	{
	  case ROTATE:
		{		
			if(_flight_output->yaw_outer_control_output==0)//ƫ����������λ
			{
				if(Total_Controller.Yaw_Angle_Control.Expect==0)//�����л���
				{
				  Total_Controller.Yaw_Angle_Control.Expect=WP_AHRS.Yaw;
				}
				Total_Controller.Yaw_Angle_Control.FeedBack=WP_AHRS.Yaw;//ƫ���Ƿ���
				PID_Control_Yaw(&Total_Controller.Yaw_Angle_Control,min_ctrl_dt);//ƫ���Ƕȿ���
				Total_Controller.Yaw_Gyro_Control.Expect=Total_Controller.Yaw_Angle_Control.Control_OutPut;//ƫ�����ٶȻ���������Դ��ƫ���Ƕȿ��������
			}
			else//����ƫ������˺�ֻ�����ڻ����ٶȿ���
			{
				Total_Controller.Yaw_Angle_Control.Expect=0;//ƫ����������0,�����нǶȿ���
				Total_Controller.Yaw_Gyro_Control.Expect=_flight_output->yaw_outer_control_output;//ƫ�����ٶȻ�������ֱ����Դ��ң���������
			}
		}
		break;
	  case AZIMUTH://���Ժ���Ƕ�
	  {
			if(_flight_output->yaw_ctrl_start==1)//����ƫ���Ƕ�����
			{
				float yaw_tmp=_flight_output->yaw_outer_control_output;
				if(yaw_tmp<0) 	yaw_tmp+=360;
				if(yaw_tmp>360) yaw_tmp-=360;
				Total_Controller.Yaw_Angle_Control.Expect=yaw_tmp;
				_flight_output->yaw_ctrl_start=0;
				_flight_output->yaw_ctrl_cnt=0;
				_flight_output->yaw_ctrl_end=0;
			}
			
			if(_flight_output->yaw_ctrl_end==0)//�ж�ƫ�����Ƿ�������
			{
				if(ABS(Total_Controller.Yaw_Angle_Control.Err)<3.0f) _flight_output->yaw_ctrl_cnt++;
				else _flight_output->yaw_ctrl_cnt/=2;
				
				if(_flight_output->yaw_ctrl_cnt>=200)  _flight_output->yaw_ctrl_end=1;
			}			
			
			Total_Controller.Yaw_Angle_Control.FeedBack=WP_AHRS.Yaw;//ƫ���Ƿ���
			PID_Control_Yaw(&Total_Controller.Yaw_Angle_Control,min_ctrl_dt);//ƫ���Ƕȿ���
			//�����ƫ�����ٶȽ�������
			float tmp=constrain_float(Total_Controller.Yaw_Angle_Control.Control_OutPut,-YAW_GYRO_CTRL_MAX,YAW_GYRO_CTRL_MAX);
			Total_Controller.Yaw_Gyro_Control.Expect=tmp;//ƫ�����ٶȻ���������Դ��ƫ���Ƕȿ��������
		}
		break;
		case CLOCKWISE://˳ʱ�롪��������Ը���ʱ�̵ĺ���Ƕ�
		{
			if(_flight_output->yaw_ctrl_start==1)//����ƫ���Ƕ�����
			{
				float yaw_tmp=WP_AHRS.Yaw-_flight_output->yaw_outer_control_output;
				if(yaw_tmp<0) 	yaw_tmp+=360;
				if(yaw_tmp>360) yaw_tmp-=360;
				Total_Controller.Yaw_Angle_Control.Expect=yaw_tmp;
				_flight_output->yaw_ctrl_start=0;
				_flight_output->yaw_ctrl_cnt=0;
				_flight_output->yaw_ctrl_end=0;
			}
			
			if(_flight_output->yaw_ctrl_end==0)//�ж�ƫ�����Ƿ�������
			{
				if(ABS(Total_Controller.Yaw_Angle_Control.Err)<3.0f) _flight_output->yaw_ctrl_cnt++;
				else _flight_output->yaw_ctrl_cnt/=2;
				
				if(_flight_output->yaw_ctrl_cnt>=200)  _flight_output->yaw_ctrl_end=1;
			}
			
			Total_Controller.Yaw_Angle_Control.FeedBack=WP_AHRS.Yaw;//ƫ���Ƿ���
			PID_Control_Yaw(&Total_Controller.Yaw_Angle_Control,min_ctrl_dt);//ƫ���Ƕȿ���
			//�����ƫ�����ٶȽ�������
			float tmp=constrain_float(Total_Controller.Yaw_Angle_Control.Control_OutPut,-YAW_GYRO_CTRL_MAX,YAW_GYRO_CTRL_MAX);
			Total_Controller.Yaw_Gyro_Control.Expect=tmp;//ƫ�����ٶȻ���������Դ��ƫ���Ƕȿ��������
		}
		break;
		case ANTI_CLOCKWISE://��ʱ�롪����Ը���ʱ�̵ĺ���Ƕ�
		{
			if(_flight_output->yaw_ctrl_start==1)//����ƫ���Ƕ�����
			{
				float yaw_tmp=WP_AHRS.Yaw+_flight_output->yaw_outer_control_output;
				if(yaw_tmp<0) 	yaw_tmp+=360;
				if(yaw_tmp>360) yaw_tmp-=360;
				Total_Controller.Yaw_Angle_Control.Expect=yaw_tmp;
				_flight_output->yaw_ctrl_start=0;
				_flight_output->yaw_ctrl_cnt=0;
				_flight_output->yaw_ctrl_end=0;
			}
			
			if(_flight_output->yaw_ctrl_end==0)//�ж�ƫ�����Ƿ�������
			{
				if(ABS(Total_Controller.Yaw_Angle_Control.Err)<3.0f) _flight_output->yaw_ctrl_cnt++;
				else _flight_output->yaw_ctrl_cnt/=2;
				
				if(_flight_output->yaw_ctrl_cnt>=200)  _flight_output->yaw_ctrl_end=1;
			}
			
			Total_Controller.Yaw_Angle_Control.FeedBack=WP_AHRS.Yaw;//ƫ���Ƿ���
			PID_Control_Yaw(&Total_Controller.Yaw_Angle_Control,min_ctrl_dt);//ƫ���Ƕȿ���
			//�����ƫ�����ٶȽ�������
			float tmp=constrain_float(Total_Controller.Yaw_Angle_Control.Control_OutPut,-YAW_GYRO_CTRL_MAX,YAW_GYRO_CTRL_MAX);
			Total_Controller.Yaw_Gyro_Control.Expect=tmp;//ƫ�����ٶȻ���������Դ��ƫ���Ƕȿ��������
		}
		break;
		case CLOCKWISE_TURN://��ĳһ���ٶ�˳ʱ����ת�೤ʱ��
		{
			uint32_t curr_time_ms=millis();
			
			if(_flight_output->yaw_ctrl_start==1)//����ƫ���Ƕ�����
			{
				//�����ƫ�����ٶȽ�������
				float tmp=constrain_float(-_flight_output->yaw_outer_control_output,-YAW_GYRO_CTRL_MAX,YAW_GYRO_CTRL_MAX);
				Total_Controller.Yaw_Gyro_Control.Expect=tmp;//ƫ�����ٶȻ���������Դ��ƫ���Ƕȿ��������
				_flight_output->yaw_ctrl_start=0;
				_flight_output->yaw_ctrl_cnt=0;
				_flight_output->yaw_ctrl_end=0;
				_flight_output->start_time_ms=curr_time_ms;//��¼��ʼת����ʱ��				
			}
			
			if(_flight_output->yaw_ctrl_end==0)//�ж�ƫ�����Ƿ�������
			{
				uint32_t tmp=curr_time_ms-_flight_output->start_time_ms;
				if(tmp>=_flight_output->execution_time_ms)					
					_flight_output->yaw_ctrl_end=1;
			}
			else
			{
				//ִ����Ϻ�,
				//1����ƫ�����ٶ�������0,
				//2��ֹͣ��ת��������ǰƫ���ǣ���Ҫ�˳�CLOCKWISE_TURNģʽ���Ƕ������Ż���Ч����Ϊ��ģʽû�ж�ƫ���ǶȽ��п���
				Total_Controller.Yaw_Gyro_Control.Expect=0;
				Total_Controller.Yaw_Angle_Control.Expect=WP_AHRS.Yaw;
			}
		}
		break;
		case ANTI_CLOCKWISE_TURN://��ĳһ���ٶ���ʱ����ת�೤ʱ��
		{
			uint32_t curr_time_ms=millis();
			
			if(_flight_output->yaw_ctrl_start==1)//����ƫ���Ƕ�����
			{
				//�����ƫ�����ٶȽ�������
				float tmp=constrain_float(_flight_output->yaw_outer_control_output,-YAW_GYRO_CTRL_MAX,YAW_GYRO_CTRL_MAX);
				Total_Controller.Yaw_Gyro_Control.Expect=tmp;//ƫ�����ٶȻ���������Դ��ƫ���Ƕȿ��������
				_flight_output->yaw_ctrl_start=0;
				_flight_output->yaw_ctrl_cnt=0;
				_flight_output->yaw_ctrl_end=0;
				_flight_output->start_time_ms=curr_time_ms;//��¼��ʼת����ʱ��				
			}
			
			if(_flight_output->yaw_ctrl_end==0)//�ж�ƫ�����Ƿ�������
			{
				uint32_t tmp=curr_time_ms-_flight_output->start_time_ms;
				if(tmp>=_flight_output->execution_time_ms)					
					_flight_output->yaw_ctrl_end=1;
			}
			else
			{
				//ִ����Ϻ�,
				//1����ƫ�����ٶ�������0,
				//2��ֹͣ��ת��������ǰƫ���ǣ���Ҫ�˳�CLOCKWISE_TURNģʽ���Ƕ������Ż���Ч����Ϊ��ģʽû�ж�ƫ���ǶȽ��п���
				Total_Controller.Yaw_Gyro_Control.Expect=0;
				Total_Controller.Yaw_Angle_Control.Expect=WP_AHRS.Yaw;
			}
		}
		break;
  }
} 

void Yaw_Fault_Check(void)
{
	static uint16_t _cnt=0;
  /*******ƫ�������쳣����жϣ���ƫ���������ܴ�ʱ��ƫ�����ٶȺ�С�����ʱΪǿ�������š����ŵص�******************************/
  if(ABS(Total_Controller.Yaw_Gyro_Control.Control_OutPut)>Total_Controller.Yaw_Gyro_Control.Control_OutPut_Limit/2//ƫ�����������Խϴ�
     &&ABS(WP_AHRS.Yaw_Gyro)<=30.0f)//ƫ�����ٶ���Ժ�С
  {
    _cnt++;
    if(_cnt>=500) _cnt=500;
  }
  else _cnt/=2;//�����㣬����������0
  
  if(_cnt>=400)//����5ms*400=2S,���⴦��
  {
    PID_Integrate_Reset(&Total_Controller.Yaw_Gyro_Control); //���ƫ�����ٶȿ��ƵĻ���
    PID_Integrate_Reset(&Total_Controller.Yaw_Angle_Control);//���ƫ���ǿ��ƵĻ���
    Total_Controller.Yaw_Angle_Control.Expect=WP_AHRS.Yaw;	 //����ǰƫ���ǣ���Ϊ����ƫ����
    _cnt=0;
  }
}

void Gyro_Control(void)//���ٶȻ�
{	
	//���������������̬�ڻ����ٶȿ���������PID������
	/***************�ڻ����ٶ�����****************/
	Total_Controller.Pitch_Gyro_Control.Expect=Total_Controller.Pitch_Angle_Control.Control_OutPut;
	Total_Controller.Roll_Gyro_Control.Expect=Total_Controller.Roll_Angle_Control.Control_OutPut;
	/***************�ڻ����ٶȷ���****************/
	Total_Controller.Pitch_Gyro_Control.FeedBack=WP_AHRS.Pitch_Gyro;
	Total_Controller.Roll_Gyro_Control.FeedBack=WP_AHRS.Roll_Gyro;
	Total_Controller.Yaw_Gyro_Control.FeedBack=Yaw_Gyro_Earth_Frame;//Yaw_Gyro;
	/***************�ڻ����ٶȿ��ƣ�΢�ֲ�����̬����****************/
	if(WP_AHRS.player_level==0||WP_AHRS.player_level==1)
	{
		PID_Control_Div_LPF_For_Gyro(&Total_Controller.Pitch_Gyro_Control,min_ctrl_dt);
		PID_Control_Div_LPF_For_Gyro(&Total_Controller.Roll_Gyro_Control,min_ctrl_dt);
		PID_Control_Div_LPF_For_Gyro(&Total_Controller.Yaw_Gyro_Control,min_ctrl_dt);	
	}		
	else if(WP_AHRS.player_level==2)//΢������+һ�׵�ͨ40hz
	{
		pid_ctrl_rpy_gyro_maple(&Total_Controller.Pitch_Gyro_Control ,min_ctrl_dt,incomplete_diff,first_order_lpf);
		pid_ctrl_rpy_gyro_maple(&Total_Controller.Roll_Gyro_Control  ,min_ctrl_dt,incomplete_diff,first_order_lpf);
		pid_ctrl_rpy_gyro_maple(&Total_Controller.Yaw_Gyro_Control   ,min_ctrl_dt,incomplete_diff,first_order_lpf);		
	}
	else if(WP_AHRS.player_level==3)//΢������+���׵�ͨ50hz
	{
		pid_ctrl_rpy_gyro_maple(&Total_Controller.Pitch_Gyro_Control ,min_ctrl_dt,incomplete_diff,second_order_lpf);
		pid_ctrl_rpy_gyro_maple(&Total_Controller.Roll_Gyro_Control  ,min_ctrl_dt,incomplete_diff,second_order_lpf);
		pid_ctrl_rpy_gyro_maple(&Total_Controller.Yaw_Gyro_Control   ,min_ctrl_dt,incomplete_diff,second_order_lpf);		
	}
	else if(WP_AHRS.player_level==4)//ֱ��΢��+�޵�ͨ
	{
		pid_ctrl_rpy_gyro_maple(&Total_Controller.Pitch_Gyro_Control ,min_ctrl_dt,direct_diff,noneed_lpf);
		pid_ctrl_rpy_gyro_maple(&Total_Controller.Roll_Gyro_Control  ,min_ctrl_dt,direct_diff,noneed_lpf);
		pid_ctrl_rpy_gyro_maple(&Total_Controller.Yaw_Gyro_Control   ,min_ctrl_dt,direct_diff,noneed_lpf);		
	}	
	/*******ƫ�������쳣����******************************/
//  Yaw_Fault_Check();
}






void Main_Leading_Control(void)
{
	if(Flight.init==0)
	{		
		Flight.yaw_ctrl_mode=ROTATE;
		Flight.yaw_outer_control_output=0;
		Flight.pitch_outer_control_output=0;
		Flight.roll_outer_control_output=0;
		Flight.init=1;
	}
	
/*********************����ң�����л���λ���ɿؽ��벻ͬģʽ****************************/	
	if(Controler_Land_Mode==2)//һ������ģʽ
	{
		land_run();
		Flight.yaw_ctrl_mode=ROTATE;
		Flight.yaw_outer_control_output  =RC_Data.rc_rpyt[RC_YAW];
		return ;
	}
	else if(Controler_GPS_Mode==2)//GPS����ģʽ  
	{
		Flight_Alt_Hold_Control(ALTHOLD_MANUAL_CTRL,NUL,NUL);//�߶ȿ���
		ncq_control_poshold();//λ�ÿ���
		Flight.yaw_ctrl_mode=ROTATE;
		Flight.yaw_outer_control_output  =RC_Data.rc_rpyt[RC_YAW];		
	  return ;
	}
	
/************************����ͨ�����Ƹ߶�ģʽ************************/		
	if(Controler_High_Mode==1)//�߶ȴ��ֶ�ģʽ
	{
		Throttle=Throttle_Control;//����ֱ����Դ��ң�������Ÿ���
		Flight.roll_outer_control_output =RC_Data.rc_rpyt[RC_ROLL];
		Flight.pitch_outer_control_output=RC_Data.rc_rpyt[RC_PITCH];
		Flight.yaw_ctrl_mode=ROTATE;
		Flight.yaw_outer_control_output=RC_Data.rc_rpyt[RC_YAW];		
	}
	else
	{		
		if(Controler_SDK1_Mode==1)//�û�SDK�������Զ�����ģʽ��ˮƽ+�߶ȿ���
		{
			Auto_Flight_Ctrl(&SDK1_Mode_Setup);
		}
		else//��ͨ���ߡ��������
		{
/************************�ڰ�ͨ������ˮƽλ��ģʽ************************/		
			switch(Controler_Horizontal_Mode)
			{
				case 1://ˮƽ����
				{
					Flight.roll_outer_control_output =RC_Data.rc_rpyt[RC_ROLL];
					Flight.pitch_outer_control_output=RC_Data.rc_rpyt[RC_PITCH];
					
					Flight.yaw_ctrl_mode=ROTATE;
					Flight.yaw_outer_control_output  =RC_Data.rc_rpyt[RC_YAW];					
				}
				break;
				case 2://ˮƽ��������ģʽ
				{
					OpticalFlow_Control_Pure(0);//��ͨ����ģʽ		
					Flight.yaw_ctrl_mode=ROTATE;
					Flight.yaw_outer_control_output  =RC_Data.rc_rpyt[RC_YAW];				
				}
				break;				
				case 3://SLAMˮƽ����ģʽ
				{
					slam_control_poshold(&VIO_SINS);			
					Flight.yaw_ctrl_mode=ROTATE;
					Flight.yaw_outer_control_output  =RC_Data.rc_rpyt[RC_YAW];				
				}
				break;
				default:
				{
					Flight.roll_outer_control_output =RC_Data.rc_rpyt[RC_ROLL];
					Flight.pitch_outer_control_output=RC_Data.rc_rpyt[RC_PITCH];	
					Flight.yaw_ctrl_mode=ROTATE;
					Flight.yaw_outer_control_output  =RC_Data.rc_rpyt[RC_YAW];						
				}					
			}
			Flight_Alt_Hold_Control(ALTHOLD_MANUAL_CTRL,NUL,NUL);//�߶ȿ���		
		}
	}
}




/************��̬�����������Ƕ�+���ٶ�****************/
void Attitude_Control(void)
{ 
	Angle_Control_Target(&Flight);//�Ƕȿ���	
  Gyro_Control();//���ٶȿ���
}

/***************************************************
������: void Total_Control(void)
˵��:	�ܿ��������У������������
1������ң�������롢��ǰ״̬����������ģʽ������+�����ֶ�������+���ȡ�����+���㣨���٣��ȣ�
2�������ϲ������������̬�������߶ȿ��Ƶ�
3�����ȣ���̬������
���:	��
����:	��
��ע:	�ϵ��ʼ��������һ��
****************************************************/
void Total_Control(void)
{
	static uint16_t main_ctrl_cnt=0,att_ctrl_cnt=0;
  /*************����������******************/
	main_ctrl_cnt++;
	if(main_ctrl_cnt>=1)
	{
		Main_Leading_Control();
		landon_earth_check();//��½�����Լ�
		main_ctrl_cnt=0;
	}		
  /*************��̬��������*****************/
	att_ctrl_cnt++;
	if(att_ctrl_cnt>=1)
	{
		Attitude_Control();
		att_ctrl_cnt=0;
	}		
}


void CarryPilot_Control(void)
{
  static uint8_t ctrl_cnt=0;
  ctrl_cnt++;  
  /*************������ģʽѡ��******************/
  if(ctrl_cnt>=4)//�Ĵ˼�����ֵ�����Ե�����������
  {  
    Controler_Mode_Select();
    ctrl_cnt=0;
  }
  Total_Control();//�ܿ�������ˮƽλ��+ˮƽ�ٶ�+��̬���Ƕ�+���ٶȣ����������߶�λ��+�߶��ٶ�+�߶ȼ��ٶȿ�����	
  Control_Output();//�����������
}




//�ر�����
//�㸲ǿ��ͣ�����ֻ�ʺ��ڸ�����Ʋ�������
//�������Ѿ��ܱ���������̬�ȶ����е�������ʹ��
//�����û���ȫ���ǣ���ƴ˰�ȫ����
//�����û������ڵ������˻�������������з���ը����ʧ�صȽ�������ʱ���Ų���·��������ʱ������
//�����㸲���󣬷�����������ʱ����������������
//���û������ܰ��������������������ڴ��㸲��⣬�κ�ʱ�򶼲�Ҫ����ң������������Ȩ
#define overturn_crash_enable 1//�㸲���ʹ��
#define overturn_crash_time 100//����㸲���ʱ��*5ms���Ƽ�ȡֵ����100~400
void overturn_check(void)//�㸲ǿ��ͣ����⣬����5msִ��һ��������
{
#if	overturn_crash_enable
	static uint32_t crash_cnt=0;
	//�¶Ⱦ�λ���������ѱ궨
	if(Gyro_Safety_Calibration_Flag==0)   return;
	if(crash_cnt==overturn_crash_time+1)  return;

	if(crash_cnt<overturn_crash_time)//��������ʱ������ʱ,ǿ�������ɻ�
	{
		if(ABS(WP_AHRS.Pitch)>40||ABS(WP_AHRS.Roll)>40)//��б����40��
			crash_cnt++;
		else crash_cnt/=2;	
	}
	else 
	{
		crash_cnt++;
		Controler_State=Lock_Controler;//ǿ������
		//RGB����˸����
		Bling_Set(&rgb_red,3000,1000,0.5,0,GPIO_PORTF_BASE,GPIO_PIN_1,1);//��ɫ
		Bling_Set(&rgb_blue,3000,1000,0.5,0,GPIO_PORTF_BASE,GPIO_PIN_2,1);//��ɫ
		Bling_Set(&rgb_green,3000,1000,0.5,0,GPIO_PORTF_BASE,GPIO_PIN_3,1);//��ɫ	
	}
#endif
}


//#define Thr_Feedforward_Compensation//﮵�ص�ѹǰ������ʹ��
#ifdef Thr_Feedforward_Compensation
	#define Battery_Cell_Voltage    4.2//1S﮵�س���4.2V
	#define Battery_Cell_Numeber    4//4S﮵��
	#define Battery_Full_Voltage    (Battery_Cell_Voltage*Battery_Cell_Numeber)
#endif

uint16_t Throttle_Output=0;
void Throttle_Angle_Compensate()//������ǲ���
{
  float CosPitch_CosRoll=ABS(WP_AHRS.cos_rpy[_PIT]*WP_AHRS.cos_rpy[_ROL]);
  float Throttle_Makeup=0;
  float Temp=0;
  if(CosPitch_CosRoll>=0.999999f)  CosPitch_CosRoll=0.999999f;
  if(CosPitch_CosRoll<=0.000001f)  CosPitch_CosRoll=0.000001f;
  if(CosPitch_CosRoll<=0.50f)  CosPitch_CosRoll=0.50f;//Pitch,RollԼ����30��
  if(Throttle>=Thr_Start)//������ת������
  {
    Temp=(uint16_t)(MAX(ABS(100*WP_AHRS.Pitch),ABS(100*WP_AHRS.Roll)));
    Temp=constrain_float(9000-Temp,0,3000)/(3000*CosPitch_CosRoll);
    Throttle_Makeup=(Throttle-Thr_Start)*Temp;//������ǲ���
    Throttle_Output=(uint16_t)(Thr_Start+Throttle_Makeup);
    Throttle_Output=(uint16_t)(constrain_float(Throttle_Output,Thr_Start,2000));
#ifdef Thr_Feedforward_Compensation 
		Throttle_Output+=(Throttle_Output-Thr_Start)*(Battery_Full_Voltage/Battery_Voltage-1.0f);
#endif
  }
  else Throttle_Output=Throttle;	
}


/**************************************************************
***************************************************************
X�Ͱ�װ��ʽ������������̬�ǹ�ϵ
                   -
                 Pitch
					3#             1#
					   *          *
-   Roll          *         Roll   +
						 *          *
					2#             4#
				         Pitch
								   +
���ٶȴ���������������X��Y��Zͬ�ᣬ������ԭ�㿴����ʱ����ת�Ƕ�Ϊ+
Y Aixs
*
*
*
*
*
*
* * * * * * * *   X Axis
(O)
*******************************************************************
******************************************************************/
uint16_t Idel_Cnt=0;
#define Idel_Transition_Gap 4//���ٵ������ʱ�� 2*5=10ms
#define Idel_Transition_Period (Thr_Idle-Thr_Min)//����������������  10ms*100=1s
uint16_t Thr_Idle_Transition_Cnt=0;
void Control_Output()
{
	overturn_check();//�㸲���
  Throttle_Angle_Compensate();//������ǲ���
  if(Controler_State==Unlock_Controler)//����
  {
    if(Landon_Earth_Flag==1)//��⵽��½����
    {
      if(_M_PWM_1<=Thr_Min&&_M_PWM_2<=Thr_Min&&_M_PWM_3<=Thr_Min&&_M_PWM_4<=Thr_Min)//ֻ���������ٽ���ʱ�Ż�����
      {
        //����ϴ��������ֵΪ���λ�����뵡��ʱ�����Ź��ɹ���
        Thr_Idle_Transition_Cnt=Idel_Transition_Period;
      }
      else//����ʱ�̽�����½����
      {
        if(Last_Landon_Earth_Flag==0)//�ϴ�Ϊ���״̬������Ϊ��½״̬���������
        {
          Controler_State=Lock_Controler;
          Bling_Set(&rgb_green,3000,200,0.5,0,GPIO_PORTF_BASE,GPIO_PIN_3,0);
        }
      }
      
      Idel_Cnt++;
      if(Idel_Cnt>=Idel_Transition_Gap)
      {
        if(Thr_Idle_Transition_Cnt>=1)
          Thr_Idle_Transition_Cnt--;
        Idel_Cnt=0;
      }
      M_PWM_1=Thr_Min+(Idel_Transition_Period-Thr_Idle_Transition_Cnt)*(Thr_Idle-Thr_Min)/Idel_Transition_Period;//���ŵ���
      M_PWM_2=Thr_Min+(Idel_Transition_Period-Thr_Idle_Transition_Cnt)*(Thr_Idle-Thr_Min)/Idel_Transition_Period;
      M_PWM_3=Thr_Min+(Idel_Transition_Period-Thr_Idle_Transition_Cnt)*(Thr_Idle-Thr_Min)/Idel_Transition_Period;
      M_PWM_4=Thr_Min+(Idel_Transition_Period-Thr_Idle_Transition_Cnt)*(Thr_Idle-Thr_Min)/Idel_Transition_Period;
      Take_Off_Reset();//�����
      //OpticalFlow_SINS_Reset();
      OpticalFlow_Ctrl_Reset();
    }
    else  //������������½������Ĭ�����
    {
			switch(Controler_High_Mode)
			{
				case 2://����ģʽ
			  {
					if(Auto_Relock_Flag_Set==1)//��������ƫ������֮��
					{
						M_PWM_1=Int_Sort(Moter1_Thr_Scale  *Throttle_Output
																+Moter1_Roll_Scale *Total_Controller.Roll_Gyro_Control.Control_OutPut
																+Moter1_Pitch_Scale*Total_Controller.Pitch_Gyro_Control.Control_OutPut
																+Moter1_Yaw_Scale  *Total_Controller.Yaw_Gyro_Control.Control_OutPut);
						M_PWM_2=Int_Sort(Moter2_Thr_Scale  *Throttle_Output
																+Moter2_Roll_Scale *Total_Controller.Roll_Gyro_Control.Control_OutPut
																+Moter2_Pitch_Scale*Total_Controller.Pitch_Gyro_Control.Control_OutPut
																+Moter2_Yaw_Scale  *Total_Controller.Yaw_Gyro_Control.Control_OutPut);
						M_PWM_3=Int_Sort(Moter3_Thr_Scale  *Throttle_Output
																+Moter3_Roll_Scale *Total_Controller.Roll_Gyro_Control.Control_OutPut
																+Moter3_Pitch_Scale*Total_Controller.Pitch_Gyro_Control.Control_OutPut
																+Moter3_Yaw_Scale  *Total_Controller.Yaw_Gyro_Control.Control_OutPut);
						M_PWM_4=Int_Sort(Moter4_Thr_Scale  *Throttle_Output
																+Moter4_Roll_Scale *Total_Controller.Roll_Gyro_Control.Control_OutPut
																+Moter4_Pitch_Scale*Total_Controller.Pitch_Gyro_Control.Control_OutPut
																+Moter4_Yaw_Scale  *Total_Controller.Yaw_Gyro_Control.Control_OutPut);
					}
					else//��������ƫ������֮ǰ
					{
						M_PWM_1=(int16_t)(Throttle_Output);	
						M_PWM_2=(int16_t)(Throttle_Output);	
						M_PWM_3=(int16_t)(Throttle_Output);	
						M_PWM_4=(int16_t)(Throttle_Output);	
					}	
					M_PWM_1=Value_Limit(Thr_Idle,2500,M_PWM_1);//������޷�
					M_PWM_2=Value_Limit(Thr_Idle,2500,M_PWM_2);
					M_PWM_3=Value_Limit(Thr_Idle,2500,M_PWM_3);
					M_PWM_4=Value_Limit(Thr_Idle,2500,M_PWM_4);
				}
				break;
				default://��̬ģʽ
				{
					if(Throttle>=Thr_Fly_Start)//�����������
					{
							M_PWM_1=Int_Sort(Moter1_Thr_Scale  *Throttle_Output
																	+Moter1_Roll_Scale *Total_Controller.Roll_Gyro_Control.Control_OutPut
																	+Moter1_Pitch_Scale*Total_Controller.Pitch_Gyro_Control.Control_OutPut
																	+Moter1_Yaw_Scale  *Total_Controller.Yaw_Gyro_Control.Control_OutPut);
							M_PWM_2=Int_Sort(Moter2_Thr_Scale  *Throttle_Output
																	+Moter2_Roll_Scale *Total_Controller.Roll_Gyro_Control.Control_OutPut
																	+Moter2_Pitch_Scale*Total_Controller.Pitch_Gyro_Control.Control_OutPut
																	+Moter2_Yaw_Scale  *Total_Controller.Yaw_Gyro_Control.Control_OutPut);
							M_PWM_3=Int_Sort(Moter3_Thr_Scale  *Throttle_Output
																	+Moter3_Roll_Scale *Total_Controller.Roll_Gyro_Control.Control_OutPut
																	+Moter3_Pitch_Scale*Total_Controller.Pitch_Gyro_Control.Control_OutPut
																	+Moter3_Yaw_Scale  *Total_Controller.Yaw_Gyro_Control.Control_OutPut);
							M_PWM_4=Int_Sort(Moter4_Thr_Scale  *Throttle_Output
																	+Moter4_Roll_Scale *Total_Controller.Roll_Gyro_Control.Control_OutPut
																	+Moter4_Pitch_Scale*Total_Controller.Pitch_Gyro_Control.Control_OutPut
																	+Moter4_Yaw_Scale  *Total_Controller.Yaw_Gyro_Control.Control_OutPut);
							
					}
					else//С���������
					{
						M_PWM_1=Int_Sort(Throttle_Output);
						M_PWM_2=Int_Sort(Throttle_Output);
						M_PWM_3=Int_Sort(Throttle_Output);
						M_PWM_4=Int_Sort(Throttle_Output);
						Take_Off_Reset();//�����
					}
					M_PWM_1=Value_Limit(Thr_Idle,2500,M_PWM_1);//������޷�
					M_PWM_2=Value_Limit(Thr_Idle,2500,M_PWM_2);
					M_PWM_3=Value_Limit(Thr_Idle,2500,M_PWM_3);
					M_PWM_4=Value_Limit(Thr_Idle,2500,M_PWM_4);		
					Throttle_Control_Reset();				
				}
				break;				
			}
		}
	}	
  else//δ�����������������λ��ͣת
  {
    M_PWM_1=Thr_Min;
    M_PWM_2=Thr_Min;
    M_PWM_3=Thr_Min;
    M_PWM_4=Thr_Min;
    Take_Off_Reset();//�����
		//OpticalFlow_SINS_Reset();
		OpticalFlow_Ctrl_Reset();
//    Throttle_Control_Reset();
	}
  _M_PWM_1=M_PWM_1;
  _M_PWM_2=M_PWM_2;
  _M_PWM_3=M_PWM_3;
  _M_PWM_4=M_PWM_4;  
  M_PWM_1=Value_Limit(0,2200,M_PWM_1);//������޷�
  M_PWM_2=Value_Limit(0,2200,M_PWM_2);
  M_PWM_3=Value_Limit(0,2200,M_PWM_3);
  M_PWM_4=Value_Limit(0,2200,M_PWM_4);
	PWM_Output(M_PWM_1,M_PWM_2,M_PWM_3,M_PWM_4);
}









/* Copyright (c)  2018-2025 Wuhan Nameless Innovation Technology Co.,Ltd. All rights reserved.*/



