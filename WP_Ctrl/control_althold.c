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
#include "control_config.h"
#include "control_althold.h"

Vector3f UAV_Cushion_Stop_Point;




float ALT_VEL_FEEDFORWARD_ENABLE_Output=0;//��ֱ�ٶ�ǰ�����������;
float ALT_VEL_FEEDFORWARD_ENABLE_Rate=0.1;//��ֱ�ٶ�ǰ����������APM����Ϊ1��0.45;
float ALT_VEL_FEEDFORWARD_ENABLE_Delta=0;//��ֱ�����ٶȱ仯��;
float Alt_Vel_Target=0,Last_Alt_Vel_Target=0;

uint16_t  Deadband=200;//������λ����
/*************����ΪĬ��ֵ��У׼ң��������Զ���ֵ**********************/
uint16_t  Deadzone_Min=400;
uint16_t  Deadzone_Max=600;
uint16_t  Thr_Top=1000;//����������г�
uint16_t  Thr_Buttom=0;//����������г�
uint16_t  Thr_Middle=500;//����������г�
uint8_t Thr_Push_Over_State=0;
uint8_t Thr_Push_Over_Deadband(void)
{
  static uint8_t thr_now=0,thr_last=0;
  static uint8_t push_flag=0;
  thr_last=thr_now;
  if(TRPY[0]<=Deadzone_Min) thr_now=0;
  else if(TRPY[0]<=Deadzone_Max) thr_now=1;
  else thr_now=2;  
  if(thr_now==0&&thr_last==1)       return push_flag=0;//�����ƹ���λ����
  else if(thr_now==1&&thr_last==0)  return push_flag=1;//�����Ƶ���λ����
  else if(thr_now==2&&thr_last==1)  return push_flag=2;//�����ƹ���λ����
  else if(thr_now==1&&thr_last==2)  return push_flag=3;//�����Ƶ���λ����
  else push_flag=0;
  return push_flag;
}

void Thr_Scale_Set(rc_calibration *rc_date)
{
  Thr_Buttom=0;
  Thr_Top=(uint16_t)(rc_date[2].max-rc_date[2].min);
  Thr_Middle=(uint16_t)((rc_date[2].max-rc_date[2].min)/2);
  //Deadband=rc_date[2].deadband;//������λ������������
  Deadzone_Min=Thr_Middle-Deadband/2;
  Deadzone_Max=Thr_Middle+Deadband/2;
}



void Flight_Alt_Hold_Control(uint8_t mode,float target_alt,float target_vel)
{	 
	 static uint8_t last_mode=0;	 
	 static systime althold_dt;
	 static uint8_t move_flag=0;
	 uint8_t alt_fixed_flag=0;
	 Get_Systime(&althold_dt);
	 High_Hold_Throttle=Thr_Hover_Default;
	 //����������������ڴ��ڿ�������10��
	 if(0.001f*althold_dt.period>=10*WP_Duty_Dt)
	 {
		//���1:���λ��ߴ�����ģʽ���뱾ģʽ
		//���2:ϵͳ���ȳ�ʱ����ϵͳ��ƺ�������£�����������ܷ���
		alt_fixed_flag=1;
	 }
	 //�߶ȿ��ƴ��Զ�ģʽ�е��ֶ�ģʽ	 
	 if(last_mode!=mode)  alt_fixed_flag=1;
	 last_mode=mode;	
   switch(mode)
	 {
		 case ALTHOLD_MANUAL_CTRL:
		 {		 
			 //1���߶�λ�ÿ�����
			 if(RC_Data.rc_rpyt[RC_THR]==0)//���Ÿ�λ����λ�����ڣ����и߶ȱ���
			 {
					//�߶�λ�û���������ٶ�����
					if(Total_Controller.Height_Position_Control.Expect==0||alt_fixed_flag==1)//���Ż��к󣬽�����ʱ�̵ĸ߶�����Ϊ�����߶ȣ�������1��
					{
						Total_Controller.Height_Position_Control.Expect=NamelessQuad.Position[_UP];	//���Ż��к󣬸��¸߶�����
					}
					Total_Controller.Height_Position_Control.FeedBack=NamelessQuad.Position[_UP];	//�߶�λ�÷���
					PID_Control(&Total_Controller.Height_Position_Control,0.005f);									//���θ߶�λ�ÿ�����
					
					Total_Controller.Height_Speed_Control.Expect=Total_Controller.Height_Position_Control.Control_OutPut;
					move_flag=0;
								 
#ifdef ALT_VEL_FEEDFORWARD_ENABLE
					float delta=Total_Controller.Height_Speed_Control.PID_Controller_Dt.Time_Delta/1000.0f;
					Alt_Vel_Target=Total_Controller.Height_Speed_Control.Expect;//�����ٶ�����
					ALT_VEL_FEEDFORWARD_ENABLE_Delta=(Alt_Vel_Target-Last_Alt_Vel_Target)/(1*delta);//�ٶ������仯��
					ALT_VEL_FEEDFORWARD_ENABLE_Output=ALT_VEL_FEEDFORWARD_ENABLE_Rate*ALT_VEL_FEEDFORWARD_ENABLE_Delta;
					Last_Alt_Vel_Target=Alt_Vel_Target;//�ϴ��ٶ�����
#endif
			 }
			 else if(RC_Data.rc_rpyt[RC_THR]>0)//���������ٶ�����
			 {
			  	//���Ÿ����ơ������ٶ�����
			 	 Total_Controller.Height_Speed_Control.Expect=RC_Data.rc_rpyt[RC_THR];//��������ٶ�300cm/s
				 Total_Controller.Height_Position_Control.Expect=0;//λ�û�������0
				 move_flag=1;
			 }
			 else if(RC_Data.rc_rpyt[RC_THR]<0)//�����½��ٶ�����
			 {
				//���Ÿ����ơ������ٶ�����
				 Total_Controller.Height_Speed_Control.Expect=RC_Data.rc_rpyt[RC_THR];//����½��ٶ�200cm/s
				 Total_Controller.Height_Position_Control.Expect=0;//λ�û�������0
				 move_flag=2;
			 }
			 
				//2���߶��ٶȿ�����
				Total_Controller.Height_Speed_Control.FeedBack=NamelessQuad.Speed[_UP];//�ߵ��ٶȹ��Ƹ����ٶȷ���
				PID_Control_Div_LPF(&Total_Controller.Height_Speed_Control,0.005f);//���θ߶��ٶȿ���
				/*******************************�����½��������������ٶ��޷���������*******************************************************************/
				if(move_flag==0)
				{
						if(Controler_SDK1_Mode==1)//������SDKģʽ�¶��������ٶȽ������Ʒ�,��ʹ���
							Total_Controller.Height_Speed_Control.Control_OutPut=constrain_float(Total_Controller.Height_Speed_Control.Control_OutPut,
																																									-SDK_Stable_Acceleration_Max,SDK_Stable_Acceleration_Max);  
				}	
				else if(move_flag==1)//�������Ƹ�ʱ���ٶȿ�����������������������½����ٶȽ��зֱ��޷���ȷ�������½�ʱ��̬ƽ��
				{
					Total_Controller.Height_Speed_Control.Control_OutPut=constrain_float(Total_Controller.Height_Speed_Control.Control_OutPut,
																																						 -Climb_Up_Acceleration_Max,Climb_Up_Acceleration_Max);                                                                
				}
				else if(move_flag==2)
				{
					Total_Controller.Height_Speed_Control.Control_OutPut=constrain_float(Total_Controller.Height_Speed_Control.Control_OutPut,
																																						 -Climb_Down_Acceleration_Max,Climb_Down_Acceleration_Max);
				}
				//3���߶ȼ��ٶȿ�����
				Total_Controller.Height_Acce_Control.Expect=Total_Controller.Height_Speed_Control.Control_OutPut;//���ٶ�����
#ifdef ALT_VEL_FEEDFORWARD_ENABLE	 
				Total_Controller.Height_Acce_Control.Expect+=ALT_VEL_FEEDFORWARD_ENABLE_Output;//����ǰ�����������
#endif	 
				Total_Controller.Height_Acce_Control.FeedBack=Acceleration_Feedback[_UP];//���ٶȷ���
				PID_Control(&Total_Controller.Height_Acce_Control,0.005f);//���θ߶ȼ��ٶȿ���
				/**************************************
				���ٶȻ�ǰ������������ʱ��ע������
				��ͣ���� = ���ٶȻ�����ֵ + ��׼��ͣ����
				��ʱ����� F = mg
				����Ҫ���a�ļ��ٶ�ʱ������� F1=m(g+a)
				F1/F = 1 + a/g
				��˴�ʱӦ�������ͣ����*(1 + a/g)
				**************************************/
				Total_Controller.Height_Acce_Control.Control_OutPut+=(Thr_Hover_Default+Total_Controller.Height_Acce_Control.Integrate-Thr_Start)
																													*(Total_Controller.Height_Acce_Control.Expect/980);
			}
			 break;
			
			case ALTHOLD_AUTO_POS_CTRL:
			{		 
			 //1���߶�λ�ÿ�����
			 if(RC_Data.rc_rpyt[RC_THR]==0)//���Ÿ�λ����λ�����ڣ����и߶ȱ���
			 {
					//�߶�λ�û���������ٶ�����
					Total_Controller.Height_Position_Control.Expect=target_alt;	//���Ŵ��ڻ��к󣬸��¸߶�����
					Total_Controller.Height_Position_Control.FeedBack=NamelessQuad.Position[_UP];	//�߶�λ�÷���
					PID_Control(&Total_Controller.Height_Position_Control,0.005f);									//���θ߶�λ�ÿ�����
					
					Total_Controller.Height_Speed_Control.Expect=Total_Controller.Height_Position_Control.Control_OutPut;
					move_flag=0;
#ifdef ALT_VEL_FEEDFORWARD_ENABLE
					float delta=Total_Controller.Height_Speed_Control.PID_Controller_Dt.Time_Delta/1000.0f;
					Alt_Vel_Target=Total_Controller.Height_Speed_Control.Expect;//�����ٶ�����
					ALT_VEL_FEEDFORWARD_ENABLE_Delta=(Alt_Vel_Target-Last_Alt_Vel_Target)/(1*delta);//�ٶ������仯��
					ALT_VEL_FEEDFORWARD_ENABLE_Output=ALT_VEL_FEEDFORWARD_ENABLE_Rate*ALT_VEL_FEEDFORWARD_ENABLE_Delta;
					Last_Alt_Vel_Target=Alt_Vel_Target;//�ϴ��ٶ�����
#endif
			 }
			 else if(RC_Data.rc_rpyt[RC_THR]>0)//���������ٶ�����
			 {
			  	//���Ÿ����ơ������ٶ�����
			 	 Total_Controller.Height_Speed_Control.Expect=RC_Data.rc_rpyt[RC_THR];//��������ٶ�300cm/s
				 Total_Controller.Height_Position_Control.Expect=0;//λ�û�������0
				 move_flag=1;
			 }
			 else if(RC_Data.rc_rpyt[RC_THR]<0)//�����½��ٶ�����
			 {
				//���Ÿ����ơ������ٶ�����
				 Total_Controller.Height_Speed_Control.Expect=RC_Data.rc_rpyt[RC_THR];//����½��ٶ�200cm/s
				 Total_Controller.Height_Position_Control.Expect=0;//λ�û�������0
				 move_flag=2;
			 }
			 
				//2���߶��ٶȿ�����
				Total_Controller.Height_Speed_Control.FeedBack=NamelessQuad.Speed[_UP];//�ߵ��ٶȹ��Ƹ����ٶȷ���
				PID_Control_Div_LPF(&Total_Controller.Height_Speed_Control,0.005f);//���θ߶��ٶȿ���
				/*******************************�����½��������������ٶ��޷���������*******************************************************************/     
				if(move_flag==1)//�������Ƹ�ʱ���ٶȿ�����������������������½����ٶȽ��зֱ��޷���ȷ�������½�ʱ��̬ƽ��
				{
					Total_Controller.Height_Speed_Control.Control_OutPut=constrain_float(Total_Controller.Height_Speed_Control.Control_OutPut,
																																						 -Climb_Up_Acceleration_Max,Climb_Up_Acceleration_Max);                                                                
				}
				else if(move_flag==2)
				{
					Total_Controller.Height_Speed_Control.Control_OutPut=constrain_float(Total_Controller.Height_Speed_Control.Control_OutPut,
																																						 -Climb_Down_Acceleration_Max,Climb_Down_Acceleration_Max);
				}
				//3���߶ȼ��ٶȿ�����
				Total_Controller.Height_Acce_Control.Expect=Total_Controller.Height_Speed_Control.Control_OutPut;//���ٶ�����
#ifdef ALT_VEL_FEEDFORWARD_ENABLE	 
				Total_Controller.Height_Acce_Control.Expect+=ALT_VEL_FEEDFORWARD_ENABLE_Output;//����ǰ�����������
#endif	 
				Total_Controller.Height_Acce_Control.FeedBack=Acceleration_Feedback[_UP];//���ٶȷ���
				PID_Control(&Total_Controller.Height_Acce_Control,0.005f);//���θ߶ȼ��ٶȿ���
				/**************************************
				���ٶȻ�ǰ������������ʱ��ע������
				��ͣ���� = ���ٶȻ�����ֵ + ��׼��ͣ����
				��ʱ����� F = mg
				����Ҫ���a�ļ��ٶ�ʱ������� F1=m(g+a)
				F1/F = 1 + a/g
				��˴�ʱӦ�������ͣ����*(1 + a/g)
				**************************************/
				Total_Controller.Height_Acce_Control.Control_OutPut+=(Thr_Hover_Default+Total_Controller.Height_Acce_Control.Integrate-Thr_Start)
																													*(Total_Controller.Height_Acce_Control.Expect/980);
			}
			 break;
			case ALTHOLD_AUTO_VEL_CTRL:
			{		 
			 //1���߶�λ�ÿ�����
			 if(RC_Data.rc_rpyt[RC_THR]==0)//���Ÿ�λ����λ�����ڣ����и߶ȱ���
			 {
					//�߶�λ�û���������ٶ�����
					Total_Controller.Height_Position_Control.Expect=0;	//���Ŵ��ڻ��к󣬸��¸߶�����					
					Total_Controller.Height_Speed_Control.Expect=target_vel;
					move_flag=0;
#ifdef ALT_VEL_FEEDFORWARD_ENABLE
					float delta=Total_Controller.Height_Speed_Control.PID_Controller_Dt.Time_Delta/1000.0f;
					Alt_Vel_Target=Total_Controller.Height_Speed_Control.Expect;//�����ٶ�����
					ALT_VEL_FEEDFORWARD_ENABLE_Delta=(Alt_Vel_Target-Last_Alt_Vel_Target)/(1*delta);//�ٶ������仯��
					ALT_VEL_FEEDFORWARD_ENABLE_Output=ALT_VEL_FEEDFORWARD_ENABLE_Rate*ALT_VEL_FEEDFORWARD_ENABLE_Delta;
					Last_Alt_Vel_Target=Alt_Vel_Target;//�ϴ��ٶ�����
#endif
			 }
			 else if(RC_Data.rc_rpyt[RC_THR]>0)//���������ٶ�����
			 {
			  	//���Ÿ����ơ������ٶ�����
			 	 Total_Controller.Height_Speed_Control.Expect=RC_Data.rc_rpyt[RC_THR];//��������ٶ�300cm/s
				 Total_Controller.Height_Position_Control.Expect=0;//λ�û�������0
				 move_flag=1;
			 }
			 else if(RC_Data.rc_rpyt[RC_THR]<0)//�����½��ٶ�����
			 {
				//���Ÿ����ơ������ٶ�����
				 Total_Controller.Height_Speed_Control.Expect=RC_Data.rc_rpyt[RC_THR];//����½��ٶ�200cm/s
				 Total_Controller.Height_Position_Control.Expect=0;//λ�û�������0
				 move_flag=2;
			 }
			 
				//2���߶��ٶȿ�����
				Total_Controller.Height_Speed_Control.FeedBack=NamelessQuad.Speed[_UP];//�ߵ��ٶȹ��Ƹ����ٶȷ���
				PID_Control_Div_LPF(&Total_Controller.Height_Speed_Control,0.005f);//���θ߶��ٶȿ���
				/*******************************�����½��������������ٶ��޷���������*******************************************************************/     
				if(move_flag==1)//�������Ƹ�ʱ���ٶȿ�����������������������½����ٶȽ��зֱ��޷���ȷ�������½�ʱ��̬ƽ��
				{
					Total_Controller.Height_Speed_Control.Control_OutPut=constrain_float(Total_Controller.Height_Speed_Control.Control_OutPut,
																																						 -Climb_Up_Acceleration_Max,Climb_Up_Acceleration_Max);                                                                
				}
				else if(move_flag==2)
				{
					Total_Controller.Height_Speed_Control.Control_OutPut=constrain_float(Total_Controller.Height_Speed_Control.Control_OutPut,
																																						 -Climb_Down_Acceleration_Max,Climb_Down_Acceleration_Max);
				}
				//3���߶ȼ��ٶȿ�����
				Total_Controller.Height_Acce_Control.Expect=Total_Controller.Height_Speed_Control.Control_OutPut;//���ٶ�����
#ifdef ALT_VEL_FEEDFORWARD_ENABLE	 
				Total_Controller.Height_Acce_Control.Expect+=ALT_VEL_FEEDFORWARD_ENABLE_Output;//����ǰ�����������
#endif	 
				Total_Controller.Height_Acce_Control.FeedBack=Acceleration_Feedback[_UP];//���ٶȷ���
				PID_Control(&Total_Controller.Height_Acce_Control,0.005f);//���θ߶ȼ��ٶȿ���
				/**************************************
				���ٶȻ�ǰ������������ʱ��ע������
				��ͣ���� = ���ٶȻ�����ֵ + ��׼��ͣ����
				��ʱ����� F = mg
				����Ҫ���a�ļ��ٶ�ʱ������� F1=m(g+a)
				F1/F = 1 + a/g
				��˴�ʱӦ�������ͣ����*(1 + a/g)
				**************************************/
				Total_Controller.Height_Acce_Control.Control_OutPut+=(Thr_Hover_Default+Total_Controller.Height_Acce_Control.Integrate-Thr_Start)
																													*(Total_Controller.Height_Acce_Control.Expect/980);
			}
			break;
	}	
	 Throttle=Int_Sort(High_Hold_Throttle+Total_Controller.Height_Acce_Control.Control_OutPut);//������Դ�ڸ߶��ٶȿ��������
}





void ncq_control_althold()
{
  static uint8_t move_flag=0;
  static systime althold_dt;
  uint8_t alt_fixed_flag=0;
  Get_Systime(&althold_dt);
  //����������������ڴ��ڿ�������10��
  if(0.001f*althold_dt.period>=10*WP_Duty_Dt)
  {
 	 //���1:���λ��ߴ�����ģʽ���뱾ģʽ
	 //���2:ϵͳ���ȳ�ʱ����ϵͳ��ƺ�������£�����������ܷ���
	 alt_fixed_flag=1;
  }
	 
  Thr_Scale_Set(&RC_Data.cal[2]);
  High_Hold_Throttle=Thr_Hover_Default;
  /*�߶ȿ�������1��*/
  /********
      **
      **
      **
      **
      **
  ********/
  /*******************************�߶ȿ�������ʼ****************************************/
  /****************���ߣ��߶�λ�û�+�ٶȻ�+���ٶȻ����������ڷֱ�Ϊ8ms��4ms��4ms*******************/
  if(RC_Data.rc_rpyt[RC_THR]==0)
  {
    //�߶�λ�û���������ٶ�����
    if(Total_Controller.Height_Position_Control.Expect==0||alt_fixed_flag==1)//���Ż��к�ֻ����һ��
    {
      Total_Controller.Height_Position_Control.Expect=NamelessQuad.Position[_UP];//���Ż��к󣬸��¸߶�����
    }
      Total_Controller.Height_Position_Control.FeedBack=NamelessQuad.Position[_UP];//����
      PID_Control(&Total_Controller.Height_Position_Control,0.005f);//���θ߶�λ�ÿ�����
      //�ڻ��ٶ�����
      Total_Controller.Height_Speed_Control.Expect=Total_Controller.Height_Position_Control.Control_OutPut;
			move_flag=0;
#ifdef ALT_VEL_FEEDFORWARD_ENABLE  //�ٶ�ǰ��������
      Alt_Vel_Target=Total_Controller.Height_Speed_Control.Expect;//�����ٶ�����
      ALT_VEL_FEEDFORWARD_ENABLE_Delta=(Alt_Vel_Target-Last_Alt_Vel_Target)/(1*Delta);//�ٶ������仯��
      ALT_VEL_FEEDFORWARD_ENABLE_Output=ALT_VEL_FEEDFORWARD_ENABLE_Rate*ALT_VEL_FEEDFORWARD_ENABLE_Delta;
      Last_Alt_Vel_Target=Alt_Vel_Target;//�ϴ��ٶ�����
#endif   
	}
  else if(RC_Data.rc_rpyt[RC_THR]>0)//���������ٶ�����
  {
    //���Ÿ����ơ������ٶ�����
    Total_Controller.Height_Speed_Control.Expect=RC_Data.rc_rpyt[RC_THR];//��������ٶ�300cm/s
    Total_Controller.Height_Position_Control.Expect=0;//λ�û�������0
    move_flag=1;
  }
  else if(RC_Data.rc_rpyt[RC_THR]<0)//�����½��ٶ�����
  {
    //���Ÿ����ơ������ٶ�����
    Total_Controller.Height_Speed_Control.Expect=RC_Data.rc_rpyt[RC_THR];//����½��ٶ�200cm/s
    Total_Controller.Height_Position_Control.Expect=0;//λ�û�������0
    move_flag=2;
  }
 
  /*�߶ȿ�������2��*/
  /********
					*
					*
   * * * *
  *
  *
  ********/
  
  /*******************************��ֱ�ٶȿ�������ʼ*******************************************************************/
    Total_Controller.Height_Speed_Control.FeedBack=NamelessQuad.Speed[_UP];//�ߵ��ٶȹ��Ƹ��ٶȷ���
		PID_Control(&Total_Controller.Height_Speed_Control,0.005f);//���θ߶��ٶȿ���
	  //ADRC_TD_Control(&ADRC_Alt_Vel_Controller,Total_Controller.Height_Speed_Control.Expect,Total_Controller.Height_Speed_Control.FeedBack);
	  //Total_Controller.Height_Speed_Control.Control_OutPut=ADRC_Alt_Vel_Controller.u;
  /*******************************�����½��������������ٶ��޷���������*******************************************************************/ 
  if(move_flag==0)
	{
    Total_Controller.Height_Speed_Control.Control_OutPut=constrain_float(Total_Controller.Height_Speed_Control.Control_OutPut,
                                                                       -SDK_Mild_Acceleration_Max,SDK_Mild_Acceleration_Max);  
	}		
  else if(move_flag==1)//�������Ƹ�ʱ���ٶȿ�����������������������½����ٶȽ��зֱ��޷���ȷ�������½�ʱ��̬ƽ��
  {
    Total_Controller.Height_Speed_Control.Control_OutPut=constrain_float(Total_Controller.Height_Speed_Control.Control_OutPut,
                                                                       -Climb_Up_Acceleration_Max,Climb_Up_Acceleration_Max);                                                                
  }
  else if(move_flag==2)
  {
    Total_Controller.Height_Speed_Control.Control_OutPut=constrain_float(Total_Controller.Height_Speed_Control.Control_OutPut,
                                                                       -Climb_Down_Acceleration_Max,Climb_Down_Acceleration_Max);
  }  
  /*******************************��ֱ�ٶȿ���������******************************************************************/
  
  /*�߶ȿ�������3��*/
  /********
         **
         **
      **    
         **
         **
  ********/
  
  /*******************************��ֱ���ٶȿ�������ʼ******************************************************************/
  Total_Controller.Height_Acce_Control.Expect=Total_Controller.Height_Speed_Control.Control_OutPut;//���ٶ�����
#ifdef ALT_VEL_FEEDFORWARD_ENABLE  //�ٶ�ǰ��������
  Total_Controller.Height_Acce_Control.Expect+=ALT_VEL_FEEDFORWARD_ENABLE_Output;//����ǰ�����������
#endif
  Total_Controller.Height_Acce_Control.FeedBack=Acceleration_Feedback[_UP];//���ٶȷ���
	PID_Control(&Total_Controller.Height_Acce_Control,0.005f);//���θ߶ȼ��ٶȿ���
  /**************************************
  ���ٶȻ�ǰ������������ʱ��ע������
  ��ͣ���� = ���ٶȻ�����ֵ + ��׼��ͣ����
  ��ʱ����� F = mg
  ����Ҫ���a�ļ��ٶ�ʱ������� F1=m(g+a)
  F1/F = 1 + a/g
  ��˴�ʱӦ�������ͣ����*(1 + a/g)
  **************************************/
  Total_Controller.Height_Acce_Control.Control_OutPut+=(High_Hold_Throttle+Total_Controller.Height_Acce_Control.Integrate-Thr_Start)*
    (Total_Controller.Height_Acce_Control.Expect/980);
  /*******************************��ֱ���ٶȿ���������******************************************************************/
  Throttle=Int_Sort(High_Hold_Throttle+Total_Controller.Height_Acce_Control.Control_OutPut);//������Դ�ڸ߶ȼ��ٶȿ��������	
  /*****************************************�߶ȿ������������������ſ�����***********************************************************/
}


/* Copyright (c)  2018-2025 Wuhan Nameless Innovation Technology Co.,Ltd. All rights reserved.*/



