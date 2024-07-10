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
#include "OpticalFlow_Control.h"


extern Vector2f OpticalFlow_Position;
extern Vector2f OpticalFlow_Speed;
extern Vector2f OpticalFlow_Speed_Err;
extern Vector2f OpticalFlow_Position_Err;

uint8_t  OpticalFlow_Pos_Ctrl_Recode=1;
Vector2f OpticalFlow_Pos_Ctrl_Expect={0};
Vector2f OpticalFlow_Pos_Ctrl_Err={0};
Vector2f OpticalFlow_Pos_Ctrl_Integrate={0};
Vector2f OpticalFlow_Pos_Ctrl_Output={0};

Vector2f OpticalFlow_Vel_Ctrl_Expect={0};
Vector2f OpticalFlow_Vel_Ctrl_Err={0};
Vector2f OpticalFlow_Vel_Ctrl_Integrate={0};
Vector2f OpticalFlow_Vel_Ctrl_Output={0};

void OpticalFlow_SINS_Reset(void)
{ 
  OpticalFlow_SINS.Position[_EAST]=0;
  OpticalFlow_SINS.Speed[_EAST]=0;
  OpticalFlow_SINS.Position[_NORTH]=0;
  OpticalFlow_SINS.Speed[_NORTH]=0;
  
	OpticalFlow_Position.x=0;
	OpticalFlow_Position.y=0;

	VIO_SINS.Position[_EAST] = current_state.position_x;
	VIO_SINS.Position[_NORTH]= current_state.position_y;
	
	VIO_SINS.Speed[_EAST]=0;
	VIO_SINS.Speed[_NORTH]=0;
	
	correct[0].acc=0;
	correct[1].acc=0;
	correct[0].vel=0;
	correct[1].vel=0;
	correct[0].pos=0;
	correct[1].pos=0;
}


void SDK_Pos_Ctrl_Reset(void)
{ 
	Total_Controller.SDK_Pitch_Position_Control.Integrate=0;
	Total_Controller.SDK_Pitch_Position_Control.Last_Err=0;
	Total_Controller.SDK_Pitch_Position_Control.Err=0;
	Total_Controller.SDK_Pitch_Position_Control.Last_Err_LPF=0;		

	Total_Controller.SDK_Roll_Position_Control.Integrate=0;
	Total_Controller.SDK_Roll_Position_Control.Last_Err=0;
	Total_Controller.SDK_Roll_Position_Control.Err=0;
	Total_Controller.SDK_Roll_Position_Control.Last_Err_LPF=0;
	
	PID_LPF_Reset(&Total_Controller.SDK_Roll_Position_Control,SDK_Roll_Position_Controler);
	PID_LPF_Reset(&Total_Controller.SDK_Pitch_Position_Control,SDK_Pitch_Position_Controler);
}

void OpticalFlow_Ctrl_Reset(void)
{ 
  OpticalFlow_Vel_Ctrl_Integrate.x=0.0f;
  OpticalFlow_Vel_Ctrl_Integrate.y=0.0f;
  OpticalFlow_Pos_Ctrl_Integrate.x=0.0f;
  OpticalFlow_Pos_Ctrl_Integrate.y=0.0f;
  OpticalFlow_Pos_Ctrl_Expect.x=0;
  OpticalFlow_Pos_Ctrl_Expect.y=0;
	OpticalFlow_Pos_Ctrl_Recode=1;
	OpticalFlow_Vel_Ctrl_Expect.x= 0;//�ٶ�����
	OpticalFlow_Vel_Ctrl_Expect.y= 0;
	
	SDK_Pos_Ctrl_Reset();
}




float OpticalFlow_Expect_Speed_Mapping(float input,uint16_t input_max,float output_max)
{
  float output_speed=0;
  float temp_scale=(float)(input/input_max);
  temp_scale=constrain_float(temp_scale,-1.0f, 1.0f);
  if(temp_scale>=0) output_speed=(float)(output_max*temp_scale*temp_scale);
  else output_speed=(float)(-output_max*temp_scale*temp_scale); 
  return output_speed;
}

void OpticalFlow_Set_Target_Point(Vector2f target)
{
  OpticalFlow_Pos_Ctrl_Expect.x=target.x;
  OpticalFlow_Pos_Ctrl_Expect.y=target.y;
}

void OpticalFlow_Set_Target_Vel(Vector2f target)
{
  OpticalFlow_Vel_Ctrl_Expect.x=target.x;
  OpticalFlow_Vel_Ctrl_Expect.y=target.y;
}

//////////////////////////////////////////////////////////////////////////


bool OPT_Is_Fix(void)
{
 if(Sensor_Flag.Ground_Health==1
	 &&(opt_data.valid==1||current_state.valid==1))
 {
	 return true;
 }
 else 
 {
	 return false;
 }
}



void OpticalFlow_Pos_Control(void)
{
	if(OPT_Is_Fix()==false)//OPT��λ״̬δ��������̬����ֱ����Դ��ң��������
	{
		Flight.roll_outer_control_output =RC_Data.rc_rpyt[RC_ROLL];
		Flight.pitch_outer_control_output=RC_Data.rc_rpyt[RC_PITCH];	
		return ;
	}
	
  static uint16_t OpticalFlow_Pos_Ctrl_Cnt=0;
  OpticalFlow_Pos_Ctrl_Cnt++;
  if(OpticalFlow_Pos_Ctrl_Cnt>=4)//20ms����һ���ٶȣ���������Ƶ�ʹ���ϵͳ��Ӧ������
  {
    //����λ��ƫ��  
    OpticalFlow_Pos_Ctrl_Err.x=OpticalFlow_Pos_Ctrl_Expect.x-OpticalFlow_SINS.Position[_EAST];
    OpticalFlow_Pos_Ctrl_Err.y=OpticalFlow_Pos_Ctrl_Expect.y-OpticalFlow_SINS.Position[_NORTH];
		
    OpticalFlow_Pos_Ctrl_Err.x=constrain_float(OpticalFlow_Pos_Ctrl_Err.x,-Total_Controller.Optical_Position_Control.Err_Max,Total_Controller.Optical_Position_Control.Err_Max);//100
    OpticalFlow_Pos_Ctrl_Err.y=constrain_float(OpticalFlow_Pos_Ctrl_Err.y,-Total_Controller.Optical_Position_Control.Err_Max,Total_Controller.Optical_Position_Control.Err_Max);		
		
    //����λ�ÿ������
    OpticalFlow_Pos_Ctrl_Output.x=Total_Controller.Optical_Position_Control.Kp*OpticalFlow_Pos_Ctrl_Err.x;
    OpticalFlow_Pos_Ctrl_Output.y=Total_Controller.Optical_Position_Control.Kp*OpticalFlow_Pos_Ctrl_Err.y;
    OpticalFlow_Pos_Ctrl_Cnt=0;
  }
}

void OpticalFlow_Pos_Control_VIO(void)
{
	if(OPT_Is_Fix()==false)//OPT��λ״̬δ��������̬����ֱ����Դ��ң��������
	{
		Flight.roll_outer_control_output =RC_Data.rc_rpyt[RC_ROLL];
		Flight.pitch_outer_control_output=RC_Data.rc_rpyt[RC_PITCH];	
		return ;
	}
	
  static uint16_t OpticalFlow_Pos_Ctrl_Cnt=0;
  OpticalFlow_Pos_Ctrl_Cnt++;
  if(OpticalFlow_Pos_Ctrl_Cnt>=4)//20ms����һ���ٶȣ���������Ƶ�ʹ���ϵͳ��Ӧ������
  {
    //����λ��ƫ��  
    OpticalFlow_Pos_Ctrl_Err.x=OpticalFlow_Pos_Ctrl_Expect.x-VIO_SINS.Position[_EAST];
    OpticalFlow_Pos_Ctrl_Err.y=OpticalFlow_Pos_Ctrl_Expect.y-VIO_SINS.Position[_NORTH];
		
    OpticalFlow_Pos_Ctrl_Err.x=constrain_float(OpticalFlow_Pos_Ctrl_Err.x,-Total_Controller.Optical_Position_Control.Err_Max,Total_Controller.Optical_Position_Control.Err_Max);//100
    OpticalFlow_Pos_Ctrl_Err.y=constrain_float(OpticalFlow_Pos_Ctrl_Err.y,-Total_Controller.Optical_Position_Control.Err_Max,Total_Controller.Optical_Position_Control.Err_Max);		

		//��������ϵ�»���Pitch��Roll������λ��ƫ��
		from_vio_to_body_frame(Total_Controller.Optical_Position_Control.Kp*OpticalFlow_Pos_Ctrl_Err.x,
													 Total_Controller.Optical_Position_Control.Kp*OpticalFlow_Pos_Ctrl_Err.y,
													 &OpticalFlow_Pos_Ctrl_Output.x,
													 &OpticalFlow_Pos_Ctrl_Output.y,
													 WP_AHRS.Yaw);
    OpticalFlow_Pos_Ctrl_Cnt=0;
  }
}


systime opt_speed_dt;
void OpticalFlow_Vel_Control(Vector2f target)
{
	Get_Systime(&opt_speed_dt);
	//����������������ڴ��ڿ�������10��
	if(0.001f*opt_speed_dt.period>=20*WP_Duty_Dt)
	{
		//���1:���δ�����ģʽ���뱾ģʽ
		//���2:ϵͳ���ȳ�ʱ����ϵͳ��ƺ�������£�����������ܷ���
	}
	
	
	if(OPT_Is_Fix()==false)//OPT��λ״̬δ��������̬����ֱ����Դ��ң��������
	{
		Flight.roll_outer_control_output =RC_Data.rc_rpyt[RC_ROLL];
		Flight.pitch_outer_control_output=RC_Data.rc_rpyt[RC_PITCH];	
		return ;
	}	
	
  static uint16_t OpticalFlow_Vel_Ctrl_Cnt=0;
  OpticalFlow_Vel_Ctrl_Expect.x= target.x;//�ٶ�����
  OpticalFlow_Vel_Ctrl_Expect.y= target.y;
  OpticalFlow_Vel_Ctrl_Cnt++;
  if(OpticalFlow_Vel_Ctrl_Cnt>=2)//10ms����һ���ٶȣ���������Ƶ�ʹ���ϵͳ��Ӧ������
  {
    OpticalFlow_Vel_Ctrl_Err.x=constrain_float(OpticalFlow_Vel_Ctrl_Expect.x-OpticalFlow_SINS.Speed[_EAST],-Total_Controller.Optical_Speed_Control.Err_Max,Total_Controller.Optical_Speed_Control.Err_Max);//30
    OpticalFlow_Vel_Ctrl_Err.y=constrain_float(OpticalFlow_Vel_Ctrl_Expect.y-OpticalFlow_SINS.Speed[_NORTH] ,-Total_Controller.Optical_Speed_Control.Err_Max,Total_Controller.Optical_Speed_Control.Err_Max);
    
    //if(ABS(OpticalFlow_Vel_Ctrl_Err.x)<=Total_Controller.Optical_Speed_Control.Integrate_Separation_Err)  
    OpticalFlow_Vel_Ctrl_Integrate.x+=Total_Controller.Optical_Speed_Control.Ki*OpticalFlow_Vel_Ctrl_Err.x;//0.1  15
    //if(ABS(OpticalFlow_Vel_Ctrl_Err.y)<=Total_Controller.Optical_Speed_Control.Integrate_Separation_Err)  
    OpticalFlow_Vel_Ctrl_Integrate.y+=Total_Controller.Optical_Speed_Control.Ki*OpticalFlow_Vel_Ctrl_Err.y;
    
    OpticalFlow_Vel_Ctrl_Integrate.x=constrain_float(OpticalFlow_Vel_Ctrl_Integrate.x,-Total_Controller.Optical_Speed_Control.Integrate_Max,Total_Controller.Optical_Speed_Control.Integrate_Max);
    OpticalFlow_Vel_Ctrl_Integrate.y=constrain_float(OpticalFlow_Vel_Ctrl_Integrate.y,-Total_Controller.Optical_Speed_Control.Integrate_Max,Total_Controller.Optical_Speed_Control.Integrate_Max);
    
    OpticalFlow_Vel_Ctrl_Output.x=OpticalFlow_Vel_Ctrl_Integrate.x+Total_Controller.Optical_Speed_Control.Kp*OpticalFlow_Vel_Ctrl_Err.x;//4.5
    OpticalFlow_Vel_Ctrl_Output.y=OpticalFlow_Vel_Ctrl_Integrate.y+Total_Controller.Optical_Speed_Control.Kp*OpticalFlow_Vel_Ctrl_Err.y;
    
    accel_target.y=-constrain_float(OpticalFlow_Vel_Ctrl_Output.y,-Total_Controller.Optical_Speed_Control.Control_OutPut_Limit,Total_Controller.Optical_Speed_Control.Control_OutPut_Limit);//450
    accel_target.x=-constrain_float(OpticalFlow_Vel_Ctrl_Output.x,-Total_Controller.Optical_Speed_Control.Control_OutPut_Limit,Total_Controller.Optical_Speed_Control.Control_OutPut_Limit);//�����˶����ٶ�
    
		ENU_Desired_Accel_Transform_Angle(accel_target,&angle_target);//�����˶����ٶ�ת������̬���
		
		Flight.roll_outer_control_output =angle_target.x;
		Flight.pitch_outer_control_output=angle_target.y;
				
    OpticalFlow_Vel_Ctrl_Cnt=0;
  }
}


void Horizontal_Control(uint8_t force_brake_flag)
{
	/////////////////////////////////SDKλ�ÿ��ƿ�ʼ////////////////////////////////////////						
	if(ngs_sdk.update_flag==true)
	{
			if(ngs_sdk.move_flag.sdk_front_flag==true||ngs_sdk.move_flag.sdk_behind_flag==true)//ǰ��/����
			{
					OpticalFlow_Pos_Ctrl_Expect.x=OpticalFlow_SINS.Position[_EAST];										  //���ұ���
					OpticalFlow_Pos_Ctrl_Expect.y=OpticalFlow_SINS.Position[_NORTH]+ngs_sdk.f_distance;	//ǰ�����		
					Total_Controller.Height_Position_Control.Expect=NamelessQuad.Position[_UP];	 				  //���±���																
			}
			
			if(ngs_sdk.move_flag.sdk_left_flag==true||ngs_sdk.move_flag.sdk_right_flag==true)//����/����
			{
				  OpticalFlow_Pos_Ctrl_Expect.x=OpticalFlow_SINS.Position[_EAST]+ngs_sdk.f_distance;   //���ҵ���
					OpticalFlow_Pos_Ctrl_Expect.y=OpticalFlow_SINS.Position[_NORTH];										 //ǰ�󱣳�			
					Total_Controller.Height_Position_Control.Expect=NamelessQuad.Position[_UP];			 	     //���±���														
			}
			
			if(ngs_sdk.move_flag.sdk_up_flag==true||ngs_sdk.move_flag.sdk_down_flag==true)//����/�½�
			{
					OpticalFlow_Pos_Ctrl_Expect.x=OpticalFlow_SINS.Position[_EAST];															//ǰ�󱣳�
					OpticalFlow_Pos_Ctrl_Expect.y=OpticalFlow_SINS.Position[_NORTH];																//���ұ���
				  Total_Controller.Height_Position_Control.Expect=NamelessQuad.Position[_UP]+ngs_sdk.f_distance; //���µ���										
			}
			ngs_sdk.move_flag.sdk_front_flag=false;
			ngs_sdk.move_flag.sdk_behind_flag=false;
			ngs_sdk.move_flag.sdk_left_flag=false;
			ngs_sdk.move_flag.sdk_right_flag=false;
			ngs_sdk.move_flag.sdk_up_flag=false;
			ngs_sdk.move_flag.sdk_down_flag=false;							
			ngs_sdk.update_flag=false;
	}

	
	if(Roll_Control==0&&Pitch_Control==0)//��ˮƽң��������
	{ 
		/**************************����λ�ÿ�����************************************/
		if(OpticalFlow_Pos_Ctrl_Recode==1||force_brake_flag==1)
		{
			OpticalFlow_Pos_Ctrl_Expect.x=OpticalFlow_SINS.Position[_EAST];
			OpticalFlow_Pos_Ctrl_Expect.y=OpticalFlow_SINS.Position[_NORTH];
			OpticalFlow_Pos_Ctrl_Recode=0;
		}	
		OpticalFlow_Pos_Control();
		/***********��ֻ��Ҫ�ٶȿ���ʱ����������ע�ͣ����޵���ʱ��*************/
		 //OpticalFlow_Pos_Ctrl_Output.x=0;
		 //OpticalFlow_Pos_Ctrl_Output.y=0;
	}
	else
	{
		OpticalFlow_Pos_Ctrl_Output.x=OpticalFlow_Expect_Speed_Mapping(-RC_Data.rc_rpyt[RC_ROLL],Pit_Rol_Max,OpticalFlow_Speed_Control_Max);
		OpticalFlow_Pos_Ctrl_Output.y=OpticalFlow_Expect_Speed_Mapping(-RC_Data.rc_rpyt[RC_PITCH],Pit_Rol_Max,OpticalFlow_Speed_Control_Max); 
		OpticalFlow_Pos_Ctrl_Expect.x=0;
		OpticalFlow_Pos_Ctrl_Expect.y=0;
		OpticalFlow_Pos_Ctrl_Recode=1;
	}
	/**************************����ģ�͵ļ��ٶ�-��̬��ӳ�䣬���ֱ�Ӹ���̬��������������20������************************************/
	OpticalFlow_Vel_Control(OpticalFlow_Pos_Ctrl_Output);//�ٶ�����
}


////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void Horizontal_Control_VIO_Pure(uint8_t force_brake_flag)
{
	if(Roll_Control==0
		 &&Pitch_Control==0)//��ˮƽң��������
	{ 
		/**************************����λ�ÿ�����************************************/
		if(OpticalFlow_Pos_Ctrl_Recode==1||force_brake_flag==1)
		{
			OpticalFlow_Pos_Ctrl_Expect.x=VIO_SINS.Position[_EAST];
			OpticalFlow_Pos_Ctrl_Expect.y=VIO_SINS.Position[_NORTH];
			OpticalFlow_Pos_Ctrl_Recode=0;
		}
		OpticalFlow_Pos_Control_VIO();
	}
	else
	{
		OpticalFlow_Pos_Ctrl_Output.x=OpticalFlow_Expect_Speed_Mapping(-RC_Data.rc_rpyt[RC_ROLL],Pit_Rol_Max,OpticalFlow_Speed_Control_Max);
		OpticalFlow_Pos_Ctrl_Output.y=OpticalFlow_Expect_Speed_Mapping(-RC_Data.rc_rpyt[RC_PITCH],Pit_Rol_Max,OpticalFlow_Speed_Control_Max); 				
		OpticalFlow_Pos_Ctrl_Expect.x=0;
		OpticalFlow_Pos_Ctrl_Expect.y=0;
		OpticalFlow_Pos_Ctrl_Recode=1;
	}
	/**************************����ģ�͵ļ��ٶ�-��̬��ӳ�䣬���ֱ�Ӹ���̬��������������20������************************************/
	OpticalFlow_Vel_Control(OpticalFlow_Pos_Ctrl_Output);//�ٶ�����
}
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void ngs_nav_ctrl_finish_predict(void)
{
	//�ж��Ƿ񵽴�Ŀ�꺽��λ��
	if(ngs_nav_ctrl.ctrl_finish_flag==0)
	{
		if(ngs_nav_ctrl.cnt<10)//����50ms����
		{
			ngs_nav_ctrl.dis_cm=pythagorous3(OpticalFlow_Pos_Ctrl_Err.x,OpticalFlow_Pos_Ctrl_Err.y,Total_Controller.Height_Position_Control.Err);
			if(ngs_nav_ctrl.dis_cm<=ngs_nav_ctrl.dis_limit_cm)	ngs_nav_ctrl.cnt++;
			else ngs_nav_ctrl.cnt/=2;
		}
		else
		{
			ngs_nav_ctrl.ctrl_finish_flag=1;
			ngs_nav_ctrl.cnt=0;
			NCLink_Send_Check_Flag[14]=1;//������Ϻ󣬷���Ӧ�����λ��
			send_check_back=3;//������Ϻ󣬷���Ӧ���ROS
		}
	}
}

void Horizontal_Control_VIO(uint8_t force_brake_flag)
{
	/////////////////////////////////SDKλ�ÿ��ƿ�ʼ////////////////////////////////////////						
	if(ngs_sdk.update_flag==true)
	{
		if(ngs_sdk.move_flag.sdk_front_flag==true||ngs_sdk.move_flag.sdk_behind_flag==true)//ǰ��/����
		{
				OpticalFlow_Pos_Ctrl_Expect.x=VIO_SINS.Position[_EAST];										  //���ұ���
				OpticalFlow_Pos_Ctrl_Expect.y=VIO_SINS.Position[_NORTH]+ngs_sdk.f_distance;	//ǰ�����		
				Total_Controller.Height_Position_Control.Expect=NamelessQuad.Position[_UP];	 				  //���±���																
		}
		
		if(ngs_sdk.move_flag.sdk_left_flag==true||ngs_sdk.move_flag.sdk_right_flag==true)//����/����
		{
				OpticalFlow_Pos_Ctrl_Expect.x=VIO_SINS.Position[_EAST]-ngs_sdk.f_distance;   //���ҵ���
				OpticalFlow_Pos_Ctrl_Expect.y=VIO_SINS.Position[_NORTH];										 //ǰ�󱣳�			
				Total_Controller.Height_Position_Control.Expect=NamelessQuad.Position[_UP];			 	     //���±���														
		}
		
		if(ngs_sdk.move_flag.sdk_up_flag==true||ngs_sdk.move_flag.sdk_down_flag==true)//����/�½�
		{
				OpticalFlow_Pos_Ctrl_Expect.x=VIO_SINS.Position[_EAST];															//ǰ�󱣳�
				OpticalFlow_Pos_Ctrl_Expect.y=VIO_SINS.Position[_NORTH];																//���ұ���
				Total_Controller.Height_Position_Control.Expect=NamelessQuad.Position[_UP]+ngs_sdk.f_distance; //���µ���										
		}
		ngs_sdk.move_flag.sdk_front_flag=false;
		ngs_sdk.move_flag.sdk_behind_flag=false;
		ngs_sdk.move_flag.sdk_left_flag=false;
		ngs_sdk.move_flag.sdk_right_flag=false;
		ngs_sdk.move_flag.sdk_up_flag=false;
		ngs_sdk.move_flag.sdk_down_flag=false;							
		ngs_sdk.update_flag=false;
	}
	
	if(ngs_nav_ctrl.update_flag==1)//����վ/ROS���ʹ��ڿ���ָ��
	{
		Horizontal_Navigation(ngs_nav_ctrl.x,
												  ngs_nav_ctrl.y,
											    ngs_nav_ctrl.z,
													ngs_nav_ctrl.nav_mode,
													ngs_nav_ctrl.frame_id);
		ngs_nav_ctrl.update_flag=0;
		ngs_nav_ctrl.ctrl_finish_flag=0;
		ngs_nav_ctrl.cnt=0;
	}
	
	switch(ngs_nav_ctrl.nav_mode)
	{
		case RELATIVE_MODE:
		case GLOBAL_MODE://����λ�ÿ���
		{
			Horizontal_Control_VIO_Pure(force_brake_flag);
			ngs_nav_ctrl_finish_predict();//����λ�ÿ��ƽ����ж�	
		}
		break;
		case CMD_VEL_MODE://�ٶȿ���ģʽ
		{
			if(ngs_nav_ctrl.x!=0||ngs_nav_ctrl.y!=0)//��ˮƽ�ٶȲ�Ϊ0ʱ���ٶȿ��Ʋ���Ч
			{	
					Vector2f target;		
					target.x=OpticalFlow_Expect_Speed_Mapping(-RC_Data.rc_rpyt[RC_ROLL],Pit_Rol_Max,OpticalFlow_Speed_Control_Max);
					target.y=OpticalFlow_Expect_Speed_Mapping(-RC_Data.rc_rpyt[RC_PITCH],Pit_Rol_Max,OpticalFlow_Speed_Control_Max);
					if(Roll_Control==0&&Pitch_Control==0)//��ˮƽң��������
					{
						if(ngs_nav_ctrl.cmd_vel_update==1)//�ٶȿ���ִ��ʱ�����
						{
							if(ngs_nav_ctrl.cmd_vel_during_cnt>0)
							{
								ngs_nav_ctrl.cmd_vel_during_cnt=ngs_nav_ctrl.cmd_vel_during_cnt-1;
								Vector2f target_tmp;
								target_tmp.x=ngs_nav_ctrl.cmd_vel_x;
								target_tmp.y=ngs_nav_ctrl.cmd_vel_y;
								OpticalFlow_Vel_Control(target_tmp);
								ngs_nav_ctrl.cmd_vel_suspend_flag=1;
							}
							else	
							{
								ngs_nav_ctrl.cmd_vel_update=0;//�ٶȿ������
								ngs_nav_ctrl.nav_mode=TRANSITION_MODE;//������Ϻ󣬽������ģʽ
								ngs_nav_ctrl.cmd_vel_suspend_flag=0;//������ֹ
								NCLink_Send_Check_Flag[14]=1;//������Ϻ󣬷���Ӧ�����λ��
								send_check_back=3;//������Ϻ󣬷���Ӧ���ROS
							}						
						}
						else OpticalFlow_Vel_Control(target);
					}
					else 
					{
						OpticalFlow_Vel_Control(target);
						//�����ֶ���˲�����ͬ������������ֹ
						ngs_nav_ctrl.cmd_vel_update=0;//ֻҪ����ң������˲�����ǿ�ƽ��������ٶȿ���
						ngs_nav_ctrl.nav_mode=TRANSITION_MODE;//������Ϻ󣬽������ģʽ
						ngs_nav_ctrl.cmd_vel_during_cnt=0;
						ngs_nav_ctrl.cmd_vel_suspend_flag=0;//��ͬΪ������ֹ
					}
			}
			else//����ˮƽ��Ȼ����λ�ÿ��ƣ�ֻ��Ӧƫ������
			{
					if(ngs_nav_ctrl.cmd_vel_update==1)//�ٶȿ���ִ��ʱ�����
					{
						if(ngs_nav_ctrl.cmd_vel_during_cnt>0)
						{
							ngs_nav_ctrl.cmd_vel_during_cnt=ngs_nav_ctrl.cmd_vel_during_cnt-1;
							//�����ڴ˿�ת��ʵ�ʲ�ִ�о�������
							//ֻ���ⲿ�����У���Ӧƫ�����ٶȿ���
							//����������0���˳��ٶȿ���ģʽ
						}
						else	
						{
							ngs_nav_ctrl.cmd_vel_update=0;//�ٶȿ������
							ngs_nav_ctrl.nav_mode=TRANSITION_MODE+1;//������Ϻ󣬽�����ɵ���һģʽ��������ȫ��û��ˢ��ˮƽ��ͣ��
							NCLink_Send_Check_Flag[14]=1;//������Ϻ󣬷���Ӧ�����λ��
							send_check_back=3;//������Ϻ󣬷���Ӧ���ROS
							ngs_nav_ctrl.cmd_vel_suspend_flag=0;//������ֹ
						}						
					}
					
					if(ngs_nav_ctrl.cmd_vel_suspend_flag==1)//�����ָ����ǣ���ǰ��ֹ����ָ����ٶȿ���ʱ���ٴν�������ѡˢ����ͣ��
					{
						Horizontal_Control_VIO_Pure(1);
						ngs_nav_ctrl.cmd_vel_suspend_flag=0;
					}
					else Horizontal_Control_VIO_Pure(force_brake_flag);
					
					//�����ֶ���˲�����ͬ������������ֹ
					if(Roll_Control!=0||Pitch_Control!=0)//����ң��������
					{
							ngs_nav_ctrl.cmd_vel_update=0;//�ٶȿ������
							ngs_nav_ctrl.nav_mode=TRANSITION_MODE+1;//������Ϻ󣬽�����ɵ���һģʽ��������ȫ��û��ˢ��ˮƽ��ͣ��
							NCLink_Send_Check_Flag[14]=1;//������Ϻ󣬷���Ӧ�����λ��
							send_check_back=3;//������Ϻ󣬷���Ӧ���ROS
							ngs_nav_ctrl.cmd_vel_suspend_flag=0;//��ͬΪ������ֹ					
					}
			}
		}
		break;
		case TRANSITION_MODE://���Ź���ģʽ��ֻ��ִ��һ�Σ�����ˢ����ͣλ��
		{
			Horizontal_Control_VIO_Pure(1);  //�������ģʽ��ˢ����ͣλ������
			ngs_nav_ctrl.nav_mode++;			   //�ԼӺ��´λ������ͨ��������ģʽ
		}
		break;
		default:Horizontal_Control_VIO_Pure(force_brake_flag);//��ͨ��������
	}
}




void OpticalFlow_X_Vel_Control(float target_x)//��ͷ���ΪX+
{ 
  static uint16_t OpticalFlow_Vel_Ctrl_Cnt=0;
  OpticalFlow_Vel_Ctrl_Expect.x= target_x;//�ٶ�����
  OpticalFlow_Vel_Ctrl_Cnt++;
  if(OpticalFlow_Vel_Ctrl_Cnt>=2)//10ms����һ���ٶȣ���������Ƶ�ʹ���ϵͳ��Ӧ������
  {
    OpticalFlow_Vel_Ctrl_Err.x=constrain_float(OpticalFlow_Vel_Ctrl_Expect.x-OpticalFlow_SINS.Speed[_EAST],-Total_Controller.Optical_Speed_Control.Err_Max,Total_Controller.Optical_Speed_Control.Err_Max);//30

    //if(ABS(OpticalFlow_Vel_Ctrl_Err.x)<=Total_Controller.Optical_Speed_Control.Integrate_Separation_Err)  
    OpticalFlow_Vel_Ctrl_Integrate.x+=Total_Controller.Optical_Speed_Control.Ki*OpticalFlow_Vel_Ctrl_Err.x;//0.1  15

    OpticalFlow_Vel_Ctrl_Integrate.x=constrain_float(OpticalFlow_Vel_Ctrl_Integrate.x,-Total_Controller.Optical_Speed_Control.Integrate_Max,Total_Controller.Optical_Speed_Control.Integrate_Max);

    OpticalFlow_Vel_Ctrl_Output.x=OpticalFlow_Vel_Ctrl_Integrate.x+Total_Controller.Optical_Speed_Control.Kp*OpticalFlow_Vel_Ctrl_Err.x;//4.5

		accel_target.x=-constrain_float(OpticalFlow_Vel_Ctrl_Output.x,-Total_Controller.Optical_Speed_Control.Control_OutPut_Limit,Total_Controller.Optical_Speed_Control.Control_OutPut_Limit);//�����˶����ٶ�
    accel_target.y=0;
		
		ENU_Desired_Accel_Transform_Angle(accel_target,&angle_target);//�����˶����ٶ�ת������̬���
		Flight.roll_outer_control_output=angle_target.x;
    OpticalFlow_Vel_Ctrl_Cnt=0;
  }
}


void OpticalFlow_Y_Vel_Control(float target_y)//��ͷǰ��ΪY+
{ 
  static uint16_t OpticalFlow_Vel_Ctrl_Cnt=0;
  OpticalFlow_Vel_Ctrl_Expect.y= target_y;
  OpticalFlow_Vel_Ctrl_Cnt++;
  if(OpticalFlow_Vel_Ctrl_Cnt>=2)//10ms����һ���ٶȣ���������Ƶ�ʹ���ϵͳ��Ӧ������
  {
    OpticalFlow_Vel_Ctrl_Err.y=constrain_float(OpticalFlow_Vel_Ctrl_Expect.y-OpticalFlow_SINS.Speed[_NORTH] ,-Total_Controller.Optical_Speed_Control.Err_Max,Total_Controller.Optical_Speed_Control.Err_Max);
    
    OpticalFlow_Vel_Ctrl_Integrate.y+=Total_Controller.Optical_Speed_Control.Ki*OpticalFlow_Vel_Ctrl_Err.y;
    
    OpticalFlow_Vel_Ctrl_Integrate.y=constrain_float(OpticalFlow_Vel_Ctrl_Integrate.y,-Total_Controller.Optical_Speed_Control.Integrate_Max,Total_Controller.Optical_Speed_Control.Integrate_Max);
    
    OpticalFlow_Vel_Ctrl_Output.y=OpticalFlow_Vel_Ctrl_Integrate.y+Total_Controller.Optical_Speed_Control.Kp*OpticalFlow_Vel_Ctrl_Err.y;
    
		accel_target.x=0;
    accel_target.y=-constrain_float(OpticalFlow_Vel_Ctrl_Output.y,-Total_Controller.Optical_Speed_Control.Control_OutPut_Limit,Total_Controller.Optical_Speed_Control.Control_OutPut_Limit);//450
    
		ENU_Desired_Accel_Transform_Angle(accel_target,&angle_target);//�����˶����ٶ�ת������̬���

		Flight.pitch_outer_control_output=angle_target.y;
		
    OpticalFlow_Vel_Ctrl_Cnt=0;
  }
}


///////////////////////////////////////////////
void OpticalFlow_Control(uint8_t force_brake_flag)
{
	if(OPT_Is_Fix()==false)//OPT��λ״̬δ��������̬����ֱ����Դ��ң��������
	{
		Flight.roll_outer_control_output =RC_Data.rc_rpyt[RC_ROLL];
		Flight.pitch_outer_control_output=RC_Data.rc_rpyt[RC_PITCH];	
		return ;
	}

	static uint8_t miss_fixed_flag=0;
	static systime opt_poshold_dt;
	Get_Systime(&opt_poshold_dt);
	//����������������ڴ��ڿ�������10��
	if(0.001f*opt_poshold_dt.period>=20*WP_Duty_Dt)
	{
		//���1:���δ�����ģʽ���뱾ģʽ
		//���2:ϵͳ���ȳ�ʱ����ϵͳ��ƺ�������£�����������ܷ���
		miss_fixed_flag=1;
	}
	
	if(miss_fixed_flag==1)//֮ǰδ���㶨λ����,δ����Ŀ���
	{
		OpticalFlow_Pos_Ctrl_Expect.x=OpticalFlow_SINS.Position[_EAST];
		OpticalFlow_Pos_Ctrl_Expect.y=OpticalFlow_SINS.Position[_NORTH];
		//��������λ
		OpticalFlow_Ctrl_Reset();
		miss_fixed_flag=0;
	}
	
    if(Roll_Control==0
       &&Pitch_Control==0)//��ˮƽң��������
    {
      /**************************����λ�ÿ�����************************************/
      if(OpticalFlow_Pos_Ctrl_Recode==1)
      {
        if(force_brake_flag==1||(WP_AHRS.rMat[8]>=0.95f
                                 &&pythagorous2(OpticalFlow_SINS.Speed[_EAST],OpticalFlow_SINS.Speed[_NORTH])<=40))//��˻��к󣬸��ݵ�ǰ�ٶȡ�����ж��Ƿ������ͣ
        {
          OpticalFlow_Pos_Ctrl_Expect.x=OpticalFlow_SINS.Position[_EAST];
          OpticalFlow_Pos_Ctrl_Expect.y=OpticalFlow_SINS.Position[_NORTH];
					OpticalFlow_Pos_Ctrl_Recode=0;
        }
        else  //��˻���δ������ͣ����ʱ��ֻ�����ٶȿ��� 
        {
          OpticalFlow_Pos_Ctrl_Output.x=0;
          OpticalFlow_Pos_Ctrl_Output.y=0;
        }
      }
      else  OpticalFlow_Pos_Control();
      
      /***********��ֻ��Ҫ�ٶȿ���ʱ����������ע�ͣ����޵���ʱ��*************/
      //OpticalFlow_Pos_Ctrl_Output.x=0;
      //OpticalFlow_Pos_Ctrl_Output.y=0;
    }
    else
    {
      OpticalFlow_Pos_Ctrl_Output.x=OpticalFlow_Expect_Speed_Mapping(-RC_Data.rc_rpyt[RC_ROLL],Pit_Rol_Max,OpticalFlow_Speed_Control_Max);
      OpticalFlow_Pos_Ctrl_Output.y=OpticalFlow_Expect_Speed_Mapping(-RC_Data.rc_rpyt[RC_PITCH],Pit_Rol_Max,OpticalFlow_Speed_Control_Max); 
      OpticalFlow_Pos_Ctrl_Expect.x=0;
      OpticalFlow_Pos_Ctrl_Expect.y=0;
			OpticalFlow_Pos_Ctrl_Recode=1;
    }
		/**************************����ģ�͵ļ��ٶ�-��̬��ӳ�䣬���ֱ�Ӹ���̬��������������20������************************************/
		OpticalFlow_Vel_Control(OpticalFlow_Pos_Ctrl_Output);//�ٶ�����
}



void OpticalFlow_Control_Pure(uint8_t force_brake_flag)
{
  if(OPT_Is_Fix()==false)//OPT��λ״̬δ��������̬����ֱ����Դ��ң��������
	{
		Flight.roll_outer_control_output =RC_Data.rc_rpyt[RC_ROLL];
		Flight.pitch_outer_control_output=RC_Data.rc_rpyt[RC_PITCH];	
		return ;
	}
	
	if(Optical_Type_Present==3||Optical_Type_Present==4)
	{
		switch(current_state.slam_sensor)
		{
			case NO_SLAM:
			{
				Horizontal_Control(force_brake_flag);
			}
			break;
			case LIDAR_2D_SLAM:
			case T265_SLAM:
			case LOAM:
			{
				Horizontal_Control_VIO(force_brake_flag);	
			};
			break;
			default:			
			{
				Horizontal_Control(force_brake_flag);			
			}		
		}
	}
	else
	{
		Horizontal_Control(force_brake_flag);
	}
}



void Color_Block_Control_Pilot(void)
{
	static uint8_t miss_cnt=1;
	static uint16_t _cnt=0;
	static uint8_t miss_flag=0;
	_cnt++;
	if(_cnt>=20)//100ms
	{	
		_cnt=0;		
	  if(Opv_Top_View_Target.target_ctrl_enable==1)//Ŀ��������
		{			
			miss_cnt=1;
			miss_flag=0;			
			Opv_Top_View_Target.target_ctrl_enable=0;
			Total_Controller.SDK_Roll_Position_Control.Expect=0;
			Total_Controller.SDK_Roll_Position_Control.FeedBack=Opv_Top_View_Target.sdk_target.x;
			PID_Control_SDK_Err_LPF(&Total_Controller.SDK_Roll_Position_Control,Opv_Top_View_Target.trust_flag,0.1f);
			
			Total_Controller.SDK_Pitch_Position_Control.Expect=0;
			Total_Controller.SDK_Pitch_Position_Control.FeedBack=Opv_Top_View_Target.sdk_target.y;
			PID_Control_SDK_Err_LPF(&Total_Controller.SDK_Pitch_Position_Control,Opv_Top_View_Target.trust_flag,0.1f);
			
			OpticalFlow_Pos_Ctrl_Output.x=-Total_Controller.SDK_Roll_Position_Control.Control_OutPut;
			OpticalFlow_Pos_Ctrl_Output.y=-Total_Controller.SDK_Pitch_Position_Control.Control_OutPut;	
		}
		else//��ʧĿ��
		{
		  miss_flag=1;
		}		
	}

	
	if(miss_flag==1)//Ŀ�궪ʧ
	{
		if(miss_cnt==1)//��ʼ��ʧ����Ŀ���������ǰλ�ú󣬽�����ͨ��������
		{
			miss_cnt=2;
			OpticalFlow_Control_Pure(1);//20ms		
		}
		else if(miss_cnt==2)//��ʧ����Ŀ��󣬽�����ͨ��������
		{
			OpticalFlow_Control_Pure(0);//20ms		
		}
	}
  else//Ŀ��δ��ʧ,10ms
	{
		OpticalFlow_Vel_Control(OpticalFlow_Pos_Ctrl_Output);//�ٶȿ�������20ms
	}			
}


void Top_APrilTag_Control_Pilot(void)
{
	static uint8_t miss_cnt=1;
	static uint16_t _cnt=0;
	static uint8_t miss_flag=0;
	_cnt++;
	if(_cnt>=20)//100ms
	{	
		_cnt=0;		
	  if(Opv_Top_View_Target.target_ctrl_enable==1)//Ŀ��������
		{			
			miss_cnt=1;
			miss_flag=0;			
			Opv_Top_View_Target.target_ctrl_enable=0;
			Total_Controller.SDK_Roll_Position_Control.Expect=0;
			Total_Controller.SDK_Roll_Position_Control.FeedBack=Opv_Top_View_Target.sdk_target.x;
			PID_Control_SDK_Err_LPF(&Total_Controller.SDK_Roll_Position_Control,Opv_Top_View_Target.trust_flag,0.1f);
			
			Total_Controller.SDK_Pitch_Position_Control.Expect=0;
			Total_Controller.SDK_Pitch_Position_Control.FeedBack=Opv_Top_View_Target.sdk_target.y;
			PID_Control_SDK_Err_LPF(&Total_Controller.SDK_Pitch_Position_Control,Opv_Top_View_Target.trust_flag,0.1f);
			
			OpticalFlow_Pos_Ctrl_Output.x=-Total_Controller.SDK_Roll_Position_Control.Control_OutPut;
			OpticalFlow_Pos_Ctrl_Output.y=-Total_Controller.SDK_Pitch_Position_Control.Control_OutPut;	
		}
		else//��ʧĿ��
		{
		  miss_flag=1;
		}		
	}

	
	if(miss_flag==1)//Ŀ�궪ʧ
	{
		if(miss_cnt==1)//��ʼ��ʧ����Ŀ���������ǰλ�ú󣬽�����ͨ��������
		{
			miss_cnt=2;
			OpticalFlow_Control_Pure(1);//20ms		
		}
		else if(miss_cnt==2)//��ʧ����Ŀ��󣬽�����ͨ��������
		{
			OpticalFlow_Control_Pure(0);//20ms		
		}
	}
  else//Ŀ��δ��ʧ,10ms
	{
		OpticalFlow_Vel_Control(OpticalFlow_Pos_Ctrl_Output);//�ٶȿ�������20ms
	}			
}


#define Forward_Keep_Distance   100//cm
#define Deadband_Keep_Distance  10
void Front_AprilTag_Control_Pilot(void)
{
	static uint8_t miss_cnt=1;
	static uint16_t _cnt=0;
	static uint8_t miss_flag=0;
	_cnt++;
	if(_cnt>=20)//100ms
	{	
		_cnt=0;		
	  if(Opv_Front_View_Target.target_ctrl_enable==1)//Ŀ��������
		{			
			miss_cnt=1;
			miss_flag=0;			
			Opv_Front_View_Target.target_ctrl_enable=0;

			Total_Controller.SDK_Roll_Position_Control.Expect=0;
			Total_Controller.SDK_Roll_Position_Control.FeedBack=Opv_Front_View_Target.sdk_target.x;
			PID_Control_SDK_Err_LPF(&Total_Controller.SDK_Roll_Position_Control,Opv_Front_View_Target.trust_flag,0.1f);
			OpticalFlow_Pos_Ctrl_Output.x=-Total_Controller.SDK_Roll_Position_Control.Control_OutPut;
			

			Total_Controller.SDK_Pitch_Position_Control.Expect=Forward_Keep_Distance;//����Ϊǰ�򱣳־��룬����ɸ���ʵ�����ж���
			Total_Controller.SDK_Pitch_Position_Control.FeedBack=Opv_Front_View_Target.apriltag_distance;
			PID_Control_SDK_Err_LPF(&Total_Controller.SDK_Pitch_Position_Control,Opv_Front_View_Target.trust_flag,0.1f);
			OpticalFlow_Pos_Ctrl_Output.y=-Total_Controller.SDK_Pitch_Position_Control.Control_OutPut;	
		}
		else//��ʧĿ��
		{
		  miss_flag=1;
		}		
	}

	
	if(miss_flag==1)//Ŀ�궪ʧ
	{
		if(miss_cnt==1)//��ʼ��ʧ����Ŀ���������ǰλ�ú󣬽�����ͨ��������
		{
			miss_cnt=2;
			OpticalFlow_Pos_Ctrl_Output.x=0;
			OpticalFlow_Pos_Ctrl_Output.y=0;
			OpticalFlow_Control_Pure(1);		
		}
		else if(miss_cnt==2)//��ʧ����Ŀ��󣬽�����ͨ��������
		{
			OpticalFlow_Control_Pure(0);		
		}
	}
  else//Ŀ��δ��ʧ
	{
		OpticalFlow_Vel_Control(OpticalFlow_Pos_Ctrl_Output);//�ٶȿ�������20ms
	}			
}








void Self_Track_Control_Pilot(void)
{
	static uint8_t miss_cnt=1;
	static uint16_t _cnt=0;
	static uint8_t miss_flag=0;
	_cnt++;
	if(_cnt>=20)//100ms
	{	
		_cnt=0;		
	  if(Opv_Top_View_Target.target_ctrl_enable==1)//Ŀ��������
		{			
			miss_cnt=1;
			miss_flag=0;			
			Opv_Top_View_Target.target_ctrl_enable=0;

			Total_Controller.SDK_Roll_Position_Control.Expect=0;
			Total_Controller.SDK_Roll_Position_Control.FeedBack=Opv_Top_View_Target.sdk_target.x;
			PID_Control_SDK_Err_LPF(&Total_Controller.SDK_Roll_Position_Control,Opv_Top_View_Target.trust_flag,0.1f);
			OpticalFlow_Pos_Ctrl_Output.x=-Total_Controller.SDK_Roll_Position_Control.Control_OutPut;
			
			static float target_speed;//5 3 2 1
			if(ABS(Opv_Top_View_Target.sdk_angle)<=10)      target_speed=8;//15
			else if(ABS(Opv_Top_View_Target.sdk_angle)<=20) target_speed=5;//10
			else if(ABS(Opv_Top_View_Target.sdk_angle)<=50) target_speed=3;//5
			else target_speed=1;//5 3 2 1
					
			OpticalFlow_Pos_Ctrl_Output.y=target_speed;	
		}
		else//��ʧĿ��
		{
		  miss_flag=1;
		}		
	}

	
	if(miss_flag==1)//Ŀ�궪ʧ
	{
		if(miss_cnt==1)//��ʼ��ʧ����Ŀ���������ǰλ�ú󣬽�����ͨ��������
		{
			miss_cnt=2;
			OpticalFlow_Pos_Ctrl_Output.x=0;
			OpticalFlow_Pos_Ctrl_Output.y=0;
			OpticalFlow_Control_Pure(1);		
		}
		else if(miss_cnt==2)//��ʧ����Ŀ��󣬽�����ͨ��������
		{
			OpticalFlow_Control_Pure(0);		
			Flight.yaw_ctrl_mode=ROTATE;
			Flight.yaw_outer_control_output  =RC_Data.rc_rpyt[RC_YAW];			
		}
	}
  else//Ŀ��δ��ʧ
	{
		OpticalFlow_Vel_Control(OpticalFlow_Pos_Ctrl_Output);//�ٶȿ�������20ms		
		Flight.yaw_ctrl_mode=ROTATE;
		Flight.yaw_outer_control_output  =-3.0f*Opv_Top_View_Target.sdk_angle;
	}			
}


#define Pole_Keep_Distance   50//cm
#define Deadband_Pole_Keep_Distance  10
void Front_Surround_Pole_Control_Pilot(void)
{
	static uint8_t miss_cnt=1;
	static uint16_t _cnt=0;
	static uint8_t miss_flag=0;
	static uint8_t gap_miss_cnt=1;
	_cnt++;
	if(_cnt>=20)//100ms
	{	
		_cnt=0;		
	  if(Opv_Front_View_Target.target_ctrl_enable==1)//Ŀ��������
		{			
			miss_cnt=1;
			miss_flag=0;			
			Opv_Front_View_Target.target_ctrl_enable=0;
			//��һ���ȼ��������ҷ������ƫ��ʱ������ƫ���Ƕȣ�ʹ�û�ͷ��׼����
			
			Flight.yaw_ctrl_mode=ROTATE;
			Flight.yaw_outer_control_output  =5.0f*Opv_Front_View_Target.sdk_target.x;
			if(ABS(Opv_Front_View_Target.sdk_target.x)<5.0f)//��ͷ�Ѿ���׼���ˣ��ɽ�����һ�������ɻ������˵ľ���
			{
				gap_miss_cnt=1;
				//�ڶ����ȼ�����ǰ�������ƫ��ʱ��������������λ�ã�ʹ�÷ɻ������˱���һ������
				if(front_tofsense_distance_valid_flag==1)//��ͷ�Ѿ���׼���ˣ��Ҳ�ഫ����������Чʱ
				{
					//ʵʱ�����Ը˾���
					Total_Controller.SDK_Pitch_Position_Control.Expect=Pole_Keep_Distance;//����Ϊǰ�򱣳־��룬����ɸ���ʵ�����ж���
					Total_Controller.SDK_Pitch_Position_Control.FeedBack=front_tofsense_distance;
					PID_Control_SDK_Err_LPF(&Total_Controller.SDK_Pitch_Position_Control,front_tofsense_distance_valid_flag,0.1f);//Opv_Front_View_Target.trust_flag
					OpticalFlow_Pos_Ctrl_Output.y=-Total_Controller.SDK_Pitch_Position_Control.Control_OutPut;
					OpticalFlow_Pos_Ctrl_Output.y=constrain_float(OpticalFlow_Pos_Ctrl_Output.y,-10,10);
					
				}
				else//��ͷ�Ѿ���׼���ˣ����ǲ�ഫ�����ľ�����Чʱ
				{
					//���������1��������̫Զ��������ͷǰ���ٶȣ���������
					OpticalFlow_Pos_Ctrl_Output.y=10;
				}

				
				if(ABS(Total_Controller.SDK_Pitch_Position_Control.Err)<10.0f)//��ͷ�Ѿ���׼���ˣ��ҷɻ������˵ľ����Ѿ�������ϣ�����ִ�к����ٶȿ���
				{
					//�������ȼ��������ҷ������ƫ�����С�����Ҿ������˾������Сʱ���ƶ���������ٶȣ�ʵ���Ƹ˷���
					OpticalFlow_Pos_Ctrl_Output.x=-5;//��ͷ�Ҳ෽���ٶ�Ϊ������ʱ��ת��				
				}
				else//��ͷ�Ѿ���׼���ˣ����Ƿɻ�������֮�������δ������ϣ��ȴ�������������ִ�к����ٶȿ���
				{
					OpticalFlow_Pos_Ctrl_Output.x=0;//����λ�ñ���
				}
			}
			else//��Ұ��ʶ�����ˣ����ǻ�ͷ��δ��׼����ʱ��ԭ����ͣ���ȴ���ͷ��׼���ٵ���ǰ�������Ƹ˷���
			{
				miss_flag=2;
			}
		}
		else//��ʧĿ��
		{
		  miss_flag=1;
		}		
	}

	
	if(miss_flag==1)//Ŀ�궪ʧ
	{
		if(miss_cnt==1)//��ʼ��ʧ����Ŀ���������ǰλ�ú󣬽�����ͨ��������
		{
			miss_cnt=2;
			OpticalFlow_Pos_Ctrl_Output.x=0;
			OpticalFlow_Pos_Ctrl_Output.y=0;
			OpticalFlow_Control_Pure(1);		
		}
		else if(miss_cnt==2)//��ʧ����Ŀ��󣬽�����ͨ��������
		{
			OpticalFlow_Control_Pure(0);	
			Flight.yaw_ctrl_mode=ROTATE;
			Flight.yaw_outer_control_output  =RC_Data.rc_rpyt[RC_YAW];			
		}
	}
	else if(miss_flag==2)//��Ұ��ʶ�����ˣ����ǻ�ͷ��δ��׼����ʱ
	{
		//ƫ������Ȩ����Ȼ��Դ������λ�÷�����ǰ�󡢺���λ�ñ���ԭ��
		if(gap_miss_cnt==1)//��ʼ��ʧ����Ŀ���������ǰλ�ú󣬽�����ͨ��������
		{
			gap_miss_cnt=2;
			OpticalFlow_Pos_Ctrl_Output.x=0;
			OpticalFlow_Pos_Ctrl_Output.y=0;
			OpticalFlow_Control_Pure(1);		
		}
		else if(gap_miss_cnt==2)//��ʧ����Ŀ��󣬽�����ͨ��������
		{
			OpticalFlow_Control_Pure(0);	
		}	
	}
  else//Ŀ��δ��ʧ
	{
		OpticalFlow_Vel_Control(OpticalFlow_Pos_Ctrl_Output);//�ٶȿ�������20ms
	}			
}



//�����ڼ����״�SLAM��λ�����£���ͨ������LC307��LC302����λ��������Ч
void Horizontal_Navigation(float x,float y,float z,uint8_t nav_mode,uint8_t frame_id)
{	
	if(nav_mode==RELATIVE_MODE)//���ģʽ
	{
		switch(frame_id)
		{
			case BODY_FRAME://��������ϵ��
			{
				float map_x=0,map_y=0;
				from_body_to_nav_frame(x,y,&map_x,&map_y,WP_AHRS.Yaw);
				OpticalFlow_Pos_Ctrl_Expect.x=VIO_SINS.Position[_EAST]+map_x;
				OpticalFlow_Pos_Ctrl_Expect.y=VIO_SINS.Position[_NORTH]+map_y;
				Total_Controller.Height_Position_Control.Expect=NamelessQuad.Position[_UP]+z;			
			}
			break;
			case MAP_FRAME://��������ϵ��
			{
				OpticalFlow_Pos_Ctrl_Expect.x=VIO_SINS.Position[_EAST]+x;
				OpticalFlow_Pos_Ctrl_Expect.y=VIO_SINS.Position[_NORTH]+y;
				Total_Controller.Height_Position_Control.Expect=NamelessQuad.Position[_UP]+z;
			}
			break;		
		}	
	}
	else if(nav_mode==GLOBAL_MODE)//ȫ��ģʽ
	{
		switch(frame_id)
		{
			case MAP_FRAME://��������ϵ��
			{
				OpticalFlow_Pos_Ctrl_Expect.x=x;
				OpticalFlow_Pos_Ctrl_Expect.y=y;
				Total_Controller.Height_Position_Control.Expect=z;
			}
			break;
			default://ԭ�ر���
			{
				OpticalFlow_Pos_Ctrl_Expect.x=VIO_SINS.Position[_EAST];
				OpticalFlow_Pos_Ctrl_Expect.y=VIO_SINS.Position[_NORTH];
				Total_Controller.Height_Position_Control.Expect=NamelessQuad.Position[_UP];
			}				
		}	
	}
}








