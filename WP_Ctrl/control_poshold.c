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
#include "control_poshold.h"

Vector3_Nav Earth_Frame_Accel_Target={0,0,0};   //��������������ϵ����������������Ŀ���˶����ٶ�����
Vector3_Nav Earth_Frame_Pos_Err={0,0,0};        //��������������ϵ����������������wλ��ƫ��
Vector2_Ang Body_Frame_Accel_Target={0,0};      //��������������ϵ��������(Y��)������(X��)����Ŀ���˶����ٶ�����
Vector2_Ang Body_Frame_Speed_Feedback={0,0};    //��������������ϵ��������(Y��)������(X��)����Ŀ���˶��ٶȷ���
Vector2_Ang Body_Frame_Pos_Err={0,0};           //���巽����λ��ƫ��
Vector2_Ang Body_Frame_Brake_Speed={0,0};       //���巽����ɲ���ٶ�
uint8 GPS_Speed_Control_Mode=0;


//��NED����ϵ�µ��������ٶ�ת���ɻ�������ϵ�µ���б�Ƕ�(pitch,roll)
void NED_Desired_Accel_Transform_Angle(Vector2f _accel_target,Vector2f *target_angle)
{
	float accel_right, accel_forward;
	float lean_angle_max = 30.0f;	
	accel_right  =-_accel_target.x;//cm/s^2
	accel_forward= _accel_target.y;//cm/s^2
	//update angle targets that will be passed to stabilize controller
	//�������������
	target_angle->x=RAD_TO_DEG*atanf((-accel_right/(GRAVITY_MSS*100))*WP_AHRS.cos_rpy[_PIT]);
	//��������������
	target_angle->y=RAD_TO_DEG*atanf(accel_forward/(GRAVITY_MSS*100));
	
	target_angle->x=constrain_float(target_angle->x,-lean_angle_max,lean_angle_max);//roll
	target_angle->y=constrain_float(target_angle->y,-lean_angle_max,lean_angle_max);//pitch
}


void ENU_Desired_Accel_Transform_Angle(Vector2f _accel_target,Vector2f *target_angle)
{
	float accel_right, accel_forward;
	float lean_angle_max = 30.0f;	
	accel_right  =  _accel_target.x;//cm/s^2
	accel_forward= -_accel_target.y;//cm/s^2
	//update angle targets that will be passed to stabilize controller	
	target_angle->x=RAD_TO_DEG*atanf( accel_right/GRAVITY_CMSS);													//�������������
	target_angle->y=RAD_TO_DEG*atanf(-accel_forward*WP_AHRS.cos_rpy[_ROL]/GRAVITY_CMSS); //��������������
	
	target_angle->x=constrain_float(target_angle->x,-lean_angle_max,lean_angle_max);//roll
	target_angle->y=constrain_float(target_angle->y,-lean_angle_max,lean_angle_max);//pitch
}


float ncq_speed_mapping(float input,uint16_t input_max,float output_max)
{
  float output_speed=0;
  float temp_scale=(float)(input/input_max);
  temp_scale=constrain_float(temp_scale,-1.0f, 1.0f);
  if(temp_scale>=0) output_speed=(float)(output_max*temp_scale*temp_scale);
  else output_speed=(float)(-output_max*temp_scale*temp_scale); 
  return output_speed;
}

Vector2f accel_target={0},angle_target={0};
void ncq_control_poshold()
{
  static uint16 position_cnt=0;//�߶��ٶȿ��Ƽ�����
  static uint16 speed_cnt=0;//�߶��ٶȿ��Ƽ�����
  static uint8_t miss_fixed_flag=0;
	static systime poshold_dt;
	if(GPS_ok()==FALSE)//�����㶨����������������ˮƽ��̬
  {
    /********��GPS����ģʽλ��0��ֱ�ӽ�����̬ģʽ���ȴ�GPS�ź��ٴ���������ʱ��***********
    *********�Զ��л���GPS����ģʽ�����Controler_Mode_Select����������й���**********/
    //�ڿ���ģʽ�����Լ��Ƿ������ٴν���GPS����ģʽ
		Flight.pitch_outer_control_output=RC_Data.rc_rpyt[RC_PITCH];
		Flight.roll_outer_control_output=RC_Data.rc_rpyt[RC_ROLL];
		miss_fixed_flag=1;		
		return ;		
  }
	
	Get_Systime(&poshold_dt);
	//����������������ڴ��ڿ�������10��
	if(0.001f*poshold_dt.period>=20*WP_Duty_Dt)
	{
		//���1:���δ�����ģʽ���뱾ģʽ
	  //���2:ϵͳ���ȳ�ʱ����ϵͳ��ƺ�������£�����������ܷ���
    miss_fixed_flag=1;
	}	
	
	if(miss_fixed_flag==1)//֮ǰδ���㶨λ����,δ����Ŀ���
	{
		miss_fixed_flag=0;
		Total_Controller.Latitude_Position_Control.Expect=NamelessQuad.Position[_NORTH];
		Total_Controller.Longitude_Position_Control.Expect=NamelessQuad.Position[_EAST];
		Total_Controller.Latitude_Speed_Control.Expect =0;
		Total_Controller.Longitude_Speed_Control.Expect=0;
		
		East_North_Ctrl_Reset();
	}	
	
  /*******************************ˮƽλ�ÿ�������ʼ***********************************************************/
  //������������ͣ�������GPS��������������ԭ��ǳ�� http://blog.csdn.net/u011992534/article/details/79408187
    if(Roll_Control==0
       &&Pitch_Control==0)//��ˮƽң��������
    {
      position_cnt++;
      if(position_cnt>8)//20ms����һ��
      {
        //λ������,��γ�������ٶȡ��߶�
        if(Total_Controller.Latitude_Position_Control.Expect==0
           &&Total_Controller.Longitude_Position_Control.Expect==0)//����˻��к�ֻ����һ��
        {
          if(get_stopping_point_xy(&UAV_Cushion_Stop_Point)==1)
          {
            Total_Controller.Latitude_Position_Control.Expect=UAV_Cushion_Stop_Point.y;
            Total_Controller.Longitude_Position_Control.Expect=UAV_Cushion_Stop_Point.x;
          }
          else//ֻ�����ٶ�ɲ��
          {
            //�ٶȿ���������
            Total_Controller.Latitude_Speed_Control.Expect =0;
            Total_Controller.Longitude_Speed_Control.Expect=0;  
          }
        }
        else
        {
          //λ�÷�������Դ�ڵ�ǰ�ߵ���λ�ù���
          Total_Controller.Latitude_Position_Control.FeedBack=NamelessQuad.Position[_NORTH];
          Total_Controller.Longitude_Position_Control.FeedBack=NamelessQuad.Position[_EAST];
          //��������ϵ��E��N������λ��ƫ��
          Earth_Frame_Pos_Err.N=Total_Controller.Latitude_Position_Control.Expect-Total_Controller.Latitude_Position_Control.FeedBack;
          Earth_Frame_Pos_Err.E=Total_Controller.Longitude_Position_Control.Expect-Total_Controller.Longitude_Position_Control.FeedBack;
          //��������ϵ�»���Pitch��Roll������λ��ƫ��
          Body_Frame_Pos_Err.Pit=-Earth_Frame_Pos_Err.E*WP_AHRS.sin_rpy[_YAW]+Earth_Frame_Pos_Err.N*WP_AHRS.cos_rpy[_YAW];
          Body_Frame_Pos_Err.Rol=Earth_Frame_Pos_Err.E*WP_AHRS.cos_rpy[_YAW]+Earth_Frame_Pos_Err.N*WP_AHRS.sin_rpy[_YAW];
          //��������ϵ�»���Pitch��Roll����������ɲ���ٶȣ�����Ϊ���������㲻����PID_Control()����
          Body_Frame_Pos_Err.Pit=constrain_float(Body_Frame_Pos_Err.Pit,-Total_Controller.Latitude_Position_Control.Err_Max, Total_Controller.Latitude_Position_Control.Err_Max);//λ��ƫ���޷�����λcm
          Body_Frame_Pos_Err.Rol=constrain_float(Body_Frame_Pos_Err.Rol,-Total_Controller.Longitude_Position_Control.Err_Max,Total_Controller.Longitude_Position_Control.Err_Max);//λ��ƫ���޷�����λcm
          
          Body_Frame_Brake_Speed.Pit=Total_Controller.Latitude_Position_Control.Kp*Body_Frame_Pos_Err.Pit;
          Body_Frame_Brake_Speed.Rol=Total_Controller.Longitude_Position_Control.Kp*Body_Frame_Pos_Err.Rol;
          //�ٶȿ���������
          Total_Controller.Latitude_Speed_Control.Expect =Body_Frame_Brake_Speed.Pit;
          Total_Controller.Longitude_Speed_Control.Expect=Body_Frame_Brake_Speed.Rol;  
        }
        position_cnt=0;//λ�ÿ�������������������ɲ���ٶ�
      }
      //����ϵ��ˮƽ�ٶȣ�ת������������ϵX-Y������
      //������Pitch��Roll����ˮƽ�ٶȿ���
      speed_cnt++;
      if(speed_cnt>=4)//10ms����һ��λ��
      {
        Body_Frame_Speed_Feedback.Pit=-NamelessQuad.Speed[_EAST]*WP_AHRS.sin_rpy[_YAW]+NamelessQuad.Speed[_NORTH]*WP_AHRS.cos_rpy[_YAW];
        Body_Frame_Speed_Feedback.Rol=NamelessQuad.Speed[_EAST]*WP_AHRS.cos_rpy[_YAW]+NamelessQuad.Speed[_NORTH]*WP_AHRS.sin_rpy[_YAW];
        //�����巽���ٶȷ�����
        Total_Controller.Latitude_Speed_Control.FeedBack=Body_Frame_Speed_Feedback.Pit;//��ͷPitch����Y������
        Total_Controller.Longitude_Speed_Control.FeedBack=Body_Frame_Speed_Feedback.Rol;//���Roll����X������
        //�����巽���ٶȿ�����
        PID_Control_Div_LPF(&Total_Controller.Latitude_Speed_Control,0.02f);
        PID_Control_Div_LPF(&Total_Controller.Longitude_Speed_Control,0.02f);
        
        accel_target.y=-Total_Controller.Latitude_Speed_Control.Control_OutPut;
        accel_target.x=Total_Controller.Longitude_Speed_Control.Control_OutPut;//�����˶����ٶ�
        ENU_Desired_Accel_Transform_Angle(accel_target,&angle_target);//�����˶����ٶ�ת������̬���
				Flight.roll_outer_control_output =angle_target.x;
				Flight.pitch_outer_control_output=angle_target.y;;	
				
        speed_cnt=0;
      }
      /*******************************ˮƽλ�ÿ���������***********************************************************/
    }
    else //ֻ����ˮƽ�ٶȿ��ƣ���ˮƽλ�ÿ���
    {
      //�����������1����������ϵ�ĺ����ٶȿ��ƣ�
      //            2����������ϵ�����ϵ��ٶȿ���
      if(GPS_Speed_Control_Mode==Angle_Mode)//�ƶ�����ˣ���Ӧ�����Ƕ�
      {
				Flight.roll_outer_control_output =RC_Data.rc_rpyt[RC_ROLL];
				Flight.pitch_outer_control_output=RC_Data.rc_rpyt[RC_PITCH];	
      }
      else//�ƶ�����ˣ���Ӧ������������ϵ����Pitch,Roll�����˶��ٶ�
      {
        speed_cnt++;
        if(speed_cnt>=4)//10ms����һ���ٶ�
        {
          Total_Controller.Latitude_Speed_Control.Expect =ncq_speed_mapping(-RC_Data.rc_rpyt[RC_PITCH],Pit_Rol_Max,Max_Horvel);
          Total_Controller.Longitude_Speed_Control.Expect=ncq_speed_mapping(RC_Data.rc_rpyt[RC_ROLL],Pit_Rol_Max,Max_Horvel);
													
          //����ϵ��ˮƽ�ٶȣ�ת������������ϵX-Y������
          //������Pitch��Roll����ˮƽ�ٶȿ���
          Body_Frame_Speed_Feedback.Pit=-NamelessQuad.Speed[_EAST]*WP_AHRS.sin_rpy[_YAW]+NamelessQuad.Speed[_NORTH]*WP_AHRS.cos_rpy[_YAW];
          Body_Frame_Speed_Feedback.Rol= NamelessQuad.Speed[_EAST]*WP_AHRS.cos_rpy[_YAW]+NamelessQuad.Speed[_NORTH]*WP_AHRS.sin_rpy[_YAW];
          
          Total_Controller.Latitude_Speed_Control.FeedBack=Body_Frame_Speed_Feedback.Pit;//��ͷPitch����Y������
          Total_Controller.Longitude_Speed_Control.FeedBack=Body_Frame_Speed_Feedback.Rol;//���Roll����X������
          
          PID_Control_Div_LPF(&Total_Controller.Latitude_Speed_Control,0.02f);
          PID_Control_Div_LPF(&Total_Controller.Longitude_Speed_Control,0.02f);//ˮƽ�ٶȿ������õ������˶����ٶ�
          
          
          accel_target.y=-Total_Controller.Latitude_Speed_Control.Control_OutPut;
          accel_target.x= Total_Controller.Longitude_Speed_Control.Control_OutPut;//�����˶����ٶ�
          ENU_Desired_Accel_Transform_Angle(accel_target,&angle_target);//�����˶����ٶ�ת������̬���
          Flight.pitch_outer_control_output=angle_target.y;
          Flight.roll_outer_control_output=angle_target.x;           
          speed_cnt=0;
        }
      }
      
      Total_Controller.Latitude_Position_Control.Expect=0;
      Total_Controller.Longitude_Position_Control.Expect=0;
    }
}



//����ģʽ�£�ң�˻��к�����ˮƽ�ٶȿ���ɲ������ɲͣ���ٸ�ֵλ��ѡ��
uint8_t get_stopping_point_xy(Vector3f *stopping_point)
{
  Vector2f curr_pos,curr_vel,curr_accel;
  float vel_total=0,accel_total=0;   
  curr_pos.x=NamelessQuad.Position[_EAST];
  curr_pos.y=NamelessQuad.Position[_NORTH];
  curr_vel.x=NamelessQuad.Speed[_EAST];
  curr_vel.y=NamelessQuad.Speed[_NORTH];   
  curr_accel.x=NamelessQuad.Acceleration[_EAST];
  curr_accel.y=NamelessQuad.Acceleration[_NORTH];
  
  vel_total=pythagorous2(curr_vel.x,curr_vel.y);
  accel_total=pythagorous2(curr_accel.x,curr_accel.y);
  
  if(vel_total <= 20.0f //��ˮƽ�ٶȵ�С�ڵ���20cm/s
     && accel_total<=40 //��ˮƽ���ٶȵ�С�ڵ���40cm/s^2
     &&WP_AHRS.rMat[8]>=0.97f)//WP_AHRS.cos_rpy[_PIT]*WP_AHRS.cos_rpy[_ROL]����������ˮƽ��̬ԼΪ15deg����������ˮƽ��̬��ԼΪ10deg  
  {
    stopping_point->x = curr_pos.x;
    stopping_point->y = curr_pos.y;
    return 1;
  }
  return 0;
}




bool slam_ok()// returns true if the GPS is ok and home position is set
{
  if(current_state.fault!=1)
  {
    return TRUE;
  }
  else
  {
    return FALSE;
  }
}



void slam_control_poshold(SINS_Lite *_ins)
{
  static uint16 position_cnt=0;//�߶��ٶȿ��Ƽ�����
  static uint16 speed_cnt=0;//�߶��ٶȿ��Ƽ�����
  static uint8_t miss_fixed_flag=0;
	static systime poshold_dt;
	if(slam_ok()==FALSE)//�����㶨����������������ˮƽ��̬
  {
    /********��GPS����ģʽλ��0��ֱ�ӽ�����̬ģʽ���ȴ�GPS�ź��ٴ���������ʱ��***********
    *********�Զ��л���GPS����ģʽ�����Controler_Mode_Select����������й���**********/
    //�ڿ���ģʽ�����Լ��Ƿ������ٴν���GPS����ģʽ
		Flight.pitch_outer_control_output=RC_Data.rc_rpyt[RC_PITCH];
		Flight.roll_outer_control_output=RC_Data.rc_rpyt[RC_ROLL];
		miss_fixed_flag=1;		
		return ;		
  }
	
	Get_Systime(&poshold_dt);
	//����������������ڴ��ڿ�������10��
	if(0.001f*poshold_dt.period>=20*WP_Duty_Dt)
	{
		//���1:���δ�����ģʽ���뱾ģʽ
	  //���2:ϵͳ���ȳ�ʱ����ϵͳ��ƺ�������£�����������ܷ���
    miss_fixed_flag=1;
	}	
	
	if(miss_fixed_flag==1)//֮ǰδ���㶨λ����,δ����Ŀ���
	{
		miss_fixed_flag=0;
		Total_Controller.Latitude_Position_Control.Expect=_ins->Position[_NORTH];
		Total_Controller.Longitude_Position_Control.Expect=_ins->Position[_EAST];

		Total_Controller.Latitude_Speed_Control.Expect =0;
		Total_Controller.Longitude_Speed_Control.Expect=0;	
		East_North_Ctrl_Reset();		
	}	
	
  /*******************************ˮƽλ�ÿ�������ʼ***********************************************************/
  //������������ͣ�������GPS��������������ԭ��ǳ�� http://blog.csdn.net/u011992534/article/details/79408187
    if(Roll_Control==0
       &&Pitch_Control==0)//��ˮƽң��������
    {
      position_cnt++;
      if(position_cnt>8)//20ms����һ��
      {
        //λ������,��γ�������ٶȡ��߶�
        if(Total_Controller.Latitude_Position_Control.Expect==0
           &&Total_Controller.Longitude_Position_Control.Expect==0)//����˻��к�ֻ����һ��
        {
					Total_Controller.Latitude_Position_Control.Expect=_ins->Position[_NORTH];
					Total_Controller.Longitude_Position_Control.Expect=_ins->Position[_EAST];
        }
        else
        {
          //λ�÷�������Դ�ڵ�ǰ�ߵ���λ�ù���
          Total_Controller.Latitude_Position_Control.FeedBack=_ins->Position[_NORTH];
          Total_Controller.Longitude_Position_Control.FeedBack=_ins->Position[_EAST];
          //��������ϵ��E��N������λ��ƫ��
          Earth_Frame_Pos_Err.N=Total_Controller.Latitude_Position_Control.Expect-Total_Controller.Latitude_Position_Control.FeedBack;
          Earth_Frame_Pos_Err.E=Total_Controller.Longitude_Position_Control.Expect-Total_Controller.Longitude_Position_Control.FeedBack;
          //��������ϵ�»���Pitch��Roll������λ��ƫ��
          Body_Frame_Pos_Err.Pit=-Earth_Frame_Pos_Err.E*WP_AHRS.sin_rpy[_YAW]+Earth_Frame_Pos_Err.N*WP_AHRS.cos_rpy[_YAW];
          Body_Frame_Pos_Err.Rol=Earth_Frame_Pos_Err.E*WP_AHRS.cos_rpy[_YAW]+Earth_Frame_Pos_Err.N*WP_AHRS.sin_rpy[_YAW];
          //��������ϵ�»���Pitch��Roll����������ɲ���ٶȣ�����Ϊ���������㲻����PID_Control()����
          Body_Frame_Pos_Err.Pit=constrain_float(Body_Frame_Pos_Err.Pit,-Total_Controller.Latitude_Position_Control.Err_Max, Total_Controller.Latitude_Position_Control.Err_Max);//λ��ƫ���޷�����λcm
          Body_Frame_Pos_Err.Rol=constrain_float(Body_Frame_Pos_Err.Rol,-Total_Controller.Longitude_Position_Control.Err_Max,Total_Controller.Longitude_Position_Control.Err_Max);//λ��ƫ���޷�����λcm
          
          Body_Frame_Brake_Speed.Pit=Total_Controller.Latitude_Position_Control.Kp*Body_Frame_Pos_Err.Pit;
          Body_Frame_Brake_Speed.Rol=Total_Controller.Longitude_Position_Control.Kp*Body_Frame_Pos_Err.Rol;
          //�ٶȿ���������
          Total_Controller.Latitude_Speed_Control.Expect =Body_Frame_Brake_Speed.Pit;
          Total_Controller.Longitude_Speed_Control.Expect=Body_Frame_Brake_Speed.Rol;  
        }
        position_cnt=0;//λ�ÿ�������������������ɲ���ٶ�
      }
      //����ϵ��ˮƽ�ٶȣ�ת������������ϵX-Y������
      //������Pitch��Roll����ˮƽ�ٶȿ���
      speed_cnt++;
      if(speed_cnt>=4)//10ms����һ��λ��
      {
        Body_Frame_Speed_Feedback.Pit=-_ins->Speed[_EAST]*WP_AHRS.sin_rpy[_YAW]+_ins->Speed[_NORTH]*WP_AHRS.cos_rpy[_YAW];
        Body_Frame_Speed_Feedback.Rol= _ins->Speed[_EAST]*WP_AHRS.cos_rpy[_YAW]+_ins->Speed[_NORTH]*WP_AHRS.sin_rpy[_YAW];
        //�����巽���ٶȷ�����
        Total_Controller.Latitude_Speed_Control.FeedBack =Body_Frame_Speed_Feedback.Pit;//��ͷPitch����Y������
        Total_Controller.Longitude_Speed_Control.FeedBack=Body_Frame_Speed_Feedback.Rol;//���Roll����X������
        //�����巽���ٶȿ�����
        PID_Control_Div_LPF(&Total_Controller.Latitude_Speed_Control,0.02f);
        PID_Control_Div_LPF(&Total_Controller.Longitude_Speed_Control,0.02f);
        
        accel_target.y=-Total_Controller.Latitude_Speed_Control.Control_OutPut;
        accel_target.x= Total_Controller.Longitude_Speed_Control.Control_OutPut;//�����˶����ٶ�
        ENU_Desired_Accel_Transform_Angle(accel_target,&angle_target);//�����˶����ٶ�ת������̬���
				Flight.roll_outer_control_output =angle_target.x;
				Flight.pitch_outer_control_output=angle_target.y;;			
        speed_cnt=0;
      }
      /*******************************ˮƽλ�ÿ���������***********************************************************/
    }
    else //ֻ����ˮƽ�ٶȿ��ƣ���ˮƽλ�ÿ���
    {	    
			//�ƶ�����ˣ���Ӧ������������ϵ����Pitch,Roll�����˶��ٶ�					
			Total_Controller.Latitude_Speed_Control.Expect =ncq_speed_mapping(-RC_Data.rc_rpyt[RC_PITCH],Pit_Rol_Max,Max_Horvel);
			Total_Controller.Longitude_Speed_Control.Expect=ncq_speed_mapping(RC_Data.rc_rpyt[RC_ROLL],Pit_Rol_Max,Max_Horvel);
			speed_cnt++;
			if(speed_cnt>=4)//10ms����һ���ٶ�
			{							
				//����ϵ��ˮƽ�ٶȣ�ת������������ϵX-Y������
				//������Pitch��Roll����ˮƽ�ٶȿ���
				Body_Frame_Speed_Feedback.Pit=-_ins->Speed[_EAST]*WP_AHRS.sin_rpy[_YAW]+_ins->Speed[_NORTH]*WP_AHRS.cos_rpy[_YAW];
				Body_Frame_Speed_Feedback.Rol= _ins->Speed[_EAST]*WP_AHRS.cos_rpy[_YAW]+_ins->Speed[_NORTH]*WP_AHRS.sin_rpy[_YAW];
				
				Total_Controller.Latitude_Speed_Control.FeedBack=Body_Frame_Speed_Feedback.Pit;//��ͷPitch����Y������
				Total_Controller.Longitude_Speed_Control.FeedBack=Body_Frame_Speed_Feedback.Rol;//���Roll����X������
				
				PID_Control_Div_LPF(&Total_Controller.Latitude_Speed_Control,0.02f);
				PID_Control_Div_LPF(&Total_Controller.Longitude_Speed_Control,0.02f);//ˮƽ�ٶȿ������õ������˶����ٶ�
						 
				accel_target.y=-Total_Controller.Latitude_Speed_Control.Control_OutPut;
				accel_target.x= Total_Controller.Longitude_Speed_Control.Control_OutPut;//�����˶����ٶ�
				ENU_Desired_Accel_Transform_Angle(accel_target,&angle_target);//�����˶����ٶ�ת������̬���
				Flight.pitch_outer_control_output=angle_target.y;
				Flight.roll_outer_control_output=angle_target.x;           
				speed_cnt=0;
			}		
      Total_Controller.Latitude_Position_Control.Expect=0;
      Total_Controller.Longitude_Position_Control.Expect=0;
    }
}

/* Copyright (c)  2018-2025 Wuhan Nameless Innovation Technology Co.,Ltd. All rights reserved.*/


