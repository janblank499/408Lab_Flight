#ifndef _OPTICALFLOW_CONTROL_H
#define _OPTICALFLOW_CONTROL_H


#define OpticalFlow_Speed_Control_Max  250


void OpticalFlow_Control_Pure(uint8_t force_brake_flag);
void OpticalFlow_Control(uint8_t force_brake_flag);
void OpticalFlow_SINS_Reset(void);
void OpticalFlow_Ctrl_Reset(void);
void OpticalFlow_Vel_Control(Vector2f target);
void OpticalFlow_Pos_Control(void);
void OpticalFlow_Y_Vel_Control(float target_y);
void OpticalFlow_X_Vel_Control(float target_x);


void Color_Block_Control_Pilot(void);
void Top_APrilTag_Control_Pilot(void);
void Self_Track_Control_Pilot(void);
void Front_AprilTag_Control_Pilot(void);
void Front_Surround_Pole_Control_Pilot(void);



void Horizontal_Navigation(float x,float y,float z,uint8_t nav_mode,uint8_t frame_id);
	

extern Vector2f OpticalFlow_Pos_Ctrl_Expect;
extern Vector2f OpticalFlow_Pos_Ctrl_Err;
extern Vector2f OpticalFlow_Pos_Ctrl_Integrate;
extern Vector2f OpticalFlow_Pos_Ctrl_Output;




#endif















//SDK����ģʽ
//  {
//    if(Roll_Control==0&&Pitch_Control==0)//��ˮƽң��������
//    {  
//      if(Opv_Top_View_Target.line_ctrl_enable==1)//�߼��
//      {
//        /***********��ֻ��Ҫ�ٶȿ���ʱ����������ע�ͣ����޵���ʱ��*************/
//        sdk_ctrl_cnt++;
//        if(sdk_ctrl_cnt>=4)//20ms
//        {
//          Total_Controller.SDK_Roll_Position_Control.Expect=0;
//          Total_Controller.SDK_Roll_Position_Control.FeedBack=Opv_Top_View_Target.sdk_target.x;
//          PID_Control_SDK_Err_LPF(&Total_Controller.SDK_Roll_Position_Control,1);   
//          accel_target.x=constrain_float(Total_Controller.SDK_Roll_Position_Control.Control_OutPut,
//                                         -Total_Controller.Optical_Speed_Control.Control_OutPut_Limit,
//                                         Total_Controller.Optical_Speed_Control.Control_OutPut_Limit);                             
//          Controller.roll_outer_control_output=constrain_float(fast_atan(accel_target.x/(GRAVITY_MSS*100))*RAD2DEG,-30,30);//roll                                
//          sdk_ctrl_cnt=0;
//        }
//				
//        if(ABS(SDK_Target_Yaw_Gyro)<=10) OpticalFlow_Y_Vel_Control(5);
//        else if(ABS(SDK_Target_Yaw_Gyro)<=20) OpticalFlow_Y_Vel_Control(3);
//        else if(ABS(SDK_Target_Yaw_Gyro)<=50) OpticalFlow_Y_Vel_Control(2);
//        else OpticalFlow_Y_Vel_Control(1);
//        //OpticalFlow_Y_Vel_Control(0);
//        OpticalFlow_Pos_Ctrl_Expect.x=0;
//        OpticalFlow_Pos_Ctrl_Expect.y=0;
//        force_brake_flag=1;
//      }
//      else if(Opv_Top_View_Target.target_ctrl_enable==1)//Ŀ��������
//      {
//        sdk_ctrl_cnt++;
//        if(sdk_ctrl_cnt>=10)//50ms
//        {				
//					Opv_Top_View_Target.target_ctrl_enable=0;
//					Total_Controller.SDK_Roll_Position_Control.Expect=0;
//					Total_Controller.SDK_Roll_Position_Control.FeedBack=Opv_Top_View_Target.sdk_target.x;
//					PID_Control_SDK_Err_LPF(&Total_Controller.SDK_Roll_Position_Control,Opv_Top_View_Target.trust_flag);
//					
//					Total_Controller.SDK_Pitch_Position_Control.Expect=0;
//					Total_Controller.SDK_Pitch_Position_Control.FeedBack=Opv_Top_View_Target.sdk_target.y;
//					PID_Control_SDK_Err_LPF(&Total_Controller.SDK_Pitch_Position_Control,Opv_Top_View_Target.trust_flag);
//					
//					accel_target.x=constrain_float(Total_Controller.SDK_Roll_Position_Control.Control_OutPut,
//																				 -Total_Controller.Optical_Speed_Control.Control_OutPut_Limit,
//																				 Total_Controller.Optical_Speed_Control.Control_OutPut_Limit);                             
//					Controller.roll_outer_control_output=constrain_float(fast_atan(accel_target.x/(GRAVITY_MSS*100))*RAD2DEG,-30,30);//roll
//					
//					
//					accel_target.y=constrain_float(Total_Controller.SDK_Pitch_Position_Control.Control_OutPut,
//																				 -Total_Controller.Optical_Speed_Control.Control_OutPut_Limit,
//																				 Total_Controller.Optical_Speed_Control.Control_OutPut_Limit);//450
//					Controller.pitch_outer_control_output=constrain_float(fast_atan(accel_target.y*WP_AHRS.cos_rpy[_ROL]/(GRAVITY_MSS*100))*RAD2DEG,-30,30);//pitch 
//					sdk_ctrl_cnt=0;
//        }				
//        OpticalFlow_Pos_Ctrl_Expect.x=0;
//        OpticalFlow_Pos_Ctrl_Expect.y=0;
//        force_brake_flag=1;
//      }
//      else//SDKģʽ�£�δ����������
//      {
//        /**************************����λ�ÿ�����************************************/
//        if(OpticalFlow_Pos_Ctrl_Expect.x==0
//           &&OpticalFlow_Pos_Ctrl_Expect.y==0)
//        {
//          if(force_brake_flag==1||(rMat[2][2]>=0.95f
//                                   &&pythagorous2(OpticalFlow_SINS.Speed[_EAST],OpticalFlow_SINS.Speed[_NORTH])<=40))//��˻��к󣬸��ݵ�ǰ�ٶȡ�����ж��Ƿ������ͣ
//          {
//            OpticalFlow_Pos_Ctrl_Expect.x=OpticalFlow_SINS.Position[_EAST];
//            OpticalFlow_Pos_Ctrl_Expect.y=OpticalFlow_SINS.Position[_NORTH];
//          }
//          else  //��˻���δ������ͣ����ʱ��ֻ�����ٶȿ��� 
//          {
//            OpticalFlow_Pos_Ctrl_Output.x=0;
//            OpticalFlow_Pos_Ctrl_Output.y=0;
//          }
//        }
//        else  OpticalFlow_Pos_Control();
//        /**************************����ģ�͵ļ��ٶ�-��̬��ӳ�䣬���ֱ�Ӹ���̬��������������20������************************************/
//        OpticalFlow_Vel_Control(OpticalFlow_Pos_Ctrl_Output);//�ٶ�����        
//        PID_LPF_Reset(&Total_Controller.SDK_Roll_Position_Control,SDK_Roll_Position_Controler);
//        PID_LPF_Reset(&Total_Controller.SDK_Pitch_Position_Control,SDK_Pitch_Position_Controler);
//				SDK_Pos_Ctrl_Reset();
//			}
//    }
//    else//SDKģʽ�´����ֶ���˲���ʱ����������Դ��ң��������
//    {
//			Controller.roll_outer_control_output =RC_Data.rc_rpyt[RC_ROLL];
//			Controller.pitch_outer_control_output=RC_Data.rc_rpyt[RC_PITCH];		
//      OpticalFlow_Pos_Ctrl_Expect.x=0;
//      OpticalFlow_Pos_Ctrl_Expect.y=0;
//      force_brake_flag=1;
//			PID_LPF_Reset(&Total_Controller.SDK_Roll_Position_Control,SDK_Roll_Position_Controler);
//			PID_LPF_Reset(&Total_Controller.SDK_Pitch_Position_Control,SDK_Pitch_Position_Controler);
//			SDK_Pos_Ctrl_Reset();
//    }  
//    OpticalFlow_Pos_Ctrl_Expect.x=0;
//    OpticalFlow_Pos_Ctrl_Expect.y=0;
//  }
/* Copyright (c)  2018-2025 Wuhan Nameless Innovation Technology Co.,Ltd. All rights reserved.*/

//			if(ABS(Opv_Front_View_Target.sdk_target.x)<5.0f)//��ͷ�Ѿ���׼���ˣ��ɽ�����һ�������ɻ������˵ľ���
//			{
//				gap_miss_cnt=1;
//				//�ڶ����ȼ�����ǰ�������ƫ��ʱ��������������λ�ã�ʹ�÷ɻ������˱���һ������
//				Total_Controller.SDK_Pitch_Position_Control.Expect=Pole_Keep_Distance;//����Ϊǰ�򱣳־��룬����ɸ���ʵ�����ж���
//				Total_Controller.SDK_Pitch_Position_Control.FeedBack=front_tofsense_distance;
//				PID_Control_SDK_Err_LPF(&Total_Controller.SDK_Pitch_Position_Control,Opv_Front_View_Target.trust_flag);
//				OpticalFlow_Pos_Ctrl_Output.y=-Total_Controller.SDK_Pitch_Position_Control.Control_OutPut;				
//				if(ABS(Total_Controller.SDK_Pitch_Position_Control.Err)<5.0f)//��ͷ�Ѿ���׼���ˣ��зɻ������˵ľ����Ѿ�������ϣ�����ִ�к����ٶȿ���
//				{
//					//�������ȼ��������ҷ������ƫ�����С�����Ҿ������˾������Сʱ���ƶ���������ٶȣ�ʵ���Ƹ˷���
//					OpticalFlow_Pos_Ctrl_Output.x=-5;//��ͷ�Ҳ෽���ٶ�Ϊ������ʱ��ת��				
//				}
//				else//��ͷ�Ѿ���׼���ˣ����Ƿɻ�������֮�������δ������ϣ��ȴ�������������ִ�к����ٶȿ���
//				{
//					OpticalFlow_Pos_Ctrl_Output.x=0;//����λ�ñ���
//				}
//			}
//			else//��Ұ��ʶ�����ˣ����ǻ�ͷ��δ��׼����ʱ��ԭ����ͣ���ȴ���ͷ��׼���ٵ���ǰ�������Ƹ˷���
//			{
//				miss_flag=2;
//			}












