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
#include "NamelessCotrun_SDK.h"

bool auto_altland(float taret_climb_rate,float target_climb_alt)
{
  return land_althold(taret_climb_rate,target_climb_alt);
}
//SDK��ģʽ��Ҫ�ڹ���ģʽ��ʹ��
uint8_t move_with_speed_target(float x_target,float y_target,float delta,SDK_Status *Status,uint16_t number)
{
  static float end_time=0;
  Vector2f vel_target;
  Testime dt;
  vel_target.x=x_target;
  vel_target.y=y_target;
  Test_Period(&dt);
  ncq_control_althold();//�߶ȿ�����Ȼ����
  if(Status->Status[number].Start_Flag==1
     &&Status->Status[number].Execute_Flag==1
       &&Status->Status[number].End_flag==1)
  {
    OpticalFlow_Control_Pure(0);//���֮�󣬽��й�����ͣ
    return 1;
  }
  else
  {
    if(Status->Status[number].Start_Flag==0) 
    {
      end_time=dt.Now_Time+delta;//��λms 
      Status->Status[number].Start_Flag=1;
    } 
    if(dt.Now_Time>end_time)
    {
      Status->Status[number].Execute_Flag=1;
      Status->Status[number].End_flag=1;
      
      OpticalFlow_Pos_Ctrl_Expect.x=0;
      OpticalFlow_Pos_Ctrl_Expect.y=0;
			OpticalFlow_Control_Pure(1);//���֮�󣬽��й�����ͣ
			
      end_time=0;
      Status->Transition_Time[number]=400;//400*5ms=2s
      return 1;//�������
    }
    else
    { 
      OpticalFlow_Pos_Ctrl_Expect.x=0;
      OpticalFlow_Pos_Ctrl_Expect.y=0;
      Status->Status[number].Execute_Flag=1;
      OpticalFlow_Vel_Control(vel_target);//�����ٶ�����
      return 0;
    }
  }
}


uint8_t move_with_xy_target(float pos_x_target,float pos_y_target,SDK_Status *Status,uint16_t number)
{
  ncq_control_althold();//�߶ȿ�����Ȼ����
  if(Status->Status[number].Start_Flag==0) 
  {
    OpticalFlow_Pos_Ctrl_Expect.x=OpticalFlow_SINS.Position[_EAST]+pos_x_target;
    OpticalFlow_Pos_Ctrl_Expect.y=OpticalFlow_SINS.Position[_NORTH]+pos_y_target;
    Status->Status[number].Start_Flag=1;
  }
  
  if(Status->Status[number].Start_Flag==1
     &&Status->Status[number].Execute_Flag==1
       &&Status->Status[number].End_flag==1)
  {
    OpticalFlow_Control_Pure(0);//���֮�󣬽��й�����ͣ
    return 1;
  }
  else
  {    
    if(pythagorous2(OpticalFlow_Pos_Ctrl_Expect.x-OpticalFlow_SINS.Position[_EAST],
                    OpticalFlow_Pos_Ctrl_Expect.y-OpticalFlow_SINS.Position[_NORTH])<=10.0f)
    {
      Status->Status[number].Execute_Flag=1;
      Status->Status[number].End_flag=1;
			Status->Transition_Time[number]=200;
			
      OpticalFlow_Pos_Ctrl_Expect.x=0;
      OpticalFlow_Pos_Ctrl_Expect.y=0;
      OpticalFlow_Control_Pure(1);//���֮�󣬽��й�����ͣ
      return 1;
    }
    else
    { 
      Status->Status[number].Execute_Flag=1; 
      OpticalFlow_Pos_Control();//����λ�ÿ���
      OpticalFlow_Vel_Control(OpticalFlow_Pos_Ctrl_Output);//�����ٶȿ��� 
      return 0;
    }
  }
}

uint8_t move_with_target(float pos_x_target,float pos_y_target,Duty_Status *Status,uint8_t *Start_flag)
{
  //static Vector2f pos_base;
  ncq_control_althold();//�߶ȿ�����Ȼ����
  if(*Start_flag==1)
  {
    //pos_base.x=OpticalFlow_SINS.Position[_EAST];
    //pos_base.y=OpticalFlow_SINS.Position[_NORTH];
    *Start_flag=0;
  }
  
  if(Status->Start_Flag==0) 
  {
    OpticalFlow_Pos_Ctrl_Expect.x=OpticalFlow_SINS.Position[_EAST]+pos_x_target;
    OpticalFlow_Pos_Ctrl_Expect.y=OpticalFlow_SINS.Position[_NORTH]+pos_y_target;
    Status->Start_Flag=1;
  }
  
  if(Status->Start_Flag==1
     &&Status->Execute_Flag==1
       &&Status->End_flag==1)
  {
    OpticalFlow_Control_Pure(0);//���֮�󣬽��й�����ͣ
    return 1;
  }
  else
  {    
    if(pythagorous2(OpticalFlow_Pos_Ctrl_Expect.x-OpticalFlow_SINS.Position[_EAST],
                    OpticalFlow_Pos_Ctrl_Expect.y-OpticalFlow_SINS.Position[_NORTH])<=8.0f)
    {
      Status->Execute_Flag=1;
      Status->End_flag=1;
      //pos_base.x=0;
      //pos_base.y=0;
      
      OpticalFlow_Pos_Ctrl_Expect.x=0;
      OpticalFlow_Pos_Ctrl_Expect.y=0;
			OpticalFlow_Control_Pure(1);//���֮�󣬽��й�����ͣ
			
      return 1;
    }
    else
    { 
      Status->Execute_Flag=1; 
      OpticalFlow_Pos_Control();//����λ�ÿ���
      OpticalFlow_Vel_Control(OpticalFlow_Pos_Ctrl_Output);//�����ٶȿ��� 
      return 0;
    }
  }
}


uint8_t move_with_z_target(float z_target,float z_vel,float delta,SDK_Status *Status,uint16_t number)
{
  static float target_rate=0;
  static float target_alt=0;
  static uint8_t end_flag=0;
  //static float z_base;
  static float end_time=0;
  Testime dt;
  Test_Period(&dt);
  OpticalFlow_Control_Pure(0);//ˮƽλ�ÿ�����Ȼ����
  
  if(Status->Status[number].Start_Flag==1
     &&Status->Status[number].Execute_Flag==1
       &&Status->Status[number].End_flag==1)
  {   
    ncq_control_althold();
    return 1;
  }
  else
  {
    if(Status->Status[number].Start_Flag==0) 
    {  
      if(z_target==0)//�ٶȿ��ƣ�һ��������ֻ����һ��
      {
        //z_base=NamelessQuad.Position[_UP];
        target_rate=z_vel;
        target_alt=0;
        end_flag=1;
      }
      else//λ�ÿ��ƣ�һ��������ֻ����һ��
      {
        //z_base=NamelessQuad.Position[_UP];
        target_rate=0;
        target_alt=NamelessQuad.Position[_UP]+z_target;
        end_flag=2;
      }
      end_time=dt.Now_Time+delta;//��λms 
      Status->Status[number].Start_Flag=1;
      
      Unwanted_Lock_Flag=0;//�����Զ�����
      //OpticalFlow_Pos_Ctrl_Expect.x=0;
      //OpticalFlow_Pos_Ctrl_Expect.y=0;
    }
    
    
    if(end_flag==1)
    {  
      auto_altland(target_rate,target_alt);
      if(dt.Now_Time>end_time)
      { 
        end_flag=0;
        target_rate=0;
        target_alt=0;
        //z_base=0;
        end_time=0;
        Status->Status[number].Execute_Flag=1;
        Status->Status[number].End_flag=1;
        Status->Transition_Time[number]=200;
        
        OpticalFlow_Pos_Ctrl_Expect.x=0;
        OpticalFlow_Pos_Ctrl_Expect.y=0;
        OpticalFlow_Control_Pure(1);//���֮�󣬽��й�����ͣ
				
        Total_Controller.Height_Position_Control.Expect=0;
        return 1;//�������
      }
      Status->Status[number].Execute_Flag=1;
    }
    else if(end_flag==2)
    {
      auto_altland(target_rate,target_alt);    
      if(ABS(target_alt-NamelessQuad.Position[_UP])<=5.0f)
      {
        end_flag=0;
        target_rate=0;
        target_alt=0;
        //z_base=0;
        end_time=0;
        Status->Status[number].Execute_Flag=1;
        Status->Status[number].End_flag=1;
        Status->Transition_Time[number]=200;
        
        OpticalFlow_Pos_Ctrl_Expect.x=0;
        OpticalFlow_Pos_Ctrl_Expect.y=0;
				OpticalFlow_Control_Pure(1);//���֮�󣬽��й�����ͣ
        
        Total_Controller.Height_Position_Control.Expect=0;
        return 1;
      }
      Status->Status[number].Execute_Flag=1;
    } 
  }
  return 0;
}

//#define NCQ_SDK_DUTY1 move_with_speed_target(10,0,2000 ,&SDK_Duty_Status,1-1)//��
//#define NCQ_SDK_DUTY2 move_with_speed_target(0,10,2000 ,&SDK_Duty_Status,2-1)//ǰ
//#define NCQ_SDK_DUTY3 move_with_speed_target(-10,0,2000,&SDK_Duty_Status,3-1)//��
//#define NCQ_SDK_DUTY4 move_with_speed_target(0,-10,2000,&SDK_Duty_Status,4-1)//��


#define NCQ_SDK_DUTY_MAX   3
#define NCQ_SDK_DUTY1 move_with_z_target(120,0,0,&SDK_Duty_Status,1-1)
#define NCQ_SDK_DUTY2 move_with_xy_target(0,100,&SDK_Duty_Status,2-1)
#define NCQ_SDK_DUTY3 move_with_z_target(0,-50,1000*100,&SDK_Duty_Status,3-1)//(-150,0,0,&SDK_Duty_Status,3-1)


SDK_Status SDK_Duty_Status;
uint16_t SDK_Duty_Cnt=0;
uint16_t SDK_Transition_Time=0;
void NCQ_SDK_Run(void)
{
  if(SDK_Duty_Status.Transition_Time[SDK_Duty_Cnt]>=1) 
    SDK_Duty_Status.Transition_Time[SDK_Duty_Cnt]--;//���Ź���ʱ��
  
  if(SDK_Duty_Status.Status[SDK_Duty_Cnt].Start_Flag==1
     &&SDK_Duty_Status.Status[SDK_Duty_Cnt].Execute_Flag==1
       &&SDK_Duty_Status.Status[SDK_Duty_Cnt].End_flag==1
         &&SDK_Duty_Status.Transition_Time[SDK_Duty_Cnt]==0)
    SDK_Duty_Cnt++;
  
  if(SDK_Duty_Cnt>=NCQ_SDK_DUTY_MAX) SDK_Duty_Cnt=NCQ_SDK_DUTY_MAX;
  
  if(SDK_Duty_Cnt==0)        NCQ_SDK_DUTY1;
  else if(SDK_Duty_Cnt==1)   NCQ_SDK_DUTY2;
  else if(SDK_Duty_Cnt==2)   NCQ_SDK_DUTY3;
  //else if(SDK_Duty_Cnt==3)   NCQ_SDK_DUTY4;
  //else if(SDK_Duty_Cnt==4)   NCQ_SDK_DUTY5;
  //else if(SDK_Duty_Cnt==5)   NCQ_SDK_DUTY6;
  //else if(SDK_Duty_Cnt==6)   NCQ_SDK_DUTY7;
  else
  {
    ncq_control_althold();//�߶ȿ���
    OpticalFlow_Control(0);//λ�ÿ���
  }
}


void NCQ_SDK_Reset(void)
{
  uint16_t i=0;
  for(i=0;i<SDK_Duty_Max;i++)
  {
    SDK_Duty_Status.Status[i].Start_Flag=0;
    SDK_Duty_Status.Status[i].Execute_Flag=0;
    SDK_Duty_Status.Status[i].End_flag=0;
  }
  SDK_Duty_Cnt=0;
}






unsigned char sdk_data_to_send[10];
void SDK_DT_Send_Data(unsigned char *dataToSend , unsigned char length)
{
  USART3_Send(sdk_data_to_send, length);
}

void SDK_DT_Send_Check(unsigned char mode,COM_SDK com)
{
  sdk_data_to_send[0]=0xFF;
  sdk_data_to_send[1]=0xFE;
  sdk_data_to_send[2]=0xA0;
  sdk_data_to_send[3]=2;
  sdk_data_to_send[4]=mode;
  sdk_data_to_send[5]=0
    ;
  u8 sum = 0;
  for(u8 i=0;i<6;i++) sum += sdk_data_to_send[i];
  sdk_data_to_send[6]=sum;
	if(com==UART3_SDK) USART3_Send(sdk_data_to_send, 7);
	if(com==UART0_SDK) USART0_Send(sdk_data_to_send, 7); 
}









#define SDK_TARGET_X_OFFSET  0
#define SDK_TARGET_Y_OFFSET  0//-12


Target_Check Opv_Top_View_Target,Opv_Front_View_Target;
float SDK_Target_Yaw_Gyro=0;

/*************************************************************************/
#define  Pixel_Size_MV    0.0024f//6um=0.000006m=0.0006cm
                       //320---0.0012
                       //160---0.0024
                       //80 ---0.0048
#define  Focal_Length_MV  0.42f

#define OV7725_Sensor_Width_MM    		3.984f//3984um
#define OV7725_Sensor_Height_MM   		2.952f//2952um
#define Pixel_Image_Width_MV    		160//320
#define Pixel_Image_Height_MV   		120//240
#define Pixel_Image_Focal_MM_MV 		4.2f
#define Pixel_Image_View_Angle_X_MV  (56.72/2)//deg(50.75/2)
#define Pixel_Image_View_Angle_Y_MV  (44.07/2)//deg(38.72/2)
/*************************************************************************/
#define  Pixel_Size_CV    0.00056f//cm
													//2592:1944����0.00014cm
													//640:480����0.00056cm
#define  Focal_Length_CV  0.36f  //����3.6mm

#define OV5647_Sensor_Width_MM    		3.674f//3674um
#define OV5647_Sensor_Height_MM   		2.738f//2738.4um#
#define Pixel_Image_Width_CV    			640//640
#define Pixel_Image_Height_CV   			480//480
#define Pixel_Image_Focal_MM_CV 			3.6f
#define Pixel_Image_View_Angle_X_CV  (53.5/2)//ԼΪ66deg���
#define Pixel_Image_View_Angle_Y_CV  (41.4/2)


#define AprilTag_Side_Length  13.6f//cm13.6

float _Pixel_Image_View_Angle_X,_Pixel_Image_View_Angle_Y;
void Get_Camera_Wide_Angle(float view_angle)
{
	float fh=5.0f/FastTan(0.5f*view_angle*DEG2RAD);
  _Pixel_Image_View_Angle_X=2*RAD2DEG*fast_atan(4/fh);
  _Pixel_Image_View_Angle_Y=2*RAD2DEG*fast_atan(3/fh);
}
	
float _P1=0,_P2=0;
uint16_t _CX=0,_CY=0;
float _TX=0,_TY=0;
float _DX=0,_DY=0;
void Sensor_Parameter_Sort(uint16_t tx,uint16_t ty,float pitch,float roll,float alt)
{
//	Get_Camera_Wide_Angle(68);//��������ͷ��ǣ�����õ�X��Y���ӽ�
	float theta_x_max=0,theta_y_max=0;
	theta_x_max=Pixel_Image_View_Angle_X_MV;
	theta_y_max=Pixel_Image_View_Angle_Y_MV;
  _P1=0.5f*Pixel_Image_Width_MV/FastTan(theta_x_max*DEG2RAD);	
	_P2=0.5f*Pixel_Image_Height_MV/FastTan(theta_y_max*DEG2RAD);
	
	_CX=Pixel_Image_Width_MV/2;
	_CY=Pixel_Image_Height_MV/2;
	
	float tmp_x=0,tmp_y=0;
	tmp_x=fast_atan((_CX-tx)/_P1);
	tmp_y=fast_atan((_CY-ty)/_P2);
	
	_TX= FastTan(tmp_x+roll*DEG2RAD) *_P1;
	_TY= FastTan(tmp_y+pitch*DEG2RAD)*_P2;
	
//	_DX=alt*_TX/_P1;
//	_DY=alt*_TY/_P2;
	_DX=0.5f*alt*_TX/_P1;
	_DY=0.5f*alt*_TY/_P2;

	Opv_Top_View_Target.sdk_target.x=_DX;
  Opv_Top_View_Target.sdk_target.y=_DY;
}
















void SDK_DT_Reset()
{
  Opv_Top_View_Target.x=0;
  Opv_Top_View_Target.y=0; 
  Opv_Top_View_Target.pixel=0;
  Opv_Top_View_Target.flag=0;
	Opv_Top_View_Target.state=0;
	Opv_Top_View_Target.angle=0;
	Opv_Top_View_Target.distance=0;
	Opv_Top_View_Target.apriltag_id=0;
	Opv_Top_View_Target.width=0;
	Opv_Top_View_Target.height=0;
	Opv_Top_View_Target.fps=0;
	Opv_Top_View_Target.reserved1=0;
	Opv_Top_View_Target.reserved2=0;
	Opv_Top_View_Target.reserved3=0;
	Opv_Top_View_Target.reserved4=0;
	Opv_Top_View_Target.range_sensor1=0;
	Opv_Top_View_Target.range_sensor2=0;
	Opv_Top_View_Target.range_sensor3=0;
	Opv_Top_View_Target.range_sensor4=0;
	
	Opv_Front_View_Target.x=0;
  Opv_Front_View_Target.y=0; 
  Opv_Front_View_Target.pixel=0;
  Opv_Front_View_Target.flag=0;
	Opv_Front_View_Target.state=0;
	Opv_Front_View_Target.angle=0;
	Opv_Front_View_Target.distance=0;
	Opv_Front_View_Target.apriltag_id=0;
	Opv_Front_View_Target.width=0;
	Opv_Front_View_Target.height=0;
	Opv_Front_View_Target.fps=0;
	Opv_Front_View_Target.reserved1=0;
	Opv_Front_View_Target.reserved2=0;
	Opv_Front_View_Target.reserved3=0;
	Opv_Front_View_Target.reserved4=0;
	Opv_Front_View_Target.range_sensor1=0;
	Opv_Front_View_Target.range_sensor2=0;
	Opv_Front_View_Target.range_sensor3=0;
	Opv_Front_View_Target.range_sensor4=0;
}




uint16_t err_cnt[2]={0};
void Openmv_Data_Receive_Anl_1(uint8_t *data_buf,uint8_t num,Target_Check *target)
{
  uint8_t sum = 0;
  for(uint8_t i=0;i<(num-1);i++)  sum+=*(data_buf+i);
  if(!(sum==*(data_buf+num-1))) 	return;//�������У������
  if(!(*(data_buf)==0xFF && *(data_buf+1)==0xFC))return;//������֡ͷ����
	target->x				  =*(data_buf+4)<<8|*(data_buf+5);
  target->y					=*(data_buf+6)<<8|*(data_buf+7);
	target->pixel			=*(data_buf+8)<<8|*(data_buf+9);  
	target->flag			=*(data_buf+10);
	target->state		  =*(data_buf+11);		
	target->angle		  =*(data_buf+12)<<8|*(data_buf+13);
	target->distance  =*(data_buf+14)<<8|*(data_buf+15);
	target->apriltag_id=*(data_buf+16)<<8|*(data_buf+17);
	target->width	    =*(data_buf+18)<<8|*(data_buf+19);
	target->height	  =*(data_buf+20)<<8|*(data_buf+21);
	target->fps  		  =*(data_buf+22);
	target->reserved1 =*(data_buf+23);
	target->reserved2 =*(data_buf+24);
	target->reserved3 =*(data_buf+25);
	target->reserved4 =*(data_buf+26);
	//��չ���봫����
	target->range_sensor1 =*(data_buf+27)<<8|*(data_buf+28);
	target->range_sensor2 =*(data_buf+29)<<8|*(data_buf+30);
	target->range_sensor3 =*(data_buf+31)<<8|*(data_buf+32);
	target->range_sensor4 =*(data_buf+33)<<8|*(data_buf+34);
	target->camera_id=*(data_buf+35);
	target->reserved1_int32=*(data_buf+36)<<24|*(data_buf+37)<<16|*(data_buf+38)<<8|*(data_buf+39);
	target->reserved2_int32=*(data_buf+40)<<24|*(data_buf+41)<<16|*(data_buf+42)<<8|*(data_buf+43);
	target->reserved3_int32=*(data_buf+44)<<24|*(data_buf+45)<<16|*(data_buf+46)<<8|*(data_buf+47);
	target->reserved4_int32=*(data_buf+48)<<24|*(data_buf+49)<<16|*(data_buf+50)<<8|*(data_buf+51);
	
	target->x_pixel_size=Pixel_Size_MV*(Pixel_Image_Width_MV/target->width);
	target->y_pixel_size=Pixel_Size_MV*(Pixel_Image_Height_MV/target->height);
  target->apriltag_distance=AprilTag_Side_Length*Focal_Length_MV/(target->x_pixel_size*FastSqrt(target->pixel));
  target->sdk_mode=*(data_buf+2);
	
	if(target->camera_id==0x01)//����ͷidΪOPENMV
	{
		switch(target->sdk_mode)
		{
			case 0xA1://����
			{
				target->target_ctrl_enable=target->flag;		
				if(target->flag!=0)  
				{
					if(target->trust_cnt<20)	 
					{
						target->trust_cnt++;
						target->trust_flag=0;
					}
					else target->trust_flag=1;
				}
				else 
				{
					target->trust_cnt/=2;
					target->trust_flag=0;
				}
				target->sdk_target_offset.x=SDK_TARGET_X_OFFSET;
				target->sdk_target_offset.y=SDK_TARGET_Y_OFFSET;	
				Sensor_Parameter_Sort(target->x,target->y,WP_AHRS.Pitch,WP_AHRS.Roll,NamelessQuad.Position[_UP]);		
			}
			break;
			case 0xA2://AprilTag���
			{
				target->target_ctrl_enable=target->flag;		
				if(target->flag!=0)  
				{
					if(target->trust_cnt<20)	 
					{
						target->trust_cnt++;
						target->trust_flag=0;
					}
					else target->trust_flag=1;
				}
				else 
				{
					target->trust_cnt/=2;
					target->trust_flag=0;
				}
				target->sdk_target_offset.x=SDK_TARGET_X_OFFSET;
				target->sdk_target_offset.y=SDK_TARGET_Y_OFFSET;		
				Sensor_Parameter_Sort(target->x,target->y,WP_AHRS.Pitch,WP_AHRS.Roll,NamelessQuad.Position[_UP]);		
			}
			break;
			case 0xA3://�߼��
			{
				target->target_ctrl_enable=target->flag;		
				if(target->flag!=0)  
				{
					if(target->trust_cnt<20)	 
					{
						target->trust_cnt++;
						target->trust_flag=0;
					}
					else target->trust_flag=1;
				}
				else 
				{
					target->trust_cnt/=2;
					target->trust_flag=0;
				}
				target->sdk_target_offset.x=SDK_TARGET_X_OFFSET;
				target->sdk_target_offset.y=SDK_TARGET_Y_OFFSET;
				Sensor_Parameter_Sort(target->x,target->y,WP_AHRS.Pitch,WP_AHRS.Roll,NamelessQuad.Position[_UP]);
				if(target->angle>90) Opv_Top_View_Target.sdk_angle=target->angle-180;
				else Opv_Top_View_Target.sdk_angle=target->angle;			
			}
			break;
			case 0xA4://AprilTag���
			{
				target->target_ctrl_enable=target->flag;		
				if(target->flag!=0)  
				{
					if(target->trust_cnt<20)	 
					{
						target->trust_cnt++;
						target->trust_flag=0;
					}
					else target->trust_flag=1;
				}
				else 
				{
					target->trust_cnt/=2;
					target->trust_flag=0;
				}
				target->sdk_target_offset.x=SDK_TARGET_X_OFFSET;
				target->sdk_target_offset.y=SDK_TARGET_Y_OFFSET;				
				Sensor_Parameter_Sort(target->x,target->y,WP_AHRS.Pitch,WP_AHRS.Roll,NamelessQuad.Position[_UP]);
			}
			break;			
			case 0xA7:
			case 0xAB://ũ������
			{		
				target->target_ctrl_enable=target->flag;		
				if(target->flag!=0)  
				{
					if(target->trust_cnt<3)//�������μ�⵽	 
					{
						target->trust_cnt++;
						target->trust_flag=0;
					}
					else 
					{
						target->trust_flag=1;//�˱�־��Ϊ�жϵײ��Ƿ�Ϊũ����ı�־λ

						//buzzer_setup(1000,0.2f,1);//��������ʾ�����й����зɿذ��ط�������ʾ������������·��в��ȶ��ʹر�
						Bling_Set(&rgb_green,2000,1000,0.5,0,GPIO_PORTF_BASE,BLING_G_PIN,0);//RGB��ɫָʾ����ʾ
					}
				}
				else 
				{
					target->trust_cnt/=2;
					target->trust_flag=0;
				}
				target->sdk_target_offset.x=SDK_TARGET_X_OFFSET;
				target->sdk_target_offset.y=SDK_TARGET_Y_OFFSET;	
				Sensor_Parameter_Sort(target->x,target->y,WP_AHRS.Pitch,WP_AHRS.Roll,NamelessQuad.Position[_UP]);		
			}
			break;
			case 0xB0://�ͻ����˻�����Ŀ����
			{
				target->target_ctrl_enable=target->flag;		
				if(target->flag!=0)  
				{
					if(target->trust_cnt<5)//�������5��	 
					{
						target->trust_cnt++;
						target->trust_flag=0;
					}
					else target->trust_flag=1;
				}
				else 
				{
					target->trust_cnt/=2;
					target->trust_flag=0;
				}
				target->sdk_target_offset.x=SDK_TARGET_X_OFFSET;
				target->sdk_target_offset.y=SDK_TARGET_Y_OFFSET;	
				Sensor_Parameter_Sort(target->x,target->y,WP_AHRS.Pitch,WP_AHRS.Roll,NamelessQuad.Position[_UP]);	
			}
			break;			
			default:
			{
				target->target_ctrl_enable=0;
				target->trust_flag=0;
				target->x=0;
				target->y=0;
			}
		}
	}
}


void Openmv_Data_Receive_Anl_2(uint8_t *data_buf,uint8_t num,Target_Check *target)
{
  uint8_t sum = 0;
	if(!(*(data_buf)==0xFF && *(data_buf+1)==0xFC))return;//������֡ͷ����
	
  for(uint8_t i=0;i<(num-1);i++)  sum+=*(data_buf+i);
  if(!(sum==*(data_buf+num-1))) 	return;//�������У������
  	
	target->x				  	=*(data_buf+4)<<8|*(data_buf+5);
  target->y						=*(data_buf+6)<<8|*(data_buf+7);
	target->pixel				=*(data_buf+8)<<8|*(data_buf+9);  
	target->flag				=*(data_buf+10);
	target->state		  	=*(data_buf+11);		
	target->angle		  	=*(data_buf+12)<<8|*(data_buf+13);
	target->distance  	=*(data_buf+14)<<8|*(data_buf+15);
	target->apriltag_id	=*(data_buf+16)<<8|*(data_buf+17);
	target->width	    	=*(data_buf+18)<<8|*(data_buf+19);
	target->height	  	=*(data_buf+20)<<8|*(data_buf+21);
	target->fps  		  	=*(data_buf+22);
	target->reserved1 	=*(data_buf+23);
	target->reserved2 	=*(data_buf+24);
	target->reserved3 	=*(data_buf+25);
	target->reserved4 	=*(data_buf+26);
	//��չ���봫����
	target->range_sensor1 =*(data_buf+27)<<8|*(data_buf+28);
	target->range_sensor2 =*(data_buf+29)<<8|*(data_buf+30);
	target->range_sensor3 =*(data_buf+31)<<8|*(data_buf+32);
	target->range_sensor4 =*(data_buf+33)<<8|*(data_buf+34);
	target->camera_id=*(data_buf+35);
	target->reserved1_int32=*(data_buf+36)<<24|*(data_buf+37)<<16|*(data_buf+38)<<8|*(data_buf+39);
	target->reserved2_int32=*(data_buf+40)<<24|*(data_buf+41)<<16|*(data_buf+42)<<8|*(data_buf+43);
	target->reserved3_int32=*(data_buf+44)<<24|*(data_buf+45)<<16|*(data_buf+46)<<8|*(data_buf+47);
	target->reserved4_int32=*(data_buf+48)<<24|*(data_buf+49)<<16|*(data_buf+50)<<8|*(data_buf+51);	
	
	target->sdk_mode=*(data_buf+2);
	
	if(target->camera_id==0x01)//����ͷidΪOPENMV
	{
		target->x_pixel_size=Pixel_Size_MV*(Pixel_Image_Width_MV/target->width);
		target->y_pixel_size=Pixel_Size_MV*(Pixel_Image_Height_MV/target->height);
		target->apriltag_distance=AprilTag_Side_Length*Focal_Length_MV/(target->x_pixel_size*FastSqrt(target->pixel));
		
		switch(target->sdk_mode)
		{
			case 0xA4://AprilTag���
			{
				target->target_ctrl_enable=target->flag;		
				if(target->flag!=0)  
				{
					if(target->trust_cnt<20)	 
					{
						target->trust_cnt++;
						target->trust_flag=0;
					}
					else target->trust_flag=1;
				}
				else 
				{
					target->trust_cnt/=2;
					target->trust_flag=0;
				}
				target->sdk_target_offset.x=SDK_TARGET_X_OFFSET;
				target->sdk_target_offset.y=SDK_TARGET_Y_OFFSET;				
				target->sdk_target.x=target->x_cm=(target->x_pixel_size*(0.5*target->width-target->x )*target->apriltag_distance)/Focal_Length_MV;
				target->sdk_target.y=target->y_cm=(target->y_pixel_size*(0.5*target->height-target->y)*target->apriltag_distance)/Focal_Length_MV;		
			}
			break;
			case 0xA5://��ɫ���˼��
			{
				target->target_ctrl_enable=target->flag;		
				if(target->flag!=0)  
				{
					if(target->trust_cnt<20)	 
					{
						target->trust_cnt++;
						target->trust_flag=0;
					}
					else target->trust_flag=1;
				}
				else 
				{
					target->trust_cnt/=2;
					target->trust_flag=0;
				}	
				//front_tofsense_distance
				target->sdk_target.x=target->x_cm=(target->x_pixel_size*(0.5*target->width-target->x )*front_tofsense_distance)/Focal_Length_MV;
				target->sdk_target.y=target->y_cm=(target->y_pixel_size*(0.5*target->height-target->y)*front_tofsense_distance)/Focal_Length_MV;		
			}
			break;
			case 0xA6:
			{
				target->target_ctrl_enable=target->flag;		
				if(target->flag!=0)  
				{
					if(target->trust_cnt<20)	 
					{
						target->trust_cnt++;
						target->trust_flag=0;
					}
					else 
					{
						target->trust_flag=1;
						barcode_id=target->apriltag_id;
					  barcode_flag=1;
					}
				}
				else 
				{
					target->trust_cnt/=2;
					target->trust_flag=0;
				}
				target->sdk_target_offset.x=SDK_TARGET_X_OFFSET;
				target->sdk_target_offset.y=SDK_TARGET_Y_OFFSET;				
				target->sdk_target.x=target->x_cm=(target->x_pixel_size*(0.5*target->width-target->x )*target->apriltag_distance)/Focal_Length_MV;
				target->sdk_target.y=target->y_cm=(target->y_pixel_size*(0.5*target->height-target->y)*target->apriltag_distance)/Focal_Length_MV;	
			}
			break;
			default:
			{
				target->target_ctrl_enable=0;
				target->trust_flag=0;
				target->x=0;
				target->y=0;		
			}
		}
	}
	else if(target->camera_id==0x02)//����ͷidΪOPENCV
	{	
		switch(target->sdk_mode)
		{
			case 0xA4://AprilTag���
			{
				target->target_ctrl_enable=target->flag;		
				if(target->flag!=0)  
				{
					if(target->trust_cnt<20)	 
					{
						target->trust_cnt++;
						target->trust_flag=0;
					}
					else target->trust_flag=1;
				}
				else 
				{
					target->trust_cnt/=2;
					target->trust_flag=0;
				}
				target->sdk_target_offset.x=SDK_TARGET_X_OFFSET;
				target->sdk_target_offset.y=SDK_TARGET_Y_OFFSET;							
//				target->x_cm=-target->reserved1_int32/10.0f;
//				target->y_cm=-target->reserved2_int32/10.0f;
//				target->z_cm= target->reserved3_int32/10.0f;
//				target->sdk_target.x=target->x_cm;
//				target->sdk_target.y=target->y_cm;			
//				target->sdk_target.z=target->z_cm;
//				target->apriltag_distance=target->sdk_target.z;
				
				target->x_pixel_size=Pixel_Size_CV*(Pixel_Image_Width_CV/target->width);
				target->y_pixel_size=Pixel_Size_CV*(Pixel_Image_Height_CV/target->height);
				target->apriltag_distance=target->z_cm=AprilTag_Side_Length*Focal_Length_CV/(target->x_pixel_size*target->pixel);
				target->sdk_target.x=target->x_cm=(target->x_pixel_size*(0.5*target->width-target->x )*target->apriltag_distance)/Focal_Length_CV;
				target->sdk_target.y=target->y_cm=(target->y_pixel_size*(0.5*target->height-target->y)*target->apriltag_distance)/Focal_Length_CV;					
			}
			break;
			case 0xA6:
			{
				target->target_ctrl_enable=target->flag;		
				if(target->flag!=0)  
				{
					if(target->trust_cnt<5)//����5�μ�⵽	 
					{
						target->trust_cnt++;
						target->trust_flag=0;
					}
					else 
					{
						target->trust_flag=1;
						barcode_id=target->apriltag_id;
					  barcode_flag=1;
					}
				}
				else 
				{
					target->trust_cnt/=2;
					target->trust_flag=0;
				}
				//target->sdk_target_offset.x=SDK_TARGET_X_OFFSET;
				//target->sdk_target_offset.y=SDK_TARGET_Y_OFFSET;				
				//target->sdk_target.x=target->x_cm=(target->x_pixel_size*(0.5*target->width-target->x )*target->apriltag_distance)/Focal_Length_CV;
				//target->sdk_target.y=target->y_cm=(target->y_pixel_size*(0.5*target->height-target->y)*target->apriltag_distance)/Focal_Length_CV;	
			}
			break;
			default:
			{
				target->target_ctrl_enable=0;
				target->trust_flag=0;
				target->x=0;
				target->y=0;	
				target->sdk_target.x=target->x_cm=0;
				target->sdk_target.y=target->y_cm=0;		
				target->sdk_target.z=target->z_cm=0;
				target->apriltag_distance=0;				
			}
		}	
	}
}


static uint8_t state[2] = {0};
static uint8_t _data_len[2] = {0},_data_cnt[2] = {0};
uint8_t _buf[2][SDK_Target_Length];
void SDK_Data_Receive_Prepare(uint8_t data,uint8_t label)
{
  if(state[label]==0&&data==0xFF)//֡ͷ1
  {
    state[label]=1;
    _buf[label][0]=data;
  }
  else if(state[label]==1&&data==0xFC)//֡ͷ2
  {
    state[label]=2;
    _buf[label][1]=data;
  }
  else if(state[label]==2&&data<0XFF)//�����ֽ�
  {
    state[label]=3;
    _buf[label][2]=data;
  }
  else if(state[label]==3&&data<50)//���ݳ���
  {
    state[label] = 4;
    _buf[label][3]=data;
    _data_len[label] = data;
    _data_cnt[label] = 0;
  }
  else if(state[label]==4&&_data_len>0)//�ж������ݳ��ȣ��ʹ���ٸ�
  {
    _data_len[label]--;
    _buf[label][4+_data_cnt[label]++]=data;
    if(_data_len[label]==0) state[label] = 5;
  }
  else if(state[label]==5)//����������У���
  {
    state[label] = 0;
    _buf[label][4+_data_cnt[label]]=data;
		if(label==0)  Openmv_Data_Receive_Anl_1(_buf[label],_data_cnt[label]+5,&Opv_Top_View_Target);
		else Openmv_Data_Receive_Anl_2(_buf[label],_data_cnt[label]+5,&Opv_Front_View_Target);
  }
  else state[label] = 0;
}


//SDK1���ݽ���
void SDK_Data_Receive_Prepare_1(uint8_t data)
{
	uint8_t label=0;
  if(state[label]==0&&data==0xFF)//֡ͷ1
  {
    state[label]=1;
    _buf[label][0]=data;
  }
  else if(state[label]==1&&data==0xFC)//֡ͷ2
  {
    state[label]=2;
    _buf[label][1]=data;
  }
  else if(state[label]==2&&data<0XFF)//�����ֽ�
  {
    state[label]=3;
    _buf[label][2]=data;
  }
  else if(state[label]==3&&data<50)//���ݳ���
  {
    state[label] = 4;
    _buf[label][3]=data;
    _data_len[label] = data;
    _data_cnt[label] = 0;
  }
  else if(state[label]==4&&_data_len>0)//�ж������ݳ��ȣ��ʹ���ٸ�
  {
    _data_len[label]--;
    _buf[label][4+_data_cnt[label]++]=data;
    if(_data_len[label]==0) state[label] = 5;
  }
  else if(state[label]==5)//����������У���
  {
    state[label] = 0;
    _buf[label][4+_data_cnt[label]]=data;
		Openmv_Data_Receive_Anl_1(_buf[label],_data_cnt[label]+5,&Opv_Top_View_Target);
  }
  else 
	{
		state[label] = 0;
	}
}


//SDK2���ݽ���
void SDK_Data_Receive_Prepare_2(uint8_t data)
{
	uint8_t label=1;
  if(state[label]==0&&data==0xFF)//֡ͷ1
  {
    state[label]=1;
    _buf[label][0]=data;
  }
  else if(state[label]==1&&data==0xFC)//֡ͷ2
  {
    state[label]=2;
    _buf[label][1]=data;
  }
  else if(state[label]==2&&data<0XFF)//�����ֽ�
  {
    state[label]=3;
    _buf[label][2]=data;
  }
  else if(state[label]==3&&data<50)//���ݳ���
  {
    state[label] = 4;
    _buf[label][3]=data;
    _data_len[label] = data;
    _data_cnt[label] = 0;
  }
  else if(state[label]==4&&_data_len>0)//�ж������ݳ��ȣ��ʹ���ٸ�
  {
    _data_len[label]--;
    _buf[label][4+_data_cnt[label]++]=data;
    if(_data_len[label]==0) state[label] = 5;
  }
  else if(state[label]==5)//����������У���
  {
    state[label] = 0;
    _buf[label][4+_data_cnt[label]]=data;
		Openmv_Data_Receive_Anl_2(_buf[label],_data_cnt[label]+5,&Opv_Front_View_Target);
  }
  else state[label] = 0;
}


uint16_t sdk_offset_cnt1=0;
void SDK_Data_Prase_1(void)
{
  static uint16_t sdk_prase_cnt=0;
  uint16_t i=0;
  sdk_prase_cnt++;
  if(sdk_prase_cnt>=2)//5*2=10ms
  {
    if(COM3_Rx_Buf.Tail<SDK_Target_Length)//0-11����λ���ڴ���
    {
      sdk_offset_cnt1=SDK_Target_Length;
    }
    else//12-23����λ���ڴ���
    {
      sdk_offset_cnt1=0;
    }
    for(i=0;i<SDK_Target_Length;i++)
    {
      SDK_Data_Receive_Prepare(COM3_Rx_Buf.Ring_Buff[sdk_offset_cnt1+i],0);
    }
    sdk_prase_cnt=0;
  }
}


uint16_t sdk_offset_cnt2=0;
void SDK_Data_Prase_2(void)
{
  static uint16_t sdk_prase_cnt=0;
  uint16_t i=0;
  sdk_prase_cnt++;
  if(sdk_prase_cnt>=2)//5*2=10ms
  {
    if(COM0_Rx_Buf.Tail<SDK_Target_Length)//0-11����λ���ڴ���
    {
      sdk_offset_cnt2=SDK_Target_Length;
    }
    else//12-23����λ���ڴ���
    {
      sdk_offset_cnt2=0;
    }
    for(i=0;i<SDK_Target_Length;i++)
    {
      SDK_Data_Receive_Prepare(COM0_Rx_Buf.Ring_Buff[sdk_offset_cnt2+i],1);
    }
    sdk_prase_cnt=0;
  }
}





void SDK_Init(void)
{
  float sdk_mode_default1=0,sdk_mode_default2=0,task_select=0;
  SDK_DT_Reset();//��λSDK�߼������
  ReadFlashParameterOne(SDK1_MODE_DEFAULT,&sdk_mode_default1);
  if(isnan(sdk_mode_default1)==0)
  {
    SDK1_Mode_Setup=(uint8_t)(sdk_mode_default1);//����
    SDK_DT_Send_Check(SDK1_Mode_Setup,UART3_SDK);//��ʼ������opemmv����ģʽ��Ĭ�����ϴι���״̬����
  }
	
  ReadFlashParameterOne(FORWARD_VISION_MODE_DEFAULT,&sdk_mode_default2);
  if(isnan(sdk_mode_default2)==0)
  {
    Forward_Vision_Mode_Setup=(uint8_t)(sdk_mode_default2);//ǰ��
    SDK_DT_Send_Check(Forward_Vision_Mode_Setup,UART0_SDK);//��ʼ��ǰ��opemmv����ģʽ��Ĭ�����ϴι���״̬����
  } 

  ReadFlashParameterOne(TASK_SELECT_AFTER_TAKEOFF,&task_select);
  if(isnan(task_select)==0) task_select_cnt=(int16_t)(task_select);//�Զ���ɺ�����ѡ��	
	else task_select=1;
}





