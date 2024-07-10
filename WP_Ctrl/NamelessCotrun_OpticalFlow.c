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
#include "NamelessCotrun_OpticalFlow.h"



#include <Time.h>
void Third_Order_Complementarity(float flow_height,Vector2f accel,Vector2f flow);

#define OpticalFlow_Ringbuf  COM2_Rx_Buf



uint8_t OpticalFlow_Is_Work=0;
float opticalflow_high=1000;//Ĭ��1m=100cm=1000mm
flow_float opt_data;
flow_integral_frame opt_origin_data;
Vector2f opt_filter_data; //�˲��������λ��
Vector2f opt_gyro_data;   //�������ٶ�
Vector2f opt_gyro_filter_data;//����������ת�����˲���Ľ��ٶ�
Vector2f gyro_filter_data;//��ͨͬ����λ�Ľ��ٶ�
Vector2f gyro_filter_data_backup[20];//��ͨͬ����λ�Ľ��ٶ�
lpf_param OpticalFlow_Parameter,OpticalFlow_gyro_lpf_param;
lpf_buf Buffer_OpticalFlow[2],Buffer_OpticalFlow_Gyro[2];
uint8_t Optical_Type_Present=1;



void OpticalFlow_Init()
{
  set_cutoff_frequency(50, 20,&OpticalFlow_Parameter);
  set_cutoff_frequency(50, 6,&OpticalFlow_gyro_lpf_param);//ͬ����λ

  float optical_type_default=0;
  ReadFlashParameterOne(OPTICAL_TYPE,&optical_type_default);
  if(isnan(optical_type_default)==0)
  {
		Optical_Type_Present=(uint8_t)(optical_type_default);
  } 
	
  //���ݲ�ͬ�Ĺ������ã����ö�Ӧ���ȵ����ݻ�����	
	if(Optical_Type_Present==1)//LC307
	{
		OpticalFlow_Is_Work=Config_Init_Uart();//������������ʼ��
	}
	else if(Optical_Type_Present==2)//LC302
	{
	  OpticalFlow_Is_Work=Config_Init_Uart();//������������ʼ��;
	}
	else if(Optical_Type_Present==3||Optical_Type_Present==4)//lidar or T265
	{
	  OpticalFlow_Is_Work=1;
	}
}



systime optical_t;
uint8_t lc30x_buf[20];
void LC30X_Data_Receive_Anl(uint8_t *data_buf);//50hz
void LC30X_OpticalFlow_Sense_Prase(uint8_t data,uint8_t *buf)
{
  static uint8_t data_len = 0,data_cnt = 0;
  static uint8_t state = 0;
  if(state==0&&data==0xfe)//�ж�֡ͷ1
  {
    state=1;
    buf[0]=data;
  }
  else if(state==1&&data==0x0a)//�ж�֡ͷ2
  {
    state=2;
    buf[1]=data;
		data_len = 10;
		data_cnt = 0;
  }
  else if(state==2&&data_len>0)//���ݽ���
  {
    data_len--;
    buf[2+data_cnt++]=data;
    if(data_len==0)  state = 3;
  }
  else if(state==3)//���У��
  {
    state = 4;
    buf[2+data_cnt++]=data;
  }
  else if(state==4)//�ж�֡ͷβ
  {
		static unsigned char last_valid=0;
		static uint32_t start_t,end_t;
		last_valid=opt_data.valid;
    state = 0;
    buf[2+data_cnt++]=data;
		
		
		LC30X_Data_Receive_Anl(buf);
		Get_Systime(&optical_t);	
		if(last_valid==1&&opt_data.valid==0)//��¼�״�ʧЧʱ��
		{
			start_t=millis();
			opt_data.ctrl_valid=1;
		}
		else if(last_valid==0&&opt_data.valid==0)//������¼ʧЧʱ��
		{
			end_t=millis();					
			if(end_t-start_t>=1000)//����ʧЧ1000ms����Ϊ�����ѹ��ϣ�������������
			{
				opt_data.ctrl_valid=0;
			}
		}
		else//������������
		{
			opt_data.ctrl_valid=1;
		}
  }
  else state = 0;
}




void opticalflow_pretreat(void)
{
	opt_filter_data.x=LPButterworth(opt_origin_data.pixel_flow_x_integral,&Buffer_OpticalFlow[0],&OpticalFlow_Parameter);
	opt_filter_data.y=LPButterworth(opt_origin_data.pixel_flow_y_integral,&Buffer_OpticalFlow[1],&OpticalFlow_Parameter);   
	opt_data.x=(opt_origin_data.pixel_flow_x_integral*opticalflow_high)/10000.0f;//��λ:���Ը߶ȵ�λmm��Ϊʵ��λ��mm
	opt_data.y=(opt_origin_data.pixel_flow_y_integral*opticalflow_high)/10000.0f;//��λ:���Ը߶ȵ�λmm��Ϊʵ��λ��mm     
	opt_data.dt=(int16_t)(opt_origin_data.integration_timespan*0.001f);//��λms
	opt_data.qual=opt_origin_data.qual;
	if(opt_data.qual==0xF5) opt_data.valid=1;
	else opt_data.valid=0;	

	opt_gyro_data.x=(opt_filter_data.x)/200.0f;//�������ٶ�rad/s
	opt_gyro_data.y=(opt_filter_data.y)/200.0f;//�������ٶ�rad/s         
	gyro_filter_data.x=LPButterworth(WP_AHRS.Roll_Gyro,&Buffer_OpticalFlow_Gyro[0],&OpticalFlow_gyro_lpf_param)/57.3f;//��������λͬ�����ٶ�
	gyro_filter_data.y=LPButterworth(WP_AHRS.Pitch_Gyro,&Buffer_OpticalFlow_Gyro[1],&OpticalFlow_gyro_lpf_param)/57.3f;//��������λͬ�����ٶ�
	
	for(uint16_t i=19;i>0;i--)
	{
		gyro_filter_data_backup[i].x=gyro_filter_data_backup[i-1].x;
		gyro_filter_data_backup[i].y=gyro_filter_data_backup[i-1].y;
	} 
	gyro_filter_data_backup[0].x=gyro_filter_data.x;
	gyro_filter_data_backup[0].y=gyro_filter_data.y;
	
	
	opt_gyro_filter_data.x=OpticalFlow_Rotate_Complementary_Filter(opt_gyro_data.x,gyro_filter_data_backup[0].x,'x',opt_data.valid);//�������ٶ��������ǽ��ٶ��ں� 
	opt_gyro_filter_data.y=OpticalFlow_Rotate_Complementary_Filter(opt_gyro_data.y,gyro_filter_data_backup[0].y,'y',opt_data.valid); //�������ٶ��������ǽ��ٶ��ں� 
}
	
	

uint8_t Optflow_Is_Okay=0;
float rpi_opticalflow_scale=0.3f;
void LC30X_Data_Receive_Anl(uint8_t *data_buf)//50hz
{	
	uint8_t xor_check = *(data_buf+2);
  for(uint8_t i=3;i<12;i++)  xor_check^=*(data_buf+i);
	if(!(*(data_buf)==0xfe&&*(data_buf+1)==0x0a))	return;//������֡ͷ����
	if(!(*(data_buf+13)==0x55)) 	return;
  //if(!(xor_check==*(data_buf+12))) return;		
	Optflow_Is_Okay=1;
	switch(Optical_Type_Present)
	{
		case 1://LC307
		{
			opt_origin_data.pixel_flow_x_integral=(int16_t)(data_buf[3]<<8)|data_buf[2];
			opt_origin_data.pixel_flow_y_integral=(int16_t)(data_buf[5]<<8)|data_buf[4];			
			opt_origin_data.pixel_flow_x_integral*=(-1);					
			opt_origin_data.pixel_flow_y_integral*=(-1);				
			opt_origin_data.integration_timespan= (int16_t)(data_buf[7]<<8)|data_buf[6];
			opt_origin_data.qual=data_buf[10]; 
		}
		break;
		case 2://LC302
		{
			opt_origin_data.pixel_flow_x_integral=(int16_t)(data_buf[3]<<8)|data_buf[2];
			opt_origin_data.pixel_flow_y_integral=(int16_t)(data_buf[5]<<8)|data_buf[4];						
			opt_origin_data.integration_timespan= (int16_t)(data_buf[7]<<8)|data_buf[6];
			opt_origin_data.qual=data_buf[10]; 	
		}
		break;	
		default://LC302
		{
			opt_origin_data.pixel_flow_x_integral=(int16_t)(data_buf[3]<<8)|data_buf[2];
			opt_origin_data.pixel_flow_y_integral=(int16_t)(data_buf[5]<<8)|data_buf[4];						
			opt_origin_data.integration_timespan= (int16_t)(data_buf[7]<<8)|data_buf[6];
			opt_origin_data.qual=data_buf[10];      
		}
	}
	opticalflow_pretreat();
}


void Optflow_Statemachine(void)
{
	if(Temperature_Stable_Flag==0||Gyro_Safety_Calibration_Flag==0) return;//�¿ؾ�λ��Ž����ں�
  if(OpticalFlow_Is_Work==1)//����ʼ��ʱ�����ڹ�������
  {
		switch(Optical_Type_Present)
		{
			case 1:
			case 2:Third_Order_Complementarity(NamelessQuad.Position[_UP],SINS_Accel_Body,opt_gyro_filter_data);break;
			case 3:
			case 4:VIO_SLAM_INS_CF();break;//altitude_kalman_filter(&slam_kf,&VIO_SINS,0.005f);break;
			default:	;	
		}
  }
}



float OpticalFlow_Rotate_Complementary_Filter(float optflow_gyro,float gyro,uint8_t axis,uint8_t valid)
{
  float optflow_gyro_filter=0;
  if(Optical_Type_Present==1||Optical_Type_Present==2)
  {
		optflow_gyro_filter=optflow_gyro-constrain_float(gyro,-ABS(optflow_gyro),ABS(optflow_gyro));
		//optflow_gyro_filter=optflow_gyro-constrain_float(gyro,-5.0f,5.0f);		
  }
  return optflow_gyro_filter;
}



SINS_Lite OpticalFlow_SINS,VIO_SINS;
Testime Optical_Delta;
Vector2f OpticalFlow_Position={0};
Vector2f OpticalFlow_Speed={0};
Vector2f OpticalFlow_Speed_Err={0};
Vector2f OpticalFlow_Position_Err={0};
uint16_t Optflow_Sync_Cnt=5;
float CF_Parameter[2]={0.05f,0.1f};//���������˲�Ȩ�� 0.1  0.1   0.08  0
//����λ���ں�Ȩ�ظ�Ϊ0����ʾ����������λ����������Ϊ�ͳɱ�����ģ��Ư�ƽϴ�����Ը���Сֵ��0.2f
#define Optical_Output_Dt  0.02f//50hz
void  OpticalFlow_INS_CF(float flow_height,Vector2f accel,Vector2f flow)
{
  float use_height=0;
  float optical_dt=0;
  Vector2f speed_delta={0};
  Test_Period(&Optical_Delta);
  optical_dt=Optical_Delta.Time_Delta/1000.0f;
	if(optical_dt>1.05f*WP_Duty_Dt||optical_dt<0.95f*WP_Duty_Dt||isnan(optical_dt)!=0)   optical_dt=WP_Duty_Dt; 
	use_height = constrain_float(flow_height,5,10000);
  if(Optflow_Is_Okay==1&&opt_data.valid==1)//�������ݹ�������ʱ��20msһ��
  {  
    OpticalFlow_Speed.x=flow.x*use_height;//�����ٶ�
    OpticalFlow_Speed.y=flow.y*use_height;//�����ٶ�
    OpticalFlow_Position.x+=OpticalFlow_Speed.x*Optical_Output_Dt;//����λ��
    OpticalFlow_Position.y+=OpticalFlow_Speed.y*Optical_Output_Dt;//����λ��

    Optflow_Is_Okay=0;
    OpticalFlow_Position_Err.x=OpticalFlow_Position.x-OpticalFlow_SINS.Pos_Backups[_EAST][Optflow_Sync_Cnt];
    OpticalFlow_Position_Err.y=OpticalFlow_Position.y-OpticalFlow_SINS.Pos_Backups[_NORTH][Optflow_Sync_Cnt];
		OpticalFlow_Speed_Err.x=OpticalFlow_Speed.x-OpticalFlow_SINS.Vel_Backups[_EAST][Optflow_Sync_Cnt];
    OpticalFlow_Speed_Err.y=OpticalFlow_Speed.y-OpticalFlow_SINS.Vel_Backups[_NORTH][Optflow_Sync_Cnt];
		
		OpticalFlow_Speed_Err.x=constrain_float(OpticalFlow_Speed_Err.x,-200,200);
		OpticalFlow_Speed_Err.y=constrain_float(OpticalFlow_Speed_Err.y,-200,200);
  }
  else
  {
    OpticalFlow_Speed_Err.x=0;
    OpticalFlow_Speed_Err.y=0;
    OpticalFlow_Position_Err.x=0;
    OpticalFlow_Position_Err.y=0;
  }
  
  OpticalFlow_SINS.Acceleration[_EAST]=-accel.x;//�ߵ����ٶ���������,��ͷ���Ϊ��
  OpticalFlow_SINS.Acceleration[_NORTH] = accel.y;//�ߵ����ٶ��������ͷ,��ͷǰ��Ϊ��
  
	speed_delta.x=OpticalFlow_SINS.Acceleration[_EAST]*optical_dt;
  speed_delta.y=OpticalFlow_SINS.Acceleration[_NORTH]*optical_dt;    
  
	OpticalFlow_SINS.Position[_EAST]+=OpticalFlow_SINS.Speed[_EAST]*optical_dt
    +0.5f*speed_delta.x*optical_dt+CF_Parameter[1]*OpticalFlow_Position_Err.x;
  OpticalFlow_SINS.Position[_NORTH]+=OpticalFlow_SINS.Speed[_NORTH]*optical_dt
    +0.5f*speed_delta.y*optical_dt+CF_Parameter[1]*OpticalFlow_Position_Err.y;

  OpticalFlow_SINS.Speed[_EAST]+=OpticalFlow_SINS.Acceleration[_EAST]*optical_dt+CF_Parameter[0]*OpticalFlow_Speed_Err.x;
  OpticalFlow_SINS.Speed[_NORTH]+=OpticalFlow_SINS.Acceleration[_NORTH]*optical_dt+CF_Parameter[0]*OpticalFlow_Speed_Err.y; 
   	
	for(uint16_t i=Num-1;i>0;i--)
	{
		OpticalFlow_SINS.Pos_Backups[_NORTH][i]=OpticalFlow_SINS.Pos_Backups[_NORTH][i-1];
		OpticalFlow_SINS.Pos_Backups[_EAST][i]=OpticalFlow_SINS.Pos_Backups[_EAST][i-1];
		OpticalFlow_SINS.Vel_Backups[_NORTH][i]=OpticalFlow_SINS.Vel_Backups[_NORTH][i-1];
		OpticalFlow_SINS.Vel_Backups[_EAST][i]=OpticalFlow_SINS.Vel_Backups[_EAST][i-1]; 		
	}   
	OpticalFlow_SINS.Pos_Backups[_NORTH][0]=OpticalFlow_SINS.Position[_NORTH];
  OpticalFlow_SINS.Pos_Backups[_EAST][0]=OpticalFlow_SINS.Position[_EAST]; 
  OpticalFlow_SINS.Vel_Backups[_NORTH][0]=OpticalFlow_SINS.Speed[_NORTH];
  OpticalFlow_SINS.Vel_Backups[_EAST][0]=OpticalFlow_SINS.Speed[_EAST];  	 
}







Vector3s correct[2];
float K_ACC_OPT=1.0f,K_VEL_OPT=3.0f,K_POS_OPT=5.0f;
void Third_Order_Complementarity(float flow_height,Vector2f accel,Vector2f flow)
{	
	float use_height=constrain_float(flow_height,5,10000);
	float obs_err[2];
	Vector2f acc,speed_delta={0};
	if(Optflow_Is_Okay==1&&opt_data.valid==1)//�������ݹ�������ʱ��20msһ��
  {  
    OpticalFlow_Speed.x=flow.x*use_height;//�����ٶ�
    OpticalFlow_Speed.y=flow.y*use_height;//�����ٶ�
    OpticalFlow_Position.x+=OpticalFlow_Speed.x*Optical_Output_Dt;//����λ��
    OpticalFlow_Position.y+=OpticalFlow_Speed.y*Optical_Output_Dt;//����λ��
		
    Optflow_Is_Okay=0;
    OpticalFlow_Position_Err.x=OpticalFlow_Position.x-OpticalFlow_SINS.Pos_Backups[_EAST][Optflow_Sync_Cnt];
    OpticalFlow_Position_Err.y=OpticalFlow_Position.y-OpticalFlow_SINS.Pos_Backups[_NORTH][Optflow_Sync_Cnt];

		OpticalFlow_Speed_Err.x=OpticalFlow_Speed.x-OpticalFlow_SINS.Vel_Backups[_EAST][Optflow_Sync_Cnt];
    OpticalFlow_Speed_Err.y=OpticalFlow_Speed.y-OpticalFlow_SINS.Vel_Backups[_NORTH][Optflow_Sync_Cnt];		
		OpticalFlow_Speed_Err.x=constrain_float(OpticalFlow_Speed_Err.x,-200,200);
		OpticalFlow_Speed_Err.y=constrain_float(OpticalFlow_Speed_Err.y,-200,200);	
	}
//	else
//	{
//    OpticalFlow_Position_Err.x=0;
//    OpticalFlow_Position_Err.y=0;
//		OpticalFlow_Speed_Err.x=0;
//    OpticalFlow_Speed_Err.y=0;			
//	}
	obs_err[0]=OpticalFlow_Position_Err.x;
	obs_err[1]=OpticalFlow_Position_Err.y;
	//��·���ַ����������ߵ�
	correct[0].acc +=obs_err[0]* K_ACC_OPT*0.005f;//���ٶȽ�����
	correct[1].acc +=obs_err[1]* K_ACC_OPT*0.005f;//���ٶȽ���
	correct[0].acc=constrain_float(correct[0].acc,-50,50);
	correct[1].acc=constrain_float(correct[1].acc,-50,50);
	correct[0].vel  =OpticalFlow_Speed_Err.x*K_VEL_OPT*0.005f;//�ٶȽ�����
	correct[1].vel  =OpticalFlow_Speed_Err.y*K_VEL_OPT*0.005f;//�ٶȽ�����
	correct[0].pos  =obs_err[0]* K_POS_OPT*0.005f;//λ�ý�����	
	correct[1].pos  =obs_err[1]* K_POS_OPT*0.005f;//λ�ý�����
	
	acc.x=-accel.x;//�ߵ����ٶ���������,��ͷ���Ϊ��
	acc.y= accel.y;//�ߵ����ٶ��������ͷ,��ͷǰ��Ϊ��
	
  OpticalFlow_SINS.Acceleration[_EAST]= acc.x+correct[0].acc;//�ߵ����ٶ���������,��ͷ���Ϊ��
  OpticalFlow_SINS.Acceleration[_NORTH] = acc.y+correct[1].acc;//�ߵ����ٶ��������ͷ,��ͷǰ��Ϊ��
  
	speed_delta.x=(OpticalFlow_SINS.Acceleration[_EAST])*0.005f;
  speed_delta.y=(OpticalFlow_SINS.Acceleration[_NORTH])*0.005f; 
 	
	OpticalFlow_SINS.Position[_EAST]+=OpticalFlow_SINS.Speed[_EAST]*0.005f+0.5f*speed_delta.x*0.005f+correct[0].pos;
	OpticalFlow_SINS.Position[_NORTH] +=OpticalFlow_SINS.Speed[_NORTH] *0.005f+0.5f*speed_delta.y*0.005f+correct[1].pos;
	OpticalFlow_SINS.Speed[_EAST]+=speed_delta.x+correct[0].vel;
	OpticalFlow_SINS.Speed[_NORTH] +=speed_delta.y+correct[1].vel;	
	
	for(uint16_t i=Num-1;i>0;i--)
	{
		OpticalFlow_SINS.Pos_Backups[_NORTH][i]=OpticalFlow_SINS.Pos_Backups[_NORTH][i-1];
		OpticalFlow_SINS.Pos_Backups[_EAST][i]=OpticalFlow_SINS.Pos_Backups[_EAST][i-1];
		OpticalFlow_SINS.Vel_Backups[_NORTH][i]=OpticalFlow_SINS.Vel_Backups[_NORTH][i-1];
		OpticalFlow_SINS.Vel_Backups[_EAST][i]=OpticalFlow_SINS.Vel_Backups[_EAST][i-1]; 		
	}   
	OpticalFlow_SINS.Pos_Backups[_NORTH][0]=OpticalFlow_SINS.Position[_NORTH];
  OpticalFlow_SINS.Pos_Backups[_EAST][0]=OpticalFlow_SINS.Position[_EAST]; 
  OpticalFlow_SINS.Vel_Backups[_NORTH][0]=OpticalFlow_SINS.Speed[_NORTH];
  OpticalFlow_SINS.Vel_Backups[_EAST][0]=OpticalFlow_SINS.Speed[_EAST];  
}



void from_vio_to_body_frame(float map_x,float map_y,float *bod_x,float *bod_y,float _yaw)
{
	float _cos=FastCos(_yaw*DEG2RAD);
	float _sin=FastSin(_yaw*DEG2RAD);
	*bod_x= -map_x*_cos-map_y*_sin;
	*bod_y= -map_x*_sin+map_y*_cos;
}


void from_body_to_nav_frame(float bod_x,float bod_y,float *map_x,float *map_y,float _yaw)
{
	float _cos=FastCos(_yaw*DEG2RAD);
	float _sin=FastSin(_yaw*DEG2RAD);
	*map_x= bod_x*_cos-bod_y*_sin;
	*map_y= bod_x*_sin+bod_y*_cos;
}

void from_slam_to_nav_frame(float slam_e,float slam_n,float *map_e,float *map_n,float _yaw)
{
	float _cos=FastCos(_yaw*DEG2RAD);
	float _sin=FastSin(_yaw*DEG2RAD);
	*map_e= slam_e*_cos+slam_n*_sin;
	*map_n=-slam_e*_sin+slam_n*_cos;
}

//30  300  100  10
#define K_ACC_RPLIDAR 30.0f
#define K_VEL_RPLIDAR 400.0f//range(300,1000)500
#define K_POS_RPLIDAR 100.0f//range(50,150)100
#define SYNC_CNT_RPLIDAR 5//10  5
//5  120  30 5
#define K_ACC_T265    5.0f
#define K_VEL_T265    120.0f
#define K_POS_T265    30.0f
#define SYNC_CNT_T265 5
/*********************************************/
//30  100  150  10
//50  150  100  10
float K_ACC_LOAM    =50.0f;
float K_VEL_LOAM    =150.0f;
float K_POS_LOAM    =100.0f;
float SYNC_CNT_LOAM =10;
/*********************************************/
float slam_fusion_param[3]={0};
uint16_t slam_sync_cnt=0;
void  VIO_SLAM_INS_CF(void)
{
  Vector2f acc={0,0},speed_delta={0,0};
	float obs_err[2];
	switch(current_state.slam_sensor)
	{
		case NO_SLAM:
		{
			return ;
		}
		case LIDAR_2D_SLAM:
		{
			slam_fusion_param[0]=K_ACC_RPLIDAR;
			slam_fusion_param[1]=K_VEL_RPLIDAR;
			slam_fusion_param[2]=K_POS_RPLIDAR;	
			slam_sync_cnt=SYNC_CNT_RPLIDAR;
			acc.x= NamelessQuad.Inertial_Acceleration[_EAST];//�ߵ����ٶ���������,��ͷ�Ҳ�Ϊ��
			acc.y= NamelessQuad.Inertial_Acceleration[_NORTH];//�ߵ����ٶ��������ͷ,��ͷǰ��Ϊ��			
		}
		break;
		case T265_SLAM:
		{
			slam_fusion_param[0]=K_ACC_T265;
			slam_fusion_param[1]=K_VEL_T265;
			slam_fusion_param[2]=K_POS_T265;	
			slam_sync_cnt=SYNC_CNT_T265;
			acc.x= NamelessQuad.Inertial_Acceleration[_EAST];//�ߵ����ٶ���������,��ͷ�Ҳ�Ϊ��
			acc.y= NamelessQuad.Inertial_Acceleration[_NORTH];//�ߵ����ٶ��������ͷ,��ͷǰ��Ϊ��			
		}
		break;
		case LOAM:
		{
			slam_fusion_param[0]=K_ACC_LOAM;
			slam_fusion_param[1]=K_VEL_LOAM;
			slam_fusion_param[2]=K_POS_LOAM;	
			slam_sync_cnt=SYNC_CNT_LOAM;
			acc.x= NamelessQuad.Inertial_Acceleration[_EAST];//�ߵ����ٶ���������,��ͷ�Ҳ�Ϊ��
			acc.y= NamelessQuad.Inertial_Acceleration[_NORTH];//�ߵ����ٶ��������ͷ,��ͷǰ��Ϊ��			
		}
		break;
		default:	return;		
	}

  if(current_state.update_flag==1)//��������SLAM����ʱ��100msһ��
  { 
		current_state.valid=1;
		current_state.update_flag=0; 
		
		OpticalFlow_Position.x= current_state.position_x;
		OpticalFlow_Position.y= current_state.position_y;	
		
    OpticalFlow_Position_Err.x=OpticalFlow_Position.x-VIO_SINS.Pos_Backups[_EAST][slam_sync_cnt];
    OpticalFlow_Position_Err.y=OpticalFlow_Position.y-VIO_SINS.Pos_Backups[_NORTH][slam_sync_cnt];
  }
	else
	{
		OpticalFlow_Position_Err.x=0;
		OpticalFlow_Position_Err.y=0;
	}
	obs_err[0]=OpticalFlow_Position_Err.x;
	obs_err[1]=OpticalFlow_Position_Err.y;
	//��·�����������ߵ�
	correct[0].acc +=obs_err[0]* slam_fusion_param[0]*0.005f;//���ٶȽ�����
	correct[1].acc +=obs_err[1]* slam_fusion_param[0]*0.005f;//���ٶȽ���
	correct[0].acc=constrain_float(correct[0].acc,-50,50);
	correct[1].acc=constrain_float(correct[1].acc,-50,50);
	correct[0].vel  =obs_err[0]* slam_fusion_param[1]*0.005f;//�ٶȽ�����
	correct[1].vel  =obs_err[1]* slam_fusion_param[1]*0.005f;//�ٶȽ�����
	correct[0].pos  =obs_err[0]* slam_fusion_param[2]*0.005f;//λ�ý�����	
	correct[1].pos  =obs_err[1]* slam_fusion_param[2]*0.005f;//λ�ý�����
	
  VIO_SINS.Acceleration[_EAST] = acc.x+correct[0].acc;//�ߵ����ٶ���������,��ͷ���Ϊ��
  VIO_SINS.Acceleration[_NORTH]= acc.y+correct[1].acc;//�ߵ����ٶ��������ͷ,��ͷǰ��Ϊ��
  
	speed_delta.x=VIO_SINS.Acceleration[_EAST] *0.005f;
  speed_delta.y=VIO_SINS.Acceleration[_NORTH]*0.005f; 
 	
	VIO_SINS.Position[_EAST]  +=VIO_SINS.Speed[_EAST] *0.005f+0.5f*speed_delta.x*0.005f+correct[0].pos;
	VIO_SINS.Position[_NORTH] +=VIO_SINS.Speed[_NORTH]*0.005f+0.5f*speed_delta.y*0.005f+correct[1].pos;
	VIO_SINS.Speed[_EAST]  +=speed_delta.x+correct[0].vel;
	VIO_SINS.Speed[_NORTH] +=speed_delta.y+correct[1].vel;	
	
	for(uint16_t i=Num-1;i>0;i--)
	{
		VIO_SINS.Pos_Backups[_NORTH][i]=VIO_SINS.Pos_Backups[_NORTH][i-1];
		VIO_SINS.Pos_Backups[_EAST][i]=VIO_SINS.Pos_Backups[_EAST][i-1];
		VIO_SINS.Vel_Backups[_NORTH][i]=VIO_SINS.Vel_Backups[_NORTH][i-1];
		VIO_SINS.Vel_Backups[_EAST][i]=VIO_SINS.Vel_Backups[_EAST][i-1]; 		
	}   
	VIO_SINS.Pos_Backups[_NORTH][0]=VIO_SINS.Position[_NORTH];
  VIO_SINS.Pos_Backups[_EAST][0]=VIO_SINS.Position[_EAST]; 
  VIO_SINS.Vel_Backups[_NORTH][0]=VIO_SINS.Speed[_NORTH];
  VIO_SINS.Vel_Backups[_EAST][0]=VIO_SINS.Speed[_EAST];
	
	from_vio_to_body_frame(VIO_SINS.Position[_EAST],
												 VIO_SINS.Position[_NORTH],
												 &OpticalFlow_SINS.Position[_EAST],
												 &OpticalFlow_SINS.Position[_NORTH],
												 WP_AHRS.Yaw);
	
	from_vio_to_body_frame(VIO_SINS.Speed[_EAST],
												 VIO_SINS.Speed[_NORTH],
												 &OpticalFlow_SINS.Speed[_EAST],
												 &OpticalFlow_SINS.Speed[_NORTH],
												 WP_AHRS.Yaw);	
}
