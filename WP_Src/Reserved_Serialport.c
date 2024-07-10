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
#include "Reserved_Serialport.h"


void NCLink_Data_Prase_Process_Lite(uint8_t *data_buf,uint8_t num);

void quad_getangle(float *q,float* rpy)
{
		rpy[1]= RADTODEG(FastAtan2(q[0]*q[1] + q[2]*q[3], 0.5f - q[1]*q[1] - q[2]*q[2]));
		rpy[0]= RADTODEG(FastAsin(-2.0f * (q[1]*q[3] - q[0]*q[2])));
		rpy[2]= RADTODEG(FastAtan2(q[1]*q[2] + q[0]*q[3], 0.5f - q[2]*q[2] - q[3]*q[3]));
}


void Reserved_Serialport_Init()
{
	switch(Reserved_Uart)
	{
		case FRONT_RANGE_FINDER:
			ConfigureUART6(9600,10);
		break;
		case GPS_M8N:
			Set_GPS_USART();
		break;
		case THIRD_PARTY_STATE:
			ConfigureUART6(460800,60*2);	//460800	
		break;
		default:Set_GPS_USART();
	}
}

static uint8_t NCLink_Head[2]={0xFF,0xFC};//����֡ͷ
static uint8_t NCLink_End[2] ={0xA1,0xA2};//����֡β
static uint8_t nclink_buf[100];//���������ݻ�����
static uint8_t send_nclink_buf[100];//���������ݻ�����
uint8_t send_check_back=0;
void Reserved_Serialport_Send(uint8_t *pui8Buffer, uint32_t ui32Count)//����N���ֽڳ��ȵ�����
{
  while(ui32Count--)
  {
    UARTCharPut(UART6_BASE, *pui8Buffer++);
  }
}

void NCLink_Send_IMU_Feedback(float q1	,float q2,float q3  ,float q0,
															float gx,float gy,float gz,
															float ax,float ay,float az,
															float px,float py,float pz,
															float vx,float vy,float vz)
{
  uint8_t sum=0,_cnt=0,i=0;
  send_nclink_buf[_cnt++]=NCLink_Head[0];
  send_nclink_buf[_cnt++]=NCLink_Head[1];
  send_nclink_buf[_cnt++]=0xE0;
  send_nclink_buf[_cnt++]=0;
		
	Float2Byte(&q1,send_nclink_buf,_cnt);//4
	_cnt+=4;
	Float2Byte(&q2,send_nclink_buf,_cnt);//8
	_cnt+=4;
	Float2Byte(&q3,send_nclink_buf,_cnt);//12
	_cnt+=4;
	Float2Byte(&q0,send_nclink_buf,_cnt);//16
	_cnt+=4;
	Float2Byte(&gx,send_nclink_buf,_cnt);//20
	_cnt+=4;
	Float2Byte(&gy,send_nclink_buf,_cnt);//24
	_cnt+=4;
	Float2Byte(&gz,send_nclink_buf,_cnt);//28
	_cnt+=4;
	Float2Byte(&ax,send_nclink_buf,_cnt);//32
	_cnt+=4;
	Float2Byte(&ay,send_nclink_buf,_cnt);//36
	_cnt+=4;
	Float2Byte(&az,send_nclink_buf,_cnt);//40
	_cnt+=4;
	
	Float2Byte(&px,send_nclink_buf,_cnt);//44
	_cnt+=4;
	Float2Byte(&py,send_nclink_buf,_cnt);//48
	_cnt+=4;
	Float2Byte(&pz,send_nclink_buf,_cnt);//52
	_cnt+=4;
	Float2Byte(&vx,send_nclink_buf,_cnt);//56
	_cnt+=4;
	Float2Byte(&vy,send_nclink_buf,_cnt);//60
	_cnt+=4;
	Float2Byte(&vz,send_nclink_buf,_cnt);//64
	_cnt+=4;
	
  send_nclink_buf[3] = _cnt-4;
  for(i=0;i<_cnt;i++) sum ^= send_nclink_buf[i]; 
  send_nclink_buf[_cnt++]=sum;
	
	send_nclink_buf[_cnt++]=NCLink_End[0];
	send_nclink_buf[_cnt++]=NCLink_End[1];
  Reserved_Serialport_Send(send_nclink_buf,_cnt);
}

void NCLink_Send_Quaternion_Feedback(float q1	,float q2,float q3  ,float q0,
																		 float gx,float gy,float gz,
																		 float ax,float ay,float az)
{
  uint8_t sum=0,_cnt=0,i=0;
  send_nclink_buf[_cnt++]=NCLink_Head[0];
  send_nclink_buf[_cnt++]=NCLink_Head[1];
  send_nclink_buf[_cnt++]=0xE1;
  send_nclink_buf[_cnt++]=0;
		
	Float2Byte(&q1,send_nclink_buf,_cnt);//4
	_cnt+=4;
	Float2Byte(&q2,send_nclink_buf,_cnt);//8
	_cnt+=4;
	Float2Byte(&q3,send_nclink_buf,_cnt);//12
	_cnt+=4;
	Float2Byte(&q0,send_nclink_buf,_cnt);//16
	_cnt+=4;
	Float2Byte(&gx,send_nclink_buf,_cnt);//20
	_cnt+=4;
	Float2Byte(&gy,send_nclink_buf,_cnt);//24
	_cnt+=4;
	Float2Byte(&gz,send_nclink_buf,_cnt);//28
	_cnt+=4;
	Float2Byte(&ax,send_nclink_buf,_cnt);//32
	_cnt+=4;
	Float2Byte(&ay,send_nclink_buf,_cnt);//36
	_cnt+=4;
	Float2Byte(&az,send_nclink_buf,_cnt);//40
	_cnt+=4;
	
  send_nclink_buf[3] = _cnt-4;
  for(i=0;i<_cnt;i++) sum ^= send_nclink_buf[i]; 
  send_nclink_buf[_cnt++]=sum;
	
	send_nclink_buf[_cnt++]=NCLink_End[0];
	send_nclink_buf[_cnt++]=NCLink_End[1];
  Reserved_Serialport_Send(send_nclink_buf,_cnt);
}




static void NCLink_Send_Feedback(uint8_t response)//����վӦ��У��
{
  uint8_t sum = 0,i=0;
  send_nclink_buf[0]=NCLink_Head[0];
  send_nclink_buf[1]=NCLink_Head[1];
  send_nclink_buf[2]=NCLINK_SEND_CHECK;
  send_nclink_buf[3]=1;
  send_nclink_buf[4]=response;
  for(i=0;i<5;i++) sum ^= send_nclink_buf[i];
  send_nclink_buf[5]=sum;
	send_nclink_buf[6]=NCLink_End[0];
	send_nclink_buf[7]=NCLink_End[1];
  Reserved_Serialport_Send(send_nclink_buf,8);
}

void NCLink_Send_IMU_Feedback_PC(void)//1.54ms
{
	static uint16_t _cnt=0;
	_cnt++;
	if(_cnt>=10)//50ms��������20hz
	{
		_cnt=0;
		//�¶��ȶ�����������ƫУ׼��ϲŷ��ͷɻ�IMU���ݸ����ؼ����
		//���⼤���״�ת��Ӱ�������Ǳ궨����
		if(Gyro_Safety_Calibration_Flag==1
			&&(Optical_Type_Present==3||Optical_Type_Present==4))//���õĶ�λ����ΪRPLIDAR/T265/3DLIDAR_UNI
		{			
			NCLink_Send_IMU_Feedback(WP_AHRS.q[1],WP_AHRS.q[2],WP_AHRS.q[3],WP_AHRS.q[0],
															 WP_AHRS.Pitch_Gyro_Rad,WP_AHRS.Roll_Gyro_Rad,WP_AHRS.Yaw_Gyro_Rad,
															 WP_AHRS.Accel_X_MPSS,WP_AHRS.Accel_Y_MPSS,WP_AHRS.Accel_Z_MPSS,
															 VIO_SINS.Position[_EAST]/100.0f,VIO_SINS.Position[_NORTH]/100.0f,NamelessQuad.Position[_UP]/100.0f,
															 VIO_SINS.Speed[_EAST]/100.0f   ,VIO_SINS.Speed[_NORTH]/100.0f   ,NamelessQuad.Speed[_UP]/100.0f);
		}
	}
	
	switch(send_check_back)
	{
		case 1:
		{
			NCLink_Send_Feedback(NCLINK_SEND_DIS);
			send_check_back=0;
		}
		break;
		case 2:
		{
			NCLink_Send_Feedback(NCLINK_SEND_NAV_CTRL);
			send_check_back=0;
		}
		break;	
		case 3:
		{
			NCLink_Send_Feedback(NCLINK_SEND_NAV_CTRL_FINISH);
			send_check_back=0;		
		}
		break;
		case 4:
		{
			NCLink_Send_Feedback(NCLINK_SEND_SLAM_SYSTEM_RESET);//��λslam
			send_check_back=0;
			//��������ʾ��λ�ɹ�
      buzzer_setup(1000,0.5f,2);			
		}
		break;
		case 5:
		{
			NCLink_Send_Feedback(NCLINK_SEND_SLAM_STOP_MOTOR);
			send_check_back=0;		
		}
		break;
		case 6:
		{
			NCLink_Send_Feedback(NCLINK_SEND_SLAM_START_MOTOR);
			send_check_back=0;		
		}
		break;
	}
}

uint32_t nclink_receive_fault_cnt=0;
void NCLink_Data_Prase_Prepare_Lite(uint8_t data)//����վ���ݽ���
{
  static uint8_t data_len = 0,data_cnt = 0;
  static uint8_t state = 0;
  if(state==0&&data==NCLink_Head[1])//�ж�֡ͷ1
  {
    state=1;
    nclink_buf[0]=data;
  }
  else if(state==1&&data==NCLink_Head[0])//�ж�֡ͷ2
  {
    state=2;
    nclink_buf[1]=data;
  }
  else if(state==2&&data<0XF1)//�����ֽ�
  {
    state=3;
    nclink_buf[2]=data;
  }
  else if(state==3&&data<100)//��Ч���ݳ���
  {
    state = 4;
    nclink_buf[3]=data;
    data_len = data;
    data_cnt = 0;
  }
  else if(state==4&&data_len>0)//���ݽ���
  {
    data_len--;
    nclink_buf[4+data_cnt++]=data;
    if(data_len==0)  state = 5;
  }
  else if(state==5)//���У��
  {
    state = 6;
    nclink_buf[4+data_cnt++]=data;
  }
	else if(state==6&&data==NCLink_End[0])//֡β0
	{
			state = 7;
			nclink_buf[4+data_cnt++]=data;
	}
	else if(state==7&&data==NCLink_End[1])//֡β1
	{
			state = 0;
			nclink_buf[4+data_cnt]=data;
		  NCLink_Data_Prase_Process_Lite(nclink_buf,data_cnt+5);//���ݽ���
	}
  else 
	{
		state = 0;
		nclink_receive_fault_cnt++;
	}
}




systime nclink_t;
void NCLink_Data_Prase_Process_Lite(uint8_t *data_buf,uint8_t num)//�ɿ����ݽ�������
{
  uint8_t sum = 0;
  for(uint8_t i=0;i<(num-3);i++)  sum ^= *(data_buf+i);
  if(!(sum==*(data_buf+num-3)))    																					return;//�ж�sum	
	if(!(*(data_buf)==NCLink_Head[1]&&*(data_buf+1)==NCLink_Head[0]))         return;//�ж�֡ͷ
	if(!(*(data_buf+num-2)==NCLink_End[0]&&*(data_buf+num-1)==NCLink_End[1])) return;//֡βУ��  

	if(*(data_buf+2)==0X0A)                             //����վ�����ƶ�����
  {		
    ngs_sdk.move_mode=*(data_buf+4),
		ngs_sdk.mode_order=*(data_buf+5);
    ngs_sdk.move_distance=(uint16_t)(*(data_buf+6)<<8)|*(data_buf+7);;
    ngs_sdk.update_flag=true;
		
		ngs_sdk.move_flag.sdk_front_flag=false;
		ngs_sdk.move_flag.sdk_behind_flag=false;
		ngs_sdk.move_flag.sdk_left_flag=false;
		ngs_sdk.move_flag.sdk_right_flag=false;
		ngs_sdk.move_flag.sdk_up_flag=false;
		ngs_sdk.move_flag.sdk_down_flag=false;
		
		if(*(data_buf+4)==SDK_FRONT)
		{					
			ngs_sdk.move_flag.sdk_front_flag=true;
			ngs_sdk.f_distance=ngs_sdk.move_distance;
		}
		else if(*(data_buf+4)==SDK_BEHIND) 
		{					
			ngs_sdk.move_flag.sdk_behind_flag=true;
			ngs_sdk.f_distance=-ngs_sdk.move_distance;
		}
		else if(*(data_buf+4)==SDK_LEFT)  
		{			
			ngs_sdk.move_flag.sdk_left_flag=true;
			ngs_sdk.f_distance=ngs_sdk.move_distance;
		}
		else if(*(data_buf+4)==SDK_RIGHT)
		{					
			ngs_sdk.move_flag.sdk_right_flag=true;
			ngs_sdk.f_distance=-ngs_sdk.move_distance;
		}
		else if(*(data_buf+4)==SDK_UP)
		{  				
			ngs_sdk.move_flag.sdk_up_flag=true;
			ngs_sdk.f_distance=ngs_sdk.move_distance;
		}
		else if(*(data_buf+4)==SDK_DOWN) 
		{					
			ngs_sdk.move_flag.sdk_down_flag=true;
			ngs_sdk.f_distance=-ngs_sdk.move_distance;
		}				
		NCLink_Send_Check_Flag[8]=1;
		Pilot_Status_Tick();	
		send_check_back=1;
  }
	else if(*(data_buf+2)==0X0C)
  {
		Guide_Flight_Lng =((int32_t)(*(data_buf+4)<<24)|(*(data_buf+5)<<16)|(*(data_buf+6)<<8)|*(data_buf+7));
		Guide_Flight_Lat =((int32_t)(*(data_buf+8)<<24)|(*(data_buf+9)<<16)|(*(data_buf+10)<<8)|*(data_buf+11));
		Guide_Flight_Cnt++;;
		Guide_Flight_Flag=1;
		Pilot_Status_Tick();
	}
	else if(*(data_buf+2)==0X0D)
	{
		Byte2Float(data_buf,4,&current_state.position_x);
		Byte2Float(data_buf,8,&current_state.position_y);
		Byte2Float(data_buf,12,&current_state.position_z);
		Byte2Float(data_buf,16,&current_state.velocity_x);
		Byte2Float(data_buf,20,&current_state.velocity_y);
		Byte2Float(data_buf,24,&current_state.velocity_z);
		current_state.position_x*=100.0f;
		current_state.position_y*=100.0f;
		current_state.position_z*=100.0f;
		current_state.velocity_x*=100.0f;
		current_state.velocity_y*=100.0f;
		current_state.velocity_z*=100.0f;
		
		
		Byte2Float(data_buf,28,&current_state.q[0]);		
		Byte2Float(data_buf,32,&current_state.q[1]);
		Byte2Float(data_buf,36,&current_state.q[2]);
		Byte2Float(data_buf,40,&current_state.q[3]);
		float _q[4];
		_q[0]=current_state.q[3]*(1.0f);
		_q[1]=current_state.q[0]*(1.0f);
		_q[2]=current_state.q[2]*(-1.0f);
		_q[3]=current_state.q[1]*(1.0f);	
		
		current_state.q[0]=_q[0];
		current_state.q[1]=_q[1];
		current_state.q[2]=_q[2];
		current_state.q[3]=_q[3];

		Byte2Float(data_buf,44,&current_state.quality);
		current_state.update_flag=*(data_buf+48);
		current_state.byte[0]=*(data_buf+49);
		current_state.byte[1]=*(data_buf+50);
		current_state.byte[2]=*(data_buf+51);
		current_state.byte[3]=*(data_buf+52);
		current_state.byte[4]=*(data_buf+53);
		current_state.byte[5]=*(data_buf+54);
		current_state.byte[6]=*(data_buf+55);
		current_state.byte[7]=*(data_buf+56);

		current_state.rec_update_flag=1;
		current_state.rec_head_update_flag=1;		
		Pilot_Status_Tick();
		Get_Systime(&nclink_t);
		current_state.slam_sensor=T265_SLAM;
	}
	else if(*(data_buf+2)==0X0E)
	{
		Byte2Float(data_buf,4,&current_state.position_x);
		Byte2Float(data_buf,8,&current_state.position_y);
		Byte2Float(data_buf,12,&current_state.position_z);
		//��Ϊ�����״�ʱ,�����������ݷֱ��ʾ��С���롢��С�������ڽǶȣ��ź�ǿ��
		Byte2Float(data_buf,16,&current_state.velocity_x);//m
		Byte2Float(data_buf,20,&current_state.velocity_y);//deg
		Byte2Float(data_buf,24,&current_state.velocity_z);
		
		current_state.position_x*=100.0f;
		current_state.position_y*=100.0f;
		current_state.position_z*=100.0f;
		
		
		if(current_state.velocity_y<0) current_state.velocity_y+=360;//356.384094
		
		for(uint16_t i=Laser_Min_Info_Num-1;i>0;i--)
		{
			min_dis_cm_backups[i]=min_dis_cm_backups[i-1];
			min_dis_angle_backups[i]=min_dis_angle_backups[i-1];
		}
			min_dis_cm_backups[0]		=current_state.velocity_x*100.0f;//ת����cm
			min_dis_angle_backups[0]=current_state.velocity_y;		
		
			
		Byte2Float(data_buf,28,&current_state.q[0]);		
		Byte2Float(data_buf,32,&current_state.q[1]);
		Byte2Float(data_buf,36,&current_state.q[2]);
		Byte2Float(data_buf,40,&current_state.q[3]);
		float _q[4];
		_q[0]=current_state.q[3]*(1.0f);
		_q[1]=current_state.q[0]*(1.0f);
		_q[2]=current_state.q[1]*(1.0f);
		_q[3]=current_state.q[2]*(1.0f);	
		
		current_state.q[0]=_q[0];
		current_state.q[1]=_q[1];
		current_state.q[2]=_q[2];
		current_state.q[3]=_q[3];
		
		Byte2Float(data_buf,44,&current_state.quality);
		current_state.update_flag=*(data_buf+48);
		current_state.byte[0]=*(data_buf+49);
		current_state.byte[1]=*(data_buf+50);
		current_state.byte[2]=*(data_buf+51);
		current_state.byte[3]=*(data_buf+52);
		current_state.byte[4]=*(data_buf+53);
		current_state.byte[5]=*(data_buf+54);
		current_state.byte[6]=*(data_buf+55);
		current_state.byte[7]=*(data_buf+56);

		current_state.rec_update_flag=1;
		current_state.rec_head_update_flag=1;		
		
		Pilot_Status_Tick();
	  Get_Systime(&nclink_t);
		current_state.slam_sensor=LIDAR_2D_SLAM;
	}	
	else if(*(data_buf+2)==0X10)//fastlio
	{ 
		Byte2Float(data_buf,4,&current_state.position_x);
		Byte2Float(data_buf,8,&current_state.position_y);
		Byte2Float(data_buf,12,&current_state.position_z);

		current_state.position_x*=100.0f;
		current_state.position_y*=100.0f;
		current_state.position_z*=100.0f;
					
		Byte2Float(data_buf,28,&current_state.q[0]);		
		Byte2Float(data_buf,32,&current_state.q[1]);
		Byte2Float(data_buf,36,&current_state.q[2]);
		Byte2Float(data_buf,40,&current_state.q[3]);
		float _q[4];
		_q[0]=current_state.q[3]*(1.0f);
		_q[1]=current_state.q[0]*(1.0f);
		_q[2]=current_state.q[1]*(1.0f);
		_q[3]=current_state.q[2]*(1.0f);	
		
		current_state.q[0]=_q[0];
		current_state.q[1]=_q[1];
		current_state.q[2]=_q[2];
		current_state.q[3]=_q[3];
		
		Byte2Float(data_buf,44,&current_state.quality);
		current_state.update_flag=*(data_buf+48);
		current_state.byte[0]=*(data_buf+49);
		current_state.byte[1]=*(data_buf+50);
		current_state.byte[2]=*(data_buf+51);
		current_state.byte[3]=*(data_buf+52);
		current_state.byte[4]=*(data_buf+53);
		current_state.byte[5]=*(data_buf+54);
		current_state.byte[6]=*(data_buf+55);
		current_state.byte[7]=*(data_buf+56);
		
		Pilot_Status_Tick();
	  Get_Systime(&nclink_t);		
		//�ж�loam�����Ѿ���������
		if(current_state.position_x==0&&current_state.position_y==0)
		{
			current_state.loam_update_flag=0;
			return ;//���ݷ���������ʱ,��ǰ�˳�
		}
		else current_state.loam_update_flag=1;
		
		current_state.rec_update_flag=1;
		current_state.rec_head_update_flag=1;		
		current_state.slam_sensor=LOAM;
	}	
	else if(*(data_buf+2)==0X0F)//��������ָ��
	{
		ngs_nav_ctrl.number=(uint16_t)(*(data_buf+4)<<8) |*(data_buf+5);
		Byte2Float(data_buf,6, &ngs_nav_ctrl.x);
		Byte2Float(data_buf,10,&ngs_nav_ctrl.y);
		Byte2Float(data_buf,14,&ngs_nav_ctrl.z);
		ngs_nav_ctrl.nav_mode=*(data_buf+18);
		ngs_nav_ctrl.frame_id=*(data_buf+19);
		ngs_nav_ctrl.cmd_vel_during_cnt=(uint32_t)(*(data_buf+20)<<24|*(data_buf+21)<<16|*(data_buf+22)<<8|*(data_buf+23));	//�ٶȿ���ʱ������ʱ��,��λMS
		if(ngs_nav_ctrl.nav_mode!=CMD_VEL_MODE)	ngs_nav_ctrl.update_flag=1;
		else 
		{
			if(ngs_nav_ctrl.frame_id==0)//������Դ��ROS
			{
				ngs_nav_ctrl.cmd_vel_x=-100*ngs_nav_ctrl.x;					 //ת����cm/s
				ngs_nav_ctrl.cmd_vel_y= 100*ngs_nav_ctrl.y;          //ת����cm/s
				ngs_nav_ctrl.cmd_vel_angular_z=57.3f*ngs_nav_ctrl.z; //ת����deg/s
			}	
			else//������Դ���������µ���վ
			{
				ngs_nav_ctrl.cmd_vel_x=-ngs_nav_ctrl.x;			   //cm/s
				ngs_nav_ctrl.cmd_vel_y= ngs_nav_ctrl.y;        //cm/s
				ngs_nav_ctrl.cmd_vel_angular_z=ngs_nav_ctrl.z; //deg/s			
			}
			//�޷�����������ִ������������ֵ�����·ɻ�����������ɵ�ʧ��
			ngs_nav_ctrl.cmd_vel_x=constrain_float(ngs_nav_ctrl.cmd_vel_x,-ngs_nav_ctrl.cmd_vel_max,ngs_nav_ctrl.cmd_vel_max);
			ngs_nav_ctrl.cmd_vel_y=constrain_float(ngs_nav_ctrl.cmd_vel_y,-ngs_nav_ctrl.cmd_vel_max,ngs_nav_ctrl.cmd_vel_max);
			ngs_nav_ctrl.cmd_vel_angular_z=constrain_float(ngs_nav_ctrl.cmd_vel_angular_z,-ngs_nav_ctrl.cmd_angular_max,ngs_nav_ctrl.cmd_angular_max);
			ngs_nav_ctrl.cmd_vel_during_cnt/=5;//����ʱ��200*5=1000ms		
			ngs_nav_ctrl.cmd_vel_update=1;
		}	
		Pilot_Status_Tick();	
		send_check_back=2;
	}	
}





us100_data us100_front;
static uint16_t us100_cnt=0;
void us100_front_statemachine(void)
{
	if(Reserved_Uart!=FRONT_RANGE_FINDER) return;
	us100_cnt++;
	if(us100_cnt>=20)
	{
		us100_cnt=0;
		us100_front.pre_last_distance=us100_front.last_distance;//���ϴθ߶�
		us100_front.last_distance=us100_front.distance;//�ϴθ߶�
		us100_front.distance=US_100_Distance(COM6_Rx_Buf.Ring_Buff[0],COM6_Rx_Buf.Ring_Buff[1]);

		us100_front.last_vel=us100_front.vel;
		us100_front.vel=(us100_front.distance-us100_front.last_distance)/0.1f;
		us100_front.acc=(us100_front.vel-us100_front.last_vel)/0.1f;		
		
		COM6_Rx_Buf.Head=1;
		COM6_Rx_Buf.Tail=0; 
		UARTCharPut(UART6_BASE,US_100_Distance_CMD);
	}
}




////////////////////////
void Uart2_Serialport_Init(void)
{
	switch(Uart2_Mode)
	{
		case 0x00:
		{
			ConfigureUART2(19200);//LC30X����
			delay_ms(1000);				 //��ʼ����ʱ
		}
		break;
		case 0x01:
		{
			ConfigureUART2(230400);//�����״�
			delay_ms(1000);				 //��ʼ����ʱ
			//lsn10_motor_ctrl(0x00);
			//send_check_back=5;
		}
		break;
		default:{ConfigureUART2(19200);delay_ms(1000);}
	}
}









//			NCLink_Send_IMU_Feedback(-WP_AHRS.q[2],WP_AHRS.q[1],WP_AHRS.q[3],WP_AHRS.q[0],
//															 -WP_AHRS.Roll_Gyro_Rad,WP_AHRS.Pitch_Gyro_Rad,WP_AHRS.Yaw_Gyro_Rad,
//															 -WP_AHRS.Accel_Y_MPSS,WP_AHRS.Accel_X_MPSS,WP_AHRS.Accel_Z_MPSS,
//															 -VIO_SINS.Position[_NORTH]/100.0f,VIO_SINS.Position[_EAST]/100.0f,NamelessQuad.Position[_UP]/100.0f,
//															 -VIO_SINS.Speed[_NORTH]/100.0f   ,VIO_SINS.Speed[_EAST]/100.0f   ,NamelessQuad.Speed[_UP]/100.0f);


//		{
//			NCLink_Send_IMU_Feedback(-WP_AHRS.q[2],WP_AHRS.q[1],WP_AHRS.q[3],WP_AHRS.q[0],
//															 -WP_AHRS.Roll_Gyro_Rad,WP_AHRS.Pitch_Gyro_Rad,WP_AHRS.Yaw_Gyro_Rad,
//															 -WP_AHRS.Accel_Y_MPSS,WP_AHRS.Accel_X_MPSS,WP_AHRS.Accel_Z_MPSS,
//															 -VIO_SINS.Position[_NORTH]/100.0f,VIO_SINS.Position[_EAST]/100.0f,NamelessQuad.Position[_UP]/100.0f,
//															 -VIO_SINS.Speed[_NORTH]/100.0f   ,VIO_SINS.Speed[_EAST]/100.0f   ,NamelessQuad.Speed[_UP]/100.0f);
//		}

//		quad_getangle(current_state.q,current_state.rpy);
//		current_state.rpy[2]=(int16_t)(current_state.rpy[2]);
//		if(current_state.rpy[2]<0.05f)   current_state.rpy[2]+=360.0f;
//		if(current_state.rpy[2]>359.95f) current_state.rpy[2]-=360.0f;


//	rpy[1]= RADTODEG(atan2f(q[0]*q[1] + q[2]*q[3], 0.5f - q[1]*q[1] - q[2]*q[2]));
//	rpy[0]= RADTODEG(asinf(-2.0f * (q[1]*q[3] - q[0]*q[2])));
//	rpy[2]= RADTODEG(atan2f(q[1]*q[2] + q[0]*q[3], 0.5f - q[2]*q[2] - q[3]*q[3]));


//			NCLink_Send_Quaternion_Feedback(WP_AHRS.q[1],WP_AHRS.q[2],WP_AHRS.q[3],WP_AHRS.q[0],
//																		  WP_AHRS.Pitch_Gyro_Rad,WP_AHRS.Roll_Gyro_Rad,WP_AHRS.Yaw_Gyro_Rad,
//															        WP_AHRS.Accel_X_MPSS,WP_AHRS.Accel_Y_MPSS,WP_AHRS.Accel_Z_MPSS);


