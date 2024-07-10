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
#include "Earth_Declination.h"
#include "SINS.h"




/***********************************************************
@��������Vector_From_BodyFrame2EarthFrame
@��ڲ�����Vector3f *bf,Vector3f *ef
@���ڲ�������
��������������ϵ�򵼺�ϵת��
@���ߣ�����С��
@���ڣ�2024/01/20
*************************************************************/
void Vector_From_BodyFrame2EarthFrame(Vector3f *bf,Vector3f *ef,float *rmat)
{
  ef->x=rmat[0]*bf->x+rmat[1]*bf->y+rmat[2]*bf->z;
  ef->y=rmat[3]*bf->x+rmat[4]*bf->y+rmat[5]*bf->z;
  ef->z=rmat[6]*bf->x+rmat[7]*bf->y+rmat[8]*bf->z;
}

/***********************************************************
@��������Vector_From_EarthFrame2BodyFrame
@��ڲ�����Vector3f *ef,Vector3f *bf
@���ڲ�������
��������������ϵ������ϵת��
@���ߣ�����С��
@���ڣ�2024/01/20
*************************************************************/
void Vector_From_EarthFrame2BodyFrame(Vector3f *ef,Vector3f *bf,float *rmat)
{
  bf->x=rmat[0]*ef->x+rmat[1]*ef->y+rmat[2]*ef->z;
  bf->y=rmat[3]*ef->x+rmat[4]*ef->y+rmat[5]*ef->z;
  bf->z=rmat[6]*ef->x+rmat[7]*ef->y+rmat[8]*ef->z;
}


//��������ϵ������ת������Pitch��Roll������
void from_enu_to_body_frame(float e,float n,float *right,float *forward,float _yaw)
{
	float _cos=FastCos(_yaw*DEG2RAD);
	float _sin=FastSin(_yaw*DEG2RAD);
	*right  = e*_cos+n*_sin;
	*forward=-e*_sin+n*_cos;
}

void from_body_to_enu_frame(float right,float forward,float *e,float *n,float _yaw)
{
	float _cos=FastCos(_yaw*DEG2RAD);
	float _sin=FastSin(_yaw*DEG2RAD);
	*e= right*_cos-forward*_sin;
	*n= right*_sin+forward*_cos;
}


SINS NamelessQuad;
float Acceleration_Feedback[3];
Vector2f SINS_Accel_Body={0,0};
/***********************************************************
@��������SINS_Prepare
@��ڲ�������
@���ڲ�������
�����������ߵ����ٶ�׼��
@���ߣ�����С��
@���ڣ�2024/01/20
*************************************************************/
void  SINS_Prepare(void)
{
  Vector2f SINS_Accel_Earth={0,0};
  Vector3f Body_Frame,Earth_Frame;
  /*Z-Y-Xŷ����ת�������Ҿ���
  //Pitch Roll  Yaw �ֱ��Ӧ�� �� ��  
  X����ת����
  R������
  {1      0        0    }
  {0      cos��    sin��  }
  {0     -sin��    cos��  }
  
  Y����ת����
  R���ȣ�
  {cos��     0        -sin��}
  {0         1        0     }
  {sin��     0        cos��}
  
  Z����ת����
  R���ȣ�
  {cos��      sin��       0}
  {-sin��     cos��       0}
  {0          0           1 }
  
  ��Z-Y-X˳����:
  ��������ϵ����������ϵ����ת����R(b2n)
  R(b2n) =R(��)^T*R(��)^T*R(��)^T
  
  R(b2n)=
  {cos��*cos��     -cos��*sin��+sin��*sin��*cos��        sin��*sin��+cos��*sin��*cos��}
  {cos��*sin��     cos��*cos�� +sin��*sin��*sin��       -cos��*sin��+cos��*sin��*sin��}
  {-sin��         cos��sin ��                          cos��cos��              }
  */
  Body_Frame.x=ins_accel_filter.x;
  Body_Frame.y=ins_accel_filter.y;
  Body_Frame.z=ins_accel_filter.z;
  Vector_From_BodyFrame2EarthFrame(&Body_Frame,&Earth_Frame,WP_AHRS.rMat);
  NamelessQuad.Inertial_Acceleration[_UP]=Earth_Frame.z;
  NamelessQuad.Inertial_Acceleration[_EAST]=Earth_Frame.x;
  NamelessQuad.Inertial_Acceleration[_NORTH]=Earth_Frame.y;
  
  
  NamelessQuad.Inertial_Acceleration[_UP]*=ACCEL_SCALE_8G;
  NamelessQuad.Inertial_Acceleration[_UP]-=GRAVITY_MSS;//��ȥ�������ٶ�
  NamelessQuad.Inertial_Acceleration[_UP]*=100;//���ٶ�cm/s^2
  
  NamelessQuad.Inertial_Acceleration[_EAST]*=ACCEL_SCALE_8G;
  NamelessQuad.Inertial_Acceleration[_EAST]*=100;//���ٶ�cm/s^2
  
  NamelessQuad.Inertial_Acceleration[_NORTH]*=ACCEL_SCALE_8G;
  NamelessQuad.Inertial_Acceleration[_NORTH]*=100;//���ٶ�cm/s^2
  
  /******************************************************************************/
  //�����˻��ڵ�������ϵ�µ���������������������˶����ٶ���ת����ǰ������˶����ٶ�:��ͷ(����)+���
  
  SINS_Accel_Earth.x=NamelessQuad.Inertial_Acceleration[_EAST];//�ص�������ϵ�����������˶����ٶ�,��λΪCM
  SINS_Accel_Earth.y=NamelessQuad.Inertial_Acceleration[_NORTH];//�ص�������ϵ�����������˶����ٶ�,��λΪCM
  
  SINS_Accel_Body.x= SINS_Accel_Earth.x*WP_AHRS.cos_rpy[_YAW]+SINS_Accel_Earth.y*WP_AHRS.sin_rpy[_YAW];  //��������˶����ٶ�  X������
  SINS_Accel_Body.y=-SINS_Accel_Earth.x*WP_AHRS.sin_rpy[_YAW]+SINS_Accel_Earth.y*WP_AHRS.cos_rpy[_YAW];  //��ͷ�����˶����ٶ�  Y������

  Body_Frame.x=WP_Sensor.acce_filter_fb[0];
  Body_Frame.y=WP_Sensor.acce_filter_fb[1];
  Body_Frame.z=WP_Sensor.acce_filter_fb[2];
		
  Vector_From_BodyFrame2EarthFrame(&Body_Frame,&Earth_Frame,WP_AHRS.rMat);
  Acceleration_Feedback[_UP]=Earth_Frame.z;
  Acceleration_Feedback[_EAST]=Earth_Frame.x;
  Acceleration_Feedback[_NORTH]=Earth_Frame.y;
  Acceleration_Feedback[_UP]*=ACCEL_SCALE_8G;
  Acceleration_Feedback[_UP]-=GRAVITY_MSS;//��ȥ�������ٶ�
  Acceleration_Feedback[_UP]*=100;//���ٶ�cm/s^2
  Acceleration_Feedback[_EAST]*=ACCEL_SCALE_8G;
  Acceleration_Feedback[_EAST]*=100;//���ٶ�cm/s^2
  Acceleration_Feedback[_NORTH]*=ACCEL_SCALE_8G;
  Acceleration_Feedback[_NORTH]*=100;//���ٶ�cm/s^2
}



/*****************�㷨�������ͽ���***************************************************
1����������ƪ֮�ߵ����ٶ�+�ٶ�+λ�����׻����ںϷ���:
http://blog.csdn.net/u011992534/article/details/61924200
2��������ߵ��ں�֮�۲⴫�����ͺ���������������˹��ͨ�˲������
����ѹ��MS5611��GPSģ��M8N����������PX4FLOW�ȣ�:
http://blog.csdn.net/u011992534/article/details/73743955
3����APMԴ�����GPS����ѹ�ƹߵ��ں�
http://blog.csdn.net/u011992534/article/details/78257684
**********************************************************************************/
uint16_t SPL06_Sync_Cnt=6;
uint16_t HCSR04_Sync_Cnt=5;
uint16_t VL53L1_Sync_Cnt=0;
uint16_t TOF_Sync_Cnt=10;


#define Ra  1.0f//1.0
#define KALMAN_DT (Baro_Update_Dt*0.001f)
float Q[2]={0.5f*Ra*KALMAN_DT*KALMAN_DT,Ra*KALMAN_DT};
float R_Baro=35;//75  150  100
float R_Ground=5;
float R_VL53L1X=0.05;
float R_TOFSENSOR=3;//5
float R=5;
float Acce_Bias_Gain_Kp={
  0.0f,//0.3   0.1~0.75
};
float Acce_Bias_Gain_Ki={
  0.01,//0.005
};
float Pre_conv[4]=
{
  1.10,0.83,//4.95 2.83								//0.18,0.1,
  0.83,1.33	//2.83 3.49								//0.1,0.18
};//��һ��Э����
float kalman_k[2]={0};//�������
/***********************************************************
@��������KalmanFilter
@��ڲ�����float Observation,//λ�ù۲���
					 uint16 Pos_Delay_Cnt,//�۲⴫������ʱ��
					 SINS *Ins_Kf,//�ߵ��ṹ��
					 float System_drive,//ϵͳԭʼ���������ߵ����ٶ�
					 float *Q,
					 float R,
					 float dt,
					 uint16 N,
					 uint8_t update_flag
@���ڲ�������
�����������߶ȷ���ߵ��������˲�
@���ߣ�����С��
@���ڣ�2024/01/20
*************************************************************/
float observation_div=0;
float last_observation=0;
float observation_acc;
uint8_t observation_reset_flag=0;
float observation_div_err[10]={0};
float observation_div_rmse[10]={0};

float observation_err[10]={0};
float observation_rmse[10]={0};

float observation_acc_err[10]={0};
float observation_acc_rmse[10]={0};
void  KalmanFilter(float Observation,//λ�ù۲���
                   uint16 Pos_Delay_Cnt,//�۲⴫������ʱ��
                   SINS *Ins_Kf,//�ߵ��ṹ��
                   float System_drive,//ϵͳԭʼ���������ߵ����ٶ�
                   float *Q,
                   float R,
                   float dt,
                   uint16 N,
                   uint8_t *update_flag)
{
  static uint16 obs_sync_cnt=0;
	static int16_t Unusual_Protect_Flag=0;
	static int16_t Unusual_Protect_Cnt=0;
	static uint8_t update_bias=0;
	float R_Temp=0;
  float Temp_conv[4]={0};//����Э����
  float Conv_Z=0,Z_Cor=0;
  float Ctemp=0;
  float rmse_temp1=0,rmse_temp2,rmse_temp3;
	//if(Unusual_Protect_Flag>0)  Unusual_Protect_Flag--;
	
	R_Temp=constrain_float(R,0,10000);
  //����״̬
  Ins_Kf->Acceleration[N]=System_drive;
  Ins_Kf->Acceleration[N]=Ins_Kf->Acce_Bias_All[N]+Ins_Kf->Acceleration[N];
  Ins_Kf->Position[N] +=Ins_Kf->Speed[N]*dt+(Ins_Kf->Acceleration[N]*dt*dt)/2.0f;
  Ins_Kf->Speed[N]+=Ins_Kf->Acceleration[N]*dt;
  if(*update_flag==1)
  {
		for(uint16_t i=9;i>0;i--)//�����׼��
		{
		  observation_div_err[i]=observation_div_err[i-1];
			observation_div_rmse[i]=observation_div_rmse[i-1];
		  observation_err[i]=observation_err[i-1];
			observation_rmse[i]=observation_rmse[i-1];
			observation_acc_err[i]=observation_acc_err[i-1];
			observation_acc_rmse[i]=observation_acc_rmse[i-1];
		}
		observation_div_err[0]=Butterworth_Buffer_Baro.Output_Butter[2]-Ins_Kf->Vel_Backups[N][8*Pos_Delay_Cnt];
	  observation_err[0]=Observation-Ins_Kf->Pos_Backups[N][2*Pos_Delay_Cnt];
		observation_acc_err[0]=Butterworth_Buffer_Baro_Acc.Output_Butter[2]-Ins_Kf->Acce_Backups[N][8*Pos_Delay_Cnt];
	
		for(int16_t i=9;i>=0;i--)//������������
		{
		  rmse_temp1+=observation_div_err[i]*observation_div_err[i];
			rmse_temp2+=observation_err[i]*observation_err[i];
			rmse_temp3+=observation_acc_err[i]*observation_acc_err[i];
		}
		observation_div_rmse[0]=safe_sqrt(rmse_temp1/10.0f);
		observation_rmse[0]=safe_sqrt(rmse_temp2/10.0f);
		observation_acc_rmse[0]=safe_sqrt(rmse_temp3/10.0f);
		
		if(   observation_acc>20000	 //40g��1g=1000cm/s^2 50000cm/s^2Լ����30g���ٶȣ���������װ�ﲻ��
				||observation_acc<-15000//-30gʧ���������1g��������
				//||(ABS(observation_div)>20*ABS(Ins_Kf->Vel_Backups[N][2*Pos_Delay_Cnt])	
		  )
		{
			Observation=Ins_Kf->Pos_Backups[N][Pos_Delay_Cnt];
			update_bias=0;
			Unusual_Protect_Flag=1;//N ms�۲��쳣�������ڣ�������bias
			Unusual_Protect_Cnt=0;
		}
		else  
		{
			last_observation=Observation;
			update_bias=1;
		}
		
		//last_observation=Observation;
		//update_bias=1;
		
    //����Э����
    Ctemp=Pre_conv[1]+Pre_conv[3]*KALMAN_DT;
    Temp_conv[0]=Pre_conv[0]+Pre_conv[2]*KALMAN_DT+Ctemp*KALMAN_DT+Q[0];
    Temp_conv[1]=Ctemp;
    Temp_conv[2]=Pre_conv[2]+Pre_conv[3]*KALMAN_DT;;
    Temp_conv[3]=Pre_conv[3]+Q[1];
    //���㿨��������
    Conv_Z=Temp_conv[0]+R_Temp;
    kalman_k[0]=Temp_conv[0]/Conv_Z;
    kalman_k[1]=Temp_conv[2]/Conv_Z;
    //�ں��������
		Z_Cor=constrain_float(Observation-Ins_Kf->Pos_Backups[N][Pos_Delay_Cnt],-Observation_Err_Max,Observation_Err_Max);	
    /*
		if(Observation_Update_Type==1
			&&(observation_rmse[0]>=50
			||observation_div_rmse[0]>100
		  ||observation_acc_rmse[0]>2000)) 
		Z_Cor/=5; 
		*/
    Ins_Kf->Position[N] +=kalman_k[0]*Z_Cor;
    Ins_Kf->Speed[N] +=kalman_k[1]*Z_Cor;
		
		if(update_bias==1||Unusual_Protect_Flag==0)
		{
			Ins_Kf->Acce_Bias[N]+=Acce_Bias_Gain_Ki*Z_Cor;
		  Ins_Kf->Acce_Bias[N]=constrain_float(Ins_Kf->Acce_Bias[N],-200,200);
		  Ins_Kf->Acce_Bias_All[N]=Acce_Bias_Gain_Kp*Z_Cor+Ins_Kf->Acce_Bias[N];			
		}

    //����״̬Э�������
    Pre_conv[0]=(1-kalman_k[0])*Temp_conv[0];
    Pre_conv[1]=(1-kalman_k[0])*Temp_conv[1];
    Pre_conv[2]=Temp_conv[2]-kalman_k[1]*Temp_conv[0];
    Pre_conv[3]=Temp_conv[3]-kalman_k[1]*Temp_conv[1]; 
		*update_flag=0;
				
		//if(ABS(Z_Cor)<=50&&Unusual_Protect_Flag==1) Unusual_Protect_Cnt++;
		if(Unusual_Protect_Flag==1) Unusual_Protect_Cnt++;
	  if(Unusual_Protect_Cnt>=2&&Unusual_Protect_Flag==1)//100ms
		{
			Unusual_Protect_Flag=0;
		  Unusual_Protect_Cnt=0;
		}
	}	
	
	obs_sync_cnt++;
	if(obs_sync_cnt>=2)//10ms����һ��
	{
		for(int16_t i=Num-1;i>0;i--)
		{
			Ins_Kf->Pos_Backups[N][i]=Ins_Kf->Pos_Backups[N][i-1];
			Ins_Kf->Vel_Backups[N][i]=Ins_Kf->Vel_Backups[N][i-1];
		}
		obs_sync_cnt=0;
  }
	Ins_Kf->Pos_Backups[N][0]=Ins_Kf->Position[N];
	Ins_Kf->Vel_Backups[N][0]=Ins_Kf->Speed[N]; 
}


void Alt_SINS_Init(void)
{
	uint16_t _cnt=0;
	if(Sensor_Flag.Ground_Health==1)
	{
		NamelessQuad.Position[_UP]=GD_Distance;
		for(_cnt=Num-1;_cnt>0;_cnt--){NamelessQuad.Pos_Backups[_UP][_cnt]=GD_Distance;}
		NamelessQuad.Pos_Backups[_UP][0]=GD_Distance;
		Total_Controller.Height_Position_Control.Expect=GD_Distance;//���ߵ��߶�����Ϊ�����߶ȣ����ҽ�����һ��
	}
	else
	{
		NamelessQuad.Position[_UP]=WP_Sensor.baro_altitude;
		for(_cnt=Num-1;_cnt>0;_cnt--){NamelessQuad.Pos_Backups[_UP][_cnt]=WP_Sensor.baro_altitude;}
		NamelessQuad.Pos_Backups[_UP][0]=WP_Sensor.baro_altitude;
		Total_Controller.Height_Position_Control.Expect=GD_Distance;//���ߵ��߶�����Ϊ�����߶ȣ����ҽ�����һ��
	}
}


uint16 High_Delay_Cnt=0;
uint8_t Observation_Update_Flag=0;
float Observation_Altitude=0;
uint8_t altitude_updtate_flag=0;
/***********************************************************
@��������Observation_Tradeoff
@��ڲ�����uint8_t HC_SR04_Enable
@���ڲ�������
�����������߶ȷ���۲⴫���������л�
@���ߣ�����С��
@���ڣ�2024/01/20
*************************************************************/
void Observation_Tradeoff(uint8_t ground_enable)
{
  static uint8_t observation_flag=2,last_observation_flag=2;
  uint16 Cnt=0;
  last_observation_flag=observation_flag;
  if(ground_enable==0)//�޳�����/�Եش���������
  {
    Observation_Altitude=WP_Sensor.baro_altitude;//�߶ȹ۲���
    High_Delay_Cnt=SPL06_Sync_Cnt;
		observation_flag=1;
		observation_div=WP_Sensor.baro_altitude_div;
		observation_acc=WP_Sensor.baro_altitude_acc;
		if(WP_Sensor.baro_updtate_flag==1)
		{
			altitude_updtate_flag=WP_Sensor.baro_updtate_flag;
			WP_Sensor.baro_updtate_flag=0;
		}
     R=R_Baro;
  }
  else//�г�����/�Եش���������
  {
		if(rangefinder_current==US100)
		{	
				if(Sensor_Flag.Ground_Health==1)
				{
					Observation_Altitude=GD_Distance;
					High_Delay_Cnt=HCSR04_Sync_Cnt;
					observation_flag=2;
					observation_div=GD_Distance_Div;
					observation_acc=GD_Distance_Acc;
					if(WP_Sensor.us100_updtate_flag==1)
					{
						altitude_updtate_flag=WP_Sensor.us100_updtate_flag;
						WP_Sensor.us100_updtate_flag=0;
					}
					R=R_Ground;
				}
				else
				{
					Observation_Altitude=WP_Sensor.baro_altitude;//�߶ȹ۲���
					High_Delay_Cnt=SPL06_Sync_Cnt;
					observation_flag=1;
					observation_div=WP_Sensor.baro_altitude_div;
					observation_acc=WP_Sensor.baro_altitude_acc;
					if(WP_Sensor.baro_updtate_flag==1)
					{
						altitude_updtate_flag=WP_Sensor.baro_updtate_flag;
						WP_Sensor.baro_updtate_flag=0;
					}
					R=R_Baro;
				} 
		}
		else if(rangefinder_current==TFMINI)
		{	
				if(Sensor_Flag.Ground_Health==1)
				{
					Observation_Altitude=GD_Distance;
					High_Delay_Cnt=5;
					observation_flag=2;
					observation_div=GD_Distance_Div;
					observation_acc=GD_Distance_Acc;
					if(WP_Sensor.tfmini_updtate_flag==1)
					{
						altitude_updtate_flag=WP_Sensor.tfmini_updtate_flag;
						WP_Sensor.tfmini_updtate_flag=0;
					}
					R=R_Ground;
				}
				else
				{
					Observation_Altitude=WP_Sensor.baro_altitude;//�߶ȹ۲���
					High_Delay_Cnt=SPL06_Sync_Cnt;
					observation_flag=1;
					observation_div=WP_Sensor.baro_altitude_div;
					observation_acc=WP_Sensor.baro_altitude_acc;
					if(WP_Sensor.baro_updtate_flag==1)
					{
						altitude_updtate_flag=WP_Sensor.baro_updtate_flag;
						WP_Sensor.baro_updtate_flag=0;
					}
					R=R_Baro;
				} 
		}
		else if(rangefinder_current==TOFSENSE)
		{	
				if(Sensor_Flag.Ground_Health==1)
				{
					Observation_Altitude=GD_Distance;
					High_Delay_Cnt=TOF_Sync_Cnt;
					observation_flag=2;
					observation_div=GD_Distance_Div;
					observation_acc=GD_Distance_Acc;
					if(WP_Sensor.tofsensor_updtate_flag==1)
					{
						altitude_updtate_flag=WP_Sensor.tofsensor_updtate_flag;
						WP_Sensor.tofsensor_updtate_flag=0;
					}
					R=R_TOFSENSOR;
				}
				else
				{
					Observation_Altitude=WP_Sensor.baro_altitude;//�߶ȹ۲���
					High_Delay_Cnt=SPL06_Sync_Cnt;
					observation_flag=1;
					observation_div=WP_Sensor.baro_altitude_div;
					observation_acc=WP_Sensor.baro_altitude_acc;
					if(WP_Sensor.baro_updtate_flag==1)
					{
						altitude_updtate_flag=WP_Sensor.baro_updtate_flag;
						WP_Sensor.baro_updtate_flag=0;
					}
					R=R_Baro;
				} 
		} 		
		else if(rangefinder_current==SMD15)
		{	
				if(Sensor_Flag.Ground_Health==1)
				{
					Observation_Altitude=GD_Distance;
					High_Delay_Cnt=TOF_Sync_Cnt;
					observation_flag=2;
					observation_div=GD_Distance_Div;
					observation_acc=GD_Distance_Acc;
					if(WP_Sensor.tofsensor_updtate_flag==1)
					{
						altitude_updtate_flag=WP_Sensor.tofsensor_updtate_flag;
						WP_Sensor.tofsensor_updtate_flag=0;
					}
					R=R_TOFSENSOR;
				}
				else
				{
					Observation_Altitude=WP_Sensor.baro_altitude;//�߶ȹ۲���
					High_Delay_Cnt=SPL06_Sync_Cnt;
					observation_flag=1;
					observation_div=WP_Sensor.baro_altitude_div;
					observation_acc=WP_Sensor.baro_altitude_acc;
					if(WP_Sensor.baro_updtate_flag==1)
					{
						altitude_updtate_flag=WP_Sensor.baro_updtate_flag;
						WP_Sensor.baro_updtate_flag=0;
					}
					R=R_Baro;
				} 
		}  		
		else if(rangefinder_current==TOFSENSE_M)
		{	
				if(Sensor_Flag.Ground_Health==1)
				{
					Observation_Altitude=GD_Distance;
					High_Delay_Cnt=TOF_Sync_Cnt;
					observation_flag=2;
					observation_div=GD_Distance_Div;
					observation_acc=GD_Distance_Acc;
					if(WP_Sensor.tofsensor_updtate_flag==1)
					{
						altitude_updtate_flag=WP_Sensor.tofsensor_updtate_flag;
						WP_Sensor.tofsensor_updtate_flag=0;
					}
					R=R_TOFSENSOR;
				}
				else
				{
					Observation_Altitude=WP_Sensor.baro_altitude;//�߶ȹ۲���
					High_Delay_Cnt=SPL06_Sync_Cnt;
					observation_flag=1;
					observation_div=WP_Sensor.baro_altitude_div;
					observation_acc=WP_Sensor.baro_altitude_acc;
					if(WP_Sensor.baro_updtate_flag==1)
					{
						altitude_updtate_flag=WP_Sensor.baro_updtate_flag;
						WP_Sensor.baro_updtate_flag=0;
					}
					R=R_Baro;
				} 
		}  		
		
    if(observation_flag==2&&last_observation_flag==1)//��ѹ���г�����
    {
			if(rangefinder_current==US100)
			{
				NamelessQuad.Position[_UP]=GD_Distance;
				for(Cnt=Num-1;Cnt>0;Cnt--){NamelessQuad.Pos_Backups[_UP][Cnt]=GD_Distance;}
				NamelessQuad.Pos_Backups[_UP][0]=GD_Distance;
				Total_Controller.Height_Position_Control.Expect=GD_Distance;//���ߵ��߶�����Ϊ�����߶ȣ����ҽ�����һ��
				R=R_Ground;
			}
			else
			{	
				NamelessQuad.Position[_UP]=GD_Distance;
				for(Cnt=Num-1;Cnt>0;Cnt--){NamelessQuad.Pos_Backups[_UP][Cnt]=GD_Distance;}
				NamelessQuad.Pos_Backups[_UP][0]=GD_Distance;
				Total_Controller.Height_Position_Control.Expect=GD_Distance;//���ߵ��߶�����Ϊ�����߶ȣ����ҽ�����һ��
				R=R_Ground;
		  }
			for(uint16_t i=0;i<Num;i++)
			{
				NamelessQuad.Pos_Backups[_UP][i]=NamelessQuad.Position[_UP];
				NamelessQuad.Vel_Backups[_UP][i]=NamelessQuad.Speed[_UP];
			}		
    }
    else if(observation_flag==1&&last_observation_flag==2)//����������ѹ��
    {
      NamelessQuad.Position[_UP]=Observation_Altitude;
      for(Cnt=Num-1;Cnt>0;Cnt--){NamelessQuad.Pos_Backups[_UP][Cnt]=Observation_Altitude;}
      NamelessQuad.Pos_Backups[_UP][0]=Observation_Altitude;
      Total_Controller.Height_Position_Control.Expect=Observation_Altitude;//���ߵ��߶�����Ϊ�����߶ȣ����ҽ�����һ��
			
			for(uint16_t i=0;i<Num;i++)
			{
				NamelessQuad.Pos_Backups[_UP][i]=NamelessQuad.Position[_UP];
				NamelessQuad.Vel_Backups[_UP][i]=NamelessQuad.Speed[_UP];
			}
			R=R_Baro;
		}
  }	
}

Testime SINS_High_Delta;
float Delta_T;
/***********************************************************
@��������Strapdown_INS_High_Kalman
@��ڲ�������
@���ڲ�������
�����������߶ȷ��򿨶����˲�
@���ߣ�����С��
@���ڣ�2024/01/20
*************************************************************/
void Strapdown_INS_High_Kalman(void)
{
  Test_Period(&SINS_High_Delta);
  Delta_T=SINS_High_Delta.Time_Delta/1000.0f;
  if(Delta_T>1.05f*WP_Duty_Dt||Delta_T<0.95f*WP_Duty_Dt||isnan(Delta_T)!=0)   Delta_T=WP_Duty_Dt;  
	float accel_z=NamelessQuad.Inertial_Acceleration[_UP];
	if(Temperature_Stable_Flag==1)
	{
		Observation_Tradeoff(1);
		KalmanFilter(Observation_Altitude,//λ�ù۲���
								 High_Delay_Cnt,//�۲⴫������ʱ��
								 &NamelessQuad,//�ߵ��ṹ��
								 accel_z,//ϵͳԭʼ���������ߵ����ٶ�
								 Q,
								 R,
								 Delta_T,
								 _UP,
								 &altitude_updtate_flag);
	}
	else
  {
		Alt_SINS_Init();
		//NamelessQuad.Acce_Bias[_UP]=-NamelessQuad.Inertial_Acceleration[_UP];
  }
}

/*************************���¼�������ͶӰ��������Դ��APM3.2 AP.Math.c�ļ�******************************/
/***********************************************************
@��������longitude_scale
@��ڲ�����Location loc
@���ڲ�������
��������������ͶӰ
@���ߣ�����С��
@���ڣ�2024/01/20
*************************************************************/
float longitude_scale(Location loc)
{
  static int32_t last_lat;
  static float scale = 1.0;
  //�Ƚ�����γ�����ֵ�������ظ��������Һ���
  if (ABS(last_lat - loc.lat) < 100000) {
    // we are within 0.01 degrees (about 1km) of the
    // same latitude. We can avoid the cos() and return
    // the same scale factor.
    return scale;
  }
  scale = cosf(loc.lat * 1.0e-7f * DEG_TO_RAD);
  scale = constrain_float(scale, 0.01f, 1.0f);
  last_lat = loc.lat;
  return scale;
}
/*
return the distance in meters in North/East plane as a N/E vector
from loc1 to loc2
*/
Vector2f location_diff(Location loc1,Location loc2)
{
  Vector2f Location_Delta;
  Location_Delta.x=(loc2.lat - loc1.lat) * LOCATION_SCALING_FACTOR;//���뵥λΪm
  Location_Delta.y=(loc2.lng - loc1.lng) * LOCATION_SCALING_FACTOR * longitude_scale(loc2);//���뵥λΪm
  return Location_Delta;
}

// return distance in meters between two locations
float get_distance(Location loc1,Location loc2){
  float dlat              = (float)(loc2.lat - loc1.lat);
  float dlong             = ((float)(loc2.lng - loc1.lng)) * longitude_scale(loc2);
  return pythagorous2(dlat, dlong) * LOCATION_SCALING_FACTOR;
}
/*************************���ϼ�������ͶӰ��������Դ��APM3.2 AP.Math.c�ļ�******************************/
float Body_Frame_To_XYZ[3]={0,0,0};//��������������ϵ��������(Y��)������(X��)����λ��ƫ��
Vector3_Nav Earth_Frame_To_XYZ={0,0,0};//��������������ϵ���� ������������������λ��ƫ��
Vector2i GPSData_To_XY_Home={1143637460,306276000};//����γ��
Location GPS_Home;//��ʼ��λ�ɹ�����Ϣ
Location GPS_Present;//��ǰλ�õ���Ϣ
uint8_t GPS_Home_Set=0;
float Declination=0;//�����ƫ��
/***********************************************************
@��������Set_GPS_Home
@��ڲ�������
@���ڲ�������
��������������GPSԭ��
@���ߣ�����С��
@���ڣ�2024/01/20
*************************************************************/
void Set_GPS_Home(void)//����Home��
{
	static uint16_t gps_home_cnt=0;
  if(GPS_Home_Set==0
	 &&GPS_Sate_Num>=9//�������ڵ���9
	 &&Horizontal_Acc_Est<=1.5f)//ˮƽλ�ù��ƾ���С��1.5m
	 {
		 if(gps_home_cnt<=50) gps_home_cnt++;//ˢ��10hz������5S����
	 }

   if(GPS_Home_Set==0&&gps_home_cnt==50)//ȫ��ֻ����һ��
  {
    GPSData_To_XY_Home.x=Longitude_Origion;//Longitude;
    GPSData_To_XY_Home.y=Latitude_Origion;//Latitude;
    GPS_Home_Set=1;//�趨������
    GPS_Home.lng=GPSData_To_XY_Home.x;
    GPS_Home.lat=GPSData_To_XY_Home.y; 
    Strapdown_INS_Reset(&NamelessQuad,_EAST,Earth_Frame_To_XYZ.E,0);//��λ�ߵ��ں�
    Strapdown_INS_Reset(&NamelessQuad,_NORTH,Earth_Frame_To_XYZ.N,0);//��λ�ߵ��ں�
    Declination=get_declination(0.0000001f*GPS_Home.lat,0.0000001f*GPS_Home.lng);//��ȡ���ش�ƫ��
  }
}

/***********************************************************
@��������GPSData_Sort
@��ڲ�������
@���ڲ�������
��������������GPSԭ�㣬�õ���γ�ȷ����λ��ƫ��
@���ߣ�����С��
@���ڣ�2024/01/20
*************************************************************/
void GPSData_Sort()
{
  Vector2f location_delta={0,0};
  GPS_Present.lng=Longitude_Origion;//���µ�ǰ��γ��
  GPS_Present.lat=Latitude_Origion;
  location_delta=location_diff(GPS_Home,GPS_Present);//���ݵ�ǰGPS��λ��Ϣ��Home��λ����Ϣ������������������λ��ƫ��
  /***********************************************************************************
  ��ȷ�µ���ϵ������������������Ϊ������:
  ������������������,�����˻����home�㣬���������ƶ�ʱ����ʱGPS_Present.lng>GPS_Home.lng���õ���location_delta.x����0;
  ����������γ������,�����˻����home�㣬���������ƶ�ʱ����ʱGPS_Present.lat>GPS_Home.lat���õ���location_delta.y����0;
  ******************************************************************************/
  location_delta.x*=100.0f;//�ص�������ϵ����������λ��ƫ��,��λΪCM
  location_delta.y*=100.0f;//�ص�������ϵ����������λ��ƫ��,��λΪCM
  Earth_Frame_To_XYZ.E=location_delta.y;//����ϵ�����Home������λ��ƫ�ƣ���λΪCM
  Earth_Frame_To_XYZ.N=location_delta.x;//����ϵ�����Home������λ��ƫ�ƣ���λΪCM
  //�����˻��ڵ�������ϵ�µ��������������������λ��ƫ����ת����ǰ�����λ��ƫ��:��ͷ(����)+���
  Body_Frame_To_XYZ[_EAST]=location_delta.x*WP_AHRS.cos_rpy[_YAW]+location_delta.y*WP_AHRS.sin_rpy[_YAW];//��ͷ����λ��ƫ��  Y������
  Body_Frame_To_XYZ[_NORTH]=-location_delta.x*WP_AHRS.sin_rpy[_YAW]+location_delta.y*WP_AHRS.cos_rpy[_YAW];  //�������λ��ƫ��  X������
}



#define Dealt_Update 0.1 //100ms����һ��
#define Process_Noise_Constant 1.0//1.0
#define Pos_Process_Noise  (0.5*Process_Noise_Constant*Dealt_Update*Dealt_Update)//0.005
#define Vel_Process_Noise  (Process_Noise_Constant*Dealt_Update)  //0.1
float   R_GPS[2]={Pos_Process_Noise,Vel_Process_Noise};
float   Q_GPS[2]={0.02,0.45};//{0.015,3.0}{0.075,3.0};
float   R_Acce_bias[2]={0.00025,0};//0.0001  0.001��ʼ�׷�ɢ
double  Pre_conv_GPS[2][4]=
{
  0.018 ,    0.005,  0.005    , 0.5,//0.0001 ,    0.00001,  0.00001    , 0.003,
  0.018 ,    0.005,  0.005    , 0.5,
};//��һ��Э����
double  K_GPS[2][2]={0};//�������
float   Acce_Bias[2]={0,0};//0  0.001
/***********************************************************
@��������KalmanFilter_Horizontal_GPS
@��ڲ�����float Position_GPS,float Vel_GPS,float Position_Last,
					 float Vel_Last,
					 float *Position,float *Vel,
					 float *Acce,float *R,
					 float *Q,float dt,uint8_t Axis
@���ڲ�������
����������GPSˮƽ���򿨶����˲��ں�
@���ߣ�����С��
@���ڣ�2024/01/20
*************************************************************/
void   KalmanFilter_Horizontal_GPS(float Position_GPS,float Vel_GPS,float Position_Last,float Vel_Last,float *Position,float *Vel,float *Acce,float *R,float *Q,float dt,uint8_t Axis)
{
  float Conv_Z=0;
  float Z_Delta[2]={0};
  float Conv_Temp=0;
  double Temp_conv[4]={0};//����Э����
  uint8 Label=0;
  if(Axis=='X') Label=0;
  else Label=1;
  //����״̬
  *Position +=*Vel*dt+((*Acce+Acce_Bias[Label])*dt*dt)/2.0f;
  *Vel+=(*Acce+Acce_Bias[Label])*dt;
  //����Э����
  Conv_Temp=Pre_conv_GPS[Label][1]+Pre_conv_GPS[Label][3]*dt;
  Temp_conv[0]=Pre_conv_GPS[Label][0]+Pre_conv_GPS[Label][2]*dt+Conv_Temp*dt+R_GPS[0];
  Temp_conv[1]=Conv_Temp;
  Temp_conv[2]=Pre_conv_GPS[Label][2]+Pre_conv_GPS[Label][3]*dt;
  Temp_conv[3]=Pre_conv_GPS[Label][3]+R_GPS[1];
  //���㿨��������
  Conv_Z=1.0/((Temp_conv[0]+Q_GPS[0]*GPS_Quality)*(Temp_conv[3]+Q_GPS[1]*GPS_Quality)-Temp_conv[1]*Temp_conv[2]);
  //��������
  K_GPS[0][0]=( Temp_conv[0]*(Temp_conv[3]+Q_GPS[1]*GPS_Quality)-Temp_conv[1]*Temp_conv[2])*Conv_Z;
  K_GPS[0][1]=(Temp_conv[1]*Q_GPS[0]*GPS_Quality)*Conv_Z;
  K_GPS[1][0]=(Temp_conv[2]*Q_GPS[1]*GPS_Quality)*Conv_Z;
  K_GPS[1][1]=(-Temp_conv[1]*Temp_conv[2]+Temp_conv[3]*(Temp_conv[0]+Q_GPS[0]*GPS_Quality))*Conv_Z;
  //�ں��������
  Z_Delta[0]=Position_GPS-Position_Last;
  Z_Delta[1]=Vel_GPS-Vel_Last;
	
	Z_Delta[0]=constrain_float(Z_Delta[0],-10000,10000);//100m
	Z_Delta[1]=constrain_float(Z_Delta[1],-2000,2000);//20m/s
	
  *Position +=K_GPS[0][0]*Z_Delta[0]+K_GPS[0][1]*Z_Delta[1];
  *Vel      +=K_GPS[1][0]*Z_Delta[0]+K_GPS[1][1]*Z_Delta[1];
	
  Acce_Bias[Label]+=R_Acce_bias[0]*Z_Delta[0]+R_Acce_bias[1]*Z_Delta[1];
	
	Acce_Bias[Label]=constrain_float(Acce_Bias[Label],-100,100);//100
	
  //����״̬Э�������
  Pre_conv_GPS[Label][0]=(1-K_GPS[0][0])*Temp_conv[0]-K_GPS[0][1]*Temp_conv[2];
  Pre_conv_GPS[Label][1]=(1-K_GPS[0][0])*Temp_conv[1]-K_GPS[0][1]*Temp_conv[3];
  Pre_conv_GPS[Label][2]=(1-K_GPS[1][1])*Temp_conv[2]-K_GPS[1][0]*Temp_conv[0];
  Pre_conv_GPS[Label][3]=(1-K_GPS[1][1])*Temp_conv[3]-K_GPS[1][0]*Temp_conv[1];
}

void Strapdown_INS_Reset(SINS *Ins,uint8_t Axis,float Pos_Target,float Vel_Target)
{
  uint16_t Cnt=0;
  Ins->Position[Axis]=Pos_Target;//λ������
  Ins->Speed[Axis]=Vel_Target;	 //�ٶ�����
  Ins->Acceleration[Axis]=0.0f;  //���ٶ�����
  Ins->Acce_Bias[Axis]=0.0f;
  for(Cnt=Num-1;Cnt>0;Cnt--)//��ʷλ��ֵ��ȫ��װ��Ϊ��ǰ�۲�ֵ
  {
    Ins->Pos_Backups[Axis][Cnt]=Pos_Target;
  }
  Ins->Pos_Backups[Axis][0]=Pos_Target;
  for(Cnt=Num-1;Cnt>0;Cnt--)//��ʷ�ٶ�ֵ��ȫ��װ��Ϊ��ǰ�۲�ֵ
  {
    Ins->Vel_Backups[Axis][Cnt]=Vel_Target;
  }
  Ins->Vel_Backups[Axis][0]=Vel_Target;
}


#define X_Axis  0
#define Y_Axis  1
uint16 GPS_Vel_Delay_Cnt=10;//5:50ms
uint16 GPS_Pos_Delay_Cnt=10;//10:100ms
Testime SINS_Horizontal_Delta;
float Horizontal_Delta_T=0;
uint8_t Filter_Defeated_Flag=0;
void KalmanFilter_Horizontal(void)
{	
  int16 i=0;
	static int16 GPS_Position_Cnt=0;
  if(GPS_Home_Set==1//��ʼhome��������
		&&Sensor_Flag.Mag_Health==TRUE)//������������Ч
  {
    GPSData_Sort();
    Test_Period(&SINS_Horizontal_Delta);
    Horizontal_Delta_T=(float)(SINS_Horizontal_Delta.Time_Delta/1000.0f);
		if(Horizontal_Delta_T>1.05f*WP_Duty_Dt||Horizontal_Delta_T<0.95f*WP_Duty_Dt||isnan(Horizontal_Delta_T)!=0)   Horizontal_Delta_T=WP_Duty_Dt; 
		
    GPS_Position_Cnt++;
    if(GPS_Position_Cnt>=2)//ÿ10ms����һ��
    {	
      for(i=Num-1;i>0;i--)
      {			
				NamelessQuad.Pos_Backups[_EAST][i]=NamelessQuad.Pos_Backups[_EAST][i-1];
				NamelessQuad.Pos_Backups[_NORTH][i] =NamelessQuad.Pos_Backups[_NORTH][i-1];
				NamelessQuad.Vel_Backups[_EAST][i]=NamelessQuad.Vel_Backups[_EAST][i-1];
				NamelessQuad.Vel_Backups[_NORTH][i] =NamelessQuad.Vel_Backups[_NORTH][i-1];	
			}
      NamelessQuad.Pos_Backups[_EAST][0]=NamelessQuad.Position[_EAST];
      NamelessQuad.Pos_Backups[_NORTH][0] =NamelessQuad.Position[_NORTH];
      NamelessQuad.Vel_Backups[_EAST][0]=NamelessQuad.Speed[_EAST];
      NamelessQuad.Vel_Backups[_NORTH][0] =NamelessQuad.Speed[_NORTH];			
      GPS_Position_Cnt=0;
    }
    NamelessQuad.Acceleration[_EAST]=NamelessQuad.Inertial_Acceleration[_EAST];
    NamelessQuad.Acceleration[_NORTH] =NamelessQuad.Inertial_Acceleration[_NORTH];
    if(GPS_Update_Flag==1)
    {
      KalmanFilter_Horizontal_GPS(Earth_Frame_To_XYZ.E,
                                  GPS_Speed_Resolve[1],
																	NamelessQuad.Pos_Backups[_EAST][GPS_Pos_Delay_Cnt],
																	NamelessQuad.Vel_Backups[_EAST][GPS_Vel_Delay_Cnt],
                                  &NamelessQuad.Position[_EAST],
                                  &NamelessQuad.Speed[_EAST],
                                  &NamelessQuad.Inertial_Acceleration[_EAST],
                                  R_GPS,Q_GPS,Horizontal_Delta_T,'X');
      KalmanFilter_Horizontal_GPS(Earth_Frame_To_XYZ.N,
                                  GPS_Speed_Resolve[0],
																	NamelessQuad.Pos_Backups[_NORTH][GPS_Pos_Delay_Cnt],
																	NamelessQuad.Vel_Backups[_NORTH][GPS_Vel_Delay_Cnt],
                                  &NamelessQuad.Position[_NORTH],
                                  &NamelessQuad.Speed[_NORTH],
                                  &NamelessQuad.Inertial_Acceleration[_NORTH],
                                  R_GPS,Q_GPS,Horizontal_Delta_T,'Y');
      GPS_Update_Flag=0;
    }
    else
    {
      NamelessQuad.Position[_EAST] +=NamelessQuad.Speed[_EAST]*Horizontal_Delta_T
        +0.5f*((NamelessQuad.Acceleration[_EAST]+Acce_Bias[0])*Horizontal_Delta_T*Horizontal_Delta_T);
			NamelessQuad.Speed[_EAST]+=(NamelessQuad.Acceleration[_EAST]+Acce_Bias[0])*Horizontal_Delta_T;
      
      NamelessQuad.Position[_NORTH] +=NamelessQuad.Speed[_NORTH]*Horizontal_Delta_T
        +0.5f*((NamelessQuad.Acceleration[_NORTH]+Acce_Bias[1])*Horizontal_Delta_T*Horizontal_Delta_T);
      NamelessQuad.Speed[_NORTH]+=(NamelessQuad.Acceleration[_NORTH]+Acce_Bias[1])*Horizontal_Delta_T;
    }
    
    if(ABS(NamelessQuad.Position[_EAST]-Earth_Frame_To_XYZ.E)>10000
       ||ABS(NamelessQuad.Position[_NORTH]-Earth_Frame_To_XYZ.N)>10000
         ||ABS(NamelessQuad.Speed[_EAST]-GPS_Speed_Resolve[1])>10000
           ||ABS(NamelessQuad.Speed[_NORTH]-GPS_Speed_Resolve[0])>10000
             )
    {
      Filter_Defeated_Flag=1;//��ʼʱ���ں�ʧ�ܱ�־
    }
  }
}


/*****************************************************/
#define DYNAMIC_PROPERTY_DN_ACCEL_G   -4.0f //-4.0f
#define DYNAMIC_PROPERTY_UP_ACCEL_G    4.0f // 4.0f
#define ACC_BIAS_MAX 100.0f



#define _KALMAN_DT 0.05f
#define ACC_NOISE_DEFAULT  1.0f
float ACC_BIAS_P=1e-4f;//0.05f
float qk_init[3]={ACC_NOISE_DEFAULT*_KALMAN_DT*_KALMAN_DT*0.5f,
								 ACC_NOISE_DEFAULT*_KALMAN_DT,
								 0};
float rk_init[2]={0.1,0};//10,0
static float p_init[3][3]={
  0.81,0.68,0,  //4.95,2.83,//0.18,0.1,
  0.68,1.25,0,  //2.83,3.49 //0.1,0.18
	0   ,	0  ,0,
};
#define history_record_period 2//2*5=10ms
uint16_t fusion_sync_cnt=10;
systime alt_obs_delta;
kalman_filter slam_kf;


void kalman_filter_init(kalman_filter *kf,float *p,float qp,float qv,float qb,float rp,float rv)
{
	for(uint8_t i=0;i<3;i++)
		for(uint8_t j=0;j<3;j++)
	{
		kf->p[i][j]=kf->p[i][j];
		kf->k[i][j]=0;
	}
	kf->qp=qp;
	kf->qv=qv;
	kf->qb=qb;
	kf->rp=rp;
	kf->rv=rv;
	kf->init=1;
}

void altitude_kalman_filter(kalman_filter *kf,SINS_Lite *_ins,float dt)
{
	if(current_state.loam_update_flag==0) return ;
	if(kf->init==0)	
	{
		//�״ν���,��ʼ���˲���
		kalman_filter_init(kf,
											&p_init[0][0],
											 qk_init[0],
											 qk_init[1],
											 qk_init[2],
											 rk_init[0],
											 rk_init[1]);
		return;
	}
#define P   kf->p
#define QP  kf->qp
#define QV  kf->qv
#define R   kf->rp	
#define K   kf->k	
#define ERR kf->err
#define CP  kf->cp
#define CV  kf->cv	
#define CB  kf->cb	
	//���ݶ����������ԣ��Թߵ����ٶȽ���Լ��
	Vector2f acc;
	acc.x= NamelessQuad.Inertial_Acceleration[_EAST];
	acc.y= NamelessQuad.Inertial_Acceleration[_NORTH];
	acc.x=constrain_float(acc.x,DYNAMIC_PROPERTY_DN_ACCEL_G*GRAVITY_MSS*100,DYNAMIC_PROPERTY_UP_ACCEL_G*GRAVITY_MSS*100);//-4G~8G
	acc.y=constrain_float(acc.y,DYNAMIC_PROPERTY_DN_ACCEL_G*GRAVITY_MSS*100,DYNAMIC_PROPERTY_UP_ACCEL_G*GRAVITY_MSS*100);//-4G~8G
	//1��ϵͳ����״̬����
  _ins->Acceleration[_EAST]=acc.x+_ins->Acce_Bias[_EAST];
	_ins->Position[_EAST]+=_ins->Speed[_EAST]*dt+0.5f*dt*dt*_ins->Acceleration[_EAST];
	_ins->Speed[_EAST]+=dt*_ins->Acceleration[_EAST];
  
	_ins->Acceleration[_NORTH]=acc.y+_ins->Acce_Bias[_NORTH];
	_ins->Position[_NORTH]+=_ins->Speed[_NORTH]*dt+0.5f*dt*dt*_ins->Acceleration[_NORTH];
	_ins->Speed[_NORTH]+=dt*_ins->Acceleration[_NORTH];
	
  if(current_state.update_flag==1)//��������SLAM����ʱ��50msһ��
  { 
		current_state.valid=1;
		current_state.update_flag=0; 

		float _dt=0.1f;
		//״̬���*
		ERR[0][0]=current_state.position_x-_ins->Pos_Backups[_EAST][fusion_sync_cnt];//5��10
		ERR[0][0]=constrain_float(ERR[0][0],-2000,2000);//��20m
		ERR[0][1]=0;//�������ٶȹ۲���
		
		ERR[1][0]=current_state.position_y-_ins->Pos_Backups[_NORTH][fusion_sync_cnt];//5��10
		ERR[1][0]=constrain_float(ERR[1][0],-2000,2000);//��20m
		ERR[1][1]=0;//�������ٶȹ۲���		
		
		float pt[3][3]={0};//����Э����		
    //2������Э����
    float ct =P[0][1]+P[1][1]*_dt;
    pt[0][0]=P[0][0]+P[1][0]*_dt+ct*_dt+QP;
    pt[0][1]=ct;
    pt[1][0]=P[1][0]+P[1][1]*_dt;
    pt[1][1]=P[1][1]+QV;
		
		//3�����㿨��������
    float cz=1/(pt[0][0]+R);
    K[0][0]=pt[0][0]*cz;//��̬ԼΪ0.069
		K[0][1]=0;
    K[1][0]=pt[1][0]*cz;//��̬ԼΪ0.096
		K[1][1]=0;
		
		CP[0]=K[0][0]*ERR[0][0]+K[0][1]*ERR[0][1];//λ��������
		CV[0]=K[1][0]*ERR[0][0]+K[1][1]*ERR[0][1];//�ٶ�������
		CB[0]=K[2][0]*ERR[0][0]+K[2][1]*ERR[0][1];//���ٶȼ�ƫ��������
		
		CP[1]=K[0][0]*ERR[1][0]+K[0][1]*ERR[1][1];//λ��������
		CV[1]=K[1][0]*ERR[1][0]+K[1][1]*ERR[1][1];//�ٶ�������
		CB[1]=K[2][0]*ERR[1][0]+K[2][1]*ERR[1][1];//���ٶȼ�ƫ��������		
		//4��ϵͳ����״̬����
	  _ins->Position[_EAST] +=CP[0];
	  _ins->Speed[_EAST]	  +=CV[0];
	  _ins->Position[_NORTH]+=CP[1];
	  _ins->Speed[_NORTH]	  +=CV[1];
		
		_ins->Acce_Bias[_EAST] +=ACC_BIAS_P*ERR[0][0]*_dt;
		_ins->Acce_Bias[_EAST]  =constrain_float(_ins->Acce_Bias[_EAST],-ACC_BIAS_MAX,ACC_BIAS_MAX);
		_ins->Acce_Bias[_NORTH]+=ACC_BIAS_P*ERR[1][0]*_dt;
		_ins->Acce_Bias[_NORTH] =constrain_float(_ins->Acce_Bias[_NORTH],-ACC_BIAS_MAX,ACC_BIAS_MAX);		
    //5������״̬Э�������
		float kt=(1-K[0][0]);
    P[0][0]=kt*pt[0][0];
    P[0][1]=kt*pt[0][1];
    P[1][0]=pt[1][0]-K[1][0]*pt[0][0];
    P[1][1]=pt[1][1]-K[1][0]*pt[0][1];
	}	
#undef P
#undef QP
#undef QV
#undef R
#undef K
#undef ERR
#undef CP
#undef CV
#undef CB
	
	static uint32_t _cnt=0;//�ߵ���ʷֵ����
	_cnt++;
	if(_cnt%history_record_period==0)
	{
		for(uint16_t i=Num-1;i>0;i--)
		{
			_ins->Pos_Backups[_EAST][i] =_ins->Pos_Backups[_EAST][i-1];
			_ins->Vel_Backups[_EAST][i] =_ins->Vel_Backups[_EAST][i-1];
			_ins->Pos_Backups[_NORTH][i] =_ins->Pos_Backups[_NORTH][i-1];
			_ins->Vel_Backups[_NORTH][i] =_ins->Vel_Backups[_NORTH][i-1];
		}	
	}
	_ins->Pos_Backups[_EAST][0]=_ins->Position[_EAST];
	_ins->Vel_Backups[_EAST][0]=_ins->Speed[_EAST];
	_ins->Pos_Backups[_NORTH][0]=_ins->Position[_NORTH];
	_ins->Vel_Backups[_NORTH][0]=_ins->Speed[_NORTH];
	//��EN����״̬��ת��RP��
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

