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
#include "Calibration.h"
#include "CalibrationRoutines.h"


/**********************************************************************************************
���򷽳�:[beta3(x-beta0)]^2+[beta4(y-beta1)]^2+[beta4(z-beta2)]^2=1
����f(x)=[beta3(x-beta0)]^2+[beta4(y-beta1)]^2+[beta4(z-beta2)]^2-1
�������:D=[beta0,beta1,beta2,beta3,beta4,beta5]
��D������Ϊdelta��detla_D����������ÿ�ε����Ĳ���
���ſɱȾ���J(x)=[j[0],j[1],j[2],j[3],j[4],j[5]]'
j[0]=-2beta3*beta3*(Xk-beta0)
j[1]=-2beta4*beta4*(Yk-beta1)
j[2]=-2beta5*beta5*(Zk-beta2)
j[3]= 2beta3*(Xk-beta0)*(Xk-beta0)
j[4]= 2beta4*(Yk-beta1)*(Yk-beta1)
j[5]= 2beta5*(Zk-beta2)*(Zk-beta2)
������С�����и�˹ţ�ٷ������������
�Ǻ�ɭ����H(x)=J*J';g(x)=-J*f(x)����ȡ��С�����Ż����⣺H*D=g
�������£�
1��������ֵD(0)=[0,0,0,1,1,1]'
2������k�ε�����ֵ(Xi,Yi,Zi),��ȡ��ǰ���ſɱȾ���j(k)�����f(k)
3�������������:H*D=g,����õ���ǰ��delta(k)
4����delta(k)�㹻С����ֹͣ����
5��������D(k+1)=D(k)+delta(k)������ִ�е�2������

//�ر�ע��
1��ע����APM�������ʵ��������f_apm(x)=-f(x)=1-[beta3(x-beta0)]^2+[beta4(y-beta1)]^2+[beta4(z-beta2)]^2
2�����ڼ����ſɱȾ���JʱJ_APM(x)=-J(x)
3������H(x)ʱ��H_APM=J_APM(x)*J_APM(x)'=[-J(x)]*[-J(x)]'=J(x)*J(x)',�ʶ��ߵ�H������һ����
4��APM�����ڼ���gʱ��g_APM=J_apm(x)*f_apm(x),���ݸ�˹ţ�ٵ���������������ⷨΪg_APM=-J_APM(x)*f_APM(x)
5��APM����ʵ���ϣ�����в����ʱ�޸��ţ��ʵ�5������ʱȡD(k+1)=D(k)-delta(k)

��زο��������£�
https://chionophilous.wordpress.com/2011/10/24/accelerometer-calibration-iv-1-implementing-gauss-newton-on-an-atmega/
https://chionophilous.wordpress.com/2012/09/08/implementing-the-gauss-newton-algorithm-for-sphere-fitting-2-of-3/
https://chionophilous.wordpress.com/2011/08/26/accelerometer-calibration-iii-improving-accuracy-with-least-squares-and-the-gauss-newton-method/
https://en.wikipedia.org/wiki/Gauss%E2%80%93Newton_algorithm
https://github.com/rolfeschmidt/muCSense/blob/master/BestSphereGaussNewtonCalibrator.cpp
https://zhuanlan.zhihu.com/p/113946848
https://www.cnblogs.com/guanglun/p/12313193.html
https://blog.csdn.net/qq_31073871/article/details/87008583
https://zhuanlan.zhihu.com/p/634190807?utm_id=0
https://www.zhihu.com/tardis/bd/art/555298443
https://blog.csdn.net/jiangxing11/article/details/127251071
**********************************************************************************************/


/***************���ٶȼ�6��������ο�APM���룬���ң���������ֳ�����**************************/
/***********************************************************
@��������Calibrate_Reset_Matrices
@��ڲ�����float dS[6], float JS[6][6]
@���ڲ�������
@�����������������ݸ�λ
@���ߣ�����С��
@���ڣ�2024/01/20
*************************************************************/
static void Calibrate_Reset_Matrices(float dS[6], float JS[6][6])
{
  int16_t j,k;
  for( j=0; j<6; j++ )
  {
    dS[j] = 0.0f;
    for( k=0; k<6; k++ )
    {
      JS[j][k] = 0.0f;
    }
  }
}


/***********************************************************
@��������gaussElimination_six_order
@��ڲ�����float mat_Y[6], float mat_A[6][6], float x[6]
@���ڲ�������
@���������������󷽳�JS*x = dS����һ���Ѿ�����������
��JS���ڵ����·���ȫ����Ϊ0��Ȼ��ش��õ����Է��̵Ľ�
@���ߣ�����С��
@���ڣ�2024/01/20
*************************************************************/
#define N 6
static void gaussElimination_six_order(float mat_Y[N],float mat_A[N][N],float x[N]) 
{
	double mat[N][N + 1];
	for(uint16_t i=0;i<N;i++)
	{
		for(uint16_t j=0;j<N+1;j++)	
		{
			if(j!=N)  mat[i][j]=mat_A[i][j];
		  else mat[i][j]=mat_Y[i];
		}
	}
	
	for (int i = 0; i < N; i++) {
			// Making the diagonal element non-zero
			if (mat[i][i] == 0) {
					for (int k = i + 1; k < N; k++) {
							if (mat[k][i] != 0) {
									for (int j = 0; j <= N; j++) {
											double temp = mat[i][j];
											mat[i][j] = mat[k][j];
											mat[k][j] = temp;
									}
									break;
							}
					}
			}

			// Making the elements below the diagonal zero
			for (int k = i + 1; k < N; k++) {
					double factor = mat[k][i] / mat[i][i];
					for (int j = 0; j <= N; j++) {
							mat[k][j] -= factor * mat[i][j];
					}
			}
	}
	// mat�Ǹ�˹��Ԫ��ľ���
	for (int i = N - 1; i >= 0; i--) {
			x[i] = mat[i][N];
			for (int j = i + 1; j < N; j++) {
					x[i] -= mat[i][j] * x[j];
			}
			x[i] /= mat[i][i];
	}		
}

static void Calibrate_Update_Matrices(float dS[6],
																			float JS[6][6],
																			float beta[6],
																			float data[3])
{
  int16_t j, k;
  float dx, b;
  float residual = 1.0;
  float jacobian[6];
	//���ݵ�ǰ����,����ı��ε��ſɱȾ���J(x),������f(x)�Դ���״̬��beta��ƫ��
  for(j=0;j<3;j++)
  {
    b = beta[3+j];
    dx = (float)data[j] - beta[j];
		//����в�f(x)=[beta3(x-beta0)]^2+[beta4(y-beta1)]^2+[beta4(z-beta2)]^2-1�����൱���Ǽ���-f(x)
    residual -= b*b*dx*dx;
    jacobian[j] = 2.0f*b*b*dx;
    jacobian[3+j] = -2.0f*b*dx*dx;
  }
  
  for(j=0;j<6;j++)
  {
		//�ſɱȾ���Ͳв�ĳ˻���g(x)
    dS[j]+=jacobian[j]*residual;
		//�ſɱȾ�����ſɱȾ���ת�ó˻�������H(x)
    for(k=0;k<6;k++)
    {
      JS[j][k]+=jacobian[j]*jacobian[k];
    }
  }
}



uint8 Calibrate_accel(Acce_Unit accel_sample[6],
                      Acce_Unit *accel_offsets,
                      Acce_Unit *accel_scale)
{
  int16_t i;
  int16_t num_iterations = 0;
  float eps = 0.000000001;
  float change = 100.0;
  float data[3]={0};
  float beta[6]={0};
  float delta[6]={0};
  float ds[6]={0};
  float JS[6][6]={0};
  bool success = TRUE;
  //��һ������������ʼֵD(0)
  beta[0] = beta[1] = beta[2] = 0;
  beta[3] = beta[4] = beta[5] = 1.0f/GRAVITY_MSS;
  while( num_iterations < 20 && change > eps )//�����͵����������� 
	{
    num_iterations++;
		//��λg=ds��H=JS
    Calibrate_Reset_Matrices(ds, JS);
		//�ڶ�������k�ε�����ֵ(Xi,Yi,Zi),��ȡ��ǰ���ſɱȾ���j(k)�����f(k)
    for( i=0; i<6; i++ ) {
      data[0] = accel_sample[i].x;
      data[1] = accel_sample[i].y;
      data[2] = accel_sample[i].z;
      Calibrate_Update_Matrices(ds, JS, beta, data);
    }
		//�����������������:H*D=g,����õ���ǰ��delta(k)
    gaussElimination_six_order(ds, JS, delta);
		//���Ĳ���delta(k)�㹻С����ֹͣ�������˴��õ���delta��ƽ����Ϊ�����о�
    change =delta[0]*delta[0] +
						delta[1]*delta[1] +
						delta[2]*delta[2] +
						delta[3]*delta[3] / (beta[3]*beta[3]) +
						delta[4]*delta[4] / (beta[4]*beta[4]) +
						delta[5]*delta[5] / (beta[5]*beta[5]);
		//���岽��D(k+1)=D(k)+delta(k)������ִ�е�2������
    for( i=0; i<6; i++ ) {
      beta[i] -= delta[i];
    }
  }
  // copy results out
  accel_scale->x = beta[3] * GRAVITY_MSS;
  accel_scale->y = beta[4] * GRAVITY_MSS;
  accel_scale->z = beta[5] * GRAVITY_MSS;
  accel_offsets->x = beta[0] * accel_scale->x;
  accel_offsets->y = beta[1] * accel_scale->y;
  accel_offsets->z = beta[2] * accel_scale->z;
  
  // sanity check scale
  if(fabsf(accel_scale->x-1.0f) > 0.2f
     || fabsf(accel_scale->y-1.0f) > 0.2f
       || fabsf(accel_scale->z-1.0f) > 0.2f )
  {
    success = FALSE;
  }
  // sanity check offsets (3.0 is roughly 3/10th of a G, 5.0 is roughly half a G)
  if(fabsf(accel_offsets->x) > 5.0f
     || fabsf(accel_offsets->y) > 5.0f
       || fabsf(accel_offsets->z) > 5.0f )
  {
    success = FALSE;
  }
  // return success or failure
  return success;
}



Acce_Unit new_offset={
  0,0,0,
};
Acce_Unit new_scales={
  1.0,1.0,1.0,
};

Acce_Unit Accel_Offset_Read={
  0,0,0,
};
Acce_Unit Accel_Scale_Read={
  0,0,0,
};


Acce_Unit Accel_Hor_Read={
  0,0,0,
};
uint8_t cal_finished_flag=0;



uint8_t flight_direction=6;
uint8_t Accel_Calibration_Flag=0;//���ٶȼ�У׼ģʽ
uint8_t Accel_Calibration_Finished[6]={0,0,0,0,0,0};//��Ӧ��У׼��ɱ�־λ
uint8_t Accel_Calibration_All_Finished=0;//6��У׼ȫ��У׼��ɱ�־λ
uint16_t Accel_Calibration_Makesure_Cnt=0;
uint16_t Accel_flight_direction_cnt=0;
/***********************************************************
@��������Accel_Calibration_Check
@��ڲ�������
@���ڲ�������
@�������������ٶȼƱ궨����ң��������λ���
����
@���ߣ�����С��
@���ڣ�2024/01/20
*************************************************************/
void Accel_Calibration_Check(void)
{
  uint16_t  i=0;
  if(Throttle_Control<=(1000+Buttom_Safe_Deadband)
     &&Yaw_Control>=Yaw_Max*Scale_Pecent_Max
       &&Roll_Control<=-Pit_Rol_Max*Scale_Pecent_Max
         &&Pitch_Control>=Pit_Rol_Max*Scale_Pecent_Max)
  {
    Accel_Calibration_Makesure_Cnt++;
  }

	if(((Throttle_Control<=(1000+Buttom_Safe_Deadband)
	 &&Yaw_Control>=Yaw_Max*Scale_Pecent_Max
		 &&Roll_Control<=-Pit_Rol_Max*Scale_Pecent_Max
			 &&Pitch_Control>=Pit_Rol_Max*Scale_Pecent_Max
				 &&Accel_Calibration_Makesure_Cnt>=200*3)||cal_flag==0x02)//��������
					 &&Controler_State==Lock_Controler)//����Ϊ����״̬�ſ��Խ���У׼ģʽ
		
  {
    Bling_Mode=1;
    Accel_Calibration_Flag=1;//���ٶ�У׼ģʽ
    cal_finished_flag=0;	
		Bling_Set(&rgb_red,2000,1000,0.2,0,GPIO_PORTF_BASE,GPIO_PIN_1,1);

    flight_direction=6;
    Accel_Calibration_All_Finished=0;//ȫ��У׼��ɱ�־λ����
    Accel_Calibration_Makesure_Cnt=0;
    for(i=0;i<6;i++)
    {
      Accel_Calibration_Finished[i]=0;//��Ӧ���־λ����
      acce_sample[i].x=0; //��ն�Ӧ��ļ��ٶȼ���
      acce_sample[i].y=0; //��ն�Ӧ��ļ��ٶȼ���
      acce_sample[i].z=0; //��ն�Ӧ��ļ��ٶȼ���
    }
    Page_Number=10;//OLED���ٶȼƽ���ҳ��
    Reset_Mag_Calibartion(1);
    Reset_RC_Calibartion(1);
    Forced_Lock_Makesure_Cnt=0;
		cal_flag=0x00;
  }
  
  if(Accel_Calibration_Flag==1)
  {
    if((Throttle_Control<=(1000+Buttom_Safe_Deadband)&&Yaw_Control<=-Yaw_Max*Scale_Pecent_Max&&Roll_Control==0&&Pitch_Control==0)
			||cal_step==0x01)
    {
      Accel_flight_direction_cnt++;
      if((Accel_flight_direction_cnt>=20)//100ms
				||cal_step==0x01)
      {
        flight_direction=0;
        Unlock_Makesure_Cnt=0;
        Lock_Makesure_Cnt=0;
      }
      cal_step=0x00;
    }
    else if((Throttle_Control<=(1000+Buttom_Safe_Deadband)&&Yaw_Control==0&&Roll_Control>=Pit_Rol_Max*Scale_Pecent_Max&&Pitch_Control==0)
			||cal_step==0x02)
    {
      Accel_flight_direction_cnt++;
      if((Accel_flight_direction_cnt>=20)//100ms
				||cal_step==0x02)
      {
        flight_direction=1;
        Unlock_Makesure_Cnt=0;
        Lock_Makesure_Cnt=0;
      }
			cal_step=0x00;
    }
    else if((Throttle_Control<=(1000+Buttom_Safe_Deadband)&&Yaw_Control==0&&Roll_Control<=-Pit_Rol_Max*Scale_Pecent_Max&&Pitch_Control==0)
			||cal_step==0x03)
    {
      Accel_flight_direction_cnt++;
      if((Accel_flight_direction_cnt>=20)//100ms
				||cal_step==0x03)
      {
        flight_direction=2;
        Unlock_Makesure_Cnt=0;
        Lock_Makesure_Cnt=0;
      }
			cal_step=0x00;
    }
    else if((Throttle_Control<=(1000+Buttom_Safe_Deadband)&&Yaw_Control==0&&Roll_Control==0&&Pitch_Control>=Pit_Rol_Max*Scale_Pecent_Max)
			||cal_step==0x04)
    {
      Accel_flight_direction_cnt++;
      if((Accel_flight_direction_cnt>=20)//100ms
				||cal_step==0x04)
      {
        flight_direction=3;
        Unlock_Makesure_Cnt=0;
        Lock_Makesure_Cnt=0;
      }
			cal_step=0x00;
    }
    else if((Throttle_Control<=(1000+Buttom_Safe_Deadband)&&Yaw_Control==0&&Roll_Control==0&&Pitch_Control<=-Pit_Rol_Max*Scale_Pecent_Max)
			||cal_step==0x05)
    {
      Accel_flight_direction_cnt++;
      if((Accel_flight_direction_cnt>=20)//100ms
				||cal_step==0x05)
      {
        flight_direction=4;
        Unlock_Makesure_Cnt=0;
        Lock_Makesure_Cnt=0;
      }
			cal_step=0x00;
    }
    else if((Throttle_Control<=(1000+Buttom_Safe_Deadband)&&Yaw_Control>Yaw_Max*Scale_Pecent_Max&&Roll_Control==0&&Pitch_Control==0)
			||cal_step==0x06)
    {
      Accel_flight_direction_cnt++;
      if((Accel_flight_direction_cnt>=20)//100ms
				||cal_step==0x06)
      {
        flight_direction=5;
        Unlock_Makesure_Cnt=0;
        Lock_Makesure_Cnt=0;
      }
			cal_step=0x00;
    }
    else
    {
      Accel_flight_direction_cnt/=2;
    }
    
    if(Accel_flight_direction_cnt>=200)  Accel_flight_direction_cnt=0;
    
  }
  
}

Acce_Unit acce_sample[6]={0};//����6�У�����6�����������
uint8_t Flash_Buf[12]={0};
/***********************************************************
@��������Accel_Calibartion
@��ڲ�������
@���ڲ�������
@�������������ٶȱ궨������ң����ֱ�ӽ���
@���ߣ�����С��
@���ڣ�2024/01/20
*************************************************************/
uint8_t Accel_Calibartion(void)
{
  uint16 i,j=0;
  float acce_sample_sum[3]={0,0,0};//���ٶȺ�����
  /*��һ��ɿ�ƽ�ţ�Z�����������Ϸ���Z axis is about 1g,X��Y is about 0g*/
  /*�ڶ���ɿ�ƽ�ţ�X�����������Ϸ���X axis is about 1g,Y��Z is about 0g*/
  /*������ɿ�ƽ�ţ�X�����������·���X axis is about -1g,Y��Z is about 0g*/
  /*������ɿ�ƽ�ţ�Y�����������·���Y axis is about -1g,X��Z is about 0g*/
  /*������ɿ�ƽ�ţ�Y�����������Ϸ���Y axis is about 1g,X��Z is about 0g*/
  /*������ɿ�ƽ�ţ�Z�����������·���Z axis is about -1g,X��Y is about 0g*/
  if(flight_direction<=5)//��⵽��Ӧ������
  {
    uint16_t num_samples=0;
    while(num_samples<1000)//����200��
    {
      if(Gyro_Length<=20.0f
         &&WP_Sensor.imu_updtate_flag==1)//ͨ��������ģ����ȷ�����徲ֹ
      {
        for(j=0;j<3;j++){
          acce_sample_sum[j]+=WP_Sensor.acce_filter[j]*RAW_TO_G;//���ٶȼ�ת��Ϊ1g������
        }
        //delay_ms(4);//���10ms��1s������ȡƽ��
        num_samples++;
        WP_Sensor.imu_updtate_flag=0;
      }
      Accel_Calibration_Finished[flight_direction]=1;//��Ӧ��У׼��ɱ�־λ��1
    }
    acce_sample[flight_direction].x=acce_sample_sum[0]/num_samples; //�����Ӧ��ļ��ٶȼ���
    acce_sample[flight_direction].y=acce_sample_sum[1]/num_samples; //�����Ӧ��ļ��ٶȼ���
    acce_sample[flight_direction].z=acce_sample_sum[2]/num_samples; //�����Ӧ��ļ��ٶȼ���
    flight_direction=6;//����������
  }
  
  if((Accel_Calibration_Finished[0]
      &Accel_Calibration_Finished[1]
        &Accel_Calibration_Finished[2]
          &Accel_Calibration_Finished[3]
            &Accel_Calibration_Finished[4]
              &Accel_Calibration_Finished[5])
     &&Accel_Calibration_All_Finished==0)//6��ȫ��У׼���
  {
    Accel_Calibration_All_Finished=1;//���ٶȼ�6��У׼��ɱ�־
    Accel_Calibration_Flag=0;//���ٶȼ�У׼�������ͷ�ң�в���
    cal_finished_flag=Calibrate_accel(acce_sample,
                             &new_offset,
                             &new_scales);//������6������
    for(i=0;i<6;i++)
    {
      Accel_Calibration_Finished[i]=0;//��Ӧ���־λ����
    }
    if(cal_finished_flag==TRUE)//���ٶȼ�У׼�ɹ�
    {
      WriteFlashParameter_Three(ACCEL_X_OFFSET1,
                                new_offset.x,
                                new_offset.y,
                                new_offset.z);
      WriteFlashParameter_Three(ACCEL_X_SCALE1,
                                new_scales.x,
                                new_scales.y,
                                new_scales.z);
      
      Parameter_Init();//��ȡд�����
      Bling_Mode=0;//�ָ�����ָʾģʽ
			Bling_Set(&rgb_red,5000,2000,0.5,0,GPIO_PORTF_BASE,GPIO_PIN_1,0);
			NCLink_Send_Check_Flag[9]=0x01;//У׼��Ϻ���У׼����
    }
    else//���ٶȼ�У׼ʧ��
    {
      Bling_Mode=0;//�ָ�����ָʾģʽ
			Bling_Set(&rgb_red,5000,2000,0.1,0,GPIO_PORTF_BASE,GPIO_PIN_1,0);
      Page_Number=0;//OLED�ָ���ҳ
    }
    return TRUE;
  }
  return FALSE;
}


/***********************************************************
@��������Reset_Accel_Calibartion
@��ڲ�����uint8_t Type
@���ڲ�������
@�������������ٶȱ궨���������ǿ�Ƹ�λ
@���ߣ�����С��
@���ڣ�2024/01/20
*************************************************************/
void Reset_Accel_Calibartion(uint8_t Type)
{
  uint16 i=0;
  for(i=0;i<6;i++)
  {
    Accel_Calibration_Finished[i]=0;//��Ӧ���־λ����
    acce_sample[i].x=0; //��ն�Ӧ��ļ��ٶȼ���
    acce_sample[i].y=0; //��ն�Ӧ��ļ��ٶȼ���
    acce_sample[i].z=0; //��ն�Ӧ��ļ��ٶȼ���
  }
  Accel_Calibration_All_Finished=0;//ȫ��У׼��ɱ�־λ����
  if(Type==1)  Accel_Calibration_Flag=0;
}


typedef struct
{
  uint8_t accel_off;
  uint8_t accel_scale;
  uint8_t mag;
}Parameter_Flag;

Parameter_Flag Parameter_Read_Flag;
float Accel_Simple_Mode=1;//Ĭ�ϼ��ٶȼƼ�У׼ģʽ
/***********************************************************
@��������Parameter_Init
@��ڲ�������
@���ڲ�������
@����������������У׼������ʼ��
@���ߣ�����С��
@���ڣ�2024/01/20
*************************************************************/
bool Parameter_Init(void)//���ٶȼơ�������У׼������ȡ
{
  bool success=TRUE;
  /************���ٶȼ���ƫ����ֵ*******/
  ReadFlashParameterThree(ACCEL_X_OFFSET1,
                          &Accel_Offset_Read.x,
                          &Accel_Offset_Read.y,
                          &Accel_Offset_Read.z);
  
  ReadFlashParameterThree(ACCEL_X_SCALE1,
                          &Accel_Scale_Read.x,
                          &Accel_Scale_Read.y,
                          &Accel_Scale_Read.z);
  /************��������ƫ****************/
  ReadFlashParameterThree(MAG_X_OFFSET,
                          &Mag_Offset_Read.x,
                          &Mag_Offset_Read.y,
                          &Mag_Offset_Read.z);
	
	ReadFlashParameterThree(HOR_CAL_ACCEL_X,
													&Accel_Hor_Read.x,
													&Accel_Hor_Read.y,
													&Accel_Hor_Read.z);	
  // sanity check scale
  if(ABS(Accel_Scale_Read.x-1.0f)>0.5f
     || ABS(Accel_Scale_Read.y-1.0f)>0.5f
       || ABS(Accel_Scale_Read.z-1.0f)>0.5f)
  {
    success = FALSE;
  }
  // sanity check offsets (3.5 is roughly 3/10th of a G, 5.0 is roughly half a G)
  if(ABS(Accel_Offset_Read.x) > 5.0f
     || ABS(Accel_Offset_Read.y) > 5.0f
       || ABS(Accel_Offset_Read.z) > 5.0f)
  {
    success = FALSE;
  }
  
  
  if(isnan(Accel_Offset_Read.x)==0
     &&isnan(Accel_Offset_Read.y)==0
       &&isnan(Accel_Offset_Read.z)==0
         &&isnan(Accel_Scale_Read.x)==0
           &&isnan(Accel_Scale_Read.y)==0
             &&isnan(Accel_Scale_Read.z)==0)//Flash���������������¼��ٶ�У��ֵ
  {    
    B[0]=Accel_Offset_Read.x;
    B[1]=Accel_Offset_Read.y;
    B[2]=Accel_Offset_Read.z;
    K[0]=Accel_Scale_Read.x;
    K[1]=Accel_Scale_Read.y;
    K[2]=Accel_Scale_Read.z;	
  }
	else if(success==TRUE)
	{
    B[0]=0;
    B[1]=0;
    B[2]=0;
    K[0]=1;
    K[1]=1;
    K[2]=1;
	}

	float _accel_simple_mode=0;	
	ReadFlashParameterOne(ACCEL_SIMPLE_MODE,&_accel_simple_mode);
	if(isnan(_accel_simple_mode)==0)   Accel_Simple_Mode=_accel_simple_mode;
	vector3f _accel_hor;	
	ReadFlashParameterThree(HOR_CAL_ACCEL_X,&_accel_hor.x,&_accel_hor.y,&_accel_hor.z);
	if(isnan(_accel_hor.x)==0
		 &&isnan(_accel_hor.y)==0
			 &&isnan(_accel_hor.z)==0)
	{
	  Accel_Hor_Read.x=_accel_hor.x;
		Accel_Hor_Read.y=_accel_hor.y;
		Accel_Hor_Read.z=_accel_hor.z;
	}
	
	if(Accel_Simple_Mode==1)//���ٶȼ�У׼������ģʽ
	{
		 if(isnan(Accel_Hor_Read.x)==0
			&&isnan(Accel_Hor_Read.y)==0
			&&isnan(Accel_Hor_Read.z)==0)//Flash����������
		{
			B[0]=Accel_Hor_Read.x;
			B[1]=Accel_Hor_Read.y;
			B[2]=Accel_Hor_Read.z;
			K[0]=1;
			K[1]=1;
			K[2]=1;
			Pitch_Offset=0;
			Roll_Offset=0;			
		}
		else
		{
			B[0]=0;
			B[1]=0;
			B[2]=0;
			K[0]=1;
			K[1]=1;
			K[2]=1;
		}
	}	
	
	
  /**********����������ƫִ��ȡ************/
  if(isnan(Mag_Offset_Read.x)==0
     &&isnan(Mag_Offset_Read.y)==0
       &&isnan(Mag_Offset_Read.z==0))
  {
    mag_offset.x=(int16_t)(Mag_Offset_Read.x);
    mag_offset.y=(int16_t)(Mag_Offset_Read.y);
    mag_offset.z=(int16_t)(Mag_Offset_Read.z);
  }
  else
  {
    mag_offset.x=0;
    mag_offset.y=0;
    mag_offset.z=0;    
  }
  return success;
}
/************���ٶȼ�6���������***********************/


/***********���������Ľ�����ȡ���������Сֵƽ��******/
uint8_t Mag_Calibration_Flag=0,Mag_Calibration_All_Finished;
uint8_t Mag_Calibration_Finished[3]={0};
uint16_t Mag_Calibration_Makesure_Cnt=0;
uint8_t  Mag_Calibration_Mode=3;
uint16_t Mag_Calibration_Cnt=0;
float Yaw_Correct=0;
/*********************************************/
const int16_t Mag_360_define[36]={
  0,10,20,30,40,50,60,70,80,90,
  100,110,120,130,140,150,160,170,180,190,
  200,210,220,230,240,250,260,270,280,290,
  300,310,320,330,340,350
};//�����ƽ��������Ƕȣ�ȷ�����ݲɼ����
uint8_t Last_Mag_360_Flag[3][36]={0};
uint8_t Mag_360_Flag[3][36]={0};
uint16_t Mag_Is_Okay_Flag[3];
Calibration Mag;
Mag_Unit DataMag;
Mag_Unit Mag_Offset_Read={
  0,0,0,
};
/***********************************************************
@��������Mag_Calibration_Check
@��ڲ�������
@���ڲ�������
@���������������Ʊ궨����ң��������λ�������
@���ߣ�����С��
@���ڣ�2024/01/20
*************************************************************/
void Mag_Calibration_Check(void)
{
  uint16_t  i=0,j=0;
  if(Throttle_Control<=(1000+Buttom_Safe_Deadband)
     &&Yaw_Control>=Yaw_Max*Scale_Pecent_Max
       &&Roll_Control>=Pit_Rol_Max*Scale_Pecent_Max
         &&Pitch_Control>=Pit_Rol_Max*Scale_Pecent_Max)
    Mag_Calibration_Makesure_Cnt++;
  
  if(((Throttle_Control<=(1000+Buttom_Safe_Deadband)
     &&Yaw_Control>=Yaw_Max*Scale_Pecent_Max
       &&Roll_Control>=Pit_Rol_Max*Scale_Pecent_Max
         &&Pitch_Control>=Pit_Rol_Max*Scale_Pecent_Max
           &&Mag_Calibration_Makesure_Cnt>200*5)||cal_flag==0x03)//����5S
             &&Controler_State==Lock_Controler)//����Ϊ����״̬�ſ��Խ���У׼ģʽ
    //���������У׼ģʽ
  {
    Bling_Mode=2;
    Mag_Calibration_Flag=1;//������У׼ģʽ
    Mag_Calibration_Mode=3;
		Bling_Set(&rgb_red,2000,2000,0.2,0,GPIO_PORTF_BASE,GPIO_PIN_1,1);		
    Mag_Calibration_Makesure_Cnt=0;
    Mag_Calibration_All_Finished=0;//ȫ��У׼��ɱ�־λ����
    for(i=0;i<3;i++)
    {
      Mag_Calibration_Finished[i]=0;//��Ӧ���־λ����
      for(j=0;j<36;j++) {Mag_360_Flag[i][j]=0;}
    }
    Page_Number=11;
    Reset_Accel_Calibartion(1);
    Reset_RC_Calibartion(1);
    Forced_Lock_Makesure_Cnt=0;
		cal_flag=0x00;
  }
  
  if(Mag_Calibration_Flag==1)
  {
    if((Throttle_Control<=(1000+Buttom_Safe_Deadband)
       &&Yaw_Control<=-Yaw_Max*Scale_Pecent_Max
         &&Roll_Control==0
           &&Pitch_Control==0) //��һ�����
					 ||cal_step==0x01) 
    {
      Mag_Calibration_Cnt++;
      if(Mag_Calibration_Cnt>=20||cal_step==0x01)
      {
        Mag_Calibration_Mode=0;
        Mag_Is_Okay_Flag[0]=0;//�������ݲɼ���ɱ�־λ��0
        Mag_Is_Okay_Flag[1]=0;//�������ݲɼ���ɱ�־λ��0
        Mag_Is_Okay_Flag[2]=0;//�������ݲɼ���ɱ�־λ��0
        for(i=0;i<36;i++) Mag_360_Flag[0][i]=0;//��ղɼ��Ǳ������ݵ�
        for(i=0;i<36;i++) Mag_360_Flag[1][i]=0;//��ղɼ��Ǳ������ݵ�
        for(i=0;i<36;i++) Mag_360_Flag[2][i]=0;//��ղɼ��Ǳ������ݵ�
        LS_Init(&Mag_LS);//�������м����
        Unlock_Makesure_Cnt=0;
        Lock_Makesure_Cnt=0;
      }
      cal_step=0x00;
    }
    else if(Throttle_Control<=(1000+Buttom_Safe_Deadband)
            &&Yaw_Control>Yaw_Max*Scale_Pecent_Max
              &&Roll_Control==0
                &&Pitch_Control==0) //�ڶ������
    {
      Mag_Calibration_Cnt++;
      if(Mag_Calibration_Cnt>=20)
      {
        Mag_Calibration_Mode=1;
        Mag_Is_Okay_Flag[0]=0;//�������ݲɼ���ɱ�־λ��0
        Mag_Is_Okay_Flag[1]=0;//�������ݲɼ���ɱ�־λ��0
        Mag_Is_Okay_Flag[2]=0;//�������ݲɼ���ɱ�־λ��0
        for(i=0;i<36;i++) Mag_360_Flag[0][i]=0;//��ղɼ��Ǳ������ݵ�
        for(i=0;i<36;i++) Mag_360_Flag[1][i]=0;//��ղɼ��Ǳ������ݵ�
        for(i=0;i<36;i++) Mag_360_Flag[2][i]=0;//��ղɼ��Ǳ������ݵ�
        LS_Init(&Mag_LS);//�������м����
        Unlock_Makesure_Cnt=0;
        Lock_Makesure_Cnt=0;
      }
    }
    else
    {
      Mag_Calibration_Cnt/=2;
    }
    if(Mag_Calibration_Cnt>=200)  Mag_Calibration_Cnt=200;
    
  }
  
}

/***********************************************************
@��������Reset_Mag_Calibartion
@��ڲ�����uint8_t Type
@���ڲ�������
@���������������Ʊ궨���������ǿ�Ƹ�λ
@���ߣ�����С��
@���ڣ�2024/01/20
*************************************************************/
void Reset_Mag_Calibartion(uint8_t Type)
{
  uint16 i=0;
  for(i=0;i<36;i++)
  {
    Mag_360_Flag[0][i]=0;//��ղɼ��ǵ�
    Mag_360_Flag[1][i]=0;//��ղɼ��ǵ�
    Mag_360_Flag[2][i]=0;//��ղɼ��ǵ�
  }
  Mag_Is_Okay_Flag[0]=0;
  Mag_Is_Okay_Flag[1]=0;
  Mag_Is_Okay_Flag[2]=0;
  Mag_Calibration_Mode=3;
  if(Type==1)  Mag_Calibration_Flag=0;
}


uint8_t Check_Plane_Sampling_Okay(uint8_t plane_number)
{
  uint8_t finished_flag=0;
  if(Mag_360_Flag[plane_number][0]&Mag_360_Flag[plane_number][1]&Mag_360_Flag[plane_number][2]
     &Mag_360_Flag[plane_number][3]&Mag_360_Flag[plane_number][4]&Mag_360_Flag[plane_number][5]
       &Mag_360_Flag[plane_number][6]&Mag_360_Flag[plane_number][7]&Mag_360_Flag[plane_number][8]
         &Mag_360_Flag[plane_number][9]&Mag_360_Flag[plane_number][10]&Mag_360_Flag[plane_number][11]
           &Mag_360_Flag[plane_number][12]&Mag_360_Flag[plane_number][13]&Mag_360_Flag[plane_number][14]
             &Mag_360_Flag[plane_number][15]&Mag_360_Flag[plane_number][16]&Mag_360_Flag[plane_number][17]
               &Mag_360_Flag[plane_number][18]&Mag_360_Flag[plane_number][19]&Mag_360_Flag[plane_number][20]
                 &Mag_360_Flag[plane_number][21]&Mag_360_Flag[plane_number][22]&Mag_360_Flag[plane_number][23]
                   &Mag_360_Flag[plane_number][24]&Mag_360_Flag[plane_number][25]&Mag_360_Flag[plane_number][26]
                     &Mag_360_Flag[plane_number][27]&Mag_360_Flag[plane_number][28]&Mag_360_Flag[plane_number][29]
                       &Mag_360_Flag[plane_number][30]&Mag_360_Flag[plane_number][31]&Mag_360_Flag[plane_number][32]
                         &Mag_360_Flag[plane_number][33]&Mag_360_Flag[plane_number][34]&Mag_360_Flag[plane_number][35])
    finished_flag=1;
  return finished_flag;
}

/***********************************************************
@��������Mag_Calibartion
@��ڲ����������ǻ��ֽǶ�ֵ�����������ԭʼֵ
@���ڲ�������
@�������������������ı궨������ң����ֱ�ӽ���
@���ߣ�����С��
@���ڣ�2024/01/20
*************************************************************/
uint8_t Mag_Calibartion(Vector3f *magdata,Vector3f_Body Circle_Angle_Calibartion)
{
  uint16 i=0;
  for(i=0;i<36;i++)
  {
    Last_Mag_360_Flag[0][i]=Mag_360_Flag[0][i];
    Last_Mag_360_Flag[1][i]=Mag_360_Flag[1][i];
    Last_Mag_360_Flag[2][i]=Mag_360_Flag[2][i];
  }
  /********��һ��Z�����������Ϸ���
  ��ʼ����ֱ����ת��Z axis is about 1g,X��Y is about 0g*/
  /********�ڶ���Y�����������Ϸ���
  ��ʼ����ֱ����ת��Y axis is about 1g,X��Z is about 0g*/
  if(Mag_Calibration_Mode<3)//��⵽��Ӧ������
  {
    for(i=0;i<36;i++)
    {
      if(ABS(Circle_Angle_Calibartion.Yaw-Mag_360_define[i])<=5.0f
         &&Mag_Calibration_Mode==0
           &&WP_Sensor.acce_filter[2]>=GRAVITY_RAW/2)//Z�������ֱ
      {
        Mag_360_Flag[0][i]=1;
      }
      
      if(ABS(Circle_Angle_Calibartion.Rol-Mag_360_define[i])<=5.0f
         &&Mag_Calibration_Mode==1
           &&WP_Sensor.acce_filter[1]>=GRAVITY_RAW/2)//Y�������ֱ
      {
        Mag_360_Flag[1][i]=1;
      }
      
      if(ABS(Circle_Angle_Calibartion.Pit-Mag_360_define[i])<=5.0f
         &&Mag_Calibration_Mode==2
           &&WP_Sensor.acce_filter[0]>=GRAVITY_RAW/2)//X�������ֱ
      {
        Mag_360_Flag[2][i]=1;
      }
    }
    if(magdata->x >= Mag.x_max)   Mag.x_max = (int16_t)(magdata->x);
    if(magdata->x <  Mag.x_min)   Mag.x_min = (int16_t)(magdata->x);
    if(magdata->y >= Mag.y_max)   Mag.y_max = (int16_t)(magdata->y);
    if(magdata->y <  Mag.y_min)   Mag.y_min = (int16_t)(magdata->y);
    if(magdata->z >= Mag.z_max)   Mag.z_max = (int16_t)(magdata->z);
    if(magdata->z <  Mag.z_min)   Mag.z_min = (int16_t)(magdata->z);
  }
  if(Check_Plane_Sampling_Okay(0))
  {
    Mag_Is_Okay_Flag[0]=1;//�������ݲɼ���ɱ�־λ��1
    for(i=0;i<36;i++)  Mag_360_Flag[0][i]=0;//��ղɼ��ǵ�
    if(Mag_Is_Okay_Flag[1]==0)//����һ������δ���
      Mag_Calibration_Mode=1;//�Զ�������һ�����ݲɼ�ģʽ
    else Mag_Calibration_Mode=3;//
  }
  
  if(Check_Plane_Sampling_Okay(1))
  {
    Mag_Is_Okay_Flag[1]=1;//�������ݲɼ���ɱ�־λ��1
    for(i=0;i<36;i++) Mag_360_Flag[1][i]=0;//��ղɼ��ǵ�
    if(Mag_Is_Okay_Flag[2]==0)//����һ������δ���
      Mag_Calibration_Mode=2;//�Զ�������һ�����ݲɼ�ģʽ
    else Mag_Calibration_Mode=3;
  }
  
  if(Check_Plane_Sampling_Okay(2))
  {
    Mag_Is_Okay_Flag[2]=1;//�������ݲɼ���ɱ�־λ��1
    for(i=0;i<36;i++) Mag_360_Flag[2][i]=0;//��ղɼ��ǵ�
    if(Mag_Is_Okay_Flag[0]==0)//����һ������δ���
      Mag_Calibration_Mode=0;//�Զ�������һ�����ݲɼ�ģʽ
    else Mag_Calibration_Mode=3;
  }
  
  
  
  if(Mag_Calibration_Mode==0)  Yaw_Correct=Circle_Angle_Calibartion.Yaw;
  else if(Mag_Calibration_Mode==1)  Yaw_Correct=Circle_Angle_Calibartion.Rol;
  else if(Mag_Calibration_Mode==2)  Yaw_Correct=Circle_Angle_Calibartion.Pit;
  else Yaw_Correct=0;
  
  
  
  if(Mag_Is_Okay_Flag[0]==1
     &&Mag_Is_Okay_Flag[1]==1
       &&Mag_Is_Okay_Flag[2]==1)//��������ȫ���ɼ���ϣ�������������
  {
    Mag.x_offset=(Mag.x_min+Mag.x_max)/2.0;
    Mag.y_offset=(Mag.y_min+Mag.y_max)/2.0;
    Mag.z_offset=(Mag.z_min+Mag.z_max)/2.0;
    Mag_Offset_Read.x=Mag.x_offset;
    Mag_Offset_Read.y=Mag.y_offset;
    Mag_Offset_Read.z=Mag.z_offset;
    Mag_Is_Okay_Flag[0]=0;
    Mag_Is_Okay_Flag[1]=0;
    Mag_Is_Okay_Flag[2]=0;
    Mag_Calibration_Flag=0;//������У׼�������ͷ�ң�в���
    Bling_Mode=0;//�ָ�����ָʾģʽ
    
    
		Bling_Set(&rgb_red,4000,2000,0.5,0,GPIO_PORTF_BASE,GPIO_PIN_1,0);  
    //д����ٶ����ƫִ�����������ƫִ 
    WriteFlashParameter_Three(MAG_X_OFFSET,
                              Mag.x_offset,
                              Mag.y_offset,
                              Mag.z_offset);
    NCLink_Send_Check_Flag[9]=0x01;//У׼��Ϻ���У׼����
    return TRUE;
  }
  return FALSE;
}


void Mag_LS_Init()
{
  LS_Init(&Mag_LS);
}
float mag_a,mag_b,mag_c,mag_r;
/***********************************************************
@��������Mag_Calibartion_LS
@��ڲ����������ǻ��ֽǶ�ֵ�����������ԭʼֵ
@���ڲ�������
@������������������С���˷�������桢����ң����ֱ�ӽ���
@���ߣ�����С��
@���ڣ�2024/01/20
*************************************************************/
uint8_t Mag_Calibartion_LS(Vector3f *magdata,Vector3f_Body Circle_Angle_Calibartion)
{
  uint16 i=0;
  for(i=0;i<36;i++)
  {
    Last_Mag_360_Flag[0][i]=Mag_360_Flag[0][i];
    Last_Mag_360_Flag[1][i]=Mag_360_Flag[1][i];
    Last_Mag_360_Flag[2][i]=Mag_360_Flag[2][i];
  }
  
  /********��һ��Z�����������Ϸ���
  ��ʼ����ֱ����ת��Z axis is about 1g,X��Y is about 0g*/
  /********�ڶ���Y�����������Ϸ���
  ��ʼ����ֱ����ת��Y axis is about 1g,X��Z is about 0g*/
  if(Mag_Calibration_Mode<3)//��⵽��Ӧ������
  {
    for(i=0;i<36;i++)
    {
      if(ABS(Circle_Angle_Calibartion.Yaw-Mag_360_define[i])<=5.f
         &&Mag_Calibration_Mode==0
           &&WP_Sensor.accel_raw.z>=GRAVITY_RAW/2)//Z�������ֱ
      {
        Mag_360_Flag[0][i]=1;
      }
      
      if(ABS(Circle_Angle_Calibartion.Rol-Mag_360_define[i])<=5.0f
         &&Mag_Calibration_Mode==1
           &&WP_Sensor.accel_raw.y>=GRAVITY_RAW/2)//Y�������ֱ
      {
        Mag_360_Flag[1][i]=1;
      }
      
      if(ABS(Circle_Angle_Calibartion.Pit-Mag_360_define[i])<=5.0f
         &&Mag_Calibration_Mode==2
           &&WP_Sensor.accel_raw.x>=GRAVITY_RAW/2)//X�������ֱ
      {
        Mag_360_Flag[2][i]=1;
      }
    }
    
    for(i=0;i<36;i++)
    {
      if((Last_Mag_360_Flag[0][i]==0&&Mag_360_Flag[0][i]==1)
         ||(Last_Mag_360_Flag[1][i]==0&&Mag_360_Flag[1][i]==1)
           ||(Last_Mag_360_Flag[2][i]==0&&Mag_360_Flag[2][i]==1))
      {
        LS_Accumulate(&Mag_LS, magdata->x,magdata->y,magdata->z);
        //LS_Calculate(&Mag_LS,36*3,0.0f,&mag_a, &mag_b, &mag_c,&mag_r);
      }
    }
  }
  
  if(Check_Plane_Sampling_Okay(0))
  {
    Mag_Is_Okay_Flag[0]=1;//�������ݲɼ���ɱ�־λ��1
    for(i=0;i<36;i++)  Mag_360_Flag[0][i]=0;//��ղɼ��ǵ�
    if(Mag_Is_Okay_Flag[1]==0)//����һ������δ���
      Mag_Calibration_Mode=1;//�Զ�������һ�����ݲɼ�ģʽ
    else Mag_Calibration_Mode=3;//
  }
  
  if(Check_Plane_Sampling_Okay(1))
  {
    Mag_Is_Okay_Flag[1]=1;//�������ݲɼ���ɱ�־λ��1
    for(i=0;i<36;i++) Mag_360_Flag[1][i]=0;//��ղɼ��ǵ�
    if(Mag_Is_Okay_Flag[2]==0)//����һ������δ���
      Mag_Calibration_Mode=2;//�Զ�������һ�����ݲɼ�ģʽ
    else Mag_Calibration_Mode=3;
  }
  
  if(Check_Plane_Sampling_Okay(2))
  {
    Mag_Is_Okay_Flag[2]=1;//�������ݲɼ���ɱ�־λ��1
    for(i=0;i<36;i++) Mag_360_Flag[2][i]=0;//��ղɼ��ǵ�
    if(Mag_Is_Okay_Flag[0]==0)//����һ������δ���
      Mag_Calibration_Mode=0;//�Զ�������һ�����ݲɼ�ģʽ
    else Mag_Calibration_Mode=3;
  }
  
  if(Mag_Calibration_Mode==0)  			Yaw_Correct=Circle_Angle_Calibartion.Yaw;
  else if(Mag_Calibration_Mode==1)  Yaw_Correct=Circle_Angle_Calibartion.Rol;
  else if(Mag_Calibration_Mode==2)  Yaw_Correct=Circle_Angle_Calibartion.Pit;
  else Yaw_Correct=0;
  
  if(Mag_Is_Okay_Flag[0]==1
     &&Mag_Is_Okay_Flag[1]==1
       &&Mag_Is_Okay_Flag[2]==1)//��������ȫ���ɼ���ϣ�������������
  {
    LS_Calculate(&Mag_LS,36*3,0.0f,&mag_a, &mag_b, &mag_c,&mag_r);
    Mag_Offset_Read.x=mag_a;
    Mag_Offset_Read.y=mag_b;
    Mag_Offset_Read.z=mag_c;
    Mag_Is_Okay_Flag[0]=0;
    Mag_Is_Okay_Flag[1]=0;
    Mag_Is_Okay_Flag[2]=0;
    Mag_Calibration_Flag=0;//������У׼�������ͷ�ң�в���
    Bling_Mode=0;//�ָ�����ָʾģʽ
    
		
		Bling_Set(&rgb_red,4000,2000,0.5,0,GPIO_PORTF_BASE,GPIO_PIN_1,0);		
    WriteFlashParameter_Three(MAG_X_OFFSET,
                              Mag_Offset_Read.x,
                              Mag_Offset_Read.y,
                              Mag_Offset_Read.z);
		NCLink_Send_Check_Flag[9]=0x01;//У׼��Ϻ���У׼����
    return TRUE;
  }
  return FALSE;
}



#define  RC_TOP_DEFAULT       2000
#define  RC_BUTTOM_DEFAULT    1000
#define  RC_MIDDLE_DEFAULT    1500
#define  RC_DEADBAND_DEFAULT  100
#define  RC_DEADBAND_PERCENT   		0.1f		//��λ����ռʵ���г̵İٷֱ�
#define  RC_THR_DEADBAND_PERCENT   0.2f		//������λ����ռʵ���г̵İٷֱ�
#define  RC_RESET_DEFAULT  1500

uint8_t RC_Read_Flag[8];
/***********************************************************
@��������RC_Calibration_Init
@��ڲ�������
@���ڲ�������
@����������ң�����г̱궨��ʼ��
@���ߣ�����С��
@���ڣ�2024/01/20
*************************************************************/
void RC_Calibration_Init()
{
	float _rc_deadband_percent=0;
  uint16_t i=0;
  float max_read[8]={0},min_read[8]={0};
  for(i=0;i<8;i++)
  {
    ReadFlashParameterTwo(RC_CH1_MAX+2*i,&max_read[i],&min_read[i]);
    if(isnan(max_read[i])==0&&isnan(min_read[i])==0)  	
      RC_Read_Flag[i]=0x01;
  }
  
  if(RC_Read_Flag[0]!=0x00
     &&RC_Read_Flag[1]!=0x00
       &&RC_Read_Flag[2]!=0x00
         &&RC_Read_Flag[3]!=0x00
           &&RC_Read_Flag[4]!=0x00
             &&RC_Read_Flag[5]!=0x00
               &&RC_Read_Flag[6]!=0x00
                 &&RC_Read_Flag[7]!=0x00)//flash�д�������
  {
    for(i=0;i<8;i++)
    {
			if(i==RC_THR_CHANNEL) _rc_deadband_percent=RC_THR_DEADBAND_PERCENT;
			else _rc_deadband_percent=RC_DEADBAND_PERCENT;
			
			RC_Data.cal[i].max=max_read[i];
			RC_Data.cal[i].min=min_read[i];
			RC_Data.cal[i].middle=(float)((max_read[i]+min_read[i])/2.0f);

			RC_Data.cal[i].deadband=(float)((max_read[i]-min_read[i])*_rc_deadband_percent/1.0f);
			RC_Data.cal[i].reverse_flag=false;
			RC_Data.cal[i].scale=(RC_Data.cal[i].max-RC_Data.cal[i].min-RC_Data.cal[i].deadband)*0.5f;	
    }
  }
  else//flash�в���������
  {
    for(i=0;i<8;i++)
    {		
			if(i==RC_THR_CHANNEL) _rc_deadband_percent=RC_THR_DEADBAND_PERCENT;
			else _rc_deadband_percent=RC_DEADBAND_PERCENT;
			
      RC_Data.cal[i].max=RC_TOP_DEFAULT;
      RC_Data.cal[i].min=RC_BUTTOM_DEFAULT;
      RC_Data.cal[i].middle=RC_MIDDLE_DEFAULT;
			
			RC_Data.cal[i].deadband=(float)((RC_Data.cal[i].max-RC_Data.cal[i].min)*_rc_deadband_percent/1.0f);;
			RC_Data.cal[i].reverse_flag=false;
			RC_Data.cal[i].scale=(RC_Data.cal[i].max-RC_Data.cal[i].min-RC_Data.cal[i].deadband)*0.5f;
    }
  }
}
/***********************************************************
@��������RC_Calibration_RESET
@��ڲ�������
@���ڲ�������
@����������ң�����г̱궨��λ
@���ߣ�����С��
@���ڣ�2024/01/20
*************************************************************/
void RC_Calibration_RESET()
{
  uint16_t i=0;
  for(i=0;i<8;i++)
  {
    RC_Data.cal[i].max=RC_RESET_DEFAULT;
    RC_Data.cal[i].min=RC_RESET_DEFAULT;
  }
}


uint8_t RC_Calibration_Trigger_Flag=0;
/***********************************************************
@��������RC_Calibration_Trigger
@��ڲ�������
@���ڲ�������
@����������ң�����г̱궨����
@���ߣ�����С��
@���ڣ�2024/01/20
*************************************************************/
void RC_Calibration_Trigger(void)
{
  if(QuadKey2==0)
  {
    delay_ms(500);
    if(QuadKey2==0)
    {
      while(QuadKey2==0);
      RC_Calibration_RESET();//��λң�����г�ֵ���ȴ�У׼���
      RC_Calibration_Trigger_Flag=1;
      Page_Number=14;
      Key_Right_Release=1;
    }
  }
  else
  {
    RC_Calibration_Init();//ֱ�Ӵ�flash���棨����DEFAULTֵ����ȡң�����г����
    RC_Calibration_Trigger_Flag=0;
  }
}

/***********************************************************
@��������RC_Calibration_Check
@��ڲ�����uint16 *rc_date
@���ڲ�������
@����������ң�����г�У׼��⣬�漰�ڲ�Flash����д����
@���ߣ�����С��
@���ڣ�2024/01/20
*************************************************************/
bool RC_Calibration_Check(uint16 *rc_date)
{
  uint16_t i=0;
  bool success_flag=FALSE;
	
  if(cal_flag==0x05)
	{
		RC_Calibration_RESET();//��λң�����г�ֵ���ȴ�У׼���
		RC_Calibration_Trigger_Flag=1;
		Page_Number=14;
		Key_Right_Release=1;
		cal_flag=0x00;
	}
	
  if(RC_Calibration_Trigger_Flag==0) return success_flag;
  for(i=0;i<8;i++)
  {
    if(rc_date[i] >= RC_Data.cal[i].max)   RC_Data.cal[i].max = rc_date[i];//����г�ֵ
    if(rc_date[i] <  RC_Data.cal[i].min)   RC_Data.cal[i].min = rc_date[i];//��С�г�ֵ
    RC_Data.cal[i].middle=(float)((RC_Data.cal[i].max+RC_Data.cal[i].min)/2.0f);//�г���λ
    RC_Data.cal[i].deadband=(float)((RC_Data.cal[i].max-RC_Data.cal[i].min)*RC_DEADBAND_PERCENT/1.0f);//���������̵İٷ�֮RC_DEADBAND_PERCENTΪ��λ����
  }
  
	if(cal_flag==0x06)
	{
		RC_Calibration_Trigger_Flag=0;
		Key_Right_Release=0;
		success_flag=TRUE;
		for(i=0;i<8;i=i+1)
		{
			WriteFlashParameter_Two(RC_CH1_MAX+2*i,
															RC_Data.cal[i].max,
															RC_Data.cal[i].min);
		}
	  cal_flag=0x00;
	}
	
	
  if(QuadKey2==0)//ң�����궨��ɺ�ͨ�������������궨����
  {
    delay_ms(2000);
    if(QuadKey2==0)
    {
      while(QuadKey2==0);
      RC_Calibration_Trigger_Flag=0;
      Key_Right_Release=0;
      success_flag=TRUE;
      for(i=0;i<8;i=i+1)
      {
        WriteFlashParameter_Two(RC_CH1_MAX+2*i,
                                RC_Data.cal[i].max,
                                RC_Data.cal[i].min);
      }
    }
  }
  return success_flag;
}

/***********************************************************
@��������Reset_RC_Calibartion
@��ڲ�����uint8_t Type
@���ڲ�������
@����������ң�����г�У׼ǿ�Ƹ�λ
@���ߣ�����С��
@���ڣ�2024/01/20
*************************************************************/
void Reset_RC_Calibartion(uint8_t Type)
{
  if(Type==1)  
  {
    RC_Calibration_Trigger_Flag=0;
    Key_Right_Release=0;
  }
}
uint16_t ESC_Calibration_Makesure_Cnt=0;
float ESC_Calibration_Flag=0;
void ESC_Calibration_Check(void)
{
  if(Throttle_Control<=(1000+Buttom_Safe_Deadband)
     &&Yaw_Control>=Yaw_Max*Scale_Pecent_Max
       &&Roll_Control>=Pit_Rol_Max*Scale_Pecent_Max
         &&Pitch_Control<=-Pit_Rol_Max*Scale_Pecent_Max)
    ESC_Calibration_Makesure_Cnt++;
  //else ESC_Calibration_Makesure_Cnt/=2;
  
  if(Throttle_Control<=(1000+Buttom_Safe_Deadband)
     &&Yaw_Control>=Yaw_Max*Scale_Pecent_Max
       &&Roll_Control>=Pit_Rol_Max*Scale_Pecent_Max
         &&Pitch_Control<=-Pit_Rol_Max*Scale_Pecent_Max
           &&ESC_Calibration_Makesure_Cnt>200*5//����5S
             &&Controler_State==Lock_Controler)//����Ϊ����״̬�ſ��Խ���У׼ģʽ
    //����ESCУ׼ģʽ
  {
    ESC_Calibration_Flag=1;
    ESC_Calibration_Makesure_Cnt=0;
    Forced_Lock_Makesure_Cnt=0;
    WriteFlashParameter(ESC_CALIBRATION_FLAG,
                        ESC_Calibration_Flag);
    Bling_Set(&rgb_red,5000,500,0.2,0,GPIO_PORTF_BASE,GPIO_PIN_1,0);	
    Page_Number=15;
  }
}

#define Thr_Chl_Num  2
/***********************************************************
@��������ESC_Calibration
@��ڲ�������
@���ڲ�������
@��������������г�У׼
@���ߣ�����С��
@���ڣ�2024/01/20
*************************************************************/
void ESC_Calibration(void)
{
  PWM_Output((uint16_t)(Receiver_PPM_Databuf[Thr_Chl_Num]),
						 (uint16_t)(Receiver_PPM_Databuf[Thr_Chl_Num]),
						 (uint16_t)(Receiver_PPM_Databuf[Thr_Chl_Num]),
						 (uint16_t)(Receiver_PPM_Databuf[Thr_Chl_Num]));
}

/***********************************************************
@��������Check_Calibration_Flag
@��ڲ�������
@���ڲ�������
@������������ǰУ׼ģʽ���
@���ߣ�����С��
@���ڣ�2024/01/20
*************************************************************/
uint8_t Check_Calibration_Flag(void)
{
  uint8_t cal_flag=0x00; 
  if(Key_Right_Release==1)      cal_flag|=0x01;//ң����У׼
  if(Accel_Calibration_Flag==1) cal_flag|=0x02;//���ٶȼ�У׼
  if(Mag_Calibration_Flag==1)   cal_flag|=0x04;//������У׼
  return cal_flag;
}

float Pitch_Offset=0,Roll_Offset=0;
float Hor_Accel_Offset[3]={0};
uint16_t Horizontal_Calibration_Makesure_Cnt=0;
/***********************************************************
@��������Horizontal_Calibration_Check
@��ڲ�������
@���ڲ�������
@��������������ˮƽУ׼
@���ߣ�����С��
@���ڣ�2024/01/20
*************************************************************/
void Horizontal_Calibration_Check(void)
{
  float acce_sample_sum[3]={0,0,0};//���ٶȺ�����
  if(Throttle_Control<=(1000+Buttom_Safe_Deadband)&&Yaw_Control>=Yaw_Max*Scale_Pecent_Max&&Roll_Control<=-Pit_Rol_Max*Scale_Pecent_Max&&Pitch_Control<=-Pit_Rol_Max*Scale_Pecent_Max)
  {
    Horizontal_Calibration_Makesure_Cnt++;
  }
  if(((Throttle_Control<=(1000+Buttom_Safe_Deadband)
     &&Yaw_Control>=Yaw_Max*Scale_Pecent_Max
       &&Roll_Control<=-Pit_Rol_Max*Scale_Pecent_Max
         &&Pitch_Control<=-Pit_Rol_Max*Scale_Pecent_Max
           &&Horizontal_Calibration_Makesure_Cnt>=50*3)||cal_flag==0x04)//����3��
             &&Controler_State==Lock_Controler)//����Ϊ����״̬�ſ��Խ���У׼ģʽ
  {
		delay_ms(2000);
    Pitch_Offset=WP_AHRS.Pitch;
    Roll_Offset=WP_AHRS.Roll;
    uint16_t num_samples=0;
    while(num_samples<200)//����200��
    {
      if(Gyro_Length<=20.0f)//ͨ��������ģ����ȷ�����徲ֹ
      {
        for(int16_t j=0;j<3;j++){
          acce_sample_sum[j]+=WP_Sensor.acce_filter[j];//���ٶȼ�ת��Ϊ1g������
        }
        delay_ms(5);//���10ms��1s������ȡƽ��
        num_samples++;
      }
    }
    Hor_Accel_Offset[0]=(acce_sample_sum[0]/num_samples)*RAW_TO_G; //�����Ӧ��ļ��ٶȼ���
    Hor_Accel_Offset[1]=(acce_sample_sum[1]/num_samples)*RAW_TO_G; //�����Ӧ��ļ��ٶȼ���
    Hor_Accel_Offset[2]=(acce_sample_sum[2]/num_samples-GRAVITY_RAW)*RAW_TO_G;//�����Ӧ��ļ��ٶȼ���
	    
    Horizontal_Calibration_Makesure_Cnt=0;
		WriteFlashParameter(HOR_CAL_ACCEL_X,Hor_Accel_Offset[0]);
		WriteFlashParameter(HOR_CAL_ACCEL_Y,Hor_Accel_Offset[1]);
		WriteFlashParameter(HOR_CAL_ACCEL_Z,Hor_Accel_Offset[2]);
		
		
    WriteFlashParameter(PITCH_OFFSET1,Pitch_Offset);
    WriteFlashParameter(ROLL_OFFSET1,Roll_Offset);
			
		Parameter_Init();	
		
    Bling_Set(&rgb_red,500,100,0.2,0,GPIO_PORTF_BASE,GPIO_PIN_1,0);
	  NCLink_Send_Check_Flag[9]=0x01;
		cal_flag=0x00;
  }
}

/***********************************************************
@��������Horizontal_Calibration_Init
@��ڲ�������
@���ڲ�������
@��������������ˮƽУ׼��ʼ��
@���ߣ�����С��
@���ڣ�2024/01/20
*************************************************************/
void Horizontal_Calibration_Init(void)
{
  float pitch_offset_temp=0,roll_offset_temp=0;	
  ReadFlashParameterOne(PITCH_OFFSET1,&pitch_offset_temp);
  ReadFlashParameterOne(ROLL_OFFSET1,&roll_offset_temp);
  
  if(isnan(pitch_offset_temp)==0)   Pitch_Offset=pitch_offset_temp;
  if(isnan(roll_offset_temp)==0)    Roll_Offset=roll_offset_temp;
}


uint16_t Headless_Mode_Makesure_Cnt=0;
float Headless_Mode_Yaw=0.0f;
/***********************************************************
@��������Headless_Mode_Calibration_Check
@��ڲ�������
@���ڲ�������
@������������ͷģʽ��ʼƫ���ǻ�ȡ
@���ߣ�����С��
@���ڣ�2024/01/20
*************************************************************/
void Headless_Mode_Calibration_Check(void)
{
  if(Throttle_Control<=(1000+Buttom_Safe_Deadband)
     &&Yaw_Control>=Yaw_Max*Scale_Pecent_Max
       &&Roll_Control<=-Pit_Rol_Max*Scale_Pecent_Max
         &&Pitch_Control==0)
  {
    Headless_Mode_Makesure_Cnt++;
  }
  if(Throttle_Control<=(1000+Buttom_Safe_Deadband)
     &&Yaw_Control>=Yaw_Max*Scale_Pecent_Max
       &&Roll_Control<=-Pit_Rol_Max*Scale_Pecent_Max
         &&Pitch_Control==0
           &&Headless_Mode_Makesure_Cnt>=200*5//��������
             &&Controler_State==Lock_Controler)//����Ϊ����״̬�ſ��Խ���У׼ģʽ
  {    
    Headless_Mode_Yaw=WP_AHRS.Yaw;
    Horizontal_Calibration_Makesure_Cnt=0;
    WriteFlashParameter(HEADLESS_MODE_YAW,Headless_Mode_Yaw);
    Bling_Set(&rgb_red,500,100,0.2,0,GPIO_PORTF_BASE,GPIO_PIN_1,0);
  }
}


uint8_t Gyro_Safety_Calibration_Flag=0;
uint16_t Gyro_Calibration_Makesure_Cnt=0;
void Gyro_Calibration_Check(vector3f *gyro)
{	
	if(Controler_State==Unlock_Controler||Page_Number!=0) return;//����״̬��+��ʾ�����ڵ�һҳ������У׼
	
	if(Throttle_Control<=(1000+Buttom_Safe_Deadband)
	 &&Yaw_Control>=Yaw_Max*Scale_Pecent_Max
	 &&Roll_Control==0
	 &&Pitch_Control<=-Scale_Pecent_Max*Pit_Rol_Max)
  {
    Gyro_Calibration_Makesure_Cnt++;
  }
	
	if(((Throttle_Control<=(1000+Buttom_Safe_Deadband)
	 &&Yaw_Control>=Yaw_Max*Scale_Pecent_Max
	 &&Roll_Control==0
	 &&Pitch_Control<=-Scale_Pecent_Max*Pit_Rol_Max           
	 &&Gyro_Calibration_Makesure_Cnt>=200*3)||cal_flag==0x01)//����3��
   &&Controler_State==Lock_Controler)//����Ϊ����״̬�ſ��Խ���У׼ģʽ
  {
		Gyro_Calibration_Makesure_Cnt=0;
		Gyro_Safety_Calibration_Flag=0;
		cal_flag=0x00;
  }	

	
	if(Gyro_Safety_Calibration_Flag==1)  return;
	
	
	
	static vector3f offset;
	static uint16_t offset_cnt=0;
  static vector3f last_gyro;
	const float scale=GYRO_CALIBRATION_COFF;
   if(ABS(gyro->x-last_gyro.x)*scale<=10.0f
		   &&ABS(gyro->y-last_gyro.y)*scale<=10.0f
	      &&ABS(gyro->z-last_gyro.z)*scale<=10.0f
	       &&Temperature_Stable_Flag==1)
	{
	  offset.x+=gyro->x;
		offset.y+=gyro->y;
		offset.z+=gyro->z;
		offset_cnt++;
	}
  else
	{
		offset.x=0;
		offset.y=0;
		offset.z=0;
		offset_cnt=0;
	}
  last_gyro.x=gyro->x;
	last_gyro.y=gyro->y;
	last_gyro.z=gyro->z;
	
	if(offset_cnt>=400)//��������2s
	{
		gyro_offset.x =(offset.x/offset_cnt);//�õ��궨ƫ��
		gyro_offset.y =(offset.y/offset_cnt);
		gyro_offset.z =(offset.z/offset_cnt);
		Euler_Angle_Init_Again();
		WriteFlashParameter_Three(GYRO_X_OFFSET,
															gyro_offset.x,
															gyro_offset.y,
															gyro_offset.z);		
		Bling_Set(&rgb_red,2000,100,0.2,0,GPIO_PORTF_BASE,GPIO_PIN_1,0);
		
		
		offset.x=0;
		offset.y=0;
		offset.z=0;
		offset_cnt=0;
		
		NCLink_Send_Check_Flag[9]=0x01;
		Gyro_Safety_Calibration_Flag=1;
		
		buzzer_setup(500,0.25f,3);
	}
}


//У׼����������ܺ���
void Calibration_All_Check(void)
{
	if(Controler_State==Unlock_Controler) return;//����״̬�²�����У׼
	Accel_Calibration_Check();			  //���ٶ�У׼���
  Mag_Calibration_Check();				  //������У׼���
  ESC_Calibration_Check();				  //����г�У׼���
}




void Reserved_Params_Init(void)
{
  float _param_value[RESERVED_PARAM_NUM];
	for(uint16_t i=0;i<RESERVED_PARAM_NUM;i++)
	{
		ReadFlashParameterOne(RESERVED_PARAM+i,&_param_value[i]);
		if(isnan(_param_value[i])==0) param_value[i]=_param_value[i];
		else param_value[i]=0;
	}
}




uint16_t Target_Height=0,Safe_Height=0,Safe_Vbat=0,Max_Height=0,Max_Radius=0,Max_Upvel=0,Max_Downvel=0,Max_Horvel=0,Reserved_Uart=0,Near_Ground_Height=0;
uint16_t Uart2_Mode=0,Avoid_Obstacle=0;
void Other_Parameter_Init(void)
{
	float targeheight,safeheight,safevbat,maxheight,maxradius,maxupvel,maxdownvel,maxhorvel,reserveduart,neargroundheight;
	float uart2mode,avoidobstacle;
	ReadFlashParameterOne(TARGET_HEIGHT,&targeheight);
	ReadFlashParameterOne(SAFE_HEIGHT,&safeheight);
	ReadFlashParameterOne(SAFE_VBAT,&safevbat);
	ReadFlashParameterOne(MAX_HEIGHT,&maxheight);
	ReadFlashParameterOne(MAX_RADIUS,&maxradius);
	ReadFlashParameterOne(MAX_UPVEL,&maxupvel);
	ReadFlashParameterOne(MAX_DOWNVEL,&maxdownvel);
	ReadFlashParameterOne(MAX_HORVEL,&maxhorvel);
 	ReadFlashParameterOne(RESERVED_UART_FUNCTION,&reserveduart);
	ReadFlashParameterOne(NEAR_GROUND_HEIGHT,&neargroundheight);
 	ReadFlashParameterOne(UART2_FUNCTION,&uart2mode);
	ReadFlashParameterOne(AVOID_OBSTACLE,&avoidobstacle);	


	
	if(isnan(targeheight)==0)   Target_Height=targeheight;
	else Target_Height=Auto_Launch_Target;
	
	if(isnan(safeheight)==0)    Safe_Height=safeheight;
	else Safe_Height=Nav_Safety_Height;
	
	if(isnan(safevbat)==0)      Safe_Vbat=safevbat;
	else Safe_Vbat=Flight_Safe_Vbat;//
	
	if(isnan(maxheight)==0)     Max_Height=maxheight;
	else Max_Height=Flight_Max_Height;//
	
	if(isnan(maxradius)==0)     Max_Radius=maxradius;
	else Max_Radius=Flight_Max_Radius;//
	
	if(isnan(maxupvel)==0)      Max_Upvel=maxupvel;
	else Max_Upvel=Climb_Up_Speed_Max;
	
	if(isnan(maxdownvel)==0)    Max_Downvel=maxdownvel;
	else Max_Downvel=Climb_Down_Speed_Max;
	
	if(isnan(maxhorvel)==0)     Max_Horvel=maxhorvel;
	else Max_Horvel=Nav_Speed_Max;

	if(isnan(reserveduart)==0)    Reserved_Uart=reserveduart;
	else Reserved_Uart=RESERVED_UART_DEFAULT;
	
	if(isnan(neargroundheight)==0)     Near_Ground_Height=neargroundheight;
	else Near_Ground_Height=Nav_Near_Ground_Height_Default;

	if(isnan(uart2mode)==0)    Uart2_Mode=uart2mode;
	else Uart2_Mode=UART2_DEFAULT;
	
	if(isnan(avoidobstacle)==0)     Avoid_Obstacle=avoidobstacle;
	else Avoid_Obstacle=AVOID_OBSTACLE_DEFAULT;
	

	Reserved_Params_Init();
}

void Other_Parameter_Default(void)
{
	Target_Height=Auto_Launch_Target;
	Safe_Height=Nav_Safety_Height;
	Safe_Vbat=Flight_Safe_Vbat;
	Max_Height=Flight_Max_Height;
	Max_Radius=Flight_Max_Radius;
	Max_Upvel=Climb_Up_Speed_Max;
	Max_Downvel=Climb_Down_Speed_Max;
	Max_Horvel=Nav_Speed_Max;
	Reserved_Uart=RESERVED_UART_DEFAULT;
	Near_Ground_Height=Nav_Near_Ground_Height_Default;
	Uart2_Mode=UART2_DEFAULT;
	Avoid_Obstacle=AVOID_OBSTACLE_DEFAULT;
	
	WriteFlashParameter(TARGET_HEIGHT,Target_Height);
	WriteFlashParameter(SAFE_HEIGHT,Safe_Height);
	WriteFlashParameter(SAFE_VBAT,Safe_Vbat);
	WriteFlashParameter(MAX_HEIGHT,Max_Height);
	WriteFlashParameter(MAX_RADIUS,Max_Radius);
	WriteFlashParameter(MAX_UPVEL,Max_Upvel);
	WriteFlashParameter(MAX_DOWNVEL,Max_Downvel);
	WriteFlashParameter(MAX_HORVEL,Max_Horvel);
	WriteFlashParameter(RESERVED_UART_FUNCTION,Reserved_Uart);
	WriteFlashParameter(NEAR_GROUND_HEIGHT,Near_Ground_Height);
	WriteFlashParameter(UART2_FUNCTION,Uart2_Mode);
	WriteFlashParameter(AVOID_OBSTACLE,Avoid_Obstacle);
}


