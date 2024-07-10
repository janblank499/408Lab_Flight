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
#include "myiic.h"
#include "ICM2068X.h"

lpf_param ins_lpf_param,accel_lpf_param,gyro_lpf_param,cal_lpf_param,accel_fb_lpf_param; 

IMU_ID_READ IMU_ID=WHO_AM_I_MPU6050;
uint8_t imu_address=ICM20689_ADRESS;
uint8_t icm_read_register[5]={0x00,0x02,0x10,0x10,0x03};
uint8_t icm_read_check[5]={0};
/***********************************************************
@��������ICM20689_Init
@��ڲ�������
@���ڲ�������
����������MPU6050��ʼ��
@���ߣ�����С��
@���ڣ�2024/01/20
*************************************************************/
uint8_t ICM20689_Init(void)//ICM20689��ʼ��
{
	uint8_t fault=0;
	i2c1Write(imu_address,PWR_MGMT_1, 0x81);//���ǿ�Ƹ�λ81
	delay_ms(100);	
	IMU_ID=(IMU_ID_READ)(Single_ReadI2C(imu_address,WHO_AM_I));
	switch(IMU_ID)
	{
		case WHO_AM_I_ICM20608D:
		case WHO_AM_I_ICM20608G:			
		{
			i2c1Write(imu_address,PWR_MGMT_1,0X80);	//��λICM20608
			delay_ms(100);
			i2c1Write(imu_address,PWR_MGMT_1, 0X01);	//����ICM20608
			for(uint8_t i=0;i<3;i++)
			{
				i2c1Write(imu_address,0x19, icm_read_register[0]);   /* ����������ڲ������� */
				i2c1Write(imu_address,0x1A, icm_read_register[1]);   /* �����ǵ�ͨ�˲�BW=92Hz */
				i2c1Write(imu_address,0x1B, icm_read_register[2]);   /* �����ǡ�500dps���� */
				i2c1Write(imu_address,0x1C, icm_read_register[3]);   /* ���ٶȼơ�4G���� */
				i2c1Write(imu_address,0x1D, icm_read_register[4]);   /* ���ٶȼƵ�ͨ�˲�BW=21.2Hz */
				delay_ms(10);
			}		
																												 //0x00���������ǡ��¶��ڲ���ͨ�˲�Ƶ�ʷ�Χ��������250hz����������306.6hz���¶�4000hz
																												 //0x01���������ǡ��¶��ڲ���ͨ�˲�Ƶ�ʷ�Χ��������176hz����������177hz���¶�188hz
																												 //0x02���������ǡ��¶��ڲ���ͨ�˲�Ƶ�ʷ�Χ��������92hz����������108.6hz���¶�98hz
																												 //0x03���������ǡ��¶��ڲ���ͨ�˲�Ƶ�ʷ�Χ��������41hz����������59hz���¶�42hz		
																												 //0x04���������ǡ��¶��ڲ���ͨ�˲�Ƶ�ʷ�Χ��������20hz����������30.5hz���¶�20hz

																												 //0x00���ü��ٶȼ��ڲ���ͨ�˲�Ƶ�ʷ�Χ�����ٶ�218.1hz����������235hz		
																												 //0x01���ü��ٶȼ��ڲ���ͨ�˲�Ƶ�ʷ�Χ�����ٶ�218.1hz����������235hz
																												 //0x02���ü��ٶȼ��ڲ���ͨ�˲�Ƶ�ʷ�Χ�����ٶ�99.0hz����������121.3hz		
																												 //0x03���ü��ٶȼ��ڲ���ͨ�˲�Ƶ�ʷ�Χ�����ٶ�44.8hz����������61.5hz
																												 //0x04���ü��ٶȼ��ڲ���ͨ�˲�Ƶ�ʷ�Χ�����ٶ�21.2hz����������31.0hz
																												 //0x05���ü��ٶȼ��ڲ���ͨ�˲�Ƶ�ʷ�Χ�����ٶ�10.2hz����������15.5hz	
			i2c1Write(imu_address,0x6C, 0x00);   /* �򿪼��ٶȼƺ������������� */
			i2c1Write(imu_address,0x1E, 0x00);   /* �رյ͹��� */
			i2c1Write(imu_address,0x23, 0x00);   /* �ر�FIFO */ 	

			delay_ms(100);	
			icm_read_check[0]=i2c1Read(imu_address,0x19);
			icm_read_check[1]=i2c1Read(imu_address,0x1A);
			icm_read_check[2]=i2c1Read(imu_address,0x1B);
			icm_read_check[3]=i2c1Read(imu_address,0x1C);
			icm_read_check[4]=i2c1Read(imu_address,0x1D);
			for(uint8_t i=0;i<5;i++)
			{
				if(icm_read_check[i]!=icm_read_register[i]) fault=1;
			}
		}
		break;
		default:
		{
			fault=1;
		}			
	}

	float tmp_player_level=0;
	ReadFlashParameterOne(DRONE_PLAYER_LEVEL ,&tmp_player_level);
  if(isnan(tmp_player_level)==0)  WP_AHRS.player_level=(uint8_t)(tmp_player_level);
	else WP_AHRS.player_level=player_level_default;
	
	float tmp_gyro_lpf_cf,tmp_accel_lpf_cf,tmp_ins_lpf_param_cf,tmp_accel_fb_lpf_param;
	if(WP_AHRS.player_level==0)//ͨ�û���
	{
		ReadFlashParameterOne(GYRO_LPF_CF ,&tmp_gyro_lpf_cf);
		if(isnan(tmp_gyro_lpf_cf)==0)  gyro_lpf_param.cf=tmp_gyro_lpf_cf;
		else gyro_lpf_param.cf=gyro_lpf_param_default1;
		
		ReadFlashParameterOne(ACCEL_LPF_CF,&tmp_accel_lpf_cf);
		if(isnan(tmp_accel_lpf_cf)==0) accel_lpf_param.cf=tmp_accel_lpf_cf;
		else accel_lpf_param.cf=accel_lpf_param_default1;
		
		ReadFlashParameterOne(INS_LPF_CF,&tmp_ins_lpf_param_cf);
		if(isnan(tmp_ins_lpf_param_cf)==0) ins_lpf_param.cf=tmp_ins_lpf_param_cf;
		else ins_lpf_param.cf=ins_lpf_param_default1;
		
		ReadFlashParameterOne(FB_LPF_CF,&tmp_accel_fb_lpf_param);
		if(isnan(tmp_accel_fb_lpf_param)==0) accel_fb_lpf_param.cf=tmp_accel_fb_lpf_param;
		else accel_fb_lpf_param.cf=accel_fb_lpf_param_default1;
	}
	else if(WP_AHRS.player_level==1)
	{
		ReadFlashParameterOne(GYRO_LPF_CF ,&tmp_gyro_lpf_cf);
		if(isnan(tmp_gyro_lpf_cf)==0)  gyro_lpf_param.cf=tmp_gyro_lpf_cf;
		else gyro_lpf_param.cf=gyro_lpf_param_default2;
		
		ReadFlashParameterOne(ACCEL_LPF_CF,&tmp_accel_lpf_cf);
		if(isnan(tmp_accel_lpf_cf)==0) accel_lpf_param.cf=tmp_accel_lpf_cf;
		else accel_lpf_param.cf=accel_lpf_param_default2;
		
		ReadFlashParameterOne(INS_LPF_CF,&tmp_ins_lpf_param_cf);
		if(isnan(tmp_ins_lpf_param_cf)==0) ins_lpf_param.cf=tmp_ins_lpf_param_cf;
		else ins_lpf_param.cf=ins_lpf_param_default2;
		
		ReadFlashParameterOne(FB_LPF_CF,&tmp_accel_fb_lpf_param);
		if(isnan(tmp_accel_fb_lpf_param)==0) accel_fb_lpf_param.cf=tmp_accel_fb_lpf_param;
		else accel_fb_lpf_param.cf=accel_fb_lpf_param_default2;		
	}
	else
	{
		ReadFlashParameterOne(GYRO_LPF_CF ,&tmp_gyro_lpf_cf);
		if(isnan(tmp_gyro_lpf_cf)==0)  gyro_lpf_param.cf=tmp_gyro_lpf_cf;
		else gyro_lpf_param.cf=gyro_lpf_param_default1;
		
		ReadFlashParameterOne(ACCEL_LPF_CF,&tmp_accel_lpf_cf);
		if(isnan(tmp_accel_lpf_cf)==0) accel_lpf_param.cf=tmp_accel_lpf_cf;
		else accel_lpf_param.cf=accel_lpf_param_default1;
		
		ReadFlashParameterOne(INS_LPF_CF,&tmp_ins_lpf_param_cf);
		if(isnan(tmp_ins_lpf_param_cf)==0) ins_lpf_param.cf=tmp_ins_lpf_param_cf;
		else ins_lpf_param.cf=ins_lpf_param_default1;
		
		ReadFlashParameterOne(FB_LPF_CF,&tmp_accel_fb_lpf_param);
		if(isnan(tmp_accel_fb_lpf_param)==0) accel_fb_lpf_param.cf=tmp_accel_fb_lpf_param;
		else accel_fb_lpf_param.cf=accel_fb_lpf_param_default1;	
	}
  delay_ms(500);	
  IMU_Calibration();
  set_cutoff_frequency(imu_sampling_hz	,gyro_lpf_param.cf    	,&gyro_lpf_param);        //��̬���ٶȷ����˲�����  95
  set_cutoff_frequency(imu_sampling_hz	,accel_lpf_param.cf   	,&accel_lpf_param);       //��̬����Ӽ������˲�ֵ30
  set_cutoff_frequency(imu_sampling_hz	,ins_lpf_param.cf     	,&ins_lpf_param);					//INS�Ӽ��˲�ֵ 60
	set_cutoff_frequency(imu_sampling_hz	,accel_fb_lpf_param.cf	,&accel_fb_lpf_param);    //INS�Ӽ��˲�ֵ 5
	set_cutoff_frequency(imu_sampling_hz	,2    									,&cal_lpf_param);		      //������У׼�Ӽ��˲�ֵ 2
	
	return fault;
}

/***********************************************************
@��������ICM20689_Read_Data
@��ڲ�����vector3f *gyro,vector3f *accel
@���ڲ�������
����������MPU6050���ݲɼ�
@���ߣ�����С��
@���ڣ�2024/01/20
*************************************************************/
void ICM20689_Read_Data(vector3f *gyro,vector3f *accel,float *temperature)//��ȡMPU6050����
{
	uint8_t buf[14];
	int16_t temp;
	i2c1ReadData(imu_address,ACCEL_XOUT_H,buf,14);
	switch(IMU_ID)
	{
		case WHO_AM_I_MPU6050:
		{
			accel->x=(int16_t)((buf[0]<<8)|buf[1]);
			accel->y=-(int16_t)((buf[2]<<8)|buf[3]);
			accel->z=-(int16_t)((buf[4]<<8)|buf[5]);	
			temp		=(int16_t)((buf[6]<<8)|buf[7]);
			gyro->x	=(int16_t)((buf[8]<<8)|buf[9]);
			gyro->y	=-(int16_t)((buf[10]<<8)|buf[11]);
			gyro->z	=-(int16_t)((buf[12]<<8)|buf[13]);	
			*temperature=36.53f+(float)(temp/340.0f);	
		}
		break;
		case WHO_AM_I_ICM20689:
		{
			accel->x=(int16_t)((buf[0]<<8)|buf[1]);
			accel->y=-(int16_t)((buf[2]<<8)|buf[3]);
			accel->z=-(int16_t)((buf[4]<<8)|buf[5]);	
			temp		=(int16_t)((buf[6]<<8)|buf[7]);
			gyro->x	=(int16_t)((buf[8]<<8)|buf[9]);
			gyro->y	=-(int16_t)((buf[10]<<8)|buf[11]);
			gyro->z	=-(int16_t)((buf[12]<<8)|buf[13]);	
			*temperature=25.0f+(double)((temp-25.0f)/326.8f);	
		}
		break;	
		case WHO_AM_I_ICM20608D:
		case WHO_AM_I_ICM20608G:
		case WHO_AM_I_ICM20602:				
		{
			accel->y=(int16_t)((buf[0]<<8)|buf[1]);
			accel->x=(int16_t)((buf[2]<<8)|buf[3]);
			accel->z=-(int16_t)((buf[4]<<8)|buf[5]);	
			temp		=(int16_t)((buf[6]<<8)|buf[7]);
			gyro->y	=(int16_t)((buf[8]<<8)|buf[9]);
			gyro->x	=(int16_t)((buf[10]<<8)|buf[11]);
			gyro->z	=-(int16_t)((buf[12]<<8)|buf[13]);	
			
			*temperature=25.0f+(double)((temp-25.0f)/326.8f);		
		}
		break;
		default:
		{
			accel->x=(int16_t)((buf[0]<<8)|buf[1]);
			accel->y=-(int16_t)((buf[2]<<8)|buf[3]);
			accel->z=-(int16_t)((buf[4]<<8)|buf[5]);	
			temp		=(int16_t)((buf[6]<<8)|buf[7]);
			gyro->x	=(int16_t)((buf[8]<<8)|buf[9]);
			gyro->y	=-(int16_t)((buf[10]<<8)|buf[11]);
			gyro->z	=-(int16_t)((buf[12]<<8)|buf[13]);	
			*temperature=36.53f+(float)(temp/340.0f);				
		}			
	}
	
}


Vector3f gyro_offset;
/***********************************************************
@��������IMU_Calibration
@��ڲ�������
@���ڲ�������
���������������ǿ�����ƫ�궨
@���ߣ�����С��
@���ڣ�2024/01/20
*************************************************************/
void IMU_Calibration(void)
{
	vector3f gyro_offset_temp;
	ReadFlashParameterOne(GYRO_X_OFFSET,&gyro_offset_temp.x);
	ReadFlashParameterOne(GYRO_Y_OFFSET,&gyro_offset_temp.y);
	ReadFlashParameterOne(GYRO_Z_OFFSET,&gyro_offset_temp.z);	
	if(isnan(gyro_offset_temp.x)==0
		&&isnan(gyro_offset_temp.y)==0
		 &&isnan(gyro_offset_temp.z)==0)//���֮ǰ�Ѿ��¶�У׼��������ʱֱ����֮ǰУ׼������ 
	{
		 gyro_offset.x=gyro_offset_temp.x;
		 gyro_offset.y=gyro_offset_temp.y;
		 gyro_offset.z=gyro_offset_temp.z;
	}
	
	Gyro_Safety_Calibration_Flag=0;//���¶��ȶ����Զ�У׼��������ƫ
}











//	0x00,//PWR_MGMT_1    00
//	0x00,//SMPLRT_DIV	   00
//	0x00,//MPU_CONFIG    02
//	0x08,//GYRO_CONFIG   08
//	0x08,//ACCEL_CONFIG  08
//	0x00//ACCEL_CONFIG2  03
