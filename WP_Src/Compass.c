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
#include "Compass.h"


#define MAG_EXTERNAL_IST8310 0
#define MAG_EXTERNAL_QMC5883 1
#define MAG_EXTERNAL_TYPE    MAG_EXTERNAL_QMC5883//MAG_EXTERNAL_IST8310 

#define MAG_GPS_ENABLE 0//�Ƿ�ʹ��GPSģ���ϴ�����

uint8_t extern_mag_id=0xff;
void Compass_Init(void)
{										//���ô�����IST8310��QMC5883
  I2C_GPIO_Config();
	Delay_Ms(10);
	if(MAG_EXTERNAL_TYPE==MAG_EXTERNAL_QMC5883) 			
	{
		//�����Զ�ȡid���ж��Ƿ�Ϊhmc5883
		if(Single_ReadI2C0(QMC5883L_WR_ADDRESS,0x0D)!=0xff) return;
		if(Single_ReadI2C0(QMC5883L_WR_ADDRESS,0x06)==0xff) return;
		
		QMC5883L_Init();
		extern_mag_id=MAG_EXTERNAL_QMC5883;
		Sensor_Flag.Mag_Health=TRUE;
	}
  else		
	{
		if(Single_ReadI2C0(IST8310_SLAVE_ADDRESS,0x00)!=0x10) return;
		
		Single_WriteI2C0(IST8310_SLAVE_ADDRESS,0x41,0x24);//����16x�ڲ�ƽ��
	  Single_WriteI2C0(IST8310_SLAVE_ADDRESS,0x42,0xC0);//Set/Reset�ڲ�ƽ��
		Single_WriteI2C0(IST8310_SLAVE_ADDRESS,IST8310_REG_CNTRL1,0x01);//Single Measurement Mode
		
		extern_mag_id=MAG_EXTERNAL_IST8310;
		Sensor_Flag.Mag_Health=TRUE;		
	}
	set_cutoff_frequency(40, 18,&Mag_Parameter);//������У׼�Ӽ��˲�ֵ	
}

/***********************************************************
@��������Compass_Read_Data
@��ڲ�����vector3f *mag
@���ڲ�������
�������������������ݲɼ�״̬��
@���ߣ�����С��
@���ڣ�2024/01/20
*************************************************************/
uint8_t Compass_Read_Data(vector3f *mag)//��ȡ����������״̬��
{
	if(extern_mag_id==0xff) return 0;//���������ݲ�����
	
  static uint16_t compass_sampling_cnt=0;
  uint8_t buf[6];
  compass_sampling_cnt++;
  if(compass_sampling_cnt==1)
  {
		if(MAG_EXTERNAL_TYPE==MAG_EXTERNAL_IST8310)	Single_WriteI2C0(IST8310_SLAVE_ADDRESS,IST8310_REG_CNTRL1,0x01);//Single Measurement Mod		
  }
  else if(compass_sampling_cnt>=20/WP_Duty_Dt_Ms)//50ms
  {
		if(MAG_EXTERNAL_TYPE==MAG_EXTERNAL_QMC5883)
		{
			if(extern_mag_id==MAG_EXTERNAL_QMC5883)//ʵ��ΪQMC5883
			{
				buf[0]=Single_ReadI2C0(QMC5883L_RD_ADDRESS,QMC5883L_DATA_READ_X_LSB);//OUT_X_L_A
				buf[1]=Single_ReadI2C0(QMC5883L_RD_ADDRESS,QMC5883L_DATA_READ_X_MSB);//OUT_X_H_A
				buf[2]=Single_ReadI2C0(QMC5883L_RD_ADDRESS,QMC5883L_DATA_READ_Y_LSB);//OUT_Y_L_A
				buf[3]=Single_ReadI2C0(QMC5883L_RD_ADDRESS,QMC5883L_DATA_READ_Y_MSB);//OUT_Y_H_A
				buf[4]=Single_ReadI2C0(QMC5883L_RD_ADDRESS,QMC5883L_DATA_READ_Z_LSB);//OUT_Z_L_A
				buf[5]=Single_ReadI2C0(QMC5883L_RD_ADDRESS,QMC5883L_DATA_READ_Z_MSB);//OUT_Z_H_A
				#if MAG_GPS_ENABLE
				mag->x= -(float)((int16_t)((buf[1]<<8)|buf[0])/QMC5883L_CONVERT_GAUSS_8G);
				mag->y=  (float)((int16_t)((buf[3]<<8)|buf[2])/QMC5883L_CONVERT_GAUSS_8G);
				mag->z= -(float)((int16_t)((buf[5]<<8)|buf[4])/QMC5883L_CONVERT_GAUSS_8G);
				#else
				mag->x= (float)((int16_t)((buf[1]<<8)|buf[0])/QMC5883L_CONVERT_GAUSS_8G);
				mag->y= (float)((int16_t)((buf[3]<<8)|buf[2])/QMC5883L_CONVERT_GAUSS_8G);
				mag->z= (float)((int16_t)((buf[5]<<8)|buf[4])/QMC5883L_CONVERT_GAUSS_8G);				
				#endif
			}
		}
		else if(MAG_EXTERNAL_TYPE==MAG_EXTERNAL_IST8310)
		{
			i2c0ReadNByte(IST8310_SLAVE_ADDRESS,0x03,buf,6);			
			mag->x= (float)( (int16_t)((buf[1]<<8)|buf[0])/330.0f);
			mag->y=-(float)( (int16_t)((buf[3]<<8)|buf[2])/330.0f);
			mag->z= (float)( (int16_t)((buf[5]<<8)|buf[4])/330.0f);			
		}		
    compass_sampling_cnt=0;
    return 1;
  }
  return 0;
}



uint16_t compass_fault_cnt=0;
void Compass_Fault_Check(void)
{ 
	if(extern_mag_id==0xff) 
	{
		Sensor_Flag.Mag_Health=FALSE;//���������ݲ�����
		return ;
	}
	
  static uint16_t compass_gap_cnt=0;
  compass_gap_cnt++;
  if(compass_gap_cnt>=40)//ÿ200ms���һ�Σ���Ϊ�����Ƹ������ڴ���5ms
  {
    compass_gap_cnt=0;
    if(WP_Sensor.last_mag_raw.x==WP_Sensor.mag_raw.x
       &&WP_Sensor.last_mag_raw.y==WP_Sensor.mag_raw.y
         &&WP_Sensor.last_mag_raw.z==WP_Sensor.mag_raw.z)
    {
      compass_fault_cnt++;
      if(compass_fault_cnt>10)  Sensor_Flag.Mag_Health=FALSE;//���������ݲ�����   
    }
    else
    {
      compass_fault_cnt/=2;
      if(compass_fault_cnt==0)  Sensor_Flag.Mag_Health=TRUE; 
    }
		WP_Sensor.last_mag_raw=WP_Sensor.mag_raw;
  }
}

