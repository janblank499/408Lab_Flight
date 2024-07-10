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
#include "I2C.h"
#include "WP_DataType.h"
#include "QMC5883.h"


uint8_t QMC5883L_Read_Reg(uint8_t reg)
{
	return Single_ReadI2C0(QMC5883L_RD_ADDRESS,reg); 
}

void QMC5883L_Write_Reg(uint8_t reg, uint8_t data)
{
	Single_WriteI2C0(QMC5883L_WR_ADDRESS,reg,data);
}


void QMC5883L_Read_Data(int16_t *MagX,int16_t *MagY,int16_t *MagZ) // (-32768 / +32768)
{
	*MagX=((int16_t)QMC5883L_Read_Reg(QMC5883L_DATA_READ_X_LSB) | (((int16_t)QMC5883L_Read_Reg(QMC5883L_DATA_READ_X_MSB))<<8));
	*MagY=((int16_t)QMC5883L_Read_Reg(QMC5883L_DATA_READ_Y_LSB) | (((int16_t)QMC5883L_Read_Reg(QMC5883L_DATA_READ_Y_MSB))<<8));
	*MagZ=((int16_t)QMC5883L_Read_Reg(QMC5883L_DATA_READ_Z_LSB) | (((int16_t)QMC5883L_Read_Reg(QMC5883L_DATA_READ_Z_MSB))<<8));
}


int16_t QMC5883L_Read_Temperature()
{
	return (((int16_t)QMC5883L_Read_Reg(QMC5883L_TEMP_READ_LSB)) | (((int16_t)QMC5883L_Read_Reg(QMC5883L_TEMP_READ_MSB))<<8))/100;
}


void QMC5883L_Initialize(_qmc5883l_MODE MODE,_qmc5883l_ODR ODR,_qmc5883l_RNG RNG,_qmc5883l_OSR OSR)
{
	QMC5883L_Write_Reg(QMC5883L_CONFIG_3,0x01);
	QMC5883L_Write_Reg(QMC5883L_CONFIG_1,MODE | ODR | RNG | OSR);
}

void QMC5883L_Reset()
{
	QMC5883L_Write_Reg(QMC5883L_CONFIG_2,0x81);
}

void QMC5883L_InterruptConfig(_qmc5883l_INT INT)
{
	if(INT==INTERRUPT_ENABLE){QMC5883L_Write_Reg(QMC5883L_CONFIG_2,0x00);}
	else {QMC5883L_Write_Reg(QMC5883L_CONFIG_2,0x01);}
}


_qmc5883l_status QMC5883L_DataIsReady()
{
	uint8_t Buffer=QMC5883L_Read_Reg(QMC5883L_STATUS);
	if((Buffer&0x00)==0x00)	  {return NO_NEW_DATA;}
	else if((Buffer&0x01)==0X01){return NEW_DATA_IS_READY;}
	return NORMAL;
}

_qmc5883l_status QMC5883L_DataIsSkipped()
{
	uint8_t Buffer=QMC5883L_Read_Reg(QMC5883L_STATUS);
	if((Buffer&0x00)==0X00)	  {return NORMAL;}
	else if((Buffer&0x04)==0X04){return DATA_SKIPPED_FOR_READING;}
		return NORMAL;
}

_qmc5883l_status QMC5883L_DataIsOverflow()
{
	uint8_t Buffer=QMC5883L_Read_Reg(QMC5883L_STATUS);
	if((Buffer&0x00)==0X00)	  {return NORMAL;}
	else if((Buffer&0x02)==0X02){return DATA_OVERFLOW;}
		return NORMAL;
}


void QMC5883L_ResetCalibration() 
{ 
	Xmin=Xmax=Ymin=Ymax=0;
} 

 
float QMC5883L_Heading(int16_t Xraw,int16_t Yraw,int16_t Zraw) 
{ 
   float X=Xraw,Y=Yraw,Z=Zraw; 
   float Heading;
   if(X<Xmin) {Xmin = X;} 
   else if(X>Xmax) {Xmax = X;} 
   if(Y<Ymin) {Ymin = Y;} 
   else if(Y>Ymax) {Ymax = Y;} 
   
   if( Xmin==Xmax || Ymin==Ymax ) {return 0.0;} 
 	 X -= (Xmax+Xmin)/2; 
   Y -= (Ymax+Ymin)/2; 
   X = X/(Xmax-Xmin); 
   Y = Y/(Ymax-Ymin); 
   Heading = atan2(Y,X); 
		//EAST
	 Heading += QMC5883L_DECLINATION_ANGLE;
	//WEST
	//Heading -= QMC5883L_DECLINATION_ANGLE;	
if(Heading <0)
{Heading += 2*M_PI;}
else if(Heading > 2*M_PI)
{Heading -= 2*M_PI;}
return Heading; 
} 


void QMC5883L_Scale(int16_t *X,int16_t *Y,int16_t *Z)
{
	*X*=QMC5883L_SCALE_FACTOR;
	*Y*=QMC5883L_SCALE_FACTOR;
	*Z*=QMC5883L_SCALE_FACTOR;
}


void QMC5883L_Init(void)
{
	QMC5883L_Reset();
	delay_ms(5);
  QMC5883L_Initialize(MODE_CONTROL_CONTINUOUS,OUTPUT_DATA_RATE_200HZ,FULL_SCALE_8G,OVER_SAMPLE_RATIO_64);
}
