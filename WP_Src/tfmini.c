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
#include "tfmini.h"



			
					
uint8_t rangefinder_current=TOFSENSE;

tfmini tfdata; 
uint16_t tfmini_tail,last_tfmini_tail,tfmini_offset;
void TFmini_Statemachine(void)
{
  static uint16_t tfmini_cnt=0;
	uint8_t check_sum=0;
	tfmini_cnt++;
  if(tfmini_cnt>=20)//100ms=10hz
  {
		tfmini_cnt=0;
		tfmini_tail=COM7_Rx_Buf.Tail;
		if(tfmini_tail<9)
		{
		  tfmini_offset=9;
		}
    else
		{
		  tfmini_offset=0;
		}			
		for(uint16_t i=tfmini_offset;i<tfmini_offset+9;i++)
		{
			if(COM7_Rx_Buf.Ring_Buff[i]==0x59
				&&COM7_Rx_Buf.Ring_Buff[i+1]==0x59)
			{
				for(uint16_t j=i;j<i+8;j++)  check_sum += COM7_Rx_Buf.Ring_Buff[j];//��У��
				if(check_sum==COM7_Rx_Buf.Ring_Buff[i+8])
				{
					tfdata.pre_last_distance=tfdata.last_distance;//���ϴθ߶�
					tfdata.last_distance=tfdata.distance;//�ϴθ߶�
					tfdata.last_vel=tfdata.vel;//�ϴ��ٶ�
					
					tfdata.distance=((COM7_Rx_Buf.Ring_Buff[i+3]*256)+COM7_Rx_Buf.Ring_Buff[i+2])/1.0f;
//					tfdata.distance*=rMat[8];
					tfdata.strength=(COM7_Rx_Buf.Ring_Buff[i+4]*256)+COM7_Rx_Buf.Ring_Buff[i+5];
					tfdata.temperature=((COM7_Rx_Buf.Ring_Buff[i+6]*256)+COM7_Rx_Buf.Ring_Buff[i+7])/8.0f-256;
					
					tfdata.vel=(tfdata.distance-tfdata.last_distance)/0.1f; //�۲��ٶ�
					tfdata.acc=(tfdata.vel-tfdata.last_vel)/0.1f;					  //�۲���ٶ�
					
					GD_Distance=tfdata.distance*WP_AHRS.rMat[8];
					GD_Distance_Div=tfdata.vel;
					GD_Distance_Acc=tfdata.acc;		
			
					WP_Sensor.tfmini_updtate_flag=1;
				}					
			}
		}
			
		
		if(tfdata.strength!=65535&&tfdata.strength>=100)
		{
			if(tfdata.distance<=1100&&tfdata.distance>1.0f)  Sensor_Flag.Ground_Health=1;
			else Sensor_Flag.Ground_Health=0;
		}			
		else Sensor_Flag.Ground_Health=0;
	}	
}


void Ground_Sensor_Statemachine(void)
{
	if(rangefinder_current==US100)						US_100_Statemachine();	  //������������״̬������
	else if(rangefinder_current==TFMINI)			TFmini_Statemachine();		//����TFMINI_PLUS������״̬������
	else if(rangefinder_current==TOFSENSE)	;			  //����TOFSensor������״̬������
	else if(rangefinder_current==SMD15)	    ;			  //����TOFSensor������״̬������
	else {;}
	Check_Front_Tofsense();
	us100_front_statemachine();
}
























//					static uint8_t fault_flag=0;
//					if(ABS(tfdata.vel)<1000&&tfdata.distance>=1.0f&&tfdata.strength!=65535)
//					{
//						if(fault_flag==1&&tfdata.distance==tfdata.pre_last_distance)//����=���ϴ�
//						{
//							fault_flag=1;
//						}			
//						else if(fault_flag==1&&tfdata.distance==tfdata.last_distance)//����=�ϴ�
//						{
//							fault_flag=1;
//						}
//						else if(fault_flag==1&&tfdata.last_distance==tfdata.pre_last_distance)//�ϴ�=���ϴ�
//						{
//							fault_flag=1;		
//						}
//						//����Ϊ�������쳣����
//						else
//						{
//							TF_Distance=tfdata.distance;
//							TF_Distance_Div=tfdata.vel;
//							TF_Distance_Acc=tfdata.acc;		
//						}
//					}
//					else
//					{
//						fault_flag=1;		
//					}	
					
					
					
					
					
					
					
					




