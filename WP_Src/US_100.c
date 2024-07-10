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
#include "US_100.h"




float GD_Distance=0,GD_Distance_Div=0,GD_Distance_Acc=0,Last_GD_Distance=0,Last_GD_Distance_Div=0;
uint16_t Start_Tail=0,Last_Start_Tail;
/***********************************************************
@��������US_100_Distance
@��ڲ�����uint8 MSB,uint8 LSB
@���ڲ�������
@�����������������߶����ݺϳ�
@���ߣ�����С��
@���ڣ�2024/01/20
*************************************************************/
float US_100_Distance(uint8 MSB,uint8 LSB)
{
  return (256*(MSB)+(LSB))/10.0f;//��λcm
}

/***********************************************************
@��������US_100_Temperature
@��ڲ�����uint8 data
@���ڲ�������
@�����������������¶����ݺϳ�
@���ߣ�����С��
@���ڣ�2024/01/20
*************************************************************/
float US_100_Temperature(uint8 data)
{
  return (data-45)/1.0;
}

/***********************************************************
@��������US_100_Start
@��ڲ�������
@���ڲ�������
@�����������������߶����ݴ���
@���ߣ�����С��
@���ڣ�2024/01/20
*************************************************************/
void US_100_Start(void)
{	
  UARTCharPut(UART7_BASE, US_100_Distance_CMD);
}


uint8_t US_100_Update_Flag=0;
uint8 US_100_Cnt=0;



us100_data us100;


/***********************************************************
@��������US_100_Statemachine
@��ڲ�������
@���ڲ�������
@�������������������ݽ���״̬��
@���ߣ�����С��
@���ڣ�2024/01/20
*************************************************************/
void US_100_Statemachine(void)
{
	static uint8_t fault_flag=0;
  US_100_Cnt++;
  if(US_100_Cnt>=20)//100ms
  {
    US_100_Start();//�������������������ⲿ�ж����洦��
    US_100_Cnt=0;
    Start_Tail=COM7_Rx_Buf.Tail;
    if(ABS(Start_Tail-Last_Start_Tail)==2)
    {
			us100.pre_last_distance=us100.last_distance;//���ϴθ߶�
			us100.last_distance=us100.distance;//�ϴθ߶�
						
      if(Start_Tail==0)
      {
        us100.distance=US_100_Distance(COM7_Rx_Buf.Ring_Buff[2],
                                    COM7_Rx_Buf.Ring_Buff[3]);
        US_100_Update_Flag=1;
      }
      else if(Start_Tail==1)
      {
        us100.distance=US_100_Distance(COM7_Rx_Buf.Ring_Buff[3],
                                    COM7_Rx_Buf.Ring_Buff[0]);
        US_100_Update_Flag=1;
      }
      else if(Start_Tail==2)
      {
        us100.distance=US_100_Distance(COM7_Rx_Buf.Ring_Buff[0],
                                    COM7_Rx_Buf.Ring_Buff[1]);
        US_100_Update_Flag=1;
      }
      else if(Start_Tail==3)
      {
        us100.distance=US_100_Distance(COM7_Rx_Buf.Ring_Buff[1],
                                    COM7_Rx_Buf.Ring_Buff[2]);
        US_100_Update_Flag=1;
      }
			
			us100.last_vel=us100.vel;
			us100.vel=(us100.distance-us100.last_distance)/0.1f;
			us100.acc=(us100.vel-us100.last_vel)/0.1f;
			if(ABS(us100.vel)<1000)
			{
				if(fault_flag==1&&us100.distance==us100.pre_last_distance)//����=���ϴ�
				{
					fault_flag=1;
				}			
				else if(fault_flag==1&&us100.distance==us100.last_distance)//����=�ϴ�
				{
					fault_flag=1;
				}
				else if(fault_flag==1&&us100.last_distance==us100.pre_last_distance)//�ϴ�=���ϴ�
				{
					fault_flag=1;		
				}
				//����Ϊ�������쳣����
				else
				{
					GD_Distance=us100.distance;
					GD_Distance_Div=us100.vel;
					GD_Distance_Acc=us100.acc;		
				}
			}
			else
			{
				fault_flag=1;		
			}	
			WP_Sensor.us100_updtate_flag=1;
    }
    Last_Start_Tail=Start_Tail;
  }		
	
  if(GD_Distance<=User_Height_Max&&GD_Distance>0)  Sensor_Flag.Ground_Health=1;
  else  Sensor_Flag.Ground_Health=0; 
}


