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
#include "Ringbuf.h"



/**
* @brief  RingBuff_Init
* @param  void
* @return void
* @author �ܽ�
* @date   2018
* @version v1.0
* @note   ��ʼ�����λ�����
*/
void RingBuff_Init(RingBuff_t *ringBuff)
{
  //��ʼ�������Ϣ
  ringBuff->Head = 0;
  ringBuff->Tail = 0;
  ringBuff->Lenght = 0;
}

/**
* @brief  Write_RingBuff
* @param  u8 data
* @return FLASE:���λ�����������д��ʧ��;TRUE:д��ɹ�
* @author �ܽ�
* @date   2018
* @version v1.0
* @note   �����λ�����д��u8���͵�����
*/
uint8_t Write_RingBuff(uint8_t data,RingBuff_t *ringBuff)
{
  if(ringBuff->Lenght >= RINGBUFF_LEN) //�жϻ������Ƿ�����
  {
    return 0;
  }
  ringBuff->Ring_Buff[ringBuff->Tail]=data;
  //ringBuff.Tail++;
  ringBuff->Tail = (ringBuff->Tail+1)%RINGBUFF_LEN;//��ֹԽ��Ƿ�����
  ringBuff->Lenght++;
  return 1;
}

/**
* @brief  Read_RingBuff
* @param  u8 *rData�����ڱ����ȡ������
* @return FLASE:���λ�����û�����ݣ���ȡʧ��;TRUE:��ȡ�ɹ�
* @author �ܽ�
* @date   2018
* @version v1.0
* @note   �ӻ��λ�������ȡһ��u8���͵�����
*/
uint8_t Read_RingBuff(uint8_t *rData,RingBuff_t *ringBuff)
{
  if(ringBuff->Lenght == 0)//�жϷǿ�
  {
    return 0;
  }
  *rData = ringBuff->Ring_Buff[ringBuff->Head];//�Ƚ��ȳ�FIFO���ӻ�����ͷ��
  //ringBuff.Head++;
  ringBuff->Head = (ringBuff->Head+1)%RINGBUFF_LEN;//��ֹԽ��Ƿ�����
  ringBuff->Lenght--;
  return 1;
}



void RingBuf_Write(unsigned char data,RingBuff_t *ringBuff,uint16_t Length)
{
  ringBuff->Ring_Buff[ringBuff->Tail]=data;//��β��׷��
  if(++ringBuff->Tail>=Length)//β�ڵ�ƫ��
    ringBuff->Tail=0;//����������󳤶� ���� �γɻ��ζ���  
  if(ringBuff->Tail==ringBuff->Head)//���β���ڵ�׷��ͷ���ڵ㣬���޸�ͷ�ڵ�ƫ��λ�ö�����������
  {
    if((++ringBuff->Head)>=Length)  
      ringBuff->Head=0; 
  }
}

uint8_t RingBuf_Read(unsigned char* pData,RingBuff_t *ringBuff)
{
  if(ringBuff->Head==ringBuff->Tail)  return 1;//���ͷβ�Ӵ���ʾ������Ϊ��
  else 
  {  
    *pData=ringBuff->Ring_Buff[ringBuff->Head];//����������ǿ���ȡͷ�ڵ�ֵ��ƫ��ͷ�ڵ�
    if((++ringBuff->Head)>=RINGBUFF_LEN)   ringBuff->Head=0;
    return 0;//����0����ʾ��ȡ���ݳɹ�
  }
}

