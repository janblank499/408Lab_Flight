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
#include "TOFSensor.h"


enum TOFSENSE_TYPE
{
	TOFSENSE_UART=0,//����0.01-5m
	TOFSENSE_P,     //����0.03-8m
  TOFSENSE_F,     //����0.05-15m
  TOFSENSE_FP,    //����0.05-25m
	TOFSENSE_NUM_MAX
};

#define GROUND_RANGE_TOFSENSE  TOFSENSE_UART //�Եز�ഫ�������� 
#define TOFSENSE_DATA_EFFECTIVE_RATE 0.85f   //������Ч�� 
uint16_t TOFSENSR_MAX_RANGE[TOFSENSE_NUM_MAX]={500,800,1500,2500}; 




#define FRAME_HEADER 					0x57
#define Frame_MARK_READ_FRAME 0x10

void Check_Front_Tofsense(void);



void NLink_Data_Send(uint8_t *buf, uint32_t cnt)  
{
	USART7_Send(buf,cnt);//�û���ֲʱ����д�˴��ڷ��ͺ���
}

int32_t NLINK_ParseInt24(nint24_t data)
{
  uint8_t *byte = (uint8_t *)(&data);
  return (int32_t)(byte[0] << 8 | byte[1] << 16 | byte[2] << 24) / 256;
}

uint8_t NLINK_VerifyCheckSum(const void *data, size_t data_length)
{
  const uint8_t *byte = (uint8_t *)data;
  uint8_t sum = 0;
  for (size_t i = 0; i < data_length - 1; ++i)
  {
    sum += byte[i];
  }
  return sum == byte[data_length - 1];
}


void  NLink_TOFSense_Read_Frame(uint8_t id)
{
	uint8_t buf[8];
	uint8_t check_sum=0;
	buf[0]=FRAME_HEADER;
	buf[1]=Frame_MARK_READ_FRAME;
	buf[2]=0xff;
	buf[3]=0xff;
	buf[4]=id;
	buf[5]=0xff;
	buf[6]=0xff;
	check_sum=buf[0]+buf[1]+buf[2]+buf[3]+buf[4]+buf[5]+buf[6];
	buf[7]=check_sum;//0x63;
	NLink_Data_Send(buf,8);
}

Testime tof_t[2];
void TOFSense_Data_Receive_Anl(uint8_t *data_buf,uint8_t num)
{
  uint8_t sum = 0;
	uint8_t id=0;
  for(uint8_t i=0;i<(num-1);i++)  sum+=*(data_buf+i);
  if(!(sum==*(data_buf+num-1))) 	return;//�������У������
  if(!(*(data_buf)==0x57 && *(data_buf+1)==0x00))return;//������֡ͷ����
	id=data_buf[3];
	
	tofdata[id].pre_last_distance=tofdata[id].last_distance;//���ϴθ߶�
	tofdata[id].last_distance=tofdata[id].distance;//�ϴθ߶�
	tofdata[id].last_vel=tofdata[id].vel;//�ϴ��ٶ�
	tofdata[id].last_system_time=tofdata[id].system_time;
	tofdata[id].id=data_buf[3];
	tofdata[id].system_time=data_buf[4]|(data_buf[5]<<8)|(data_buf[6]<<16)|(data_buf[7]<<24);
	tofdata[id].dis=(int32_t)(data_buf[8]<< 8|data_buf[9]<<16|data_buf[10]<<24)/256;
	tofdata[id].dis_status=data_buf[11];
	tofdata[id].signal_strength=data_buf[12]|(data_buf[13]<<8);
	tofdata[id].distance=tofdata[id].dis/10.0f;//cm
	tofdata[id].vel=(tofdata[id].distance-tofdata[id].last_distance)/0.1f; //�۲��ٶ�
	tofdata[id].acc=(tofdata[id].vel-tofdata[id].last_vel)/0.1f;					  //�۲���ٶ�	

	if(id==0)
	{
		//��ֵ���۲�߶ȹ۲���
		GD_Distance=tofdata[0].distance*WP_AHRS.rMat[8];
		GD_Distance_Div=tofdata[0].vel;
		GD_Distance_Acc=tofdata[0].acc;		
		WP_Sensor.tofsensor_updtate_flag=1;	
		
		if((GD_Distance<=TOFSENSR_MAX_RANGE[GROUND_RANGE_TOFSENSE]*TOFSENSE_DATA_EFFECTIVE_RATE 
			&&GD_Distance>-0.01f)//��þ�����������
			&&(tofdata[0].signal_strength!=0))//���ź�ǿ�Ȳ�Ϊ0
			Sensor_Flag.Ground_Health=1;
		else  Sensor_Flag.Ground_Health=0;
	}		
}



void TOFSense_Prase(uint8_t data)
{
	static uint8_t tofsense_buf[20];
  static uint8_t data_len = 0,data_cnt = 0;
  static uint8_t state = 0;
  if(state==0&&data==0x57)//�ж�֡ͷ1
  {
    state=1;
    tofsense_buf[0]=data;
		Test_Period(&tof_t[0]);
  }
  else if(state==1&&data==0x00)//�ж�֡ͷ2
  {
    state=2;
    tofsense_buf[1]=data;
  }
  else if(state==2&&data==0xff)//�����ֽ�
  {
    state=3;
    tofsense_buf[2]=data;
  }
  else if(state==3&&data<=0xff)//id
  {
    state = 4;
    tofsense_buf[3]=data;
    data_len = 11;
    data_cnt = 0;
  }
  else if(state==4&&data_len>0)//���ݽ���
  {
    data_len--;
    tofsense_buf[4+data_cnt++]=data;
    if(data_len==0)  state = 5;
  }
  else if(state==5)//��У��
  {
    state = 0;
    tofsense_buf[4+data_cnt++]=data;
		TOFSense_Data_Receive_Anl(tofsense_buf,16);
		Test_Period(&tof_t[1]);
  }
  else state = 0;
}


ntsm_frame0_raw_t_4x4 tsm4x4;
ntsm_frame0_pixel tofsense_m[16];
void TOFSense_M_Prase(uint8_t data)
{
	static uint8_t tofsense_m_buf[112];
  static uint16_t data_len = 0,data_cnt = 0;
  static uint8_t state = 0;
  if(state==0&&data==0x57)//�ж�֡ͷ1
  {
    state=1;
    tofsense_m_buf[0]=data;
		Test_Period(&tof_t[0]);
  }
  else if(state==1&&data==0x01)//�ж�֡ͷ2
  {
    state=2;
    tofsense_m_buf[1]=data;
  }
  else if(state==2&&data==0xff)//�����ֽ�
  {
    state=3;
    tofsense_m_buf[2]=data;
  }
  else if(state==3&&data<=0xff)//id
  {
    state = 4;
    tofsense_m_buf[3]=data;
  }
  else if(state==4)//system time 0
  {
		state = 5;
    tofsense_m_buf[4]=data;
  }
  else if(state==5)//system time 1
  {
		state = 6;
    tofsense_m_buf[5]=data;
  }
  else if(state==6)//system time 2
  {
		state = 7;
    tofsense_m_buf[6]=data;
  }
  else if(state==7)//system time 3
  {
		state = 8;
    tofsense_m_buf[7]=data;
  }
  else if(state==8)//zone map
  {
    state = 9;
    tofsense_m_buf[8]=data;
		data_len=96+6;
    data_cnt = 0;
  }
  else if(state==9&&data_len>0)//���ݽ���
  {
    data_len--;
    tofsense_m_buf[9+data_cnt++]=data;
    if(data_len==0)  state = 10;
  }
  else if(state==10)//��У��
  {
    state = 0;
    tofsense_m_buf[9+data_cnt++]=data;
		memcpy(&tsm4x4,tofsense_m_buf,112);
		
		if(tsm4x4.pixel_count!=16) return;
		if(!NLINK_VerifyCheckSum(tofsense_m_buf,9+data_cnt)) return;
		Test_Period(&tof_t[1]);
		for(uint16_t i=0;i<16;i++)
		{
			tofsense_m[i].dis_status = tsm4x4.pixels[i].dis_status;
			tofsense_m[i].signal_strength = tsm4x4.pixels[i].signal_strength;
			tofsense_m[i].dis_mm=NLINK_ParseInt24(tsm4x4.pixels[i].dis)/1000.0f;
		}
		
		//��ֵ���۲�߶ȹ۲���
		float tmp=0;
		uint8_t status=(tofsense_m[5].dis_status!=0xff)
									|(tofsense_m[6].dis_status!=0xff)
									|(tofsense_m[9].dis_status!=0xff)
									|(tofsense_m[10].dis_status!=0xff)
									|(tofsense_m[0].dis_status!=0xff)
									|(tofsense_m[3].dis_status!=0xff)
									|(tofsense_m[12].dis_status!=0xff)
									|(tofsense_m[15].dis_status!=0xff);
		
		if(status==0) return;//������һ������δ������
		
		//��ȡ������ֵ��Ϊ�߶ȹ۲�ֵ
		if(tofsense_m[5].dis_status!=0xff)  tmp=fmaxf(tmp,tofsense_m[5].dis_mm);
		if(tofsense_m[6].dis_status!=0xff)  tmp=fmaxf(tmp,tofsense_m[6].dis_mm);
		if(tofsense_m[9].dis_status!=0xff)  tmp=fmaxf(tmp,tofsense_m[9].dis_mm);
		if(tofsense_m[10].dis_status!=0xff) tmp=fmaxf(tmp,tofsense_m[10].dis_mm);
    
		if(tofsense_m[0].dis_status!=0xff)  tmp=fmaxf(tmp,tofsense_m[0].dis_mm);
		if(tofsense_m[3].dis_status!=0xff)  tmp=fmaxf(tmp,tofsense_m[3].dis_mm);
		if(tofsense_m[12].dis_status!=0xff) tmp=fmaxf(tmp,tofsense_m[12].dis_mm);
		if(tofsense_m[15].dis_status!=0xff) tmp=fmaxf(tmp,tofsense_m[15].dis_mm);
		
		GD_Distance=0.1f*tmp*WP_AHRS.rMat[8];
		GD_Distance_Div=0;
		GD_Distance_Acc=0;		
		WP_Sensor.tofsensor_updtate_flag=1;	
		
		if((GD_Distance<=350&&GD_Distance>-0.01f)//��þ�����������
			&&status)//����δ������
			Sensor_Flag.Ground_Health=1;
		else  Sensor_Flag.Ground_Health=0;
  }
  else 
	{
		state = 0;
	}
}


TOFSensor_Data tofdata[TOFSENSE_MAX_NUMBER]; 
systime tofsensor;
void TOF_Statemachine(void)
{
	static uint16_t inquire_cnt=0;
	//inquire_cnt++;
	switch(inquire_cnt)
	{
		case 0 :
		{
			NLink_TOFSense_Read_Frame(0x00);
		}
		break;
		case 1 :
		{
			NLink_TOFSense_Read_Frame(0x01);	
		}
		break;
		case 2:
		{
			NLink_TOFSense_Read_Frame(0x02);
		}
		break;
		case 3:
		{
			NLink_TOFSense_Read_Frame(0x03);
		}
		break;
		case 4:
		{
			NLink_TOFSense_Read_Frame(0x04);
		}
		break;
		case 5:
		{
			NLink_TOFSense_Read_Frame(0x05);
		}
		break;
		case 6:
		{
			NLink_TOFSense_Read_Frame(0x06);
		}
		break;
		case 7:
		{
			NLink_TOFSense_Read_Frame(0x07);
		}
		break;
		case 8:
		{
			NLink_TOFSense_Read_Frame(0x08);
		}
		break;
		case 9:
		{
			inquire_cnt=0;
			NLink_TOFSense_Read_Frame(0x09);
		}
		break;
		default:	;
	}
}





float front_tofsense_distance=0;
uint8_t front_tofsense_distance_valid_cnt=0;
uint8_t front_tofsense_distance_valid_flag=0;
void Check_Front_Tofsense(void)
{
	float _dis[5]={0},tmp=0;
	front_tofsense_distance_valid_cnt=0;
	front_tofsense_distance_valid_flag=0;
	uint8_t i=0;
	for(uint8_t j=1;j<5;j++)
	{
		if(tofdata[j].signal_strength!=0) 
		{
			front_tofsense_distance_valid_cnt++;
			_dis[i++]=tofdata[j].distance;
			front_tofsense_distance_valid_flag=1;
		}
	}
	
	//����һ�鳬�������US100
	if(us100_front.distance>0&&us100_front.distance<200)
	{
		front_tofsense_distance_valid_cnt++;
		_dis[i++]=us100_front.distance;
		front_tofsense_distance_valid_flag=1;
	}
		
	if(front_tofsense_distance_valid_cnt==0) 			front_tofsense_distance=200;//û��ɨ�赽���ˣ�Ĭ�ϸ�200cm
	else if(front_tofsense_distance_valid_cnt==1) front_tofsense_distance=_dis[0];
	else if(front_tofsense_distance_valid_cnt==2) front_tofsense_distance=fminf(_dis[0],_dis[1]);
	else if(front_tofsense_distance_valid_cnt==3) 
	{
		tmp=fminf(_dis[0],_dis[1]);
		tmp=fminf(tmp,_dis[2]);
		front_tofsense_distance=tmp;
	}
	else if(front_tofsense_distance_valid_cnt==4)  
	{
		tmp=fminf(_dis[0],_dis[1]);
		tmp=fminf(tmp,_dis[2]);
		tmp=fminf(tmp,_dis[3]);
		front_tofsense_distance=tmp;
	}
	else if(front_tofsense_distance_valid_cnt==5)  
	{
		tmp=fminf(_dis[0],_dis[1]);
		tmp=fminf(tmp,_dis[2]);
		tmp=fminf(tmp,_dis[3]);
		tmp=fminf(tmp,_dis[4]);
		front_tofsense_distance=tmp;
	}	
	
}
	














uint8_t send_buf[18]={'\0'};
//��������
u8 countsum(u8 *buf)
{
	uint8_t len = 0;
	uint8_t checksum =0;
	len = sizeof(buf)+1;
	while(len --)
	{
		checksum += *buf;
		buf++;
	}
	
	//���������λ
	checksum &=0xFF;
	
	return checksum;
}


//ֹͣɨ��
void stop_scan(void)
{
	send_buf[0] = 0xAA;
	send_buf[1] = 0x55;
	send_buf[2] = 0x61;
	send_buf[3] = 0x00;
	send_buf[4] = 0x60;
	
	USART7_Send(send_buf,5);
} 

//��ʼɨ��
void start_scan(void)
{
	send_buf[0] = 0xAA;
	send_buf[1] = 0x55;
	send_buf[2] = 0x60;
	send_buf[3] = 0x00;
	send_buf[4] = 0x5F;
	
	USART7_Send(send_buf,5);
}

//�����ñ�׼���ݵĸ�ʽ���
void SMD15_setstandard(void)
{
	send_buf[0] = 0xAA;
	send_buf[1] = 0x55;
	send_buf[2] = 0x67;
	send_buf[3] = 0x01;
	send_buf[4] = 0x00;
	send_buf[5] = 0x67;
	
	USART7_Send(send_buf,6);
}

//������pixhawk���ݵĸ�ʽ���-����ģʽ�״�ֱ����������Ϣ�����ڵ������ֿ���ֱ����ʾ 
void SMD15_setpixhawk(void)//��ʹ��
{
	send_buf[0] = 0xAA;
	send_buf[1] = 0x55;
	send_buf[2] = 0x67;
	send_buf[3] = 0x01;
	send_buf[4] = 0x01;
	send_buf[5] = 0x68;
	
	USART7_Send(send_buf,6);
}

//���ò����� 
//230400��460800��512000��921600��1500000 �ֱ��Ӧ���� 0-4��Ĭ��Ϊ 460800��
void SMD15_setbaudrate(u8 i)
{
	send_buf[0] = 0xAA;
	send_buf[1] = 0x55;
	send_buf[2] = 0x66;
	send_buf[3] = 0x01;
	
	switch(i)
	{
		case 0:send_buf[4] = 0x00;break;
		case 1:send_buf[4] = 0x01;break;
		case 2:send_buf[4] = 0x02;break;
		case 3:send_buf[4] = 0x03;break;
		case 4:send_buf[4] = 0x04;break;
		default :break;
	}
	send_buf[5] = countsum(send_buf);
	USART7_Send(send_buf,6);
}

//�������Ƶ�� 
//10hz��100hz��200hz��500hz��1000hz��1800hz ���Ƶ�ʣ��ֱ��Ӧ���� 0-5��Ĭ��Ϊ100hz��
void SMD15_setScanfHZ(u8 hz)
{
	send_buf[0] = 0xAA;
	send_buf[1] = 0x55;
	send_buf[2] = 0x64;
	send_buf[3] = 0x01;
	
	switch(hz)
	{
		case 0:send_buf[4] = 0x00;break;
		case 1:send_buf[4] = 0x01;break;
		case 2:send_buf[4] = 0x02;break;
		case 3:send_buf[4] = 0x03;break;
		case 4:send_buf[4] = 0x04;break;
		case 5:send_buf[4] = 0x05;break;
		default :break;
	}
	send_buf[5] = countsum(send_buf);
	USART7_Send(send_buf,6);
}



systime sdm15_t;
void sdm15_prase(uint8_t data)
{
	static uint8_t sdm15_buf[9];
  static uint8_t data_len = 0,data_cnt = 0;
  static uint8_t state = 0;
  if(state==0&&data==0xAA)//�ж�֡ͷ1
  {
    state=1;
    sdm15_buf[0]=data;
  }
  else if(state==1&&data==0x55)//�ж�֡ͷ2
  {
    state=2;
    sdm15_buf[1]=data;
    data_cnt = 0;
  }
  else if(state==2&&data==0x60)//��������
  {
    state=3;
    sdm15_buf[2]=data;
    data_cnt = 0;
  }
  else if(state==3&&data==0x04)//�ж����ݳ���
  {
    state=4;
    sdm15_buf[3]=data;
    data_len = 4;
    data_cnt = 0;
  }
  else if(state==4&&data_len>0)//���ݽ���
  {
    data_len--;
    sdm15_buf[4+data_cnt++]=data;
    if(data_len==0)  state = 5;
  }
  else if(state==5)//��У��
  {
    state = 0;
    sdm15_buf[4+data_cnt++]=data;
		uint8_t sum=0;
		for(uint8_t i=0;i<8;i++)
		{
			sum+=sdm15_buf[i];
		}
		if(sum==sdm15_buf[8])//�����У��
		{
			tofdata[0].pre_last_distance=tofdata[0].last_distance;//���ϴθ߶�
			tofdata[0].last_distance=tofdata[0].distance;//�ϴθ߶�
			tofdata[0].last_vel=tofdata[0].vel;//�ϴ��ٶ�
			tofdata[0].last_system_time=tofdata[0].system_time;
			tofdata[0].system_time=millis();
			/*********************************************************************************/	
			Get_Systime(&sdm15_t);
			tofdata[0].dis        		 = sdm15_buf[4] | sdm15_buf[5]<<8;
			tofdata[0].signal_strength = sdm15_buf[6];
			tofdata[0].range_precision = sdm15_buf[7];
			tofdata[0].dis_status    	= 0;
			tofdata[0].id          		= 0;

			tofdata[0].distance=tofdata[0].dis*0.1f;//cm		
			tofdata[0].update_flag=1;
			if(tofdata[0].signal_strength>=50)
			{
				if(tofdata[0].distance<1200&&tofdata[0].distance>1.0f)	tofdata[0].valid=1;
				else tofdata[0].valid=0;
			}
			else tofdata[0].valid=0;
			tofdata[0].period_ms=sdm15_t.period_int;
      /*********************************************************************************/	
			tofdata[0].vel=(tofdata[0].distance-tofdata[0].last_distance)/0.1f; //�۲��ٶ�
			tofdata[0].acc=(tofdata[0].vel-tofdata[0].last_vel)/0.1f;					  //�۲���ٶ�
			
			//��ֵ���۲�߶ȹ۲���
			GD_Distance=tofdata[0].distance*WP_AHRS.rMat[8];
			GD_Distance_Div=tofdata[0].vel;
			GD_Distance_Acc=tofdata[0].acc;		
			WP_Sensor.tofsensor_updtate_flag=1;	
			Sensor_Flag.Ground_Health=tofdata[0].valid;
		}
		memset(sdm15_buf,0,9);//��һ������
  }
  else 
	{
		state = 0;
	}
}


//��ʼ������SMD15
void SMD15_init(uint32_t baudrate)//Ҫ���ڴ���2��ʼ������
{
	delay_ms(200);//�ȴ����ڳ�ʼ�����
//	//ֹͣɨ��
//	stop_scan();
//	delay_ms(1);
	SMD15_setstandard();//�����ñ�׼���ݵĸ�ʽ���
//	SMD15_setScanfHZ(1);//����ɨ��Ƶ��
//	delay_ms(5);
//	uint8_t baud;
//	//�����״��ʼ�� 230400��460800��512000��921600��1500000
//	if     (baudrate ==230400) 	baud = 0;
//	else if(baudrate ==460800) 	baud = 1;
//	else if(baudrate ==512000)	baud = 2;
//	else if(baudrate ==921600) 	baud = 3;//����stm32��ߵĲ����ʣ�������4.5M,���ɴﵽ���9.2MHz
//	else if(baudrate ==1500000) baud = 4;	
//	SMD15_setbaudrate(baud);//���ò����� ���óɹ��������״���Ч һ�㲻����
//	//��ʼɨ��
	start_scan();
}






void tofsense_init(void)
{
	float tmp_ground_sensor=US100;
	ReadFlashParameterOne(GROUND_DISTANCE_DEFAULT,&tmp_ground_sensor);
  if(isnan(tmp_ground_sensor)==0)	rangefinder_current=(uint8_t)(tmp_ground_sensor);
  switch(rangefinder_current)
	{
		case US100:
			ConfigureUART7(9600);
		break;
		case TFMINI:
			ConfigureUART7(115200);
		break;	
		case TOFSENSE:
			ConfigureUART7(921600);
		break;	
		case SMD15:
			ConfigureUART7(460800);
			SMD15_init(460800);
		break;
		case TOFSENSE_M:
			ConfigureUART7(921600);
		break;
		default:{ConfigureUART7(9600);}		
	}		
}



