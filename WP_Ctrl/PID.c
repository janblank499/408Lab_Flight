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
#include "Filter.h"
#include "PID.h"

#define NC330_PRODUCT_NUMBER  0//���ݲ�ͬ�������ã�ѡ��Ĭ��PID����
															 //0��NC280+NC330+NC360Ĭ�ϲ���
															 //1��NC360��������3D�״ﶨλ������Ĭ�ϲ��� 
															 //2��NC280+����Ȧ
															 
//4S﮵�أ���ӯ����20AĬ�Ϲ̼���NC360���ܣ����Ƶ��KV980������3��9045������
//4S﮵�أ���ӯ����20AĬ�Ϲ̼���NC330���ܣ����Ƶ��KV980��Ǭ��8045������
//4S﮵�أ�BLHeli 30A�������С������Ż��汾�̼���NC280���ܣ����Ƶ��KV1400��7040��Ҷ������

/*
1���ַ����־��  		2ƫ���޷�ֵ��			3���ַ���ƫ��ֵ��
4����ֵ         	 	5�����޷�ֵ��			6���Ʋ���Kp��
7���Ʋ���Ki��   	 	8���Ʋ���Kd��			9�������������  
10�ϴο����������	11������޷��ȣ�
*/
const float Control_Unit[18][11]=
{
#if (NC330_PRODUCT_NUMBER==0)
	//4S﮵�أ���ӯ����20AĬ�Ϲ̼���NC360���ܣ����Ƶ��KV980��9045������
  {0 ,30  ,0  ,0 , 100,  5.00   ,0.00  ,0.00  ,0  ,0 , 500},//Pitch_Angle;ƫ���Ƕ�
  {0 ,600 ,0  ,0 , 300,  1.00   ,2.00  ,5.00  ,0  ,0 , 800},//Pitch_Gyro;ƫ�����ٶ�
  {0 ,30  ,0  ,0 , 100,  5.00   ,0.00  ,0.00  ,0  ,0 , 500},//Roll_Angle;�����
  {0 ,600 ,0  ,0 , 300,  1.00   ,2.00  ,5.00  ,0  ,0 , 800},//Roll_Gyro;������ٶ�
  {0 ,45  ,0  ,0 , 150 , 5.00   ,0.00  ,0.00  ,0  ,0 , 500},//Yaw_Angle;ƫ����
  {0 ,300 ,0  ,0 , 200,  2.00   ,1.00  ,2.00  ,0  ,0 , 800},//Yaw_Gyro;ƫ�����ٶ� 
#endif
		
#if (NC330_PRODUCT_NUMBER==1)
	//4S﮵�أ���ӯ����20AĬ�Ϲ̼���NC360���ܣ����Ƶ��KV980��9045������
  {0 ,30  ,0  ,0 , 100,  5.00   ,0.00  ,0.00  ,0  ,0 , 500 },//Pitch_Angle;ƫ���Ƕ�
  {0 ,500 ,0  ,0 , 300,  0.80   ,2.00  ,5.00  ,0  ,0 , 800 },//Pitch_Gyro;ƫ�����ٶ�
  {0 ,30  ,0  ,0 , 100,  5.00   ,0.00  ,0.00  ,0  ,0 , 500 },//Roll_Angle;�����
  {0 ,500 ,0  ,0 , 300,  0.80   ,2.00  ,5.00  ,0  ,0 , 800 },//Roll_Gyro;������ٶ�
  {0 ,45  ,0  ,0 , 150 , 5.00   ,0.00  ,0.00  ,0  ,0 , 500 },//Yaw_Angle;ƫ����
  {0 ,300 ,0  ,0 , 200,  2.00   ,1.00  ,2.00  ,0  ,0 , 800 },//Yaw_Gyro;ƫ�����ٶ� 
#endif
	
#if (NC330_PRODUCT_NUMBER==2)//NC280+����Ȧ
  {0 ,30  ,0  ,0 , 100,  5.00   ,0.00  ,0.00  ,0  ,0 , 500},//Pitch_Angle;ƫ���Ƕ�
  {0 ,600 ,0  ,0 , 300,  1.20   ,2.00  ,6.00  ,0  ,0 , 800},//Pitch_Gyro;ƫ�����ٶ�
  {0 ,30  ,0  ,0 , 100,  5.00   ,0.00  ,0.00  ,0  ,0 , 500},//Roll_Angle;�����
  {0 ,600 ,0  ,0 , 300,  1.20   ,2.00  ,6.00  ,0  ,0 , 800},//Roll_Gyro;������ٶ�
  {0 ,45  ,0  ,0 , 150 , 5.00   ,0.00  ,0.00  ,0  ,0 , 500},//Yaw_Angle;ƫ����
  {0 ,300 ,0  ,0 , 200,  3.00   ,1.00  ,2.50  ,0  ,0 , 800},//Yaw_Gyro;ƫ�����ٶ�  
#endif
	
  //���߲���
  //�߶ȵ���������ƣ���ƫ���޷����������Ϊ����������½��ٶ�400cm/s
  //Z���ٶȱ���+���ֿ��ƣ���ƫ���޷�
  {0 ,200 ,0  ,0 ,100 ,  1.00     ,0.000   ,0    ,0  ,0 ,500 },//High_Position;���θ߶�λ��
  {0 ,600 ,0  ,0 ,500 ,  5.00     ,0.000   ,0    ,0  ,0 ,1000},//High_Speed;���������ٶ�
	/*
	1���ַ����־��  		2ƫ���޷�ֵ��			3���ַ���ƫ��ֵ��
	4����ֵ         	 	5�����޷�ֵ��			6���Ʋ���Kp��
	7���Ʋ���Ki��   	 	8���Ʋ���Kd��			9�������������  
	10�ϴο����������	11������޷��ȣ�
	*/
  /*                                       Kp        Ki        Kd            */
  /*1  2  3  4  5  6   7  8   9   10  11    12        13        14  15  16  17*/
  {0 ,180 ,0  ,0 ,8,   0.200    ,0.000    ,0    ,0    ,0 ,300},//Longitude_Position;ˮƽ����λ��
  {0 ,500 ,0  ,0 ,450, 1.800    ,0.450    ,0    ,0    ,0 ,800},//Longitude_Speed;ˮƽ�����ٶ�
  {0 ,180 ,0  ,0 ,8,   0.200    ,0.000    ,0    ,0    ,0 ,300},//Latitude_Position;ˮƽγ��λ��
  {0 ,500 ,0  ,0 ,450, 1.800    ,0.450    ,0    ,0    ,0 ,800},//Latitude_Speed;ˮƽγ���ٶ�
  /*************���ٶȿ�����****************/
  //���������ٶ�1000cm/s^2
  {0 ,1500 ,0  ,0 ,750,0.10    ,2.0000    ,0.0  ,0   ,0 ,1000},//��ֱ���ٶȿ�����		0.12  1.5
		
  {0 ,100  ,0  ,0 ,3,  0.32    ,0.0000    ,0    ,0   ,0 ,150}, //Ԥ���û�1������
  {0 ,100  ,0  ,0 ,15, 0.45    ,0.0000    ,0.0  ,0   ,0 ,25 }, //Ԥ���û�2������
  /*************����λ�á��ٶȿ�����****************/
  {0 ,100   ,15  ,0 ,15,  0.30  ,0.00  ,0   ,0   ,0 ,30},//����ˮƽλ�ÿ�����  0.40        0.25
  {0 ,100   ,30  ,0 ,200, 3.00  ,0.02  ,0.0 ,0   ,0 ,450},//����ˮƽ�ٶȿ����� 3.00  ,0.03 4.50  ,0.05
		
  {0 ,50    ,30  ,0 ,100, 0.50  ,0.00  ,2.5 ,0   ,0 ,300},//SDKλ�ÿ�����   2.5  150
};



lpf_param lpf_param_gyro,lpf_param_gyro_maple;
lpf_param Control_Device_Div_LPF_Parameter;
lpf_param Control_Device_Err_LPF_Parameter;
lpf_param Control_Device_SDK_Err_LPF_Parameter;
AllControler Total_Controller;//ϵͳ�ܿ�����
void PID_Init(PID_Controler *Controler,Controler_Label Label)
{
  Controler->Err_Limit_Flag=1;//ƫ���޷���־
  Controler->Integrate_Limit_Flag=1;//�����޷���־
  Controler->Integrate_Separation_Flag=(uint8)(Control_Unit[Label][0]);//1���ַ����־
  Controler->Expect=0;//����
  Controler->FeedBack=0;//����ֵ
  Controler->Err=0;//ƫ��
  Controler->Last_Err=0;//�ϴ�ƫ��
  Controler->Err_Max=Control_Unit[Label][1];//2ƫ���޷�ֵ
  Controler->Integrate_Separation_Err=Control_Unit[Label][2];//3���ַ���ƫ��ֵ
  Controler->Integrate=Control_Unit[Label][3];//4����ֵ
  Controler->Integrate_Max=Control_Unit[Label][4];//5�����޷�ֵ
  Controler->Kp=Control_Unit[Label][5];//6���Ʋ���Kp
  Controler->Ki=Control_Unit[Label][6];//7���Ʋ���Ki
  Controler->Kd=Control_Unit[Label][7];//8���Ʋ���Ki
  Controler->Control_OutPut=Control_Unit[Label][8];//9�����������
  Controler->Last_Control_OutPut=Control_Unit[Label][9];//10�ϴο����������
  Controler->Control_OutPut_Limit=Control_Unit[Label][10];//11�ϴο����������
}



void set_d_lpf_alpha(int16_t cutoff_frequency, float time_step,float *_d_lpf_alpha)
{    
    // calculate alpha
    float rc = 1/(2*PI*cutoff_frequency);
    *_d_lpf_alpha = time_step / (time_step + rc);
}


void Total_PID_Init(void)
{
  PID_Init(&Total_Controller.Pitch_Angle_Control,Pitch_Angle_Controler);
  PID_Init(&Total_Controller.Pitch_Gyro_Control,Pitch_Gyro_Controler);
  PID_Init(&Total_Controller.Roll_Angle_Control,Roll_Angle_Controler);
  PID_Init(&Total_Controller.Roll_Gyro_Control,Roll_Gyro_Controler);
  PID_Init(&Total_Controller.Yaw_Angle_Control,Yaw_Angle_Controler);
  PID_Init(&Total_Controller.Yaw_Gyro_Control,Yaw_Gyro_Controler);
  PID_Init(&Total_Controller.Height_Position_Control,Height_Position_Controler);
  PID_Init(&Total_Controller.Height_Speed_Control,Height_Speed_Controler);
  PID_Init(&Total_Controller.Longitude_Position_Control,Longitude_Position_Controler);
  PID_Init(&Total_Controller.Longitude_Speed_Control,Longitude_Speed_Controler);
  PID_Init(&Total_Controller.Latitude_Position_Control,Latitude_Position_Controler);
  PID_Init(&Total_Controller.Latitude_Speed_Control,Latitude_Speed_Controler);  
  PID_Init(&Total_Controller.Height_Acce_Control,Height_Acce_Controler);
	
  PID_Init(&Total_Controller.Reserved_User1_Control,Reserved_User1_Controler);
  PID_Init(&Total_Controller.Reserved_User2_Control,Reserved_User2_Controler);
	
  PID_Init(&Total_Controller.Optical_Position_Control,Optical_Position_Controler);
  PID_Init(&Total_Controller.Optical_Speed_Control,Optical_Speed_Controler);
	
	
  PID_Init(&Total_Controller.SDK_Roll_Position_Control,SDK_Roll_Position_Controler);
  PID_Init(&Total_Controller.SDK_Pitch_Position_Control,SDK_Roll_Position_Controler);
  
  set_cutoff_frequency(100, 10 ,&Control_Device_SDK_Err_LPF_Parameter);//100 5
	//set_cutoff_frequency(10, 4 ,&Control_Device_SDK_Err_LPF_Parameter);
	
  set_cutoff_frequency(fc_ctrl_hz, 5 ,&Control_Device_Err_LPF_Parameter);//10  PID������ƫ���ͨ
  set_cutoff_frequency(fc_ctrl_hz, 20,&Control_Device_Div_LPF_Parameter); //20
  set_cutoff_frequency(fc_ctrl_hz, 15,&lpf_param_gyro);//15  30  10
	
	set_cutoff_frequency(fc_ctrl_hz, 49,&lpf_param_gyro_maple);//��̬���ٶȿ�������ͨ�˲������
	
	set_d_lpf_alpha(40,1.0f/fc_ctrl_hz,&Total_Controller.Pitch_Gyro_Control.d_lpf_alpha);//40
	set_d_lpf_alpha(40,1.0f/fc_ctrl_hz,&Total_Controller.Roll_Gyro_Control.d_lpf_alpha);
	set_d_lpf_alpha(40,1.0f/fc_ctrl_hz,&Total_Controller.Yaw_Gyro_Control.d_lpf_alpha);
}

float PID_Control(PID_Controler *Controler,float period_second)
{
  float controller_dt=0;

	Get_Systime(&Controler->Systime_t);
	controller_dt=Controler->Systime_t.period/1000.0000f;
	if(controller_dt>1.05f*period_second||controller_dt<0.95f*period_second||isnan(controller_dt)!=0)   controller_dt=period_second;
	if(controller_dt<0.0001f) return 0;
	
  /*******ƫ�����*********************/
  Controler->Last_Err=Controler->Err;//�����ϴ�ƫ��
  Controler->Err=Controler->Expect-Controler->FeedBack;//������ȥ�����õ�ƫ��
  if(Controler->Err_Limit_Flag==1)//ƫ���޷��ȱ�־λ
  {
    if(Controler->Err>=Controler->Err_Max)   Controler->Err= Controler->Err_Max;
    if(Controler->Err<=-Controler->Err_Max)  Controler->Err=-Controler->Err_Max;
  }
  /*******���ּ���*********************/
  if(Controler->Integrate_Separation_Flag==1)//���ַ����־λ
  {
    if(ABS(Controler->Err)<=Controler->Integrate_Separation_Err)
      Controler->Integrate+=Controler->Ki*Controler->Err*controller_dt;
  }
  else
  {
    Controler->Integrate+=Controler->Ki*Controler->Err*controller_dt;
  }
  /*******�����޷�*********************/
  if(Controler->Integrate_Limit_Flag==1)//�������Ʒ��ȱ�־
  {
    if(Controler->Integrate>=Controler->Integrate_Max)   Controler->Integrate=Controler->Integrate_Max;
    if(Controler->Integrate<=-Controler->Integrate_Max)  Controler->Integrate=-Controler->Integrate_Max ;
  }
  /*******���������*********************/
  Controler->Last_Control_OutPut=Controler->Control_OutPut;//���ֵ����
  Controler->Control_OutPut=Controler->Kp*Controler->Err//����
													 +Controler->Integrate//����
													 +Controler->Kd*(Controler->Err-Controler->Last_Err);//΢��
  /*******������޷�*********************/
  if(Controler->Control_OutPut>=Controler->Control_OutPut_Limit)
    Controler->Control_OutPut=Controler->Control_OutPut_Limit;
  if(Controler->Control_OutPut<=-Controler->Control_OutPut_Limit)
    Controler->Control_OutPut=-Controler->Control_OutPut_Limit;
  /*******���������*********************/
  return Controler->Control_OutPut;
}

float PID_Control_Yaw(PID_Controler *Controler,float period_second)
{
  float controller_dt=0;
	Get_Systime(&Controler->Systime_t);
	controller_dt=Controler->Systime_t.period/1000.0000f;
	if(controller_dt>1.05f*period_second||controller_dt<0.95f*period_second||isnan(controller_dt)!=0)   controller_dt=period_second;
	if(controller_dt<0.0001f) return 0;
	
  /*******ƫ�����*********************/
  Controler->Last_Err=Controler->Err;//�����ϴ�ƫ��
  Controler->Err=Controler->Expect-Controler->FeedBack;//������ȥ�����õ�ƫ��
	
	
  /***********************ƫ����ƫ���+-180����*****************************/
	if(Controler->Err<-180)  Controler->Err=Controler->Err+360;
	if(Controler->Err>180)  Controler->Err=Controler->Err-360;
	
  if(Controler->Err_Limit_Flag==1)//ƫ���޷��ȱ�־λ
  {
    if(Controler->Err>=Controler->Err_Max)   Controler->Err= Controler->Err_Max;
    if(Controler->Err<=-Controler->Err_Max)  Controler->Err=-Controler->Err_Max;
  }
  /*******���ּ���*********************/
  if(Controler->Integrate_Separation_Flag==1)//���ַ����־λ
  {
    if(ABS(Controler->Err)<=Controler->Integrate_Separation_Err)
      Controler->Integrate+=Controler->Ki*Controler->Err*controller_dt;
  }
  else
  {
    Controler->Integrate+=Controler->Ki*Controler->Err*controller_dt;
  }
  /*******�����޷�*********************/
  if(Controler->Integrate_Limit_Flag==1)//�������Ʒ��ȱ�־
  {
    if(Controler->Integrate>=Controler->Integrate_Max)
      Controler->Integrate=Controler->Integrate_Max;
    if(Controler->Integrate<=-Controler->Integrate_Max)
      Controler->Integrate=-Controler->Integrate_Max ;
  }
  /*******���������*********************/
  Controler->Last_Control_OutPut=Controler->Control_OutPut;//���ֵ����
  Controler->Control_OutPut= Controler->Kp*Controler->Err//����
														+Controler->Integrate//����
														+Controler->Kd*(Controler->Err-Controler->Last_Err);//΢��
  /*******������޷�*********************/
  if(Controler->Control_OutPut>=Controler->Control_OutPut_Limit)
    Controler->Control_OutPut=Controler->Control_OutPut_Limit;
  if(Controler->Control_OutPut<=-Controler->Control_OutPut_Limit)
    Controler->Control_OutPut=-Controler->Control_OutPut_Limit;
  /*******���������*********************/
  return Controler->Control_OutPut;
}



float Control_Device_LPF(float curr_input,lpf_buf *Buffer,lpf_param *Parameter)
{
	if(Buffer->Output_Butter[0]==0&&
		 Buffer->Output_Butter[1]==0&&
		 Buffer->Output_Butter[2]==0&&
		 Buffer->Input_Butter[0]==0&&
		 Buffer->Input_Butter[1]==0&&
		 Buffer->Input_Butter[2]==0)
	{
		Buffer->Output_Butter[0]=curr_input;
		Buffer->Output_Butter[1]=curr_input;
		Buffer->Output_Butter[2]=curr_input;
		Buffer->Input_Butter[0]=curr_input;
		Buffer->Input_Butter[1]=curr_input;
		Buffer->Input_Butter[2]=curr_input;
		return curr_input;
	}
	
  /* ���ٶȼ�Butterworth�˲� */
  /* ��ȡ����x(n) */
  Buffer->Input_Butter[2]=curr_input;
  /* Butterworth�˲� */
  Buffer->Output_Butter[2]=Parameter->b[0] * Buffer->Input_Butter[2]
													+Parameter->b[1] * Buffer->Input_Butter[1]
													+Parameter->b[2] * Buffer->Input_Butter[0]
													-Parameter->a[1] * Buffer->Output_Butter[1]
													-Parameter->a[2] * Buffer->Output_Butter[0];
  /* x(n) ���б��� */
  Buffer->Input_Butter[0]=Buffer->Input_Butter[1];
  Buffer->Input_Butter[1]=Buffer->Input_Butter[2];
  /* y(n) ���б��� */
  Buffer->Output_Butter[0]=Buffer->Output_Butter[1];
  Buffer->Output_Butter[1]=Buffer->Output_Butter[2];
	
	for(uint16_t i=0;i<3;i++)
	{
	  if(isnan(Buffer->Output_Butter[i])==1
			||isnan(Buffer->Input_Butter[i])==1)		
			{		
				Buffer->Output_Butter[0]=curr_input;
				Buffer->Output_Butter[1]=curr_input;
				Buffer->Output_Butter[2]=curr_input;
				Buffer->Input_Butter[0]=curr_input;
				Buffer->Input_Butter[1]=curr_input;
				Buffer->Input_Butter[2]=curr_input;
				return curr_input;
			}
	}	
  return Buffer->Output_Butter[2];
}

float PID_Control_Div_LPF(PID_Controler *Controler,float period_second)
{
  int16  i=0;
  float controller_dt=0;
	Get_Systime(&Controler->Systime_t);
	controller_dt=Controler->Systime_t.period/1000.0000f;
	if(controller_dt>1.05f*period_second||controller_dt<0.95f*period_second||isnan(controller_dt)!=0)   controller_dt=period_second;
	if(controller_dt<0.0001f) return 0;
	
  /*******ƫ�����*********************/
  Controler->Last_Err=Controler->Err;//�����ϴ�ƫ��
  Controler->Err=Controler->Expect-Controler->FeedBack;//������ȥ�����õ�ƫ��
  Controler->Dis_Err=Controler->Err-Controler->Last_Err;//ԭʼ΢��
  
//  /******************************************/
//  //��ֵ�˲�����֤�õ����ݲ����䣬����������Ծʱ��΢������쳣
//	float tempa,tempb,tempc,max,min;//���ڷ������˲�  
//	tempa=Controler->Pre_Last_Dis_Err_LPF;
//  tempb=Controler->Last_Dis_Err_LPF;
//  tempc=Controler->Dis_Err;
//  max = tempa > tempb ? tempa:tempb;
//  max = max > tempc ? max:tempc;
//  min = tempa < tempb ? tempa:tempb;
//  min = min < tempc ? min:tempc;
//  if(tempa > min && tempa < max)    Controler->Dis_Err = tempa;
//  if(tempb > min  && tempb < max )  Controler->Dis_Err = tempb;
//  if(tempc > min  &&  tempc < max)  Controler->Dis_Err = tempc;
//  Controler->Pre_Last_Dis_Err_LPF = Controler->Last_Dis_Err_LPF;
//  Controler->Last_Dis_Err_LPF = Controler->Dis_Err;
//  /*****************************************/
  
  for(i=4;i>0;i--)//���ֵ�ͨ��΢�����
  {
    Controler->Dis_Error_History[i]=Controler->Dis_Error_History[i-1];
  }
  Controler->Dis_Error_History[0]=Control_Device_LPF(Controler->Dis_Err,
                                                     &Controler->_lpf_buf,
                                                     &Control_Device_Div_LPF_Parameter);//������˹��ͨ��õ���΢����,20hz
  
  if(Controler->Err_Limit_Flag==1)//ƫ���޷��ȱ�־λ
  {
    if(Controler->Err>=Controler->Err_Max)   Controler->Err= Controler->Err_Max;
    if(Controler->Err<=-Controler->Err_Max)  Controler->Err=-Controler->Err_Max;
  }
  /*******���ּ���*********************/
  if(Controler->Integrate_Separation_Flag==1)//���ַ����־λ
  {
    if(ABS(Controler->Err)<=Controler->Integrate_Separation_Err)
      Controler->Integrate+=Controler->Ki*Controler->Err*controller_dt;
  }
  else
  {
    Controler->Integrate+=Controler->Ki*Controler->Err*controller_dt;
  }
  /*******�����޷�*********************/
  if(Controler->Integrate_Limit_Flag==1)//�������Ʒ��ȱ�־
  {
    if(Controler->Integrate>=Controler->Integrate_Max)
      Controler->Integrate=Controler->Integrate_Max;
    if(Controler->Integrate<=-Controler->Integrate_Max)
      Controler->Integrate=-Controler->Integrate_Max ;
  }
  /*******���������*********************/
  Controler->Last_Control_OutPut=Controler->Control_OutPut;//���ֵ����
  Controler->Control_OutPut=Controler->Kp*Controler->Err//����
    +Controler->Integrate//����
      //+Controler->Kd*Controler->Dis_Err;//΢��
      +Controler->Kd*Controler->Dis_Error_History[0];//΢������Դ�ڰ�����˹��ͨ�˲���
  /*******������޷�*********************/
  if(Controler->Control_OutPut>=Controler->Control_OutPut_Limit)
    Controler->Control_OutPut=Controler->Control_OutPut_Limit;
  if(Controler->Control_OutPut<=-Controler->Control_OutPut_Limit)
    Controler->Control_OutPut=-Controler->Control_OutPut_Limit;
  /*******���������*********************/
  return Controler->Control_OutPut;
}


float PID_Control_Err_LPF(PID_Controler *Controler,float period_second)
{
  float controller_dt=0;
	Get_Systime(&Controler->Systime_t);
	controller_dt=Controler->Systime_t.period/1000.0000f;
	if(controller_dt>1.05f*period_second||controller_dt<0.95f*period_second||isnan(controller_dt)!=0)   controller_dt=period_second;
	if(controller_dt<0.0001f) return 0;
	
  /*******ƫ�����*********************/
  Controler->Last_Err=Controler->Err;//�����ϴ�ƫ��
  Controler->Err=Controler->Expect-Controler->FeedBack;//������ȥ�����õ�ƫ��
  Controler->Dis_Err=Controler->Err-Controler->Last_Err;//ԭʼ΢��
  
  Controler->Last_Err_LPF=Controler->Err_LPF;
  Controler->Err_LPF=Control_Device_LPF(Controler->Err,
                                        &Controler->_lpf_buf,
                                        &Control_Device_Err_LPF_Parameter);//������˹��ͨ��õ���΢����,20hz
  
  Controler->Dis_Err_LPF=Controler->Err_LPF-Controler->Last_Err_LPF;//ƫ�����ͨ���΢����
  
  if(Controler->Err_Limit_Flag==1)//ƫ���޷��ȱ�־λ
  {
    if(Controler->Err_LPF>=Controler->Err_Max)   Controler->Err_LPF= Controler->Err_Max;
    if(Controler->Err_LPF<=-Controler->Err_Max)  Controler->Err_LPF=-Controler->Err_Max;
  }
  /*******���ּ���*********************/
  if(Controler->Integrate_Separation_Flag==1)//���ַ����־λ
  {
    if(ABS(Controler->Err_LPF)<=Controler->Integrate_Separation_Err)
      Controler->Integrate+=Controler->Ki*Controler->Err_LPF*controller_dt;
  }
  else
  {
    Controler->Integrate+=Controler->Ki*Controler->Err_LPF*controller_dt;
  }
  /*******�����޷�*********************/
  if(Controler->Integrate_Limit_Flag==1)//�������Ʒ��ȱ�־
  {
    if(Controler->Integrate>=Controler->Integrate_Max)
      Controler->Integrate=Controler->Integrate_Max;
    if(Controler->Integrate<=-Controler->Integrate_Max)
      Controler->Integrate=-Controler->Integrate_Max ;
  }
  /*******���������*********************/
  Controler->Last_Control_OutPut=Controler->Control_OutPut;//���ֵ����
  Controler->Control_OutPut=Controler->Kp*Controler->Err_LPF//����
													 +Controler->Integrate//����
													 +Controler->Kd*Controler->Dis_Err_LPF;//�Ѷ�ƫ���ͨ���˴����ٶ�΢�������ͨ
  /*******������޷�*********************/
  if(Controler->Control_OutPut>=Controler->Control_OutPut_Limit)
    Controler->Control_OutPut=Controler->Control_OutPut_Limit;
  if(Controler->Control_OutPut<=-Controler->Control_OutPut_Limit)
    Controler->Control_OutPut=-Controler->Control_OutPut_Limit;
  /*******���������*********************/
  return Controler->Control_OutPut;
}


float PID_Control_SDK_Err_LPF(PID_Controler *Controler,uint8_t Differential_Enable_Flag,float period_second)
{
  float controller_dt=0;
	Get_Systime(&Controler->Systime_t);
	controller_dt=Controler->Systime_t.period/1000.0000f;
	if(controller_dt>1.05f*period_second||controller_dt<0.95f*period_second||isnan(controller_dt)!=0)   controller_dt=period_second;
	if(controller_dt<0.0001f) return 0;
	
  /*******ƫ�����*********************/
  Controler->Last_Err=Controler->Err;//�����ϴ�ƫ��
  Controler->Err=Controler->Expect-Controler->FeedBack;//������ȥ�����õ�ƫ��
  
	Controler->Dis_Err=Controler->Err-Controler->Last_Err;//ԭʼ΢��
  
  Controler->Last_Err_LPF=Controler->Err_LPF;
  Controler->Err_LPF=Control_Device_LPF(Controler->Err,
                                        &Controler->_lpf_buf,
                                        &Control_Device_SDK_Err_LPF_Parameter);//������˹��ͨ��õ���΢����,20hz
  if(Differential_Enable_Flag==0)
	{
		Controler->_lpf_buf.Input_Butter[0]=0;
		Controler->_lpf_buf.Input_Butter[1]=0;
		Controler->_lpf_buf.Input_Butter[2]=0;
		Controler->_lpf_buf.Output_Butter[0]=0;
		Controler->_lpf_buf.Output_Butter[1]=0;
		Controler->_lpf_buf.Output_Butter[2]=0;
		Controler->Last_Err_LPF=0;
		Controler->Err_LPF=0;
	}  
  Controler->Dis_Err_LPF=Controler->Err_LPF-Controler->Last_Err_LPF;//ƫ�����ͨ���΢����

	
  if(Controler->Err_Limit_Flag==1)//ƫ���޷��ȱ�־λ
  {
    if(Controler->Err_LPF>=Controler->Err_Max)   Controler->Err_LPF= Controler->Err_Max;
    if(Controler->Err_LPF<=-Controler->Err_Max)  Controler->Err_LPF=-Controler->Err_Max;
  }
  /*******���ּ���*********************/
  if(Controler->Integrate_Separation_Flag==1)//���ַ����־λ
  {
    if(ABS(Controler->Err_LPF)<=Controler->Integrate_Separation_Err)
      Controler->Integrate+=Controler->Ki*Controler->Err_LPF*controller_dt;
  }
  else
  {
    Controler->Integrate+=Controler->Ki*Controler->Err_LPF*controller_dt;
  }
  /*******�����޷�*********************/
  if(Controler->Integrate_Limit_Flag==1)//�������Ʒ��ȱ�־
  {
    if(Controler->Integrate>=Controler->Integrate_Max)
      Controler->Integrate=Controler->Integrate_Max;
    if(Controler->Integrate<=-Controler->Integrate_Max)
      Controler->Integrate=-Controler->Integrate_Max ;
  }
  /*******���������*********************/
  Controler->Last_Control_OutPut=Controler->Control_OutPut;//���ֵ����

  Controler->Control_OutPut=Controler->Kp*Controler->Err_LPF//����
														+Controler->Integrate//����	
															+Controler->Kd*Controler->Dis_Err_LPF;//�Ѷ�ƫ���ͨ���˴����ٶ�΢�������ͨ
  /*******������޷�*********************/
  if(Controler->Control_OutPut>=Controler->Control_OutPut_Limit)
    Controler->Control_OutPut=Controler->Control_OutPut_Limit;
  if(Controler->Control_OutPut<=-Controler->Control_OutPut_Limit)
    Controler->Control_OutPut=-Controler->Control_OutPut_Limit;
  /*******���������*********************/
  return Controler->Control_OutPut;
}

void PID_LPF_Reset(PID_Controler *Controler,Controler_Label Label)
{
  Controler->_lpf_buf.Input_Butter[0]=0;
  Controler->_lpf_buf.Input_Butter[1]=0;
  Controler->_lpf_buf.Input_Butter[2]=0;
  Controler->_lpf_buf.Output_Butter[0]=0;
  Controler->_lpf_buf.Output_Butter[1]=0;
  Controler->_lpf_buf.Output_Butter[2]=0;
  Controler->Last_Err_LPF=0;
  Controler->Err_LPF=0;
}


float Differential_Forward_PID_Control_Div_LPF(PID_Controler *Controler,float period_second)//΢������PID������
{
  int16  i=0;
  float tempa,tempb,tempc,max,min;//���ڷ������˲�
  float controller_dt=0;
	Get_Systime(&Controler->Systime_t);
	controller_dt=Controler->Systime_t.period/1000.0000f;
	if(controller_dt>1.05f*period_second||controller_dt<0.95f*period_second||isnan(controller_dt)!=0)   controller_dt=period_second;
	if(controller_dt<0.0001f) return 0;
	
  /*******ƫ�����*********************/
  Controler->Last_Err=Controler->Err;//�����ϴ�ƫ��
  Controler->Err=Controler->Expect-Controler->FeedBack;//������ȥ�����õ�ƫ��
  Controler->Dis_Err=-(Controler->FeedBack-Controler->Last_FeedBack);//ֻ�Է����ź�΢��
  Controler->Last_FeedBack=Controler->FeedBack;//��¼�ϴη���ֵ
  /******************************************/
  //��ֵ�˲�����֤�õ����ݲ����䣬����������Ծʱ��΢������쳣
  tempa=Controler->Pre_Last_Dis_Err_LPF;
  tempb=Controler->Last_Dis_Err_LPF;
  tempc=Controler->Dis_Err;
  max = tempa > tempb ? tempa:tempb;
  max = max > tempc ? max:tempc;
  min = tempa < tempb ? tempa:tempb;
  min = min < tempc ? min:tempc;
  if(tempa > min && tempa < max)    Controler->Dis_Err = tempa;
  if(tempb > min  && tempb < max )  Controler->Dis_Err = tempb;
  if(tempc > min  &&  tempc < max)  Controler->Dis_Err = tempc;
  Controler->Pre_Last_Dis_Err_LPF = Controler->Last_Dis_Err_LPF;
  Controler->Last_Dis_Err_LPF = Controler->Dis_Err;
  /*****************************************/  
  for(i=4;i>0;i--)//���ֵ�ͨ��΢�����
  {
    Controler->Dis_Error_History[i]=Controler->Dis_Error_History[i-1];
  }
  Controler->Dis_Error_History[0]=Control_Device_LPF(Controler->Dis_Err,
                                                     &Controler->_lpf_buf,
                                                     &Control_Device_Div_LPF_Parameter);//������˹��ͨ��õ���΢����,20hz 
  if(Controler->Err_Limit_Flag==1)//ƫ���޷��ȱ�־λ
  {
    if(Controler->Err>=Controler->Err_Max)   Controler->Err= Controler->Err_Max;
    if(Controler->Err<=-Controler->Err_Max)  Controler->Err=-Controler->Err_Max;
  }
  /*******���ּ���*********************/
  if(Controler->Integrate_Separation_Flag==1)//���ַ����־λ
  {
    if(ABS(Controler->Err)<=Controler->Integrate_Separation_Err)
      Controler->Integrate+=Controler->Ki*Controler->Err*controller_dt;
  }
  else
  {
    Controler->Integrate+=Controler->Ki*Controler->Err*controller_dt;
  }
  /*******�����޷�*********************/
  if(Controler->Integrate_Limit_Flag==1)//�������Ʒ��ȱ�־
  {
    if(Controler->Integrate>=Controler->Integrate_Max)
      Controler->Integrate=Controler->Integrate_Max;
    if(Controler->Integrate<=-Controler->Integrate_Max)
      Controler->Integrate=-Controler->Integrate_Max ;
  }
  /*******���������*********************/
  Controler->Last_Control_OutPut=Controler->Control_OutPut;//���ֵ����
  Controler->Control_OutPut=Controler->Kp*Controler->Err//����
    +Controler->Integrate//����
      //+Controler->Kd*Controler->Dis_Err;//΢��
      +Controler->Kd*Controler->Dis_Error_History[0];//΢������Դ�ڰ�����˹��ͨ�˲���
  /*******������޷�*********************/
  if(Controler->Control_OutPut>=Controler->Control_OutPut_Limit)
    Controler->Control_OutPut=Controler->Control_OutPut_Limit;
  if(Controler->Control_OutPut<=-Controler->Control_OutPut_Limit)
    Controler->Control_OutPut=-Controler->Control_OutPut_Limit;
  /*******���������*********************/
  return Controler->Control_OutPut;
}

float PID_Control_Div_LPF_For_Gyro(PID_Controler *Controler,float period_second)
{
  int16  i=0; 
  float controller_dt=0;
	Get_Systime(&Controler->Systime_t);
	controller_dt=Controler->Systime_t.period/1000.0000f;	
	if(controller_dt>1.05f*period_second||controller_dt<0.95f*period_second||isnan(controller_dt)!=0)   controller_dt=period_second;
	if(controller_dt<0.0001f) return 0;
	
  /*******ƫ�����*********************/
  Controler->Pre_Last_Err=Controler->Last_Err;//���ϴ�ƫ��
  Controler->Last_Err=Controler->Err;//�����ϴ�ƫ��
  Controler->Err=Controler->Expect-Controler->FeedBack;//������ȥ�����õ�ƫ��
	Controler->Dis_Err=(Controler->Err-Controler->Last_Err);
	Controler->Dis_Err=constrain_float(Controler->Dis_Err,-25.0f,25.0f);
 
  for(i=4;i>0;i--)//���ֵ�ͨ��΢�����
  {
    Controler->Dis_Error_History[i]=Controler->Dis_Error_History[i-1];
  }
  Controler->Dis_Error_History[0]=Control_Device_LPF(Controler->Dis_Err,
                                                     &Controler->_lpf_buf,
                                                     &lpf_param_gyro);//������˹��ͨ��õ���΢����,30hz
  Controler->Dis_Error_History[0]=constrain_float(Controler->Dis_Error_History[0],-500,500);//΢�����޷�
  //Controler->Adaptable_Kd=Controler->Kd*(1+ABS(Controler->Dis_Error_History[0])/500.0f);//����Ӧ΢�ֲ���
  Controler->Adaptable_Kd=Controler->Kd;
	
	
  if(Controler->Err_Limit_Flag==1)//ƫ���޷��ȱ�־λ
  {
    if(Controler->Err>=Controler->Err_Max)   Controler->Err= Controler->Err_Max;
    if(Controler->Err<=-Controler->Err_Max)  Controler->Err=-Controler->Err_Max;
  }
  /*******���ּ���*********************/
  if(Controler->Integrate_Separation_Flag==1)//���ַ����־λ
  {
    if(ABS(Controler->Err)<=Controler->Integrate_Separation_Err)
      Controler->Integrate+=Controler->Ki*Controler->Err*controller_dt;
  }
  else
  {
    Controler->Integrate+=Controler->Ki*Controler->Err*controller_dt;
  }
  /*******�����޷�*********************/
  if(Controler->Integrate_Limit_Flag==1)//�������Ʒ��ȱ�־
  {
    if(Controler->Integrate>=Controler->Integrate_Max)
      Controler->Integrate=Controler->Integrate_Max;
    if(Controler->Integrate<=-Controler->Integrate_Max)
      Controler->Integrate=-Controler->Integrate_Max ;
  }
  /*******���������*********************/
  Controler->Last_Control_OutPut=Controler->Control_OutPut;//���ֵ����
  Controler->Control_OutPut=Controler->Kp*Controler->Err//����
    +Controler->Integrate//����
      //+Controler->Kd*Controler->Dis_Err;//΢��
      //+Controler->Kd*Controler->Dis_Error_History[0];//΢������Դ�ڰ�����˹��ͨ�˲���
      +Controler->Adaptable_Kd*Controler->Dis_Error_History[0];//΢������Դ�ڰ�����˹��ͨ�˲���
  /*******������޷�*********************/
  if(Controler->Control_OutPut>=Controler->Control_OutPut_Limit)
    Controler->Control_OutPut=Controler->Control_OutPut_Limit;
  if(Controler->Control_OutPut<=-Controler->Control_OutPut_Limit)
    Controler->Control_OutPut=-Controler->Control_OutPut_Limit;
  /*******���������*********************/
  return Controler->Control_OutPut;
}


float pid_ctrl_rpy_gyro_maple(PID_Controler *ctrl,float period_second,diff_mode _diff_mode,lpf_mode _lpf_mode)
{
  uint16_t  i=0;
  float _dt=0;
  Get_Systime(&ctrl->Systime_t);
  _dt=ctrl->Systime_t.period/1000.0f;
	
	if(_dt>1.05f*period_second||_dt<0.95f*period_second||isnan(_dt)!=0)   _dt=period_second;
	if(_dt<0.0001f) return 0;
	
  /*******ƫ�����*********************/
	ctrl->Pre_Last_Err=ctrl->Last_Err;//���ϴ�ƫ��
  ctrl->Last_Err=ctrl->Err;//�����ϴ�ƫ��
  ctrl->Err=ctrl->Expect-ctrl->FeedBack;//������ȥ�����õ�ƫ��
	/*******΢�ּ���*********************/
	ctrl->Expect_Div   = ctrl->Expect     - ctrl->Last_Expect;   //������΢��
	ctrl->Last_Expect	 = ctrl->Expect;												   //��¼�ϴ�����
	ctrl->Feedback_Div  = ctrl->FeedBack   - ctrl->Last_FeedBack;//������΢��
	ctrl->Last_FeedBack = ctrl->FeedBack;												 //��¼�ϴη���ֵ
	ctrl->Combine_Div  = ctrl->Expect_Div - ctrl->Feedback_Div;  //���΢��
	
	
	if(_diff_mode==interval_diff)//����΢�֡����������ͷ�������΢�ֺ������������΢�ֵ����ڿɸ��ݽǶ��⻷������
	{
		ctrl->Dis_Err=ctrl->Combine_Div;
	}
	else if(_diff_mode==incomplete_diff)//΢�����С���ֻ�Է����ź�΢��
	{
		ctrl->Dis_Err=-ctrl->Feedback_Div;
	}
  else ctrl->Dis_Err=ctrl->Err-ctrl->Last_Err;//ԭʼ΢�֣�Ҳ�����ü����һ�β�����΢��
	
	ctrl->Dis_Err=constrain_float(ctrl->Dis_Err,-50.0f,50.0f);//ԭʼ΢�����޷�	
	
  for(i=4;i>0;i--)//���ֵ�ͨ��΢�����
  {
    ctrl->Dis_Error_History[i]=ctrl->Dis_Error_History[i-1];
  }
  ctrl->Dis_Error_History[0]=Control_Device_LPF(ctrl->Dis_Err,
																							 &ctrl->_lpf_buf,
																				       &lpf_param_gyro_maple);//������˹��ͨ��õ���΢����	
	
	//��΢�������,һ�׵�ͨ�˲�
	ctrl->derivative = ctrl->Dis_Err;
	ctrl->derivative = ctrl->last_derivative + ctrl->d_lpf_alpha * (ctrl->derivative - ctrl->last_derivative);
	ctrl->last_derivative = ctrl->derivative;

	//ѡ��΢���ź��˲���ʽ
	switch(_lpf_mode)
	{
		case first_order_lpf://һ�׵�ͨ
		{
			ctrl->Dis_Err_LPF=ctrl->derivative;
		}
		break;
		case second_order_lpf://���׵�ͨ	
		{
			ctrl->Dis_Err_LPF=ctrl->Dis_Error_History[0];
		}
		break;
		case noneed_lpf://����ͨ	
		{
			ctrl->Dis_Err_LPF=ctrl->Dis_Err;
		}
		break;     
		default:ctrl->Dis_Err_LPF=ctrl->Dis_Err;
	}

	if(ctrl->Err_Limit_Flag==1)	ctrl->Err=constrain_float(ctrl->Err,-ctrl->Err_Max,ctrl->Err_Max);//ƫ���޷��ȱ�־λ
  /*******���ּ���*********************/
  if(ctrl->Integrate_Separation_Flag==1)//���ַ����־λ
  {
    if(FastAbs(ctrl->Err)<=ctrl->Integrate_Separation_Err) 
			ctrl->Integrate+=ctrl->Ki*ctrl->Err*_dt;
  }
  else	ctrl->Integrate+=ctrl->Ki*ctrl->Err*_dt;
  /*******�����޷�*********************/
  if(ctrl->Integrate_Limit_Flag==1)	
		ctrl->Integrate=constrain_float(ctrl->Integrate,-ctrl->Integrate_Max,ctrl->Integrate_Max);//������������Ʒ��ȱ�־
	
	/*******���������*********************/
	ctrl->Last_Control_OutPut=ctrl->Control_OutPut;//���ֵ����
	ctrl->Control_OutPut=ctrl->Kp*ctrl->Err+ctrl->Integrate;//����+����
	ctrl->Control_OutPut+=ctrl->Kd*constrain_float(ctrl->Dis_Err_LPF,-25.0f,25.0f);//����΢�ֿ���		

  /*******������޷�*********************/
	ctrl->Control_OutPut=constrain_float(ctrl->Control_OutPut,-ctrl->Control_OutPut_Limit,ctrl->Control_OutPut_Limit);
  /*******���������*********************/
  return ctrl->Control_OutPut;
}


void  PID_Integrate_Reset(PID_Controler *Controler)  {Controler->Integrate=0.0f;}


void East_North_Ctrl_Reset(void)
{
	PID_Integrate_Reset(&Total_Controller.Latitude_Speed_Control);//���ˮƽ�ٶȿ�����������
	PID_Integrate_Reset(&Total_Controller.Latitude_Position_Control);//���ˮƽλ�ÿ�����������
	PID_Integrate_Reset(&Total_Controller.Longitude_Speed_Control);//���ˮƽ�ٶȿ�����������
	PID_Integrate_Reset(&Total_Controller.Longitude_Position_Control);//���ˮƽλ�ÿ�����������
}



void Take_Off_Reset(void)
{
  PID_Integrate_Reset(&Total_Controller.Roll_Gyro_Control);//���ǰ���λ���
  PID_Integrate_Reset(&Total_Controller.Pitch_Gyro_Control);
  PID_Integrate_Reset(&Total_Controller.Yaw_Gyro_Control);
  PID_Integrate_Reset(&Total_Controller.Pitch_Angle_Control);
  PID_Integrate_Reset(&Total_Controller.Roll_Angle_Control);
  PID_Integrate_Reset(&Total_Controller.Yaw_Angle_Control);
  
  PID_Integrate_Reset(&Total_Controller.Longitude_Speed_Control);//λ�ÿ����ٶȻ�������
  PID_Integrate_Reset(&Total_Controller.Latitude_Speed_Control);
}

void Throttle_Control_Reset(void)
{
  PID_Integrate_Reset(&Total_Controller.Height_Acce_Control);
  PID_Integrate_Reset(&Total_Controller.Height_Speed_Control);
  PID_Integrate_Reset(&Total_Controller.Height_Position_Control);
	
	PID_LPF_Reset(&Total_Controller.Height_Acce_Control,Height_Acce_Controler);
  Total_Controller.Height_Acce_Control.Expect=0;//4����
  Total_Controller.Height_Acce_Control.FeedBack=0;//5����ֵ
  Total_Controller.Height_Acce_Control.Err=0;//6ƫ��
  Total_Controller.Height_Acce_Control.Last_Err=0;//7�ϴ�ƫ��
  Total_Controller.Height_Acce_Control.Integrate=0;//10����ֵ
  Total_Controller.Height_Acce_Control.Control_OutPut=0;//15�����������
  Total_Controller.Height_Acce_Control.Last_Control_OutPut=0;//16�ϴο����������
}

#define PID_USE_NUM  14
Vector3f_pid pid_params[PID_USE_NUM]={0};
uint8_t Sort_PID_Flag=0;
/***************************************************
������: void Save_Or_Reset_PID_Parameter(void)
˵��:	PID�����ڲ�Flash����д����
���:	��
����:	��
��ע:	��ϵ���վʹ��
ע���ߣ�����С��
****************************************************/
void Save_Or_Reset_PID_Parameter()
{
  if(Sort_PID_Flag==1)//������վ����PID����д��Flash
  {
		Sort_PID_Flag=0;
    pid_params[0].p=Total_Controller.Pitch_Gyro_Control.Kp;
    pid_params[0].i=Total_Controller.Pitch_Gyro_Control.Ki;
    pid_params[0].d=Total_Controller.Pitch_Gyro_Control.Kd;
    
    pid_params[1].p=Total_Controller.Roll_Gyro_Control.Kp;
    pid_params[1].i=Total_Controller.Roll_Gyro_Control.Ki;
    pid_params[1].d=Total_Controller.Roll_Gyro_Control.Kd;
    
    pid_params[2].p=Total_Controller.Yaw_Gyro_Control.Kp;
    pid_params[2].i=Total_Controller.Yaw_Gyro_Control.Ki;
    pid_params[2].d=Total_Controller.Yaw_Gyro_Control.Kd;
    
    pid_params[3].p=Total_Controller.Pitch_Angle_Control.Kp;
    pid_params[3].i=Total_Controller.Pitch_Angle_Control.Ki;
    pid_params[3].d=Total_Controller.Pitch_Angle_Control.Kd;
    
    pid_params[4].p=Total_Controller.Roll_Angle_Control.Kp;
    pid_params[4].i=Total_Controller.Roll_Angle_Control.Ki;
    pid_params[4].d=Total_Controller.Roll_Angle_Control.Kd;
    
    pid_params[5].p=Total_Controller.Yaw_Angle_Control.Kp;
    pid_params[5].i=Total_Controller.Yaw_Angle_Control.Ki;
    pid_params[5].d=Total_Controller.Yaw_Angle_Control.Kd;
    
    pid_params[6].p=Total_Controller.Height_Position_Control.Kp;
    pid_params[6].i=Total_Controller.Height_Position_Control.Ki;
    pid_params[6].d=Total_Controller.Height_Position_Control.Kd;
    
    pid_params[7].p=Total_Controller.Height_Speed_Control.Kp;
    pid_params[7].i=Total_Controller.Height_Speed_Control.Ki;
    pid_params[7].d=Total_Controller.Height_Speed_Control.Kd;
    
		pid_params[8].p=Total_Controller.Height_Acce_Control.Kp;
    pid_params[8].i=Total_Controller.Height_Acce_Control.Ki;
    pid_params[8].d=Total_Controller.Height_Acce_Control.Kd;
    
    pid_params[9].p=Total_Controller.Latitude_Speed_Control.Kp;
    pid_params[9].i=Total_Controller.Latitude_Speed_Control.Ki;
    pid_params[9].d=Total_Controller.Latitude_Speed_Control.Kd;
    
    pid_params[10].p=Total_Controller.Latitude_Position_Control.Kp;
    pid_params[10].i=Total_Controller.Latitude_Position_Control.Ki;
    pid_params[10].d=Total_Controller.Latitude_Position_Control.Kd;
       
    pid_params[11].p=Total_Controller.Optical_Position_Control.Kp;
    pid_params[11].i=Total_Controller.Optical_Position_Control.Ki;
    pid_params[11].d=Total_Controller.Optical_Position_Control.Kd;
    
    pid_params[12].p=Total_Controller.Optical_Speed_Control.Kp;
    pid_params[12].i=Total_Controller.Optical_Speed_Control.Ki;
    pid_params[12].d=Total_Controller.Optical_Speed_Control.Kd;
    
    pid_params[13].p=Total_Controller.SDK_Roll_Position_Control.Kp;
    pid_params[13].i=Total_Controller.SDK_Roll_Position_Control.Ki;
    pid_params[13].d=Total_Controller.SDK_Roll_Position_Control.Kd;
    Save_PID_Parameter();
  }
  else if(Sort_PID_Flag==2)//����λPID��������д��Flash
  {
		Sort_PID_Flag=0;
    Total_PID_Init();//��PID��������Ϊ����Control_Unit���������
    pid_params[0].p=Total_Controller.Pitch_Gyro_Control.Kp;
    pid_params[0].i=Total_Controller.Pitch_Gyro_Control.Ki;
    pid_params[0].d=Total_Controller.Pitch_Gyro_Control.Kd;
    
    pid_params[1].p=Total_Controller.Roll_Gyro_Control.Kp;
    pid_params[1].i=Total_Controller.Roll_Gyro_Control.Ki;
    pid_params[1].d=Total_Controller.Roll_Gyro_Control.Kd;
    
    pid_params[2].p=Total_Controller.Yaw_Gyro_Control.Kp;
    pid_params[2].i=Total_Controller.Yaw_Gyro_Control.Ki;
    pid_params[2].d=Total_Controller.Yaw_Gyro_Control.Kd;
    
    pid_params[3].p=Total_Controller.Pitch_Angle_Control.Kp;
    pid_params[3].i=Total_Controller.Pitch_Angle_Control.Ki;
    pid_params[3].d=Total_Controller.Pitch_Angle_Control.Kd;
    
    pid_params[4].p=Total_Controller.Roll_Angle_Control.Kp;
    pid_params[4].i=Total_Controller.Roll_Angle_Control.Ki;
    pid_params[4].d=Total_Controller.Roll_Angle_Control.Kd;
    
    pid_params[5].p=Total_Controller.Yaw_Angle_Control.Kp;
    pid_params[5].i=Total_Controller.Yaw_Angle_Control.Ki;
    pid_params[5].d=Total_Controller.Yaw_Angle_Control.Kd;
    
    pid_params[6].p=Total_Controller.Height_Position_Control.Kp;
    pid_params[6].i=Total_Controller.Height_Position_Control.Ki;
    pid_params[6].d=Total_Controller.Height_Position_Control.Kd;
    
    pid_params[7].p=Total_Controller.Height_Speed_Control.Kp;
    pid_params[7].i=Total_Controller.Height_Speed_Control.Ki;
    pid_params[7].d=Total_Controller.Height_Speed_Control.Kd;
   
	  pid_params[8].p=Total_Controller.Height_Acce_Control.Kp;
    pid_params[8].i=Total_Controller.Height_Acce_Control.Ki;
    pid_params[8].d=Total_Controller.Height_Acce_Control.Kd;  
		
    pid_params[9].p=Total_Controller.Latitude_Speed_Control.Kp;
    pid_params[9].i=Total_Controller.Latitude_Speed_Control.Ki;
    pid_params[9].d=Total_Controller.Latitude_Speed_Control.Kd;
    
    pid_params[10].p=Total_Controller.Latitude_Position_Control.Kp;
    pid_params[10].i=Total_Controller.Latitude_Position_Control.Ki;
    pid_params[10].d=Total_Controller.Latitude_Position_Control.Kd;
   
    pid_params[11].p=Total_Controller.Optical_Position_Control.Kp;
    pid_params[11].i=Total_Controller.Optical_Position_Control.Ki;
    pid_params[11].d=Total_Controller.Optical_Position_Control.Kd;
    
    pid_params[12].p=Total_Controller.Optical_Speed_Control.Kp;
    pid_params[12].i=Total_Controller.Optical_Speed_Control.Ki;
    pid_params[12].d=Total_Controller.Optical_Speed_Control.Kd;
    
    pid_params[13].p=Total_Controller.SDK_Roll_Position_Control.Kp;
    pid_params[13].i=Total_Controller.SDK_Roll_Position_Control.Ki;
    pid_params[13].d=Total_Controller.SDK_Roll_Position_Control.Kd;
    Save_PID_Parameter();
    
    NCLink_Send_Ask_Flag[0]=1;//�ָ�Ĭ�ϲ����󣬽����µ����ݷ����õ���վ
    NCLink_Send_Ask_Flag[1]=1;
    NCLink_Send_Ask_Flag[2]=1;
    NCLink_Send_Ask_Flag[3]=1;
    NCLink_Send_Ask_Flag[4]=1;
    NCLink_Send_Ask_Flag[5]=1;
  }
  else if(Sort_PID_Flag==3)//����λPID��������д��Flash
  {
    Sort_PID_Flag=0;
    Total_PID_Init();//��PID��������Ϊ����Control_Unit���������
    pid_params[0].p=Total_Controller.Pitch_Gyro_Control.Kp;
    pid_params[0].i=Total_Controller.Pitch_Gyro_Control.Ki;
    pid_params[0].d=Total_Controller.Pitch_Gyro_Control.Kd;
    
    pid_params[1].p=Total_Controller.Roll_Gyro_Control.Kp;
    pid_params[1].i=Total_Controller.Roll_Gyro_Control.Ki;
    pid_params[1].d=Total_Controller.Roll_Gyro_Control.Kd;
    
    pid_params[2].p=Total_Controller.Yaw_Gyro_Control.Kp;
    pid_params[2].i=Total_Controller.Yaw_Gyro_Control.Ki;
    pid_params[2].d=Total_Controller.Yaw_Gyro_Control.Kd;
    
    pid_params[3].p=Total_Controller.Pitch_Angle_Control.Kp;
    pid_params[3].i=Total_Controller.Pitch_Angle_Control.Ki;
    pid_params[3].d=Total_Controller.Pitch_Angle_Control.Kd;
    
    pid_params[4].p=Total_Controller.Roll_Angle_Control.Kp;
    pid_params[4].i=Total_Controller.Roll_Angle_Control.Ki;
    pid_params[4].d=Total_Controller.Roll_Angle_Control.Kd;
    
    pid_params[5].p=Total_Controller.Yaw_Angle_Control.Kp;
    pid_params[5].i=Total_Controller.Yaw_Angle_Control.Ki;
    pid_params[5].d=Total_Controller.Yaw_Angle_Control.Kd;
    
    pid_params[6].p=Total_Controller.Height_Position_Control.Kp;
    pid_params[6].i=Total_Controller.Height_Position_Control.Ki;
    pid_params[6].d=Total_Controller.Height_Position_Control.Kd;
    
    pid_params[7].p=Total_Controller.Height_Speed_Control.Kp;
    pid_params[7].i=Total_Controller.Height_Speed_Control.Ki;
    pid_params[7].d=Total_Controller.Height_Speed_Control.Kd;
    
		pid_params[8].p=Total_Controller.Height_Acce_Control.Kp;
    pid_params[8].i=Total_Controller.Height_Acce_Control.Ki;
    pid_params[8].d=Total_Controller.Height_Acce_Control.Kd;    
    
		pid_params[9].p=Total_Controller.Latitude_Speed_Control.Kp;
    pid_params[9].i=Total_Controller.Latitude_Speed_Control.Ki;
    pid_params[9].d=Total_Controller.Latitude_Speed_Control.Kd;
    
    pid_params[10].p=Total_Controller.Latitude_Position_Control.Kp;
    pid_params[10].i=Total_Controller.Latitude_Position_Control.Ki;
    pid_params[10].d=Total_Controller.Latitude_Position_Control.Kd;
        
    pid_params[11].p=Total_Controller.Optical_Position_Control.Kp;
    pid_params[11].i=Total_Controller.Optical_Position_Control.Ki;
    pid_params[11].d=Total_Controller.Optical_Position_Control.Kd;
    
    pid_params[12].p=Total_Controller.Optical_Speed_Control.Kp;
    pid_params[12].i=Total_Controller.Optical_Speed_Control.Ki;
    pid_params[12].d=Total_Controller.Optical_Speed_Control.Kd;
    
    pid_params[13].p=Total_Controller.SDK_Roll_Position_Control.Kp;
    pid_params[13].i=Total_Controller.SDK_Roll_Position_Control.Ki;
    pid_params[13].d=Total_Controller.SDK_Roll_Position_Control.Kd;
	
    Save_PID_Parameter();
		
    NCLink_Send_Ask_Flag[0]=1;//�ָ�Ĭ�ϲ����󣬽����µ����ݷ����õ���վ
    NCLink_Send_Ask_Flag[1]=1;
    NCLink_Send_Ask_Flag[2]=1;
    NCLink_Send_Ask_Flag[3]=1;
    NCLink_Send_Ask_Flag[4]=1;
    NCLink_Send_Ask_Flag[5]=1;
  }
}



uint8_t pid_params_valid[PID_USE_NUM];
void PID_Paramter_Init_With_Flash()
{
  for(uint8_t i=0;i<PID_USE_NUM;i++)
  {
    ReadFlashParameterThree(PID1_PARAMETER_KP+3*i,&pid_params[i].p,&pid_params[i].i,&pid_params[i].d);
    if(isnan(pid_params[i].p)==0&&isnan(pid_params[i].i)==0&&isnan(pid_params[i].d)==0)
      pid_params_valid[i]=0x01;//eeprom������Ч
  }
	
  if(pid_params_valid[0]!=0x00 &&pid_params_valid[1] !=0x00 &&pid_params_valid[2] !=0x00
   &&pid_params_valid[3]!=0x00 &&pid_params_valid[4] !=0x00 &&pid_params_valid[5] !=0x00
   &&pid_params_valid[6]!=0x00 &&pid_params_valid[7] !=0x00 &&pid_params_valid[8] !=0x00
   &&pid_params_valid[9]!=0x00 &&pid_params_valid[10]!=0x00 &&pid_params_valid[11]!=0x00
	 &&pid_params_valid[12]!=0x00&&pid_params_valid[13]!=0x00)//Flash����������������PID����ֵ
  {
		Total_PID_Init();
    Total_Controller.Pitch_Gyro_Control.Kp=pid_params[0].p;
    Total_Controller.Pitch_Gyro_Control.Ki=pid_params[0].i;
    Total_Controller.Pitch_Gyro_Control.Kd=pid_params[0].d;
    
    Total_Controller.Roll_Gyro_Control.Kp=pid_params[1].p;
    Total_Controller.Roll_Gyro_Control.Ki=pid_params[1].i;
    Total_Controller.Roll_Gyro_Control.Kd=pid_params[1].d;
    
    Total_Controller.Yaw_Gyro_Control.Kp=pid_params[2].p;
    Total_Controller.Yaw_Gyro_Control.Ki=pid_params[2].i;
    Total_Controller.Yaw_Gyro_Control.Kd=pid_params[2].d;
    
    Total_Controller.Pitch_Angle_Control.Kp=pid_params[3].p;
    Total_Controller.Pitch_Angle_Control.Ki=pid_params[3].i;
    Total_Controller.Pitch_Angle_Control.Kd=pid_params[3].d;
    
    Total_Controller.Roll_Angle_Control.Kp=pid_params[4].p;
    Total_Controller.Roll_Angle_Control.Ki=pid_params[4].i;
    Total_Controller.Roll_Angle_Control.Kd=pid_params[4].d;
    
    Total_Controller.Yaw_Angle_Control.Kp=pid_params[5].p;
    Total_Controller.Yaw_Angle_Control.Ki=pid_params[5].i;
    Total_Controller.Yaw_Angle_Control.Kd=pid_params[5].d;
    
    Total_Controller.Height_Position_Control.Kp=pid_params[6].p;
    Total_Controller.Height_Position_Control.Ki=pid_params[6].i;
    Total_Controller.Height_Position_Control.Kd=pid_params[6].d;
    
    Total_Controller.Height_Speed_Control.Kp=pid_params[7].p;
    Total_Controller.Height_Speed_Control.Ki=pid_params[7].i;
    Total_Controller.Height_Speed_Control.Kd=pid_params[7].d;
				
    Total_Controller.Height_Acce_Control.Kp=pid_params[8].p;
    Total_Controller.Height_Acce_Control.Ki=pid_params[8].i;
    Total_Controller.Height_Acce_Control.Kd=pid_params[8].d;		
    
    Total_Controller.Latitude_Speed_Control.Kp=pid_params[9].p;
    Total_Controller.Latitude_Speed_Control.Ki=pid_params[9].i;
    Total_Controller.Latitude_Speed_Control.Kd=pid_params[9].d;
    
    Total_Controller.Latitude_Position_Control.Kp=pid_params[10].p;
    Total_Controller.Latitude_Position_Control.Ki=pid_params[10].i;
    Total_Controller.Latitude_Position_Control.Kd=pid_params[10].d;
    /***********************λ�ÿ��ƣ�λ�á��ٶȲ�������һ��PID����**********************************************************/
    Total_Controller.Longitude_Speed_Control.Kp=pid_params[9].p;
    Total_Controller.Longitude_Speed_Control.Ki=pid_params[9].i;
    Total_Controller.Longitude_Speed_Control.Kd=pid_params[9].d;
    
    Total_Controller.Longitude_Position_Control.Kp=pid_params[10].p;
    Total_Controller.Longitude_Position_Control.Ki=pid_params[10].i;
    Total_Controller.Longitude_Position_Control.Kd=pid_params[10].d;
			   
    Total_Controller.Optical_Position_Control.Kp=pid_params[11].p;
    Total_Controller.Optical_Position_Control.Ki=pid_params[11].i;
    Total_Controller.Optical_Position_Control.Kd=pid_params[11].d;
    
    Total_Controller.Optical_Speed_Control.Kp=pid_params[12].p;
    Total_Controller.Optical_Speed_Control.Ki=pid_params[12].i;
    Total_Controller.Optical_Speed_Control.Kd=pid_params[12].d;
    
    Total_Controller.SDK_Roll_Position_Control.Kp=pid_params[13].p;
    Total_Controller.SDK_Roll_Position_Control.Ki=pid_params[13].i;
    Total_Controller.SDK_Roll_Position_Control.Kd=pid_params[13].d;
    
    Total_Controller.SDK_Pitch_Position_Control.Kp=pid_params[13].p;
    Total_Controller.SDK_Pitch_Position_Control.Ki=pid_params[13].i;
    Total_Controller.SDK_Pitch_Position_Control.Kd=pid_params[13].d;
  }
	else//�״����س��������ʱ���ὫĬ��pid����д��eeprom��
	{
		Sort_PID_Flag=3;
	}
}

void Save_PID_Parameter(void)
{
  for(uint8_t i=0;i<PID_USE_NUM;i++)
  {
    WriteFlashParameter_Three(PID1_PARAMETER_KP+i*3,pid_params[i].p,pid_params[i].i,pid_params[i].d);
  }
}







//	/******************************************/
//  //��ֵ�˲�����֤�õ����ݲ����䣬����������Ծʱ��΢������쳣
//	float tempa,tempb,tempc,max,min;//���ڷ������˲�  
//	tempa=Controler->Pre_Last_Dis_Err_LPF;
//  tempb=Controler->Last_Dis_Err_LPF;
//  tempc=Controler->Dis_Err;
//  max = tempa > tempb ? tempa:tempb;
//  max = max > tempc ? max:tempc;
//  min = tempa < tempb ? tempa:tempb;
//  min = min < tempc ? min:tempc;
//  if(tempa > min && tempa < max)    Controler->Dis_Err = tempa;
//  if(tempb > min  && tempb < max )  Controler->Dis_Err = tempb;
//  if(tempc > min  &&  tempc < max)  Controler->Dis_Err = tempc;
//  Controler->Pre_Last_Dis_Err_LPF = Controler->Last_Dis_Err_LPF;
//  Controler->Last_Dis_Err_LPF = Controler->Dis_Err;
//  /*****************************************/








