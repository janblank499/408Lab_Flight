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
#include "Developer_Mode.h"

uint8_t SDK1_Mode_Setup=0x00;					 //SDK������Ƽ�������ͬʱҲ���������ײ��Ӿ�����ģʽ����
uint8_t Forward_Vision_Mode_Setup=0x00;//ǰ���Ӿ��������Ĺ���ģʽ
int16_t task_select_cnt=1;//����ѡ�������,������Զ���ɺ�,���ڵ�ǰcase�Ļ�����
													//�Լӻ����Լ�task_select_cnt,�Ӷ�ʵ���Զ���ɺ�,ִ�в�ͬ�ķ�������
//�ر���ʾ������1~9�в�û���Զ���ɵĳ�����Ҫ�Լ��ֶ���ɺ���ȥ��sdkͨ��ִ�У�
//Ҳ����������10���Զ���ɺ���ִ����Ϻ�,�޸�SDK1_Mode_Setup��ֵ����ִ��1~9
void Auto_Flight_Ctrl(uint8_t *mode)
{
	static uint16_t openmv_work_mode=0;
	switch(*mode)
	{
    case 0://�û�����ָ����SDK������ģʽ:���+ǰ��+����
		{
			NCQ_SDK_Run();//ˮ�ں�ƽ����+�߶ȿ���			
			Flight.yaw_ctrl_mode=ROTATE;
			Flight.yaw_outer_control_output  =RC_Data.rc_rpyt[RC_YAW];
		}
		break;		
    case 1://����OPENMV�Ӿ�׷��ɫ��
		{
			Color_Block_Control_Pilot();//����OPENMV�Ӿ�ˮƽ׷��		
			Flight.yaw_ctrl_mode=ROTATE;
			Flight.yaw_outer_control_output  =RC_Data.rc_rpyt[RC_YAW];			
			Flight_Alt_Hold_Control(ALTHOLD_MANUAL_CTRL,NUL,NUL);//�߶ȿ���
		}
		break;
    case 2://����OPENMV�Ӿ�׷��AprilTag�������߼���׷��ɫ��һ��
		{
			Top_APrilTag_Control_Pilot();//����OPENMV�Ӿ�ˮƽ׷��		
			Flight.yaw_ctrl_mode=ROTATE;
			Flight.yaw_outer_control_output  =RC_Data.rc_rpyt[RC_YAW];			
			Flight_Alt_Hold_Control(ALTHOLD_MANUAL_CTRL,NUL,NUL);//�߶ȿ���
		}
		break;		
    case 3://����OPENMVѭ�����ƣ�Ĭ�Ϻ��ߣ���ֵ����openmv�����е���
		{
			Self_Track_Control_Pilot();//ѭ�������ں���ˮƽ����+ƫ������
			Flight_Alt_Hold_Control(ALTHOLD_MANUAL_CTRL,NUL,NUL);//�߶ȿ���			
		}
		break;
    case 4://ǰ��OPENMV�Ӿ�׷��	
		{
			Front_AprilTag_Control_Pilot();//ǰ��OPENMV�Ӿ�׷��
			Flight.yaw_ctrl_mode=ROTATE;
			Flight.yaw_outer_control_output  =RC_Data.rc_rpyt[RC_YAW];
			Flight_Alt_Hold_Control(ALTHOLD_MANUAL_CTRL,NUL,NUL);//�߶ȿ���
		}
		break;		
    case 5://�Զ��Ƹ�,��Ҫ�Լ���װ���/������
		{
			Front_Surround_Pole_Control_Pilot();
			Flight_Alt_Hold_Control(ALTHOLD_MANUAL_CTRL,NUL,NUL);//�߶ȿ���
		}
		break;
    case 6:
		{
			flight_subtask_1();//˳ʱ��ת��90�ȣ���ɺ���
		}
		break;
    case 7:
		{
			flight_subtask_3();//��10deg/s�Ľ��ٶ�˳ʱ��ת��10000ms����ɺ���
		}
		break;
    case 8:
		{
		  flight_subtask_5();//��������ϵ�����λ��,�����ι켣
		}
		break;
    case 9:
		{
			flight_subtask_7();//��������ϵ�£����ڳ�ʼ��ľ�������λ��,�����ι켣
		}
		break;
    case 10:
		{
			flight_subtask_8();//������й켣Բ���뾶����������������
		}
		break;
    case 11://�Զ���ɵ�ĳһ�߶�
		{
			if(Auto_Takeoff(Target_Height)==1)//WORK_HEIGHT_CM
			{
				//*mode+=1;  //����Ŀ��߶Ⱥ��л�����һSDK����
				//*mode+=4;//����Ŀ��߶Ⱥ��л�2022��7��ʡ����һ��������
				//*mode+=5;//����Ŀ��߶Ⱥ��л�2022��7��ʡ���ڶ���������
				//*mode+=6;//����Ŀ��߶Ⱥ��л�2022��7��ʡ��������������
				*mode+=task_select_cnt;
			}
		}
		break;
    case 12://2021�����ֲ�����˻�����
		{
			if(openmv_work_mode==0)//ֻ����һ��
			{
				openmv_work_mode=0x07;
				SDK_DT_Send_Check(openmv_work_mode,UART3_SDK);//������֮�󣬽��ײ�OPENMV���óɼ��ũ����ģʽ
			}
			//2021�������ƾ���G��ֲ�����˻�
		  Agriculture_UAV_Closeloop();//��������
			//Agriculture_UAV_Innovation();//���Ӳ���
		}
		break;
    case 13://ROS��λ���������˻�
		{
			ros_flight_support();
		}
		break;
    case 14: //�������ƿ��ƺ��������������������µ���վ����
		{
			basic_auto_flight_support();
		}
		break;
		case 15://2022������ͻ����˻����⡪������
		{
			if(openmv_work_mode==0)//ֻ����һ��
			{
				openmv_work_mode=0x10;
				SDK_DT_Send_Check(openmv_work_mode,UART3_SDK);//������֮�󣬽��ײ�OPENMV���ó�ɫ�顢��״���ģʽ
			}
			//2022���µ�����ƾ���B���ͻ����˻�������1����
			Deliver_UAV_Basic();
		}
		break;
		case 16://2022������ͻ����˻����⡪������
		{
			if(openmv_work_mode==0)//ֻ����һ��
			{
				openmv_work_mode=0x10;
				SDK_DT_Send_Check(openmv_work_mode,UART3_SDK);//������֮�󣬽��ײ�OPENMV���ó�ɫ�顢��״���ģʽ
			}
			//2022���µ�����ƾ���B���ͻ����˻�������2����
			Deliver_UAV_Innovation();
		}
		break;		
		case 17://2022������ͻ����˻����⡪������
		{
			//2022���µ�����ƾ���B���ͻ����˻�������3����
			Deliver_UAV_Hulahoop();
		}
		break;
		case 18://�Զ���ɵ�ĳһ�߶�
		{
			if(Auto_Takeoff(Target_Height)==1)
			{
				*mode+=1;//����Ŀ��߶Ⱥ��л�����һSDK����
			}
		}
		break;		
		case 19://�û��Զ��庽�����-������α�̣��Ϳ���ʵ��3ά�ռ��ڵ����ɺ����������
		{
			//�û�ͨ������վ�Զ�����߰����ֶ�������ά�ĺ���λ�ã����˻����α����������㣬��ǰ���֧��28�����㣬����������չ
			Navigation_User_Setpoint();
		}
		break;
		case 20://�Զ���ɵ�180cm�߶ȣ���ɺ����task_select_cntֵ������ִ�л��������Ƿ�������
		{
			if(Auto_Takeoff(180)==1)//�����߶�180cm
			{
				*mode+=task_select_cnt;//task_select_cnt����Ϊ1:����Ŀ��߶Ⱥ��л���������������
															 //task_select_cnt����Ϊ2:����Ŀ��߶Ⱥ��л������Ӳ�������
			}
		}	
		break;	
		case 21://2023��TI����G�⡪����������
		{
			Air_Ground_Extinguish_Fire_System_Basic();//���˻���������󷵺�����
		}
		break;
		case 22://2023��TI����G�⡪�����Ӳ���
		{
			Air_Ground_Extinguish_Fire_System_Innovation();//���˻���������󷵺�����+�����+����Դ��Ϣ���͸�С��
		}
		break;		
		case 23://2023��TI����G��Ŀ������������С������״̬ʱ�ã�����ʵ�ʷ���,ֱ�ӷ��ͻ�Դ������Ϣ��С��
		{
			//�����sdk��ͻ����������Ϣ��ʵ�ַɻ�����;�м�⵽��Դ��Ϣ��
			//�ٷ�����С��һ����Ч���������û���С������ܵ������е���
			fire_flag=1;	//��Դ���±�־λ
			fire_x=65+35; //ȥ����ʼƫ�ú��Դ��������X
			fire_y=110+35;//ȥ����ʼƫ�ú��Դ��������Y
		}
		break;	
		case 24:
		{
			//�û�Ԥ�����񣬱�д��ע�����break����
		}		
		case 25:
		{
			//�û�Ԥ�����񣬱�д��ע�����break����
		}
		case 26:
		{
			//�û�Ԥ�����񣬱�д��ע�����break����
		}		
		case 27:
		{
			//�û�Ԥ�����񣬱�д��ע�����break����
		}
		case 28:
		{
			//�û�Ԥ�����񣬱�д��ע�����break����
		}	
		case 30:
		{
			//�û�Ԥ�����񣬱�д��ע�����break����
		}			
    case 31://ǰ��Ԥ��case�����������ִ�д�����
		{
			Flight.roll_outer_control_output =RC_Data.rc_rpyt[RC_ROLL];
			Flight.pitch_outer_control_output=RC_Data.rc_rpyt[RC_PITCH];			
			Flight.yaw_ctrl_mode=ROTATE;
			Flight.yaw_outer_control_output  =RC_Data.rc_rpyt[RC_YAW];
			Flight_Alt_Hold_Control(ALTHOLD_MANUAL_CTRL,NUL,NUL);//�߶ȿ���
		}
		break;
		case 32://SDKģʽ��ԭ�ؽ��������浡�ٺ�ͣ��,��������ִ����ɺ���
		{
			OpticalFlow_Control(0);		
			Flight.yaw_ctrl_mode=ROTATE;
			Flight.yaw_outer_control_output  =RC_Data.rc_rpyt[RC_YAW];
		  Flight_Alt_Hold_Control(ALTHOLD_AUTO_VEL_CTRL,NUL,-50);//�߶ȿ���
		}
    break;		
	  default:
		{
			Flight.roll_outer_control_output =RC_Data.rc_rpyt[RC_ROLL];
			Flight.pitch_outer_control_output=RC_Data.rc_rpyt[RC_PITCH];	
			Flight.yaw_ctrl_mode=ROTATE;
			Flight.yaw_outer_control_output  =RC_Data.rc_rpyt[RC_YAW];
			Flight_Alt_Hold_Control(ALTHOLD_MANUAL_CTRL,NUL,NUL);//�߶ȿ���
		}
	}
}


