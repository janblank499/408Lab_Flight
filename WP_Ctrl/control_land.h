#ifndef _CONTROL_LAND_H
#define _CONTROL_LAND_H

#define Faraway_Distance 600//����home��Զ����ΪA����
#define Near_Distance 150//����home��Զ����ΪB����
#define Arrive_Distance 0//�ﵽhome�㣬��ΪC����


#define First_Nav_Rate   100//��A->B������һ��Ѳ���ٶȣ���λcm
#define Second_Nav_Rate  80//��B->C����������Ѳ���ٶȣ���λcm
#define Third_Nav_Rate   60//����C�㸽������������Ѳ���ٶ�, ��λcm
#define Default_Nav_rate 50//Ĭ��Ѳ���ٶ�


#define Nav_Safety_Height      800				//���뷵�����Զʱ����������İ�ȫѲ���߶ȣ���λcm
#define Nav_Near_Ground_Height_Default 100//�ӽ�����ĸ߶ȣ���λcm
#define Nav_Climb_Rate  100								//�����뷵�����Զʱ���ҵ�ǰ�߶�С��Nav_Safety_Heightʱ��ԭ������ʱ���ٶȡ�
#define Nav_Near_Ground_Rate -30  
#define Nav_Decline_Rate -50//��������home�����Ϸ�ʱ��ԭ���½�������ʱ���ٶ�
#define Nav_Rapid_Decline_Rate -100//�����½�������ʱ���ٶ�


#define Nav_Transition_Period 400//5ms*400=2S


void land_state_check(void);
void land_reset(void);
void landon_earth_check(void);
void land_run(void);
bool GPS_ok(void);
bool land_althold(float taret_climb_rate,float target_climb_alt);


#endif

