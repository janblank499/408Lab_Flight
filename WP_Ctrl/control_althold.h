#ifndef _CONTROL_ALTHOLD_H
#define _CONTROL_ALTHOLD_H



//#define ALT_VEL_FEEDFORWARD_ENABLE 		 1 //�ٶ�ǰ������ʹ��


typedef enum 
{
  ALTHOLD_MANUAL_CTRL=0,		//�߶��ֶ�����
  ALTHOLD_AUTO_POS_CTRL,		//�߶�ֱ��λ�ÿ���
	ALTHOLD_AUTO_VEL_CTRL,    //�߶�ֱ���ٶȿ���
}ALTHOLD_CTRL_MODE;

#define NUL 0


void Flight_Alt_Hold_Control(uint8_t mode,float target_alt,float target_vel);



void Thr_Scale_Set(rc_calibration *rc_date);
void ncq_control_althold(void);
float get_stopping_point_z(Vector3f *stopping_point);
uint8_t Thr_Push_Over_Deadband(void);


extern float ALT_VEL_FEEDFORWARD_ENABLE_Output;//��ֱ�ٶ�ǰ�����������;
extern float ALT_VEL_FEEDFORWARD_ENABLE_Rate;//��ֱ�ٶ�ǰ����������APM����Ϊ1��0.45;
extern float ALT_VEL_FEEDFORWARD_ENABLE_Delta;//��ֱ�����ٶȱ仯��;
extern float Last_Alt_Vel_Target;
extern float Alt_Vel_Target;
extern Vector3f UAV_Cushion_Stop_Point;

extern uint16_t  Deadband;//������λ����
extern uint16_t  Deadzone_Min;
extern uint16_t  Deadzone_Max;
extern uint16_t  Thr_Top;//����������г�
extern uint16_t  Thr_Buttom;//����������г�
extern uint8_t Thr_Push_Over_State;
#endif

