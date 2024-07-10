#ifndef _CONTROL_POSHOLD_H
#define _CONTROL_POSHOLD_H

float ncq_speed_mapping(float input,uint16_t input_max,float output_max);
void ncq_control_poshold(void);
uint8_t get_stopping_point_xy(Vector3f *stopping_point);
uint8_t Reach_Target_Point(void);
void ncq_control_guide_poshold(uint32_t lng_target,uint32_t lat_target,uint8_t *target_update);
void ENU_Desired_Accel_Transform_Angle(Vector2f _accel_target,Vector2f *target_angle);
void slam_control_poshold(SINS_Lite *_ins);


extern Vector3_Nav Earth_Frame_Accel_Target;   //��������������ϵ����������������Ŀ���˶����ٶ�����
extern Vector3_Nav Earth_Frame_Pos_Err;        //��������������ϵ����������������wλ��ƫ��
extern Vector2_Ang Body_Frame_Accel_Target;      //��������������ϵ��������(Y��)������(X��)����Ŀ���˶����ٶ�����
extern Vector2_Ang Body_Frame_Speed_Feedback;    //��������������ϵ��������(Y��)������(X��)����Ŀ���˶��ٶȷ���
extern Vector2_Ang Body_Frame_Pos_Err;           //���巽����λ��ƫ��
extern Vector2_Ang Body_Frame_Brake_Speed;       //���巽����ɲ���ٶ�
extern uint8 GPS_Speed_Control_Mode;
extern Vector2f accel_target,angle_target;
#endif

