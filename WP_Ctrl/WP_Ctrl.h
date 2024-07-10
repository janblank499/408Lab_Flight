#ifndef _WP_CTRL_
#define _WP_CTRL_

#define Aerocraft_Axis_Mode_Default  4
#define Aerocraft_Axis_Mode Aerocraft_Axis_Mode_Default
          
#if (Aerocraft_Axis_Mode==6)//������������ӳ���
/*
     ��ͷ
    3   1
    *   *   
      
5 *   *   * 6

    *   *
    2   4
���У�1��4��5��ʱ��ת��
     2��3��6��ʱ��ת��
*/
#define Moter1_Thr_Scale 1.0f
#define Moter2_Thr_Scale 1.0f
#define Moter3_Thr_Scale 1.0f
#define Moter4_Thr_Scale 1.0f
#define Moter5_Thr_Scale 1.0f
#define Moter6_Thr_Scale 1.0f


#define Moter1_Roll_Scale -0.75f//-0.5f
#define Moter2_Roll_Scale  0.75f//0.5f
#define Moter3_Roll_Scale  0.75f//0.5f
#define Moter4_Roll_Scale -0.75f//-0.5f
#define Moter5_Roll_Scale  0.75f//1.0f
#define Moter6_Roll_Scale -0.75f//-1.0f
          
#define Moter1_Pitch_Scale  0.866025f
#define Moter2_Pitch_Scale -0.866025f
#define Moter3_Pitch_Scale  0.866025f
#define Moter4_Pitch_Scale -0.866025f
#define Moter5_Pitch_Scale  0.0f
#define Moter6_Pitch_Scale  0.0f
          
#define Moter1_Yaw_Scale -1.0f
#define Moter2_Yaw_Scale 1.0f
#define Moter3_Yaw_Scale 1.0f
#define Moter4_Yaw_Scale -1.0f
#define Moter5_Yaw_Scale -1.0f
#define Moter6_Yaw_Scale 1.0f

#else//������������ӳ���
/*
      ��ͷ
    3     1
    *     *   
       * 
    *     *
    2     4
���У�1��2��ʱ��ת��
      3��4��ʱ��ת��
*/
#define Moter1_Thr_Scale 1.0f
#define Moter2_Thr_Scale 1.0f
#define Moter3_Thr_Scale 1.0f
#define Moter4_Thr_Scale 1.0f
#define Moter5_Thr_Scale 0.0f
#define Moter6_Thr_Scale 0.0f

#define Moter1_Roll_Scale -1.0f
#define Moter2_Roll_Scale 1.0f
#define Moter3_Roll_Scale 1.0f
#define Moter4_Roll_Scale -1.0f
#define Moter5_Roll_Scale 0.0f
#define Moter6_Roll_Scale 0.0f
          
#define Moter1_Pitch_Scale 1.0f
#define Moter2_Pitch_Scale -1.0f
#define Moter3_Pitch_Scale 1.0f
#define Moter4_Pitch_Scale -1.0f
#define Moter5_Pitch_Scale 0.0f
#define Moter6_Pitch_Scale 0.0f
          
#define Moter1_Yaw_Scale -1.0f
#define Moter2_Yaw_Scale -1.0f
#define Moter3_Yaw_Scale 1.0f
#define Moter4_Yaw_Scale 1.0f
#define Moter5_Yaw_Scale 0.0f
#define Moter6_Yaw_Scale 0.0f
#endif
          
          
enum YAW_CTRL_MODE
{
	ROTATE=0,							//�ֶ�ƫ������ģʽ
  	AZIMUTH=1,						//����ƫ���Ƕȿ���ģʽ
	CLOCKWISE=2,					//���ƫ���Ƕ�˳ʱ�����ģʽ	
	ANTI_CLOCKWISE=3,			//���ƫ���Ƕ���ʱ�����ģʽ	
	CLOCKWISE_TURN=4,			//���ٶȿ���˳ʱ��ģʽ
	ANTI_CLOCKWISE_TURN=5,//���ٶȿ�����ʱ��ģʽ
};

typedef enum
{
	MOTOR1=0,
	MOTOR2,
	MOTOR3,
	MOTOR4,
	MOTOR5,
	MOTOR6,
	MOTOR7,
	MOTOR8,
	MOTOR_NUM
}motor;

typedef struct
{
	float throttle_control_output;	  	//���ſ��������������������δʹ��
	float roll_control_output;			  	//�����̬���������������������δʹ��
	float pitch_control_output;			  	//������̬���������������������δʹ��
	float yaw_control_output;				  	//ƫ����̬���������������������δʹ��
	float roll_outer_control_output;  	//�����̬����������
	float pitch_outer_control_output; 	//������̬����������
  float yaw_outer_control_output;	  	//ƫ����̬����������
  uint16_t motor_output[MOTOR_NUM];		//���ӳ�����ֵ��������δʹ��
  uint16_t temperature_control_output;//�¶ȿ��������ֵ��������δʹ��
	uint16_t yaw_ctrl_cnt;							//ƫ�����Ƽ�����
	uint8_t yaw_ctrl_mode;							//ƫ������ģʽ
	uint8_t yaw_ctrl_start;							//ƫ�����ƿ�ʼ��־λ
	uint8_t yaw_ctrl_end;								//ƫ�����ƽ�����־λ
	uint32_t start_time_ms;							//ƫ�����ƿ�ʼʱ��
	uint32_t execution_time_ms;					//ƫ������ִ��ʱ��
	uint8_t init;												//ƫ�����Ƴ�ʼ����־λ
	//uint8_t yaw_least_cost_enable;      //ƫ��������С����ʹ�ܱ�־λ
	uint8_t roll_pitch_angle_limit_enable;
}Controller_Output;


extern Controller_Output Flight;




#define Optical_Enable  1//����������ͣ��־
#define ADRC_MODE  0
#define PID_MODE   1
#define TEST_MODE  2
//#define GYRO_CONTROL_MODE  TEST_MODE
#define GYRO_CONTROL_MODE  PID_MODE
//#define GYRO_CONTROL_MODE  ADRC_MODE



void Total_Control(void);
void Control_Output(void);
void Landon_Earth_Check(void);//�Լ촥�ؽ��뵡��ģʽ
void CarryPilot_Control(void);
	

extern float LPButter_Vel_Error(float curr_input);
extern uint8_t Controler_High_Mode,Controler_SDK1_Mode;
extern uint8_t Controler_Horizontal_Mode,Last_Controler_Horizontal_Mode;
extern uint16_t High_Hold_Throttle;
extern uint8_t Landon_Earth_Flag;
extern uint16 Throttle;
extern uint8_t Last_Landon_Earth_Flag,Landon_Earth_Flag;
extern uint16_t Landon_Earth_Cnt;
extern uint16_t Throttle_Output;
extern uint8_t Landon_Earth_Flag;
extern int16_t M_PWM_1,M_PWM_2,M_PWM_3,M_PWM_4,M_PWM_5,M_PWM_6;//�ĸ�������PWM
extern Vector2_Ang Body_Frame_Pos_Err;
extern Vector3_Nav Earth_Frame_Pos_Err;

#endif

