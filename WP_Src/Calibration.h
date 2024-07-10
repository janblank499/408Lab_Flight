#ifndef _CALIBRATION_H
#define _CALIBRATION_H





typedef struct
{
 float x;
 float y;
 float z;
}Acce_Unit;

typedef struct
{
 float x;
 float y;
 float z;
}Mag_Unit;


typedef struct {
    int16_t x_max;
    int16_t y_max;
    int16_t z_max;
    int16_t x_min;
    int16_t y_min;
    int16_t z_min;
    float x_offset;
    float y_offset;
    float z_offset;
}Calibration;

void Calibrationer(void);
void Accel_Calibration_Check(void);
uint8_t Accel_Calibartion(void);
bool Parameter_Init(void);
void Mag_Calibration_Check(void);
uint8_t Mag_Calibartion(Vector3f *magdata,Vector3f_Body Circle_Angle_Calibartion);
void Reset_Mag_Calibartion(uint8_t Type);
void Reset_Accel_Calibartion(uint8_t Type);

void Mag_LS_Init(void);
uint8_t Mag_Calibartion_LS(Vector3f *magdata,Vector3f_Body Circle_Angle_Calibartion);

void ESC_Calibration(void);
void RC_Calibration_Trigger(void);
bool RC_Calibration_Check(uint16 *rc_date);
void ESC_HardWave_Init(void);//ֻ��ʼ��У׼����ı�Ҫ��Դ
void ESC_Calibration_Check(void);
uint8_t Check_Calibration_Flag(void);
void Reset_RC_Calibartion(uint8_t Type);
void Horizontal_Calibration_Check(void);
void Horizontal_Calibration_Init(void);
void Headless_Mode_Calibration_Check(void);
void Gyro_Calibration_Check(vector3f *gyro);
void Other_Parameter_Init(void);
void Other_Parameter_Default(void);
void Calibration_All_Check(void);	


extern Acce_Unit acce_sample[6];
extern uint8_t  Mag_Calibration_Mode;
extern uint8_t flight_direction;
extern Acce_Unit Accel_Offset_Read,Accel_Scale_Read,Accel_Hor_Read;
extern Mag_Unit DataMag;
extern Mag_Unit Mag_Offset_Read;
extern Calibration Mag;
extern uint8_t Mag_360_Flag[3][36];
extern uint16_t Mag_Is_Okay_Flag[3];
extern float Yaw_Correct;
extern Vector2f MagN;


extern float Hor_Accel_Offset[3];
extern float mag_a,mag_b,mag_c,mag_r;
extern float ESC_Calibration_Flag;
extern float Pitch_Offset,Roll_Offset;
extern float Headless_Mode_Yaw;
extern uint8_t Gyro_Safety_Calibration_Flag;
extern uint16_t Target_Height,Safe_Height,Safe_Vbat,Max_Height,Max_Radius,Max_Upvel,Max_Downvel,Max_Horvel,Reserved_Uart,Near_Ground_Height;
extern uint16_t Uart2_Mode,Avoid_Obstacle;
extern float Accel_Simple_Mode;
#endif

