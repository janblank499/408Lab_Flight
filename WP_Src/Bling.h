#ifndef __BLING_H
#define __BLING_H


typedef struct
{
  uint16_t Bling_Contiune_Time;//��˸����ʱ��
  uint16_t Bling_Period;//��˸����
  float  Bling_Percent;//��˸ռ�ձ�
  uint16_t  Bling_Cnt;//��˸������
  uint32_t Port; //�˿�
  uint16_t Pin;//����
  uint8_t Endless_Flag;//�޾�ģʽ
}Bling_Light;


void Bling_Set(Bling_Light *Light,
               uint32_t Continue_time,//����ʱ��
               uint16_t Period,//����100ms~1000ms
               float Percent,//0~100%
               uint16_t  Cnt,
               uint32_t Port,
               uint16_t Pin,
							 uint8_t Flag);
void Bling_Process(Bling_Light *Light);
void Bling_Working(uint16 bling_mode);
void Bling_Init(void);
void Quad_Start_Bling(void);
extern Bling_Light rgb_red,rgb_blue,rgb_green,Light_4;
extern uint16_t Bling_Mode;

							 
							 
#define Light_Blue  rgb_blue
#define Light_Red   rgb_red
#define Light_Green rgb_green
							 

#define BLING_B_PIN GPIO_PIN_2//��ɫ
#define BLING_R_PIN GPIO_PIN_1//��ɫ
#define BLING_G_PIN GPIO_PIN_3//��ɫ

#endif
