#ifndef __BLING_H
#define __BLING_H


typedef struct
{
  uint16_t Bling_Contiune_Time;//闪烁持续时间
  uint16_t Bling_Period;//闪烁周期
  float  Bling_Percent;//闪烁占空比
  uint16_t  Bling_Cnt;//闪烁计数器
  uint32_t Port; //端口
  uint16_t Pin;//引脚
  uint8_t Endless_Flag;//无尽模式
}Bling_Light;


void Bling_Set(Bling_Light *Light,
               uint32_t Continue_time,//持续时间
               uint16_t Period,//周期100ms~1000ms
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
							 

#define BLING_B_PIN GPIO_PIN_2//蓝色
#define BLING_R_PIN GPIO_PIN_1//红色
#define BLING_G_PIN GPIO_PIN_3//绿色

#endif
