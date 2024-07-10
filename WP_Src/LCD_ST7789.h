#ifndef __LCD_ST7789_H
#define __LCD_ST7789_H			

#define	u8 unsigned char
#define	u16 unsigned int
#define	u32 unsigned long

#define Bit_SET   1
#define Bit_RESET 0
#define SYSCTL_PERIPH_GPIO_OLED SYSCTL_PERIPH_GPIOA
#define GPIO_OLED   GPIO_PORTA_BASE 
//-----------------OLED�˿ڶ���---------------- 

#define OLED_SCLK_Clr() GPIOPinWrite(GPIO_OLED, GPIO_PIN_5,0)//CLK
#define OLED_SCLK_Set() GPIOPinWrite(GPIO_OLED, GPIO_PIN_5,GPIO_PIN_5)
#define OLED_SDIN_Clr() GPIOPinWrite(GPIO_OLED, GPIO_PIN_4,0)//DIN
#define OLED_SDIN_Set() GPIOPinWrite(GPIO_OLED, GPIO_PIN_4,GPIO_PIN_4)
#define OLED_RST_Clr() 	GPIOPinWrite(GPIO_OLED, GPIO_PIN_3,0)//RES
#define OLED_RST_Set() 	GPIOPinWrite(GPIO_OLED, GPIO_PIN_3,GPIO_PIN_3)
#define OLED_DC_Clr() 	GPIOPinWrite(GPIO_OLED, GPIO_PIN_2,0)//DC
#define OLED_DC_Set() 	GPIOPinWrite(GPIO_OLED, GPIO_PIN_2,GPIO_PIN_2)		     
#define OLED_CS_Clr() 	 ;//CS
#define OLED_CS_Set()  	 ;	     
#define OLED_BLK_Clr()   ;//BLK
#define OLED_BLK_Set()   ;

#define OLED_CMD  0	//д����
#define OLED_DATA 1	//д����


//������ɫ
#define LCD_WHITE        0xFFFF
#define LCD_BLACK        0x0000	  
#define BLUE           	 0x001F  
#define BRED             0XF81F
#define GRED 			       0XFFE0
#define GBLUE			       0X07FF
#define RED           	 0xF800
#define MAGENTA       	 0xF81F
#define GREEN         	 0x07E0
#define CYAN          	 0x7FFF
#define YELLOW        	 0xFFE0
#define BROWN 			     0XBC40 //��ɫ
#define BRRED 			     0XFC07 //�غ�ɫ
#define GRAY  			     0X8430 //��ɫ
//GUI��ɫ
#define DARKBLUE      	 0X01CF	//����ɫ
#define LIGHTBLUE      	 0X7D7C	//ǳ��ɫ  
#define GRAYBLUE       	 0X5458 //����ɫ
//������ɫΪPANEL����ɫ 
#define LIGHTGREEN     	 0X841F //ǳ��ɫ
#define LGRAY 			     0XC618 //ǳ��ɫ(PANNEL),���屳��ɫ
#define LGRAYBLUE        0XA651 //ǳ����ɫ(�м����ɫ)
#define LBBLUE           0X2B12 //ǳ����ɫ(ѡ����Ŀ�ķ�ɫ)

#define USE_HORIZONTAL 2  //���ú�������������ʾ 0��1Ϊ���� 2��3Ϊ����


#if USE_HORIZONTAL==0||USE_HORIZONTAL==1
#define LCD_W 135
#define LCD_H 240
#else
#define LCD_W 240
#define LCD_H 135
#endif


void LCD_Writ_Bus(u8 dat);
void LCD_WR_DATA8(u8 dat);
void LCD_WR_DATA(u16 dat);
void LCD_WR_REG(u8 dat);
void LCD_Address_Set(u16 x1,u16 y1,u16 x2,u16 y2);
void Lcd_Init(void); 
void ST7789_LCD_Clear(u16 Color);
void ST7789_LCD_ShowChinese(u16 x,u16 y,u8 index,u8 size,u16 color);
void ST7789_LCD_DrawPoint(u16 x,u16 y,u16 color);
void ST7789_LCD_DrawPoint_big(u16 x,u16 y,u16 colory);
void ST7789_LCD_Fill(u16 xsta,u16 ysta,u16 xend,u16 yend,u16 color);
void ST7789_LCD_DrawLine(u16 x1,u16 y1,u16 x2,u16 y2,u16 color);
void ST7789_LCD_DrawRectangle(u16 x1, u16 y1, u16 x2, u16 y2,u16 color);
void ST7789_Draw_Circle(u16 x0,u16 y0,u8 r,u16 color);
void ST7789_LCD_ShowChar(u16 x,u16 y,u8 num,u8 mode,u16 color);
void ST7789_LCD_ShowString(u16 x,u16 y,const u8 *p,u16 color);
u32 mypow(u8 m,u8 n);
void ST7789_LCD_ShowNum(u16 x,u16 y,u16 num,u8 len,u16 color);
void ST7789_LCD_ShowNum1(u16 x,u16 y,float num,u8 len,u16 color);
void ST7789_LCD_ShowPicture(u16 x1,u16 y1,u16 x2,u16 y2);
			

void st7789_write_6_8_string(uint8_t x,uint8_t y,uint8_t ch[]);
void st7789_write_6_8_number(unsigned char x,unsigned char y, float number);
void st7789_write_6_8_char(uint8_t x,uint8_t y,uint8_t ucData);

extern  u16 BACK_COLOR;   //����ɫ


#endif  
	 
	 



