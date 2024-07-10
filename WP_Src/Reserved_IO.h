#ifndef __RESERVED_IO_H__
#define __RESERVED_IO_H__

typedef struct
{
	uint16_t times;				 //Ԥ����˸�ܴ���
	uint8_t  reset;				 //��˸���̸�λ��־
	uint16_t cnt;					 //��˸���Ƽ�����
	uint16_t times_cnt;		 //��¼����˸����
	uint8_t  end;					 //��˸��ɱ�־λ
	uint32_t port;				 //��˸���ڵĶ˿�
	uint8_t pin;					 //��˸���ڵ�GPIO
	uint32_t period;			 //��˸����
	float light_on_percent;//���������ڵ���ʱ��ٷֱ�
}_laser_light;


extern _laser_light  laser_light1,laser_light2,buzzer;
void Reserved_IO_Init(void);

void Laser_Light_Work(_laser_light *light);
void Board_Buzzer_Work(_laser_light *light);
void buzzer_setup(uint32_t _period,float _light_on_percent,uint16_t _times);

#endif


