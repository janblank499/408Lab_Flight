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
#include "WP_PWM.h"

static uint16_t period;

//#define PWM_PERIOD_MAX  3125//2.5ms��������400hz
#define PWM_PERIOD_MAX    6250//5ms��������200hz
#define PWM_PERIOD_MAX_20MS  (3125*8) 


/***********************************************************
@��������PWM_Init
@��ڲ�������
@���ڲ�������
@����������PWM��ʼ��
@���ߣ�����С��
@���ڣ�2024/01/20
*************************************************************/
#if FLIGHT_ESC_PWM==0
void PWM_Init(void)
{
  SysCtlPWMClockSet(SYSCTL_PWMDIV_64); // Set divider to 80M/8=10M=0.1us
  SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM0); // Enable PWM peripheral
  SysCtlDelay(2); // Insert a few cycles after enabling the peripheral to allow the clock to be fully activated
  SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB); // Enable GPIOB peripheral
  SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOC); // Enable GPIOC peripheral
  SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE); // Enable GPIOE peripheral
  SysCtlDelay(2); // Insert a few cycles after enabling the peripheral to allow the clock to be fully activated
  // Use alternate function
  GPIOPinConfigure(GPIO_PB6_M0PWM0);
  GPIOPinConfigure(GPIO_PB7_M0PWM1);
	
  GPIOPinConfigure(GPIO_PB4_M0PWM2);
  GPIOPinConfigure(GPIO_PB5_M0PWM3);
  GPIOPinConfigure(GPIO_PE4_M0PWM4);
  GPIOPinConfigure(GPIO_PE5_M0PWM5);
	
  // Use pin with PWM peripheral
  GPIOPinTypePWM(GPIO_PORTB_BASE, GPIO_PIN_6);//M0PWM0
  GPIOPinTypePWM(GPIO_PORTB_BASE, GPIO_PIN_7);//M0PWM1
	
  GPIOPinTypePWM(GPIO_PORTB_BASE, GPIO_PIN_4);//M0PWM2
  GPIOPinTypePWM(GPIO_PORTB_BASE, GPIO_PIN_5);//M0PWM3
  GPIOPinTypePWM(GPIO_PORTE_BASE, GPIO_PIN_4);//M0PWM4
  GPIOPinTypePWM(GPIO_PORTE_BASE, GPIO_PIN_5);//M0PWM5
	
  // Configure the PWM generator for count down mode with immediate updates to the parameters
  PWMGenConfigure(PWM0_BASE, PWM_GEN_0, PWM_GEN_MODE_DOWN | PWM_GEN_MODE_NO_SYNC);
  PWMGenConfigure(PWM0_BASE, PWM_GEN_1, PWM_GEN_MODE_DOWN | PWM_GEN_MODE_NO_SYNC);
  PWMGenConfigure(PWM0_BASE, PWM_GEN_2, PWM_GEN_MODE_DOWN | PWM_GEN_MODE_NO_SYNC);
  //PWMGenConfigure(PWM0_BASE, PWM_GEN_3, PWM_GEN_MODE_DOWN | PWM_GEN_MODE_NO_SYNC);
	
  // The period is set to 2.5ms (400 Hz)
  period = PWM_PERIOD_MAX; 
  PWMGenPeriodSet(PWM0_BASE, PWM_GEN_0, period); // Set the period
  PWMGenPeriodSet(PWM0_BASE, PWM_GEN_1, period);
  PWMGenPeriodSet(PWM0_BASE, PWM_GEN_2, period); // Set the period
  //PWMGenPeriodSet(PWM0_BASE, PWM_GEN_3, period);
  // Start the timers in generator 0 and 1
  PWMGenEnable(PWM0_BASE, PWM_GEN_0);
  PWMGenEnable(PWM0_BASE, PWM_GEN_1);
  PWMGenEnable(PWM0_BASE, PWM_GEN_2);
  // Enable the outputs
  PWMOutputState(PWM0_BASE, PWM_OUT_0_BIT | PWM_OUT_1_BIT 
													| PWM_OUT_2_BIT | PWM_OUT_3_BIT
													| PWM_OUT_4_BIT | PWM_OUT_5_BIT
													,  true);
  PWM_Output(1000,1000,1000,1000);
	//Ԥ��PWM
	Reserved_PWM1_Output(1500);//PB7����M0PWM1������Ӧ��չ������PWM�˿ڵı��P3
	Reserved_PWM2_Output(pinch_pwm_us);//PB6����M0PWM0������Ӧ��չ������PWM�˿ڵı��P4
}

/***********************************************************
@��������PWM_Output
@��ڲ�����uint16_t width1,uint16_t width2,uint16_t width3,
uint16_t width4,uint16_t width5,uint16_t width6,
uint16_t width7,uint16_t width8
@���ڲ�������
@����������PWM���
@���ߣ�����С��
@���ڣ�2024/01/20
*************************************************************/
void PWM_Output(uint16_t width1,uint16_t width2,uint16_t width3,uint16_t width4)
{
	uint16_t pwm[4]={0};
	pwm[0]=1.25*width1;
	pwm[1]=1.25*width2;
	pwm[2]=1.25*width3;
	pwm[3]=1.25*width4;
	PWMPulseWidthSet(PWM0_BASE,PWM_OUT_5,pwm[0]);//PE5����M0PWM5 
	PWMPulseWidthSet(PWM0_BASE,PWM_OUT_4,pwm[1]);//PE4����M0PWM4 
	PWMPulseWidthSet(PWM0_BASE,PWM_OUT_2,pwm[2]);//PB4����M0PWM2 
	PWMPulseWidthSet(PWM0_BASE,PWM_OUT_3,pwm[3]);//PB5����M0PWM3 
}      

void Reserved_PWM1_Output(uint16_t us)
{
	PWMPulseWidthSet(PWM0_BASE,PWM_OUT_1,1.25*us);//PB7����M0PWM1������Ӧ��չ������PWM�˿ڵı��P3
}

void Reserved_PWM2_Output(uint16_t us)
{
	PWMPulseWidthSet(PWM0_BASE,PWM_OUT_0,1.25*us);//PB6����M0PWM0������Ӧ��չ������PWM�˿ڵı��P4
}


#else
void PWM_Init(void)
{
  SysCtlPWMClockSet(SYSCTL_PWMDIV_64); // Set divider to 80M/8=10M=0.1us
  SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM0); // Enable PWM peripheral
  SysCtlDelay(2); // Insert a few cycles after enabling the peripheral to allow the clock to be fully activated
  SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB); // Enable GPIOB peripheral
  SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOC); // Enable GPIOC peripheral
  SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE); // Enable GPIOE peripheral
  SysCtlDelay(2); // Insert a few cycles after enabling the peripheral to allow the clock to be fully activated
  // Use alternate function
  GPIOPinConfigure(GPIO_PB6_M0PWM0);//PB6����M0PWM0
  GPIOPinConfigure(GPIO_PB7_M0PWM1);//PB7����M0PWM1
  GPIOPinConfigure(GPIO_PB4_M0PWM2);//PB4����M0PWM2
  GPIOPinConfigure(GPIO_PB5_M0PWM3);//PB5����M0PWM3
  GPIOPinConfigure(GPIO_PC4_M0PWM6);//PB6����M0PWM6
  GPIOPinConfigure(GPIO_PC5_M0PWM7);//PB6����M0PWM7
  // Use pin with PWM peripheral
  GPIOPinTypePWM(GPIO_PORTB_BASE, GPIO_PIN_6);//M0PWM0
  GPIOPinTypePWM(GPIO_PORTB_BASE, GPIO_PIN_7);//M0PWM1
  GPIOPinTypePWM(GPIO_PORTB_BASE, GPIO_PIN_4);//M0PWM2
  GPIOPinTypePWM(GPIO_PORTB_BASE, GPIO_PIN_5);//M0PWM3
  GPIOPinTypePWM(GPIO_PORTC_BASE, GPIO_PIN_4);//M0PWM6
  GPIOPinTypePWM(GPIO_PORTC_BASE, GPIO_PIN_5);//M0PWM7
	
  // Configure the PWM generator for count down mode with immediate updates to the parameters
  PWMGenConfigure(PWM0_BASE, PWM_GEN_0, PWM_GEN_MODE_DOWN | PWM_GEN_MODE_NO_SYNC);
  PWMGenConfigure(PWM0_BASE, PWM_GEN_1, PWM_GEN_MODE_DOWN | PWM_GEN_MODE_NO_SYNC);
  PWMGenConfigure(PWM0_BASE, PWM_GEN_3, PWM_GEN_MODE_DOWN | PWM_GEN_MODE_NO_SYNC);
	
  // The period is set to 2.5ms (400 Hz)
  period = PWM_PERIOD_MAX; 
  PWMGenPeriodSet(PWM0_BASE, PWM_GEN_0, period); // Set the period
  PWMGenPeriodSet(PWM0_BASE, PWM_GEN_1, period);
  PWMGenPeriodSet(PWM0_BASE, PWM_GEN_3, period);
  // Start the timers in generator 0 and 1
  PWMGenEnable(PWM0_BASE, PWM_GEN_0);
  PWMGenEnable(PWM0_BASE, PWM_GEN_1);
  PWMGenEnable(PWM0_BASE, PWM_GEN_3);
  // Enable the outputs
  PWMOutputState(PWM0_BASE, PWM_OUT_0_BIT | PWM_OUT_1_BIT 
													| PWM_OUT_2_BIT | PWM_OUT_3_BIT
													| PWM_OUT_6_BIT | PWM_OUT_7_BIT 
													,  true);
  PWM_Output(1000,1000,1000,1000);
	//Ԥ��PWM
	Reserved_PWM1_Output(1500);//PB4����M0PWM2������Ӧ��չ��EPWM�˿ڵı��P7
	Reserved_PWM2_Output(1500);//PB5����M0PWM3������Ӧ��չ��EPWM�˿ڵı��P8
}

/***********************************************************
@��������PWM_Output
@��ڲ�����uint16_t width1,uint16_t width2,uint16_t width3,uint16_t width4
@���ڲ�������
@����������PWM���
@���ߣ�����С��
@���ڣ�2024/01/20
*************************************************************/
void PWM_Output(uint16_t width1,uint16_t width2,uint16_t width3,uint16_t width4)
{
	uint16_t pwm[4]={0};
	pwm[0]=1.25*width1;
	pwm[1]=1.25*width2;
	pwm[2]=1.25*width3;
	pwm[3]=1.25*width4;
	PWMPulseWidthSet(PWM0_BASE,PWM_OUT_6,pwm[0]);//PC4����M0PWM6 
  PWMPulseWidthSet(PWM0_BASE,PWM_OUT_7,pwm[1]);//PC5����M0PWM7
  PWMPulseWidthSet(PWM0_BASE,PWM_OUT_1,pwm[2]);//PB7����M0PWM1
  PWMPulseWidthSet(PWM0_BASE,PWM_OUT_0,pwm[3]);//PB6����M0PWM0
}      

void Reserved_PWM1_Output(uint16_t us)
{
	PWMPulseWidthSet(PWM0_BASE,PWM_OUT_2,1.25*us);//PB4����M0PWM2������Ӧ��չ��EPWM�˿ڵı��P7
}

void Reserved_PWM2_Output(uint16_t us)
{
	PWMPulseWidthSet(PWM0_BASE,PWM_OUT_3,1.25*us);//PB5����M0PWM3������Ӧ��չ��EPWM�˿ڵı��P8
}


#endif

