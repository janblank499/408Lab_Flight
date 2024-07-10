#ifndef __HEADFILE_H__
#define __HEADFILE_H__




#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <stdbool.h>
#include <math.h>





#define  USER_INT0  0x00   //PPM     ң����PPM���ݽ���  0x00<<6
#define  USER_INT1  0x20   //UART6   GPS���ݽ���/ROSͨѶ����	921600/460800												 
#define  USER_INT2  0x40   //UART0   �ײ�OPENMV���ݽ���	256000
                           //UART3   ǰ��OPENMV���ݽ���	256000
#define  USER_INT3  0x60   //UART7   ������ͨѶ����   921600
													 //UART1   �������µ���վ����	921600													
#define  USER_INT4  0x80   //UART2   �������ݽ���19200/�����״�230400
#define  USER_INT5  0xA0	 //TIMER1A   1ms
#define  USER_INT6  0xC0   //TIMER0		 5ms
#define  USER_INT7  0xE0   //TIMER2    10ms
													 //ADC0SS0

#define ABS(X)  (((X)>0)?(X):-(X))
#define MAX(a,b)  ((a)>(b)?(a):(b))
#define MIN(a,b)  ((a)>(b)?(b):(a))

#define fc_ctrl_hz    200
#define fc_ctrl_dt   (1.0f/fc_ctrl_hz)
#define min_ctrl_dt   fc_ctrl_dt 


#define imu_sampling_hz 		1000
#define imu_sampling_dt 	 (1.0f/imu_sampling_hz)
#define imu_sampling_dt_ms (1000*imu_sampling_dt)


#define WP_Duty_Freq  200
#define WP_Duty_Dt  (1.0f/WP_Duty_Freq)
#define WP_Duty_Dt_Ms  (WP_Duty_Dt*1000)
#define Delta WP_Duty_Dt


#define Baro_Update_Dt  50.0f//20ms����


#define player_level_default 3//0:��ͨ���
															//1:רҵ���
															//2:Ӳ�����1
															//3:Ӳ�����2
															//4:Ӳ�����3

#define gyro_lpf_param_default1 	  50
#define accel_lpf_param_default1 	  30
#define ins_lpf_param_default1 		  30
#define accel_fb_lpf_param_default1 5

#define gyro_lpf_param_default2 		95//60
#define accel_lpf_param_default2 	  30//20
#define ins_lpf_param_default2 		  60//60
#define accel_fb_lpf_param_default2 5//10


typedef   signed           char int8;
typedef unsigned           char u8;
typedef unsigned           char uint8;
typedef unsigned           char byte;
typedef   signed short     int int16;
typedef unsigned short     int uint16;
typedef unsigned short     int u16;
typedef unsigned long     int u32; 


typedef enum
{
	WHO_AM_I_MPU6050  =0x68,
	WHO_AM_I_ICM20689 =0x98,
	WHO_AM_I_ICM20608D=0xAE,
	WHO_AM_I_ICM20608G=0xAF,
	WHO_AM_I_ICM20602=0x12,
}IMU_ID_READ;



#include "hw_memmap.h"
#include "hw_types.h"
#include "hw_ints.h"
#include "debug.h"
#include "fpu.h"
#include "gpio.h"
#include "pin_map.h"
#include "pwm.h"
#include "rom.h"
#include "sysctl.h"
#include "uart.h"
#include "interrupt.h"
#include "timer.h"
#include "hw_gpio.h"
#include "eeprom.h"
/////////////////////////
#include "WP_DataType.h"
#include "Usart.h"
#include "NCLink.h"
#include "uartstdio.h"
#include "Time.h"
#include "Time_Cnt.h"
#include "Schedule.h"
#include "License.h"
#include "ssd1306.h"
#include "oled.h"
#include "ICM2068X.h"
#include "Soft_I2C.h"
#include "ist8310.h"
#include "QMC5883.h"
#include "spl06.h"
#include "PPM.h"
#include "wp_flash.h"
#include "PID.h"
#include "Ringbuf.h"
#include "WP_PWM.h"
#include "Sensor.h"
#include "Key.h"
#include "Filter.h"
#include "SINS.h"
#include "Calibration.h"
#include "CalibrationRoutines.h"
#include "US_100.h"
#include "tfmini.h"
#include "TOFSensor.h"
#include "NamelessCotrun_OpticalFlow.h"
#include "OpticalFlow_LC306.h"
#include "OpticalFlow_Control.h"
#include "NamelessCotrun_SDK.h"
#include "WP_ADC.h"
#include "Bling.h"
#include "GPS.h"
#include "RC.h"
#include "SYSTEM.h"
#include "WP_Ctrl.h"
#include "control_althold.h"
#include "control_poshold.h"
#include "control_land.h"
#include "control_config.h"
#include "Developer_Mode.h"
#include "ADC_Button.h"
#include "Temperature_Ctrl.h"
#include "Compass.h"
#include "Reserved_IO.h"
#include "Reserved_Serialport.h"
#include "Subtask_Demo.h"
#include "LSN10.h"
#include "speaker.h"

extern SINS NamelessQuad;
extern Sensor WP_Sensor;
extern AHRS WP_AHRS;
#endif




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




