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
#include "uart.h"
#include "Usart.h"
#include "Ringbuf.h"


void UART6_IRQHandler(void);


//����ѭ�����л������ݶ���
RingBuff_t COM0_Rx_Buf,COM1_Rx_Buf,COM2_Rx_Buf,COM3_Rx_Buf,COM4_Rx_Buf,COM5_Rx_Buf,COM6_Rx_Buf,COM7_Rx_Buf;
/***********************************************************
@��������UART0_IRQHandler
@��ڲ�������
@���ڲ�������
��������������0���ݽ���
@���ߣ�����С��
@���ڣ�2024/01/20
*************************************************************/
void UART0_IRQHandler(void)//UART0�жϺ���
{	
  //��ȡ�жϱ�־ ԭʼ�ж�״̬ �������жϱ�־		
  uint32_t flag = UARTIntStatus(UART0_BASE,1);
  //����жϱ�־	
  UARTIntClear(UART0_BASE,flag);		
  //�ж�FIFO�Ƿ�������		
  while(UARTCharsAvail(UART0_BASE))		
  {
		uint8_t ch=UARTCharGet(UART0_BASE);
		SDK_Data_Receive_Prepare_2(ch);		
//		RingBuf_Write(ch,&COM0_Rx_Buf,SDK_Target_Length*2);//�����ζ�������д����
//		if(COM0_Rx_Buf.Ring_Buff[0]!=0xFF)
//		{
//			COM0_Rx_Buf.Head=1;
//			COM0_Rx_Buf.Tail=0; 
//		}		
  }
}

/***********************************************************
@��������ConfigureUART0
@��ڲ�������
@���ڲ�������
��������������0����
@���ߣ�����С��
@���ڣ�2024/01/20
*************************************************************/
void ConfigureUART0(unsigned long bound)//����0��ʼ��
{
  SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);//ʹ��GPIO����		
  SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);//ʹ��UART����
  GPIOPinConfigure(GPIO_PA0_U0RX);//GPIOģʽ���� PA0--RX PA1--TX 
  GPIOPinConfigure(GPIO_PA1_U0TX);
  GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);//GPIO��UARTģʽ����
	UARTConfigSetExpClk(UART0_BASE, SysCtlClockGet(), bound,
										(UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE |
										 UART_CONFIG_PAR_NONE));
	
  //UARTЭ������ ������115200 8λ 1ֹͣλ  ��У��λ	
  //UART����FIFO Ĭ��FIFO LevelΪ4/8 �Ĵ�����8�ֽں�����ж�	//���ú����1λ�Ͳ����ж�	
  UARTFIFODisable(UART0_BASE);//ʹ��UART0�ж�	IntEnable(INT_UART0);	
  UARTIntEnable(UART0_BASE,UART_INT_RX);//ʹ��UART0�����ж�		
  UARTIntRegister(UART0_BASE,UART0_IRQHandler);//UART�жϵ�ַע��	
  IntPrioritySet(INT_UART0, USER_INT2);
}

/***********************************************************
@��������USART0_Send
@��ڲ�����uint8_t *pui8Buffer, uint32_t ui32Count
@���ڲ�������
��������������0����N���ֽ�����
@���ߣ�����С��
@���ڣ�2024/01/20
*************************************************************/
void USART0_Send(uint8_t *pui8Buffer, uint32_t ui32Count)//����N���ֽڳ��ȵ�����
{
  while(ui32Count--)
  {
    UARTCharPut(UART0_BASE, *pui8Buffer++);
  }
}
/***********************************************************
@��������wust_sendware
@��ڲ�����unsigned char *wareaddr, int16_t waresize
@���ڲ�������
����������ɽ������ʾ��������
@���ߣ�����С��
@���ڣ�2024/01/20
*************************************************************/
void wust_sendware(unsigned char *wareaddr, int16_t waresize)//ɽ�ⷢ�Ͳ���
{
#define CMD_WARE     3
  uint8 cmdf[2] = {CMD_WARE, ~CMD_WARE};//֡ͷ
  uint8 cmdr[2] = {~CMD_WARE, CMD_WARE};//֡β
  USART1_Send(cmdf, sizeof(cmdf));
  USART1_Send(wareaddr, waresize);
  USART1_Send(cmdr, sizeof(cmdr));
}

/***********************************************************
@��������UART1_IRQHandler
@��ڲ�������
@���ڲ�������
��������������1���ݽ���
@���ߣ�����С��
@���ڣ�2024/01/20
*************************************************************/
void UART1_IRQHandler(void)//UART1�жϺ���
{				
  uint32_t flag = UARTIntStatus(UART1_BASE,1);//��ȡ�жϱ�־ ԭʼ�ж�״̬ �����жϱ�־	
  UARTIntClear(UART1_BASE,flag);//����жϱ�־	
  while(UARTCharsAvail(UART1_BASE))//�ж�FIFO�Ƿ�������		
  {			
    //RingBuf_Write(UARTCharGet(UART1_BASE),&COM1_Rx_Buf,100);//�����ζ�������д����	
    NCLink_Data_Prase_Prepare(UARTCharGet(UART1_BASE)); 		
  }
}


/***********************************************************
@��������USART1_Send
@��ڲ�����uint8_t *pui8Buffer, uint32_t ui32Count
@���ڲ�������
��������������1����N���ֽ�����
@���ߣ�����С��
@���ڣ�2024/01/20
*************************************************************/
void USART1_Send(uint8_t *pui8Buffer, uint32_t ui32Count)//����N���ֽڳ��ȵ�����
{
  while(ui32Count--)
  {
    UARTCharPut(UART1_BASE, *pui8Buffer++);
  }
}
/***********************************************************
@��������ConfigureUART1
@��ڲ�������
@���ڲ�������
��������������1����
@���ߣ�����С��
@���ڣ�2024/01/20
*************************************************************/
void ConfigureUART1(unsigned long bound)//����1��ʼ��
{
  SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);//ʹ��GPIO����		
  SysCtlPeripheralEnable(SYSCTL_PERIPH_UART1);//ʹ��UART����
  GPIOPinConfigure(GPIO_PB0_U1RX);//GPIOģʽ���� PB0--RX PB1--TX 
  GPIOPinConfigure(GPIO_PB1_U1TX);
  GPIOPinTypeUART(GPIO_PORTB_BASE, GPIO_PIN_0 | GPIO_PIN_1);//GPIO��UARTģʽ����
	UARTConfigSetExpClk(UART1_BASE, SysCtlClockGet(), bound,
                      (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE |
                       UART_CONFIG_PAR_NONE));
  UARTFIFODisable(UART1_BASE);//ʹ��UART1�ж�	
  UARTIntEnable(UART1_BASE,UART_INT_RX);//ʹ��UART1�����ж�		
  UARTIntRegister(UART1_BASE,UART1_IRQHandler);//UART1�жϵ�ַע��	
  IntPrioritySet(INT_UART1, USER_INT3);
}


/***********************************************************
@��������UART2_IRQHandler
@��ڲ�������
@���ڲ�������
��������������2���ݽ���
@���ߣ�����С��
@���ڣ�2024/01/20
*************************************************************/
void UART2_IRQHandler(void)
{
  uint32_t flag = UARTIntStatus(UART2_BASE,1);//��ȡ�жϱ�־ ԭʼ�ж�״̬ �����жϱ�־		
  UARTIntClear(UART2_BASE,flag);//����жϱ�־	
  while(UARTCharsAvail(UART2_BASE))//�ж�FIFO�Ƿ�������				
  {		
		uint8_t ch=UARTCharGet(UART2_BASE);
		switch(Uart2_Mode)
		{
			case 0x00:
				LC30X_OpticalFlow_Sense_Prase(ch,lc30x_buf);
			break;
			case 0x01:
				RingBuf_Write(ch,&COM2_Rx_Buf,58*2);//�����ζ�������д����	
				if(COM2_Rx_Buf.Ring_Buff[0]!=0xA5)
				{
					COM2_Rx_Buf.Head=1;
					COM2_Rx_Buf.Tail=0; 
				}	
			break;
			default:LC30X_OpticalFlow_Sense_Prase(ch,lc30x_buf);
		}		
  }
}

/***********************************************************
@��������USART2_Send
@��ڲ�����uint8_t *pui8Buffer, uint32_t ui32Count
@���ڲ�������
��������������2����N���ֽ�����
@���ߣ�����С��
@���ڣ�2024/01/20
*************************************************************/
void USART2_Send(uint8_t *pui8Buffer, uint32_t ui32Count)//����N���ֽڳ��ȵ�����
{
  while(ui32Count--)
  {
    UARTCharPut(UART2_BASE, *pui8Buffer++);
  }
}
/***********************************************************
@��������ConfigureUART2
@��ڲ�������
@���ڲ�������
��������������2����
@���ߣ�����С��
@���ڣ�2024/01/20
*************************************************************/
void ConfigureUART2(unsigned long bound)//����6��ʼ��
{
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);//ʹ��GPIO����		
  SysCtlPeripheralEnable(SYSCTL_PERIPH_UART2);//ʹ��UART����
  
  HWREG(GPIO_PORTD_BASE + GPIO_O_LOCK) = GPIO_LOCK_KEY;//����PD6
  HWREG(GPIO_PORTD_BASE + GPIO_O_CR) |= 0x80;//ȷ��
  HWREG(GPIO_PORTD_BASE + GPIO_O_LOCK) = 0;//��������
  
  GPIOPinConfigure(GPIO_PD6_U2RX);//GPIOģʽ���� PD6--RX PD7--TX 
  GPIOPinConfigure(GPIO_PD7_U2TX);
  GPIOPinTypeUART(GPIO_PORTD_BASE, GPIO_PIN_6 | GPIO_PIN_7);//GPIO��UARTģʽ����
  UARTConfigSetExpClk(UART2_BASE, SysCtlClockGet(), bound,
                      (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE |
                       UART_CONFIG_PAR_NONE));
  UARTFIFODisable(UART2_BASE);//ʹ��UART2�ж�	
  UARTIntEnable(UART2_BASE,UART_INT_RX);//ʹ��UART6�����ж�		
	
  OpticalFlow_Init();//�����˲�������ʼ��
	
  UARTIntRegister(UART2_BASE,UART2_IRQHandler);//UART�жϵ�ַע��	
  IntPrioritySet(INT_UART2, USER_INT4);
}

/***********************************************************
@��������UART3_IRQHandler
@��ڲ�������
@���ڲ�������
��������������3���ݽ���
@���ߣ�����С��
@���ڣ�2024/01/20
*************************************************************/
void UART3_IRQHandler(void)
{		
  uint32_t flag = UARTIntStatus(UART3_BASE,1);//��ȡ�жϱ�־ ԭʼ�ж�״̬ �����жϱ�־		
  UARTIntClear(UART3_BASE,flag);//����жϱ�־			
  while(UARTCharsAvail(UART3_BASE))//�ж�FIFO�Ƿ�������		
  {	
		uint8_t ch=UARTCharGet(UART3_BASE);
		SDK_Data_Receive_Prepare_1(ch);
//		RingBuf_Write(ch,&COM3_Rx_Buf,SDK_Target_Length*2);//�����ζ�������д����
//		if(COM3_Rx_Buf.Ring_Buff[0]!=0xFF)
//		{
//			COM3_Rx_Buf.Head=1;
//			COM3_Rx_Buf.Tail=0; 
//		}	
	}	
}

/***********************************************************
@��������USART3_Send
@��ڲ�����uint8_t *pui8Buffer, uint32_t ui32Count
@���ڲ�������
��������������3����N���ֽ�����
@���ߣ�����С��
@���ڣ�2024/01/20
*************************************************************/
void USART3_Send(uint8_t *pui8Buffer, uint32_t ui32Count)//����N���ֽڳ��ȵ�����
{
  while(ui32Count--)
  {
    UARTCharPut(UART3_BASE, *pui8Buffer++);
  }
}

/***********************************************************
@��������ConfigureUART3
@��ڲ�������
@���ڲ�������
��������������3����
@���ߣ�����С��
@���ڣ�2024/01/20
*************************************************************/
void ConfigureUART3(void)//����3��ʼ��
{
  SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOC);//ʹ��GPIO����		
  SysCtlPeripheralEnable(SYSCTL_PERIPH_UART3);//ʹ��UART����
  GPIOPinConfigure(GPIO_PC6_U3RX);//GPIOģʽ���� PC6--RX PC7--TX 
  GPIOPinConfigure(GPIO_PC7_U3TX);
  GPIOPinTypeUART(GPIO_PORTC_BASE, GPIO_PIN_6 | GPIO_PIN_7);//GPIO��UARTģʽ����
  UARTConfigSetExpClk(UART3_BASE, SysCtlClockGet(), 256000,
                      (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE |
                       UART_CONFIG_PAR_NONE));
  UARTFIFODisable(UART3_BASE);//ʹ��UART0�ж�	
  UARTIntEnable(UART3_BASE,UART_INT_RX);//ʹ��UART3�����ж�		
  UARTIntRegister(UART3_BASE,UART3_IRQHandler);//UART3�жϵ�ַע��	
  IntPrioritySet(INT_UART3, USER_INT2);
}

/***********************************************************
@��������UART6_IRQHandler
@��ڲ�������
@���ڲ�������
��������������6���ݽ���
@���ߣ�����С��
@���ڣ�2024/01/20
*************************************************************/
uint16_t UART6_LEN=0;
void UART6_IRQHandler(void)
{		
  uint32_t flag = UARTIntStatus(UART6_BASE,1);//��ȡ�жϱ�־ ԭʼ�ж�״̬ �����жϱ�־		
  UARTIntClear(UART6_BASE,flag);//����жϱ�־	
  while(UARTCharsAvail(UART6_BASE))//�ж�FIFO�Ƿ�������		
  {
		switch(Reserved_Uart)
		{
			case FRONT_RANGE_FINDER:
			{
				RingBuf_Write(UARTCharGet(UART6_BASE),&COM6_Rx_Buf,UART6_LEN);//�����ζ�������д����
			}
			break;
			case GPS_M8N:
			{
				RingBuf_Write(UARTCharGet(UART6_BASE),&COM6_Rx_Buf,UART6_LEN);//�����ζ�������д����
				if(COM6_Rx_Buf.Ring_Buff[0]!=0XB5)
				{
					COM6_Rx_Buf.Head=1;
					COM6_Rx_Buf.Tail=0; 
				}
			}			
			break;
			case THIRD_PARTY_STATE:
			{
				//uint8_t data=UARTCharGet(UART6_BASE);
				//RingBuf_Write(data,&COM6_Rx_Buf,UART6_LEN);//�����ζ�������д����
				//if(COM6_Rx_Buf.Ring_Buff[0]!=0xFC)	{COM6_Rx_Buf.Head=1;COM6_Rx_Buf.Tail=0;}
				NCLink_Data_Prase_Prepare_Lite(UARTCharGet(UART6_BASE)); 
			}
			break;
			default:
			{
				RingBuf_Write(UARTCharGet(UART6_BASE),&COM6_Rx_Buf,UART6_LEN);//�����ζ�������д����
				if(COM6_Rx_Buf.Ring_Buff[0]!=0XB5)
				{
					COM6_Rx_Buf.Head=1;
					COM6_Rx_Buf.Tail=0; 
				}
			}	
		}		
  }
}

/***********************************************************
@��������USART6_Send
@��ڲ�����uint8_t *pui8Buffer, uint32_t ui32Count
@���ڲ�������
��������������6����N���ֽ�����
@���ߣ�����С��
@���ڣ�2024/01/20
*************************************************************/
void USART6_Send(uint8_t *pui8Buffer, uint32_t ui32Count)//����N���ֽڳ��ȵ�����
{
  while(ui32Count--)
  {
    UARTCharPut(UART6_BASE, *pui8Buffer++);
  }
}


/***********************************************************
@��������ConfigureUART6
@��ڲ�������
@���ڲ�������
��������������6����
@���ߣ�����С��
@���ڣ�2024/01/20
*************************************************************/
void ConfigureUART6(unsigned long bound,uint16_t len)
{
	UART6_LEN=len;
  SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);//ʹ��GPIO����		
  SysCtlPeripheralEnable(SYSCTL_PERIPH_UART6);//ʹ��UART����
  
	HWREG(GPIO_PORTD_BASE + GPIO_O_LOCK) = GPIO_LOCK_KEY;//����PD6
	HWREG(GPIO_PORTD_BASE + GPIO_O_CR) |= 0x80;//ȷ��
	HWREG(GPIO_PORTD_BASE + GPIO_O_LOCK) = 0;//��������
	
	GPIOPinConfigure(GPIO_PD4_U6RX);//GPIOģʽ���� PD4--RX PD5--TX 
  GPIOPinConfigure(GPIO_PD5_U6TX);
  GPIOPinTypeUART(GPIO_PORTD_BASE, GPIO_PIN_4 | GPIO_PIN_5);//GPIO��UARTģʽ����
  UARTConfigSetExpClk(UART6_BASE, SysCtlClockGet(), bound,
                      (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE |
                       UART_CONFIG_PAR_NONE));
  UARTFIFODisable(UART6_BASE);//ʹ��UART0�ж�	
  UARTIntEnable(UART6_BASE,UART_INT_RX);//ʹ��UART6�����ж�		
	
  UARTIntRegister(UART6_BASE,UART6_IRQHandler);//UART6�жϵ�ַע��	
  IntPrioritySet(INT_UART6, USER_INT1);
}





/***********************************************************
@��������UART7_IRQHandler
@��ڲ�������
@���ڲ�������
��������������7���ݽ���
@���ߣ�����С��
@���ڣ�2024/01/20
*************************************************************/
void UART7_IRQHandler(void)//UART2�жϺ���
{		
  uint32_t flag = UARTIntStatus(UART7_BASE,1);//��ȡ�жϱ�־ ԭʼ�ж�״̬ �����жϱ�־	
  UARTIntClear(UART7_BASE,flag);//����жϱ�־		
  while(UARTCharsAvail(UART7_BASE))//�ж�FIFO�Ƿ�������			
  {			
		switch(rangefinder_current)
		{
			case US100:
			{
				RingBuf_Write(UARTCharGet(UART7_BASE),&COM7_Rx_Buf,4);//�����ζ�������д����
			}			
			break;
			case TFMINI:
			{
				RingBuf_Write(UARTCharGet(UART7_BASE),&COM7_Rx_Buf,18);//�����ζ�������д����
				if(COM7_Rx_Buf.Ring_Buff[0]!=0x59)
				{
					COM7_Rx_Buf.Head=1;
					COM7_Rx_Buf.Tail=0; 
				}	
			}
			break;	
			case TOFSENSE:
			{
				uint8_t data=UARTCharGet(UART7_BASE);
				RingBuf_Write(data,&COM7_Rx_Buf,16*TOFSENSE_CURRENT_WORK);//�����ζ�������д����  32
				TOFSense_Prase(data);
				if(COM7_Rx_Buf.Ring_Buff[0]!=0x57)
				{
					COM7_Rx_Buf.Head=1;
					COM7_Rx_Buf.Tail=0; 
				}	
			}
			break;	
			case SMD15:
			{
				sdm15_prase(UARTCharGet(UART7_BASE));
			}
			break;
			case TOFSENSE_M:
			{
				TOFSense_M_Prase(UARTCharGet(UART7_BASE));
			}
			break;
			default:{RingBuf_Write(UARTCharGet(UART7_BASE),&COM7_Rx_Buf,4);}	//�����ζ�������д����	
		}
  }
}

/***********************************************************
@��������USART7_Send
@��ڲ�����uint8_t *pui8Buffer, uint32_t ui32Count
@���ڲ�������
��������������7����N���ֽ�����
@���ߣ�����С��
@���ڣ�2024/01/20
*************************************************************/
void USART7_Send(uint8_t *pui8Buffer, uint32_t ui32Count)//����N���ֽڳ��ȵ�����
{
  while(ui32Count--)
  {
    UARTCharPut(UART7_BASE, *pui8Buffer++);
  }
}

void ConfigureUART7(unsigned long baud)//����7��ʼ��
{
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);//ʹ��GPIO����		
	SysCtlPeripheralEnable(SYSCTL_PERIPH_UART7);//ʹ��UART����
	GPIOPinConfigure(GPIO_PE0_U7RX);//GPIOģʽ���� PE0--RX PE1--TX 
	GPIOPinConfigure(GPIO_PE1_U7TX);
	GPIOPinTypeUART(GPIO_PORTE_BASE, GPIO_PIN_0 | GPIO_PIN_1);//GPIO��UARTģʽ����
	UARTConfigSetExpClk(UART7_BASE, SysCtlClockGet(), baud,
										(UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE |
										 UART_CONFIG_PAR_NONE));
	RingBuff_Init(&COM7_Rx_Buf); 	
	UARTFIFODisable(UART7_BASE);//ʹ��UART0�ж�	
	UARTIntEnable(UART7_BASE,UART_INT_RX);//ʹ��UART0�����ж�		
	UARTIntRegister(UART7_BASE,UART7_IRQHandler);//UART�жϵ�ַע��	
	IntPrioritySet(INT_UART7, USER_INT3);
}




/***********************************************************
@��������Vcan_Send
@��ڲ�������
@���ڲ�������
����������ɽ�����ݷ��ͺ�����Ĭ�Ϸ���8��ͨ������������Ϊfloat
��ÿ��ͨ�����ݿ����Լ�����
@���ߣ�����С��
@���ڣ�2024/01/20
*************************************************************/
void Vcan_Send(void)//ɽ�����վ����
{
  static float DataBuf[8];	
	DataBuf[0]=Receiver_PPM_Databuf[0];
  DataBuf[1]=Receiver_PPM_Databuf[1];
  DataBuf[2]=Receiver_PPM_Databuf[2];
  DataBuf[3]=Receiver_PPM_Databuf[3];
  DataBuf[4]=Receiver_PPM_Databuf[4];
  DataBuf[5]=Receiver_PPM_Databuf[5];
  DataBuf[6]=Receiver_PPM_Databuf[6];
  DataBuf[7]=Receiver_PPM_Databuf[7];
  wust_sendware((unsigned char *)DataBuf,sizeof(DataBuf));
}




