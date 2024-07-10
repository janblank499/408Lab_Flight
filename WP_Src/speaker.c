#include "Headfile.h"
#include "speaker.h"


extern void NCLink_GS_Prase_Data(uint8_t data);

#if FLIGHT_ESC_PWM==0
void UART4_IRQHandler(void)//UART1�жϺ���
{				
  uint32_t flag = UARTIntStatus(UART4_BASE,1);//��ȡ�жϱ�־ ԭʼ�ж�״̬ �����жϱ�־		
  UARTIntClear(UART4_BASE,flag);//����жϱ�־	
  while(UARTCharsAvail(UART4_BASE))//�ж�FIFO�Ƿ�������		
  {
		uint8_t ch=UARTCharGet(UART4_BASE);
		NCLink_GS_Prase_Data(ch);
  }
}


void ConfigureUART_Speaker(unsigned long bound)//����4��ʼ��
{
  SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOC);//ʹ��GPIO����		
  SysCtlPeripheralEnable(SYSCTL_PERIPH_UART4);//ʹ��UART����
  GPIOPinConfigure(GPIO_PC4_U4RX);//GPIOģʽ���� PC4--RX PC5--TX 
  GPIOPinConfigure(GPIO_PC5_U4TX);
  GPIOPinTypeUART(GPIO_PORTC_BASE, GPIO_PIN_4 | GPIO_PIN_5);//GPIO��UARTģʽ����
	UARTClockSourceSet(UART4_BASE, UART_CLOCK_PIOSC);
	UARTStdioConfig(0, bound, 16000000);
  UARTFIFODisable(UART4_BASE);//ʹ��UART4�ж�	
  UARTIntEnable(UART4_BASE,UART_INT_RX);//ʹ��UART1�����ж�		
  UARTIntRegister(UART4_BASE,UART4_IRQHandler);//UART1�жϵ�ַע��	
  IntPrioritySet(INT_UART4, USER_INT2);
} 



void Speaker_Send(uint8_t *pui8Buffer, uint32_t ui32Count)//����N���ֽڳ��ȵ�����
{
  while(ui32Count--)
  {
    UARTCharPut(UART4_BASE, *pui8Buffer++);
  }
}
#else
void UART5_IRQHandler(void)//UART1�жϺ���
{				
  uint32_t flag = UARTIntStatus(UART5_BASE,1);//��ȡ�жϱ�־ ԭʼ�ж�״̬ �����жϱ�־		
  UARTIntClear(UART5_BASE,flag);//����жϱ�־	
  while(UARTCharsAvail(UART5_BASE))//�ж�FIFO�Ƿ�������		
  {
		uint8_t ch=UARTCharGet(UART5_BASE);
		NCLink_GS_Prase_Data(ch);
  }
}


void ConfigureUART_Speaker(unsigned long bound)//����4��ʼ��
{
  SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);//ʹ��GPIO����		
  SysCtlPeripheralEnable(SYSCTL_PERIPH_UART5);//ʹ��UART����
  GPIOPinConfigure(GPIO_PE4_U5RX);//GPIOģʽ���� PC4--RX PC5--TX 
  GPIOPinConfigure(GPIO_PE5_U5TX);
  GPIOPinTypeUART(GPIO_PORTE_BASE, GPIO_PIN_4 | GPIO_PIN_5);//GPIO��UARTģʽ����
	UARTClockSourceSet(UART5_BASE, UART_CLOCK_PIOSC);
	UARTStdioConfig(1, bound, 16000000);
  UARTFIFODisable(UART5_BASE);//ʹ��UART5�ж�	
  UARTIntEnable(UART5_BASE,UART_INT_RX);//ʹ��UART1�����ж�		
  UARTIntRegister(UART5_BASE,UART5_IRQHandler);//UART1�жϵ�ַע��	
  IntPrioritySet(INT_UART5, USER_INT2);
} 


void Speaker_Send(uint8_t *pui8Buffer, uint32_t ui32Count)//����N���ֽڳ��ȵ�����
{
  while(ui32Count--)
  {
    UARTCharPut(UART5_BASE, *pui8Buffer++);
  }
}
#endif





/**************оƬ��������*********************/
uint8_t SYN_StopCom[]={0xFD,0X00,0X01,0X02};//ֹͣ�ϳ�
uint8_t SYN_SuspendCom[]={0XFD,0X00,0X01,0X03};//��ͣ�ϳ�
uint8_t SYN_RecoverCom[]={0XFD,0X00,0X01,0X04};//�ָ��ϳ�
uint8_t SYN_ChackCom[]={0XFD,0X00,0X01,0X21};//״̬��ѯ
uint8_t SYN_PowerDownCom[]={0XFD,0X00,0X01,0X88};//����POWER DOWN ״̬����



void syn6658_frame_package(uint8_t *data)
{
	/****************��Ҫ���͵��ı�**********************************/ 
	 unsigned char frame_info[50]; //������ı�����
	 unsigned int len;  
	 len=strlen((const char *)data); 			//��Ҫ�����ı��ĳ���
	/*****************֡�̶�������Ϣ**************************************/           
	 frame_info[0]=0xFD ; 		//����֡ͷFD
	 frame_info[1]=0x00 ; 		//�������������ȵĸ��ֽ�
	 frame_info[2]=len+2; 		//�������������ȵĵ��ֽ�
	 frame_info[3]=0x01 ; 		//���������֣��ϳɲ�������		 		 
	 frame_info[4]=0x01;      //�ı������ʽ��GBK 
	/*******************����֡��Ϣ***************************************/		  
	 memcpy(&frame_info[5], data, len);
	 Speaker_Send(frame_info,5+len); //����֡����
}


uint16_t speaker_mode=0;
void speaker_notify_run(uint16_t *mode)
{
	switch(*mode)
	{
		case 0:
		{
			//����
		}
		break;
		case 1:
		{
			*mode=0;
			syn6658_frame_package((uint8_t *)"sound124 �������ʹ���ȡ��");
		}
		break;
		default:
		{
			//����
		}
	}
	
}

