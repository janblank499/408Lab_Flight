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
#include "Soft_I2C.h"
#include "myiic.h"

//#define USE_SOFT_I2C

#define SYSCTL_PERIPH_GPIO_I2C SYSCTL_PERIPH_GPIOB
#define GPIO_STRENGTH_I2C GPIO_STRENGTH_4MA
#define GPIO_I2C   		GPIO_PORTB_BASE 
#define SCL_PIN   		GPIO_PIN_2
#define SDA_PIN   		GPIO_PIN_3
#define I2C_READ_SDA  GPIOPinRead(GPIO_I2C,SDA_PIN)			//SDA 
#define I2C_SDA_H  		GPIOPinWrite(GPIO_I2C,SDA_PIN,SDA_PIN)//SDA 
#define I2C_SDA_L 		GPIOPinWrite(GPIO_I2C,SDA_PIN,0)			//SDA
#define I2C_SCL_H  		GPIOPinWrite(GPIO_I2C,SCL_PIN,SCL_PIN)//SCL
#define I2C_SCL_L  		GPIOPinWrite(GPIO_I2C,SCL_PIN,0)			//SCL


void I2C_GPIO_Config(void)
{
#if defined(USE_SOFT_I2C)	
  SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIO_I2C);
	
	HWREG(GPIO_I2C + GPIO_O_LOCK) = GPIO_LOCK_KEY;//����
	HWREG(GPIO_I2C + GPIO_O_CR) |= 0x000000FF;//ȷ��
	HWREG(GPIO_I2C + GPIO_O_LOCK) = 0;//��������
	
  GPIOPinTypeGPIOOutput(GPIO_I2C, SCL_PIN);
  GPIOPinTypeGPIOOutput(GPIO_I2C, SDA_PIN);
	GPIOPadConfigSet(GPIO_I2C,SCL_PIN,GPIO_STRENGTH_I2C,GPIO_PIN_TYPE_STD);
	GPIOPadConfigSet(GPIO_I2C,SDA_PIN,GPIO_STRENGTH_I2C,GPIO_PIN_TYPE_STD);
	
  Delay_Ms(10);
  GPIOPinWrite(GPIO_I2C, SCL_PIN, SCL_PIN);//���ø�
  GPIOPinWrite(GPIO_I2C, SDA_PIN, SDA_PIN);//���ø�
#else
  Init_I2C0();
#endif	
}

#if defined(USE_SOFT_I2C)
void I2C_SDA_OUT(void)
{
	GPIOPinTypeGPIOOutput(GPIO_I2C, SDA_PIN);
	GPIOPadConfigSet(GPIO_I2C,SDA_PIN,GPIO_STRENGTH_I2C,GPIO_PIN_TYPE_STD);///GPIO_PIN_TYPE_STD);
}

void I2C_SDA_IN(void)
{
  GPIOPinTypeGPIOInput(GPIO_I2C, SDA_PIN);
	GPIOPadConfigSet(GPIO_I2C,SDA_PIN,GPIO_STRENGTH_I2C,GPIO_PIN_TYPE_STD_WPU);
}

#define SDA_OUT I2C_SDA_OUT()
#define SDA_IN  I2C_SDA_IN()
#define sdaRead I2C_READ_SDA  
#define SDA_H   I2C_SDA_H  		 
#define SDA_L   I2C_SDA_L
#define SCL_H   I2C_SCL_H  		
#define SCL_L   I2C_SCL_L  		

uint16_t  hu=5;
static void i2cDelay()
{
//  volatile int i = 100;//7
//  while (i)
//  i--;
	//delay_us(hu);
	__nop();
	__nop();
	__nop();
	__nop();
	__nop();
	__nop();
	__nop();
	__nop();
	__nop();
		__nop();
	__nop();
	__nop();
	__nop();
	__nop();
	__nop();
	__nop();
	__nop();
	__nop();
		__nop();
	__nop();
	__nop();
	__nop();
	__nop();
	__nop();
	__nop();
	__nop();
	__nop();
		__nop();
	__nop();
	__nop();
	__nop();
	__nop();
	__nop();
	__nop();
	__nop();
	__nop();
}
 
// SCL�ߵ�ƽ�ڼ䣬SDA�����½���Ϊ��ʼ�ź�
static bool i2cStart()
{
    SDA_OUT;
    SDA_H;
	  SCL_H;   
    i2cDelay();
    if (!sdaRead)  // ���SDAΪ�͵�ƽ��������æ���˳�
        return false;
    SDA_L;
    if (sdaRead)  // ���SDAΪ�ߵ�ƽ��������æ���˳�
        return false;
    SDA_L;
    return true;
}
 
// SCL�ߵ�ƽ�ڼ䣬SDA����������Ϊֹͣ�ź�
static void i2cStop(void)
{
    SDA_OUT;
    SDA_L;
	  SCL_L;  
    i2cDelay();  // STOP:when CLK is high DATA from low to high 
    //SCL_H;
    SDA_H;  
    i2cDelay();
}
 
static void i2cAck(void)
{
    SCL_L;
	  SDA_OUT; 
    SDA_L;
    i2cDelay();   
    SCL_H;
    i2cDelay();
    SCL_L;
}
 
static void i2cNoAck(void)
{
    SCL_L;
	  SDA_OUT;
    SDA_H;
    i2cDelay();
    SCL_H;
    i2cDelay();
    SCL_L;
}
 
// SCL�ߵ�ƽ�ڼ䣬SDA��ƽ�����豸���ͱ�ʾӦ��
static bool i2cWaitAck(void)
{
    uint16_t errTimes = 0;
    SDA_H;
	  i2cDelay();
	  SDA_IN;
    SCL_H;
    i2cDelay();
    while (sdaRead) {
        if (errTimes++ > 200) //20
				{
            SCL_L;
            return false;
        }           
        i2cDelay();
    }
    SCL_L;
    return true;
}
 
// �������ݣ����ݴӸ�λ����λ����  
static void i2cSendByte(uint8_t byte)  
{
    uint8_t i = 8;
 
    SDA_OUT;
    while (i--) {      
        SCL_L;  // ʱ���ź�Ϊ�͵�ƽ�ڼ䣬���������ߵ�ƽ�仯
        i2cDelay();
        if (byte & 0x80)
            SDA_H;
        else
            SDA_L; 
        byte <<= 1; 
        i2cDelay();
        SCL_H;
        i2cDelay();
    }
    SCL_L;
}
 
static uint8_t i2cReceiveByte()  
{
    uint8_t i = 8;
    uint8_t byte = 0;
    SDA_IN;
    SDA_H;
    while (i--) {
        byte <<= 1;
        SCL_H;
        i2cDelay();
        if (sdaRead) {
            byte |= 0x01;
        }
        SCL_L;
        i2cDelay();
    }
    SCL_L;
    return byte; 
}
 
 

 
/**
 * ͨ��I2C����дһ�ֽ�����
 * @param[in] dev:�豸I2C��ַ
 * @param[in] reg:�Ĵ�����ַ
 * @param[in] data:Ҫд�������
 */
bool i2cWriteOneByte(uint8_t dev, uint8_t reg, uint8_t data)
{
    if (!i2cStart())        
        return false;
    i2cSendByte(dev << 1);  // �ӻ���ַ�ɸ�7λ+��дλ����   
    if (!i2cWaitAck()) {     
        i2cStop();
        return false;
    }
    i2cSendByte(reg);       
    i2cWaitAck();
    i2cSendByte(data);     
    i2cWaitAck();
    return true;
}
 

/**
 *  
 * @param[in] dev:�豸I2C��ַ
 * @param[in] reg:�Ĵ�����ַ
 * @param[in] len:�ֽ��� 
 * @param[in] data:��д������� 
 */
uint8_t i2cReadOneBytes(uint8_t dev, uint8_t reg)
{
	unsigned char REG_data;
  i2cStart();
  i2cSendByte(dev<<1);
	i2cWaitAck();
  i2cSendByte(reg);
	i2cWaitAck();
  i2cStart();
  i2cSendByte((dev << 1) | 0x01);
  i2cWaitAck();
	REG_data=i2cReceiveByte();
  i2cStop();
  return REG_data;	
} 


/**
 *  
 * @param[in] dev:�豸I2C��ַ
 * @param[in] reg:�Ĵ�����ַ
 * @param[in] len:�ֽ��� 
 * @param[in] data:��д������� 
 */
bool i2cWriteBytes(uint8_t dev, uint8_t reg, uint8_t len, uint8_t *data)
{
    uint8_t i;
 
    if (!i2cStart())        
        return false;
    i2cSendByte(dev << 1);          
    if (!i2cWaitAck()) {     
        i2cStop();
        return false;
    }
    i2cSendByte(dev);   
    i2cWaitAck();
    for (i = 0; i < len; i++) {
        i2cSendByte(data[i]);
        if (!i2cWaitAck()) {
            i2cStop();
            return false;
        }
    }
    i2cStop();
    return true;
}
 
 
/**
 * ��I2C�豸�ж�ȡ����
 * @param[in] dev:�豸I2C��ַ
 * @param[in] reg:�Ĵ�����ַ
 * @param[in] len:�����ֽ���
 * @param[out] data:����������
 */
bool i2cReadBytes(uint8_t dev, uint8_t reg, uint8_t len, uint8_t *data)
{
    if (!i2cStart())        
        return false;
    i2cSendByte(dev << 1);      
    if (!i2cWaitAck()) {     
        i2cStop();
        return false;
    }
    i2cSendByte(reg);     
    i2cWaitAck();
    i2cStart();           
    i2cSendByte((dev << 1) | 0x01);  // ������ַ+������    
    i2cWaitAck();
    while (len) {
        *data = i2cReceiveByte();
        if (len == 1)
            i2cNoAck();  // ���һ���ֽڲ�Ӧ��
        else
            i2cAck();
        data++;
        len--;
    }
    i2cStop();
    return true;
}
#endif


void Single_WriteI2C0(unsigned char SlaveAddress,unsigned char REG_Address,unsigned char REG_data)
{
#if defined(USE_SOFT_I2C)
  i2cWriteOneByte(SlaveAddress,REG_Address,REG_data);
#else
  i2c0Write(SlaveAddress,REG_Address,REG_data);
#endif
}

//**************************************
unsigned char Single_ReadI2C0(unsigned char SlaveAddress,unsigned char REG_Address)
{
#if defined(USE_SOFT_I2C)
  return i2cReadOneBytes(SlaveAddress,REG_Address);
#else
  return i2c0Read(SlaveAddress,REG_Address);
#endif
}


void i2c0ReadNByte(uint8_t addr, uint8_t regAddr, uint8_t *data, uint8_t length)
{
#if defined(USE_SOFT_I2C)
  i2cReadBytes(addr, regAddr,length,data);
#else
  i2c0ReadData(addr,regAddr, data,length);
#endif
}

