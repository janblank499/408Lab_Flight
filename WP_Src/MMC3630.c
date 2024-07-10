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
*               �ͻ�ʹ���ĵá��Ľ������������http://www.openedv.com/forum.php?mod=viewthread&tid=234214&extra=page=1
*               �Ա����̣�https://shop348646912.taobao.com/?spm=2013.1.1000126.2.5ce78a88ht1sO2
*               ��˾����:www.nameless.tech
*               �޸�����:2022/03/01                  
*               �汾����Ӯ��PRO����CarryPilot_V4.0.3
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
#include "MMC3630.h"

int I2C_Write_Reg(unsigned char i2c_add, unsigned char reg_add, unsigned char cmd)
{
	/* i2c_add is the 7-bit i2c address of the sensor
	 * reg_add is the register address to wtite
	 * cmd is the value that need to be written to the register
	 * I2C operating successfully, return 1, otherwise return 0;
	 */
	
	/* 
	.
	. Need to be implemented by user. 
	.
	*/	 
	Single_WriteI2C0(i2c_add,reg_add,cmd);
	return 1;
}


int I2C_Read_Reg(unsigned char i2c_add, unsigned char reg_add, unsigned char *data)
{
	/* i2c_add is the 7-bit i2c address of the sensor
	 * reg_add is the register address to read
	 * data is the first address of the buffer that need to store the register value
	 * I2C operating successfully, return 1, otherwise return 0;	
	 */
	 	
	/* 
	.
	. Need to be implemented by user. 
	.
	*/	
	*data=Single_ReadI2C0(i2c_add,reg_add); 
	return 1;
}

int I2C_MultiRead_Reg(unsigned char i2c_add, unsigned char reg_add, int num, unsigned char *data)
{
	/* i2c_add is the 7-bit i2c address of the sensor
	 * reg_add is the first register address to read
	 * num is the number of the register to read	
	 * data is the first address of the buffer that need to store the register value
	 * I2C operating successfully, return 1, otherwise return 0;	
	 */
	
	/* 
	.
	. Need to be implemented by user. 
	.
	*/
	i2c0ReadNByte(i2c_add,reg_add,data,num);
	return 1;
}



/* Default sensor otp compensation matrix */
float fOtpMatrix[3] = {1.0f,1.0f,1.35f};

/* Function declaration */

/**
 * @brief OTP read done check
 */
int MMC3630KJ_Check_OTP(void);

/**
 * @brief Check Product ID
 */
int MMC3630KJ_CheckID(void);

/**
 * @brief Reset the sensor by software
 */
void MMC3630KJ_Software_Reset(void);

/**
 * @brief Get the sensitivity compensation value
 */
void MMC3630KJ_GetCompMatrix(void);

/**
 * @brief Change the SET/RESET pulse width
 */
void MMC3630KJ_SetPulseWidth(void);

/**
 * @brief Set the output resolution
 */
void MMC3630KJ_SetOutputResolution(unsigned char res);

/**
 * @brief Enable the meas_done interrupt 
 */
void MMC3630KJ_INT_Meas_Done_Enable(void);
	
/**
 * @brief Enable the MDT interrupt
 */
void MMC3630KJ_INT_MDT_Enable(void);

/**
 * @brief Clear Meas_T_Done interrupt
 */
void MMC3630KJ_INT_Meas_T_Done_Clear(void);

/**
 * @brief Clear Meas_M_Done interrupt
 */
void MMC3630KJ_INT_Meas_M_Done_Clear(void);

/**
 * @brief Clear MDT interrupt
 */
void MMC3630KJ_INT_MDT_Clear(void);

/*********************************************************************************
* decription: OTP read done check
*********************************************************************************/
int MMC3630KJ_Check_OTP(void)
{
	unsigned char reg_val = 0;
	
	Delay_Ms(5);	
	/* Read register 0x07, check OTP_Read_Done bit */
	I2C_Read_Reg(MMC3630KJ_7BITI2C_ADDRESS, MMC3630KJ_REG_STATUS, &reg_val);	
	if((reg_val&0x10) != MMC3630KJ_OTP_READ_DONE_BIT)
		return -1;
	
	return 1;
}

/*********************************************************************************
* decription: Check Product ID
*********************************************************************************/
int MMC3630KJ_CheckID(void)
{
	unsigned char pro_id = 0;
	
	/* Read register 0x2F, check product ID */
	I2C_Read_Reg(MMC3630KJ_7BITI2C_ADDRESS, MMC3630KJ_REG_PRODUCTID, &pro_id);	
	if(pro_id != MMC3630KJ_PRODUCT_ID)
		return -1;
	
	return 1;
}

/*********************************************************************************
* decription: Reset the sensor by software
*********************************************************************************/
void MMC3630KJ_Software_Reset(void)
{
	/* Write 0x80 to register 0x09, set SW_RST bit high */
	I2C_Write_Reg(MMC3630KJ_7BITI2C_ADDRESS, MMC3630KJ_REG_CTRL1, MMC3630KJ_CMD_SW_RST);
	
	/* Delay at least 5ms to finish the software reset operation */
	Delay_Ms(5);
	
	return;	
}

/*********************************************************************************
* decription: Read the sensitivity compensation registers 0x2A and 0x2B
*********************************************************************************/
void MMC3630KJ_GetCompMatrix(void)
{
	uint8_t reg_data[2] = {0}; 

	/* write 0xE1 to reg 0x0F, password */
	I2C_Write_Reg(MMC3630KJ_7BITI2C_ADDRESS, MMC3630KJ_REG_PASSWORD, MMC3630KJ_CMD_PASSWORD);
	
	/* write 0x11 to reg 0x12 */
	I2C_Write_Reg(MMC3630KJ_7BITI2C_ADDRESS, MMC3630KJ_REG_OTPMODE, MMC3630KJ_CMD_OTP_OPER);
	
	/* write 0x80 to reg 0x13 */
	I2C_Write_Reg(MMC3630KJ_7BITI2C_ADDRESS, MMC3630KJ_REG_TESTMODE, MMC3630KJ_CMD_OTP_MR);  

	/* write 0x80 to reg 0x0A, set ULP_SEL '1' */	
	I2C_Write_Reg(MMC3630KJ_7BITI2C_ADDRESS, MMC3630KJ_REG_CTRL2, MMC3630KJ_CMD_OTP_ACT); 	
	
	/* read 0x2A and 0x2B register value */
	I2C_MultiRead_Reg(MMC3630KJ_7BITI2C_ADDRESS, MMC3630KJ_REG_OTP, 2, reg_data);
	
	/* write 0x00 to reg 0x0A, set ULP_SEL '0' */
	I2C_Write_Reg(MMC3630KJ_7BITI2C_ADDRESS, MMC3630KJ_REG_CTRL2, MMC3630KJ_CMD_OTP_NACT); 
	
	/* get sensitivity compensation value */
	fOtpMatrix[0] = 1.0f;
	fOtpMatrix[1] = OTP_CONVERT(reg_data[0]&0x3f) + 1.0f;
	fOtpMatrix[2] = (OTP_CONVERT((reg_data[1]&0x0f)<<2 | (reg_data[0]&0xc0)>>6) + 1.0f) * 1.35f;

	return;
}

/*********************************************************************************
* decription: Change the SET/RESET pulse width
*********************************************************************************/
void MMC3630KJ_SetPulseWidth(void)
{
	unsigned char reg_val;
	
	/* Write 0x00 to register 0x0A, set ULP_SEL bit low */						    
	I2C_Write_Reg(MMC3630KJ_7BITI2C_ADDRESS, MMC3630KJ_REG_CTRL2, MMC3630KJ_CMD_OTP_NACT);		
	
	/* Write 0xE1 to register 0x0F, write password */
	I2C_Write_Reg(MMC3630KJ_7BITI2C_ADDRESS, MMC3630KJ_REG_PASSWORD, MMC3630KJ_CMD_PASSWORD);		
	
	/* Read and write register 0x20, set SR_PW<1:0> = 00 = 1us */
	I2C_Read_Reg(MMC3630KJ_7BITI2C_ADDRESS, MMC3630KJ_REG_SR_PWIDTH, &reg_val); 
	reg_val = reg_val & 0xE7;							    
	I2C_Write_Reg(MMC3630KJ_7BITI2C_ADDRESS, MMC3630KJ_REG_SR_PWIDTH, reg_val); 		

	return;	
}

/*********************************************************************************
* decription: SET operation when using dual supply 
*********************************************************************************/
void MMC3630KJ_SET(void)
{	
	/* Write 0x08 to register 0x08, set SET bit high */
	I2C_Write_Reg(MMC3630KJ_7BITI2C_ADDRESS, MMC3630KJ_REG_CTRL0, MMC3630KJ_CMD_SET);
	
	/* Delay at least 1ms to finish the SET operation */
	//Delay_Ms(1);
	return;	
}

/*********************************************************************************
* decription: RESET operation when using dual supply 
*********************************************************************************/
void MMC3630KJ_RESET(void)
{	
	/* Write 0x10 to register 0x08, set RESET bit high */
	I2C_Write_Reg(MMC3630KJ_7BITI2C_ADDRESS, MMC3630KJ_REG_CTRL0, MMC3630KJ_CMD_RESET);
	
	/* Delay at least 1ms to finish the RESET operation */
	Delay_Ms(1);
	
	return;	
}

/*********************************************************************************
* decription: Set the output resolution
*********************************************************************************/
void MMC3630KJ_SetOutputResolution(unsigned char res)
{
	/* Write register 0x09, Set BW<1:0> = 0x00, 0x01, 0x02, or 0x03 */
	I2C_Write_Reg(MMC3630KJ_7BITI2C_ADDRESS, MMC3630KJ_REG_CTRL1, res);
	
	return;		
}

/*********************************************************************************
* decription: Enable the int when a mag or temp measuremet event is completed
*********************************************************************************/
void MMC3630KJ_INT_Meas_Done_Enable(void)
{
	/* Write register 0x0A, Set INT_Meas_Done_EN high */
	I2C_Write_Reg(MMC3630KJ_7BITI2C_ADDRESS, MMC3630KJ_REG_CTRL2, MMC3630KJ_CMD_INT_MD_EN);
	
	return;		
}

/*********************************************************************************
* decription: Enable the int when a motion is detected 
*********************************************************************************/
void MMC3630KJ_INT_MDT_Enable(void)
{
	/* Step size is 4mG, threshold = mdt_threshold*4mG */
	unsigned char mdt_threshold = 0x0E;
	
	/* Write register 0x0B, Set X, Y and Z threshold */
	I2C_Write_Reg(MMC3630KJ_7BITI2C_ADDRESS, MMC3630KJ_REG_X_THD, mdt_threshold);
	I2C_Write_Reg(MMC3630KJ_7BITI2C_ADDRESS, MMC3630KJ_REG_Y_THD, mdt_threshold);
	I2C_Write_Reg(MMC3630KJ_7BITI2C_ADDRESS, MMC3630KJ_REG_Z_THD, mdt_threshold);
	
	/* Write register 0x0A, Set CM_Freq [3:0], Set INT_MDT_EN high */
	I2C_Write_Reg(MMC3630KJ_7BITI2C_ADDRESS, MMC3630KJ_REG_CTRL2, MMC3630KJ_CMD_CM_14HZ | MMC3630KJ_CMD_INT_MDT_EN);
	
	/* Write register 0x08, Set Start_MDT high, Start the motion detector */
	I2C_Write_Reg(MMC3630KJ_7BITI2C_ADDRESS, MMC3630KJ_REG_CTRL0, MMC3630KJ_CMD_START_MDT);
	
	return;		
}

/*********************************************************************************
* decription: Clear Meas_T_Done interrupt
*********************************************************************************/
void MMC3630KJ_INT_Meas_T_Done_Clear(void)
{
	/* Write register 0x07, Set Meas_T_Done bit high, clear Meas_T_Done interrupt */
	I2C_Write_Reg(MMC3630KJ_7BITI2C_ADDRESS, MMC3630KJ_REG_STATUS, MMC3630KJ_MEAS_T_DONE_BIT);
	
	return;		
}

/*********************************************************************************
* decription: Clear Meas_M_Done interrupt
*********************************************************************************/
void MMC3630KJ_INT_Meas_M_Done_Clear(void)
{
	/* Write register 0x07, Set Meas_M_Done bit high, clear Meas_M_Done interrupt */
	I2C_Write_Reg(MMC3630KJ_7BITI2C_ADDRESS, MMC3630KJ_REG_STATUS, MMC3630KJ_MEAS_M_DONE_BIT);
	
	return;		
}

/*********************************************************************************
* decription: Clear MDT interrupt
*********************************************************************************/
void MMC3630KJ_INT_MDT_Clear(void)
{
	/* Write register 0x07, Set Motion Detected bit high, clear MDT interrupt */
	I2C_Write_Reg(MMC3630KJ_7BITI2C_ADDRESS, MMC3630KJ_REG_STATUS, MMC3630KJ_MDT_BIT);
	
	return;		
}

/*********************************************************************************
* decription: Initial the sensor when power on
*********************************************************************************/
int MMC3630KJ_Initialization(void)
{
	int ret = 0;
	
	/* Check OTP Read status */
	ret = MMC3630KJ_Check_OTP();
	if(ret<0)
		return ret;
	
	/* Check product ID */
	ret = MMC3630KJ_CheckID();
	if(ret<0)
		return ret;	
	
	/* Get sensitivity compensation value */
	MMC3630KJ_GetCompMatrix();
	
//	/* Change the SET/RESET pulse width to 1us */
//	MMC3630KJ_SetPulseWidth();
	
	/* SET operation */
	MMC3630KJ_SET();
	
	/* Set output resolution */
	MMC3630KJ_SetOutputResolution(MMC3630KJ_CMD_600HZ);
	
	/*ʹ�ܲ���*/
	MMC3630KJ_Enable();
	return 1;
}
/*********************************************************************************
* decription: Enable sensor when from pown down mode to normal mode
*********************************************************************************/
void MMC3630KJ_Enable(void)
{
	/* Write 0x01 to register 0x08, set TM_M bit high */
	I2C_Write_Reg(MMC3630KJ_7BITI2C_ADDRESS, MMC3630KJ_REG_CTRL0, MMC3630KJ_CMD_TM_M);
	Delay_Ms(10);
	
	return;
}
/*********************************************************************************
* decription: Disable sensor
*********************************************************************************/
void MMC3630KJ_Disable(void)
{
	return;
}

/*********************************************************************************
* decription: Read the temperature output
*********************************************************************************/
void MMC3630KJ_GetTemperature(float *t_out)
{
	uint8_t reg_status = 0;
	uint8_t reg_t = 0;
	
	/* Write 0x02 to register 0x08, set TM_T bit high */
	I2C_Write_Reg(MMC3630KJ_7BITI2C_ADDRESS, MMC3630KJ_REG_CTRL0, MMC3630KJ_CMD_TM_T);
	Delay_Ms(1);
	
	/* Read register 0x07, check Meas_T_Done bit */		
	I2C_Read_Reg(MMC3630KJ_7BITI2C_ADDRESS,MMC3630KJ_REG_STATUS,&reg_status);	
	while((reg_status&0x02) != 0x02){
		Delay_Ms(1);
		I2C_Read_Reg(MMC3630KJ_7BITI2C_ADDRESS,MMC3630KJ_REG_STATUS,&reg_status);	
	}
	
	/* Read register 0x06 */	
	I2C_Read_Reg(MMC3630KJ_7BITI2C_ADDRESS, MMC3630KJ_REG_TEMP, &reg_t);
		
	/* The temperature output has not been calibrated, can not present the ambient temperature*/
	t_out[0] = (float)reg_t*MMC3630KJ_T_SENSITIVITY + MMC3630KJ_T_ZERO;//unit is degree Celsius
	
	return;
}

/*********************************************************************************
* decription: Read the data register and convert to magnetic field vector
*********************************************************************************/
void MMC3630KJ_GetData(float *mag_out)
{
	uint8_t reg_status = 0;
	
	uint8_t data_reg[6] ={0};
	uint16_t data_temp[3] = {0};
	uint16_t mag_rawdata[3]={0};
	
	float mag_temp[3] = {0};

	/* Write 0x01 to register 0x08, set TM_M bit high */
	I2C_Write_Reg(MMC3630KJ_7BITI2C_ADDRESS, MMC3630KJ_REG_CTRL0, MMC3630KJ_CMD_TM_M);

	/* Read register 0x07, check Meas_M_Done bit */		
	I2C_Read_Reg(MMC3630KJ_7BITI2C_ADDRESS,MMC3630KJ_REG_STATUS,&reg_status);	
	while((reg_status&0x01) != 0x01){
		Delay_Ms(1);
		I2C_Read_Reg(MMC3630KJ_7BITI2C_ADDRESS,MMC3630KJ_REG_STATUS,&reg_status);	
	}
	
	/* Read register 0x00-0x05 */	
	I2C_MultiRead_Reg(MMC3630KJ_7BITI2C_ADDRESS, MMC3630KJ_REG_DATA, 6, data_reg);
		
	/* The output raw data unit is "count or LSB" */ 
	data_temp[0]=(uint16_t)(data_reg[1]<< 8 | data_reg[0]);
	data_temp[1]=(uint16_t)(data_reg[3]<< 8 | data_reg[2]);
	data_temp[2]=(uint16_t)(data_reg[5]<< 8 | data_reg[4]);
	
	/* Transform register data, get x, y, z output */
	mag_rawdata[0] = data_temp[0];
	mag_rawdata[1] = data_temp[1] - data_temp[2] + 32768;
	mag_rawdata[2] = data_temp[1] + data_temp[2] - 32768;

	/* Transform to unit Gauss */
	mag_temp[0] = ((float)mag_rawdata[0] - MMC3630KJ_OFFSET)/MMC3630KJ_SENSITIVITY; //unit Gauss
	mag_temp[1] = ((float)mag_rawdata[1] - MMC3630KJ_OFFSET)/MMC3630KJ_SENSITIVITY;
	mag_temp[2] = ((float)mag_rawdata[2] - MMC3630KJ_OFFSET)/MMC3630KJ_SENSITIVITY;

	/* Sensitivity compensation */
	mag_out[0] = mag_temp[0]*fOtpMatrix[0]; //unit Gauss	
	mag_out[1] = mag_temp[1]*fOtpMatrix[1];
	mag_out[2] = mag_temp[2]*fOtpMatrix[2];
	
	return;
}
