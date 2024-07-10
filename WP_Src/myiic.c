/* Copyright (c)  2019-2030 Wuhan Nameless Innovation Technology Co.,Ltd. All rights reserved.*/
/*----------------------------------------------------------------------------------------------------------------------/
																									开源并不等于免费
																									开源并不等于免费
																									开源并不等于免费
																									重要的事情说三遍
								先驱者的历史已经证明，在当前国内略浮躁+躺平+内卷的大环境下，对于毫无收益的开源项目，单靠坊间飞控爱好者、
								个人情怀式、自发地主动输出去参与开源项目的方式行不通，好的开源项目需要请专职人员做好售后技术服务、配套
								手册和视频教程要覆盖新手入门到进阶阶段，使用过程中对用户反馈问题和需求进行统计、在实践中完成对产品的一
								次次完善与迭代升级。
-----------------------------------------------------------------------------------------------------------------------
*                                                 为什么选择无名创新？
*                                         感动人心价格厚道，最靠谱的开源飞控；
*                                         国内业界良心之作，最精致的售后服务；
*                                         追求极致用户体验，高效进阶学习之路；
*                                         萌新不再孤单求索，合理把握开源尺度；
*                                         响应国家扶贫号召，促进教育体制公平；
*                                         新时代奋斗最出彩，建人类命运共同体。 
-----------------------------------------------------------------------------------------------------------------------
*               生命不息、奋斗不止；前人栽树，后人乘凉！！！
*               开源不易，且学且珍惜，祝早日逆袭、进阶成功！！！
*               学习优秀者，简历可推荐到DJI、ZEROTECH、XAG、AEE、GDU、AUTEL、EWATT、HIGH GREAT等公司就业
*               求职简历请发送：15671678205@163.com，需备注求职意向单位、岗位、待遇等
*               无名创新开源飞控QQ群：2号群465082224、1号群540707961
*               CSDN博客：http://blog.csdn.net/u011992534
*               B站教学视频：https://space.bilibili.com/67803559/#/video				优酷ID：NamelessCotrun无名小哥
*               无名创新国内首款TI开源飞控设计初衷、知乎专栏:https://zhuanlan.zhihu.com/p/54471146
*               TI教育无人机品质供应商，开源-教学-培养-竞赛,盘古 TI MCU系统板 NController多功能控制器https://item.taobao.com/item.htm?spm=a21n57.1.0.0.7200523c4JP61D&id=697442280363&ns=1&abbucket=19#detail 
*               淘宝店铺：https://shop348646912.taobao.com/?spm=2013.1.1000126.2.5ce78a88ht1sO2
*               公司官网:www.nameless.tech
*               修改日期:2024/01/20                  
*               版本：躺赢者PRO_V3——CarryPilot_V6.0.5
*               版权所有，盗版必究。
*               Copyright(C) 2019-2030 武汉无名创新科技有限公司 
*               All rights reserved
-----------------------------------------------------------------------------------------------------------------------
*               重要提示：
*               正常淘宝咸鱼转手的飞控、赠送朋友、传给学弟的都可以进售后群学习交流，
*               不得在网上销售无名创新资料，公司开放代码有软件著作权保护版权，他人不得将
*               资料代码传网上供他人下载，不得以谋利为目去销售资料代码，发现有此类操作者，
*               公司会提前告知，请1天内及时处理，否则你的侵权违规行为会被贴出在抖音、
*               今日头条、百家号、公司官网、微信公众平台、技术博客、知乎等平台予以公示曝光
*               此种侵权所为会成为个人终身污点，影响升学、找工作、社会声誉、很快就很在无人机界出名，后果很严重。
*               因此行为给公司造成重大损失者，会以法律途径解决，感谢您的合作，谢谢！！！
----------------------------------------------------------------------------------------------------------------------*/
/* Copyright (C) 2015 Kristian Sloth Lauszus. All rights reserved.

This software may be distributed and modified under the terms of the GNU
General Public License version 2 (GPL2) as published by the Free Software
Foundation and appearing in the file GPL2.TXT included in the packaging of
this file. Please note that GPL2 Section 2[b] requires that all works based
on this software must also be made publicly available under the terms of
the GPL2 ("Copyleft").

Contact information
-------------------

Kristian Sloth Lauszus
Web      :  http://www.lauszus.com
e-mail   :  lauszus@gmail.com
*/

#include <stdint.h>
#include <stdbool.h>

#include "I2C.h"

#include "hw_memmap.h"
#include "gpio.h"
#include "pin_map.h"
#include "myiic.h"
#include "sysctl.h"
/***********************************************************
@函数名：Init_I2C
@入口参数：无
@出口参数：无
功能描述：TM4C硬件I2C初始化
@作者：无名小哥
@日期：2024/01/20
*************************************************************/
void Init_I2C1(uint32_t clk) {
  SysCtlPeripheralEnable(SYSCTL_PERIPH_I2C1); // Enable I2C1 peripheral
  SysCtlDelay(2); // Insert a few cycles after enabling the peripheral to allow the clock to be fully activated
  SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA); // Enable GPIOA peripheral
  SysCtlDelay(2); // Insert a few cycles after enabling the peripheral to allow the clock to be fully activated
  
  // Use alternate function
  GPIOPinConfigure(GPIO_PA6_I2C1SCL);
  GPIOPinConfigure(GPIO_PA7_I2C1SDA);
  
  GPIOPinTypeI2CSCL(GPIO_PORTA_BASE, GPIO_PIN_6); // Use pin with I2C SCL peripheral
  GPIOPinTypeI2C(GPIO_PORTA_BASE, GPIO_PIN_7); // Use pin with I2C peripheral
	
  I2CMasterInitExpClk(I2C1_BASE, clk,true); // Enable and set frequency to 400 kHz  100
  //I2CMasterInitExpClk(I2C1_BASE, SysCtlClockGet(),true);          
  SysCtlDelay(2); // Insert a few cycles after enabling the I2C to allow the clock to be fully activated
}

/***********************************************************
@函数名：i2cWriteData
@入口参数：uint8_t addr, uint8_t regAddr, uint8_t *data, uint8_t length
@出口参数：无
功能描述：I2C写数据
@作者：无名小哥
@日期：2024/01/20
*************************************************************/
void i2c1WriteData(uint8_t addr, uint8_t regAddr, uint8_t *data, uint8_t length) {
  I2CMasterSlaveAddrSet(I2C1_BASE, addr, false); // Set to write mode
  
  I2CMasterDataPut(I2C1_BASE, regAddr); // Place address into data register
  I2CMasterControl(I2C1_BASE, I2C_MASTER_CMD_BURST_SEND_START); // Send start condition
  while (I2CMasterBusy(I2C1_BASE)); // Wait until transfer is done
  
  for (uint8_t i = 0; i < length - 1; i++) {
    I2CMasterDataPut(I2C1_BASE, data[i]); // Place data into data register
    I2CMasterControl(I2C1_BASE, I2C_MASTER_CMD_BURST_SEND_CONT); // Send continues condition
    while (I2CMasterBusy(I2C1_BASE)); // Wait until transfer is done
  }
  
  I2CMasterDataPut(I2C1_BASE, data[length - 1]); // Place data into data register
  I2CMasterControl(I2C1_BASE, I2C_MASTER_CMD_BURST_SEND_FINISH); // Send finish condition
  while (I2CMasterBusy(I2C1_BASE)); // Wait until transfer is done
}

/***********************************************************
@函数名：i2cRead
@入口参数：uint8_t addr, uint8_t regAddr
@出口参数：无
功能描述：I2C读数据
@作者：无名小哥
@日期：2024/01/20
*************************************************************/
uint8_t i2c1Read(uint8_t addr, uint8_t regAddr) {
  I2CMasterSlaveAddrSet(I2C1_BASE, addr, false); // Set to write mode
  
  I2CMasterDataPut(I2C1_BASE, regAddr); // Place address into data register
  I2CMasterControl(I2C1_BASE, I2C_MASTER_CMD_SINGLE_SEND); // Send data
  while (I2CMasterBusy(I2C1_BASE)); // Wait until transfer is done
  
  I2CMasterSlaveAddrSet(I2C1_BASE, addr, true); // Set to read mode
  
  I2CMasterControl(I2C1_BASE, I2C_MASTER_CMD_SINGLE_RECEIVE); // Tell master to read data
  while (I2CMasterBusy(I2C1_BASE)); // Wait until transfer is done
  return I2CMasterDataGet(I2C1_BASE); // Read data
}

/***********************************************************
@函数名：i2cWrite
@入口参数：uint8_t addr, uint8_t regAddr, uint8_t data
@出口参数：无
功能描述：I2C写数据
@作者：无名小哥
@日期：2024/01/20
*************************************************************/
void i2c1Write(uint8_t addr, uint8_t regAddr, uint8_t data) {
  i2c1WriteData(addr, regAddr, &data, 1);
}


/***********************************************************
@函数名：i2cReadData
@入口参数：uint8_t addr, uint8_t regAddr, uint8_t *data,
uint8_t length
@出口参数：无
功能描述：I2C读数据
@作者：无名小哥
@日期：2024/01/20
*************************************************************/
void i2c1ReadData(uint8_t addr, uint8_t regAddr, uint8_t *data, uint8_t length) {
  I2CMasterSlaveAddrSet(I2C1_BASE, addr, false); // Set to write mode
  I2CMasterDataPut(I2C1_BASE, regAddr); // Place address into data register
  I2CMasterControl(I2C1_BASE, I2C_MASTER_CMD_SINGLE_SEND); // Send data
  while (I2CMasterBusy(I2C1_BASE)); // Wait until transfer is done 
  I2CMasterSlaveAddrSet(I2C1_BASE, addr, true); // Set to read mode
  I2CMasterControl(I2C1_BASE, I2C_MASTER_CMD_BURST_RECEIVE_START); // Send start condition
  while (I2CMasterBusy(I2C1_BASE)); // Wait until transfer is done
  data[0] = I2CMasterDataGet(I2C1_BASE); // Place data into data register 
  for (uint8_t i = 1; i < length - 1; i++) 
	{
    I2CMasterControl(I2C1_BASE, I2C_MASTER_CMD_BURST_RECEIVE_CONT); // Send continues condition
    while (I2CMasterBusy(I2C1_BASE)); // Wait until transfer is done
    data[i] = I2CMasterDataGet(I2C1_BASE); // Place data into data register
  }
  I2CMasterControl(I2C1_BASE, I2C_MASTER_CMD_BURST_RECEIVE_FINISH); // Send finish condition
  while (I2CMasterBusy(I2C1_BASE)); // Wait until transfer is done
  data[length - 1] = I2CMasterDataGet(I2C1_BASE); // Place data into data register
}


void Single_WriteI2C(unsigned char SlaveAddress,unsigned char REG_Address,unsigned char REG_data)
{
  i2c1Write(SlaveAddress,REG_Address,REG_data);
}	

unsigned char Single_ReadI2C(unsigned char SlaveAddress,unsigned char REG_Address)
{
  return i2c1Read(SlaveAddress,REG_Address);
}

short int Double_ReadI2C(unsigned char SlaveAddress,unsigned char REG_Address)
{
  unsigned char msb , lsb ;
  msb = i2c1Read(SlaveAddress,REG_Address);
  lsb = i2c1Read(SlaveAddress,REG_Address+1);
  return ( ((short int)msb) << 8 | lsb) ;
}






#define TRY_OUT_MAX  500//1000
uint32_t system_clk_frq=0;
////////////////////////
/***********************************************************
@函数名：Init_I2C2
@入口参数：无
@出口参数：无
功能描述：TM4C硬件I2C初始化
@作者：无名小哥
@日期：2024/01/20
*************************************************************/
void Init_I2C0(void) {
  SysCtlPeripheralEnable(SYSCTL_PERIPH_I2C0); // Enable I2C0 peripheral
	SysCtlPeripheralReset(SYSCTL_PERIPH_I2C0);
  SysCtlDelay(2); // Insert a few cycles after enabling the peripheral to allow the clock to be fully activated
  SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB); // Enable GPIOB peripheral
  SysCtlDelay(2); // Insert a few cycles after enabling the peripheral to allow the clock to be fully activated
  
  // Use alternate function
  GPIOPinConfigure(GPIO_PB2_I2C0SCL);
  GPIOPinConfigure(GPIO_PB3_I2C0SDA);
  
  GPIOPinTypeI2CSCL(GPIO_PORTB_BASE, GPIO_PIN_2); // Use pin with I2C SCL peripheral
  GPIOPinTypeI2C(GPIO_PORTB_BASE, GPIO_PIN_3); // Use pin with I2C peripheral
  
	system_clk_frq=SysCtlClockGet();
  I2CMasterInitExpClk(I2C0_BASE, SysCtlClockGet(),true); // Enable and set frequency to 100 kHz
//	I2CMasterInitExpClk(I2C1_BASE, 400*100000,true); // Enable and set frequency to 400 kHz  100
	SysCtlDelay(2); // Insert a few cycles after enabling the I2C to allow the clock to be fully activated	
}

/***********************************************************
@函数名：i2cWriteData
@入口参数：uint8_t addr, uint8_t regAddr, uint8_t *data, uint8_t length
@出口参数：无
功能描述：I2C写数据
@作者：无名小哥
@日期：2024/01/20
*************************************************************/

void i2c0WriteData(uint8_t addr, uint8_t regAddr, uint8_t *data, uint8_t length) 
{
	int16_t i2c0_try_cnt=0;
  I2CMasterSlaveAddrSet(I2C0_BASE, addr, false); // Set to write mode  
  I2CMasterDataPut(I2C0_BASE, regAddr); // Place address into data register
  I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_BURST_SEND_START); // Send start condition 
	
	i2c0_try_cnt=TRY_OUT_MAX;
	while(I2CMasterBusy(I2C0_BASE)&&i2c0_try_cnt--) ; // Wait until transfer is done
  
	for (uint8_t i = 0; i < length - 1; i++) 
	{
    I2CMasterDataPut(I2C0_BASE, data[i]); // Place data into data register
    I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_BURST_SEND_CONT); // Send continues condition
    i2c0_try_cnt=TRY_OUT_MAX;
		while(I2CMasterBusy(I2C0_BASE)&&i2c0_try_cnt--){;} // Wait until transfer is done
  }
  
  I2CMasterDataPut(I2C0_BASE, data[length - 1]); // Place data into data register
  I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_BURST_SEND_FINISH); // Send finish condition
	
	i2c0_try_cnt=TRY_OUT_MAX;
  while(I2CMasterBusy(I2C0_BASE)&&i2c0_try_cnt--){;}// Wait until transfer is done
}

/***********************************************************
@函数名：i2cRead
@入口参数：uint8_t addr, uint8_t regAddr
@出口参数：无
功能描述：I2C读数据
@作者：无名小哥
@日期：2024/01/20
*************************************************************/
uint8_t i2c0Read(uint8_t addr, uint8_t regAddr) {
  int16_t i2c0_try_cnt=0;
	I2CMasterSlaveAddrSet(I2C0_BASE, addr, false); // Set to write mode
  
  I2CMasterDataPut(I2C0_BASE, regAddr); // Place address into data register
  I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_SINGLE_SEND); // Send data
	
	i2c0_try_cnt=TRY_OUT_MAX;
  while (I2CMasterBusy(I2C0_BASE)&&i2c0_try_cnt--){;} // Wait until transfer is done
  
  I2CMasterSlaveAddrSet(I2C0_BASE, addr, true); // Set to read mode
  I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_SINGLE_RECEIVE); // Tell master to read data
  
	i2c0_try_cnt=TRY_OUT_MAX;
	while(I2CMasterBusy(I2C0_BASE)&&i2c0_try_cnt--){;}  // Wait until transfer is done
	
  return I2CMasterDataGet(I2C0_BASE); // Read data
}

/***********************************************************
@函数名：i2cWrite
@入口参数：uint8_t addr, uint8_t regAddr, uint8_t data
@出口参数：无
功能描述：I2C写数据
@作者：无名小哥
@日期：2024/01/20
*************************************************************/
void i2c0Write(uint8_t addr, uint8_t regAddr, uint8_t data) {
  i2c0WriteData(addr, regAddr, &data, 1);
}


/***********************************************************
@函数名：i2cReadData
@入口参数：uint8_t addr, uint8_t regAddr, uint8_t *data,
uint8_t length
@出口参数：无
功能描述：I2C读数据
@作者：无名小哥
@日期：2024/01/20
*************************************************************/
void i2c0ReadData(uint8_t addr, uint8_t regAddr, uint8_t *data, uint8_t length) {
	int16_t i2c0_try_cnt=0;
  I2CMasterSlaveAddrSet(I2C0_BASE, addr, false); // Set to write mode
  I2CMasterDataPut(I2C0_BASE, regAddr); // Place address into data register
  I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_SINGLE_SEND); // Send data
	
	i2c0_try_cnt=TRY_OUT_MAX;
  while (I2CMasterBusy(I2C0_BASE)&&i2c0_try_cnt--){;} // Wait until transfer is done 
	
	
  I2CMasterSlaveAddrSet(I2C0_BASE, addr, true); // Set to read mode
  I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_BURST_RECEIVE_START); // Send start condition
	
	i2c0_try_cnt=TRY_OUT_MAX;
  while (I2CMasterBusy(I2C0_BASE)&&i2c0_try_cnt--){;} // Wait until transfer is done
	
	
  data[0] = I2CMasterDataGet(I2C0_BASE); // Place data into data register 
  for (uint8_t i = 1; i < length - 1; i++) 
	{
    I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_BURST_RECEIVE_CONT); // Send continues condition
		
		i2c0_try_cnt=TRY_OUT_MAX;
    while (I2CMasterBusy(I2C0_BASE)&&i2c0_try_cnt--){;} // Wait until transfer is done
		
    data[i] = I2CMasterDataGet(I2C0_BASE); // Place data into data register
  }
  I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_BURST_RECEIVE_FINISH); // Send finish condition
	
	i2c0_try_cnt=TRY_OUT_MAX;
  while (I2CMasterBusy(I2C0_BASE)&&i2c0_try_cnt--){;} // Wait until transfer is done

  data[length - 1] = I2CMasterDataGet(I2C0_BASE); // Place data into data register
}
