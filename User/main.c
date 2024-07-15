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

#include "Headfile.h"
/***********************************************************
@函数名：main
@入口参数：无
@出口参数：无
@功能描述：主函数，对系统芯片资源、飞控外设进行初始化后，执行
while（1）里面非主要任务非主要任务是指对周期没有严格要求或者
执行时间严重耗时的子函数，例如：电压采集、按键扫描、显示屏刷 
新、地面站发送、加速度计标定、磁力计标定、遥控器行程标定、参
数保存等。
@作者：无名小哥
@日期：2024/01/20
*************************************************************/
int main(void)
{
  HardWave_Init();//芯片资源、飞控外设初始化
  while(1)//主循环：约20ms
  {	
		Get_Battery_Voltage();						//测量电池电压		
		Key_Scan(Key_Right_Release);			//按键扫描
		QuadShow(1);			  							//OLED显示，飞控两个按键同时按下，停止/恢复自动刷新
		Accel_Calibartion();              //加速度计6面校准
		Mag_Calibartion_LS(&WP_Sensor.mag_raw,Circle_Angle);//磁力计椭球校准
		RC_Calibration_Check(Receiver_PPM_Databuf);//遥控器行程校准
		Horizontal_Calibration_Check();		//机架水平校准检测
		Save_Or_Reset_PID_Parameter();		//运用地面站，修改控制参数
  }
}






/*************************************************************************************************************************************************
附送2021年电赛G题植保无人机解决方案——竞赛无人机搭积木式编程	https://www.bilibili.com/read/cv15844252?spm_id_from=333.999.0.0
电赛G题植保无人机国奖标准方案学习样例——NC360深度开源竞赛无人机开发平台	https://www.bilibili.com/video/BV11T4y1v7uW/

NC360深度开源竞赛无人机开发平台
提供2021年全国大学生电子设计竞赛G题植保无人机
基础部分+发挥部分国奖标准学习训练全套方案
视频演示：https://www.bilibili.com/video/BV11T4y1v7uW/
5_竞赛无人机搭积木式编程 ——以2021年电赛国奖标准完整复现为例学习
https://www.bilibili.com/read/cv15844252

提供2022年全国大学生电子设计竞赛B题送货无人机开源训练学习方案
https://www.bilibili.com/video/BV1PB4y1t7eM/?vd_source=fa3e626a57e95e09ecf1b8f1627e58ac
2022年7月电赛B题送货无人机加装硬件介绍
https://www.bilibili.com/video/BV1re4y1D7zJ/?vd_source=fa3e626a57e95e09ecf1b8f1627e58ac

提供2023年全国大学生电子设计竞赛G题空地协同智能消防系统开源训练学习方案
https://blog.csdn.net/u011992534/article/details/133386366
空地协同智能消防系统发挥部分实现案例——2023TI大学生电子设计竞赛G题全国一等奖标准学习训练开源方案
https://www.bilibili.com/video/BV17z4y1P7pf/
简易机械手的接线与行程设置——无人机加装夹取投放装置
https://www.bilibili.com/video/BV1dC4y1E7ML/
*************************************************************************************************************************************************/


/******************************觉得很有帮助，欢迎打赏一碗热干面（武科大二号门老汉口红油热干面4元一碗），无名小哥支付宝：1094744141@qq.com********/
/******************************由于街道市容整改，原二号门热干面摊已拆除，欢迎打赏一瓶————和其正**************************************************/
/*
																																		 _oo0oo_
																																		o8888888o
																																		88" . "88
																																		(| -_- |)
																																		0\  =  /0
																																	___/`---'\___
																																.' \\|     |// '.
																															 / \\|||  :  |||// \
																															/ _||||| -:- |||||- \
																														 |   | \\\  -  /// |   |
																														 | \_|  ''\---/''  |_/ |
																														 \  .-\__  '-'  ___/-. /
																													 ___'. .'  /--.--\  `. .'___
																												."" '<  `.___\_<|>_/___.' >' "".
																											 | | :  `- \`.;`\ _ /`;.`/ - ` : | |
																											 \  \ `_.   \_ __\ /__ _/   .-` /  /
																									 =====`-.____`.___ \_____/___.-`___.-'=====
																																		 `=---='
*/
/*
-----------------------------------------------------------------------------------------------------------------------/
*               本程序只供购买者学习使用，版权著作权属于武汉无名创新科技有限公司，无名创新团队将飞控程序源码提供给购买者，
*               购买者要为无名创新团队提供保护，未经作者许可，不得将源代码提供给他人，不得将源代码放到网上供他人随意下载， 
*               更不能以此销售牟利，如发现上述行为，无名创新团队将诉之以法律解决！！！
-----------------------------------------------------------------------------------------------------------------------*/


