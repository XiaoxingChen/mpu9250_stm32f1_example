/* main.c file
编写者：lisn3188
网址：www.chiplab7.com
作者E-mail：lisn3188@163.com
编译环境：MDK-Lite  Version: 4.23
初版时间: 2012-05-05
测试： 本程序已在第七实验室的mini IMU上完成测试
Mini IMU AHRS 模块官方销售地址：Http://chiplab7.taobao.com
功能：
1.初始化各个传感器，
2.运行姿态解算和高度测量
3.将解算的姿态和各个传感器的输出上传到 MiniIMU AHRS 测试软件
4.响应 PC发送的命令
------------------------------------
*/

#include "common.h"  //包含所有的驱动 头文件
#include "upload_state_machine.h"

#define Upload_Speed  15   //数据上传速度  单位 Hz
#define upload_time (1000000/Upload_Speed)/2  //计算上传的时间。单位为us

/**************************实现函数********************************************
*函数原型:		int main(void)
*功　　能:		主程序
*******************************************************************************/
int main(void)
{
	int16_t Math_hz=0;
	unsigned char PC_comm; //PC 命令关键字节	 
	float ypr[3]; // yaw pitch roll
	u8 state = 1;
	uint32_t system_micrsecond;
	OrientationEstimator estimator;
	/* 配置系统时钟为72M 使用外部8M晶体+PLL*/      

	delay_init(72);		//延时初始化
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);	
	GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable, ENABLE);
	Initial_UART1(115200L);
	load_config();  //从flash中读取配置信息 -->eeprom.c
	IIC_Init();	 //初始化I2C接口
	delay_ms(300);	//等待器件上电
	IMU_init(&estimator); //初始化IMU和传感器
	system_micrsecond=micros();
	while(1){	//主循环
		
	IMU_getYawPitchRoll(&estimator, ypr); //姿态更新
	Math_hz++; //解算次数 ++

//-------------上位机------------------------------
	//是否到了更新 上位机的时间了？
	if((micros()-system_micrsecond)>upload_time){
		update_upload_state(&state, ypr, &Math_hz);
		system_micrsecond = micros();	 //取系统时间 单位 us 
	}
//--------------------------------------------------
	//处理PC发送来的命令
	if((PC_comm=UART2_CommandRoute())!=0xff)
	{
		switch(PC_comm){ //检查命令标识
		case Gyro_init:			MPU6050_InitGyro_Offset(); break; //读取陀螺仪零偏
	}
	}// 处理PC 发送的命令

	}//主循环 while(1) 结束

}  //main	

//------------------End of File----------------------------
