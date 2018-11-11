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

//上传数据的状态机
#define REIMU  0x01 //上传解算的姿态数据
#define REMOV  0x02	//上传传感器的输出
#define REHMC  0x03	//上传磁力计的标定值

#define Upload_Speed  15   //数据上传速度  单位 Hz
#define upload_time (1000000/Upload_Speed)/2  //计算上传的时间。单位为us

int16_t ax, ay, az;	
int16_t gx, gy, gz;
int16_t hx, hy, hz;
int32_t Temperature = 0, Pressure = 0, Altitude = 0;
uint32_t system_micrsecond;
int16_t hmcvalue[3];
u8 state= REIMU;  //发送特定帧 的状态机
/**************************实现函数********************************************
*函数原型:		int main(void)
*功　　能:		主程序
*******************************************************************************/
int main(void)
{
	int16_t Math_hz=0;
	unsigned char PC_comm; //PC 命令关键字节	 
	float ypr[3]; // yaw pitch roll
	/* 配置系统时钟为72M 使用外部8M晶体+PLL*/      
    //SystemInit();
	delay_init(72);		//延时初始化
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);	
	GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable, ENABLE);
    Initial_LED_GPIO();	//初始化STM32-SDK板子上的LED接口
	Initial_PWMLED();
	Initial_UART1(115200L);
	Initial_UART2(115200L);
	load_config();  //从flash中读取配置信息 -->eeprom.c
	IIC_Init();	 //初始化I2C接口
	delay_ms(300);	//等待器件上电
	//UART1_Put_String("Initialize...\r\n");
	IMU_init(); //初始化IMU和传感器
	system_micrsecond=micros();
	while(1){	//主循环
		
	//delay_ms(1); //延时，不要算那么快。
	IMU_getYawPitchRoll(ypr); //姿态更新
	Math_hz++; //解算次数 ++
	BMP180_Routing(); //处理BMP018 事务 开启转换和读取结果将在这个子程序中进行 

//-------------上位机------------------------------
	//是否到了更新 上位机的时间了？
	if((micros()-system_micrsecond)>upload_time){
	switch(state){ 
	case REIMU:
	BMP180_getTemperat(&Temperature); //读取最近的温度值
	BMP180_getPress(&Pressure);	   //读取最近的气压测量值
	BMP180_getAlt(&Altitude);	   //读取相对高度
	UART1_ReportIMU((int16_t)(ypr[0]*10.0),(int16_t)(ypr[1]*10.0),
	(int16_t)(ypr[2]*10.0),Altitude/10,Temperature,Pressure/10,Math_hz*16);
	UART2_ReportIMU((int16_t)(ypr[0]*10.0),(int16_t)(ypr[1]*10.0),
	(int16_t)(ypr[2]*10.0),Altitude/10,Temperature,Pressure/10,Math_hz*Upload_Speed);
	Math_hz=0;
	state = REMOV; //更改状态。
	break;
	case REMOV:
	MPU6050_getlastMotion6(&ax, &ay, &az, &gx, &gy, &gz);
	HMC58X3_getlastValues(&hx,&hy,&hz);
	UART1_ReportMotion(ax,ay,az,gx,gy,gz,hx,hy,hz);
	UART2_ReportMotion(ax,ay,az,gx,gy,gz,hx,hy,hz);
	state = REIMU;
	if(HMC5883_calib)state = REHMC; //需要发送当前磁力计标定值
	break;
	default: 
	UART2_ReportHMC(HMC5883_maxx,HMC5883_maxy,HMC5883_maxz,
		 HMC5883_minx,HMC5883_miny,HMC5883_minz,0);//发送标定值
	state = REIMU;
	break;
	}//switch(state) 			 
	system_micrsecond=micros();	 //取系统时间 单位 us 
	LED_Change();	//LED1改变亮度
	}
//--------------------------------------------------
	//处理PC发送来的命令
	if((PC_comm=UART2_CommandRoute())!=0xff)
	{
	switch(PC_comm){ //检查命令标识
	case Gyro_init:			MPU6050_InitGyro_Offset(); break; //读取陀螺仪零偏
	case High_init:			BMP180_ResetAlt(0); 	break;		//气压高度 清零
	case HMC_calib_begin:	HMC5883L_Start_Calib();	break; //开始磁力计标定
	case HMC_calib:		HMC5883L_Save_Calib();	break;   //保存磁力计标定
	}
	}// 处理PC 发送的命令

	}//主循环 while(1) 结束

}  //main	

//------------------End of File----------------------------
