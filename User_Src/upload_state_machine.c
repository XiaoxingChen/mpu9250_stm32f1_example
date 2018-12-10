#include "upload_state_machine.h"

void update_upload_state(uint8_t* p_state, const float ypr[3], int16_t* p_math_hz)
{
    #define REIMU  0x01 //上传解算的姿态数据
    #define REMOV  0x02	//上传传感器的输出
    #define REHMC  0x03	//上传磁力计的标定值

    float Altitude = 0;
    float Temperature = 0;
    float Pressure = 0;

    int16_t ax, ay, az;	
    int16_t gx, gy, gz;
    int16_t hx, hy, hz;

    switch(*p_state)
    { 
        case REIMU:
            UART1_ReportIMU((int16_t)(ypr[0]*10.0),(int16_t)(ypr[1]*10.0),
            (int16_t)(ypr[2]*10.0),Altitude/10,Temperature,Pressure/10, (*p_math_hz) * 16);
            (*p_math_hz) = 0;
            (*p_state) = REMOV; //更改状态。
            break;
        case REMOV:
            MPU6050_getlastMotion6(&ax, &ay, &az, &gx, &gy, &gz);
            UART1_ReportMotion(ax,ay,az,gx,gy,gz,hx,hy,hz);
            (*p_state) = REIMU;
            break;
        default: 
            (*p_state) = REIMU;
            break;
	}//switch((*p_state)) 			 

}
