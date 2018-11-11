/* eeprom.c file
编写者：lisn3188
网址：www.chiplab7.net
作者E-mail：lisn3188@163.com
编译环境：MDK-Lite  Version: 4.23
初版时间: 2012-05-05
测试： 本程序已在第七实验室的mini IMU上完成测试
功能：
将Flash用作EEPROM 用于保存偏置和标定数据
------------------------------------
 */			  

#include "eeprom.h"

struct data_map Config;	//配置信息

void load_config(void){
	int16_t i;
	int16_t *ptr = &Config.is_good;
	int16_t *temp_addr = (int16_t *)PAGE_Config;
	FLASH_Unlock();
	for(i=0 ; i< sizeof(Config)/2;i++){
		*ptr = *temp_addr;
		temp_addr++;
		ptr++;
	}
	FLASH_Lock();
	if(Config.is_good != (int16_t)0xA55A){ //数据无效 ，此时需要装载默认值。
		Config.is_good = 0xA55A;
		Config.dGx_offset = 0;
		Config.dGy_offset = 0;
		Config.dGz_offset = 0;
	
		Config.dMx_offset = 0;
		Config.dMy_offset = 0;
		Config.dMz_offset = 0;
	
		Config.dMx_scale =1.0f;
		Config.dMy_scale =1.0f;
		Config.dMz_scale =1.0f;
	
		Write_config();	 //将默认值写入flash
	}
}

//将当前配置写入flash
void Write_config(void){
	int16_t i;
	int16_t *ptr = &Config.is_good;
	uint32_t ptemp_addr = PAGE_Config;
	FLASH_Unlock();
 	FLASH_ErasePage(PAGE_Config); //擦 页
	for(i=0;i<sizeof(Config)/2;i++){
	 	FLASH_ProgramHalfWord(ptemp_addr,ptr[i]);
	 	ptemp_addr+=2;
	}
	FLASH_Lock();
}

//------------------End of File----------------------------
