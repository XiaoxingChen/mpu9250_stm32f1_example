#include "base_timer.h"
#include "delay.h"

void delay_init(u8 SYSCLK)
{
	base_timer_initialize();
}

void delay_ms(u16 nms)
{
	base_timer_delay_ms(nms);
}

void delay_us(u32 nus)
{
	base_timer_delay_us(nus);
}