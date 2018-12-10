#include "base_timer.h"
#include <stm32f10x.h>

static uint32_t systick_max_count = 0xFFFFFF;
static uint32_t ms_count;
static uint32_t us_count;
static uint32_t reload_value;
static volatile int32_t _base_counter;

static void base_timer_start()
{
    SysTick->CTRL |= SysTick_CTRL_ENABLE_Msk;
}

static void base_timer_do_update()
{
	++_base_counter;
	_base_counter &= 0x7FFFFFFF;
}

//static void base_timer_stop()
//{
//    SysTick->CTRL &= ~(uint16_t)SysTick_CTRL_ENABLE_Msk;
//}


void base_timer_initialize()
{
	_base_counter = 0;	
	ms_count = SystemCoreClock/8/1000;
	us_count = ms_count / 1000;
	reload_value = ms_count;
	SystemCoreClockUpdate();
	SysTick_Config(reload_value);
	SysTick_CLKSourceConfig(SysTick_CLKSource_HCLK_Div8); //21MHz. Different from F1.
	base_timer_start();
}

void base_timer_delay_ms(uint16_t value)
{
	int32_t in_time = _base_counter;
	while(_base_counter - in_time < value);
}

uint32_t base_timer_get_us(void)
{
	uint32_t in_tick = SysTick->VAL;
	uint32_t micros =  (reload_value - in_tick) / us_count;
	return micros;
}

void base_timer_delay_us(uint16_t value)
{
	uint32_t in_tick = SysTick->VAL;
	uint32_t in_base_count = _base_counter;
	uint32_t count_num = (value-1) * us_count;
	uint32_t target_tick;
	if(in_tick > count_num)
	{
		target_tick = in_tick - count_num;
	}else
	{
		target_tick = reload_value + in_tick - count_num;
		while(in_base_count  == _base_counter);
	}
	while(SysTick->VAL > target_tick);
}

int32_t base_timer_get_time()
{
    return _base_counter;
}

void base_timer_isr()
{
	base_timer_do_update();
	SCB->ICSR = SCB_ICSR_PENDSTCLR_Msk;
}
