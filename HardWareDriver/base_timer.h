#ifndef BASE_TIMER_H
#define BASE_TIMER_H
#include <stdint.h>

void base_timer_initialize(void);
void base_timer_delay_ms(uint16_t value);
void base_timer_delay_us(uint16_t value);
int32_t base_timer_get_time(void);
uint32_t base_timer_get_us(void);
void base_timer_isr(void);
#endif
