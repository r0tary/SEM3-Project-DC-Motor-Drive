#ifndef _PWM_LIB_H_
#define _PWM_LIB_H_

#include <stdint.h>

void PWM_init(void);

uint16_t adc_read(uint8_t adc_channel);

void timer_1_innit(void);


#endif