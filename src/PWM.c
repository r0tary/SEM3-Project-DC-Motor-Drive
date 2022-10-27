#include "PWM.h"
#include "avr/io.h"

void PWM_init(void){
    DDRD |= (1<<PD5); //PWM output at OC0B pin (8)

    OCR0B = 230;

    TCCR0A |= (1<<COM0B1) | (1<<WGM01) | (1<<WGM00);//Non Inverting Fast PWM
    TCCR0B |= (1<<CS00); //64 prescaler
}
