#include "PWM_TIMERS.h"
#include "avr/io.h"

void PWM_init(void){
    DDRD |= (1<<PD5); //PWM output at OC0B pin (8)

    OCR0B = 230;

    TCCR0A |= (1<<COM0B1) | (1<<WGM01) | (1<<WGM00);//Non Inverting Fast PWM
    TCCR0B |= (1<<CS00); //64 prescaler
}

uint16_t adc_read(uint8_t adc_channel){
    ADMUX &= 0xf0; //clear any previously used channel, but keep internal reference
    ADMUX |= adc_channel; //set the desired channel
    //start a conversion
    ADCSRA |= (1<<ADSC);

    //now wait for the conversion to complete
    while( (ADCSRA & (1<<ADSC)));

    //now we have the result, so we return it to the calling function as a 16 bit unsigned int
    return ADC*0.25;
}

void timer_1_innit(){
    TCCR1A|=(1<<WGM01); //Setup for timers, 90% of use you wanna set it like this
	OCR1A = 0xF9; //Just whatever you want the timer to count to, I set it like this since I stole it, this counts to 1ms with my prescaler. See the lecture for the formula.
	TIMSK1|=(1<<OCIE0A); //Setup for timer interrupt, this is how you wanna set it up 99% of always.
	
    sei(); //Start enable interrupt, this is used last in the interrupt initialization.
}