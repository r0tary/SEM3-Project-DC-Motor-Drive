#include "PWM.h"
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
