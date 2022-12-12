#include <avr/interrupt.h>

#include "PWM_TIMERS.h"
#include "avr/io.h"


void PWM_init(void){
    DDRD |= (1<<PD5); //PWM output at OC0B pin (8)

    //OCR0B = adc_read(3);

    TCCR0A |= (1<<COM0B1) | (1<<WGM01) | (1<<WGM00);//Non Inverting Fast PWM
    TCCR0B |= (1<<CS00); //No prescaler
}
/*
uint16_t adc_read(uint8_t adc_channel){
    ADMUX &= 0xf0; //clear any previously used channel, but keep internal reference
    ADMUX |= adc_channel; //set the desired channel
    //start a conversion
    ADCSRA |= (1<<ADSC);

    //now wait for the conversion to complete
    while( (ADCSRA & (1<<ADSC)));

    //now we have the result, so we return it to the calling function as a 16 bit unsigned int
    return (int)(0.25*ADC);
}*/

void timer_1_innit(){
    TCCR1A = 0;
    TCCR1B = 0;

    PORTD |= (1<<PORTD7) | (1<<PORTD4) | (1<<PORTD6);
    TCCR1B |= (1<<WGM12);//clear timer on compare match mode
    OCR1A = 1599;
    TIMSK1 |= (1<<OCIE1A);

    PCICR |= (1<<PCIE2);//set PCIE2 to enable the group for PCINT17...PCINT23
    PCMSK2 |= (1<<PCINT20) | (1<<PCINT22) | (1<<PCINT23); //Enable only PCINT20, 22, 23 interrupt from the group.
    
    sei();
}
