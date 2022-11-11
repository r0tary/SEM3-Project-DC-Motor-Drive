/*
Author: Jonas Andresen
Title: Exam Project
Date: 08/06/2022
*/

#define F_CPU 16000000UL
#include <stdio.h>
#include <avr/io.h>
#include <util/delay.h>
#include <stdlib.h>
#include <avr/interrupt.h> 
#include <avr/eeprom.h> 
#include "usart.h"
#define BAUDRATE 9600
#define BAUD_PRESCALER ((F_CPU / (BAUDRATE * 16UL))-1)
volatile uint16_t ms = 0, s = 0;

int main(){
	uart_init();
	io_redirect();
	DDRD = 0x00;
	DDRB = 0x00;
	PORTD|=(1<<PORTD7) | (1<<PORTD4) | (1<<PORTD6);
	TCCR1A|=(1<<WGM01); //Setup for timers, 90% of use you wanna set it like this
	OCR1A = 0xf9; //Just whatever you want the timer to count to, I set it like this since I stole it, this counts to 1ms with my prescaler. See the lecture for the formula.
	TIMSK1|=(1<<OCIE0A); //Setup for timer interrupt, this is how you wanna set it up 99% of always.
	sei(); //Start enable interrupt, this is used last in the interrupt initialization.
	uint8_t state1 = 0, state2 = 0, flag = 1;
	uint16_t  RPM = 0;
	while (1)
	{
		state1 = PIND;
		
		if (state1 != state2 && flag == 1)
		{
			TCCR1B|=(1<<CS11) | (1<<CS10); //Start timer with prescaler 64
			
			flag = 0;
			
		}
		if (state1 != state2 && flag==0) 
		{
			TCCR1B|=(0<<CS11) | (0<<CS10); //Stop timer
			
			RPM = (0.16667/(ms+s*1000))*60000; //Get the RPM
			
			ms = 0;
			s = 0;
			
			flag = 1;
		}
		
		printf("State1: %d \n",RPM);
		
		state2 = state1;
	}
	}

ISR (TIMER1_COMPA_vect){
	ms++;
	
	if (ms==1001)
	{
		ms=0;
		s++;
	}
}
