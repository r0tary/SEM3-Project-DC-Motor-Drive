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
volatile uint16_t s = 0, ms = 0, us = 0, flag = 1, r = 0;

int main(){
	uart_init();
	io_redirect();
	DDRD = 0x00;
	DDRB = 0x00;
	TCCR1A = 0;
	TCCR1B = 0;
	PORTD|=(1<<PORTD7) | (1<<PORTD4) | (1<<PORTD6);
	TCCR1B|=(1<<WGM12); //Setup for timers, 90% of use you wanna set it like this
	OCR1A = 1599; //Just whatever you want the timer to count to, I set it like this since I stole it, this counts to 1ms with my prescaler. See the lecture for the formula.
	TIMSK1|=(1<<OCIE1A); //Setup for timer interrupt, this is how you wanna set it up 99% of always.
	PCICR |= (1 << PCIE2);    // set PCIE2 to enable the group for PCINT17..PCINT23
	PCMSK2 |= (1 << PCINT20) | (1<<PCINT22) | (1<<PCINT23);  // Enable only PCINT20,22,23 interrupt from the group
	sei(); //Start enable interrupt, this is used last in the interrupt initialization.
	uint16_t  RPM = 0;
	while (1)
	{
		
		if (flag==0) 
		{
			TCCR1B|= (0 << CS12) | (0 << CS11) | (0 << CS10); //Stop timer
			
			RPM = (1/(0.1*us+ms+s*1000))*60000; //Get the RPM
			
			us = 0;
			ms = 0;
			s = 0;
			
			TCCR1B|= (0 << CS12) | (0 << CS11) | (1 << CS10); //Start timer with prescaler 1
			
			flag = 1;
		}
		
		printf("State1: %d \n",RPM);
	}
	}

ISR (TIMER1_COMPA_vect){
	us++;
	
	if (us>=10)
	{
		us=0;
		ms++;
	}
	
	if (ms>=1000)
	{
		ms=0;
		s++;
	}
}

ISR (PCINT2_vect){
	r++;
	
	if (r==12)
	{
	flag = 0;
	r = 0;
	}
}
