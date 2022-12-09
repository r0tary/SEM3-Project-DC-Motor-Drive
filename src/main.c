/*
 * SEM_3_Project
 * Electrical drive to control the speed of a DC motor
 * Created: 26.10.2022.
 * Author : Group 3
 */ 
#define F_CPU 16000000UL

#include <avr/io.h>
#include <util/delay.h>
#include <stdint.h>
#include <avr/interrupt.h>
#include <stdio.h>

//Included header files
#include "usart.h"
#include "PWM_TIMERS.h"

//PIN definitions
#define PoMeter 3
#define Button0 0
#define mask 1 

//function definitions
int get_DC(uint8_t);
void get_RPM(void);
uint16_t adc_read(uint8_t adc_channel);

//global variable definitions
uint8_t  flag = 0;
uint16_t RPM = 0;
volatile uint16_t r = 0, us = 0, ms = 0, s = 0, sixthRotationTime=0;
volatile char power_on=0;
int DutyCycle = 0;

int main(void){
  
  //variables
  short TOP = 255;
  int potmeter,error, previousError =0,  deltaT;
  float integral;
  float kp= 0.4, ki = 2.1;

  //Initilization
  uart_init();
  io_redirect();
  PWM_init();//initialize PWM
  OCR0B = 0;//duty cycle

  //For ADC module
  ADMUX  = (1<<REFS0);//Select Vref = AVcc
  ADCSRA = (1<<ADPS2)|(1<<ADPS1)|(1<<ADPS0)|(1<<ADEN); //Set prescaler to 128 and turn on the ADC module

  //PIN definitions
  DDRC = 0x00; //input from Potentiometer
  DDRB = 0b00100000;//led_builtin as output
  PORTB  |= (1<<PORTB0);//button pullup
  PORTD |= (1<<PORTD7) | (1<<PORTD4) | (1<<PORTD6);//hall sensor pullups

  //interrupts
  PCICR |= (1<<PCIE2)|(1<<PCIE0);//set PCIE2 to enable the group for PCINT17...PCINT23
  PCMSK2 |= (1<<PCINT20) | (1<<PCINT22) | (1<<PCINT23); //Enable only PCINT20, 22, 23 interrupt from the group.
  PCMSK0 |= (1<<PCINT0);
  sei();

  //timer1 init
  TCCR1B = (1<<CS12)|(1<<CS10);//1024 prescaler >> 15625Hz, 0.000064 sec for a tick, 64us

  //timer2 init
  TCCR2B = (1<<CS22)|(1<<CS21)|(1<<CS20);//1024 prescaler, >> 15625 Hz, 64us for a tick


  while(1){
    potmeter = adc_read(PoMeter);
    get_RPM();
    previousError = error;
    deltaT = TCNT2;
    TCNT2 = 0;
    error = potmeter - (RPM/14);
    integral += 0.000064*deltaT*(error+previousError)/2;
    DutyCycle = (int)(kp * error)+(int)ki*integral;
    if(DutyCycle<0)DutyCycle=0;
    if(DutyCycle>255)DutyCycle = 255;
    //DutyCycle = 30;
    //OCR0B = 30;
    
    if ((DutyCycle!=OCR0B)){  //update dutycycle in pwm register

      if ((DutyCycle<=TOP*0.9) && (DutyCycle >= TOP*0.1)){
        OCR0B = DutyCycle;
      }
      else if(DutyCycle > TOP*0.9){ //avoiding 100% dutycycle to let bootstrap capacitor charge
        OCR0B = TOP*0.9;
      }
      else if(DutyCycle < 0.1*TOP){//turning off the motor
        OCR0B = 0;
      }

      TCCR0A |= (1<<COM0B1) | (1<<WGM01) | (1<<WGM00);//Non Inverting Fast PWM
      TCCR0B |= (1<<CS00); //No prescaler
      
    }
    printf("%6d, %6d, %6d, %6d, %6d,%d,%f\n",DutyCycle, OCR0B, error, potmeter, RPM/14,deltaT,integral);

  }
  //return 0;
}

void get_RPM(){
  RPM = (int)(1.0/(0.000064 * sixthRotationTime * 6));
}

uint16_t adc_read(uint8_t adc_channel){
    ADMUX &= 0xf0; //clear any previously used channel, but keep internal reference
    ADMUX |= adc_channel; //set the desired channel
    //start a conversion
    ADCSRA |= (1<<ADSC);

    //now wait for the conversion to complete
    while( (ADCSRA & (1<<ADSC)));

    //now we have the result, so we return it to the calling function as a 16 bit unsigned int
    return (int)(0.25*ADC);
}

ISR (PCINT2_vect){
  sixthRotationTime = TCNT1;
  TCNT1 = 0;
}
/*
ISR (PCINT0_vect){
  //switching the last bit of power_on, if more than one second passed since the last switching. The seconds are stored from 7-1 bits, 
  //the state in the last one; also only changes it if the button is released
  if(((PINB&1)==1) && (s!=(power_on>>1))) power_on = (s<<1)| (power_on^1);

}*/