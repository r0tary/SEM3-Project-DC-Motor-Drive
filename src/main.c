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
//#include "usart.h"
#include "PWM_TIMERS.h"
#include "SSD1306_x32.h"
#include "font.h"
#include "i2cmaster.h"


//PIN definitions
#define PoMeter 3
#define Button0 0
#define mask 0b01 


//function definitions
int get_DC(uint8_t);
void get_RPM(void);

//global variable definitions
uint8_t  flag = 0;
uint16_t RPM = 0;
volatile uint16_t r = 0, us = 0, ms = 0, s = 0;
short DutyCycle = 0;

int main(void){

  //variables
  short TOP = 256;
  
  //Initilization
  i2c_init();//initialize I2C communication
  SSD1306_setup();//Setup for display
  PWM_init();//initialize PWM
    OCR0B = TOP*0.9;//90% duty cycle
  timer_1_innit();

  SSD1306_clear();//Clears display incase of left over characters
  SSD1306_update();//Pushes to the display
  grid_status(ON);//Enables character grids (size 25x4) 
  
  //For ADC module
  ADMUX  = (1<<REFS0);//Select Vref = AVcc
  ADCSRA = (1<<ADPS2)|(1<<ADPS1)|(1<<ADPS0)|(1<<ADEN); //Set prescaler to 128 and turn on the ADC module

  //PIN definitions
  DDRC = 0x00; //input from Potentiometer
  
  DDRB = 0b0000;
  PORTB  |= (1<<PORTB0);
  
  while(1){
    DutyCycle = adc_read(PoMeter);

    if (DutyCycle!=OCR0B){
      print_String("ADC: ",0,0);
      print_int(DutyCycle,5,0);

        if ((DutyCycle<=TOP*0.9) && (DutyCycle>=TOP*0.1)){
          OCR0B = adc_read(PoMeter);

          TCCR0A |= (1<<COM0B1) | (1<<WGM01) | (1<<WGM00);//Non Inverting Fast PWM
          TCCR0B |= (1<<CS00); //No prescaler
          
          print_String("D_C: ",7,0);
          print_int(OCR0B,12,0);

          print_String("RPM: ",0,1);
          SSD1306_update();
        }
    }

    get_RPM();
  }
  return 0;

}


void get_RPM(){
  if (flag == 0){
    TCCR1B |= (0 << CS12) | (0 << CS11) | (0 << CS10);//Stop timer

    RPM = (1/(0.1*us+ms+s*1000))*60000; //Get the RPM

    us = 0;
    ms = 0;
    s = 0;

    TCCR1B|= (0 << CS12) | (0 << CS11) | (1 << CS10); //Start timer with prescaler 1

    flag = 1;
  }
  print_int(RPM,5,1);
}

ISR (TIMER1_COMPA_vect){
	us++;

  if (us >= 10){
    us = 0;
    ms++;
  }

  if (ms >= 1000){
    ms = 0;
    s++;
  }
}

ISR (PCINT2_vect){
  r++;
  
  if(r == 12){
    flag = 0;
    r = 0;
  }

}