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
#include "PWM.h"
#include "SSD1306_x32.h"
#include "font.h"
#include "i2cmaster.h"


//PIN definitions
#define PoMeter 3
#define Button0 0
#define mask 0b01 

//function definitions
int get_DC(uint8_t);

//global variable definitions
uint8_t State = 0;
int DutyCycle = 0;

int main(void){
  //variables
  short TOP = 255;

  //Initilization
  i2c_init();//initialize I2C communication
  SSD1306_setup();//Setup for display
  PWM_init();//initialize PWM
    OCR0B = TOP*0.9;//90% duty cycle
      
  SSD1306_clear();//Clears display incase of left over characters
  SSD1306_update();//Pushes to the display
  grid_status(ON);//Enables character grids (size 25x4) 
  
  //For ADC module
  ADMUX  = (1<<REFS0);//Select Vref = AVcc
  ADCSRA = (1<<ADPS2)|(1<<ADPS1)|(1<<ADPS0)|(1<<ADEN); //Set prescaler to 128 and turn on the ADC module

  //PIN definitions
  DDRC = 0x00; //input from Potentiometer
  DDRB = 0b00;
  PORTB  = 0b10; 

  while(1){
    print_String("State = ",0,1);
      print_int(State,9,1);
    print_String("ADC = ",0,2);
      print_int(adc_read(PoMeter),7,2);
    print_String("D_C = ",11,2);
      print_int(OCR0B,17,2);
    //printf("State = %d, ADC = %d, D_C = %d\n",State, adc_read(PoMeter), OCR0B);

    if (((PINB & (mask<<Button0)) == 0) && (State == 0)){
      State = 1;
      print_int(State,9,1);
      _delay_ms(300);
      
      while(1){
        DutyCycle = adc_read(PoMeter);
        print_int(adc_read(PoMeter),7,2);

        if (((PINB & (mask<<Button0)) == 0) && (State == 1)){
          _delay_ms(300);
          break;
        }
        SSD1306_update();
      }
      
      State = 0;
      OCR0B = DutyCycle;
    }
    SSD1306_update();
  }
}

int get_DC(uint8_t PoPin){
  int DC;
  DC = adc_read(PoPin);

  return DC;
}
