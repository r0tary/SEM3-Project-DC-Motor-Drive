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

#define BAUDRATE 57600
#define BAUD_PRESCALER ((F_CPU / (BAUDRATE * 16UL))-1)

//function definitions
int get_DC(uint8_t);
void get_RPM(void);

//global variable definitions
uint8_t change_DC = 0, state_1 = 0, state_2 = 0, flag = 0;
uint16_t RPM = 0;
volatile uint16_t ms = 0, s = 0;
short DutyCycle = 0;

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
  
  DDRB = 0b0000;
  PORTB  |= (1<<PORTB0) | (1<<PORTB1);
  
  DDRD = 0x00;
  PORTD |= (1<<PORTD7) | (1<<PORTD4); 

  while(1){
    print_String("Change D_C = ",0,1);
      print_int(change_DC,14,1);
    print_String("ADC = ",0,2);
      print_int(adc_read(PoMeter),7,2);
    print_String("D_C = ",11,2);
      print_int(OCR0B,17,2);
    //printf("State = %d, ADC = %d, D_C = %d\n",State, adc_read(PoMeter), OCR0B);

    state_1 = PIND;
    get_RPM();

    if (((PINB & (mask<<Button0)) == 0) && (change_DC == 0)){
      change_DC = 1;
      print_int(change_DC,14,1);
      _delay_ms(300);
      
      while(1){
        DutyCycle = adc_read(PoMeter);
        print_int(adc_read(PoMeter),7,2);

        if (((PINB & (mask<<Button0)) == 0) && (change_DC == 1)){
          _delay_ms(300);
          break;
        }
        SSD1306_update();
      }
      
      change_DC = 0;
      OCR0B = DutyCycle;
    }
    SSD1306_update();
  }
}

void get_RPM(){
  if (state_1 != state_2 && flag == 1)
		{
			TCCR1B|=(1<<CS11) | (1<<CS10); //Start timer with prescaler 64
			
			flag = 0;
			
		}
		if (state_1 != state_2 && flag==0) 
		{
			TCCR1B|=(0<<CS11) | (0<<CS10); //Stop timer
			
			RPM = (0.16667/(ms+s*1000))*60000; //Get the RPM
			
			ms = 0;
			s = 0;
			
			flag = 1;
		}
  
  print_String("RPM = ",0,3);
  print_int(RPM,6,3);
}


ISR (TIMER1_COMPA_vect){
	ms++;
	
	if (ms==1001)
	{
		ms=0;
		s++;
	}
}

