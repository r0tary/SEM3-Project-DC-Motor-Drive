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
uint8_t flag = 0, ptr = 0;
uint16_t RPM = 0, SRPM[3];
volatile uint16_t r = 0, us = 0, ms = 0, s = 0;
int DutyCycle = 0;

int main(void) {
  //variables
  short TOP = 255;
  int potmeter, error = 0, previousError = 0, deltaT;
  float integral = 0;
  float kp = 0.4, ki = 1.99;
  
  //Initilization
  uart_init();
  io_redirect();
  PWM_init(); //initialize PWM
  OCR0B = 0; //duty cycle
  timer_1_innit();

  //For ADC module
  ADMUX = (1 << REFS0); //Select Vref = AVcc
  ADCSRA = (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0) | (1 << ADEN); //Set prescaler to 128 and turn on the ADC module

  //PIN definitions
  DDRC = 0x00; //input from Potentiometer
  DDRB = 0b00100000; //led_builtin as output
  PORTB |= (1 << PORTB0); //button pullup
  
  TCCR2B = (1 << CS22) | (1 << CS21) | (1 << CS20); //1024 prescaler, >> 15625 Hz, 64us for a tick
  
  while (1) {
    potmeter = adc_read(PoMeter);//reads Potmeter reading
    
    if(potmeter>0.9*TOP) potmeter = 0.9*TOP;//Limit POT to below 90%
    
    get_RPM();//get RPM measurement 
    previousError = error;
    deltaT = TCNT2;//get time difference between two errors for integration
    TCNT2 = 0;
    error = potmeter - (RPM / 14);//calculate error

    //Calculating the integral of the error
    if (integral <= TOP){
      integral += 0.000064 * deltaT * (error + previousError) / 2;}
    else integral = TOP;
	
    //Calaculate the correct DutyCycle needed
    DutyCycle = (int)(kp * error) + (int)ki * integral;

    //Update duty cycle in register
    if ((DutyCycle != OCR0B)){
      //Limit Duty Cycle/PWM between 10% and 90% 
      if ((DutyCycle <= TOP * 0.9) && (DutyCycle >= TOP * 0.1)) {
        OCR0B = DutyCycle;
      }
      else if (DutyCycle > TOP * 0.9) {//avoiding 100% dutycycle to let bootstrap capacitor charge
        OCR0B = TOP * 0.9;
      }
      else if (DutyCycle < 0.1 * TOP) {//turning off the motor
        OCR0B = 0;
      }
      
      //Reset PWM registers after updating OCR0B
      TCCR0A |= (1 << COM0B1) | (1 << WGM01) | (1 << WGM00); //Non Inverting Fast PWM
      TCCR0B |= (1 << CS00); //No prescaler
    }
    printf("%6d, %6d, %6d, %6d, %6d, %f\n", DutyCycle, OCR0B, error, potmeter, RPM, integral);
  }
}

void get_RPM() {

  if (flag == 0) {
    RPM = (4 / (0.1 * us + ms + s * 1000)) * 60000; //Get the RPM
    SRPM[ptr] = RPM;//Add the current RPM calculation to the SRPM array for smoothing
    
    ptr++;//Keep count of where in the array we are
    if (ptr > 2) {
      ptr = 0;
    }

    RPM = 0;//Set the current RPM to 0 since it has to be replaced with the smoothed RPM
    
    for (int i = 0; i < 3; i++) { //sum up the RPM for 3 measurements
      RPM += SRPM[i];
    }
    
    RPM = RPM / 3; //Take the average time of the previous 3 measurements

    us = ms = s = 0; //Reset the timer

    TCCR1B |= (0 << CS12) | (0 << CS11) | (1 << CS10); //Start timer with prescaler 1

    flag = 1; //Set flag to 1 and wait for the next rotation
  }

  //Had a problem where the RPM did not properly reset
  if (s >= 2) {
    RPM = 0;
  }
}

ISR(TIMER1_COMPA_vect) {
  us++; //Add a tenth of a millisecond to the us register

  //Add a millisecond to ms counter if 10 us is counted
  if (us >= 10) { 
    us = 0;
    ms++;
  }

  //Add a second to the s counter if the ms counter is 1000
  if (ms >= 1000) {
    ms = 0;
    s++;
  }
}

ISR(PCINT2_vect) {
  r++; //1/12 of a rotation

  //Set the flag to 0 to start calculation in the get_RPM function every 4th rotation
  if (r >= 48) {
    flag = 0;
    r = 0;
  }
}
