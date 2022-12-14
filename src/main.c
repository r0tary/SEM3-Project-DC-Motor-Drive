/*
 * SEM_3_Project
 * Electrical drive to control the speed of a DC motor
 * Created: 26.10.2022.
 * Author : Group 3
 */
//bahaaaaa
#define F_CPU 16000000UL

#include <avr/io.h>

#include <util/delay.h>

#include <stdint.h>

#include <avr/interrupt.h>

#include <stdio.h>

//Included header files
#include "usart.h"

#include "PWM_TIMERS.h"
 //#include "pid.h"

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
volatile char power_on = 0;
int DutyCycle = 0;

int main(void) {

  //variables
  short TOP = 255;
  int potmeter, error = 0, previousError = 0, deltaT;
  float integral = 0;
  float kp = 1, ki = 2.9;
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
  //timer2 init
  TCCR2B = (1 << CS22) | (1 << CS21) | (1 << CS20); //1024 prescaler, >> 15625 Hz, 64us for a tick

  while (1) {
    potmeter = adc_read(PoMeter);
    if(potmeter>0.9*TOP) potmeter = 0.9*TOP;
    get_RPM();
    previousError = error;
    deltaT = TCNT2;
    TCNT2 = 0;
    //if(potmeter > 0.9*TOP)potmeter = 0.9*TOP;
    error = potmeter - (RPM / 14);

    if (integral <= TOP){
        integral += 0.000064 * deltaT * (error + previousError) / 2;}
    else integral = TOP;
    
    DutyCycle = (int)(kp * error) + (int) ki * integral;
    
    if (DutyCycle < 0) DutyCycle = 0;
    if (DutyCycle > 255) DutyCycle = 255;
    //DutyCycle = 30;p
    //OCR0B = 30;

    if ((DutyCycle != OCR0B)) { //update dutycycle in pwm register

      if ((DutyCycle <= TOP * 0.9) && (DutyCycle >= TOP * 0.1)) {
        OCR0B = DutyCycle;
      }
      else if (DutyCycle > TOP * 0.9) { //avoiding 100% dutycycle to let bootstrap capacitor charge
        OCR0B = TOP * 0.9;
      }
      else if (DutyCycle < 0.1 * TOP) { //turning off the motor
        OCR0B = 0;
      }

      TCCR0A |= (1 << COM0B1) | (1 << WGM01) | (1 << WGM00); //Non Inverting Fast PWM
      TCCR0B |= (1 << CS00); //No prescaler
    }
    printf("%6d, %6d, %6d, %6d, %6d, %f, %d\n", DutyCycle, OCR0B, error, potmeter, RPM, integral, s);

  }
  //return 0;
}

void get_RPM() {

  if (flag == 0) {
    //TCCR1B |= (0 << CS12) | (0 << CS11) | (0 << CS10);//Stop timer

    RPM = (4 / (0.1 * us + ms + s * 1000)) * 60000; //Get the RPM
    SRPM[ptr] = RPM;
    ptr++;
    if (ptr > 2) {
      ptr = 0;
    }
    RPM = 0;
    for (int i = 0; i < 3; i++) { //sum up the time for one rotation
      RPM += SRPM[i];
    }
    RPM = RPM / 3;
    us = 0;
    ms = 0;
    s = 0;

    TCCR1B |= (0 << CS12) | (0 << CS11) | (1 << CS10); //Start timer with prescaler 1

    flag = 1;
  }

  if (s >= 2) {
    RPM = 0;
  }
  //print_String("    ",0,3);

}
uint16_t adc_read(uint8_t adc_channel) {
  ADMUX &= 0xf0; //clear any previously used channel, but keep internal reference
  ADMUX |= adc_channel; //set the desired channel
  //start a conversion
  ADCSRA |= (1 << ADSC);

  //now wait for the conversion to complete
  while ((ADCSRA & (1 << ADSC)));

  //now we have the result, so we return it to the calling function as a 16 bit unsigned int
  return (int)(0.25 * ADC);
}

ISR(TIMER1_COMPA_vect) {
  us++;

  if (us >= 10) {
    us = 0;
    ms++;
  }

  if (ms >= 1000) {
    ms = 0;
    s++;
  }
}

ISR(PCINT2_vect) {
  r++; //1/12 of a rotation

  if (r >= 48) {
    flag = 0;
    r = 0;
  }
}
/*
ISR (PCINT0_vect){
  //switching the last bit of power_on, if more than one second passed since the last switching. The seconds are stored from 7-1 bits, 
  //the state in the last one; also only changes it if the button is released
  if(((PINB&1)==1) && (s!=(power_on>>1))) power_on = (s<<1)| (power_on^1);

}*/
