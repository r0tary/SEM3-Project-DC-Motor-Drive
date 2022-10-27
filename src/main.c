/*
 * SEM_3_Project
 * Electrical drive to control the speed of a DC motor
 * Created: 26.10.2022.
 * Author : Group 3
 */ 

#define F_CPU 16000000UL

#include <avr/io.h>
#include <util/delay.h>

#include "i2cmaster.h"
#include "PWM.h"


int main(void){
  int D_C = 90;
  int TOP = 256;
  i2c_init();//initialize I2C communication
  PWM_init();//initialize PWM
  OCR0B = TOP*0.9;

  while(1){

  }
  return 0;
}

