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
  i2c_init();//initialize I2C communication
  PWM_init();//initialize PWM

  while(1){

  }
  return 0;
}

