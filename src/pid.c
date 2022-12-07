

#include "pid.h"
#include <avr/io.h>
#include <stdint.h>

float pid(int rpm, int setpoint){ //it takes current and target rpm scaled from 0-255
    int pwm;
    if(rpm>255) rpm = 255;//if there is a measurement error
    float error, kp = 0.005, ki, kd;
    error = setpoint - rpm;
    pwm = kp*error;
    return pwm;//this gets added to the current setting

} 
