

#include "pid.h"
#include <avr/io.h>
#include <stdint.h>

int pid(int rpm, int setpoint){ //it takes current and target rpm scaled from 0-255
    unsigned int pwm;
    if(rpm>255) rpm = 255;
    float error, kp = 0.2, ki, kd;
    error = setpoint - rpm;
    pwm = kp*error;
    return (int)pwm;//this gets added to the current setting

}
