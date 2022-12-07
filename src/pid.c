

#include "pid.h"
#include <avr/io.h>
#include <stdint.h>

unsigned int pwm(int rpm, int setpoint){
    unsigned int pwm;
    if(rpm>255) rpm = 255;
    float error, kp = 0.2, ki, kd;
    error = setpoint - rpm;
    pwm = kp*error;
    if(pwm<0) pwm = 0;
    return (short)pwm;

}
