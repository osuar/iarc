/*
This file links to the header files for the other included files for IR communication
*/

#ifndef ALL_H 
#define ALL_H

#include <stdio.h>
#include <stdlib.h>
#include <avr/io.h>
#include "avr_compiler.h"

#include "usart.h"
#include "adc.h"
#include "servo.h"
#include "filter.h"
#include "trig.h"
#include "struct.h"

#define STEPS 28 //number of steps in a full sweep of the servo
#define NUM_SERVOS 1 //number of servos in use
#define INCREMENT 1 //change in the duty cycle needed to ellicit a one increment step
#define LENGTH 2 // number of block of data that can be held
#define SERVO_SPEED 50 // this determines the ammount of delay between the movement of the servo
#endif

