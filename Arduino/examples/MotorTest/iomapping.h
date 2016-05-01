#ifndef IOMAPPING_H
#define IOMAPPING_H

#if defined(ARDUINO) && ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

/* Ultrasonic pin definition */
#define usTriggerPin   7
#define usEchoPin      6  

/* IR sensor pin definition */
#define irSensPin        A3

/* Servo pin definition */
#define servoPin     5

/* Linesensor pin definition */
#define NUM_LINESENSORS         3  // number of sensors used
#define NUM_SAMPLES_PER_SENSOR  4  // average 4 analog samples per sensor reading
#define line1Pin     A0
#define line2Pin     A1
#define line3Pin     A2

/* Motor Controller Pin definition */
//#define USE_BRAKE_CONTROL

#define m1SpeedControl     3  //M1 Speed Control
#define m2SpeedControl     11 //M2 Speed Control
#define m1DirectionControl 12 //M1 Direction Control
#define m2DirectionControl 13 //M2 Direction Control
#ifdef USE_BRAKE_CONTROL
#define m1BrakeControl     9  //M1 Speed Control
#define m2BrakeControl     8  //M2 Brake Control
#endif


#endif /* IOMAPPING_H */

