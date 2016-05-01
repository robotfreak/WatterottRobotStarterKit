/*
  MotorTest.ino - Watterott Starter Robot Kit (WSRK)
  Author: RobotFreak www.robotfreak.de/blog
  Copyright (c) 2015 RobotFreak All Rights Reserved
 
  For information about the Watterott Starter Robot Kit (WSRK)
  visit http://www.watterott.com/de/StarterKit-Roboter
 
  The minimum circuit:
  * Watterott Starter Robot Kit (WSRK) Arduino Uno + Arduino Motor Shield 
  * USB cable

  Alternative circuit:
  * Watterott Starter Robot Kit (WSRK) Arduino Pro + Arduino Motor Shield 
  * FTDI compatible USB serial module/cable
  * optional Adafruit Bluefruit EZ-Link Module connected to FTDI connector
 
 */
#include <inttypes.h>
#include <Servo.h>
#include "iomapping.h"
#include <WRSK_MotorControl.h>

#define DEBUG_LEVEL 2

WRSK_MotorControl motors( m1DirectionControl, m1SpeedControl, m2DirectionControl, m2SpeedControl, DEBUG_LEVEL);

int speed_l;
int speed_r;
int dly;
int debugLevel = DEBUG_LEVEL;

Servo SensorServo;  // Mit diesem Element wird der Servo gesteuert

void setup() {

  Serial.begin(38400);                 // Sets the baud rate to 38400
  speed_l = 0;
  speed_r = 0;
  // Servo initialiseren und auf 90° stellen
  SensorServo.attach( servoPin );
  SensorServo.write( 90 );
  delay( 500 );

  Serial.println("WMR-SHR MotorTest Version 1.0");   
}


void loop()
{
  int i;

  motors.driveWheels(0,0);
  Serial.println("Left wheel forward");
  for(i=0; i<=100; i++)
  {
    speed_l = i;
    speed_r = 0;
    motors.driveWheels(speed_l,speed_r);
    delay(10);
  }
  for(i=100; i>=0; i--)
  {
    speed_l = i;
    speed_r = 0;
    motors.driveWheels(speed_l,speed_r);
    delay(10);
  }
  Serial.println("Left wheel backward");
  for(i=0; i>=-100; i--)
  {
    speed_l = i;
    speed_r = 0;
    motors.driveWheels(speed_l,speed_r);
    delay(10);
  }
  for(i=-100; i<=0; i++)
  {
    speed_l = i;
    speed_r = 0;
    motors.driveWheels(speed_l,speed_r);
    delay(10);
  }
  Serial.println("Right wheel forward");
  for(i=0; i<=100; i++)
  {
    speed_l = 0;
    speed_r = i;
    motors.driveWheels(speed_l,speed_r);
    delay(10);
  }
  for(i=100; i>=0; i--)
  {
    speed_l = 0;
    speed_r = i;
    motors.driveWheels(speed_l,speed_r);
    delay(10);
  }
  Serial.println("Right wheel backward");
  for(i=0; i>=-100; i--)
  {
    speed_l = 0;
    speed_r = i;
    motors.driveWheels(speed_l,speed_r);
    delay(10);
  }
  for(i=-100; i<=0; i++)
  {
    speed_l = 0;
    speed_r = i;
    motors.driveWheels(speed_l,speed_r);
    delay(10);
  }
  delay(500);
}



