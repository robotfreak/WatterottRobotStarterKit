# Ultraschallsensor Erweiterung

Der Ultraschall Sensor kann anstelle des Sharp Infrarotsensors eingesetzt werden. Der Sensor hat einen breiteren Erfassungswinkel. Auf das Scannen der Umgebung mit dem Servo kann weitestgehend verzichtet werden.

## Was wird benötigt?

* HC-SR04 Ultraschallsensor
* 4-polige Buchsenleisten
* 4-polige Stiftleiste
* 4 Drähte 15cm lang
* Plexiglas Halterung (im Roboter Starer Kit enthalten)
* Lötkolben, Seitenschneider

## Durchführung

Der Sensor wird gemäß Verdrahtungs Schema verdrahtet und mit der Plexiglas Halterung am Servo Horn befestigt

![Sensor](images/HC-SR04-Mod.jpg "Sensor")

## Verdrahtung

![Verdrahtung](../Fritzing/HC-SR04-Erweiterung_Steckplatine.png "Verdrahtung")

## Arduino Sketch

Die Funktionen zum Auslesen des Sensors sind Teil der Watterott Roboter Starter Kit (WRSK) Bibliothek.

Das Beispiel Sketch zum Kollionsvermeider (examples/ObstacleAvoiding) 

```arduino
/*
  ObstacleAvoiding.ino - Watterott Starter Robot Kit (WSRK)
  Author: RobotFreak www.robotfreak.de/blog
  Copyright (c) 2015 RobotFreak All Rights Reserved
 
  For information about the Watterott Starter Robot Kit (WSRK)
  visit http://www.watterott.com/de/StarterKit-Roboter
 
  The minimum circuit:
  * Watterott Starter Robot Kit (WSRK) Arduino Uno + Arduino Motor Shield 
  * Sharp GP2D12 or GP2D120 Infrared distance sensor connected to A3
  * or HC-SR04 Ultrasonic distance sensor connected to pin 6, 7
  * USB cable

  Alternative circuit:
  * Watterott Starter Robot Kit (WSRK) Arduino Pro + Arduino Motor Shield 
  * FTDI compatible USB serial module/cable
  * Sharp GP2D12 or GP2D120 Infrared distance sensor connected to A3
  * or HC-SR04 Ultrasonic distance sensor connected to pin 6, 7
  * optional Adafruit Bluefruit EZ-Link Module connected to FTDI connector

 */
#include <inttypes.h>
#include <Servo.h>
#include "iomapping.h"
#include <WRSK_UltrasonicSensor.h>
#include <WRSK_MotorControl.h>

#define DEBUG_LEVEL 2

#define LOW_SPEED 50
#define MID_SPEED 70
#define TOP_SPEED 100

WRSK_MotorControl motors( m1DirectionControl, m1SpeedControl, m2DirectionControl, m2SpeedControl, DEBUG_LEVEL);
WRSK_UltrasonicSensor us(usEchoPin, usTriggerPin, DEBUG_LEVEL);

int speed_l;
int speed_r;
int dly;
float distance;
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

  Serial.println("WMR-SHR obstacle Avoiding Version 1.0");   

  distance = us.read();  // get distance
  while (distance > 10.0)
  {
    distance =  us.read();  // get distance
    delay(100);
  }
  Serial.println("Let's go");
}


void loop()
{
  int i;

  distance =  us.read();  // get distance

  if (distance > 0.0)
  {
    if (distance <= 10.0)
    {
      motors.driveWheels(0,0);
      delay(500);
      speed_l = -MID_SPEED;
      speed_r = MID_SPEED;
      dly = 5;
      do {
        distance =  us.read();  // get distance
        delay(40);
        motors.driveWheels(speed_l, speed_r);
        //      motors.driveWheelsRamp(speed_l, speed_r, dly);
      } 
      while(distance < 20.0);  
      speed_l = 0;
      speed_r = 0;
      motors.driveWheels(speed_l,speed_r);
      //    motors.driveWheelsRamp(speed_l, speed_r, dly);
      delay(500);
    }
    else if (distance < 30.0)
    {
      speed_l = MID_SPEED;
      speed_r = MID_SPEED;
    }
    else
    {
      speed_l = TOP_SPEED;
      speed_r = TOP_SPEED;
    }
    dly = 10;
    motors.driveWheels(speed_l, speed_r);
    //  motors.driveWheelsRamp(speed_l, speed_r, dly);
  }

  delay(40);

}
```
