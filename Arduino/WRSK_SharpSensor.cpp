#if defined(ARDUINO) && ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

#include "WRSK_SharpSensor.h"

/*
 * sharpdistance.cpp Sharp GP2Dxx IR sensor functions
 * taken from https://code.google.com/p/rbots/
 *
 */

WRSK_SharpSensor::WRSK_SharpSensor(int _sensPin)
{
  this->sensType = SHARP_GP12;
  this->sensPin = _sensPin;
  this->dbgLevel = 0;
}

WRSK_SharpSensor::WRSK_SharpSensor(int _sensType, int _sensPin, int _dbgLevel)
{
  this->sensType = _sensType;
  this->sensPin = _sensPin;
  this->dbgLevel = _dbgLevel;
}

float WRSK_SharpSensor::read(void)
{
  if (this->sensType == SHARP_GP12)
    return readGP2D12Range(this->sensPin, this->dbgLevel);
  else if (this->sensType == SHARP_GP120)
    return readGP2D120Range(this->sensPin, this->dbgLevel);
  else
    return -1;
}

float WRSK_SharpSensor::readGP2D12Range(int pin, int debugLevel) 
{
  float val;
  int tmp;
  
  tmp = analogRead(pin);
  if(tmp < 3)
    return -1;  //invalid value
    
  val = (6787.0 / ((float)tmp - 3.0))-4.0;
  if (debugLevel > 1) 
  {
    Serial.print("Distance: ");
    Serial.print(val, DEC);
    Serial.println(" cm");
  }
  return val;
}

float WRSK_SharpSensor::readGP2D120Range(int pin, int debugLevel) 
{
  float val;
  int tmp;
  
  tmp = analogRead(pin);
  if(tmp < 3)
    return -1;  //invalid value
    
  val = (2914.0 / ((float)tmp + 5.0))-1.0;
  if (debugLevel > 1) 
  {
    Serial.print("Distance: ");
    Serial.print(val, DEC);
    Serial.println(" cm");
  }
  return val;
}

