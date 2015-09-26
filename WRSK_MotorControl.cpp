#if defined(ARDUINO) && ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif


#include "WRSK_MotorControl.h"

WRSK_MotorControl::WRSK_MotorControl(void)
{
  this->dirPin1 = 3;
  this->pwmPin1 = 12;
  this->brkPin1 = 9;
  this->dirPin2 = 13;
  this->pwmPin2 = 11;
  this->brkPin2 = 8;
  this->useBrake = true;
  this->dbgLevel = 0;
  initMotorControl();
} 

WRSK_MotorControl::WRSK_MotorControl(int _dirPin1, int _pwmPin1, int _dirPin2, int _pwmPin2, int _dbgLevel)
{
  this->dirPin1 = _dirPin1;
  this->pwmPin1 = _pwmPin1;
  this->dirPin2 = _dirPin2;
  this->pwmPin2 = _pwmPin2;
  this->useBrake = false;
  this->dbgLevel = _dbgLevel;
  initMotorControl();
} 

WRSK_MotorControl::WRSK_MotorControl(int _dirPin1, int _pwmPin1, int _brkPin1, int _dirPin2, int _pwmPin2, int _brkPin2, int _dbgLevel)
{
  this->dirPin1 = _dirPin1;
  this->pwmPin1 = _pwmPin1;
  this->brkPin1 = _brkPin1;
  this->dirPin2 = _dirPin2;
  this->pwmPin2 = _pwmPin2;
  this->brkPin2 = _brkPin2;
  this->dbgLevel = _dbgLevel;
  this->useBrake = false;
  initMotorControl();
} 

void WRSK_MotorControl::initMotorControl(void)
{
  pinMode(this->dirPin1, OUTPUT);
  pinMode(this->dirPin2, OUTPUT);
  pinMode(this->pwmPin1, OUTPUT);
  pinMode(this->pwmPin2, OUTPUT);
  if (this->useBrake == true)
  {
    pinMode(this->brkPin1, OUTPUT);
    pinMode(this->brkPin2, OUTPUT);
    digitalWrite(this->brkPin1, LOW);
    digitalWrite(this->brkPin2, LOW);
  }
  analogWrite(this->pwmPin1, 0);
  analogWrite(this->pwmPin2, 0);
  maxRunTime = 2000;
}


// Drive DC motors to move the robot using values in range -100 to 100 for left and right
unsigned long WRSK_MotorControl::driveWheels(int valueLeft, int valueRight) 
{

  // Set left motor pins to turn in the desired direction
  if (valueLeft < 0){
    digitalWrite(this->dirPin2,LOW);
  }
  else {
    digitalWrite(this->dirPin2,HIGH);
  }
  // Set right motor pins to turn in the desired direction
  if (valueRight < 0){
    digitalWrite(this->dirPin1,LOW);
  }
  else {
    digitalWrite(this->dirPin1,HIGH);
  }
  // Maps "w" values to the wider range that the motor responds to
  valueLeft = map(abs(valueLeft), 0, 100, 0, MAX_SPEED_LIMIT);
  valueRight = map(abs(valueRight), 0, 100, 0, MAX_SPEED_LIMIT);
  analogWrite(this->pwmPin2,valueLeft);
  analogWrite(this->pwmPin1,valueRight);

  return millis() + maxRunTime;
}

void WRSK_MotorControl::driveWheelsRamp(int speed1, int speed2, int dly)
{
  int temp_speed1 = last_speed1;
  int temp_speed2 = last_speed2;

  if (this->dbgLevel > 1)
  {
    Serial.print("MoveRamp: ");
    Serial.print(speed1, DEC);
    Serial.print(", ");
    Serial.print(speed2, DEC);
    Serial.print(", ");
    Serial.println(dly, DEC);
  }
  while ((speed1 != temp_speed1) ||
         (speed2 != temp_speed2))
   {
      if (temp_speed1 > speed1)
        temp_speed1--;
      else if (temp_speed1 < speed1)
        temp_speed1++;

      if (temp_speed2 > speed2)
        temp_speed2--;
      else if (temp_speed2 < speed2)
        temp_speed2++;
      driveWheels(temp_speed1, temp_speed2);
      delay(dly);
   }
}

