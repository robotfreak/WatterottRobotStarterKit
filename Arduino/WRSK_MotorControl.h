#ifndef MOTORCONTROL_H
#define MOTORCONTROL_H

#if defined(ARDUINO) && ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

#define MAX_SPEED_LIMIT   150
class WRSK_MotorControl {
  public: 
    WRSK_MotorControl();
    WRSK_MotorControl(int _dirPin1, int _pwmPin1, int _dirPin2, int _pwmPin2, int _dbgLevel);
    WRSK_MotorControl(int _dirPin1, int _pwmPin1, int _brkPin1, int _dirPin2, int _pwmPin2, int _brkPin2, int _dbgLevel);
    unsigned long driveWheels(int valueLeft, int valueRight); 
    void driveWheelsRamp(int valueLeft, int valueRight, int dly); 

  private:
    boolean useBrake;
    int last_speed1, last_speed2;
    int dirPin1, dirPin2;
    int pwmPin1, pwmPin2;
    int brkPin1, brkPin2;
    int dbgLevel;
    long maxRunTime;

    void initMotorControl(void); 
};
#endif /* MOTORCONTROL_H */

