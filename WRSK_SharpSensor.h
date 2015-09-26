#ifndef SHARPDISTANCE_H
#define SHARPDISTANCE_H

#if defined(ARDUINO) && ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

#define SHARP_GP12 1
#define SHARP_GP120 2

class WRSK_SharpSensor {
  public:
    WRSK_SharpSensor(int _sensPin);
    WRSK_SharpSensor(int _sensType, int _sensPin, int _dbgLevel);
    float read();

  private:
    int sensType;
    int sensPin;
    int dbgLevel;
    float readGP2D12Range(int pin, int debugLevel); 
    float readGP2D120Range(int pin, int debugLevel); 
};
#endif /* SHARPDISTANCE_H */

