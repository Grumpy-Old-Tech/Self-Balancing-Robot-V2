//
// Settings Class
//

#ifndef SETTINGS_H
#define SETTINGS_H

#include "Arduino.h"
#include <EEPROM.h>

// Settings Version 
#define SETTING_VER   "SBR2.0"

// Parameters Structure
typedef struct {
  float   pidP;
  float   pidI;
  float   pidD;
  float   pidDB;
  float   pidOPMin;
  float   pidOPMax;
  float   turnSpeed;
  float   maxSpeed;
  char    ver[sizeof(SETTING_VER)];
} ParametersType;

class Settings
{
  public:
    Settings(ParametersType defaults);
    void save();
    void sendParamatersOnSerial();
    void processCommandString(String command);
    
    ParametersType *parameters;

  private:
    byte buffer[sizeof(ParametersType)];
};

#endif
