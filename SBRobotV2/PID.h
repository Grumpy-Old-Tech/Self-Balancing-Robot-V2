//
// PID Class
//

#ifndef PID_H
#define PID_H

#include "Arduino.h"

class PID
{
  public:
    PID();
    void reset();
    void setTuningParameters(float p, float i, float d, bool action);
    void setDeadband(float db);
    void setOutputLimited(float opMin, float opMax);
    float calculate();

  private:
    float pTerm;
    float iTerm;
    float dTerm;
    bool  revAction;
    float iMemory;
    float lastError;
    float outputMin;
    float outputMax;
    float deadband;

  public:
    float setpoint;
    float processVariable;
    float output;
    float selfBalanceSetpoint;
};

#endif
