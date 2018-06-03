//
// PID Class
//

#include "Arduino.h"
#include "PID.h"

PID::PID() {
  iMemory = 0;
  lastError = 0;
  outputMin = 0;
  outputMax = 100;
  deadband = 0;
  revAction = 0;
}

void PID::reset() {
  output = 0;
  iMemory = 0;
  selfBalanceSetpoint = 0;
}

void PID::setTuningParameters(float p, float i, float d, bool action) {
  pTerm = p;
  iTerm = i;
  dTerm = d;
  revAction = action;
}
    
void PID::setDeadband(float db) {
  deadband = db;
}
    
void PID::setOutputLimited(float opMin, float opMax) {
  outputMin = opMin;
  outputMax = opMax;
  iMemory = constrain(output, outputMin, outputMax);
}

float PID::calculate() {
  
  float error;
  error = processVariable - setpoint - selfBalanceSetpoint;

  // Brake function
  if (output > 10 || output < -10) {
    error += output * 0.015;
  }

  iMemory += iTerm * error;
  iMemory = constrain(iMemory, outputMin, outputMax);

  output = pTerm * error + iMemory + dTerm * (error - lastError);
  output = constrain(output, outputMin, outputMax);
  
  lastError = error;

  if (output < deadband && output > -deadband) {
    output = 0;
  }

  if (revAction) {

    output = output * -1.0;
  }
}

