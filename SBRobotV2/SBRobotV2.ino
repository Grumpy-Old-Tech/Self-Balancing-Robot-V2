
//  SBRobot V2
//  - this version: 1.0
//  - this code by: Grumpy Old Tech - Neville Kripp (12th May 2018)
//
//
// Special thanks to Jeff Rowsberg for his work on the I2Cdev and MPU library
//
// Feel fee to use and abuse my code as you wish taking into account the that my changes are also 
// subject to the MIT license below.


/* ==========  LICENSE  ==================================
 I2Cdev device library code is placed under the MIT license
 Copyright (c) 2011 Jeff Rowberg
 
 Permission is hereby granted, free of charge, to any person obtaining a copy
 of this software and associated documentation files (the "Software"), to deal
 in the Software without restriction, including without limitation the rights
 to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 copies of the Software, and to permit persons to whom the Software is
 furnished to do so, subject to the following conditions:
 
 The above copyright notice and this permission notice shall be included in
 all copies or substantial portions of the Software.
 
 THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 THE SOFTWARE.
 =========================================================
 */


// Setup debug defines
//#define SEND_BATTERY
//#define SEND_ANGLE_CONTROL

// Includes
#include <Wire.h>
#include <I2Cdev.h>
#include <MPU6050_6Axis_MotionApps20.h>
#include "PID.h"

// Digital Pins
#define BATTERY_INDICATOR_PIN   8
#define READY_INDICATOR_PIN     9
#define LEFT_PULSE_PIN          4
#define LEFT_DIRECTION_PIN      5
#define RIGHT_PULSE_PIN         6
#define RIGHT_DIRECTION_PIN     7

// Analog Pins
#define BATERY_VOLTAGE_PIN      0

// State Engine
#define MPU_INIT                0
#define STARTING                1
#define BALANCING               2
#define BATTERY_LOW             3
#define MPU_ERROR               4
int currentState;

// PID Paramaters
#define pidP                    15
#define pidI                    1.5
#define pidD                    2
#define pidDB                   5
#define pidOPMin                -400
#define pidOPMax                400

// MPU
MPU6050         mpu;
uint8_t         devStatus;
uint16_t        packetSize;
uint16_t        fifoCount;
uint8_t         fifoBuffer[64]; 

// Motion
Quaternion      q;
VectorFloat     gravity;
float           ypr[3];

// Bluetooth Comms
byte            receivedByte;
int             receivedCounter;

// Loop Timing
unsigned long   loopTimer;
unsigned long   loopTime = 8000;

// Control
PID             pid;
float           turnSpeed;
float           maxSpeed;
float           angle;
float           outputLeft;
float           outputRight;
int             leftMotor;
volatile int    throttleLeftMotor;
int             throttleCounterLeftMotor;
int             throttleLeftMotorMemory;
int             rightMotor;
volatile int    throttleRightMotor;
int             throttleCounterRightMotor;
int             throttleRightMotorMemory;


//**************************************
// Setup Function
//**************************************

void setup() {
  
  Wire.begin();
  Wire.setClock(400000);

  Serial.begin(9600);
  while (!Serial);
  //delay(1000);
  //Serial.println("AT");
  //delay(2000);
  //Serial.println("AT+NAMERobot");
  //delay(2000);

  
  // Setup Digital Pins
  pinMode(BATTERY_INDICATOR_PIN, OUTPUT);                     // Setup Digital Pins
  pinMode(READY_INDICATOR_PIN, OUTPUT);
  pinMode(LEFT_PULSE_PIN, OUTPUT);
  pinMode(LEFT_DIRECTION_PIN, OUTPUT);
  pinMode(RIGHT_PULSE_PIN, OUTPUT);
  pinMode(RIGHT_DIRECTION_PIN, OUTPUT);

 digitalWrite(BATTERY_INDICATOR_PIN, HIGH);
 digitalWrite(READY_INDICATOR_PIN, HIGH);
 
  TCCR2A = 0;                                                 // Setup Timed Interrupt for motor control - execute ISR(TIMER2_COMPA_vect) routine every 20 uS
  TCCR2B = 0;
  TIMSK2 |= (1 << OCIE2A);
  TCCR2B |= (1 << CS21);
  OCR2A = 39;
  TCCR2A |= (1 << WGM21);

  pid.reset();                                                // Setup Control Parameters
  pid.setTuningParameters(pidP, pidI, pidD, false); 
  pid.setDeadband(pidDB);
  pid.setOutputLimited(pidOPMin, pidOPMax);
  turnSpeed = 30;
  maxSpeed = 150;
  
  Serial.println(F("Initializing I2C devices..."));           // Initialise the MPU
  mpu.initialize();
  Serial.println(F("Testing device connection..."));
  if (mpu.testConnection()) {
    
    Serial.println(F("MPU Connection sucessful"));
    Serial.println(F("Initialising DMP..."));
    devStatus = mpu.dmpInitialize();

    mpu.setXGyroOffset(28);                                   // Set from setup application
    mpu.setYGyroOffset(-5);
    mpu.setZGyroOffset(-52);
    mpu.setXAccelOffset(-5879);
    mpu.setYAccelOffset(-1203);
    mpu.setZAccelOffset(1188);

    if (devStatus == 0) {                                     // If all good then set state to Init else Error

      Serial.println(F("Enabling DMP..."));
      mpu.setDMPEnabled(true);
      Serial.println(F("DMP ready"));
      packetSize = mpu.dmpGetFIFOPacketSize();
      currentState = MPU_INIT;
     }
     else {

      Serial.print(F("DMP Initialisation failed (code"));
      Serial.print(devStatus);
      Serial.println(F(")"));

      currentState = MPU_ERROR;
    }
  }
  else {                                                       // Set state to Error
    
    Serial.println(F("MPU Connection failed"));

    currentState = MPU_ERROR;
  }
}

//**************************************
// Loop Function
//**************************************

void loop() {

  checkSerialComms();
  checkBatteryVoltage();

  switch (currentState) {

    case MPU_ERROR:
    {
      throttleLeftMotor = 0;                                  // Make sure motor controls are at zero
      throttleRightMotor = 0;

      digitalWrite(BATTERY_INDICATOR_PIN, HIGH);              // Slow flash Ready and Battery indicators = display a fault
      digitalWrite(READY_INDICATOR_PIN, HIGH);
      delay(5000);
      digitalWrite(BATTERY_INDICATOR_PIN, LOW);
      digitalWrite(READY_INDICATOR_PIN, LOW);
      delay(5000);
      break;
    }
    case BATTERY_LOW:
    {
      throttleLeftMotor = 0;                                  // Make sure motor controls are at zero
      throttleRightMotor = 0;

      digitalWrite(BATTERY_INDICATOR_PIN, LOW);               // Turn on low battery indicator

      digitalWrite(READY_INDICATOR_PIN, HIGH);                // Turn off Ready Indicator
      
      break;
    }
    case MPU_INIT:
    {
      throttleLeftMotor = 0;                                  // Make sure motor controls are at zero
      throttleRightMotor = 0;

      int waitTime = 10;                                      //Just want to wait some time for the MPU to settle
      while (waitTime > 0) {

        digitalWrite(READY_INDICATOR_PIN, LOW);               // Quick flash Ready indicator - waiting 10 sec
        delay(500);
        digitalWrite(READY_INDICATOR_PIN, HIGH);
        delay(500);
        waitTime--;
      }
  
      loopTimer = micros() + loopTime;                            // Setup for starting
      currentState = STARTING;
      break;
    }
    case STARTING:
    {
      throttleLeftMotor = 0;                                  // Make sure motor controls are at zero
      throttleRightMotor = 0;
      
      digitalWrite(READY_INDICATOR_PIN, LOW);                 // Leave indicator On to indicate ready
      
      checkMPU();                                             // Get the Angle
      angle = ypr[1] * 180/M_PI;

      if (between(angle, -0.5, 0.5)) {                        // Change to Balance if upright

        currentState = BALANCING;
      }
      break;
    }
    case BALANCING:
    {
      checkMPU();                                             // Get the Angle
      angle = ypr[1] * 180/M_PI;

      if (!between(angle,-30,30)) {                           // Change back to Starting if toppled over

        pid.reset();
        currentState = STARTING;
      }
      else {

        calculateDriveSignals();                              // Calculate drive signals
      }
      break;
    }
    default:
    {
      currentState = MPU_ERROR;                               // Set to error is undefined state
      break;
    }
  }

  #ifdef SEND_ANGLE_CONTROL
    Serial.print("State: ");
    Serial.print(currentState);
    Serial.print("\tSetpoint: ");
    Serial.print(pid.setpoint);
    Serial.print("\tSelf Balance Setpoint: ");
    Serial.print(pid.selfBalanceSetpoint);
    Serial.print("\tAngle: ");
    Serial.print(angle);
    Serial.print("\tOutput: ");
    Serial.print(pid.output);
    Serial.print("\tOutput Left: ");
    Serial.print(outputLeft);
    Serial.print("\tOutput Right: ");
    Serial.println(outputRight);
  #endif  

  while(loopTimer > micros());
  loopTimer = micros() + loopTime;
}

//**************************************
// checkMPU Function
//**************************************

void checkMPU() {

  // get current FIFO count
  fifoCount = mpu.getFIFOCount();

  if (fifoCount == 1024) {                                    // check for overflow (this should never happen unless our code is too inefficient)
      
    mpu.resetFIFO();                                          // reset so we can continue cleanly
    Serial.println(F("FIFO overflow!"));
  } 
  else if (fifoCount >= packetSize) {                         // if fifo contains at least one packet, get the last packet 

    while (fifoCount >= packetSize) {

      mpu.getFIFOBytes(fifoBuffer, packetSize);
      fifoCount -= packetSize;
    }

    mpu.dmpGetQuaternion(&q, fifoBuffer);                     // Calculate YPR
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
  }
}

//**************************************
// calculateDriveSignals Function
//**************************************

void calculateDriveSignals() {

  pid.processVariable = angle;                                // Process PID
  pid.calculate();

  outputLeft = pid.output;                                    // Calculate control signals
  outputRight = pid.output;

  if (receivedByte & 0b00000010) {                            // Turn Left
    outputLeft += turnSpeed;
    outputRight -= turnSpeed;
  }

  if (receivedByte & 0b00000001) {                            // Turn Right
    
    outputLeft -= turnSpeed;
    outputRight += turnSpeed;
  }

  if (receivedByte & 0b00000100) {                            // Forward
    
    if (pid.setpoint > -2.5) {
      
      pid.setpoint -= 0.05;
    }
    if (pid.output > -maxSpeed) {
      
      pid.setpoint -= 0.005;
    }
  }

  if (receivedByte & 0b00001000) {                            // Reverse
    
    if (pid.setpoint < 2.5) {
      
      pid.setpoint += 0.05;
    }
    if (pid.output < maxSpeed) {
      
      pid.setpoint += 0.005;
    }
  }
  
  if (!(receivedByte & B00001100)) {                          // Not forward or reverse
    
    if (pid.setpoint > 0.5) {
      
      pid.setpoint -= 0.05;
    }
    else if (pid.setpoint < -0.5) {
      
      pid.setpoint += 0.05;
    }
    else {
      
      pid.setpoint = 0;
    }
  }

  if (pid.setpoint == 0) {                                    // Adjust balance setpoint
    
    if (pid.output < 0) {
      
      pid.selfBalanceSetpoint += 0.0015;
    }
    if (pid.output > 0) {
      
      pid.selfBalanceSetpoint -= 0.0015;
    }
  }

    
  // Motor Calculations

  if (outputLeft > 0) {                                       // Compensate for the non-linear behaviour of the stepper motors
    
    outputLeft = 405 - (1 / (outputLeft + 9)) * 5500;
  }
  else if (outputLeft < 0) {
    
    outputLeft = -405 - (1 / (outputLeft - 9)) * 5500;
  }

  if (outputRight > 0) {
    
    outputRight = 405 - (1 / (outputRight + 9)) * 5500;
  }
  else if (outputRight < 0) {
    
    outputRight = -405 - (1 / (outputRight - 9)) * 5500;
  }

  if (outputLeft > 0) {                                       // Calculate the needed pulse time for the left and right stepper motor controllers
        
    leftMotor = 400 - outputLeft;
  }
  else if (outputLeft < 0) {
    
    leftMotor = -400 - outputLeft;
  }
  else {
    
    leftMotor = 0;
  }

  if (outputRight > 0) {
    
    rightMotor = 400 - outputRight;
  }
  else if (outputRight < 0) {
    
    rightMotor = -400 - outputRight;
  }
  else {
    
    rightMotor = 0;
  }

  throttleLeftMotor = leftMotor;                              // Copy for interrupt to use                
  throttleRightMotor = rightMotor;
}

//**************************************
// Motor Drive Interrupt Function
//**************************************

ISR(TIMER2_COMPA_vect) {
  
  // Left motor pulse calculations
  throttleCounterLeftMotor ++;                                //Increase the throttle_counter_left_motor variable by 1 every time this routine is executed
  if (throttleCounterLeftMotor > throttleLeftMotorMemory) {   //If the number of loops is larger then the throttle_left_motor_memory variable
    throttleCounterLeftMotor = 0;                             //Reset the throttle_counter_left_motor variable
    throttleLeftMotorMemory = throttleLeftMotor;              //Load the next throttle_left_motor variable
    if (throttleLeftMotorMemory < 0) {                        //If the throttle_left_motor_memory is negative
      PORTD &= 0b01111111;                                    //Set output 7 low to reverse the direction of the stepper controller
      throttleLeftMotorMemory *= -1;                          //Invert the throttle_left_motor_memory variable
    }
    else {
      PORTD |= 0b10000000;                                    // Set output 7 high for a forward direction of the stepper motor
    }
  }
  else if (throttleCounterLeftMotor == 1) {
    PORTD |= 0b01000000;                                      //Set output 6 high to create a pulse for the stepper controller
  }
  else if (throttleCounterLeftMotor == 2) {
    PORTD &= 0b10111111;                                      //Set output 6 low because the pulse only has to last for 20us 
  }

  // Right motor pulse calculations
  throttleCounterRightMotor ++;                               //Increase the throttle_counter_right_motor variable by 1 every time the routine is executed
  if (throttleCounterRightMotor > throttleRightMotorMemory) { //If the number of loops is larger then the throttle_right_motor_memory variable
    throttleCounterRightMotor = 0;                            //Reset the throttle_counter_right_motor variable
    throttleRightMotorMemory = throttleRightMotor;            //Load the next throttle_right_motor variable
    if (throttleRightMotorMemory < 0) {                       //If the throttle_right_motor_memory is negative
      PORTD |= 0b00100000;                                    //Set output 5 low to reverse the direction of the stepper controller
      throttleRightMotorMemory *= -1;                         //Invert the throttle_right_motor_memory variable
    }
    else {
      PORTD &= 0b11011111;                                    //Set output 5 high for a forward direction of the stepper motor
    }
  }
  else if (throttleCounterRightMotor == 1) {
    PORTD |= 0b00010000;                                      //Set output 4 high to create a pulse for the stepper controller
  }
  else if (throttleCounterRightMotor == 2) {
    PORTD &= 0b11101111;                                      //Set output 4 low because the pulse only has to last for 20us
  }
}

//**************************************
// Helper Routines
//**************************************

void checkSerialComms() {

   if (Serial.available()) {
    
    receivedByte = Serial.read();
    receivedCounter = 0;
  }
  if (receivedCounter <= 25) {
    
    receivedCounter++;
  }
  else {
    
    receivedByte = 0x00;
  }
}

void checkBatteryVoltage() {

  int rawValue = analogRead(BATERY_VOLTAGE_PIN); 
  int batteryVoltage = map(rawValue, 0, 1023, 0, 1280);
  
  #ifdef SEND_BATTERY                                         // Calibrate battery 
    Serial.print("Raw: ");                                    //   - adjust supply voltage until raw is 1024 
    Serial.print(rawValue);                                   //   - then set high map value to be
    Serial.print("\tMapped: ");                               //     actual voltage to 2 decimal places
    Serial.println(batteryVoltage);                           //      eg 12.8V = 1280
  #endif  

  if (between(batteryVoltage, 800, 1050)) {

    currentState = BATTERY_LOW;
    Serial.println("Battery Very Low - disabled drive");
  }
  else if (batteryVoltage < 1070) {

    Serial.println("Battery Low");
  }
}

bool between(float value, float low, float high) {

  if (value > low && value < high) {

    return true;
  }
  else {

    return false;
  }
}


