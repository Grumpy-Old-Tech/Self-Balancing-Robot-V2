
float output;
float outputLeft;
float outputRight;
int leftMotor, throttleLeftMotor, throttleCounterLeftMotor, throttleLeftMotorMemory;
int rightMotor, throttleRightMotor, throttleCounterRightMotor, throttleRightMotorMemory;

unsigned long loopTimer;
byte  receivedByte;

void setup() {
  // put your setup code here, to run once:

  Serial.begin(9600);           //Start the serial port at 9600 bps


  TCCR2A = 0;                                                               //Make sure that the TCCR2A register is set to zero
  TCCR2B = 0;                                                               //Make sure that the TCCR2A register is set to zero
  TIMSK2 |= (1 << OCIE2A);                                                  //Set the interupt enable bit OCIE2A in the TIMSK2 register
  TCCR2B |= (1 << CS21);                                                    //Set the CS21 bit in the TCCRB register to set the prescaler to 8
  OCR2A = 39;                                                               //The compare register is set to 39 => 20us / (1s / (16.000.000MHz / 8)) - 1
  TCCR2A |= (1 << WGM21);                                                   //Set counter 2 to CTC (clear timer on compare) mode

  loopTimer = micros() + 4000;
}

void loop() {
  // put your main code here, to run repeatedly:

 if (Serial.available()) {
    receivedByte = Serial.read();

    if (receivedByte == 'q') {

        output += 5;
    }
    else if (receivedByte == 'a') {

        output -= 5;
    }
    else if (receivedByte == 's') {

        output = 0;
    }
 }

 outputLeft = output;
 outputRight = output;

     // Compensate for the non-linear behaviour of the stepper motors
  if (outputLeft > 0) {
    outputLeft = 405 - (1/(outputLeft + 9)) * 5500;
  }
  else if (outputLeft < 0) {
    outputLeft = -405 - (1/(outputLeft - 9)) * 5500;
  }
  
  if (outputRight > 0) {
    outputRight = 405 - (1/(outputRight + 9)) * 5500;
  }
  else if (outputRight < 0) {
    outputRight = -405 - (1/(outputRight - 9)) * 5500;
  }

  // Calculate the needed pulse time for the left and right stepper motor controllers
  if (outputLeft > 0) {
    leftMotor = 400 - outputLeft;
  }
  else if (output < 0) {
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

  throttleLeftMotor = leftMotor;
  throttleRightMotor = rightMotor;
    
  Serial.print(output);
  Serial.print(",");
  Serial.print(throttleLeftMotor);
  Serial.print(",");
  Serial.println(throttleRightMotor);
  // Delay 4 milliseconds
  while(loopTimer > micros());
  loopTimer += 4000;
}

//--------------------------
// Interrupt
//--------------------------

ISR(TIMER2_COMPA_vect) {
  
  //Left motor pulse calculations
  throttleCounterLeftMotor ++;                                //Increase the throttle_counter_left_motor variable by 1 every time this routine is executed
  if (throttleCounterLeftMotor > throttleLeftMotorMemory) {   //If the number of loops is larger then the throttle_left_motor_memory variable
    throttleCounterLeftMotor = 0;                             //Reset the throttle_counter_left_motor variable
    throttleLeftMotorMemory = throttleLeftMotor;              //Load the next throttle_left_motor variable
    if (throttleLeftMotorMemory < 0) {                        //If the throttle_left_motor_memory is negative
      PORTD &= 0b01111111;                                    //Set output 7 low to reverse the direction of the stepper controller
      throttleLeftMotorMemory *= -1;                          //Invert the throttle_left_motor_memory variable
  //     Serial.println("r");
    }
    else {
      PORTD |= 0b10000000;                                 // Set output 7 high for a forward direction of the stepper motor
    //  Serial.println("f");
    }
  }
  else if (throttleCounterLeftMotor == 1) {
    PORTD |= 0b01000000;                                      //Set output 6 high to create a pulse for the stepper controller
  }
  else if (throttleCounterLeftMotor == 2) {
    PORTD &= 0b10111111;                                      //Set output 6 low because the pulse only has to last for 20us 
  }

 
  
  //right motor pulse calculations
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

