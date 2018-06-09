//
// Settings Class
//

#include "Arduino.h"
#include "Settings.h"

Settings::Settings(ParametersType defaults) {

  memcpy(buffer, &defaults, sizeof(defaults));

  // Read version number from EEPROM
  char savedVersion[sizeof(SETTING_VER)];
  int eepromVersionOffset = sizeof(buffer) - sizeof(SETTING_VER);
  EEPROM.get(eepromVersionOffset,savedVersion);

  // If version in EEPROM is correct, overwrite the default coonfig with eeprom values
  if (strcmp(savedVersion,SETTING_VER) == 0) {

    // Overwrite the structure with EEPROM data
    EEPROM.get(0,buffer);
  }
 
  parameters = (ParametersType *) buffer;
}

void Settings::save() {
  
  EEPROM.put(0,buffer);
}

void Settings::sendParamatersOnSerial() {

    Serial.print("pidP:");
    Serial.println(parameters->pidP);
    Serial.print("pidI:");
    Serial.println(parameters->pidI);
    Serial.print("pidD");
    Serial.println(parameters->pidD);
    Serial.print("pidDB:");
    Serial.println(parameters->pidDB);
    Serial.print("pidOPMin:");
    Serial.println(parameters->pidOPMin);
    Serial.print("pidOPMax:");
    Serial.println(parameters->pidOPMax);
    Serial.print("turnSpeed:");
    Serial.println(parameters->turnSpeed);
    Serial.print("maxSpeed:");
    Serial.println(parameters->maxSpeed);
}

void Settings::processCommandString(String command) {

  Serial.print("Command: ");
  Serial.println(command);

  if (command.startsWith("pidP")) {

    String number = command.substring(4);
    parameters->pidP = number.toFloat();
    Serial.println("OK");
  }
  else if (command.startsWith("pidI")) {

    String number = command.substring(4);
    parameters->pidI = number.toFloat();
    Serial.println("OK");    
  }
  else if (command.startsWith("pidD")) {
    
    String number = command.substring(4);
    parameters->pidD = number.toFloat();
    Serial.println("OK");    
  }
  else if (command.startsWith("pidDB")) {
    
    String number = command.substring(5);
    parameters->pidDB = number.toFloat();
    Serial.println("OK");    
  }
  else if (command.startsWith("pidOPMin")) {

    String number = command.substring(8);
    parameters->pidOPMin = number.toFloat();
    Serial.println("OK");    
  }
  else if (command.startsWith("pidOPMax")) {

    String number = command.substring(8);
    parameters->pidOPMax = number.toFloat();
    Serial.println("OK");    
  }
  else if (command.startsWith("turnSpeed")) {

    String number = command.substring(9);
    parameters->turnSpeed = number.toFloat();
    Serial.println("OK");        
  }
  else if (command.startsWith("maxSpeed")) {

    String number = command.substring(8);
    parameters->maxSpeed = number.toFloat();
    Serial.println("OK");           
  }
  else {

    Serial.println("Invalid");
  }
}

