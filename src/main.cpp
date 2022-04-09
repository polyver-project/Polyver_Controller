/**
 * @file main.cpp
 * @author Diego Andrade 
 * @brief Kinematic controller for Polyver robots, overall handler of modules
 * @version 0.1
 * @date 2022-04-09
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#include <Arduino.h>
#include "DriveController.hpp"

DriveController driveController;

void setup() {
    // Serial setup
    // Serial.begin(115200);
    Serial.begin(9600);         // For easy debugging

    // Motor setup
    
}

void loop() {

    String command = Serial.readString();

}
