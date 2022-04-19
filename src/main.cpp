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

DriveController* driveController;
// DualTB9051FTGMotorShield md;

void setup() {
    // Serial setup
    // Serial.begin(115200);
    Serial.begin(9600);                         // For easy debugging
    Serial.println("\nStarting program");

    

    driveController = new DriveController();

    // Motor setup
    //driveController.Move(0, 3);

    //driveController->SetSpeed(100, 100);

    // md.init();
    // md.flipM1(true);
    // md.flipM2(false);
    // md.enableDrivers();
    // delay(1);
    // md.setSpeeds(100, 100);
}

void loop() {
    // String command = Serial.readString();
    // driveController.UpdateOutput();
    // driveController.CalculateNextPath();

    Serial.print("Left: ");
    Serial.print(driveController->ReadLeft());
    Serial.print("    Right: ");
    Serial.print(driveController->ReadRight());
    Serial.println();

    // Serial.print("Fault1: ");
    // Serial.print(md.getM1Fault());
    // Serial.print("    Fault2: ");
    // Serial.print(md.getM2Fault());
    // Serial.println();

    delay(200);
    driveController->SetSpeed(100, 100);
    delay(2000);
    driveController->SetSpeed(0, 0);
    delay(5000);
}
