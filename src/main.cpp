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
#include <Encoder.h>

#include <DriveController.hpp>

// ---- Globals ----------------------------
String command;

// ---- Setup Functions --------------------
ISR(TIMER3_COMPA_vect) {
    DriveController::StrictUpdate();
    // Serial.print("*");
}

void setup() {
    // ---- Serial setup -------------------
    Serial.begin(9600);
    // Serial.begin(115200);

    while (!Serial)
        ; // Wait for serial
    Serial.println("\nStarting program");

    // ---- Periodic Function setup --------
    noInterrupts();
    TCCR3A = 0;
    TCCR3B = 0;
    TCNT3  = 0;

    OCR3A = 16;                                             // 0.001024 seconds
    TCCR3A |= 0;                                            // CTC mode
    TCCR3B |= (1 << WGM32) | (1 << CS32) | (1 << CS30);     // 1024 prescale

    TIMSK3 |= (1 << OCIE3A);

    Serial.println("Setup done");
    interrupts();

    DriveController::Init();
    // DriveController::driveMD.setSpeeds(100, 100);
    // DriveController::driveMD.setM1Speed(100);
    // DriveController::driveMD.setM2Speed(100);
    // DriveController::driveMD.enableDrivers();
    

    // Initial path setup
    // driveController.driveMD.setM1Speed(200);
    // DriveController::driveMD->setSpeeds(200, 0);
    // analogWrite(9, 50);
}

// ---- Command parser ---------------------
/**
 * @return true if command has finished being built
 */
bool buildCommand() {
    if (Serial.available() == 0)
        return false;

    if (Serial.peek() == '\n') {
        Serial.read();      // Consume newline character
        return true;
    }

    command.concat((char)Serial.read());

    return false;
}

void handleCommand(String command) {    
    // command.toLowerCase();

    // Using first char as command indicator for now
    char firstChar = command.charAt(0);

    switch (firstChar) {
        // Move command formatted as "x: ### y: ###"
        case 'x':
            {
                uint8_t y_loc = command.indexOf('y');
            
                // Check for valid y
                if (y_loc == -1) {
                    return;
                }

                DriveController::Move(
                    command.substring(command.indexOf('x')+2, y_loc).toInt(), 
                    command.substring(y_loc+2).toInt());
            }
            break;

        // Mode command formatted as "m: 0-1"
        case 'm':
            {
                DriveController::SetMode(
                    command.substring(command.indexOf('m')+2).toInt());
            }
            break;

        // Setspeed command for manual mode formatted as "s:#left# #right#" 
        case 's': 
            {   
                uint8_t space_loc = command.indexOf(' ');

                if (space_loc == -1) {
                    return;
                }

                DriveController::ManualMove(
                    command.substring(command.indexOf('s')+2, space_loc).toInt(), 
                    command.substring(space_loc).toInt());
            }
            break;

        default:
            break;
    }



    // Other commands
}

// ---- Main entry -------------------------

long x = 0;
uint32_t last_path_head = 0;

void loop() {
    // Command handling
    if (buildCommand()) {
        handleCommand(command);
        command = "";       // Clear command
    }

    // Update Drive Controller
    DriveController::IrregularUpdate();









    // ---- Safety Monitoring --------------
    if (!Serial) {
        DriveController::Kill();
    }
}