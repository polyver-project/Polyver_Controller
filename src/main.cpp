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
#include <TinyGPSPlus.h>

#include <DriveController.hpp>

// ---- Globals ----------------------------
String command;
TinyGPSPlus gps_decoder;
float lat, lon;
unsigned long last_gps_update = 0;

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
    DriveController::SetMode(DRIVE_MODE_PATH);

    // Initial path setup
    // driveController.driveMD.setM1Speed(200);
    // DriveController::driveMD->setSpeeds(200, 0);
    // analogWrite(9, 50);


    Serial3.begin(9600);
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
                uint8_t space_loc = command.lastIndexOf(' ');

                if (space_loc == -1) {
                    return;
                }

                DriveController::ManualMove(
                    command.substring(command.indexOf('s')+2, space_loc).toInt(), 
                    command.substring(space_loc).toInt());
            }
            break;

        case 'l':
            {
               DriveController::ManualLeftMove(
                    command.substring(command.indexOf('l')+2).toInt()); 
            }
            break;
        
        case 'r':
            {
               DriveController::ManualRightMove(
                    command.substring(command.indexOf('r')+2).toInt()); 
            }
            break;


        default:
            break;
    }



    // Other commands
}

// ---- Main entry -------------------------
void loop() {
    // Command handling
    if (buildCommand()) {
        handleCommand(command);
        command = "";       // Clear command
    }

    // Update Drive Controller
    DriveController::IrregularUpdate(1000);

    // GPS data
    if (Serial3.available()) {
        int inByte = Serial3.read();
        gps_decoder.encode(inByte);

        if (gps_decoder.location.isValid() && ((millis() - last_gps_update) > 5000)) {
            Serial.print("lat,");
            Serial.print(gps_decoder.location.lat(), 6);
            Serial.print(F(",lon,"));
            Serial.print(gps_decoder.location.lng(), 6);
            Serial.println();

            last_gps_update = millis();
        }
    }


    // ---- Safety Monitoring --------------
    if (!Serial) {
        DriveController::Kill();
    }
}