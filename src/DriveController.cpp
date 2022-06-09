/**
 * @file DriveController.cpp
 * @author Diego Andrade
 * @brief implementation of DriveController.h
 * @version 0.1
 * @date 2022-04-09
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#include "DriveController.hpp"


// ---- Static Initializations -------------
uint8_t DriveController::drive_mode = DRIVE_MODE_MANUAL;
unsigned long DriveController::last_manual_speed_update_ms = 0;

double* DriveController::pid_left_input;
double* DriveController::pid_left_setpoint;
double* DriveController::pid_left_output;

double* DriveController::pid_right_input;
double* DriveController::pid_right_setpoint;
double* DriveController::pid_right_output;

AutoPID* DriveController::pid_left;
AutoPID* DriveController::pid_right;

PolarPoint* DriveController::path;
uint32_t DriveController::path_head = 0;
uint32_t DriveController::path_tail = 0;

unsigned long DriveController::last_irregular_update_ms = 0;

uint16_t DriveController::over_power_accumilator = 1;
bool DriveController::left_disabled = false;
bool DriveController::right_disabled = false;

DualTB9051FTGMotorShield DriveController::driveMD;
int DriveController::left_output = 0;
int DriveController::right_output = 0;

Encoder DriveController::leftEncoder(DRIVE_LEFT_PIN_A, DRIVE_LEFT_PIN_B);
Encoder DriveController::rightEncoder(DRIVE_RIGHT_PIN_A, DRIVE_RIGHT_PIN_B);

// ---- Public Methods ---------------------
void DriveController::Init() {
    //  Shield setup
    driveMD.init();
    driveMD.flipM1(true);
    driveMD.flipM2(false);

    // PID setup
    pid_left_input = new double;
    pid_left_setpoint = new double;
    pid_left_output = new double;

    pid_right_input = new double;
    pid_right_setpoint = new double;
    pid_right_output = new double;

    pid_left = new AutoPID(
        pid_left_input, pid_left_setpoint, pid_left_output,
        DRIVE_MIN_SPEED, DRIVE_MAX_SPEED,
        DRIVE_LEFT_KP, DRIVE_LEFT_KI, DRIVE_LEFT_KD
    );

    pid_right = new AutoPID(
        pid_right_input, pid_right_setpoint, pid_right_output,
        DRIVE_MIN_SPEED, DRIVE_MAX_SPEED,
        DRIVE_RIGHT_KP, DRIVE_RIGHT_KI, DRIVE_RIGHT_KD
    );

    pid_left->setBangBang(0,0);
    pid_right->setBangBang(0,0);

    pid_left->setTimeStep(100);
    pid_right->setTimeStep(100);

    *pid_left_input    = 0;
    *pid_left_setpoint = 0;

    *pid_right_input    = 0;
    *pid_right_setpoint = 0;

    // Path setup
    path = new PolarPoint[DRIVE_PATH_BUFFER_SIZE];

    // Inserting a keep position as first path point
    path[0].r_inches = 0;
    path[0].rads = 0;
}

void DriveController::Kill() {
    SetLeft(0);
    SetRight(0);
    drive_mode = DRIVE_MODE_MANUAL;
}

void DriveController::ManualMove(int left_speed, int right_speed) {
    if (drive_mode != DRIVE_MODE_MANUAL) {
        return;
    }

    // Update watch dog timer
    last_manual_speed_update_ms = millis();

    // Set output
    left_output = left_speed;
    right_output = right_speed;
}

void DriveController::ManualLeftMove(int speed) {
    if (drive_mode != DRIVE_MODE_MANUAL) {
        return;
    }

    // Update watch dog timer
    last_manual_speed_update_ms = millis();

    // Set output
    left_output = speed;
}

void DriveController::ManualRightMove(int speed) {
    if (drive_mode != DRIVE_MODE_MANUAL) {
        return;
    }

    // Update watch dog timer
    last_manual_speed_update_ms = millis();

    // Set output
    right_output = speed;
}

void DriveController::Move(int8_t delta_x, int8_t delta_y) {
    if (drive_mode != DRIVE_MODE_PATH) {
        return;
    }

    if (delta_x == 0 && delta_y == 0) {
        return;
    }

    Serial.print("Adding path x: ");
    Serial.print(delta_x);
    Serial.print(" y: ");
    Serial.println(delta_y);

    // TODO: Should this return some success code?
    // Buffer full condtions
    if ((path_tail == DRIVE_PATH_BUFFER_SIZE - 1) && (path_head == 0)) {
        return;
    } else if (path_tail + 1 == path_head) {
        return;
    }

    float r_inches = sqrt(delta_x*delta_x + delta_y*delta_y);
    float rads = atan2(delta_y, delta_x);
    
    // Adjust for Y being forward direction
    rads -= 1.57079632679;

    Serial.print("calc r: ");
    Serial.println(r_inches);
    Serial.print("calc rads: ");
    Serial.println(rads);
    // Serial.println(path_head);

    // Update previous point if connituing in same direction, but not holding
    if (rads == 0 && path[path_head].r_inches != 0) {
        path[path_head].r_inches += r_inches;
        return;
    }

    // Adding to path in two moves: turn, move
    // turn
    path_tail = (path_tail == DRIVE_PATH_BUFFER_SIZE - 1) ? 0 : path_tail + 1;
    path[path_tail].rads = rads;
    path[path_tail].r_inches = 0;
    
    // move
    path_tail = (path_tail == DRIVE_PATH_BUFFER_SIZE - 1) ? 0 : path_tail + 1;
    path[path_tail].r_inches = r_inches;
    path[path_tail].rads = 0;

    Serial.print("Path added: ");
    Serial.println(path_tail);
}

void DriveController::StrictUpdate() {

    switch (drive_mode) {
        case DRIVE_MODE_MANUAL:
            // Watch dog timer
            if ((millis() - last_manual_speed_update_ms) > DRIVE_MANUAL_TIMEOUT_MS) {
                left_output = 0;
                right_output = 0;
            }
            break;
        
        case DRIVE_MODE_PATH: 
            // Serial.println("Updating path: ");
            if (path[path_head].r_inches == 0 && path[path_head].rads == 0) {
                left_output = 0;
                right_output = 0; 
            } else {
                // Get encoder positions
                *pid_left_input = leftEncoder.read();
                *pid_right_input = rightEncoder.read();

                // Calculate setpoint
                // Turning
                if (path[path_head].r_inches == 0) {
                    *pid_left_setpoint = -1.0 * path[path_head].rads * DRIVE_IN_CENTER_TO_WHEEL * DRIVE_TICKS_PER_IN;
                    *pid_right_setpoint = path[path_head].rads * DRIVE_IN_CENTER_TO_WHEEL * DRIVE_TICKS_PER_IN;
                } else {
                    *pid_left_setpoint = path[path_head].r_inches * DRIVE_TICKS_PER_IN;
                    *pid_right_setpoint = path[path_head].r_inches * DRIVE_TICKS_PER_IN;
                }

                // Update output
                pid_left->run();
                pid_right->run();
                
                // Set output
                left_output = (pid_left->atSetPoint(DRIVE_SETPOINT_DEAD_ZONE)) ? 0 : *pid_left_output;
                right_output = (pid_right->atSetPoint(DRIVE_SETPOINT_DEAD_ZONE)) ? 0 : *pid_right_output;
            }
            break;
        
        default:
            left_output = 0;
            right_output = 0; 
            break;
    }

    // Just disable and return if not moving
    if (left_output == 0 && right_output == 0) {
        SetLeft(0);
        SetRight(0);
        return;
    }

    // // Motor fault handling
    // // if (driveMD.getM1Fault() || driveMD.getM2Fault()) {
    // //     left_output = 0;
    // //     right_output = 0;

    // //     Serial.println("Fault");
    // // }

    // Power limit handling
    if ((driveMD.getM1CurrentMilliamps() > DRIVE_MAX_MILLIAMPS) || (driveMD.getM2CurrentMilliamps() > DRIVE_MAX_MILLIAMPS)) {
        over_power_accumilator++;
    } else if (over_power_accumilator > 1){
        over_power_accumilator--;
    }

    left_output /= over_power_accumilator;
    right_output /= over_power_accumilator;

    // // Serial.println(left_output);
    // // Serial.println(right_output);
    // // Serial.println();

    // Set motor speeds
    SetLeft(left_output);
    SetRight(right_output);
}

void DriveController::IrregularUpdate(unsigned long lose_period_ms) {
    // Check if update is needed
    if ((millis() - last_irregular_update_ms) < lose_period_ms) {
        return;
    }
    

    switch (drive_mode) {
        case DRIVE_MODE_PATH:
            CalculateNextPath();
            break;
        
        default:
            break;
    }
    
    // Lastly, update "timer"
    last_irregular_update_ms = millis();
}


void DriveController::SetMode(uint8_t mode) {
    switch (mode)
    {
    case DRIVE_MODE_MANUAL:
        drive_mode = DRIVE_MODE_MANUAL;
        Serial.println("Manual mode");
        break;

    case DRIVE_MODE_PATH:
        drive_mode = DRIVE_MODE_PATH;
        Serial.println("Path Follower mode");
        break;

    default:
        break;
    }
}

// ---- Private Methods --------------------
void DriveController::SetLeft(int speed) {
    // Dead zoning
    // if (left_output <= SPEED_DEAD_ZONE && left_output >= (-1 * SPEED_DEAD_ZONE)) {
    //     left_output = 0;
    // }

    // No action needed
    if (speed == 0 && left_disabled) {
        return;
    }

    // Disable once
    if (speed == 0) {
        left_output = 0;
        left_disabled = true;
        driveMD.setM1Speed(0);
        driveMD.disableM1Driver();
        return;
    } 

    // Re-enable once
    if (left_disabled) {
        driveMD.enableM1Driver();
        left_disabled = false;
    }

    // Speed limiting
    left_output  = (speed > DRIVE_MAX_SPEED)  ? DRIVE_MAX_SPEED : ((speed < DRIVE_MIN_SPEED) ? DRIVE_MIN_SPEED : left_output);

    // Set output
    driveMD.setM1Speed(left_output);
}

void DriveController::SetRight(int speed) {
    // Dead zoning
    // if (right_output <= SPEED_DEAD_ZONE && right_output >= (-1 * SPEED_DEAD_ZONE)) {
    //     right_output = 0;
    // }

    // No action needed
    if (speed == 0 && right_disabled) {
        return;
    }

    // Disable once
    if (speed == 0) {
        right_output = 0;
        right_disabled = true;
        driveMD.setM2Speed(0);
        driveMD.disableM2Driver();
        return;
    } 

    // Re-enable once
    if (right_disabled) {
        driveMD.enableM2Driver();
        right_disabled = false;
    }

    // Speed limiting
    right_output  = (speed > DRIVE_MAX_SPEED)  ? DRIVE_MAX_SPEED : ((speed < DRIVE_MIN_SPEED) ? DRIVE_MIN_SPEED : right_output);

    // Set output
    driveMD.setM2Speed(right_output);
}


/**
 * @brief Calculate next path to follow if reached previous direction
 * 
 * @return true if moved on to new path 
 * @return false if still executing last path
 */
bool DriveController::CalculateNextPath() {
    // Check if at set point
    if (pid_left->atSetPoint(DRIVE_SETPOINT_DEAD_ZONE) && pid_right->atSetPoint(DRIVE_SETPOINT_DEAD_ZONE)) {
        
        // Check if next path available to move to
        if (path_head != path_tail) {
            Serial.print("Passing ");
            Serial.println(path_head);
            path_head = (path_head == (DRIVE_PATH_BUFFER_SIZE - 1)) ? 0 : (path_head + 1);
            leftEncoder.write(0);
            rightEncoder.write(0);
        }

        return true;
    }

    return false;
}