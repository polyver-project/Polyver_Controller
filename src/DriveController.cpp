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
double* DriveController::pid_left_input;
double* DriveController::pid_left_setpoint;
double* DriveController::pid_left_output;

double* DriveController::pid_right_input;
double* DriveController::pid_right_setpoint;
double* DriveController::pid_right_output;

AutoPID DriveController::pid_left(
    DriveController::pid_left_input, 
    DriveController::pid_left_setpoint,
    DriveController::pid_left_output,
    0.0,
    300.0,
    KP_LEFT, KI_LEFT, KD_LEFT);

AutoPID DriveController::pid_right( 
    DriveController::pid_right_input, 
    DriveController::pid_right_setpoint,
    DriveController::pid_right_output,
    0.0, 300.0,
    KP_RIGHT, KI_RIGHT, KD_RIGHT);
// int16_t DriveController::left_output = 0;
// int16_t DriveController::right_output = 0;

// uint16_t DriveController::over_power_accumilator = 1;
// float DriveController::I_left_accumilator = 0;
// float DriveController::I_right_accumilator = 0;

PolarPoint* DriveController::path;
uint32_t DriveController::path_head = 0;
uint32_t DriveController::path_tail = 0;

DualTB9051FTGMotorShield DriveController::driveMD;
Encoder DriveController::leftEncoder(DRIVE_LEFT_PIN_A, DRIVE_LEFT_PIN_B);
Encoder DriveController::rightEncoder(DRIVE_RIGHT_PIN_A, DRIVE_RIGHT_PIN_B);

// ---- Definitions ------------------------
void DriveController::Init() {
    //  Shield setup
    driveMD.init();
    driveMD.flipM1(true);                   // Right wheel inverted
    driveMD.flipM2(false);

    // PID setup
    pid_left_input = new double;
    pid_left_setpoint = new double;
    pid_left_output = new double;

    pid_right_input = new double;
    pid_right_setpoint = new double;
    pid_right_output = new double;

    // Path setup
    path = new PolarPoint[PATH_BUFFER_SIZE];

    // Inserting a keep position as first path point
    path[0].r_inches = 0;
    path[0].rads = 0;
}

void DriveController::Kill() {
    driveMD.disableDrivers();
}

void DriveController::Move(int8_t delta_x, int8_t delta_y) {
    Serial.print("Adding path x: ");
    Serial.print(delta_x);
    Serial.print(" y: ");
    Serial.println(delta_y);

    // TODO: Should this return some success code?
    // Buffer full condtions
    if ((path_tail == PATH_BUFFER_SIZE - 1) && (path_head == 0)) {
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

    // Update previous point if connituing in same direction
    if (rads == 0) {
        path[path_head].r_inches += r_inches;
        return;
    }

    // Adding to path in two moves: turn, move
    // turn
    path_tail = (path_tail == PATH_BUFFER_SIZE - 1) ? 0 : path_tail + 1;
    path[path_tail].rads = rads;
    path[path_tail].r_inches = 0;
    
    // move
    path_tail = (path_tail == PATH_BUFFER_SIZE - 1) ? 0 : path_tail + 1;
    path[path_tail].r_inches = r_inches;
    path[path_tail].rads = 0;

    Serial.print("Path added: ");
    Serial.println(path_tail);
}

void DriveController::UpdateOutput() {
    // Serial.println("Updating path: ");
    if (path[path_head].r_inches == 0 && path[path_head].rads == 0) {
        driveMD.disableDrivers();
        return;
    }

    // Calculate input
    *pid_left_input = leftEncoder.read() * DRIVE_IN_PER_ENCODER;
    *pid_right_input = rightEncoder.read() * DRIVE_IN_PER_ENCODER;

    // Calculate setpoint
    // Turning
    if (path[path_head].r_inches == 0) {
        *pid_left_setpoint =  path[path_head].rads * DRIVE_IN_CENTER_TO_WHEEL;
        *pid_right_setpoint = -1 * path[path_head].rads * DRIVE_IN_CENTER_TO_WHEEL;
    } else {
        *pid_left_setpoint = path[path_head].r_inches;
        *pid_right_setpoint = path[path_head].r_inches;
    }

    // Update output
    pid_left.run();
    pid_right.run();

    // Serial.print("Dest L: ");
    // Serial.println(destination_left);
    // Serial.print("Dest R: ");
    // Serial.println(destination_right);

    // // Calculate Error
    // double error_left  = destination_left -  leftEncoder.read() * IN_PER_ENCODER;
    // double error_right = destination_right - rightEncoder.read() * IN_PER_ENCODER;

    // // Deadzoning error
    // if (error_left < ERROR_DEAD_ZONE) {
    //     error_left = 0;
    // }
    // if (error_right < ERROR_DEAD_ZONE) {
    //     error_right = 0;
    // }

    // // Update Integrator
    // I_left_accumilator += KI_LEFT * error_left;
    // I_right_accumilator += KI_RIGHT * error_right;

    // if (error_left == 0) {
    //     I_left_accumilator = 0;
    // }

    // if (error_right == 0) {
    //     I_right_accumilator = 0;
    // }
    // // Serial.println(error_left);
    // // Serial.println(error_right);

    // // PI calculation 
    // left_output = KP_LEFT * error_left + I_left_accumilator;
    // right_output = KP_RIGHT * error_right + I_right_accumilator;

    // // Serial.println(left_output);
    // // Serial.println(right_output);

    // // Motor fault handling
    // // if (driveMD.getM1Fault() || driveMD.getM2Fault()) {
    // //     left_output = 0;
    // //     right_output = 0;

    // //     Serial.println("Fault");
    // // }

    // // Power limit handling
    // if (driveMD.getM1CurrentMilliamps() > MAX_MILLIAMPS || driveMD.getM2CurrentMilliamps() > MAX_MILLIAMPS) {
    //     over_power_accumilator++;
    // } else if (over_power_accumilator > 1){
    //     over_power_accumilator--;
    // }

    // left_output /= over_power_accumilator;
    // right_output /= over_power_accumilator;

    // // Serial.println(left_output);
    // // Serial.println(right_output);
    // // Serial.println();

    // // Speed limiting
    // left_output  = (left_output <= MAX_SPEED)  ? left_output  : MAX_SPEED;
    // right_output = (right_output <= MAX_SPEED) ? right_output : MAX_SPEED;

    // // Dead zoning
    // if (left_output <= SPEED_DEAD_ZONE && left_output >= (-1 * SPEED_DEAD_ZONE)) {
    //     left_output = 0;
    // }
    // if (right_output <= SPEED_DEAD_ZONE && right_output >= (-1 * SPEED_DEAD_ZONE)) {
    //     right_output = 0;
    // }

    // Set output
    if (*pid_left_output == 0 && *pid_right_output == 0) {
        driveMD.disableDrivers();
    } else {
        driveMD.enableDrivers();
        driveMD.setSpeeds(*pid_right_output, *pid_left_output);
    }

}

/**
 * @brief Calculate next path to follow if reached previous direction
 * 
 * @return true if moved on to new path 
 * @return false if still executing last path
 */
bool DriveController::CalculateNextPath() {
    if (path_head == path_tail) {

        return false;
    }

    if (*pid_left_output == 0) {
        path_head = (path_head == PATH_BUFFER_SIZE - 1) ? 0 : path_head + 1;
        leftEncoder.write(0);
        rightEncoder.write(0);
        return true;
    }

    return false;
}