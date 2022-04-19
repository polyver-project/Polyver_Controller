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

DriveController::DriveController() 
:   leftEncoder{LEFT_ENCODER_PIN_A, LEFT_ENCODER_PIN_B}, 
    rightEncoder{RIGHT_ENCODER_PIN_A, RIGHT_ENCODER_PIN_B},
    path_head{0}, path_tail{0},
    leftOutput{0}, rightOutput{0}
{
    // Motor setup
    driveMD.init();
    driveMD.flipM1(true);                   // Right wheel inverted
    driveMD.flipM2(false);
    delay(1);

    // Should this be done here? or in CalculateNextPath()
    driveMD.enableDrivers();
}

DriveController::~DriveController() {
    // Clean up code
    driveMD.disableDrivers();
}

void DriveController::Move(int8_t delta_x, int8_t delta_y) {
    // TODO: Should this return some success code?
    // Buffer full condtions
    if ((path_tail == PATH_BUFFER_SIZE - 1) && (path_head == 0)) {
        return;
    } else if (path_tail + 1 == path_head) {
        return;
    }

    int32_t r_inches = sqrt(delta_x*delta_x + delta_y*delta_y);
    int32_t rads = (delta_x != 0) ? atan2(delta_y, delta_x) : 0;

    // Update previous point if connituing in same direction
    if (rads == 0) {
        path[path_tail].r_inches += r_inches;
        return;
    }

    // Adding new path
    path_tail = (path_tail == PATH_BUFFER_SIZE - 1) ? 0 : path_tail + 1;
    path[path_tail].r_inches = r_inches;
    path[path_tail].rads = rads;
}

void DriveController::UpdateOutput() {
    // Calculate destination
    int32_t destination_left = (path[path_head].rads != 0) ?
        -1 * path[path_head].rads * IN_CENTER_TO_WHEEL :        // Distance when turning, note -1 crrection 
        path[path_head].r_inches;                               // Straight
    int32_t destination_right = (path[path_head].rads != 0) ?
        path[path_head].rads * IN_CENTER_TO_WHEEL : 
        path[path_head].r_inches;

    // Calculate Error
    int32_t error_left = destination_left -  leftEncoder.read() * IN_PER_ENCODER;
    int32_t error_right = destination_right - rightEncoder.read() * IN_PER_ENCODER;

    // P calculation 
    leftOutput = KP_LEFT * error_left;
    rightOutput = KP_RIGHT * error_right;

    // Motor fault handling
    if (driveMD.getM1Fault() || driveMD.getM2Fault()) {
        leftOutput = 0;
        rightOutput = 0;
    }

    // Power limit handling
    leftOutput *= (MAX_MILLIAMPS - driveMD.getM1CurrentMilliamps()) / MAX_MILLIAMPS;
    rightOutput *= (MAX_MILLIAMPS - driveMD.getM2CurrentMilliamps()) / MAX_MILLIAMPS;

    // Set output
    driveMD.setSpeeds(rightOutput, leftOutput);
}

void DriveController::CalculateNextPath() {
    if (path_head == path_tail)
        return;

    if (leftOutput < PID_DEAD_ZONE && rightOutput < PID_DEAD_ZONE) {
        // Move head
        path_head = (path_head == PATH_BUFFER_SIZE - 1) ? 0 : path_head + 1;

        // Reset encoder count
        leftEncoder.write(0);
        rightEncoder.write(0);
        return;
    }
}