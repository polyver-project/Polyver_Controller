#ifndef DRIVECONTROLLER_HPP
#define DRIVECONTROLLER_HPP
/**
 * @file DriveController.hpp
 * @author Diego Andrade
 * @brief Drive kinematic controller, smart handling of motion
 * @version 0.1
 * @date 2022-04-09
 * 
 * @copyright Copyright (c) 2022
 * 
 */
#include <Arduino.h>

#include "DualTB9051FTGMotorShield.h"
#include "Encoder.h"

// ------- Hardware Config ---------
// M2 is left motor
#define LEFT_ENCODER_PIN_A A2
#define LEFT_ENCODER_PIN_B A3

// M1 is right motor 
#define RIGHT_ENCODER_PIN_A 3
#define RIGHT_ENCODER_PIN_B 5

#define IN_CENTER_TO_WHEEL 3.0625f    // In inches

#define IN_PER_ENCODER 0.2f


// ------- Software Config ---------
#define PATH_BUFFER_SIZE 1000

#define MAX_MILLIAMPS 1000.0f

#define KP_LEFT 0
#define KD_LEFT 0

#define KP_RIGHT 0
#define KD_RIGHT 0

#define PID_DEAD_ZONE 5


/** @brief Polyver drivetrain kinematic controller with smart handling of motion */
class DriveController {
    
    public:
        DriveController();
        ~DriveController();

        /** @brief DriveController path point */
        typedef struct PolarPoint {
            int32_t r_inches;
            int32_t rads;
        } PolarPoint;

        /** @brief Adds point to path 
         * @param delta_x The delta offset to desired point
         * @param delta_y The delta offset to desired point
        */
        void Move(int8_t delta_x, int8_t delta_y);

        /** @brief Update motor output via PID controller, to be called often */
        void UpdateOutput();

        /** @brief Calculates next steps and new setpoints, can be called slower */
        void CalculateNextPath();

        // -------- DEBUG TEMPORARY ----------
        int32_t ReadLeft() {
            return leftEncoder.read();
        }
        
        int32_t ReadRight() {
            return rightEncoder.read();
        }
        
        void SetSpeed(int8_t left, int8_t right) {
            driveMD.setSpeeds(right, left);
        }

    private:
        DualTB9051FTGMotorShield driveMD; 
        Encoder leftEncoder;
        Encoder rightEncoder;
        
        // TODO: Implement circular buffer logic
        PolarPoint path[PATH_BUFFER_SIZE];              // path robot will follow
        uint32_t path_head;
        uint32_t path_tail;

        int8_t leftOutput;
        int8_t rightOutput;
};



#endif