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
#include <DualTB9051FTGMotorShield.h>
#include <Encoder.h>
#include <AutoPID.h>

// ------- Hardware Config ---------
// M2 is left motor
#define DRIVE_LEFT_PIN_A 18
#define DRIVE_LEFT_PIN_B 19

// M1 is right motor 
#define DRIVE_RIGHT_PIN_A 20
#define DRIVE_RIGHT_PIN_B 21

#define DRIVE_IN_CENTER_TO_WHEEL 3.0625    // In inches

#define DRIVE_IN_PER_ENCODER 0.0064f


// ------- Software Config ---------
#define PATH_BUFFER_SIZE 10

#define MAX_MILLIAMPS 1000.0f
#define MAX_SPEED     300
#define MIN_SPEED     0

#define KP_LEFT 1.0
#define KI_LEFT 1.0
#define KD_LEFT 0.0

#define KP_RIGHT 1.0
#define KI_RIGHT 1.0
#define KD_RIGHT 0.0

#define SPEED_DEAD_ZONE 50              // Cutoff at distance: kP / SPEED_DEAD_ZONE = 1 in
#define ERROR_DEAD_ZONE 0.1             // Inches

// ---- Typedefs ---------------------------
/** @brief DriveController path point */
typedef struct PolarPoint {
    double r_inches;
    double rads;
} PolarPoint;

/** @brief Polyver drivetrain kinematic controller with smart handling of motion */
class DriveController {
    public:
        // DriveController();
        // ~DriveController();

        static void Init();

        /** @brief Adds point to path. Note: Robot faces Y-axis
         * @param delta_x The delta offset to desired point
         * @param delta_y The delta offset to desired point
        */
        static void Move(int8_t delta_x, int8_t delta_y);

        /** @brief Update motor output via PID controller, to be called often */
        static void UpdateOutput();

        /** @brief Calculates next steps and new setpoints, can be called slower */
        static bool CalculateNextPath();

        /** @brief Disable motors and place into standby mode */
        static void Kill();

    // private:
        // ---- PID ------------------------
        static double* pid_left_input;
        static double* pid_left_setpoint;
        static double* pid_left_output;

        static double* pid_right_input;
        static double* pid_right_setpoint;
        static double* pid_right_output;

        // static int16_t right_output;

        // static uint16_t over_power_accumilator;
        // static float I_left_accumilator;
        // static float I_right_accumilator;

        static AutoPID pid_left;
        static AutoPID pid_right;
        
        // ---- Path controll data ---------
        static PolarPoint* path;
        static uint32_t path_head;     // Points to current executing position
        static uint32_t path_tail;     // Points to last valid position

        // ---- Polulu motor driver --------
        static DualTB9051FTGMotorShield driveMD; 

        // ---- Encoders -------------------
        static Encoder leftEncoder;
        static Encoder rightEncoder;
};

#endif