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

// ---- Hardware Config --------------------
// M1 is left motor
#define DRIVE_LEFT_PIN_A 18
#define DRIVE_LEFT_PIN_B 19

// M2 is right motor 
#define DRIVE_RIGHT_PIN_A 20
#define DRIVE_RIGHT_PIN_B 21

// #define DRIVE_IN_CENTER_TO_WHEEL 3.0625    // In inches
#define DRIVE_IN_CENTER_TO_WHEEL 6.125    // In inches

#define DRIVE_TICKS_PER_IN 156.25


// ---- Software Config --------------------
#define DRIVE_MANUAL_TIMEOUT_MS 1000

#define DRIVE_PATH_BUFFER_SIZE 10

#define DRIVE_MAX_MILLIAMPS 4000.0f
#define DRIVE_MAX_SPEED     150
#define DRIVE_MIN_SPEED     -150

#define DRIVE_LEFT_KP 0.2
#define DRIVE_LEFT_KI 0.0
#define DRIVE_LEFT_KD -23000.0

#define DRIVE_RIGHT_KP 0.2
#define DRIVE_RIGHT_KI 0.0
#define DRIVE_RIGHT_KD -23000.0

#define DRIVE_SPEED_DEAD_ZONE 50
#define DRIVE_ERROR_DEAD_ZONE 0.1                           // Inches
#define DRIVE_SETPOINT_DEAD_ZONE (3 * DRIVE_TICKS_PER_IN)   // Inches

// ---- Drive Modes ------------------------
#define DRIVE_MODE_MANUAL 0
#define DRIVE_MODE_PATH 1

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

        static void SetMode(uint8_t mode);

        // ---- Manual Control Mode --------
        static void ManualMove(int left_speed, int right_speed);
        static void ManualLeftMove(int speed);
        static void ManualRightMove(int speed);

        // ---- PID Path Control Mode ------
        /** @brief Adds point to path. Note: Robot faces Y-axis
         * @param delta_x The delta offset to desired point
         * @param delta_y The delta offset to desired point
        */
        static void Move(int8_t delta_x, int8_t delta_y);

        // ---- Standard Operation ---------
        static void IrregularUpdate(unsigned long lose_period_ms = 3000);
        static void StrictUpdate();

        /** @brief Disable motors and place into standby mode */
        static void Kill();

    // private:
        static unsigned long last_irregular_update_ms;

        // ---- Drive Mode -----------------
        static uint8_t drive_mode;
        static unsigned long last_manual_speed_update_ms;

        // ---- PID ------------------------
        static double* pid_left_input;
        static double* pid_left_setpoint;
        static double* pid_left_output;

        static double* pid_right_input;
        static double* pid_right_setpoint;
        static double* pid_right_output;

        static AutoPID* pid_left;
        static AutoPID* pid_right;

        // ---- Path controll data ---------
        static PolarPoint* path;
        static uint32_t path_head;     // Points to current executing position
        static uint32_t path_tail;     // Points to last valid position

        // --- Safety ----------------------
        static uint16_t over_power_accumilator;
        static bool left_disabled;
        static bool right_disabled;

        // ---- Polulu motor driver --------
        static DualTB9051FTGMotorShield driveMD; 
        static int left_output;
        static int right_output;

        // ---- Encoders -------------------
        static Encoder leftEncoder;
        static Encoder rightEncoder;

        static void SetLeft(int speed);
        static void SetRight(int speed);

        /** @brief Calculates next steps and new setpoints, can be called slower */
        static bool CalculateNextPath();
};

#endif