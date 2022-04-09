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

#define PATH_BUFFER_SIZE 1000

class DriveController {

private:
    DualTB9051FTGMotorShield driveMD;       
    
    uint8_t path[PATH_BUFFER_SIZE];         // path robot will follow

public:
    DriveController();

    int8_t Start();

    /** @brief Adds point to path */
    void AddPoint(uint8_t point);

};



#endif