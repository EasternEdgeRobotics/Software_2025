#ifndef WATERWITCH_CONSTANTS_H
#define WATERWITCH_CONSTANTS_H

#include <string>

enum CONTROL_AXES {
    SURGE = 0,
    SWAY = 1,
    HEAVE = 2,
    PITCH = 3,
    ROLL = 4,
    YAW = 5
};

// Indicates the axes that each thruster can influence, and in what direction
extern const int8_t THRUSTER_CONFIG_MATRIX[6][6];

// Names of the thrusters
extern const std::string THRUSTER_NAMES[6];

#endif // WATERWITCH_CONSTANTS_H