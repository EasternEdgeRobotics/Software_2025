#ifndef WATERWITCH_CONSTANTS_H
#define WATERWITCH_CONSTANTS_H

#include <string>
#include <unordered_map>
#include <cstdint>


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

// Map from thruster name to thruster address on RP2040
extern const std::unordered_map<std::string, uint8_t> THRUSTER_MAP;

// The maximum amount that the thrusters' thrust value changes in one iteration
extern const float THRUSTER_ACCELERATION;

extern const int RP2040_ADDRESS;

// The rate at which we send control input TO the board in milliseconds
extern const int SOFTWARE_TO_BOARD_COMMUNICATION_RATE;

#endif // WATERWITCH_CONSTANTS_H