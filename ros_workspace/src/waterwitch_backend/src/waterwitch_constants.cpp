#include "waterwitch_constants.h"

const int8_t THRUSTER_CONFIG_MATRIX[6][6] = {
    {-1, 1, 0, 0, 0, 1}, // for star
    {-1, -1, 0, 0, 0, -1}, // for port
    {1, 1, 0, 0, 0, -1}, // aft star
    {1, -1, 0, 0, 0, 1}, // aft port
    {0, 0, -1, 1, 0, 0}, // star top
    {0, 0, -1, -1, 0, 0} // port top
};

const std::string THRUSTER_NAMES[6] = {
    "for_star",
    "for_port",
    "aft_star",
    "aft_port",
    "star_top",
    "port_top"
};

const float THRUSTER_ACCELERATION = 0.1f;

const int SOFTWARE_TO_BOARD_COMMUNICATION_RATE = 100;