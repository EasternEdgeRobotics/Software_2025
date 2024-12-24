#include "waterwitch_constants.h"

const int8_t THRUSTER_CONFIG_MATRIX[6][6] = {
    {1, 0, 0, 0, 0, 1},
    {1, 0, 0, 0, 0, -1},
    {0, 1, 0, 0, 1, 0},
    {0, 1, 0, 0, -1, 0},
    {0, 0, 1, 1, 0, 0},
    {0, 0, 1, -1, 0, 0}
};

const std::string THRUSTER_NAMES[6] = {
    "for_star",
    "for_port",
    "aft_star",
    "aft_port",
    "star_top",
    "port_top"
};