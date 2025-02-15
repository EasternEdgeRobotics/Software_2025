#include "waterwitch_constants.h"

// TERMINOLOGY
// for star -> forward right
// for port -> forward left
// aft star -> backward right
// aft port -> backward left
// star top -> upward right
// port top -> upward left


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

const std::unordered_map<std::string, uint8_t> THRUSTER_MAP = {
    {"for_star", 0},
    {"for_port", 1},
    {"aft_star", 2},
    {"aft_port", 3},
    {"star_top", 4},
    {"port_top", 5}
};


const float THRUSTER_ACCELERATION = 0.1f;

const int RP2040_ADDRESS = 0x69;

const int SOFTWARE_TO_BOARD_COMMUNICATION_RATE = 100;