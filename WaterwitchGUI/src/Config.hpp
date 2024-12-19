#include <string>
#include <iostream>

using namespace std;

enum class ButtonAction { NONE, SURGE_FORWARD, SURGE_BACKWARD, SWAY_LEFT, SWAY_RIGHT, TURN_LEFT, TURN_RIGHT, HEAVE_UP, HEAVE_DOWN, SIZE };
const char* buttonActionLabels[] = { "None", "Surge Forward", "Surge Backward", "Sway Left", "Sway Right", "Turn Left", "Turn Right", "Heave Up", "Heave Down" };
enum class AxisAction { NONE, SURGE, SWAY, TURN, HEAVE, SIZE };
const char* axisActionLabels[] = { "None", "Surge", "Sway", "Turn", "Heave" };

class Config {
    public:
        char cam1ip[64];
        char cam2ip[64];
        char cam3ip[64];
        char name[64];
        string controller;
        float deadzone = 0.1;
        vector<ButtonAction> buttonActions;
        vector<AxisAction> axisActions;
};