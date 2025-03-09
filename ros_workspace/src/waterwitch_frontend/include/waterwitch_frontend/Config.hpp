#include <string>
#include <iostream>
#include <vector>
#include <array>
#include <unordered_map>
#include <algorithm>

using namespace std;

enum class ButtonAction { NONE, SURGE_FORWARD, SURGE_BACKWARD, SWAY_LEFT, SWAY_RIGHT, YAW_LEFT, YAW_RIGHT, HEAVE_UP, HEAVE_DOWN, BRIGHTEN_LED, DIM_LED,
                        TURN_FRONT_SERVO_CW, TURN_FRONT_SERVO_CCW, TURN_BACK_SERVO_CW, TURN_BACK_SERVO_CCW, CONFIGURATION_MODE, SIZE };
const char* buttonActionLabels[] = { "None", "Surge Forward", "Surge Backward", "Sway Left", "Sway Right", "Yaw Left", "Yaw Right", "Heave Up", "Heave Down", "Brighten LED", "Dim LED"
                        "Turn Front Servo CW", "Turn Front Servo CCW", "Turn Back Servo CW", "Turn Back Servo CCW", "Configuration Mode" };
const char* buttonActionCodes[] = { "none", "surge_forward", "surge_backward", "sway_left", "sway_right", "yaw_left", "yaw_right", "heave_up", "heave_down", "brighten_led", "dim_led" 
                        "turn_front_servo_cw", "turn_front_servo_ccw", "turn_back_servo_cw", "turn_back_servo_ccw", "configuration_mode"};
enum class AxisAction { NONE, SURGE, SWAY, YAW, HEAVE, SIZE };
const char* axisActionLabels[] = { "None", "Surge", "Sway", "Yaw", "Heave" };
const char* axisActionCodes[] = { "none", "surge", "sway", "yaw", "heave" };

ButtonAction stringToButtonAction(const string& action) {
    auto it = std::find_if(std::begin(buttonActionCodes), std::end(buttonActionCodes), [&](const char* code) { return strcmp(code, action.c_str()) == 0; });
    if (it != std::end(buttonActionCodes)) return static_cast<ButtonAction>(std::distance(std::begin(buttonActionCodes), it));
    return ButtonAction::NONE;
}

AxisAction stringToAxisAction(const string& action) {
    auto it = std::find_if(std::begin(axisActionCodes), std::end(axisActionCodes), [&](const char* code) { return strcmp(code, action.c_str()) == 0; });
    if (it != std::end(axisActionCodes)) return static_cast<AxisAction>(std::distance(std::begin(axisActionCodes), it));
    return AxisAction::NONE;
}

class UserConfig {
    public:
        char cam1ip[64];
        char cam2ip[64];
        char cam3ip[64]; 
        char name[64];
        float deadzone = 0.1;
        vector<ButtonAction> buttonActions;
        vector<AxisAction> axisActions;
};

class WaterwitchConfig {
    public:
        char servo1ip[64];
        char servo2ip[64];
        std::array<char[64], 6> thruster_map;
};