#include <string>
#include <iostream>
#include <vector>
#include <unordered_map>
#include <algorithm>

using namespace std;

enum class ButtonAction { NONE, SURGE_FORWARD, SURGE_BACKWARD, SWAY_LEFT, SWAY_RIGHT, YAW_LEFT, YAW_RIGHT, HEAVE_UP, HEAVE_DOWN, BRIGHTEN_LED, DIM_LED, SIZE };
const char* buttonActionLabels[] = { "None", "Surge Forward", "Surge Backward", "Sway Left", "Sway Right", "Yaw Left", "Yaw Right", "Heave Up", "Heave Down", "Brighten LED", "Dim LED" };
const char* buttonActionCodes[] = { "none", "surge_forward", "surge_backward", "sway_left", "sway_right", "yaw_left", "yaw_right", "heave_up", "heave_down", "brighten_led", "dim_led" };
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

class Config {
    public:
        char cam1ip[64];
        char cam2ip[64];
        char cam3ip[64];
        char name[64];
        float deadzone = 0.1;
        vector<ButtonAction> buttonActions;
        vector<AxisAction> axisActions;
};