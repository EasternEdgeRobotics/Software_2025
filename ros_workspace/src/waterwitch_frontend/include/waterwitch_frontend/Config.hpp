#include <string>
#include <iostream>
#include <vector>
#include <array>
#include <unordered_map>
#include <algorithm>

using namespace std;

enum class ButtonAction { NONE, SURGE_FORWARD, SURGE_BACKWARD, SWAY_LEFT, SWAY_RIGHT, YAW_LEFT, YAW_RIGHT, HEAVE_UP, HEAVE_DOWN, BRIGHTEN_LED, DIM_LED,
                        TURN_FRONT_SERVO_CW, TURN_FRONT_SERVO_CCW, TURN_BACK_SERVO_CW, TURN_BACK_SERVO_CCW, CONFIGURATION_MODE, FLIP_CAMERA_1, FLIP_CAMERA_2, FLIP_CAMERA_3,
                        FRONT_CAMERA_SERVO_ANGLE_1, FRONT_CAMERA_SERVO_ANGLE_2, FRONT_CAMERA_SERVO_ANGLE_3, BACK_CAMERA_SERVO_ANGLE_1, BACK_CAMERA_SERVO_ANGLE_2,
                        BACK_CAMERA_SERVO_ANGLE_3, ROLL_CW, ROLL_CCW, SIZE };
const char* buttonActionLabels[] = { "None", "Surge Forward", "Surge Backward", "Sway Left", "Sway Right", "Yaw Left", "Yaw Right", "Heave Up", "Heave Down", "Brighten LED", "Dim LED",
    "Turn Front Servo CW", "Turn Front Servo CCW", "Turn Back Servo CW", "Turn Back Servo CCW", "Configuration Mode", "Flip Camera 1", "Flip Camera 2", "Flip Camera 3",
    "Front Camera Servo Angle 1", "Front Camera Servo Angle 2", "Front Camera Servo Angle 3", "Back Camera Servo Angle 1", "Back Camera Servo Angle 2", "Back Camera Servo Angle 3",
    "Roll CW", "Roll CCW"};
const char* buttonActionCodes[] = { "none", "surge_forward", "surge_backward", "sway_left", "sway_right", "yaw_left", "yaw_right", "heave_up", "heave_down", "brighten_led", "dim_led",
    "turn_front_servo_cw", "turn_front_servo_ccw", "turn_back_servo_cw", "turn_back_servo_ccw", "configuration_mode", "flip_camera_1", "flip_camera_2", "flip_camera_3",
    "front_camera_servo_angle_1", "front_camera_servo_angle_2", "front_camera_servo_angle_3", "back_camera_servo_angle_1", "back_camera_servo_angle_2", "back_camera_servo_angle_3",
    "roll_cw", "roll_ccw"};

enum class AxisAction { NONE, SURGE, SWAY, YAW, ROLL, HEAVE, SIZE };
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
        char servo1SSHTarget[64];
        char servo2SSHTarget[64];
        std::array<char[64], 6> thruster_map = {"0", "1", "2", "3", "4", "5"};
        std::array<bool, 6> reverse_thrusters = {false, false, false, false, false};
        std::array<bool, 6> stronger_side_positive = {false, false, false, false, false};
        float thruster_stronger_side_attenuation_constant = 1.0f;
        float thruster_acceleration = 1.0f;
        std::array<int, 3> front_camera_preset_servo_angles = {0, 135, 270};
        std::array<int, 3> back_camera_preset_servo_angles = {0, 135, 270};
};