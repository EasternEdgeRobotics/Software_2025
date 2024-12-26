#ifndef GUI_H
#define GUI_H

#include "eer_interfaces/msg/pilot_input.hpp"
#include "rclcpp/rclcpp.hpp"

void launchGUI(const rclcpp::Publisher<eer_interfaces::msg::PilotInput>::SharedPtr& publisher, bool launch_controller = false);
void publishControllerInput(const eer_interfaces::msg::PilotInput& input, const rclcpp::Publisher<eer_interfaces::msg::PilotInput>::SharedPtr& publisher);

#endif // GUI_H