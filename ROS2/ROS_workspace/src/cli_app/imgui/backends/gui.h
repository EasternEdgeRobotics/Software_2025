#ifndef GUI_H
#define GUI_H

#include "eer_messages/msg/pilot_input.hpp"
#include "rclcpp/rclcpp.hpp"

void launchGUI(bool launch_controller = false);
void publishControllerInput(const eer_messages::msg::PilotInput& input, const rclcpp::Publisher<eer_messages::msg::PilotInput>::SharedPtr& publisher);

#endif // GUI_H