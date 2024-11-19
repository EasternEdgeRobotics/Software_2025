#ifndef CONTROLLER_H
#define CONTROLLER_H

#include <rclcpp/rclcpp.hpp>
#include <eer_messages/msg/pilot_input.hpp>

void publishControllerInput(const eer_messages::msg::PilotInput &input, std::shared_ptr<rclcpp::Publisher<eer_messages::msg::PilotInput>> publisher);

#endif // CONTROLLER_H