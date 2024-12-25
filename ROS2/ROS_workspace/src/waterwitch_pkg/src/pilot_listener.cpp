#include <memory>
#include <thread>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "eer_messages/msg/pilot_input.hpp"
#include "waterwitch_pkg/msg/waterwitch_thrust_values.hpp"
#include "waterwitch_constants.h"

class PilotListener : public rclcpp::Node
{
public:
  PilotListener() : Node("PilotListener")
  {
    // Create create a subscriber associated with this instance of the PilotListener class
    // Upon receiving a message, the call the pilot_listener_callback method of the same PilotListener class instance
    pilot_listener = this->create_subscription<eer_messages::msg::PilotInput>(
      "pilot_input", 10, std::bind(&PilotListener::pilot_listener_callback, this, std::placeholders::_1));

    thrust_publisher = this->create_publisher<waterwitch_pkg::msg::WaterwitchThrustValues>("thrust_values", 10);
  }

private:
  rclcpp::Subscription<eer_messages::msg::PilotInput>::SharedPtr pilot_listener;
  rclcpp::Publisher<waterwitch_pkg::msg::WaterwitchThrustValues>::SharedPtr thrust_publisher;

  void pilot_listener_callback(eer_messages::msg::PilotInput::UniquePtr pilot_input)
  { 

    // Lambda function to compute thrust value for each thruster
    auto compute_thrust_value = [](int thruster_index, const eer_messages::msg::PilotInput* pilot_input, waterwitch_pkg::msg::WaterwitchThrustValues& thrust_msg) {
      // Each thrust value should be clamped between -1:1
      thrust_msg.values[thruster_index] = std::max(-1.0f, std::min((pilot_input->surge * THRUSTER_CONFIG_MATRIX[thruster_index][SURGE] +
                                                                    pilot_input->sway * THRUSTER_CONFIG_MATRIX[thruster_index][SWAY] +
                                                                    pilot_input->heave * THRUSTER_CONFIG_MATRIX[thruster_index][HEAVE] +
                                                                    pilot_input->pitch * THRUSTER_CONFIG_MATRIX[thruster_index][PITCH] +
                                                                    pilot_input->roll * THRUSTER_CONFIG_MATRIX[thruster_index][ROLL] +
                                                                    pilot_input->yaw * THRUSTER_CONFIG_MATRIX[thruster_index][YAW]) / 100.0f,
                                                                    1.0f));
    };

    waterwitch_pkg::msg::WaterwitchThrustValues thrust_msg;

    std::vector<std::thread> threads;

    for (int thruster_index = 0; thruster_index < 6; thruster_index++) {
      // Compute the thrust value of each thruster in a thread to improve performance
      // Each thread will access a different thruster_index in thrust_msg
      threads.emplace_back(compute_thrust_value, thruster_index, pilot_input.get(), std::ref(thrust_msg));
    }

    for (auto& thread : threads) {
      thread.join();
    }

    thrust_publisher->publish(thrust_msg);
  }
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PilotListener>());
  rclcpp::shutdown();
  return 0;
}