#include <memory>
#include <thread>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"
#include "eer_interfaces/msg/waterwitch_thrust_values.hpp"
#include "waterwitch_constants.h"


class SimulationBotControl : public rclcpp::Node
{
public:
  SimulationBotControl() : Node("SimulationBotControl")
  {
    
    for (int thruster_index = 0; thruster_index < 6; thruster_index++) {
      simulation_thruster_publishers[thruster_index] = this->create_publisher<std_msgs::msg::Int32>("/waterwitch/" + THRUSTER_NAMES[thruster_index], 10);
    }

    auto thrust_values_subscriber_callback =
      [this](eer_interfaces::msg::WaterwitchThrustValues::UniquePtr thrust_values_msg) -> void {
        for (int thruster_index = 0; thruster_index < 6; thruster_index++) {

            std_msgs::msg::Int32 simulation_thrust_msg;

            // Map the thrust value from the range [-1, 1] to [0, 255]
            simulation_thrust_msg.data = static_cast<int32_t>((thrust_values_msg->values[thruster_index] + 1) * 127.5);

            // Gazebo doesn't accept 0 as a value, so set it to 1 in that case
            if (simulation_thrust_msg.data == 0) simulation_thrust_msg.data = 1;

            simulation_thruster_publishers[thruster_index]->publish(simulation_thrust_msg);
        }
      };

    thrust_values_subscriber =
      this->create_subscription<eer_interfaces::msg::WaterwitchThrustValues>(
        "thrust_values", 10, thrust_values_subscriber_callback);
  }

private:
  rclcpp::Subscription<eer_interfaces::msg::WaterwitchThrustValues>::SharedPtr thrust_values_subscriber;
  std::array<rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr, 6> simulation_thruster_publishers;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SimulationBotControl>());
  rclcpp::shutdown();
  return 0;
}