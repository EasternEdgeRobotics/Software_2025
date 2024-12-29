#include <memory>
#include <thread>
#include <vector>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "eer_interfaces/msg/pilot_input.hpp"
#include "eer_interfaces/msg/waterwitch_thrust_values.hpp"
#include "waterwitch_constants.h"

class PilotListener : public rclcpp::Node
{
public:
  PilotListener() : Node("PilotListener")
  {

    // Ensure that the current_thrust values and target_thrust_values are initialized to 0
    current_thrust.values.fill(0.0f);
    target_thrust_values.fill(0.0f);

    // Create a subscriber on the pilot input topic. The pilot_listener_callback method will be called whenever a new message is received.
    pilot_listener = this->create_subscription<eer_interfaces::msg::PilotInput>(
      "pilot_input", 10, std::bind(&PilotListener::pilot_listener_callback, this, std::placeholders::_1));

    // Create a publisher on the thrust_values topic
    thrust_publisher = this->create_publisher<eer_interfaces::msg::WaterwitchThrustValues>("thrust_values", 10);

    // Create a timer that will call the software_to_board_communication_timer_callback method at a fixed rate
    software_to_board_communication_timer = this->create_wall_timer(std::chrono::milliseconds(SOFTWARE_TO_BOARD_COMMUNICATION_RATE), 
      std::bind(&PilotListener::software_to_board_communication_timer_callback, this));
  }

private:
  rclcpp::Subscription<eer_interfaces::msg::PilotInput>::SharedPtr pilot_listener;
  rclcpp::Publisher<eer_interfaces::msg::WaterwitchThrustValues>::SharedPtr thrust_publisher;
  rclcpp::TimerBase::SharedPtr software_to_board_communication_timer;

  std::array<std::mutex, 6> target_thrust_mutexes;

  std::array<float, 6> target_thrust_values;
  eer_interfaces::msg::WaterwitchThrustValues current_thrust;

  void pilot_listener_callback(eer_interfaces::msg::PilotInput::UniquePtr pilot_input)
  { 

    // Lambda function to compute thrust value for each thruster
    auto compute_thrust_value = [this](int thruster_index, const eer_interfaces::msg::PilotInput* pilot_input, std::array<float, 6>& target_thrust_values) 
    {
      // Ensure that the software_to_board_communication_timer_callback method does not access the target_thrust_values while they are being updated
      std::lock_guard<std::mutex> lock(target_thrust_mutexes[thruster_index]);

      // Each thrust value should be clamped between -1:1
      target_thrust_values[thruster_index] = std::max(-1.0f, std::min((pilot_input->surge * THRUSTER_CONFIG_MATRIX[thruster_index][SURGE] +
                                                                    pilot_input->sway * THRUSTER_CONFIG_MATRIX[thruster_index][SWAY] +
                                                                    pilot_input->heave * THRUSTER_CONFIG_MATRIX[thruster_index][HEAVE] +
                                                                    pilot_input->pitch * THRUSTER_CONFIG_MATRIX[thruster_index][PITCH] +
                                                                    pilot_input->roll * THRUSTER_CONFIG_MATRIX[thruster_index][ROLL] +
                                                                    pilot_input->yaw * THRUSTER_CONFIG_MATRIX[thruster_index][YAW]) / 100.0f,
                                                                    1.0f));
    };

    std::vector<std::thread> threads;

    for (int thruster_index = 0; thruster_index < 6; thruster_index++) 
    {
      // Compute the thrust value of each thruster in a thread to improve performance
      threads.emplace_back(compute_thrust_value, thruster_index, pilot_input.get(), std::ref(target_thrust_values));
    }

    for (auto& thread : threads) 
    {
      // Ensure all threads have finished executing before continuing
      thread.join();
    }
  }

  void software_to_board_communication_timer_callback()
  {
    for (int thruster_index = 0; thruster_index < 6; thruster_index++) {

      // Ensure that the pilot_listener_callback method does not update the target_thrust_values while they are being accessed
      std::lock_guard<std::mutex> lock(target_thrust_mutexes[thruster_index]);

      // Only update the thrust value of each thruster if the target value is different
      if (target_thrust_values[thruster_index] != current_thrust.values[thruster_index]) 
      {
        float difference = target_thrust_values[thruster_index] - current_thrust.values[thruster_index];

        if (abs(difference) > THRUSTER_ACCELERATION) 
        { 
          current_thrust.values[thruster_index] += std::copysign(difference * THRUSTER_ACCELERATION, difference);
        } 
        else
        {
          current_thrust.values[thruster_index] = target_thrust_values[thruster_index];
        }
      }
    }

    // Publish the current thrust values. These can be interperted by either the hardware (RP2040 on the board) or the simulator
    thrust_publisher->publish(current_thrust);
  }
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  auto pilot_listener_node = std::make_shared<PilotListener>();
  
  // Use a multi-threaded executor to improve performance since pilot_listener is thread-safe
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(pilot_listener_node);
  executor.spin();

  rclcpp::shutdown();
  return 0;
}