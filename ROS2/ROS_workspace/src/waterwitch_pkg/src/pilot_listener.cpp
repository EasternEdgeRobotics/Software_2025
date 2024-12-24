#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "eer_messages/msg/pilot_input.hpp"

class PilotListener : public rclcpp::Node
{
public:
  PilotListener() : Node("PilotListener")
  {
    // Create create a subscriber associated with this instance of the PilotListener class
    // Upon receiving a message, the call the pilot_listener_callback method of the same PilotListener class instance
    pilot_listener = this->create_subscription<eer_messages::msg::PilotInput>(
      "pilot_input", 10, std::bind(&PilotListener::pilot_listener_callback, this, std::placeholders::_1));

  }

private:
  rclcpp::Subscription<eer_messages::msg::PilotInput>::SharedPtr pilot_listener;

  void pilot_listener_callback(eer_messages::msg::PilotInput::UniquePtr msg)
  {
    RCLCPP_INFO(this->get_logger(), "I heard: '%u'", msg->surge);
  }
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PilotListener>());
  rclcpp::shutdown();
  return 0;
}