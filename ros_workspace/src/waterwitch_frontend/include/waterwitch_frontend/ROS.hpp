#ifndef ROS_HPP
#define ROS_HPP

#include "rclcpp/rclcpp.hpp"
#include "eer_interfaces/srv/list_config.hpp"
#include "eer_interfaces/msg/save_config.hpp"
#include "eer_interfaces/msg/pilot_input.hpp"
#include "eer_interfaces/msg/thruster_multipliers.hpp"
#include "Power.hpp"

class SaveConfigPublisher : public rclcpp::Node {
    public:
        SaveConfigPublisher() : Node("save_config_publisher") {
            publisher_ = this->create_publisher<eer_interfaces::msg::SaveConfig>("/save_config", 10);
        }

        void saveConfig(const std::string & name, const std::string & data) {
            auto msg = eer_interfaces::msg::SaveConfig();
            msg.name = name;
            msg.data = data;
            publisher_->publish(msg);
        }

    private:
        rclcpp::Publisher<eer_interfaces::msg::SaveConfig>::SharedPtr publisher_;
};

class PilotInputPublisher : public rclcpp::Node {
    public:
        PilotInputPublisher() : Node("pilot_input_publisher") {
            publisher_ = this->create_publisher<eer_interfaces::msg::PilotInput>("/pilot_input", 10);
        }

        void sendInput(const int& surge,
        const int& sway,
        const int& heave,
        const int& yaw,
        const bool& brightenLED,
        const bool& dimLED) {
            auto msg = eer_interfaces::msg::PilotInput();
            msg.surge = surge;
            msg.sway = sway;
            msg.heave = heave;
            msg.yaw = yaw;
            //may need to remove these? not sure if leds are on the bot this year
            msg.brighten_led = brightenLED;
            msg.dim_led = dimLED;
            publisher_->publish(msg);
            // ########################
            // Add more inputs to this function
            // ########################
        }

    private:
        rclcpp::Publisher<eer_interfaces::msg::PilotInput>::SharedPtr publisher_;
};

class ThrusterMultipliersPublisher : public rclcpp::Node {
    public:
        ThrusterMultipliersPublisher(Power* powerPtr) : Node("thruster_multipliers_publisher") {
            publisher_ = this->create_publisher<eer_interfaces::msg::ThrusterMultipliers>("/thruster_multipliers", 10);
            this->powerPtr = powerPtr;
        }

        void sendPower() {
            Power power = *powerPtr;

            //surge sway heave itch roll yaw
            auto msg = eer_interfaces::msg::ThrusterMultipliers();
            msg.power = power.power;
            msg.surge = power.surge;
            msg.sway = power.sway;
            msg.heave = power.heave;
            msg.pitch = 0; //pitch is not possible with our thruster configuration
            //msg.roll = 0; //roll is possible, but not used? can be discussed later
            msg.yaw = power.yaw;
            publisher_->publish(msg);
        }

    private:
        rclcpp::Publisher<eer_interfaces::msg::ThrusterMultipliers>::SharedPtr publisher_;
  
        Power* powerPtr;
};

std::array<std::vector<std::string>, 2> getConfigs() {
    auto node = rclcpp::Node::make_shared("list_configs_client");
    auto client = node->create_client<eer_interfaces::srv::ListConfig>("list_configs");
    auto request = std::make_shared<eer_interfaces::srv::ListConfig::Request>();
  
    while (!client->wait_for_service(1s)) {
        if (!rclcpp::ok()) {
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
            return std::array<std::vector<std::string>, 2>();
        }
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service not available, waiting again...");
    }

    std::array<std::vector<std::string>, 2> output;
  
    auto result = client->async_send_request(request);
    if (rclcpp::spin_until_future_complete(node, result) == rclcpp::FutureReturnCode::SUCCESS) {
        auto response = result.get();
        output[0] = response->names;
        output[1] = response->configs;
        for (const auto & config : response->configs) {
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Config: %s", config.c_str());
        }
    } else {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call service list_configs");
    }

    return output;
}

#endif