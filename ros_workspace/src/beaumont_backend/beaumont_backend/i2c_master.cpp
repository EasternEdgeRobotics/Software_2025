#include <rclcpp/rclcpp.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/action.hpp>
#include <rclcpp/callback_group.hpp>
#include <rclcpp/executors.hpp>

#include <std_msgs/msg/string.hpp>
#include <eer_interfaces/msg/thruster_multipliers.hpp>
#include <eer_interfaces/msg/pilot_input.hpp>
#include <eer_interfaces/action/beaumont_auto_mode.hpp>
#include <beaumont_backend/srv/hsv_colours.hpp>

#include <cmath>
#include <map>
#include <string>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>
#include <unistd.h>
#include <cerrno>
#include <cstring>
#include <iostream>
#include <vector>

class Thruster {
public:
    Thruster(int bus, const std::string& thruster_position) 
        : bus_(bus), thruster_position_(thruster_position), thruster_armed_(false), target_(127), current_(127) {
        arm_thruster();
    }

    void arm_thruster() {
        try {
            if (ioctl(bus_, I2C_SLAVE, RP2040_ADDRESS) < 0) {
                throw std::runtime_error("Failed to acquire bus access and/or talk to slave");
            }
            if (i2c_smbus_write_byte_data(bus_, THRUSTER_CHANNELS[thruster_position_], 127) < 0) {
                throw std::runtime_error("Failed to write to I2C device");
            }
            thruster_armed_ = true;
        } catch (const std::exception& e) {
            std::cerr << e.what() << std::endl;
        }
    }

    void fly(double speed) {
        target_ = speed * 127 + 127;
    }

    void tick() {
        if (!thruster_armed_) {
            arm_thruster();
            return;
        }

        if (current_ != target_) {
            if (std::abs(target_ - current_) > THRUSTER_ACCELERATION) {
                double direction = (target_ - current_) / std::abs(target_ - current_);
                current_ += static_cast<int>(direction * THRUSTER_ACCELERATION);

                if (current_ > 254) current_ = 254;
                if (current_ < 0) current_ = 0;

                try {
                    if (i2c_smbus_write_byte_data(bus_, THRUSTER_CHANNELS[thruster_position_], current_) < 0) {
                        throw std::runtime_error("Failed to write to I2C device");
                    }
                } catch (const std::exception& e) {
                    thruster_armed_ = false;
                    std::cerr << e.what() << std::endl;
                }
            } else {
                current_ = target_;
                try {
                    if (i2c_smbus_write_byte_data(bus_, THRUSTER_CHANNELS[thruster_position_], current_) < 0) {
                        throw std::runtime_error("Failed to write to I2C device");
                    }
                } catch (const std::exception& e) {
                    thruster_armed_ = false;
                    std::cerr << e.what() << std::endl;
                }
            }
        }
    }

    bool is_thruster_armed() const {
        return thruster_armed_;
    }

private:
    int bus_;
    std::string thruster_position_;
    bool thruster_armed_;
    int target_;
    int current_;
};

class I2CMaster : public rclcpp::Node {
public:
    I2CMaster()
        : Node("i2c_master"), autonomous_mode_active_(false), debugger_mode_(false), thruster_calibration_mode_(false), headlight_led_brightness_(50) {
        
        // Subscribers
        copilot_listener_ = this->create_subscription<eer_interfaces::msg::ThrusterMultipliers>(
            "thruster_multipliers", 100, std::bind(&I2CMaster::copilot_listener_callback, this, std::placeholders::_1));
        pilot_listener_ = this->create_subscription<eer_interfaces::msg::PilotInput>(
            "pilot_input", 1, std::bind(&I2CMaster::pilot_listener_callback, this, std::placeholders::_1));

        // Publishers
        diagnostics_data_publisher_1_ = this->create_publisher<std_msgs::msg::String>("diagnostics_data_1", 10);
        diagnostics_data_publisher_2_ = this->create_publisher<std_msgs::msg::String>("diagnostics_data_2", 10);

        // Autonomous movement node
        action_client_ = rclcpp_action::create_client<eer_interfaces::action::BeaumontAutoMode>(
            this, "autonomus_brain_coral_transplant");

        // Client to fetch the hsv colour values camera saved on the task_manager database
        brain_coral_hsv_colour_bounds_client_ = this->create_client<beaumont_backend::srv::HSVColours>(
            "set_color", rclcpp::callback_group::CallbackGroupType::Reentrant);

        // The default bounds filter for the colour Black
        brain_coral_hsv_colour_bounds_["upper_hsv"] = {10, 10, 40};
        brain_coral_hsv_colour_bounds_["lower_hsv"] = {0, 0, 0};

        // Thruster calibration mode
        if (thruster_calibration_mode_) {
            debugger_mode_ = true;
            current_thruster_ = 0;
        }

        // Debugger
        if (debugger_mode_) {
            debugger_ = this->create_publisher<std_msgs::msg::String>("debugger", 10);
        }

        // Initialize I2C
        i2c_ = open("/dev/i2c-1", O_RDWR);
        if (i2c_ < 0) {
            RCLCPP_ERROR(this->get_logger(), "No Hardware on I2C bus");
        }

        // Initialize IMU
        imu_detected_ = false;
        if (i2c_ >= 0) {
            // Connect to BNO055
            try {
                // Assuming BNO055_I2C is a class that handles the IMU sensor
                imu_sensor_ = std::make_shared<BNO055_I2C>(i2c_);
                last_temperature_val_ = 0xFFFF; // per recommendation on (https://learn.adafruit.com/adafruit-bno055-absolute-orientation-sensor/python-circuitpython)
                imu_detected_ = true;
                RCLCPP_INFO(this->get_logger(), "BNO055 DETECTED ON I2C BUS");
            } catch (...) {
                RCLCPP_ERROR(this->get_logger(), "CANNOT FIND BNO055 ON I2C BUS!");
            }
        }

        // Initialize SMBus
        bus_ = open("/dev/i2c-1", O_RDWR);
        if (bus_ < 0) {
            RCLCPP_ERROR(this->get_logger(), "COULD NOT INITIALIZE SMBUS");
        } else {
            RCLCPP_INFO(this->get_logger(), "INITIALIZED SMBUS!");
        }

        // Initialize Thrusters
        power_multiplier_ = 0.2;
        surge_multiplier_ = 0;
        sway_multiplier_ = 0;
        heave_multiplier_ = 0;
        pitch_multiplier_ = 0;
        yaw_multiplier_ = 0;

        if (bus_ >= 0) {
            for (const auto& thruster_position : THRUSTER_CHANNELS) {
                connected_channels_[THRUSTER_CHANNELS[thruster_position.first]] = std::make_shared<Thruster>(bus_, thruster_position.first);
            }
        }

        // Initialize ADCs
        if (bus_ >= 0) {
            for (const auto& key : ADC_ADDRESSES) {
                configured_adcs_[key.first] = false;
            }
        }
    }

private:
    void copilot_listener_callback(const eer_interfaces::msg::ThrusterMultipliers::SharedPtr msg) {
        power_multiplier_ = static_cast<float>(msg->power) / 100;
        surge_multiplier_ = static_cast<float>(msg->surge) / 100;
        sway_multiplier_ = static_cast<float>(msg->sway) / 100;
        heave_multiplier_ = static_cast<float>(msg->heave) / 100;
        pitch_multiplier_ = static_cast<float>(msg->pitch) / 100;
        yaw_multiplier_ = static_cast<float>(msg->yaw) / 100;
    }

    void tick_thrusters() {
        for (auto& channel : connected_channels_) {
            channel.second->tick();
        }
    }

    std::shared_ptr<std_msgs::msg::String> obtain_temp_sensor_data(std::shared_ptr<std_msgs::msg::String> diagnostics_data) {
        uint8_t temperature_reading_register = 0b00000000;
        uint8_t temperature_reading_register_length_in_bytes = 2;

        for (const auto& [key, address] : TEMPERATURE_SENSOR_ADDRESSES) {
            try {
                uint8_t read[2];
                if (read_i2c_block_data(address, temperature_reading_register, temperature_reading_register_length_in_bytes, read) < 0) {
                    throw std::runtime_error("Failed to read I2C block data");
                }

                // Convert to 11 bits
                int temp_read = ((read[0] << 8) + read[1]) >> 5;

                // According to the datasheet (https://www.nxp.com/docs/en/data-sheet/LM75B.pdf), the reading is in two's complement form.
                // The line below obtains magnitude and assigns correct sign
                temp_read = (temp_read >= 0b10000000000) ? -((0b11111111111 - temp_read) + 1) : temp_read;

                // Data sheet says to use 0.125 for conversion to Celsius
                diagnostics_data->data += key + ": " + std::to_string(round(static_cast<float>(temp_read) * 0.125 * 10000) / 10000) + "C\n";
            } catch (const std::exception& e) {
                diagnostics_data->data += key + ": 0C\n";
            }
        }

        return diagnostics_data;
    }

    void pilot_listener_callback(const eer_interfaces::msg::PilotInput::SharedPtr msg) {
        if (!autonomous_mode_active_ || autonomous_mode_active_) {
            auto thruster_values = rov_math(msg);

            if (bus_ >= 0) {
                for (const auto& thruster_position : THRUSTER_CHANNELS) {
                    connected_channels_[THRUSTER_CHANNELS[thruster_position.first]]->fly(thruster_values[thruster_position.first]);

                    // Thrusters should be armed by the time the first pilot input is received
                    if (!connected_channels_[THRUSTER_CHANNELS[thruster_position.first]]->is_thruster_armed()) {
                        RCLCPP_ERROR(this->get_logger(), "Thruster %s not armed", thruster_position.first.c_str());
                    }
                }

                tick_thrusters();
                stm32_communications(msg);
            }
        }

        if (msg->enter_auto_mode) {
            if (!autonomous_mode_active_) {
                autonomous_mode_active_ = true;
                send_autonomous_mode_goal();
            } else {
                auto future = goal_handle_->cancel_goal_async();
                future.then([this](auto) { cancel_done(); });
            }
        }
    }

    void stm32_communications(const eer_interfaces::msg::PilotInput::SharedPtr controller_inputs) {
        if (bus_ >= 0) {
            if (controller_inputs->open_claw || controller_inputs->close_claw) {
                uint8_t claw_value = controller_inputs->open_claw ? 0x10 : 0x00;
                try {
                    write_byte_data(STM32_ADDRESS, 0x01, claw_value);
                } catch (const std::exception& e) {
                    RCLCPP_ERROR(this->get_logger(), "COULD NOT PERFORM ACTION WITH ACTION ADDRESS (0x10, %d)", claw_value);
                }
            } else {
                try {
                    write_byte_data(STM32_ADDRESS, 0x01, 0xaa); // Send stop signal
                } catch (const std::exception& e) {
                    // Ignore error
                }
            }

            if (controller_inputs->turn_stepper_cw || controller_inputs->turn_stepper_ccw) {
                uint8_t stepper_value = controller_inputs->turn_stepper_cw ? 0x10 : 0x00;
                try {
                    write_byte_data(STM32_ADDRESS, 0x02, stepper_value);
                } catch (const std::exception& e) {
                    RCLCPP_ERROR(this->get_logger(), "COULD NOT PERFORM ACTION WITH ACTION ADDRESS (0x02, %d)", stepper_value);
                }
            } else {
                try {
                    write_byte_data(STM32_ADDRESS, 0x02, 0xaa); // Send stop signal
                } catch (const std::exception& e) {
                    // Ignore error
                }
            }

            std::vector<uint8_t> led_addresses = {0x21, 0x22, 0x23, 0x24};

            if (controller_inputs->brighten_led || controller_inputs->dim_led) {
                headlight_led_brightness_ += controller_inputs->brighten_led ? 10 : -10;

                if (headlight_led_brightness_ > 99) {
                    headlight_led_brightness_ = 99;
                } else if (headlight_led_brightness_ < 0) {
                    headlight_led_brightness_ = 0;
                }

                for (const auto& led_address : led_addresses) {
                    try {
                        write_byte_data(STM32_ADDRESS, led_address, headlight_led_brightness_);
                    } catch (const std::exception& e) {
                        RCLCPP_ERROR(this->get_logger(), "COULD NOT PERFORM ACTION WITH ACTION ADDRESS (%d, %d)", led_address, headlight_led_brightness_);
                    }
                }
            }

            auto diagnostics_data = std::make_shared<std_msgs::msg::String>();

            if (imu_detected_) {
                diagnostics_data = obtain_imu_data(diagnostics_data);
            }

            uint8_t outside_temperature_probe_register = 0x30;

            if (controller_inputs->read_outside_temperature_probe) {
                diagnostics_data = obtain_adc_data(diagnostics_data);
                diagnostics_data = obtain_temp_sensor_data(diagnostics_data);

                bool stm_locked = false;

                try {
                    write_byte_data(STM32_ADDRESS, outside_temperature_probe_register, 1);
                    stm_locked = true;
                } catch (const std::exception& e) {
                    RCLCPP_ERROR(this->get_logger(), "Cannot write to STM32 for temperature probe");
                }

                // Data will arrive based on this format (https://cdn.sparkfun.com/datasheets/Sensors/Temp/DS18B20.pdf)
                int readLSBs = -1;
                int readMSBs = -1;
                while (stm_locked) {
                    try {
                        if (readLSBs == -1) {
                            readLSBs = read_byte_data(STM32_ADDRESS);
                        } else if (readMSBs == -1) {
                            readMSBs = read_byte_data(STM32_ADDRESS);
                        } else {
                            stm_locked = false;
                        }
                    } catch (...) {
                        continue;
                    }
                }

                diagnostics_data->data += "Outside Temperature Probe: " + std::to_string(round((static_cast<float>(((readMSBs << 8) | readLSBs) & 0b0000011111111111) / 16) * (-1 if readMSBs >= 16 else 1) * 10000) / 10000) + "C\n";
            }

            diagnostics_data_publisher_->publish(*diagnostics_data);
        }
    }

    int read_i2c_block_data(int addr, uint8_t reg, uint8_t length, uint8_t* data) {
        if (ioctl(bus_, I2C_SLAVE, addr) < 0) {
            return -1;
        }
        if (write(bus_, &reg, 1) != 1) {
            return -1;
        }
        if (read(bus_, data, length) != length) {
            return -1;
        }
        return 0;
    }

    int write_byte_data(int addr, uint8_t reg, uint8_t value) {
        if (ioctl(bus_, I2C_SLAVE, addr) < 0) {
            return -1;
        }
        uint8_t data[2] = {reg, value};
        if (write(bus_, data, 2) != 2) {
            return -1;
        }
        return 0;
    }

    uint8_t read_byte_data(int addr) {
        if (ioctl(bus_, I2C_SLAVE, addr) < 0) {
            throw std::runtime_error("Failed to set I2C address");
        }
        uint8_t value;
        if (read(bus_, &value, 1) != 1) {
            throw std::runtime_error("Failed to read I2C data");
        }
        return value;
    }

    std::string vector_to_string(const std::vector<double>& vec) {
        std::string result = "[";
        for (size_t i = 0; i < vec.size(); ++i) {
            result += std::to_string(round(vec[i] * 10000) / 10000);
            if (i < vec.size() - 1) {
                result += ", ";
            }
        }
        result += "]";
        return result;
    }

    rclcpp::Subscription<eer_interfaces::msg::ThrusterMultipliers>::SharedPtr copilot_listener_;
    rclcpp::Subscription<eer_interfaces::msg::PilotInput>::SharedPtr pilot_listener_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr diagnostics_data_publisher_1_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr diagnostics_data_publisher_2_;
    rclcpp_action::Client<eer_interfaces::action::BeaumontAutoMode>::SharedPtr action_client_;
    rclcpp::Client<beaumont_backend::srv::HSVColours>::SharedPtr brain_coral_hsv_colour_bounds_client_;
    bool autonomous_mode_active_;
    std::map<std::string, std::vector<int>> brain_coral_hsv_colour_bounds_;
    bool debugger_mode_;
    bool thruster_calibration_mode_;
    int headlight_led_brightness_;
    int current_thruster_;
    int i2c_;
    bool imu_detected_;
    std::shared_ptr<BNO055_I2C> imu_sensor_;
    uint16_t last_temperature_val_;
    int bus_;
    double power_multiplier_;
    double surge_multiplier_;
    double sway_multiplier_;
    double heave_multiplier_;
    double pitch_multiplier_;
    double yaw_multiplier_;
    std::map<int, std::shared_ptr<Thruster>> connected_channels_;




int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("i2c_master");

    // Create callback groups and executors
    auto callback_group = node->create_callback_group(rclcpp::callback_group::CallbackGroupType::Reentrant);
    auto executor = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();

    // Initialize I2C bus
    int bus = open("/dev/i2c-1", O_RDWR);
    if (bus < 0) {
        RCLCPP_ERROR(node->get_logger(), "Failed to open the i2c bus");
        return 1;
    }

    if (ioctl(bus, I2C_SLAVE, RP2040_ADDRESS) < 0) {
        RCLCPP_ERROR(node->get_logger(), "Failed to acquire bus access and/or talk to slave");
        close(bus);
        return 1;
    }

    // Create Thruster objects
    Thruster thruster1(bus, "for-port-top");
    Thruster thruster2(bus, "for-star-top");
    // Add more thrusters as needed

    // Spin the node
    executor->add_node(node);
    executor->spin();

    close(bus);
    rclcpp::shutdown();
    return 0;
}