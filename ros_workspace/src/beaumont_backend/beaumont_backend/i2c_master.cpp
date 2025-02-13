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
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>
#include <cstring>
#include <iostream>
#include <vector>

// Thruster channels are based on Beaumont
std::map<std::string, int> THRUSTER_CHANNELS = {
    {"for-port-top", 1},
    {"for-star-top", 0},
    {"aft-port-top", 4},
    {"aft-star-top", 3},
    {"for-port-bot", 2},
    {"for-star-bot", 6},
    {"aft-port-bot", 5},
    {"aft-star-bot", 7}
};

// In the thruster class, the target speed is set by the user. 
// Each thruster accelerates towards the target speed by the acceleration below
const int THRUSTER_ACCELERATION = 8;

// Max % acceleration per second can be calculated by (THRUSTER_ACCELERATION/254)*(pilot input frequency in Hz)
// Where 0 is max reverse speed, 127 is center speed, and 254 is max forward speed

const int RP2040_ADDRESS = 0x08;

const int STM32_ADDRESS = 0x69;

std::map<std::string, int> ADC_ADDRESSES = {
    {"adc_48v_bus", 0x55},
    {"adc_12v_bus", 0x56},
    {"adc_5v_bus", 0x59}
};

std::map<std::string, std::pair<int, int>> ADC_VOLTAGE_DIVIDER_VALUES = {
    {"adc_48v_bus", {20000, 1000}},
    {"adc_12v_bus", {5000, 1000}},
    {"adc_5v_bus", {2000, 1000}}
};

std::map<std::string, int> TEMPERATURE_SENSOR_ADDRESSES = {
    {"power_board_u8", 0x48},
    {"power_board_u9", 0x49},
    {"power_board_u10", 0x4a},
    {"mega_board_ic2", 0x4b},
    {"power_board_u11", 0x4c},
    {"mega_board_ic1", 0x4e}
};

// How often to read data on i2c bus
const int DIAGNOSTICS_REQUESTS_PERIOD = 2;


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

    // Drive thrusters using given speed parameters.
    
    // :param speed: speed of thrusters with a valid range of -1.0 to 1.0
    void fly(double speed) {
        target_ = speed * 127 + 127;
    }

    void tick() {


        if (!thruster_armed_) {
            arm_thruster();
            return;
        }

        // Must accelerate towards target speed
        if (current_ != target_) {

            if (std::abs(target_ - current_) > THRUSTER_ACCELERATION) {

                // Determine the directtion in which thruster must accelerate
                double direction = (target_ - current_) / std::abs(target_ - current_);

                // Accelerate towards desired direction by THRUSTER_ACCELERATION
                current_ += static_cast<int>(direction * THRUSTER_ACCELERATION);

                // Ensure the current speed does not go out of range
                if (current_ > 254) current_ = 254;
                if (current_ < 0) current_ = 0;

                // If communication with RP2040 fails, attempt to arm the thruster again as soon as communication is reestablished
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

                // If communication with RP2040 fails, attempt to arm the thruster again as soon as communication is reestablished
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

    /*
    This class will communicate to multiple devices on the same I2C bus. These include the following:

        1. RP2040: Talks to the 8 Thrusters connected to the bot.
        2. STM32: Talks to claw, bilge pump motor, dimmable LEDs, and outside temperature probe. 
        3. 3 Analog digital converters (ADCs) on the 48v, 12v, and 5v buses to mesure board health. 
        4. 6 Temperature sensors on the boards (4 on power board, 2 on mega board).
        5. BNO055 Inertial Mesurement Unit (IMU). Communication will be done using the adafruit adafruit_bno055 library.

    All aforementioned devices will be on the same I2C bus, meaning that it is only possible to communicate to one at a time. This means these processes
    are not thread-safe. Thus, communication to each of these devices will be done sequentially and only when requested by the pilot (see self.pilot_listener_callback).
    */

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

        
        /*################################################################
        ################### THRUSTER CALIBRATION MODE ###################
        ################################################################*/
    
        if (thruster_calibration_mode_) {
            debugger_mode_ = true;
            current_thruster_ = 0;
        }


        /*###############################################
        ################### DEBUGGER ####################
        ###############################################*/

        if (debugger_mode_) {
            debugger_ = this->create_publisher<std_msgs::msg::String>("debugger", 10);
        }

        /*###############################################################
        ################### INITIALIZE ADAFRUIT I2C ###################
        ###############################################################*/

        i2c_ = open("/dev/i2c-1", O_RDWR);
        if (i2c_ < 0) {
            RCLCPP_ERROR(this->get_logger(), "No Hardware on I2C bus");
        }

        /*#################################################
        ###################### IMU ######################
        #################################################*/

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

        /*##############################################################
        ###################### Initialize SMBUS ######################
        ##############################################################*/

        bus_ = open("/dev/i2c-1", O_RDWR);
        if (bus_ < 0) {
            RCLCPP_ERROR(this->get_logger(), "COULD NOT INITIALIZE SMBUS");
        } else {
            RCLCPP_INFO(this->get_logger(), "INITIALIZED SMBUS!");
        }

        /*#################################################
        ################### THRUSTERS ###################
        #################################################*/
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

        /*##################################################
        ###################### ADCs ######################
        ##################################################*/

        if (bus_ >= 0) {
            for (const auto& key : ADC_ADDRESSES) {
                configured_adcs_[key.first] = false;
            }
        }
    }

    void send_autonomous_mode_goal() {
        fetch_brain_coral_hsv_colour_bounds();

        auto goal_msg = beaumont_interfaces::action::BeaumontAutoMode::Goal();
        goal_msg.is_for_sim = false;

        // HSV (hue, shade, value) bounds for filtering brain coral area
        goal_msg.lower_hsv_bound = brain_coral_hsv_colour_bounds_["lower_hsv"];
        goal_msg.upper_hsv_bound = brain_coral_hsv_colour_bounds_["upper_hsv"];

        goal_msg.starting_power = static_cast<int>(power_multiplier_ * 100);
        goal_msg.starting_surge = static_cast<int>(surge_multiplier_ * 100);
        goal_msg.starting_sway = static_cast<int>(sway_multiplier_ * 100);
        goal_msg.starting_heave = static_cast<int>(heave_multiplier_ * 100);
        goal_msg.starting_pitch = static_cast<int>(pitch_multiplier_ * 100);
        goal_msg.starting_yaw = static_cast<int>(yaw_multiplier_ * 100);

        action_client_->wait_for_action_server();

        auto send_goal_options = rclcpp_action::Client<beaumont_interfaces::action::BeaumontAutoMode>::SendGoalOptions();
        send_goal_options.feedback_callback = std::bind(&I2CMaster::feedback_callback, this, std::placeholders::_1, std::placeholders::_2);
        send_goal_options.result_callback = std::bind(&I2CMaster::goal_response_callback, this, std::placeholders::_1);

        auto goal_handle_future = action_client_->async_send_goal(goal_msg, send_goal_options);
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

    std::shared_ptr<std_msgs::msg::String> obtain_imu_data(std::shared_ptr<std_msgs::msg::String> diagnostics_data) {
        /*
        Grabs relevant information from the IMU on the I2C Bus.
        Every time a piece of data is accessed, that is an i2c read transaction. These transactions finish by the time 
        the value is returned. See (https://github.com/adafruit/Adafruit_CircuitPython_BNO055/blob/main/adafruit_bno055.py).
        */
        diagnostics_data->data += "Temperature: " + std::to_string(imu_sensor_.temperature) + "C\n";

        // This code below was recommended by Adafruit (https://learn.adafruit.com/adafruit-bno055-absolute-orientation-sensor/python-circuitpython)
        if (std::abs(diagnostics_data->temperature - last_temperature_val_) == 128) {
            diagnostics_data->temperature = 0b00111111 & diagnostics_data->temperature;
        }
        last_temperature_val_ = diagnostics_data->temperature;

        diagnostics_data->data += "Acceleration: " + vector_to_string(imu_sensor_.acceleration) + "\n";
        diagnostics_data->data += "Magnetic: " + vector_to_string(imu_sensor_.magnetic) + "\n";
        diagnostics_data->data += "Euler: " + vector_to_string(imu_sensor_.euler) + "\n";
        diagnostics_data->data += "Linear Acceleration: " + vector_to_string(imu_sensor_.linear_acceleration) + "\n";

        return diagnostics_data;
    }

    std::shared_ptr<std_msgs::msg::String> obtain_adc_data(std::shared_ptr<std_msgs::msg::String> diagnostics_data) {
        /*
        Initially, this function will set ADC configurations. This should be done on power-up as recommended on ADC datasheet.

        In configuration, the 3 MSB set the automatic conversion mode, which configures the ADC to continually perform conversions
        without read requests. Based on the table in the datasheet, the three bits set the cycle time (i.e. sample rate) of 
        these automatic conversions.

        Not configuring the ADC to automatic conversion mode will essentially make it so that the sample rate is the read 
        rate (1Hz). This means every piece of data read will be one second old.
        
        The other bits have to do with how the ADC handles alert conditions (when readings exceed a user-set min or max).

        The below code does everything needed for our purposes. All ADC registers are listed below:
        0b000 - Conversion Result (r) (we read this for voltage)
        0b001 - Alert Status (r/w) (ignored)
        0b010 - Configuration (r/w) (we configure it for auto-conversion mode)
        0b011 - Low Limit (r/w) (We leave this as 0)
        0b100 - High Limit (r/w) (We leave this as 4096 or 2**12)
        0b110 - Lowest Conversion to Date (r/w) (ignored)
        0b111 - Highest Conversion to Date (r/w) (ignored)
        */

        for (auto& [key, is_configured] : configured_adcs_) {
            if (is_configured) { // Assume ADC is configured. Read conversion result
                try {
                    uint8_t conversion_result_register = 0b00000000;
                    uint8_t conversion_result_length_in_bytes = 2;
                    uint8_t read[2];

                    if (read_i2c_block_data(ADC_ADDRESSES[key], conversion_result_register, conversion_result_length_in_bytes, read) < 0) {
                        throw std::runtime_error("I2C read error");
                    }

                    int first_byte_data = read[0] << 6; // This will contain the 4 MSBs
                    int second_byte_data = read[1] >> 2; // This will contain the 6 LSBs
                    
                    // Combine the first and second byte conversion result data to get the 10 bit value,
                    // Divide this by 3.3/(2**10) since analog input precision is 1024 from 0 to 3.3V
                    double adc_read_voltage = (first_byte_data + second_byte_data) * (3.3 / (1 << 10));

                    double voltage = (static_cast<double>(ADC_VOLTAGE_DIVIDER_VALUES[key].first + ADC_VOLTAGE_DIVIDER_VALUES[key].second) / ADC_VOLTAGE_DIVIDER_VALUES[key].second) * adc_read_voltage;
                    
                    diagnostics_data->data += key + ": " + std::to_string(round(voltage * 10000) / 10000) + "V\n";
                } catch (const std::runtime_error& e) {
                    diagnostics_data->data += key + ": 0V\n";
                    configured_adcs_[key] = false;
                }
            } else {
                uint8_t configuration_address = 0b00000010;
                uint8_t configuration_value = 0b10100000; // (1.7 kilo samples per second)

                try {
                    write_byte_data(ADC_ADDRESSES[key], configuration_address, configuration_value);
                    uint8_t configuration = read_byte_data(ADC_ADDRESSES[key], configuration_address);

                    if (configuration == configuration_value) {
                        std::cout << "CONFIGURED ADC " << key << std::endl;
                        configured_adcs_[key] = true;
                    } else {
                        throw std::runtime_error("Configuration mismatch");
                    }
                } catch (const std::runtime_error& e) {
                    configured_adcs_[key] = false;
                }
            }
        }

        return diagnostics_data;
    }


    std::shared_ptr<std_msgs::msg::String> obtain_temp_sensor_data(std::shared_ptr<std_msgs::msg::String> diagnostics_data) {
        /*
         The temperature sensor does not need to be configured, as the default configuration 
        options are good. The registers are as follows:

        0b00 - Temperature Reading (r) (11 MSBs of two bytes)
        0b01 - Configuration (r/w) (ignored)
        0b10 - Hysteresis (r/w) (ignored, used for alerts) (default 75 celcius)
        0b11 - Overtemperature Shutdown Value (r/w) (ignored) (default 80 celcius)

        The alert is indicated by a seperate pin (not an i2c register)
        */
        
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
        /*
        Communications to the STM32 is required for the 12 DC motor (claw linear actuator),
        bilge pump, dimmable LEDs, and outside temperature probe readings.

        Each action will be achieved through:

        self.bus.write_byte_data(STM32_ADDRESS, action_register, value)
        or
        self.bus.read_byte(STM32_ADDRESS)

        Possible actions and associated registers are as follows:

        STM32 Address: 0x69

        DC Motor Structure (Bytes listed from left to right): 
        Byte 1: Command and Motor Selection
        -- Command: 0 (Hex)
        -- Motor Number: 1 or 2 (Hex)
        -- Example Byte: 0x01 (DC Motor 1)
        Byte 2: Motor Direction and Speed (UNUSED)
        -- Motor Direction: 0 or 1 (will figure out which is forward when we test)
        -- Speed: Unused right now so set to 0
        -- Example Byte: 0x10 (Direction = 1)

        STEPPER Structure:
        Byte 1: Command and Stepper Selection
        -- Command: 1 (hex)
        -- Stepper Selection: 1 or 2
        -- Example: 0x12 (Stepper 2)
        Byte 2: Direction and Speed
        -- Direction: 0 or 1
        -- Speed: Unused right now
        -- Example: 0x10 (Go in direction 1)

        LED Structure:
        Byte 1: Command and LED Selection
        -- Command: 2 (Hex)
        -- LED Selection: 1-4
        -- Example: 0x24 (LED 4)
        Byte 2: Brightness (0-99)

        Outside Temperature Probe reading mode:
        Byte 1: Select probe
        -- Command: 3 (hex)
        -- Temperature Probe Selection: 0
        -- Example: 0x30
        Byte 2: Data
        -- Can be any byte

        Outside Temperature Probe reading:
        Must read from temperature probe TWICE to take
        STM32 out of temperature probe reading mode.
        Initiate two indivisual read_byte transactions
        on the STM32 address.
        */
        
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

    std::map<std::string, double> rov_math(const ControllerInputs& controller_inputs) {
        /*
        Determines how much power to give to each thruster based on pilot input.
        */

        std::map<std::string, double> thruster_values;

        // Old Thruster Math
        double overall_scaling_factor = std::sqrt(3);

        double surge = controller_inputs.surge * power_multiplier_ * surge_multiplier_ * 0.01;
        double sway = controller_inputs.sway * power_multiplier_ * sway_multiplier_ * 0.01;
        double yaw = controller_inputs.yaw * power_multiplier_ * yaw_multiplier_ * 0.01;

        double heave = 0;
        if (controller_inputs.heave_up || controller_inputs.heave_down) {
            heave = (controller_inputs.heave_up ? power_multiplier_ * heave_multiplier_ : 0) + 
                    (controller_inputs.heave_down ? -power_multiplier_ * heave_multiplier_ : 0);
        } else {
            heave = controller_inputs.heave * power_multiplier_ * heave_multiplier_ * 0.01;
        }

        double pitch = 0;
        if (controller_inputs.pitch_up || controller_inputs.pitch_down) {
            pitch = (controller_inputs.pitch_up ? power_multiplier_ * pitch_multiplier_ : 0) + 
                    (controller_inputs.pitch_down ? -power_multiplier_ * pitch_multiplier_ : 0);
        } else {
            pitch = controller_inputs.pitch * power_multiplier_ * pitch_multiplier_ * 0.01;
        }

        double sum_of_magnitudes_of_linear_movements = std::abs(surge) + std::abs(sway) + std::abs(heave);
        double sum_of_magnitudes_of_rotational_movements = std::abs(pitch) + std::abs(yaw);

        double strafe_power = std::sqrt(surge * surge + sway * sway + heave * heave);
        double strafe_scaling_coefficient = strafe_power / sum_of_magnitudes_of_linear_movements;
        double strafe_average_coefficient = strafe_power / (strafe_power + sum_of_magnitudes_of_rotational_movements);
        double combined_strafe_coefficient = strafe_scaling_coefficient * strafe_average_coefficient;
        double rotation_average_coefficient = sum_of_magnitudes_of_rotational_movements / (strafe_power + sum_of_magnitudes_of_rotational_movements);

        thruster_values["for-port-bot"] = (((-surge) + (sway) + (heave)) * combined_strafe_coefficient + ((pitch) + (yaw)) * rotation_average_coefficient) / overall_scaling_factor;
        thruster_values["for-star-bot"] = (((-surge) + (-sway) + (heave)) * combined_strafe_coefficient + ((pitch) + (-yaw)) * rotation_average_coefficient) / overall_scaling_factor;
        thruster_values["aft-port-bot"] = (((surge) + (sway) + (heave)) * combined_strafe_coefficient + ((-pitch) + (-yaw)) * rotation_average_coefficient) / -overall_scaling_factor;
        thruster_values["aft-star-bot"] = (((surge) + (-sway) + (heave)) * combined_strafe_coefficient + ((-pitch) + (yaw)) * rotation_average_coefficient) / overall_scaling_factor;
        thruster_values["for-port-top"] = (((-surge) + (sway) + (-heave)) * combined_strafe_coefficient + ((-pitch) + (yaw)) * rotation_average_coefficient) / overall_scaling_factor;
        thruster_values["for-star-top"] = (((-surge) + (-sway) + (-heave)) * combined_strafe_coefficient + ((-pitch) + (-yaw)) * rotation_average_coefficient) / overall_scaling_factor;
        thruster_values["aft-port-top"] = (((surge) + (sway) + (-heave)) * combined_strafe_coefficient + ((pitch) + (-yaw)) * rotation_average_coefficient) / overall_scaling_factor;
        thruster_values["aft-star-top"] = (((surge) + (-sway) + (-heave)) * combined_strafe_coefficient + ((pitch) + (yaw)) * rotation_average_coefficient) / overall_scaling_factor;

        // Individual Thruster Testing
        if (thruster_calibration_mode_) {
            std_msgs::msg::String current_thruster_and_value;

            if (controller_inputs.brighten_led) {
                current_thruster_ = (current_thruster_ == 0) ? 7 : current_thruster_ - 1;
                current_thruster_and_value.data = std::to_string(current_thruster_) + ": " + std::to_string(controller_inputs.surge);
                debugger_.publish(current_thruster_and_value);
            } else if (controller_inputs.dim_led) {
                current_thruster_ = (current_thruster_ == 7) ? 0 : current_thruster_ + 1;
                current_thruster_and_value.data = std::to_string(current_thruster_) + ": " + std::to_string(controller_inputs.surge);
                debugger_.publish(current_thruster_and_value);
            }

            for (const auto& [thruster_position, channel] : THRUSTER_CHANNELS) {
                if (channel == current_thruster_) {
                    thruster_values[thruster_position] = surge;
                } else {
                    thruster_values[thruster_position] = 0;
                }
            }
        }

        return thruster_values;
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

    void fetch_brain_coral_hsv_colour_bounds() {
        auto request = std::make_shared<beaumont_interfaces::srv::HSVColours::Request>();
        request->load_to_database = false;

        brain_coral_hsv_colour_bounds_client_->wait_for_service();

        auto future = brain_coral_hsv_colour_bounds_client_->async_send_request(request);

        future.wait();
        if (future.get()->success) {
            brain_coral_hsv_colour_bounds_["upper_hsv"] = future.get()->upper_hsv;
            brain_coral_hsv_colour_bounds_["lower_hsv"] = future.get()->lower_hsv;
        } else {
            RCLCPP_INFO(this->get_logger(), "No HSV colour bounds stored in database. Will keep using default.");
        }
    }

    void goal_response_callback(const rclcpp_action::ClientGoalHandle<beaumont_interfaces::action::BeaumontAutoMode>::WrappedResult &result) {
        if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
            RCLCPP_INFO(this->get_logger(), "Goal accepted :)");
            auto get_result_future = result.goal_handle->async_result();
            get_result_future.wait();
            get_result_callback(get_result_future.get());
        } else {
            RCLCPP_INFO(this->get_logger(), "Goal rejected :(");
        }
    }

    void get_result_callback(const beaumont_interfaces::action::BeaumontAutoMode::Result::SharedPtr result) {
        auto autonomous_mode_status = std_msgs::msg::String();
        autonomous_mode_status.data = "Autonomous Mode off, " + std::string(result->success ? "Mission Success" : "Mission Failed");
        diagnostics_data_publisher_2_->publish(autonomous_mode_status);
        autonomous_mode_active_ = false;
    }

    void feedback_callback(rclcpp_action::ClientGoalHandle<beaumont_interfaces::action::BeaumontAutoMode>::SharedPtr, const std::shared_ptr<const beaumont_interfaces::action::BeaumontAutoMode::Feedback> feedback) {
        auto autonomous_mode_status = std_msgs::msg::String();
        autonomous_mode_status.data = "Autonomous Mode on, " + feedback->status;
        diagnostics_data_publisher_2_->publish(autonomous_mode_status);
    }

    void cancel_done(const rclcpp_action::ClientGoalHandle<beaumont_interfaces::action::BeaumontAutoMode>::WrappedResult &result) {
        auto cancel_response = result.goal_handle->async_cancel();
        cancel_response.wait();
        auto autonomous_mode_status = std_msgs::msg::String();
        if (!cancel_response.get()->goals_canceling.empty()) {
            autonomous_mode_status.data = "Auto mode successfully canceled";
        } else {
            autonomous_mode_status.data = "Auto mode failed to cancel";
        }
        diagnostics_data_publisher_2_->publish(autonomous_mode_status);
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