#include <memory>
#include <thread>
#include <vector>

#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"
#include "std_msgs/msg/float64.hpp"
#include "eer_interfaces/msg/waterwitch_control.hpp"
#include "waterwitch_constants.h"

#define I2C_BUS "/dev/i2c-1"

class I2CMaster : public rclcpp::Node
{
public:
  I2CMaster() : Node("I2CMaster")
  {

    auto control_values_subscriber_callback =
      [this](eer_interfaces::msg::WaterwitchControl::UniquePtr control_values_msg) -> void {
        for (int thruster_index = 0; thruster_index < 6; thruster_index++) {

            // Map the thrust value from the range [-1, 1] to [0, 255]
            uint8_t thrust = static_cast<uint8_t>((control_values_msg->thrust[thruster_index] + 1) * 127.5);

            // Publish on i2c bus
            write_to_i2c(RP2040_ADDRESS, 2, control_values_msg->thruster_map[thruster_index], thrust);
        }
      };

    // Open the i2c file
    if ((i2c_file = open(I2C_BUS, O_RDWR)) < 0) {
      RCLCPP_ERROR(this->get_logger(), "Failed to open the i2c bus");
    }

    control_values_subscriber =
      this->create_subscription<eer_interfaces::msg::WaterwitchControl>(
        "waterwitch_control_values", 10, control_values_subscriber_callback);  
  }

  // Deconstructor for I2CMaster
  ~I2CMaster()
  { 
    // Close the i2c file
    if(i2c_file >= 0){
      close(i2c_file);
    }
  }

private:
  rclcpp::Subscription<eer_interfaces::msg::WaterwitchControl>::SharedPtr control_values_subscriber;
  int i2c_file;

  void write_to_i2c(int device_address, uint8_t num_bytes, uint8_t byte_1, uint8_t byte_2 = 0)
  {
    
    // Raise an error if the i2c file is not open
    if (i2c_file < 0) {
      RCLCPP_ERROR(this->get_logger(), "I2C bus is not open");
      return;
    }

    if (ioctl(i2c_file, I2C_SLAVE, device_address) < 0) {
      RCLCPP_ERROR(this->get_logger(), "Failed to acquire bus access and/or talk to slave");
      return;
    }

    if (num_bytes == 1) {
      uint8_t buffer[1];
      buffer[0] = byte_1;
      if (write(i2c_file, buffer, 1) != 1) {
        RCLCPP_ERROR(this->get_logger(), "Failed to write to the i2c bus");
      }
    }
    else if (num_bytes == 2)
    {
      uint8_t buffer[2];
      buffer[0] = byte_1;
      buffer[1] = byte_2;
      if (write(i2c_file, buffer, 2) != 2) {
        RCLCPP_ERROR(this->get_logger(), "Failed to write to the i2c bus");
      }
    }
  }
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<I2CMaster>());
  rclcpp::shutdown();
  return 0;
}