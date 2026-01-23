#ifndef ARDUINO_ARM_HARDWARE_INTERFACE_HPP
#define ARDUINO_ARM_HARDWARE_INTERFACE_HPP

#include <memory>
#include <string>
#include <vector>
#include <termios.h>

#include "hardware_interface/system_interface.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/state.hpp"

namespace arduino_arm_hardware  // <-- Changed namespace
{

class ArduinoArmHardwareInterface : public hardware_interface::SystemInterface  // <-- Changed class name
{
public:
  // Lifecycle callbacks
  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareComponentInterfaceParams & params) override;

  hardware_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  // Read & Write joint states
  hardware_interface::return_type read(
    const rclcpp::Time & time,
    const rclcpp::Duration & period) override;

  hardware_interface::return_type write(
    const rclcpp::Time & time,
    const rclcpp::Duration & period) override;

private:
  
  std::vector<std::string> joint_names_;
  // Joint states & commands
  std::vector<double> hw_positions_;  // Current joint positions
  std::vector<double> hw_commands_;   // Target joint positions
  std::string serial_buffer_;
  
  // Serial communication
  std::string port_;       // e.g., "/dev/ttyACM0"
  int serial_fd_{-1};      // File descriptor
  struct termios tty_;     // Serial port settings

  // Serial helper functions
  bool open_serial();                  // Open and configure serial port
  void close_serial();                 // Close serial port
  bool write_serial(const std::string & data); // Send data to Arduino
  std::string read_serial();           // Read data from Arduino
};

} // namespace arduino_arm_hardware

#endif // ARDUINO_ARM_HARDWARE_INTERFACE_HPP
