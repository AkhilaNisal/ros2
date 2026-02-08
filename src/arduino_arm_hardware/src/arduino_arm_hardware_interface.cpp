#include "arduino_arm_hardware/arduino_arm_hardware_interface.hpp"
#include "pluginlib/class_list_macros.hpp"

#include <fcntl.h>
#include <unistd.h>
#include <cstring>
#include <sstream>
#include <iostream>
#include <cmath> // for std::isnan

namespace arduino_arm_hardware 
{

hardware_interface::CallbackReturn
ArduinoArmHardwareInterface::on_init(
  const hardware_interface::HardwareComponentInterfaceParams & params)
{
  if (SystemInterface::on_init(params) != CallbackReturn::SUCCESS)
    return CallbackReturn::ERROR;

  port_ = params.hardware_info.hardware_parameters.at("port");


  size_t n_joints = params.hardware_info.joints.size();
  hw_positions_.resize(n_joints, 0.0);
  hw_commands_.resize(n_joints, 0.0);
  joint_names_.resize(n_joints);

  for (size_t i = 0; i < n_joints; ++i) {
        joint_names_[i] = params.hardware_info.joints[i].name;
        RCLCPP_INFO(rclcpp::get_logger("ArduinoArmHardware"),
                    "Found joint: %s", joint_names_[i].c_str());
    }


  RCLCPP_INFO(rclcpp::get_logger("ArduinoArmHardware"), 
              "Initialized %zu joints on port %s", n_joints, port_.c_str());

  
  return CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn
ArduinoArmHardwareInterface::on_configure(const rclcpp_lifecycle::State &)
{
  if (!open_serial())
  {
    RCLCPP_ERROR(rclcpp::get_logger("ArduinoArmHardware"), 
                 "Failed to open serial port %s", port_.c_str());
    return CallbackReturn::ERROR;
  }

  RCLCPP_INFO(rclcpp::get_logger("ArduinoArmHardware"), 
              "Serial port %s opened", port_.c_str());
  return CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn
ArduinoArmHardwareInterface::on_activate(const rclcpp_lifecycle::State &)
{
  // hw_positions_ = hw_commands_;
  for (size_t i = 0; i < hw_positions_.size(); ++i)
        set_state(joint_names_[i] + "/position", hw_positions_[i]);

  RCLCPP_INFO(rclcpp::get_logger("ArduinoArmHardware"), "Arduino hardware activated");
  return CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn
ArduinoArmHardwareInterface::on_deactivate(const rclcpp_lifecycle::State &)
{
  close_serial();
  RCLCPP_INFO(rclcpp::get_logger("ArduinoArmHardware"), "Arduino hardware deactivated");
  return CallbackReturn::SUCCESS;
}

hardware_interface::return_type ArduinoArmHardwareInterface::write(
    const rclcpp::Time &, const rclcpp::Duration &)
{
    if (serial_fd_ < 0)
        return hardware_interface::return_type::ERROR;

    // Fetch commands safely
    for (size_t i = 0; i < hw_commands_.size(); ++i)
    {
        double cmd = get_command(joint_names_[i] + "/position");
        if (std::isnan(cmd))  // fallback to current position
            cmd = hw_positions_[i];
        hw_commands_[i] = cmd;

        // -------- DEBUG: print received command --------
        // RCLCPP_INFO(rclcpp::get_logger("ArduinoArmHardware"), 
        //             "Received command from MoveIt: Joint %s -> %f", 
        //             joint_names_[i].c_str(), hw_commands_[i]);
    }

    // Send to Arduino
    std::ostringstream ss;
    ss << "J:";
    for (size_t i = 0; i < hw_commands_.size(); ++i)
    {
        ss << hw_commands_[i];
        if (i + 1 < hw_commands_.size()) ss << ",";
    }
    ss << "\n";

    // RCLCPP_INFO(rclcpp::get_logger("ArduinoArmHardware"),
    //             "Sending to Arduino: %s", ss.str().c_str());

    if (!write_serial(ss.str()))
    {
        RCLCPP_WARN(rclcpp::get_logger("ArduinoArmHardware"), "Failed to write to Arduino");
        return hardware_interface::return_type::ERROR;
    }

    return hardware_interface::return_type::OK;
}




hardware_interface::return_type
ArduinoArmHardwareInterface::read(const rclcpp::Time &, const rclcpp::Duration &)
{
  if (serial_fd_ < 0)
  {
    // hw_positions_ = hw_commands_;
    for (size_t i = 0; i < hw_positions_.size(); ++i)
          set_state(joint_names_[i] + "/position", hw_commands_[i]);

    return hardware_interface::return_type::OK;
  }



  std::string line = read_serial();


   // ------------------ DEBUG PRINT ------------------
  if (!line.empty())
  {
    // RCLCPP_INFO(rclcpp::get_logger("ArduinoArmHardware"), "Received from Arduino: %s", line.c_str());
    // std::cout << "Received from Arduino: " << line << std::endl;
  }

  if (!line.empty() && line[0] == 'S')
  {
    std::stringstream ss(line.substr(2)); // skip 'S:'
    for (size_t i = 0; i < hw_positions_.size(); ++i)
    {
      char comma;
      ss >> hw_positions_[i];
      ss >> comma; // ignore comma
      set_state(joint_names_[i] + "/position", hw_positions_[i]);
    }
  }
  else
  {
    // hw_positions_ = hw_commands_; // assume perfect tracking
    for (size_t i = 0; i < hw_positions_.size(); ++i)
            set_state(joint_names_[i] + "/position", hw_commands_[i]);

  }

  return hardware_interface::return_type::OK;
}

// ------------------- Serial helpers -------------------

bool ArduinoArmHardwareInterface::open_serial()
{
  serial_fd_ = ::open(port_.c_str(), O_RDWR | O_NOCTTY | O_SYNC);
  if (serial_fd_ < 0) return false;

  memset(&tty_, 0, sizeof tty_);
  if (tcgetattr(serial_fd_, &tty_) != 0) return false;

  cfsetospeed(&tty_, B115200);
  cfsetispeed(&tty_, B115200);

  tty_.c_cflag = (tty_.c_cflag & ~CSIZE) | CS8;
  tty_.c_cflag |= (CLOCAL | CREAD);
  tty_.c_cflag &= ~(PARENB | CSTOPB | CRTSCTS);
  tty_.c_lflag = 0;
  tty_.c_iflag = 0;
  tty_.c_oflag = 0;

  tty_.c_cc[VMIN] = 1;
  tty_.c_cc[VTIME] = 5;  //5;

  return tcsetattr(serial_fd_, TCSANOW, &tty_) == 0;
}

void ArduinoArmHardwareInterface::close_serial()
{
  if (serial_fd_ >= 0)
  {
    ::close(serial_fd_);
    serial_fd_ = -1;
  }
}

bool ArduinoArmHardwareInterface::write_serial(const std::string & data)
{
  if (serial_fd_ < 0) return false;
  ssize_t n = ::write(serial_fd_, data.c_str(), data.size());
  return n > 0;
}

std::string ArduinoArmHardwareInterface::read_serial()
{
    if (serial_fd_ < 0) return "";

    char buf[256];
    ssize_t n = ::read(serial_fd_, buf, sizeof(buf));
    if (n <= 0) return "";

    serial_buffer_ += std::string(buf, n);

    size_t pos = serial_buffer_.find('\n');
    if (pos != std::string::npos)
    {
        std::string line = serial_buffer_.substr(0, pos);
        serial_buffer_ = serial_buffer_.substr(pos + 1); // remove processed line
        return line;
    }

    return ""; // no complete line yet
}

} // namespace arduino_arm_hardware

// ------------------- Pluginlib export -------------------
PLUGINLIB_EXPORT_CLASS(
  arduino_arm_hardware::ArduinoArmHardwareInterface, // <-- updated
  hardware_interface::SystemInterface)
