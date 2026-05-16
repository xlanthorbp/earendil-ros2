#include "earendil_hardware/earendil_system.hpp"

#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <vector>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <iostream>
#include <sstream>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace earendil_hardware
{

hardware_interface::CallbackReturn EarendilSystemHardware::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (
    hardware_interface::SystemInterface::on_init(info) !=
    hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  // Read parameters from URDF
  port_ = info_.hardware_parameters["port"];
  baud_rate_ = std::stoi(info_.hardware_parameters["baud_rate"]);
  enc_ticks_per_rev_ = std::stod(info_.hardware_parameters["enc_ticks_per_rev"]);

  hw_positions_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_velocities_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_commands_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());

  for (const hardware_interface::ComponentInfo & joint : info_.joints)
  {
    if (joint.command_interfaces.size() != 1)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("EarendilSystemHardware"),
        "Joint '%s' has %zu command interfaces found. 1 expected.", joint.name.c_str(),
        joint.command_interfaces.size());
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.command_interfaces[0].name != hardware_interface::HW_IF_VELOCITY)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("EarendilSystemHardware"),
        "Joint '%s' have %s command interfaces found. '%s' expected.", joint.name.c_str(),
        joint.command_interfaces[0].name.c_str(), hardware_interface::HW_IF_VELOCITY);
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.state_interfaces.size() != 2)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("EarendilSystemHardware"),
        "Joint '%s' has %zu state interface. 2 expected.", joint.name.c_str(),
        joint.state_interfaces.size());
      return hardware_interface::CallbackReturn::ERROR;
    }
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> EarendilSystemHardware::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  for (auto i = 0u; i < info_.joints.size(); i++)
  {
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_positions_[i]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_velocities_[i]));
  }

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> EarendilSystemHardware::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  for (auto i = 0u; i < info_.joints.size(); i++)
  {
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
      info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_commands_[i]));
  }

  return command_interfaces;
}

bool EarendilSystemHardware::connect_serial()
{
  serial_fd_ = open(port_.c_str(), O_RDWR | O_NOCTTY | O_SYNC);
  if (serial_fd_ < 0) {
    RCLCPP_ERROR(rclcpp::get_logger("EarendilSystemHardware"), "Error opening %s", port_.c_str());
    return false;
  }

  struct termios tty;
  if (tcgetattr(serial_fd_, &tty) != 0) {
    RCLCPP_ERROR(rclcpp::get_logger("EarendilSystemHardware"), "Error from tcgetattr");
    return false;
  }

  cfsetospeed(&tty, B115200); // Fixed at 115200 for now
  cfsetispeed(&tty, B115200);

  tty.c_cflag |= (CLOCAL | CREAD);    // ignore modem controls, enable reading
  tty.c_cflag &= ~CSIZE;
  tty.c_cflag |= CS8;         // 8-bit characters
  tty.c_cflag &= ~PARENB;     // no parity bit
  tty.c_cflag &= ~CSTOPB;     // only need 1 stop bit
  tty.c_cflag &= ~CRTSCTS;    // no hardware flowcontrol

  // setup for non-canonical mode
  tty.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL | IXON);
  tty.c_lflag &= ~(ECHO | ECHONL | ICANON | ISIG | IEXTEN);
  tty.c_oflag &= ~OPOST;

  // fetch bytes as they become available
  tty.c_cc[VMIN] = 0;
  tty.c_cc[VTIME] = 1; // 0.1s timeout

  if (tcsetattr(serial_fd_, TCSANOW, &tty) != 0) {
    RCLCPP_ERROR(rclcpp::get_logger("EarendilSystemHardware"), "Error from tcsetattr");
    return false;
  }
  
  // Clear buffers
  tcflush(serial_fd_, TCIOFLUSH);
  return true;
}

void EarendilSystemHardware::disconnect_serial()
{
  if (serial_fd_ >= 0) {
    close(serial_fd_);
    serial_fd_ = -1;
  }
}

hardware_interface::CallbackReturn EarendilSystemHardware::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  if (!connect_serial()) {
    return hardware_interface::CallbackReturn::ERROR;
  }
  
  RCLCPP_INFO(rclcpp::get_logger("EarendilSystemHardware"), "Successfully connected to %s", port_.c_str());

  for (auto i = 0u; i < hw_positions_.size(); i++)
  {
    hw_commands_[i] = 0.0;
    hw_positions_[i] = 0.0;
    hw_velocities_[i] = 0.0;
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn EarendilSystemHardware::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // Stop motors before disconnecting
  send_motor_command(0.0, 0.0);
  disconnect_serial();
  
  RCLCPP_INFO(rclcpp::get_logger("EarendilSystemHardware"), "Disconnected from serial port.");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type EarendilSystemHardware::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & period)
{
  // Ask Arduino for encoder ticks
  std::string req = "e\n";
  ::write(serial_fd_, req.c_str(), req.length());

  char buf[256];
  int n = ::read(serial_fd_, buf, sizeof(buf) - 1);
  if (n > 0) {
    buf[n] = '\0';
    std::string response(buf);
    
    // Parse response "E <left_ticks> <right_ticks>"
    // Simplified parsing assuming clean data
    if (response.length() > 2 && response[0] == 'E') {
      std::stringstream ss(response.substr(1));
      long left_ticks, right_ticks;
      if (ss >> left_ticks >> right_ticks) {
        
        // Convert ticks to radians
        double left_pos = (left_ticks / enc_ticks_per_rev_) * 2.0 * M_PI;
        double right_pos = (right_ticks / enc_ticks_per_rev_) * 2.0 * M_PI;
        
        // Compute velocity
        hw_velocities_[0] = (left_pos - hw_positions_[0]) / period.seconds();
        hw_velocities_[1] = (right_pos - hw_positions_[1]) / period.seconds();
        
        // Update positions
        hw_positions_[0] = left_pos;
        hw_positions_[1] = right_pos;
      }
    }
  }

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type EarendilSystemHardware::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  // hw_commands_ are in rad/s.
  // We send rad/s to the Arduino. The Arduino will convert rad/s to PWM via PID.
  send_motor_command(hw_commands_[0], hw_commands_[1]);
  return hardware_interface::return_type::OK;
}

bool EarendilSystemHardware::send_motor_command(double left_cmd, double right_cmd)
{
  if (serial_fd_ < 0) return false;
  
  // Protocol: "m <left_rad_s> <right_rad_s>\n"
  std::stringstream ss;
  ss << "m " << left_cmd << " " << right_cmd << "\n";
  std::string cmd = ss.str();
  
  int n = ::write(serial_fd_, cmd.c_str(), cmd.length());
  return n == (int)cmd.length();
}

}  // namespace earendil_hardware

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  earendil_hardware::EarendilSystemHardware, hardware_interface::SystemInterface)
