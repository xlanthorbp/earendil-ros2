#ifndef EARENDIL_HARDWARE__EARENDIL_SYSTEM_HPP_
#define EARENDIL_HARDWARE__EARENDIL_SYSTEM_HPP_

#include <memory>
#include <string>
#include <vector>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/macros.hpp"

namespace earendil_hardware
{
class EarendilSystemHardware : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(EarendilSystemHardware)

  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareInfo & info) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  // Serial communication details
  int serial_fd_ = -1;
  std::string port_;
  int baud_rate_;
  int timeout_ms_;

  bool connect_serial();
  void disconnect_serial();
  bool send_motor_command(double left_cmd, double right_cmd);
  bool read_encoder_values(double & left_pos, double & right_pos);

  // Hardware states & commands
  std::vector<double> hw_commands_; // [0] = left wheel rad/s, [1] = right wheel rad/s
  std::vector<double> hw_positions_; // [0] = left wheel rad, [1] = right wheel rad
  std::vector<double> hw_velocities_; // [0] = left wheel rad/s, [1] = right wheel rad/s

  double enc_ticks_per_rev_;
};

}  // namespace earendil_hardware

#endif  // EARENDIL_HARDWARE__EARENDIL_SYSTEM_HPP_
