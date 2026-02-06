#ifndef DIFFDRIVE_ARDUINO_HARDWARE__DIFFDRIVE_ARDUINO_HARDWARE_HPP_
#define DIFFDRIVE_ARDUINO_HARDWARE__DIFFDRIVE_ARDUINO_HARDWARE_HPP_

#include <hardware_interface/system_interface.hpp>
#include <hardware_interface/types/hardware_interface_return_values.hpp>
#include <hardware_interface/types/hardware_interface_type_values.hpp>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/macros.hpp>
#include <rclcpp_lifecycle/state.hpp>

#include <memory>
#include <string>
#include <vector>

namespace chudovishe_hardware
{

class SerialPort;

class DiffDriveArduinoHardware : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(DiffDriveArduinoHardware)

  hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  hardware_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State & /*previous_state*/) override;
  hardware_interface::CallbackReturn on_cleanup(const rclcpp_lifecycle::State & /*previous_state*/) override;

  hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State & /*previous_state*/) override;
  hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & /*previous_state*/) override;

  hardware_interface::return_type read(const rclcpp::Time & /*time*/, const rclcpp::Duration & period) override;
  hardware_interface::return_type write(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/) override;

private:
  // Parameters (from <ros2_control><hardware><param ...>)
  std::string device_;
  int baud_rate_{57600};
  int timeout_ms_{50};
  int enc_counts_per_rev_{1123};
  bool reset_encoders_on_activate_{true};

  // 2 joints: left, right
  std::vector<std::string> joint_names_;

  // State
  std::vector<double> pos_;      // rad
  std::vector<double> vel_;      // rad/s
  std::vector<double> prev_pos_; // rad

  // Commands
  std::vector<double> cmd_vel_;  // rad/s

  rclcpp::Logger logger_{rclcpp::get_logger("chudovishe_hardware_interface")};
  std::unique_ptr<SerialPort> serial_;

  bool sendLine(const std::string & s);
  bool readLine(std::string & out);
};

}  // namespace diffdrive_arduino_hardware

#endif  // DIFFDRIVE_ARDUINO_HARDWARE__DIFFDRIVE_ARDUINO_HARDWARE_HPP_
