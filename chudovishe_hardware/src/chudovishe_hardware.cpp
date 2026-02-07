#include "chudovishe_hardware/chudovishe_hardware.hpp"
#include "chudovishe_hardware/arduino_comms.hpp"
#include <pluginlib/class_list_macros.hpp>

#include <cmath>
#include <cstring>
#include <sstream>
#include <string>

// POSIX serial
#include <fcntl.h>
#include <poll.h>
#include <termios.h>
#include <unistd.h>

namespace chudovishe_hardware
{

static constexpr double TWO_PI = 6.2831853071795864769;

hardware_interface::CallbackReturn DiffDriveArduinoHardware::on_init(const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS) {
    return hardware_interface::CallbackReturn::ERROR;
  }

  if (info_.joints.size() != 2) {
    RCLCPP_ERROR(logger_, "Expected exactly 2 joints, got %zu", info_.joints.size());
    return hardware_interface::CallbackReturn::ERROR;
  }

  joint_names_.resize(2);
  joint_names_[0] = info_.joints[0].name;
  joint_names_[1] = info_.joints[1].name;

  auto getp = [&](const std::string & key, const std::string & def) {
    auto it = info_.hardware_parameters.find(key);
    return (it == info_.hardware_parameters.end()) ? def : it->second;
  };

  device_ = getp("device", "/dev/ttyACM0");
  baud_rate_ = std::stoi(getp("baud_rate", "57600"));
  timeout_ms_ = std::stoi(getp("timeout_ms", "50"));
  enc_counts_per_rev_ = std::stoi(getp("enc_counts_per_rev", "1123"));
  reset_encoders_on_activate_ = (getp("reset_encoders_on_activate", "true") == "true");

  pos_.assign(2, 0.0);
  vel_.assign(2, 0.0);
  prev_pos_.assign(2, 0.0);
  cmd_vel_.assign(2, 0.0);

  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> DiffDriveArduinoHardware::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> states;
  states.reserve(4);
  for (size_t i = 0; i < 2; ++i) {
    states.emplace_back(joint_names_[i], hardware_interface::HW_IF_POSITION, &pos_[i]);
    states.emplace_back(joint_names_[i], hardware_interface::HW_IF_VELOCITY, &vel_[i]);
  }
  return states;
}

std::vector<hardware_interface::CommandInterface> DiffDriveArduinoHardware::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> cmds;
  cmds.reserve(2);
  for (size_t i = 0; i < 2; ++i) {
    cmds.emplace_back(joint_names_[i], hardware_interface::HW_IF_VELOCITY, &cmd_vel_[i]);
  }
  return cmds;
}

hardware_interface::CallbackReturn DiffDriveArduinoHardware::on_configure(const rclcpp_lifecycle::State &)
{
  serial_ = std::make_unique<ArduinoComms>();
  // if (!) {
  //   return hardware_interface::CallbackReturn::ERROR;
  // }
  serial_->setup(device_, baud_rate_, timeout_ms_);
  // optional reset
  serial_->sendMsg("r\n");
  prev_pos_.assign(2, 0.0);
  pos_.assign(2, 0.0);
  vel_.assign(2, 0.0);

  RCLCPP_INFO(logger_, "Configured. Serial=%s @ %d", device_.c_str(), baud_rate_);
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn DiffDriveArduinoHardware::on_cleanup(const rclcpp_lifecycle::State &)
{
  if (serial_) {
    serial_.reset();
  }
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn DiffDriveArduinoHardware::on_activate(const rclcpp_lifecycle::State &)
{
  cmd_vel_[0] = 0.0;
  cmd_vel_[1] = 0.0;
  write(rclcpp::Time(0), rclcpp::Duration(0, 0));

  if (reset_encoders_on_activate_) {
    sendLine("r\n");
    prev_pos_.assign(2, 0.0);
    pos_.assign(2, 0.0);
    vel_.assign(2, 0.0);
  }

  RCLCPP_INFO(logger_, "Activated");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn DiffDriveArduinoHardware::on_deactivate(const rclcpp_lifecycle::State &)
{
  cmd_vel_[0] = 0.0;
  cmd_vel_[1] = 0.0;
  write(rclcpp::Time(0), rclcpp::Duration(0, 0));
  RCLCPP_INFO(logger_, "Deactivated");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type DiffDriveArduinoHardware::read(
  const rclcpp::Time &, const rclcpp::Duration & period)
{

  int left_counts = 0, right_counts = 0;
  serial_->readEncoderValues(left_counts, right_counts);

  const double rad_per_count = TWO_PI / static_cast<double>(enc_counts_per_rev_);
  pos_[0] = static_cast<double>(left_counts) * rad_per_count;
  pos_[1] = static_cast<double>(right_counts) * rad_per_count;

  const double dt = period.seconds();
  if (dt > 0.0) {
    vel_[0] = (pos_[0] - prev_pos_[0]) / dt;
    vel_[1] = (pos_[1] - prev_pos_[1]) / dt;
  } else {
    vel_[0] = vel_[1] = 0.0;
  }

  prev_pos_ = pos_;
  return hardware_interface::return_type::OK;
}


hardware_interface::return_type DiffDriveArduinoHardware::write(const rclcpp::Time &, const rclcpp::Duration &)
{
  auto safe = [&](double v) { return std::isfinite(v) ? v : 0.0; };

  const double left_rad_s  = safe(cmd_vel_[0]);
  const double right_rad_s = safe(cmd_vel_[1]);

  RCLCPP_INFO(logger_, "Configured. Serial=%f %f", cmd_vel_[0], cmd_vel_[1]);

  // firmware expects ticks/sec for 'm' :contentReference[oaicite:4]{index=4}
  const double ticks_per_rad = static_cast<double>(enc_counts_per_rev_) / TWO_PI;
  const long left_ticks_s  = lround(left_rad_s * ticks_per_rad);
  const long right_ticks_s = lround(right_rad_s * ticks_per_rad);

  std::string command = "m " + std::to_string(left_ticks_s) + " " + std::to_string(right_ticks_s) + "\n";
  serial_->sendMsg(command);

  return hardware_interface::return_type::OK;
}

}  // namespace chudovishe_hardware

PLUGINLIB_EXPORT_CLASS(chudovishe_hardware::DiffDriveArduinoHardware, hardware_interface::SystemInterface)
