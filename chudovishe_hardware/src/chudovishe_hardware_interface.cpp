#include "chudovishe_hardware/chudovishe_hardware_interface.hpp"

#include <pluginlib/class_list_macros.hpp>

#include <cmath>
#include <cstdint>
#include <cstring>
#include <sstream>
#include <string>
#include <utility>

// POSIX serial
#include <fcntl.h>
#include <poll.h>
#include <termios.h>
#include <unistd.h>

namespace chudovishe_hardware
{

static constexpr double TWO_PI = 6.2831853071795864769;

class SerialPort
{
public:
  SerialPort() = default;
  ~SerialPort() { closePort(); }

  bool openPort(const std::string & device, int baud_rate, int timeout_ms, rclcpp::Logger logger)
  {
    logger_ = logger;
    timeout_ms_ = timeout_ms;

    fd_ = ::open(device.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (fd_ < 0) {
      RCLCPP_ERROR(logger_, "Failed to open serial device '%s': %s", device.c_str(), strerror(errno));
      return false;
    }

    termios tty{};
    if (tcgetattr(fd_, &tty) != 0) {
      RCLCPP_ERROR(logger_, "tcgetattr failed: %s", strerror(errno));
      closePort();
      return false;
    }

    // Raw mode
    cfmakeraw(&tty);

    // 8N1
    tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;
    tty.c_cflag |= (CLOCAL | CREAD);
    tty.c_cflag &= ~(PARENB | CSTOPB | CRTSCTS);

    // Non-blocking reads controlled via poll()
    tty.c_cc[VMIN]  = 0;
    tty.c_cc[VTIME] = 0;

    speed_t speed{};
    if (!baudToSpeed(baud_rate, speed)) {
      RCLCPP_ERROR(logger_, "Unsupported baud rate: %d", baud_rate);
      closePort();
      return false;
    }
    cfsetispeed(&tty, speed);
    cfsetospeed(&tty, speed);

    if (tcsetattr(fd_, TCSANOW, &tty) != 0) {
      RCLCPP_ERROR(logger_, "tcsetattr failed: %s", strerror(errno));
      closePort();
      return false;
    }

    tcflush(fd_, TCIOFLUSH);
    return true;
  }

  void closePort()
  {
    if (fd_ >= 0) {
      ::close(fd_);
      fd_ = -1;
    }
  }

  bool writeAll(const std::string & s)
  {
    if (fd_ < 0) return false;

    const char * data = s.c_str();
    size_t left = s.size();

    while (left > 0) {
      const ssize_t n = ::write(fd_, data, left);
      if (n < 0) {
        if (errno == EAGAIN || errno == EWOULDBLOCK) {
          // wait a bit
          if (!waitWritable()) return false;
          continue;
        }
        RCLCPP_ERROR(logger_, "Serial write failed: %s", strerror(errno));
        return false;
      }
      data += n;
      left -= static_cast<size_t>(n);
    }
    return true;
  }

  bool readLine(std::string & out)
  {
    out.clear();
    if (fd_ < 0) return false;

    // Read until '\n'
    while (true) {
      if (!waitReadable()) {
        RCLCPP_WARN(logger_, "Serial read timeout");
        return false;
      }

      char c{};
      const ssize_t n = ::read(fd_, &c, 1);
      if (n < 0) {
        if (errno == EAGAIN || errno == EWOULDBLOCK) continue;
        RCLCPP_ERROR(logger_, "Serial read failed: %s", strerror(errno));
        return false;
      }
      if (n == 0) continue;

      if (c == '\r') continue;
      if (c == '\n') return true;

      out.push_back(c);

      // safety cap
      if (out.size() > 256) {
        RCLCPP_ERROR(logger_, "Serial line too long");
        return false;
      }
    }
  }

private:
  bool baudToSpeed(int baud, speed_t & out)
  {
    switch (baud) {
      case 9600: out = B9600; return true;
      case 19200: out = B19200; return true;
      case 38400: out = B38400; return true;
      case 57600: out = B57600; return true;
      case 115200: out = B115200; return true;
      default: return false;
    }
  }

  bool waitReadable()
  {
    pollfd pfd{};
    pfd.fd = fd_;
    pfd.events = POLLIN;
    const int r = ::poll(&pfd, 1, timeout_ms_);
    return r > 0 && (pfd.revents & POLLIN);
  }

  bool waitWritable()
  {
    pollfd pfd{};
    pfd.fd = fd_;
    pfd.events = POLLOUT;
    const int r = ::poll(&pfd, 1, timeout_ms_);
    return r > 0 && (pfd.revents & POLLOUT);
  }

  int fd_{-1};
  int timeout_ms_{50};
  rclcpp::Logger logger_{rclcpp::get_logger("serial_port")};
};

hardware_interface::CallbackReturn DiffDriveArduinoHardware::on_init(
  const hardware_interface::HardwareInfo & info)
{
  RCLCPP_INFO(logger_, "On init");
  if (hardware_interface::SystemInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS) {
    return hardware_interface::CallbackReturn::ERROR;
  }

  // Expect exactly 2 joints (left, right)
  if (info_.joints.size() != 2) {
    RCLCPP_ERROR(logger_, "Expected exactly 2 joints, got %zu", info_.joints.size());
    return hardware_interface::CallbackReturn::ERROR;
  }

  joint_names_.resize(2);
  joint_names_[0] = info_.joints[0].name;
  joint_names_[1] = info_.joints[1].name;

  // Params
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
  serial_ = std::make_unique<SerialPort>();
  if (!serial_->openPort(device_, baud_rate_, timeout_ms_, logger_)) {
    return hardware_interface::CallbackReturn::ERROR;
  }

  // Optional: reset encoders immediately
  sendLine("r\n");
  prev_pos_[0] = prev_pos_[1] = 0.0;
  pos_[0] = pos_[1] = 0.0;
  vel_[0] = vel_[1] = 0.0;

  RCLCPP_INFO(logger_, "Configured. Serial=%s @ %d", device_.c_str(), baud_rate_);
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn DiffDriveArduinoHardware::on_cleanup(const rclcpp_lifecycle::State &)
{
  if (serial_) {
    serial_->closePort();
    serial_.reset();
  }
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn DiffDriveArduinoHardware::on_activate(const rclcpp_lifecycle::State &)
{
  // Stop motors
  cmd_vel_[0] = 0.0;
  cmd_vel_[1] = 0.0;
  write(rclcpp::Time(0), rclcpp::Duration(0, 0));

  if (reset_encoders_on_activate_) {
    sendLine("r\n");
    prev_pos_[0] = prev_pos_[1] = 0.0;
    pos_[0] = pos_[1] = 0.0;
    vel_[0] = vel_[1] = 0.0;
  }

  RCLCPP_INFO(logger_, "Activated");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn DiffDriveArduinoHardware::on_deactivate(const rclcpp_lifecycle::State &)
{
  // Stop motors
  cmd_vel_[0] = 0.0;
  cmd_vel_[1] = 0.0;
  write(rclcpp::Time(0), rclcpp::Duration(0, 0));

  RCLCPP_INFO(logger_, "Deactivated");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type DiffDriveArduinoHardware::read(const rclcpp::Time &, const rclcpp::Duration & period)
{
  // Query encoders: 'e' -> "left right"
  if (!sendLine("e\n")) return hardware_interface::return_type::ERROR;

  std::string line;
  if (!readLine(line)) return hardware_interface::return_type::ERROR;

  long left_counts = 0;
  long right_counts = 0;

  std::istringstream iss(line);
  if (!(iss >> left_counts >> right_counts)) {
    RCLCPP_ERROR(logger_, "Bad encoder line: '%s'", line.c_str());
    return hardware_interface::return_type::ERROR;
  }

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
  // diff_drive_controller writes wheel joint velocity commands (rad/s). :contentReference[oaicite:3]{index=3}
  auto safe = [&](double v) {
    if (!std::isfinite(v)) return 0.0;
    return v;
  };

  const double left_rad_s  = safe(cmd_vel_[0]);
  const double right_rad_s = safe(cmd_vel_[1]);

  // Convert rad/s -> ticks/s because Arduino firmware expects ticks per second for 'm'. :contentReference[oaicite:4]{index=4}
  const double ticks_per_rad = static_cast<double>(enc_counts_per_rev_) / TWO_PI;
  const long left_ticks_s  = lround(left_rad_s * ticks_per_rad);
  const long right_ticks_s = lround(right_rad_s * ticks_per_rad);

  std::ostringstream oss;
  oss << "m " << left_ticks_s << " " << right_ticks_s << "\n";
  if (!sendLine(oss.str())) return hardware_interface::return_type::ERROR;

  return hardware_interface::return_type::OK;
}

bool DiffDriveArduinoHardware::sendLine(const std::string & s)
{
  if (!serial_) return false;
  return serial_->writeAll(s);
}

bool DiffDriveArduinoHardware::readLine(std::string & out)
{
  if (!serial_) return false;
  return serial_->readLine(out);
}

}  // namespace diffdrive_arduino_hardware

PLUGINLIB_EXPORT_CLASS(
  chudovishe_hardware::DiffDriveArduinoHardware,
  hardware_interface::SystemInterface)
