#include "chudovishe_hardware/arduino_comms.hpp"

#include <rclcpp/rclcpp.hpp>

#include <cerrno>
#include <chrono>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <fcntl.h>
#include <sstream>
#include <stdexcept>
#include <string>
#include <sys/select.h>
#include <termios.h>
#include <unistd.h>

namespace
{
// Keep the read timeout configured in setup() (ms).
static int32_t g_timeout_ms = 0;

[[nodiscard]] speed_t speedFromBaud(const int32_t baud)
{
  // Guard each macro so this compiles on more POSIX platforms.
  switch (baud)
  {
#ifdef B50
    case 50: return B50;
#endif
#ifdef B75
    case 75: return B75;
#endif
#ifdef B110
    case 110: return B110;
#endif
#ifdef B134
    case 134: return B134;
#endif
#ifdef B150
    case 150: return B150;
#endif
#ifdef B200
    case 200: return B200;
#endif
#ifdef B300
    case 300: return B300;
#endif
#ifdef B600
    case 600: return B600;
#endif
#ifdef B1200
    case 1200: return B1200;
#endif
#ifdef B1800
    case 1800: return B1800;
#endif
#ifdef B2400
    case 2400: return B2400;
#endif
#ifdef B4800
    case 4800: return B4800;
#endif
#ifdef B9600
    case 9600: return B9600;
#endif
#ifdef B19200
    case 19200: return B19200;
#endif
#ifdef B38400
    case 38400: return B38400;
#endif
#ifdef B57600
    case 57600: return B57600;
#endif
#ifdef B115200
    case 115200: return B115200;
#endif
#ifdef B230400
    case 230400: return B230400;
#endif
#ifdef B460800
    case 460800: return B460800;
#endif
#ifdef B500000
    case 500000: return B500000;
#endif
#ifdef B576000
    case 576000: return B576000;
#endif
#ifdef B921600
    case 921600: return B921600;
#endif
#ifdef B1000000
    case 1000000: return B1000000;
#endif
#ifdef B1152000
    case 1152000: return B1152000;
#endif
#ifdef B1500000
    case 1500000: return B1500000;
#endif
#ifdef B2000000
    case 2000000: return B2000000;
#endif
#ifdef B2500000
    case 2500000: return B2500000;
#endif
#ifdef B3000000
    case 3000000: return B3000000;
#endif
#ifdef B3500000
    case 3500000: return B3500000;
#endif
#ifdef B4000000
    case 4000000: return B4000000;
#endif
    default:
      throw std::runtime_error("Unsupported baud rate: " + std::to_string(baud));
  }
}

[[nodiscard]] std::string trimCRLF(std::string s)
{
  while (!s.empty() && (s.back() == '\n' || s.back() == '\r'))
  {
    s.pop_back();
  }
  return s;
}

inline void closeFd(int &fd) noexcept
{
  if (fd >= 0)
  {
    ::close(fd);
    fd = -1;
  }
}

void writeAll(const int fd, const char *data, size_t len)
{
  while (len > 0)
  {
    const ssize_t n = ::write(fd, data, len);
    if (n < 0)
    {
      if (errno == EINTR)
      {
        continue;
      }
      throw std::runtime_error(std::string("write() failed: ") + std::strerror(errno));
    }
    data += static_cast<size_t>(n);
    len -= static_cast<size_t>(n);
  }
}

[[nodiscard]] bool readLineWithTimeout(
  const int fd,
  std::string &out,
  const int32_t timeout_ms,
  const char delimiter = '\n',
  const size_t max_len = 4096)
{
  out.clear();
  using clock = std::chrono::steady_clock;

  // timeout_ms <= 0 => wait indefinitely (чтобы не ломать поведение "0 = без таймаута")
  const bool infinite = (timeout_ms <= 0);
  const auto start = clock::now();

  while (out.size() < max_len)
  {
    int32_t remaining_ms = -1;
    if (!infinite)
    {
      const auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(clock::now() - start).count();
      remaining_ms = timeout_ms - static_cast<int32_t>(elapsed);
      if (remaining_ms <= 0)
      {
        return false;  // timeout
      }
    }

    fd_set rfds;
    FD_ZERO(&rfds);
    FD_SET(fd, &rfds);

    timeval tv;
    timeval *tv_ptr = nullptr;
    if (!infinite)
    {
      tv.tv_sec = remaining_ms / 1000;
      tv.tv_usec = (remaining_ms % 1000) * 1000;
      tv_ptr = &tv;
    }

    const int rv = ::select(fd + 1, &rfds, nullptr, nullptr, tv_ptr);
    if (rv == 0)
    {
      return false;  // timeout
    }
    if (rv < 0)
    {
      if (errno == EINTR)
      {
        continue;
      }
      throw std::runtime_error(std::string("select() failed: ") + std::strerror(errno));
    }

    char ch = 0;
    const ssize_t n = ::read(fd, &ch, 1);
    if (n < 0)
    {
      if (errno == EINTR)
      {
        continue;
      }
      throw std::runtime_error(std::string("read() failed: ") + std::strerror(errno));
    }
    if (n == 0)
    {
      // В режиме VMIN=0/VTIME=0 возможны пустые чтения.
      continue;
    }

    if (ch == delimiter)
    {
      return true;
    }

    out.push_back(ch);
  }

  // Line too long: return what we have.
  return true;
}

}  // namespace

ArduinoComms::~ArduinoComms()
{
  closeFd(fd_);
}

ArduinoComms::ArduinoComms(ArduinoComms &&other) noexcept
: fd_(other.fd_)
{
  other.fd_ = -1;
}

ArduinoComms &ArduinoComms::operator=(ArduinoComms &&other) noexcept
{
  if (this != &other)
  {
    closeFd(fd_);
    fd_ = other.fd_;
    other.fd_ = -1;
  }
  return *this;
}

void ArduinoComms::setup(const std::string &serial_device, int32_t baud_rate, int32_t timeout_ms)
{
  g_timeout_ms = timeout_ms;

  closeFd(fd_);

  fd_ = ::open(serial_device.c_str(), O_RDWR | O_NOCTTY | O_SYNC);
  if (fd_ < 0)
  {
    throw std::runtime_error(std::string("open() failed for ") + serial_device + ": " + std::strerror(errno));
  }

  termios tty{};
  if (::tcgetattr(fd_, &tty) != 0)
  {
    const int saved_errno = errno;
    closeFd(fd_);
    throw std::runtime_error(std::string("tcgetattr() failed: ") + std::strerror(saved_errno));
  }

  ::cfmakeraw(&tty);

  // 8N1, no flow control.
  tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;
  tty.c_cflag |= (CLOCAL | CREAD);
  tty.c_cflag &= ~(PARENB | PARODD);
  tty.c_cflag &= ~CSTOPB;
#ifdef CRTSCTS
  tty.c_cflag &= ~CRTSCTS;
#endif

  tty.c_iflag &= ~(IXON | IXOFF | IXANY);

  // Reads: non-blocking at termios level; we do timeout via select()
  tty.c_cc[VMIN] = 0;
  tty.c_cc[VTIME] = 0;

  const speed_t speed = speedFromBaud(baud_rate);
  if (::cfsetispeed(&tty, speed) != 0 || ::cfsetospeed(&tty, speed) != 0)
  {
    const int saved_errno = errno;
    closeFd(fd_);
    throw std::runtime_error(std::string("cfset*speed() failed: ") + std::strerror(saved_errno));
  }

  if (::tcsetattr(fd_, TCSANOW, &tty) != 0)
  {
    const int saved_errno = errno;
    closeFd(fd_);
    throw std::runtime_error(std::string("tcsetattr() failed: ") + std::strerror(saved_errno));
  }

  (void)::tcflush(fd_, TCIOFLUSH);
}

void ArduinoComms::sendEmptyMsg()
{
  (void)sendMsg("\r");
}

void ArduinoComms::readEncoderValues(int &val_1, int &val_2)
{
  const std::string response = sendMsg("e\r");

  const std::string delimiter = " ";
  const size_t del_pos = response.find(delimiter);
  if (del_pos == std::string::npos)
  {
    return;
  }

  const std::string token_1 = response.substr(0, del_pos);
  const std::string token_2 = response.substr(del_pos + delimiter.length());

  val_1 = std::atoi(token_1.c_str());
  val_2 = std::atoi(token_2.c_str());
}

void ArduinoComms::setMotorValues(int val_1, int val_2)
{
  std::stringstream ss;
  ss << "m " << val_1 << " " << val_2 << "\r";
  (void)sendMsg(ss.str(), false);
}

void ArduinoComms::setPidValues(float k_p, float k_d, float k_i, float k_o)
{
  std::stringstream ss;
  ss << "u " << k_p << ":" << k_d << ":" << k_i << ":" << k_o << "\r";
  (void)sendMsg(ss.str());
}

std::string ArduinoComms::sendMsg(const std::string &msg_to_send, bool print_output)
{
  if (fd_ < 0)
  {
    throw std::runtime_error("Serial port is not open");
  }

  std::string response;

  try
  {
    writeAll(fd_, msg_to_send.data(), msg_to_send.size());
    (void)::tcdrain(fd_);

    const bool got_line = readLineWithTimeout(fd_, response, g_timeout_ms, '\n');
    if (!got_line)
    {
      response.clear();
    }
  }
  catch (const std::exception &)
  {
    response.clear();
  }

  response = trimCRLF(response);

  if (print_output)
  {
    // RCLCPP_INFO_STREAM(rclcpp::get_logger("arduino_comms"), "Sent: " << msg_to_send);
    // RCLCPP_INFO_STREAM(rclcpp::get_logger("arduino_comms"), "Received: " << response);
  }

  return response;
}
