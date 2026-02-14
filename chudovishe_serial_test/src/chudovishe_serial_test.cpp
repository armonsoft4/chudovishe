#include <rclcpp/rclcpp.hpp>
#include <serial/serial.h>

#include <chrono>
#include <mutex>
#include <string>
#include <algorithm>
#include <thread>

using namespace std::chrono_literals;

class SerialNode : public rclcpp::Node
{
public:
  SerialNode() : Node("chudovishe_serial_test")
  {
    device_ = declare_parameter<std::string>("device", "/dev/ttyACM0");
    baud_   = declare_parameter<int>("baud", 57600);
    timeout_ms_ = declare_parameter<int>("timeout_ms", 100);
    send_period_ms_ = declare_parameter<int>("send_period_ms", 100);   // как часто слать "1"
    arduino_boot_ms_ = declare_parameter<int>("arduino_boot_ms", 1500); // пауза после открытия порта

    openPort();

    timer_ = create_wall_timer(
      std::chrono::milliseconds(send_period_ms_),
      std::bind(&SerialNode::tick, this)
    );

    RCLCPP_INFO(get_logger(), "Serial spammer started: %s @ %d, period=%dms",
                device_.c_str(), baud_, send_period_ms_);
  }

  ~SerialNode() override
  {
    std::lock_guard<std::mutex> lk(mutex_);
    if (serial_.isOpen()) {
      try { serial_.close(); } catch (...) {}
    }
  }

private:
  void openPort()
  {
    std::lock_guard<std::mutex> lk(mutex_);
    try {
      if (serial_.isOpen()) serial_.close();

      serial_.setPort(device_);
      serial_.setBaudrate(static_cast<uint32_t>(baud_));

      // ВАЖНО: в твоей версии setTimeout принимает Timeout& => timeout_ должен жить как member
      timeout_ = serial::Timeout::simpleTimeout(static_cast<uint32_t>(timeout_ms_));
      serial_.setTimeout(timeout_);

      serial_.open();

      // Arduino часто ресетится при открытии порта — подождём, чтобы не потерять первые данные
      std::this_thread::sleep_for(std::chrono::milliseconds(arduino_boot_ms_));

      rx_buffer_.clear();
      RCLCPP_INFO(get_logger(), "Opened port %s", device_.c_str());
    }
    catch (const std::exception &e) {
      RCLCPP_WARN(get_logger(), "Open failed (%s): %s", device_.c_str(), e.what());
    }
  }

  void tick()
  {
    std::lock_guard<std::mutex> lk(mutex_);

    // если порт не открыт — пробуем открыть (прямо тут)
    if (!serial_.isOpen()) {
      // отпустим lock? (не обязательно, но ок)
      // здесь проще: закрыть/открыть под lock
      // (если хочешь идеально — вынести openPort без lock, но для теста не нужно)
      try { serial_.close(); } catch (...) {}
      // открываем повторно
      // (чтобы не рекурсить — просто напрямую)
      try {
        serial_.setPort(device_);
        serial_.setBaudrate(static_cast<uint32_t>(baud_));
        timeout_ = serial::Timeout::simpleTimeout(static_cast<uint32_t>(timeout_ms_));
        serial_.setTimeout(timeout_);
        serial_.open();
        std::this_thread::sleep_for(std::chrono::milliseconds(arduino_boot_ms_));
        rx_buffer_.clear();
        RCLCPP_INFO(get_logger(), "Re-opened port %s", device_.c_str());
      } catch (const std::exception &e) {
        RCLCPP_WARN(get_logger(), "Re-open failed: %s", e.what());
        return;
      }
    }

    // 1) ПОСТОЯННО отправляем "1\n"
    try {
      serial_.write(std::string("1\n"));
      // Можно убрать, если спамит лог:
      // RCLCPP_INFO(get_logger(), "TX: 1");
    }
    catch (const std::exception &e) {
      RCLCPP_WARN(get_logger(), "Write error: %s", e.what());
      try { serial_.close(); } catch (...) {}
      return;
    }

    // 2) ЧИТАЕМ всё, что пришло, и выводим только готовые строки до '\n'
    try {
      const size_t avail = serial_.available();
      if (avail > 0) {
        std::string chunk = serial_.read(avail);
        rx_buffer_ += chunk;

        size_t pos;
        while ((pos = rx_buffer_.find('\n')) != std::string::npos) {
          std::string line = rx_buffer_.substr(0, pos);
          rx_buffer_.erase(0, pos + 1);

          if (!line.empty() && line.back() == '\r') line.pop_back();
          if (!line.empty()) {
            RCLCPP_INFO(get_logger(), "RX: %s", line.c_str());
          }
        }

        // защита от бесконечного роста буфера (если нет '\n')
        if (rx_buffer_.size() > 8192) {
          rx_buffer_.erase(0, rx_buffer_.size() - 4096);
        }
      }
    }
    catch (const std::exception &e) {
      RCLCPP_WARN(get_logger(), "Read error: %s", e.what());
      try { serial_.close(); } catch (...) {}
      return;
    }
  }

private:
  std::string device_;
  int baud_;
  int timeout_ms_;
  int send_period_ms_;
  int arduino_boot_ms_;

  serial::Serial serial_;
  serial::Timeout timeout_;
  std::mutex mutex_;

  std::string rx_buffer_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SerialNode>());
  rclcpp::shutdown();
  return 0;
}
