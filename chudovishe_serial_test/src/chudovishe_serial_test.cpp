#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

#include <serial/serial.h>

#include <atomic>
#include <chrono>
#include <mutex>
#include <thread>
#include <string>
#include <algorithm>

class SerialNode : public rclcpp::Node
{
public:
  SerialNode()
  : Node("chudovishe_serial_test"),
    running_(true)
  {
    port_ = declare_parameter<std::string>("port", "/dev/ttyACM0");
    baud_ = declare_parameter<int>("baud", 57600);
    timeout_ms_ = declare_parameter<int>("timeout_ms", 100);
    reconnect_ms_ = declare_parameter<int>("reconnect_ms", 1000);
    chunk_size_ = declare_parameter<int>("chunk_size", 256);
    add_newline_on_tx_ = declare_parameter<bool>("add_newline_on_tx", true);
    max_buffer_ = declare_parameter<int>("max_buffer", 8192);
    log_rx_ = declare_parameter<bool>("log_rx", true);

    rx_pub_ = create_publisher<std_msgs::msg::String>("serial_rx", 10);

    tx_sub_ = create_subscription<std_msgs::msg::String>(
      "serial_tx",
      10,
      std::bind(&SerialNode::onTx, this, std::placeholders::_1)
    );

    openSerial();  // первичное открытие

    reader_thread_ = std::thread([this]() { readLoop(); });

    RCLCPP_INFO(get_logger(),
                "Started. port=%s baud=%d timeout_ms=%d",
                port_.c_str(), baud_, timeout_ms_);
  }

  ~SerialNode() override
  {
    running_.store(false);
    if (reader_thread_.joinable()) {
      reader_thread_.join();
    }

    std::lock_guard<std::mutex> lock(serial_mutex_);
    if (serial_.isOpen()) {
      serial_.close();
    }
  }

private:
  void openSerial()
  {
    std::lock_guard<std::mutex> lock(serial_mutex_);

    try {
      if (serial_.isOpen()) {
        serial_.close();
      }

      serial_.setPort(port_);
      serial_.setBaudrate(static_cast<uint32_t>(baud_));

      // ВАЖНО: в твоей версии serial::Serial::setTimeout принимает Timeout& (не rvalue)
      timeout_ = serial::Timeout::simpleTimeout(static_cast<uint32_t>(timeout_ms_));
      serial_.setTimeout(timeout_);

      serial_.open();

      // Arduino часто ресетится при открытии порта — подождём, чтобы не терять первые байты
      std::this_thread::sleep_for(std::chrono::milliseconds(1500));

      rx_buffer_.clear();

      RCLCPP_INFO(get_logger(), "Serial opened: %s @ %d", port_.c_str(), baud_);
    }
    catch (const std::exception &e) {
      RCLCPP_WARN(get_logger(), "Open serial failed (%s): %s", port_.c_str(), e.what());
    }
  }

  void closeSerialLocked()
  {
    // вызывать ТОЛЬКО когда serial_mutex_ уже захвачен
    try {
      if (serial_.isOpen()) {
        serial_.close();
      }
    } catch (...) {
      // ignore
    }
  }

  void onTx(const std_msgs::msg::String::SharedPtr msg)
  {
    std::string out = msg->data;
    if (add_newline_on_tx_) {
      if (out.empty() || out.back() != '\n') out.push_back('\n');
    }

    std::lock_guard<std::mutex> lock(serial_mutex_);
    if (!serial_.isOpen()) {
      RCLCPP_WARN(get_logger(), "TX: serial not open");
      return;
    }

    try {
      serial_.write(reinterpret_cast<const uint8_t*>(out.data()), out.size());
    }
    catch (const std::exception &e) {
      RCLCPP_WARN(get_logger(), "TX error: %s", e.what());
      closeSerialLocked();
    }
  }

  void publishLine(const std::string &line)
  {
    if (log_rx_) {
      RCLCPP_INFO(get_logger(), "RX: %s", line.c_str());
    }
    std_msgs::msg::String m;
    m.data = line;
    rx_pub_->publish(m);
  }

  void processBufferLines()
  {
    // rx_buffer_ уже содержит данные, ищем '\n'
    for (;;) {
      auto pos = rx_buffer_.find('\n');
      if (pos == std::string::npos) break;

      std::string line = rx_buffer_.substr(0, pos);
      rx_buffer_.erase(0, pos + 1);

      // поддержка \r\n
      if (!line.empty() && line.back() == '\r') {
        line.pop_back();
      }

      // пустые строки можно игнорировать
      if (!line.empty()) {
        publishLine(line);
      }
    }

    // защита от раздувания, если нет '\n'
    if (rx_buffer_.size() > static_cast<size_t>(max_buffer_)) {
      rx_buffer_.erase(0, rx_buffer_.size() - static_cast<size_t>(max_buffer_));
    }
  }

  void readLoop()
  {
    while (rclcpp::ok() && running_.load()) {

      bool need_reopen = false;

      {
        std::lock_guard<std::mutex> lock(serial_mutex_);

        if (!serial_.isOpen()) {
          need_reopen = true;
        } else {
          try {
            const size_t avail = serial_.available();
            if (avail > 0) {
              const size_t to_read = std::min<size_t>(avail, static_cast<size_t>(chunk_size_));

              std::string chunk;
              chunk.resize(to_read);

              const size_t n = serial_.read(reinterpret_cast<uint8_t*>(&chunk[0]), to_read);
              chunk.resize(n);

              rx_buffer_ += chunk;
              processBufferLines();
            }
          }
          catch (const std::exception &e) {
            RCLCPP_WARN(get_logger(), "RX error: %s", e.what());
            closeSerialLocked();
            need_reopen = true;
          }
        }
      }

      if (need_reopen) {
        std::this_thread::sleep_for(std::chrono::milliseconds(reconnect_ms_));
        openSerial();
      } else {
        // небольшой сон, чтобы не грузить CPU
        std::this_thread::sleep_for(std::chrono::milliseconds(2));
      }
    }
  }

private:
  // ROS
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr rx_pub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr tx_sub_;

  // Serial
  serial::Serial serial_;
  serial::Timeout timeout_;   // ВАЖНО: должен жить как lvalue
  std::mutex serial_mutex_;

  // Reader thread
  std::thread reader_thread_;
  std::atomic<bool> running_;

  // Buffering
  std::string rx_buffer_;

  // Params
  std::string port_;
  int baud_;
  int timeout_ms_;
  int reconnect_ms_;
  int chunk_size_;
  bool add_newline_on_tx_;
  int max_buffer_;
  bool log_rx_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SerialNode>());
  rclcpp::shutdown();
  return 0;
}
