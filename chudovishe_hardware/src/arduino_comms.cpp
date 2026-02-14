#include "chudovishe_hardware/arduino_comms.hpp"

#include <rclcpp/rclcpp.hpp>

#include <serial/serial.h>

#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <memory>
#include <mutex>
#include <sstream>
#include <string>
#include <thread>
#include <unordered_map>

using namespace std::chrono_literals;

namespace
{

struct InstanceState
{
  std::mutex mtx;

  std::string device{"/dev/ttyACM0"};
  int32_t baud{57600};
  int32_t timeout_ms{100};
  int32_t arduino_boot_ms{1500};

  // Важно: setTimeout(timeout) требует lvalue => timeout должен жить долго
  serial::Timeout timeout_obj;

  // RX буфер как в chudovishe_serial_test.cpp
  std::string rx_buffer;
};

// Храним состояние по this (чтобы не менять header)
static std::mutex g_map_mtx;
static std::unordered_map<const ArduinoComms *, std::shared_ptr<InstanceState>> g_states;

static std::shared_ptr<InstanceState> get_state(const ArduinoComms *self)
{
  std::lock_guard<std::mutex> lk(g_map_mtx);
  auto &ptr = g_states[self];
  if (!ptr) ptr = std::make_shared<InstanceState>();
  return ptr;
}

static inline void trim_cr(std::string &s)
{
  if (!s.empty() && s.back() == '\r') s.pop_back();
}

static bool ensure_open(serial::Serial &ser, const std::shared_ptr<InstanceState> &st, rclcpp::Logger logger)
{
  try {
    if (ser.isOpen()) return true;

    ser.setPort(st->device);
    ser.setBaudrate(static_cast<uint32_t>(st->baud));

    st->timeout_obj = serial::Timeout::simpleTimeout(static_cast<uint32_t>(st->timeout_ms));
    ser.setTimeout(st->timeout_obj);

    ser.open();

    // Arduino Mega часто ресетится при открытии порта
    std::this_thread::sleep_for(std::chrono::milliseconds(st->arduino_boot_ms));
    st->rx_buffer.clear();

    RCLCPP_INFO(logger, "ArduinoComms: opened %s @ %d", st->device.c_str(), st->baud);
    return true;
  }
  catch (const std::exception &e) {
    RCLCPP_WARN(logger, "ArduinoComms: open failed (%s): %s", st->device.c_str(), e.what());
    try { ser.close(); } catch (...) {}
    return false;
  }
}

static bool pump_lines(serial::Serial &ser, const std::shared_ptr<InstanceState> &st, std::string &out_line)
{
  const size_t avail = ser.available();
  if (avail == 0) return false;

  std::string chunk = ser.read(avail);
  st->rx_buffer += chunk;

  bool got_any = false;
  size_t pos = 0;
  while ((pos = st->rx_buffer.find('\n')) != std::string::npos) {
    std::string line = st->rx_buffer.substr(0, pos);
    st->rx_buffer.erase(0, pos + 1);

    trim_cr(line);
    if (!line.empty()) {
      out_line = line;   // берём последнюю готовую строку
      got_any = true;
    }
  }

  // защита от бесконечного роста
  if (st->rx_buffer.size() > 8192) {
    st->rx_buffer.erase(0, st->rx_buffer.size() - 4096);
  }

  return got_any;
}

static bool wait_two_ints_line(
  serial::Serial &ser,
  const std::shared_ptr<InstanceState> &st,
  int &a, int &b,
  rclcpp::Logger logger,
  int32_t timeout_ms)
{
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(timeout_ms);
  std::string line;

  while (std::chrono::steady_clock::now() < deadline) {
    try {
      if (pump_lines(ser, st, line)) {
        long la = 0, lb = 0;
        if (std::sscanf(line.c_str(), "%ld %ld", &la, &lb) == 2) {
          a = static_cast<int>(la);
          b = static_cast<int>(lb);
          return true;
        }
        // если это не "два числа" — просто ждём следующую строку
      } else {
        std::this_thread::sleep_for(2ms);
      }
    }
    catch (const std::exception &e) {
      RCLCPP_WARN(logger, "ArduinoComms: read error: %s", e.what());
      try { ser.close(); } catch (...) {}
      return false;
    }
  }

  return false;
}

} // namespace

// ================== ArduinoComms ==================

void ArduinoComms::setup(const std::string &serial_device, int32_t baud_rate, int32_t timeout_ms)
{
  logger_ = rclcpp::get_logger("chudovishe_serial");

  auto st = get_state(this);
  {
    std::lock_guard<std::mutex> lk(st->mtx);
    st->device = serial_device;
    st->baud = baud_rate;
    st->timeout_ms = timeout_ms;
    // st->arduino_boot_ms можно оставить 1500 (как в serial_test)
  }

  // Откроем порт сразу
  std::lock_guard<std::mutex> lk(st->mtx);
  (void)ensure_open(serial_conn_, st, logger_);
}

void ArduinoComms::sendEmptyMsg()
{
  auto st = get_state(this);
  std::lock_guard<std::mutex> lk(st->mtx);

  if (!ensure_open(serial_conn_, st, logger_)) return;

  try {
    serial_conn_.write(std::string("\n"));
  } catch (const std::exception &e) {
    RCLCPP_WARN(logger_, "ArduinoComms: write error: %s", e.what());
    try { serial_conn_.close(); } catch (...) {}
  }
}

void ArduinoComms::setMotorValues(long val_1, long val_2)
{
  auto st = get_state(this);
  std::lock_guard<std::mutex> lk(st->mtx);

  if (!ensure_open(serial_conn_, st, logger_)) return;

  std::stringstream ss;
  // как в прошивке: "m L R\n"
  ss << "m " << val_1 << " " << val_2 << "\n";

  try {
    serial_conn_.write(ss.str());
    // НЕ ждём ответ (как и в serial_test: просто TX)
  }
  catch (const std::exception &e) {
    RCLCPP_WARN(logger_, "ArduinoComms: write motor cmd error: %s", e.what());
    try { serial_conn_.close(); } catch (...) {}
  }
}

void ArduinoComms::readEncoderValues(int &val_1, int &val_2)
{
  // По твоей прошивке: запрос "1\n" -> ответ "<L> <R>\n" (скорости в тиках/сек)
  auto st = get_state(this);
  std::lock_guard<std::mutex> lk(st->mtx);

  if (!ensure_open(serial_conn_, st, logger_)) {
    val_1 = 0;
    val_2 = 0;
    return;
  }

  try {
    serial_conn_.write(std::string("1\n"));
  }
  catch (const std::exception &e) {
    RCLCPP_WARN(logger_, "ArduinoComms: write request error: %s", e.what());
    try { serial_conn_.close(); } catch (...) {}
    val_1 = 0;
    val_2 = 0;
    return;
  }

  int a = 0, b = 0;
  const bool ok = wait_two_ints_line(serial_conn_, st, a, b, logger_, st->timeout_ms);

  if (ok) {
    val_1 = a;
    val_2 = b;
  } else {
    // если ответа нет — отдаём нули (без падения)
    val_1 = 0;
    val_2 = 0;
  }
}

void ArduinoComms::setPidValues(float k_p, float k_d, float k_i, float k_o)
{
  // PID ты не используешь — оставляем заглушку, чтобы не ломать интерфейс
  (void)k_p; (void)k_d; (void)k_i; (void)k_o;
}

std::string ArduinoComms::sendMsg(const std::string &msg_to_send, bool print_output)
{
  auto st = get_state(this);
  std::lock_guard<std::mutex> lk(st->mtx);

  if (!ensure_open(serial_conn_, st, logger_)) return std::string();

  try {
    serial_conn_.write(msg_to_send);
  }
  catch (const std::exception &e) {
    RCLCPP_WARN(logger_, "ArduinoComms: sendMsg write error: %s", e.what());
    try { serial_conn_.close(); } catch (...) {}
    return std::string();
  }

  // Пытаемся быстро получить последнюю строку (если Arduino что-то ответит)
  std::string last_line;
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(st->timeout_ms);

  while (std::chrono::steady_clock::now() < deadline) {
    try {
      std::string line;
      if (pump_lines(serial_conn_, st, line)) {
        last_line = line;
      } else {
        std::this_thread::sleep_for(2ms);
      }
    }
    catch (const std::exception &e) {
      RCLCPP_WARN(logger_, "ArduinoComms: sendMsg read error: %s", e.what());
      try { serial_conn_.close(); } catch (...) {}
      break;
    }
  }

  RCLCPP_INFO_STREAM(logger_, "Sent: " << msg_to_send);
  if (!last_line.empty()) {
    RCLCPP_INFO_STREAM(logger_, "Received: " << last_line);
  }

  return last_line;
}
