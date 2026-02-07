#ifndef DIFFDRIVE_ARDUINO_ARDUINO_COMMS_H
#define DIFFDRIVE_ARDUINO_ARDUINO_COMMS_H

#include <cstdint>
#include <string>

class ArduinoComms
{
public:
  ArduinoComms() = default;

  ArduinoComms(const std::string &serial_device, int32_t baud_rate, int32_t timeout_ms)
  {
    setup(serial_device, baud_rate, timeout_ms);
  }

  ~ArduinoComms();

  ArduinoComms(const ArduinoComms &) = delete;
  ArduinoComms &operator=(const ArduinoComms &) = delete;

  ArduinoComms(ArduinoComms &&other) noexcept;
  ArduinoComms &operator=(ArduinoComms &&other) noexcept;

  void setup(const std::string &serial_device, int32_t baud_rate, int32_t timeout_ms);

  void sendEmptyMsg();
  void readEncoderValues(int &val_1, int &val_2);
  void setMotorValues(int val_1, int val_2);
  void setPidValues(float k_p, float k_d, float k_i, float k_o);

  bool connected() const { return fd_ >= 0; }

  std::string sendMsg(const std::string &msg_to_send, bool print_output = false);

private:
  int fd_{-1};  ///< POSIX file descriptor for the serial device
};

#endif // DIFFDRIVE_ARDUINO_ARDUINO_COMMS_H
