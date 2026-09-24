#include <chrono>
#include <ctime>
#include <thread>
#include <vector>

#include "wt_imu/serial_port.hpp"
#include "wt_imu/wit_commands.hpp"

namespace wt_imu
{
namespace
{

bool send(SerialPort &serial, std::initializer_list<uint8_t> command)
{
  return serial.write(std::vector<uint8_t>(command));
}

bool write_register(SerialPort &serial, uint8_t address, uint16_t value)
{
  return send(serial, {0xff, 0xaa, address, static_cast<uint8_t>(value & 0xff), static_cast<uint8_t>(value >> 8)});
}

bool unlock(SerialPort &serial)
{
  return send(serial, {0xff, 0xaa, 0x69, 0x88, 0xb5});
}

} /* namespace */

bool apply_sensor_settings(SerialPort &serial, const SensorSettings &settings)
{
  if (!settings.configure) {
    return true;
  }

  const bool has_changes = settings.output_rate >= 0 || settings.content >= 0;
  if (!has_changes) {
    return true;
  }
  if (!unlock(serial)) {
    return false;
  }
  std::this_thread::sleep_for(std::chrono::milliseconds(100));

  bool success = true;
  if (settings.output_rate >= 0) {
    success = write_register(serial, 0x03, static_cast<uint16_t>(settings.output_rate)) && success;
  }
  if (settings.content >= 0) {
    success = write_register(serial, 0x02, static_cast<uint16_t>(settings.content)) && success;
  }
  if (success && settings.save && (settings.output_rate >= 0 || settings.content >= 0)) {
    success = write_register(serial, 0x00, 0x0000) && success;
  }
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  return success;
}

bool synchronize_sensor_time(SerialPort &serial, const rclcpp::Time &timestamp)
{
  if (timestamp.nanoseconds() == 0) {
    return false;
  }

  const auto seconds = static_cast<std::time_t>(timestamp.seconds());
  std::tm utc{};
  if (gmtime_r(&seconds, &utc) == nullptr) {
    return false;
  }
  const auto milliseconds = static_cast<uint16_t>((timestamp.nanoseconds() / 1000000) % 1000);

  if (!unlock(serial)) {
    return false;
  }
  std::this_thread::sleep_for(std::chrono::milliseconds(100));

  return write_register(serial, 0x30, static_cast<uint16_t>((utc.tm_mon + 1) << 8 | (utc.tm_year % 100))) &&
         write_register(serial, 0x31, static_cast<uint16_t>(utc.tm_hour << 8 | utc.tm_mday)) &&
         write_register(serial, 0x32, static_cast<uint16_t>(utc.tm_sec << 8 | utc.tm_min)) &&
         write_register(serial, 0x33, milliseconds) && write_register(serial, 0x00, 0x0000);
}

} // namespace wt_imu
