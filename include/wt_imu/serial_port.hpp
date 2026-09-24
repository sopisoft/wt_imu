#ifndef WT_IMU__SERIAL_PORT_HPP_
#define WT_IMU__SERIAL_PORT_HPP_

#include <atomic>
#include <functional>
#include <mutex>
#include <string>
#include <thread>
#include <vector>
#include <cstddef>
#include <cstdint>

namespace wt_imu
{

class SerialPort
{
public:
  using Callback = std::function<void(const uint8_t *, size_t)>;
  SerialPort() = default;
  ~SerialPort();
  SerialPort(const SerialPort &) = delete;
  SerialPort &operator=(const SerialPort &) = delete;
  void open(const std::string &device, int baudrate, Callback callback);
  void close();
  bool write(const std::vector<uint8_t> &data);
  bool connected() const
  {
    return connected_;
  }

private:
  void read_loop();
  int fd_{-1};
  std::atomic<bool> running_{false};
  std::atomic<bool> connected_{false};
  std::jthread thread_;
  Callback callback_;
  mutable std::mutex io_mutex_;
};

} // namespace wt_imu

#endif // WT_IMU__SERIAL_PORT_HPP_
