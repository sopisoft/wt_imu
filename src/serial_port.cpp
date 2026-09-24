#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

#include <cerrno>
#include <cstring>
#include <chrono>
#include <stdexcept>
#include <utility>

#include "wt_imu/serial_port.hpp"

namespace wt_imu
{
namespace
{
speed_t baud_constant(int baudrate)
{
  switch (baudrate) {
    case 4800:
      return B4800;
    case 9600:
      return B9600;
    case 19200:
      return B19200;
    case 38400:
      return B38400;
    case 57600:
      return B57600;
    case 115200:
      return B115200;
    case 230400:
      return B230400;
    case 460800:
      return B460800;
    case 921600:
      return B921600;
    default:
      throw std::invalid_argument("unsupported baudrate");
  }
}
} /* namespace */

void SerialPort::open(const std::string &device, int baudrate, Callback callback)
{
  close();
  fd_ = ::open(device.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
  if (fd_ < 0) {
    throw std::runtime_error("cannot open " + device + ": " + std::strerror(errno));
  }
  termios tty{};
  if (tcgetattr(fd_, &tty) != 0) {
    close();
    throw std::runtime_error("cannot configure " + device);
  }
  cfmakeraw(&tty);
  const auto speed = baud_constant(baudrate);
  cfsetispeed(&tty, speed);
  cfsetospeed(&tty, speed);
  tty.c_cflag |= CLOCAL | CREAD;
  if (tcsetattr(fd_, TCSANOW, &tty) != 0) {
    close();
    throw std::runtime_error("cannot apply serial settings");
  }
  callback_ = std::move(callback);
  running_ = true;
  connected_ = true;
  thread_ = std::jthread([this] { read_loop(); });
}

void SerialPort::read_loop()
{
  uint8_t bytes[256];
  while (running_) {
    const auto count = ::read(fd_, bytes, sizeof(bytes));
    if (count > 0) {
      callback_(bytes, static_cast<size_t>(count));
    } else if (count < 0 && errno != EAGAIN && errno != EINTR) {
      connected_ = false;
      running_ = false;
      break;
    } else {
      std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }
  }
}

void SerialPort::close()
{
  running_ = false;
  connected_ = false;
  if (thread_.joinable()) {
    thread_.request_stop();
  }
  thread_ = std::jthread{};
  if (fd_ >= 0) {
    ::close(fd_);
    fd_ = -1;
  }
}

bool SerialPort::write(const std::vector<uint8_t> &data)
{
  if (data.empty() || !connected_) {
    return false;
  }
  std::lock_guard<std::mutex> lock(io_mutex_);
  size_t offset = 0;
  while (offset < data.size()) {
    const auto count = ::write(fd_, data.data() + offset, data.size() - offset);
    if (count <= 0) {
      connected_ = false;
      running_ = false;
      return false;
    }
    offset += static_cast<size_t>(count);
  }
  return true;
}

SerialPort::~SerialPort()
{
  close();
}
} // namespace wt_imu
