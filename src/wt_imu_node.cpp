#include <algorithm>
#include <chrono>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>

#include <rclcpp/rclcpp.hpp>

#include "wt_imu/serial_port.hpp"
#include "wt_imu/topic_publishers.hpp"
#include "wt_imu/wit_commands.hpp"
#include "wt_imu/wit_protocol.hpp"

namespace wt_imu
{

class Node final : public rclcpp::Node
{
public:
  explicit Node(const rclcpp::NodeOptions &options)
      : rclcpp::Node("wt_imu", options), protocol_(declare_parameter("magnetometer.scale_tesla_per_lsb", 0.013e-6)),
        publishers_(*this)
  {
    device_ = declare_parameter("device", "/dev/ttyUSB0");
    const auto legacy_device = declare_parameter("device_port", std::string{});
    if (!legacy_device.empty()) {
      device_ = legacy_device;
    }

    const auto legacy_baudrate = declare_parameter("baudrate", 0);
    baudrate_ = legacy_baudrate > 0 ? legacy_baudrate : declare_parameter("baud_rate", 115200);
    auto_reconnect_ = declare_parameter("auto_reconnect", true);
    reconnect_interval_sec_ = declare_parameter("reconnect_interval_sec", 1.0);
    sync_sensor_time_ = declare_parameter("sync_sensor_time", true);
    settings_.configure = declare_parameter("configure_sensor", false);
    settings_.output_rate = declare_parameter("settings.output_rate", -1);
    settings_.content = declare_parameter("settings.content", -1);
    settings_.save = declare_parameter("settings.save", true);

    reconnect_timer_ = create_wall_timer(std::chrono::duration_cast<std::chrono::milliseconds>(
                                           std::chrono::duration<double>(std::max(0.1, reconnect_interval_sec_))),
                                         [this]() { reconnect_if_needed(); });

    if (!connect_sensor() && !auto_reconnect_) {
      throw std::runtime_error("cannot connect to WitMotion IMU");
    }
  }

  ~Node() override
  {
    serial_.close();
  }

private:
  bool connect_sensor()
  {
    try {
      serial_.open(device_, baudrate_, [this](const uint8_t *data, size_t size) { receive(data, size); });
      RCLCPP_INFO(get_logger(), "Connected to %s at %d baud", device_.c_str(), baudrate_);

      if (!apply_sensor_settings(serial_, settings_)) {
        throw std::runtime_error("failed to apply sensor settings");
      }
      if (sync_sensor_time_ && !synchronize_sensor_time(serial_, now())) {
        RCLCPP_WARN(get_logger(), "Sensor time synchronization failed");
      }
      return true;
    } catch (const std::exception &error) {
      serial_.close();
      RCLCPP_WARN(get_logger(), "IMU connection/setup failed: %s", error.what());
      return false;
    }
  }

  void reconnect_if_needed()
  {
    if (!auto_reconnect_ || serial_.connected()) {
      return;
    }
    RCLCPP_INFO(get_logger(), "Attempting to reconnect to %s", device_.c_str());
    connect_sensor();
  }

  void receive(const uint8_t *data, size_t size)
  {
    std::scoped_lock lock(mutex_);
    protocol_.feed(data, size);
    while (const auto sample = protocol_.take_sample()) {
      publishers_.publish(*sample, now());
    }
  }

  std::string device_;
  int baudrate_{};
  bool auto_reconnect_{};
  double reconnect_interval_sec_{};
  bool sync_sensor_time_{};
  SensorSettings settings_;
  SerialPort serial_;
  WitProtocol protocol_;
  TopicPublishers publishers_;
  std::mutex mutex_;
  rclcpp::TimerBase::SharedPtr reconnect_timer_;
};

} // namespace wt_imu

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<wt_imu::Node>(rclcpp::NodeOptions{}));
  rclcpp::shutdown();
  return 0;
}
