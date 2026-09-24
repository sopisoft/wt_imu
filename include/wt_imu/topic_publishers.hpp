// Copyright 2026 sopi
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#ifndef WT_IMU__TOPIC_PUBLISHERS_HPP_
#define WT_IMU__TOPIC_PUBLISHERS_HPP_

#include <array>
#include <string>

#include <geometry_msgs/msg/quaternion_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/fluid_pressure.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/magnetic_field.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <sensor_msgs/msg/temperature.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/u_int16_multi_array.hpp>

#include "wt_imu/wit_protocol.hpp"

namespace wt_imu
{

class TopicPublishers
{
public:
  explicit TopicPublishers(rclcpp::Node &node);
  void publish(const Sample &sample, const rclcpp::Time &stamp);

private:
  struct TopicConfig {
    bool enabled;
    std::string topic;
    std::string frame_id;
  };

  TopicConfig declare_topic(const std::string &name, const std::string &default_topic);
  static geometry_msgs::msg::Quaternion quaternion_from_angles(const std::array<double, 3> &angles);
  void publish_imu(const Sample &sample, const rclcpp::Time &stamp);
  void publish_auxiliary(const Sample &sample, const rclcpp::Time &stamp);
  void publish_magnetometer(const Sample &sample, const rclcpp::Time &stamp);
  void publish_temperature(const Sample &sample, const rclcpp::Time &stamp);
  void publish_pressure_altitude(const Sample &sample, const rclcpp::Time &stamp);
  void publish_quaternion(const Sample &sample, const rclcpp::Time &stamp);
  void publish_gps(const Sample &sample, const rclcpp::Time &stamp);
  void publish_velocity(const Sample &sample, const rclcpp::Time &stamp);
  void publish_digital_ports(const Sample &sample);

  rclcpp::Node &node_;
  std::string frame_id_;
  TopicConfig imu_;
  TopicConfig magnetometer_;
  TopicConfig temperature_;
  TopicConfig pressure_;
  TopicConfig altitude_;
  TopicConfig quaternion_;
  TopicConfig gps_;
  TopicConfig velocity_;
  TopicConfig digital_port_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
  rclcpp::Publisher<sensor_msgs::msg::MagneticField>::SharedPtr magnetometer_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Temperature>::SharedPtr temperature_pub_;
  rclcpp::Publisher<sensor_msgs::msg::FluidPressure>::SharedPtr pressure_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr altitude_pub_;
  rclcpp::Publisher<geometry_msgs::msg::QuaternionStamped>::SharedPtr quaternion_pub_;
  rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr gps_pub_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr velocity_pub_;
  rclcpp::Publisher<std_msgs::msg::UInt16MultiArray>::SharedPtr digital_port_pub_;
};

} // namespace wt_imu

#endif // WT_IMU__TOPIC_PUBLISHERS_HPP_
