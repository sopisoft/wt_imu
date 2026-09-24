#include <cmath>
#include <sensor_msgs/msg/nav_sat_status.hpp>

#include "wt_imu/topic_publishers.hpp"

namespace wt_imu
{

TopicPublishers::TopicPublishers(rclcpp::Node &node) : node_(node), frame_id_(node.declare_parameter("frame_id", "imu"))
{
  imu_ = declare_topic("imu", "imu");
  magnetometer_ = declare_topic("magnetometer", "magnetometer");
  temperature_ = declare_topic("temperature", "temperature");
  pressure_ = declare_topic("pressure", "pressure");
  altitude_ = declare_topic("altitude", "altitude");
  quaternion_ = declare_topic("quaternion", "quaternion");
  gps_ = declare_topic("gps", "gps");
  velocity_ = declare_topic("velocity", "velocity");
  digital_port_ = declare_topic("digital_port", "digital_port");

  if (imu_.enabled) {
    imu_pub_ = node_.create_publisher<sensor_msgs::msg::Imu>(imu_.topic, 10);
  }
  if (magnetometer_.enabled) {
    magnetometer_pub_ = node_.create_publisher<sensor_msgs::msg::MagneticField>(magnetometer_.topic, 10);
  }
  if (temperature_.enabled) {
    temperature_pub_ = node_.create_publisher<sensor_msgs::msg::Temperature>(temperature_.topic, 10);
  }
  if (pressure_.enabled) {
    pressure_pub_ = node_.create_publisher<sensor_msgs::msg::FluidPressure>(pressure_.topic, 10);
  }
  if (altitude_.enabled) {
    altitude_pub_ = node_.create_publisher<std_msgs::msg::Float32>(altitude_.topic, 10);
  }
  if (quaternion_.enabled) {
    quaternion_pub_ = node_.create_publisher<geometry_msgs::msg::QuaternionStamped>(quaternion_.topic, 10);
  }
  if (gps_.enabled) {
    gps_pub_ = node_.create_publisher<sensor_msgs::msg::NavSatFix>(gps_.topic, 10);
  }
  if (velocity_.enabled) {
    velocity_pub_ = node_.create_publisher<geometry_msgs::msg::TwistStamped>(velocity_.topic, 10);
  }
  if (digital_port_.enabled) {
    digital_port_pub_ = node_.create_publisher<std_msgs::msg::UInt16MultiArray>(digital_port_.topic, 10);
  }
}

TopicPublishers::TopicConfig TopicPublishers::declare_topic(const std::string &name, const std::string &default_topic)
{
  TopicConfig config{node_.declare_parameter(name + ".enable", true),
                     node_.declare_parameter(name + ".topic", default_topic),
                     node_.declare_parameter(name + ".frame_id", "")};
  if (config.topic.empty()) {
    config.topic = default_topic;
  }
  if (config.frame_id.empty()) {
    config.frame_id = frame_id_;
  }
  return config;
}

void TopicPublishers::publish(const Sample &sample, const rclcpp::Time &stamp)
{
  publish_imu(sample, stamp);
  publish_auxiliary(sample, stamp);
}

void TopicPublishers::publish_imu(const Sample &sample, const rclcpp::Time &stamp)
{
  if (!imu_pub_ || (!sample.acceleration && !sample.angular_velocity && !sample.angles && !sample.quaternion)) {
    return;
  }
  sensor_msgs::msg::Imu message;
  message.header.stamp = stamp;
  message.header.frame_id = imu_.frame_id;
  message.orientation_covariance[0] = -1.0;
  message.angular_velocity_covariance[0] = -1.0;
  message.linear_acceleration_covariance[0] = -1.0;
  if (sample.acceleration) {
    message.linear_acceleration.x = (*sample.acceleration)[0];
    message.linear_acceleration.y = (*sample.acceleration)[1];
    message.linear_acceleration.z = (*sample.acceleration)[2];
    message.linear_acceleration_covariance[0] = 0.0;
  }
  if (sample.angular_velocity) {
    message.angular_velocity.x = (*sample.angular_velocity)[0];
    message.angular_velocity.y = (*sample.angular_velocity)[1];
    message.angular_velocity.z = (*sample.angular_velocity)[2];
    message.angular_velocity_covariance[0] = 0.0;
  }
  if (sample.quaternion) {
    message.orientation.w = (*sample.quaternion)[0];
    message.orientation.x = (*sample.quaternion)[1];
    message.orientation.y = (*sample.quaternion)[2];
    message.orientation.z = (*sample.quaternion)[3];
    message.orientation_covariance[0] = 0.0;
  } else if (sample.angles) {
    message.orientation = quaternion_from_angles(*sample.angles);
    message.orientation_covariance[0] = 0.0;
  }
  imu_pub_->publish(message);
}

geometry_msgs::msg::Quaternion TopicPublishers::quaternion_from_angles(const std::array<double, 3> &angles)
{
  const auto roll = angles[0] / 2.0;
  const auto pitch = angles[1] / 2.0;
  const auto yaw = angles[2] / 2.0;
  const auto cr = std::cos(roll), sr = std::sin(roll);
  const auto cp = std::cos(pitch), sp = std::sin(pitch);
  const auto cy = std::cos(yaw), sy = std::sin(yaw);
  geometry_msgs::msg::Quaternion quaternion;
  quaternion.w = cr * cp * cy + sr * sp * sy;
  quaternion.x = sr * cp * cy - cr * sp * sy;
  quaternion.y = cr * sp * cy + sr * cp * sy;
  quaternion.z = cr * cp * sy - sr * sp * cy;
  return quaternion;
}

void TopicPublishers::publish_auxiliary(const Sample &sample, const rclcpp::Time &stamp)
{
  publish_magnetometer(sample, stamp);
  publish_temperature(sample, stamp);
  publish_pressure_altitude(sample, stamp);
  publish_quaternion(sample, stamp);
  publish_gps(sample, stamp);
  publish_velocity(sample, stamp);
  publish_digital_ports(sample);
}

void TopicPublishers::publish_magnetometer(const Sample &sample, const rclcpp::Time &stamp)
{
  if (!sample.magnetic_field || !magnetometer_pub_) {
    return;
  }
  sensor_msgs::msg::MagneticField message;
  message.header.stamp = stamp;
  message.header.frame_id = magnetometer_.frame_id;
  message.magnetic_field.x = (*sample.magnetic_field)[0];
  message.magnetic_field.y = (*sample.magnetic_field)[1];
  message.magnetic_field.z = (*sample.magnetic_field)[2];
  magnetometer_pub_->publish(message);
}

void TopicPublishers::publish_temperature(const Sample &sample, const rclcpp::Time &stamp)
{
  if (!sample.temperature || !temperature_pub_) {
    return;
  }
  sensor_msgs::msg::Temperature message;
  message.header.stamp = stamp;
  message.header.frame_id = temperature_.frame_id;
  message.temperature = *sample.temperature;
  temperature_pub_->publish(message);
}

void TopicPublishers::publish_pressure_altitude(const Sample &sample, const rclcpp::Time &stamp)
{
  if (!sample.pressure_altitude) {
    return;
  }
  if (pressure_pub_) {
    sensor_msgs::msg::FluidPressure message;
    message.header.stamp = stamp;
    message.header.frame_id = pressure_.frame_id;
    message.fluid_pressure = sample.pressure_altitude->pressure;
    pressure_pub_->publish(message);
  }
  if (altitude_pub_) {
    std_msgs::msg::Float32 message;
    message.data = static_cast<float>(sample.pressure_altitude->altitude);
    altitude_pub_->publish(message);
  }
}

void TopicPublishers::publish_quaternion(const Sample &sample, const rclcpp::Time &stamp)
{
  if (!sample.quaternion || !quaternion_pub_) {
    return;
  }
  geometry_msgs::msg::QuaternionStamped message;
  message.header.stamp = stamp;
  message.header.frame_id = quaternion_.frame_id;
  message.quaternion.w = (*sample.quaternion)[0];
  message.quaternion.x = (*sample.quaternion)[1];
  message.quaternion.y = (*sample.quaternion)[2];
  message.quaternion.z = (*sample.quaternion)[3];
  quaternion_pub_->publish(message);
}

void TopicPublishers::publish_gps(const Sample &sample, const rclcpp::Time &stamp)
{
  if (!sample.gps || !gps_pub_) {
    return;
  }
  sensor_msgs::msg::NavSatFix message;
  message.header.stamp = stamp;
  message.header.frame_id = gps_.frame_id;
  message.latitude = sample.gps->latitude;
  message.longitude = sample.gps->longitude;
  if (sample.velocity) {
    message.altitude = sample.velocity->altitude;
  }
  message.position_covariance_type = sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_UNKNOWN;
  message.status.status = sensor_msgs::msg::NavSatStatus::STATUS_NO_FIX;
  message.status.service = sensor_msgs::msg::NavSatStatus::SERVICE_GPS;
  if (sample.gsa && sample.gsa->satellites > 0) {
    message.status.status = sensor_msgs::msg::NavSatStatus::STATUS_FIX;
  }
  gps_pub_->publish(message);
}

void TopicPublishers::publish_velocity(const Sample &sample, const rclcpp::Time &stamp)
{
  if (!sample.velocity || !velocity_pub_) {
    return;
  }
  geometry_msgs::msg::TwistStamped message;
  message.header.stamp = stamp;
  message.header.frame_id = velocity_.frame_id;
  message.twist.linear.x = sample.velocity->speed;
  velocity_pub_->publish(message);
}

void TopicPublishers::publish_digital_ports(const Sample &sample)
{
  if (!sample.digital_ports || !digital_port_pub_) {
    return;
  }
  std_msgs::msg::UInt16MultiArray message;
  message.data.assign(sample.digital_ports->begin(), sample.digital_ports->end());
  digital_port_pub_->publish(message);
}

} // namespace wt_imu
