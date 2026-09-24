// Copyright 2026 sopi
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#ifndef WT_IMU__WIT_COMMANDS_HPP_
#define WT_IMU__WIT_COMMANDS_HPP_

#include <rclcpp/rclcpp.hpp>

namespace wt_imu
{

class SerialPort;

struct SensorSettings {
  bool configure{false};
  int output_rate{-1};
  int content{-1};
  bool save{true};
};

bool apply_sensor_settings(SerialPort &serial, const SensorSettings &settings);
bool synchronize_sensor_time(SerialPort &serial, const rclcpp::Time &timestamp);

} // namespace wt_imu

#endif // WT_IMU__WIT_COMMANDS_HPP_
