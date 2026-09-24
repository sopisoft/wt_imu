// Copyright 2026 sopi
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#ifndef WT_IMU__WIT_PROTOCOL_HPP_
#define WT_IMU__WIT_PROTOCOL_HPP_

#include <array>
#include <cstdint>
#include <deque>
#include <optional>
#include <vector>

namespace wt_imu
{

struct Sample {
  struct Time {
    int year;
    int month;
    int day;
    int hour;
    int minute;
    int second;
    int millisecond;
  };
  struct PressureAltitude {
    double pressure;
    double altitude;
  };
  struct Gps {
    double longitude;
    double latitude;
  };
  struct Velocity {
    double altitude;
    double yaw;
    double speed;
  };
  struct Gsa {
    int satellites;
    double pdop;
    double hdop;
    double vdop;
  };

  std::optional<Time> time;
  std::optional<std::array<double, 3>> acceleration;     // m/s^2
  std::optional<std::array<double, 3>> angular_velocity; // rad/s
  std::optional<std::array<double, 3>> angles;           // rad, roll/pitch/yaw
  std::optional<std::array<double, 3>> magnetic_field;   // tesla
  std::optional<std::array<double, 4>> quaternion;       // w/x/y/z
  std::optional<double> temperature;                     // deg C
  std::optional<std::array<uint16_t, 4>> digital_ports;
  std::optional<PressureAltitude> pressure_altitude;
  std::optional<Gps> gps;
  std::optional<Velocity> velocity;
  std::optional<Gsa> gsa;
};

class WitProtocol
{
public:
  explicit WitProtocol(double magnetic_field_scale_tesla_per_lsb = 0.013e-6);

  // Feed arbitrary serial chunks. A complete output cycle queues one sample.
  void feed(const uint8_t *data, size_t size);
  void feed(const std::vector<uint8_t> &data)
  {
    feed(data.data(), data.size());
  }
  std::optional<Sample> take_sample();

  static bool valid_packet(const uint8_t *packet, size_t size);
  static std::vector<uint8_t> make_packet(uint8_t id, const std::array<uint8_t, 8> &data);

private:
  void finish_sample();
  bool parse_packet(const uint8_t *packet);
  std::vector<uint8_t> buffer_;
  std::deque<Sample> samples_;
  Sample sample_;
  std::array<bool, 256> packet_seen_{};
  double magnetic_field_scale_tesla_per_lsb_;
  bool has_sample_{false};
};

} // namespace wt_imu

#endif // WT_IMU__WIT_PROTOCOL_HPP_
