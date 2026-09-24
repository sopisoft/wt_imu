#include <algorithm>
#include <utility>

#include "wt_imu/wit_protocol.hpp"

namespace wt_imu
{
namespace
{
constexpr double kGravity = 9.80665;
constexpr double kPi = 3.14159265358979323846;
constexpr double kDegToRad = kPi / 180.0;

int16_t i16(const uint8_t *p)
{
  return static_cast<int16_t>(static_cast<uint16_t>(p[0]) | (static_cast<uint16_t>(p[1]) << 8));
}
uint16_t u16(const uint8_t *p)
{
  return static_cast<uint16_t>(p[0]) | (static_cast<uint16_t>(p[1]) << 8);
}
uint32_t u32(const uint8_t *p)
{
  return static_cast<uint32_t>(p[0]) | (static_cast<uint32_t>(p[1]) << 8) | (static_cast<uint32_t>(p[2]) << 16) |
         (static_cast<uint32_t>(p[3]) << 24);
}
int32_t i32(const uint8_t *p)
{
  return static_cast<int32_t>(u32(p));
}
std::array<double, 3> vec3(const uint8_t *p, double scale)
{
  return {i16(p) * scale, i16(p + 2) * scale, i16(p + 4) * scale};
}

double gps_coordinate(int32_t raw)
{
  const int64_t value = raw;
  const bool negative = value < 0;
  const int64_t magnitude = negative ? -value : value;
  const auto degrees = magnitude / 10000000;
  const auto minutes = static_cast<double>(magnitude % 10000000) / 100000.0;
  const auto coordinate = static_cast<double>(degrees) + minutes / 60.0;
  return negative ? -coordinate : coordinate;
}

bool supported_packet(uint8_t id)
{
  switch (id) {
    case 0x50:
    case 0x51:
    case 0x52:
    case 0x53:
    case 0x54:
    case 0x55:
    case 0x56:
    case 0x57:
    case 0x58:
    case 0x59:
    case 0x5A:
      return true;
    default:
      return false;
  }
}
} /* namespace */

WitProtocol::WitProtocol(double magnetic_field_scale_tesla_per_lsb)
    : magnetic_field_scale_tesla_per_lsb_(magnetic_field_scale_tesla_per_lsb)
{
}

bool WitProtocol::valid_packet(const uint8_t *packet, size_t size)
{
  if (packet == nullptr || size != 11 || packet[0] != 0x55) {
    return false;
  }
  uint8_t checksum = 0;
  for (size_t i = 0; i < 10; ++i) {
    checksum = static_cast<uint8_t>(checksum + packet[i]);
  }
  return checksum == packet[10];
}

std::vector<uint8_t> WitProtocol::make_packet(uint8_t id, const std::array<uint8_t, 8> &data)
{
  std::vector<uint8_t> packet{0x55, id};
  packet.insert(packet.end(), data.begin(), data.end());
  uint8_t checksum = 0;
  for (uint8_t byte : packet) {
    checksum = static_cast<uint8_t>(checksum + byte);
  }
  packet.push_back(checksum);
  return packet;
}

void WitProtocol::feed(const uint8_t *data, size_t size)
{
  if (size == 0) {
    return;
  }
  buffer_.insert(buffer_.end(), data, data + size);
  while (buffer_.size() >= 11) {
    auto start = std::find(buffer_.begin(), buffer_.end(), 0x55);
    if (start == buffer_.end()) {
      buffer_.clear();
      return;
    }
    buffer_.erase(buffer_.begin(), start);
    if (buffer_.size() < 11) {
      return;
    }
    if (valid_packet(buffer_.data(), 11)) {
      const auto id = buffer_[1];
      if (supported_packet(id) && packet_seen_[id]) {
        finish_sample();
      }
      if (parse_packet(buffer_.data())) {
        packet_seen_[id] = true;
      }
      buffer_.erase(buffer_.begin(), buffer_.begin() + 11);
    } else {
      buffer_.erase(buffer_.begin());
    }
  }
}

void WitProtocol::finish_sample()
{
  if (!has_sample_) {
    return;
  }
  samples_.push_back(sample_);
  sample_ = Sample{};
  packet_seen_.fill(false);
  has_sample_ = false;
}

bool WitProtocol::parse_packet(const uint8_t *p)
{
  const auto *d = p + 2;
  switch (p[1]) {
    case 0x50:
      sample_.time = Sample::Time{2000 + d[0], d[1], d[2], d[3], d[4], d[5], static_cast<int>(u16(d + 6))};
      break;
    case 0x51: {
      sample_.acceleration = vec3(d, 16.0 / 32768.0 * kGravity);
      sample_.temperature = i16(d + 6) / 100.0;
      break;
    }
    case 0x52:
      sample_.angular_velocity = vec3(d, 2000.0 / 32768.0 * kDegToRad);
      break;
    case 0x53:
      sample_.angles = vec3(d, 180.0 / 32768.0 * kDegToRad);
      break;
    case 0x54:
      sample_.magnetic_field = vec3(d, magnetic_field_scale_tesla_per_lsb_);
      break;
    case 0x55:
      sample_.digital_ports = std::array<uint16_t, 4>{u16(d), u16(d + 2), u16(d + 4), u16(d + 6)};
      break;
    case 0x56:
      sample_.pressure_altitude =
        Sample::PressureAltitude{static_cast<double>(u32(d)), static_cast<double>(i32(d + 4)) * 0.01};
      break;
    case 0x57:
      sample_.gps = Sample::Gps{gps_coordinate(i32(d)), gps_coordinate(i32(d + 4))};
      break;
    case 0x58:
      sample_.velocity =
        Sample::Velocity{i16(d) * 0.1, u16(d + 2) * 0.01 * kDegToRad,
                         (static_cast<uint32_t>(u16(d + 4)) | (static_cast<uint32_t>(u16(d + 6)) << 16)) / 1000.0};
      break;
    case 0x59:
      sample_.quaternion = {i16(d) / 32768.0, i16(d + 2) / 32768.0, i16(d + 4) / 32768.0, i16(d + 6) / 32768.0};
      break;
    case 0x5A:
      sample_.gsa = Sample::Gsa{u16(d), u16(d + 2) * 0.01, u16(d + 4) * 0.01, u16(d + 6) * 0.01};
      break;
    default:
      return false;
  }
  has_sample_ = true;
  return true;
}

std::optional<Sample> WitProtocol::take_sample()
{
  if (samples_.empty()) {
    return std::nullopt;
  }
  auto result = std::move(samples_.front());
  samples_.pop_front();
  return result;
}
} // namespace wt_imu
