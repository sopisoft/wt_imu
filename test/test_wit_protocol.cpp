#include <gtest/gtest.h>
#include <array>
#include <vector>

#include "wt_imu/wit_protocol.hpp"

TEST(WitProtocol, DecodesAccelerationAndChecksum)
{
  const auto packet = wt_imu::WitProtocol::make_packet(0x51, {0x00, 0x10, 0x00, 0x00, 0x00, 0x00, 0x10, 0x27});
  wt_imu::WitProtocol parser;
  parser.feed(packet.data(), 3);
  parser.feed(packet.data() + 3, packet.size() - 3);
  parser.feed(packet);
  const auto sample = parser.take_sample();
  ASSERT_TRUE(sample);
  ASSERT_TRUE(sample->acceleration);
  EXPECT_NEAR((*sample->acceleration)[0], 2.0 * 9.80665, 1e-5);
  ASSERT_TRUE(sample->temperature);
  EXPECT_DOUBLE_EQ(*sample->temperature, 100.0);
}

TEST(WitProtocol, IgnoresBadChecksumAndResynchronizes)
{
  auto bad = wt_imu::WitProtocol::make_packet(0x53, {0, 0, 0, 0, 0, 0, 0, 0});
  bad.back()++;
  const auto good = wt_imu::WitProtocol::make_packet(0x53, {0, 0, 0, 0, 0, 0, 0, 0});
  bad.insert(bad.end(), good.begin(), good.end());
  bad.insert(bad.end(), good.begin(), good.end());
  wt_imu::WitProtocol parser;
  parser.feed(bad);
  EXPECT_TRUE(parser.take_sample());
}

TEST(WitProtocol, QueuesMultipleOutputCycles)
{
  const auto first = wt_imu::WitProtocol::make_packet(0x51, {0x00, 0x10, 0, 0, 0, 0, 0, 0});
  const auto second = wt_imu::WitProtocol::make_packet(0x51, {0x00, 0x20, 0, 0, 0, 0, 0, 0});
  std::vector<uint8_t> data;
  data.insert(data.end(), first.begin(), first.end());
  data.insert(data.end(), second.begin(), second.end());
  data.insert(data.end(), first.begin(), first.end());

  wt_imu::WitProtocol parser;
  parser.feed(data);
  const auto first_sample = parser.take_sample();
  const auto second_sample = parser.take_sample();
  ASSERT_TRUE(first_sample);
  ASSERT_TRUE(second_sample);
  ASSERT_TRUE(first_sample->acceleration);
  ASSERT_TRUE(second_sample->acceleration);
  EXPECT_LT((*first_sample->acceleration)[0], (*second_sample->acceleration)[0]);
}

TEST(WitProtocol, KeepsTemperatureFromAccelerationPacket)
{
  const auto acceleration = wt_imu::WitProtocol::make_packet(0x51, {0, 0, 0, 0, 0, 0, 0x78, 0x09});
  const auto angular_velocity = wt_imu::WitProtocol::make_packet(0x52, {0, 0, 0, 0, 0, 0, 0, 0});
  const auto magnetic_field = wt_imu::WitProtocol::make_packet(0x54, {0, 0, 0, 0, 0, 0, 0, 0});
  wt_imu::WitProtocol parser;
  parser.feed(acceleration);
  parser.feed(angular_velocity);
  parser.feed(magnetic_field);
  parser.feed(acceleration);

  const auto sample = parser.take_sample();
  ASSERT_TRUE(sample);
  ASSERT_TRUE(sample->temperature);
  EXPECT_DOUBLE_EQ(*sample->temperature, 24.24);
}

TEST(WitProtocol, KeepsZeroTemperature)
{
  const auto zero_temperature = wt_imu::WitProtocol::make_packet(0x51, {0, 0, 0, 0, 0, 0, 0, 0});
  wt_imu::WitProtocol parser;
  parser.feed(zero_temperature);
  parser.feed(zero_temperature);

  const auto sample = parser.take_sample();
  ASSERT_TRUE(sample);
  ASSERT_TRUE(sample->temperature);
  EXPECT_DOUBLE_EQ(*sample->temperature, 0.0);
}

TEST(WitProtocol, ConvertsMagneticFieldToTesla)
{
  const auto magnetic_field = wt_imu::WitProtocol::make_packet(0x54, {0xe8, 0x03, 0, 0, 0, 0, 0, 0});
  wt_imu::WitProtocol parser;
  parser.feed(magnetic_field);
  parser.feed(magnetic_field);

  const auto sample = parser.take_sample();
  ASSERT_TRUE(sample);
  ASSERT_TRUE(sample->magnetic_field);
  EXPECT_DOUBLE_EQ((*sample->magnetic_field)[0], 1000.0 * 0.013e-6);
  EXPECT_FALSE(sample->temperature);
}

TEST(WitProtocol, ConvertsGpsAndVelocityUnits)
{
  std::array<uint8_t, 8> gps_data{};
  const auto set_i32 = [&gps_data](size_t offset, int32_t value) {
    const auto raw = static_cast<uint32_t>(value);
    gps_data[offset] = static_cast<uint8_t>(raw);
    gps_data[offset + 1] = static_cast<uint8_t>(raw >> 8);
    gps_data[offset + 2] = static_cast<uint8_t>(raw >> 16);
    gps_data[offset + 3] = static_cast<uint8_t>(raw >> 24);
  };
  set_i32(0, 1394112345);
  set_i32(4, 351234567);

  const auto gps = wt_imu::WitProtocol::make_packet(0x57, gps_data);
  const auto velocity = wt_imu::WitProtocol::make_packet(0x58, {0xe8, 0x03, 0xb8, 0x88, 0x10, 0x0e, 0x00, 0x00});
  wt_imu::WitProtocol parser;
  parser.feed(gps);
  parser.feed(velocity);
  parser.feed(gps);
  parser.feed(velocity);

  const auto sample = parser.take_sample();
  ASSERT_TRUE(sample);
  ASSERT_TRUE(sample->gps);
  ASSERT_TRUE(sample->velocity);
  EXPECT_NEAR(sample->gps->longitude, 139.0 + 41.12345 / 60.0, 1e-7);
  EXPECT_NEAR(sample->gps->latitude, 35.0 + 12.34567 / 60.0, 1e-7);
  EXPECT_DOUBLE_EQ(sample->velocity->altitude, 100.0);
  EXPECT_NEAR(sample->velocity->yaw, 350.0 * 3.14159265358979323846 / 180.0, 1e-12);
  EXPECT_DOUBLE_EQ(sample->velocity->speed, 3.6);
}

TEST(WitProtocol, RejectsNullPacket)
{
  EXPECT_FALSE(wt_imu::WitProtocol::valid_packet(nullptr, 11));
}
