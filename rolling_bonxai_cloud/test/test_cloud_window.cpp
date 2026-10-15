// SPDX-License-Identifier: MIT
#include <gtest/gtest.h>
#include <zstd.h>

#include <cmath>
#include <cstring>
#include <limits>

#include "rolling_bonxai_cloud/cloud_window.hpp"
#include "rolling_bonxai_cloud/window_conversion.hpp"

using namespace RollingBonxai;
TEST(CloudWindow, HalfOpenBoundsAndNegativeCoordinates) {
  CloudWindow window({-10, 0, 0}, {4, 6, 8});
  EXPECT_TRUE(window.add({-12, -3, -4, 2, 3}));
  EXPECT_TRUE(window.add({-10, 0, 0, 4, 5}));
  EXPECT_FALSE(window.add({-8, 0, 0, 6, 7}));
  EXPECT_FALSE(window.add({-13, 0, 0, 8, 9}));
  ASSERT_EQ(window.points().size(), 2u);
}
TEST(CloudWindow, LosslessGeometryAndRadiometryIncludingUnknown) {
  CloudWindow window({0, 0, 0}, {10, 10, 10});
  window.add({1.234567f, -2.345678f, 3.456789f, 65535, 42.5});
  window.add({0, 0, 0, std::numeric_limits<float>::quiet_NaN(), -0.f});
  const auto bytes = window.compress();
  const auto restored = decompressWindow(bytes, 2);
  ASSERT_EQ(restored.size(), 2u);
  EXPECT_EQ(std::memcmp(restored.data(), window.points().data(), 40), 0);
}
TEST(CloudWindow, EmptyReplacementIsValid) {
  CloudWindow window({0, 0, 0}, {1, 1, 1});
  EXPECT_TRUE(decompressWindow(window.compress(), 0).empty());
}
TEST(CloudWindow, WireBytePlanesHaveDefinedChannelAndEndianOrder) {
  CloudWindow window({0, 0, 0}, {10, 10, 10});
  window.add({1.f, -2.f, .5f, 4.f, 8.f});
  window.add({-0.f, 2.f, -.5f, 16.f, 32.f});
  const auto encoded = window.compress();
  uint8_t raw[40];
  ASSERT_EQ(ZSTD_decompress(raw, sizeof(raw), encoded.data(), encoded.size()), sizeof(raw));
  // Low two bytes are zero; plane 2 contains each IEEE-754 mantissa/exponent
  // byte, then plane 3 contains the sign/high exponent byte. Channels precede
  // points within a plane. This expectation is independent of the decoder.
  const uint8_t expected[40] = {0, 0, 0,    0,    0,    0,    0,    0,    0,    0,    0,    0,   0,    0,
                                0, 0, 0,    0,    0,    0,    0x80, 0,    0,    0,    0,    0,   0x80, 0x80,
                                0, 0, 0x3f, 0x80, 0xc0, 0x40, 0x3f, 0xbf, 0x40, 0x41, 0x41, 0x42};
  EXPECT_EQ(std::memcmp(raw, expected, sizeof(raw)), 0);
}
TEST(CloudWindow, RejectsCorruptionAndTrailingFrames) {
  CloudWindow window({0, 0, 0}, {1, 1, 1});
  window.add({0, 0, 0, 1, 2});
  auto payload = window.compress();
  payload.back() ^= 1;
  EXPECT_THROW(decompressWindow(payload, 1), std::runtime_error);
  payload = window.compress();
  payload.push_back(0);
  EXPECT_THROW(decompressWindow(payload, 1), std::runtime_error);
}
TEST(CloudWindow, RejectsDeclaredSizesAndReceiverBudgets) {
  CloudWindow window({0, 0, 0}, {1, 1, 1});
  window.add({0, 0, 0, 1, 2});
  const auto p = window.compress();
  EXPECT_THROW(decompressWindow(p, 2), std::runtime_error);
  EXPECT_THROW(decompressWindow(p, 1, 0), std::runtime_error);
  EXPECT_THROW(decompressWindow(p, 1, 1, 1), std::runtime_error);
}
TEST(CloudWindow, RejectsInvalidGeometryBoundsAndBudgetOverflow) {
  EXPECT_THROW(CloudWindow({0, 0, 0}, {1, 0, 1}), std::invalid_argument);
  CloudWindow window({0, 0, 0}, {1, 1, 1}, {1, 1024});
  window.add({0, 0, 0, 1, 2});
  EXPECT_THROW(window.add({0, 0, 0, 1, 2}), std::runtime_error);
  EXPECT_THROW(window.compress(), std::runtime_error);
  EXPECT_THROW(window.add({0, 0, 0, std::numeric_limits<float>::infinity(), 1}), std::invalid_argument);
  CloudWindow tiny({0, 0, 0}, {1, 1, 1}, {1, 1});
  EXPECT_THROW(tiny.compress(), std::runtime_error);
}
rolling_bonxai_msgs::msg::SurfaceMapWindow packet() {
  CloudWindow window({0, 0, 0}, {2, 2, 2});
  window.add({.5, -.25, 0, 10, 20});
  rolling_bonxai_msgs::msg::SurfaceMapWindow w;
  w.header.frame_id = "platform_a/map";
  w.schema_version = 1;
  w.source_id = "platform_a";
  w.map_id = "recording_1";
  w.center_keyframe = 0;
  w.keyframe_count = 1;
  w.center_pose.orientation.w = 1;
  w.extent.x = w.extent.y = w.extent.z = 2.;
  w.resolution = .05;
  w.encoding = kSurfaceEncoding;
  w.point_count = 1;
  w.uncompressed_bytes = 20;
  const auto data = window.compress();
  w.data.assign(data.begin(), data.end());
  return w;
}
TEST(CloudWindow, ReceiverPreservesFrameAndCompletePointRecords) {
  auto w = packet();
  w.resolution = 0.;  // The sender may explicitly disable map voxelization.
  const auto cloud = cloudFromWindow(w);
  EXPECT_EQ(cloud.header.frame_id, "platform_a/map");
  EXPECT_EQ(cloud.point_step, 20u);
  ASSERT_EQ(cloud.fields.size(), 5u);
  EXPECT_EQ(cloud.fields[4].name, "reflectivity");
  const auto p = decompressWindow(w.data.empty() ? nullptr : &w.data[0], w.data.size(), 1);
  EXPECT_EQ(std::memcmp(cloud.data.data(), p.data(), 20), 0);
}
TEST(CloudWindow, ReceiverRejectsInvalidMetadataAndFalseBounds) {
  auto w = packet();
  w.schema_version = 2;
  EXPECT_THROW(cloudFromWindow(w), std::invalid_argument);
  w = packet();
  w.center_pose.orientation.w = 0;
  EXPECT_THROW(cloudFromWindow(w), std::invalid_argument);
  w = packet();
  w.uncompressed_bytes = 19;
  EXPECT_THROW(cloudFromWindow(w), std::invalid_argument);
  w = packet();
  w.center_pose.position.x = 100;
  EXPECT_THROW(cloudFromWindow(w), std::invalid_argument);
}
