// Copyright 2026 Wuji Technology
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
// THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
// THE SOFTWARE.

#include <gtest/gtest.h>
#include <wuji_sdk.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <limits>
#include <string>
#include <utility>
#include <vector>

#include <wuji_msgs/msg/fingertip_tactile.hpp>

#include "wuji_hand2_driver/conversion/tactile_decoder.hpp"

namespace wuji_hand2_driver::conversion {
namespace {

constexpr const char* kOfficialFields =
    "\"point_stride\":6,\"point_fields\":["
    "{\"name\":\"fx\",\"offset\":0,\"type\":\"i16\",\"scale\":0.01},"
    "{\"name\":\"fy\",\"offset\":2,\"type\":\"i16\",\"scale\":0.01},"
    "{\"name\":\"fz\",\"offset\":4,\"type\":\"i16\",\"scale\":0.01}],"
    "\"aggregate_stride\":8,\"aggregate_fields\":["
    "{\"name\":\"fx\",\"offset\":0,\"type\":\"i16\",\"scale\":0.01},"
    "{\"name\":\"fy\",\"offset\":2,\"type\":\"i16\",\"scale\":0.01},"
    "{\"name\":\"fz\",\"offset\":4,\"type\":\"i16\",\"scale\":0.01},"
    "{\"name\":\"temperature\",\"offset\":6,\"type\":\"i16\",\"scale\":0.1}]";

constexpr const char* kTinyFormat =
    "{\"v\":1,\"encoding\":\"point_array\",\"point_count\":1,"
    "\"point_stride\":6,\"point_fields\":["
    "{\"name\":\"fx\",\"offset\":0,\"type\":\"i16\",\"scale\":0.01},"
    "{\"name\":\"fy\",\"offset\":2,\"type\":\"i16\",\"scale\":0.01},"
    "{\"name\":\"fz\",\"offset\":4,\"type\":\"i16\",\"scale\":0.01}],"
    "\"aggregate_stride\":8,\"aggregate_fields\":["
    "{\"name\":\"fx\",\"offset\":0,\"type\":\"i16\",\"scale\":0.01},"
    "{\"name\":\"fy\",\"offset\":2,\"type\":\"i16\",\"scale\":0.01},"
    "{\"name\":\"fz\",\"offset\":4,\"type\":\"i16\",\"scale\":0.01},"
    "{\"name\":\"temperature\",\"offset\":6,\"type\":\"i16\",\"scale\":0.1}]}";

std::string official_format(std::size_t point_count, bool shuffled = false) {
  if (!shuffled) {
    return "{\"v\":1,\"encoding\":\"point_array\",\"point_count\":" + std::to_string(point_count) +
           "," + kOfficialFields + "}";
  }
  return "{\"aggregate_fields\":["
         "{\"scale\":0.1,\"type\":\"i16\",\"offset\":6,\"name\":\"temperature\"},"
         "{\"scale\":0.01,\"type\":\"i16\",\"offset\":4,\"name\":\"fz\"},"
         "{\"scale\":0.01,\"type\":\"i16\",\"offset\":0,\"name\":\"fx\"},"
         "{\"scale\":0.01,\"type\":\"i16\",\"offset\":2,\"name\":\"fy\"}],"
         "\"point_fields\":["
         "{\"scale\":0.01,\"type\":\"i16\",\"offset\":4,\"name\":\"fz\"},"
         "{\"scale\":0.01,\"type\":\"i16\",\"offset\":0,\"name\":\"fx\"},"
         "{\"scale\":0.01,\"type\":\"i16\",\"offset\":2,\"name\":\"fy\"}],"
         "\"aggregate_stride\":8,\"point_stride\":6,\"point_count\":" +
         std::to_string(point_count) + ",\"encoding\":\"point_array\",\"v\":1}";
}

WujiFingertipSensorInfo make_info(const std::string& format, std::uint32_t digest = 0x1234ABCDU) {
  WujiFingertipSensorInfo info{};
  info.digest = digest;
  info.device_type = 0x0004;
  info.rate_hz = 100.0F;
  if (format.size() < sizeof(info.format)) {
    std::memcpy(info.format, format.data(), format.size());
    info.format[format.size()] = '\0';
  }
  return info;
}

void set_header(WujiFrameHeader& header, std::uint64_t timestamp_us, const char* frame_id) {
  header = {};
  header.timestamp_us = timestamp_us;
  std::strncpy(header.frame_id, frame_id, sizeof(header.frame_id) - 1);
}

template <std::size_t N>
WujiFingertipSensorData make_frame(std::array<std::uint8_t, N>& payload,
                                   std::uint32_t digest = 0x1234ABCDU) {
  WujiFingertipSensorData frame{};
  set_header(frame.header, 1234567, "thumb_tip_sensor_frame");
  frame.info_digest = digest;
  frame.data = payload.data();
  frame.data_len = payload.size();
  return frame;
}

wuji_msgs::msg::FingertipTactile sentinel_output() {
  wuji_msgs::msg::FingertipTactile output;
  output.header.stamp.sec = 7;
  output.header.stamp.nanosec = 8;
  output.header.frame_id = "unchanged";
  output.info_digest = 99;
  output.point_forces.resize(1);
  output.point_forces[0].x = 1.0;
  output.point_forces[0].y = 2.0;
  output.point_forces[0].z = 3.0;
  output.total_force.x = 4.0;
  output.total_force.y = 5.0;
  output.total_force.z = 6.0;
  output.temperature = 7.0F;
  return output;
}

void expect_sentinel(const wuji_msgs::msg::FingertipTactile& output) {
  EXPECT_EQ(output.header.stamp.sec, 7);
  EXPECT_EQ(output.header.stamp.nanosec, 8U);
  EXPECT_EQ(output.header.frame_id, "unchanged");
  EXPECT_EQ(output.info_digest, 99U);
  ASSERT_EQ(output.point_forces.size(), 1U);
  EXPECT_DOUBLE_EQ(output.point_forces[0].x, 1.0);
  EXPECT_DOUBLE_EQ(output.point_forces[0].y, 2.0);
  EXPECT_DOUBLE_EQ(output.point_forces[0].z, 3.0);
  EXPECT_DOUBLE_EQ(output.total_force.x, 4.0);
  EXPECT_DOUBLE_EQ(output.total_force.y, 5.0);
  EXPECT_DOUBLE_EQ(output.total_force.z, 6.0);
  EXPECT_FLOAT_EQ(output.temperature, 7.0F);
}

TEST(TactileDecoder, DecodesOfficialFortyPointThumbWithExactUnitsDigestAndHeader) {
  std::array<std::uint8_t, 248> payload{
      {0x85, 0xff, 0xfa, 0x00, 0xd4, 0xfe, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
       0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
       0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
       0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
       0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
       0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
       0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
       0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
       0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
       0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
       0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
       0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
       0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
       0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
       0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
       0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x41, 0x01, 0x22, 0xff, 0x6f, 0x00,
       0x0c, 0xfe, 0xe8, 0x03, 0x30, 0xf8, 0xfd, 0x00}};
  TactileDecoder decoder;
  ASSERT_TRUE(decoder.configure(make_info(official_format(40))));
  WujiFingertipSensorData frame = make_frame(payload);
  wuji_msgs::msg::FingertipTactile output;

  const auto status = decoder.decode(frame, output);

  ASSERT_TRUE(status) << status.message;
  EXPECT_EQ(output.header.stamp.sec, 1);
  EXPECT_EQ(output.header.stamp.nanosec, 234567000U);
  EXPECT_EQ(output.header.frame_id, "thumb_tip_sensor_frame");
  EXPECT_EQ(output.info_digest, 0x1234ABCDU);
  ASSERT_EQ(output.point_forces.size(), 40U);
  EXPECT_NEAR(output.point_forces[0].x, -1.23, 1e-12);
  EXPECT_NEAR(output.point_forces[0].y, 2.50, 1e-12);
  EXPECT_NEAR(output.point_forces[0].z, -3.00, 1e-12);
  EXPECT_NEAR(output.point_forces[39].x, 3.21, 1e-12);
  EXPECT_NEAR(output.point_forces[39].y, -2.22, 1e-12);
  EXPECT_NEAR(output.point_forces[39].z, 1.11, 1e-12);
  EXPECT_NEAR(output.total_force.x, -5.0, 1e-12);
  EXPECT_NEAR(output.total_force.y, 10.0, 1e-12);
  EXPECT_NEAR(output.total_force.z, -20.0, 1e-12);
  EXPECT_FLOAT_EQ(output.temperature, 25.3F);
}

TEST(TactileDecoder, DecodesOfficialThirtyFourPointLayoutWithShuffledDefinitions) {
  std::array<std::uint8_t, 212> payload{
      {0x64, 0x00, 0x38, 0xff, 0x2c, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
       0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
       0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
       0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
       0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
       0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
       0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
       0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
       0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
       0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
       0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
       0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
       0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
       0x00, 0x00, 0x00, 0x70, 0xfe, 0xf4, 0x01, 0xa8, 0xfd, 0xbc, 0x02, 0xe0, 0xfc, 0x84, 0x03,
       0xc9, 0xff}};
  TactileDecoder decoder;
  ASSERT_TRUE(decoder.configure(make_info(official_format(34, true), 0x01020304U)));
  WujiFingertipSensorData frame = make_frame(payload, 0x01020304U);
  set_header(frame.header, 2000001, "index_tip_sensor_frame");
  wuji_msgs::msg::FingertipTactile output;

  ASSERT_TRUE(decoder.decode(frame, output));
  ASSERT_EQ(output.point_forces.size(), 34U);
  EXPECT_DOUBLE_EQ(output.point_forces[0].x, 1.0);
  EXPECT_DOUBLE_EQ(output.point_forces[0].y, -2.0);
  EXPECT_DOUBLE_EQ(output.point_forces[0].z, 3.0);
  EXPECT_DOUBLE_EQ(output.point_forces[33].x, -4.0);
  EXPECT_DOUBLE_EQ(output.point_forces[33].y, 5.0);
  EXPECT_DOUBLE_EQ(output.point_forces[33].z, -6.0);
  EXPECT_DOUBLE_EQ(output.total_force.x, 7.0);
  EXPECT_DOUBLE_EQ(output.total_force.y, -8.0);
  EXPECT_DOUBLE_EQ(output.total_force.z, 9.0);
  EXPECT_FLOAT_EQ(output.temperature, -5.5F);
  EXPECT_EQ(output.header.frame_id, "index_tip_sensor_frame");
}

TEST(TactileDecoder, RejectsMalformedMetadataSchemas) {
  const std::vector<std::string> invalid_formats{
      "",
      "{",
      "[]",
      "{\"v\":2,\"encoding\":\"point_array\"}",
      "{\"v\":1,\"encoding\":\"other\"}",
      "{\"v\":1,\"encoding\":\"point_array\",\"point_count\":0}",
      "{\"v\":1,\"encoding\":\"point_array\",\"point_count\":1.5}",
      "{\"v\":1,\"encoding\":\"point_array\",\"point_count\":1,"
      "\"point_stride\":-1,\"point_fields\":[],\"aggregate_stride\":1,"
      "\"aggregate_fields\":[]}",
      "{\"v\":1,\"encoding\":\"point_array\","
      "\"point_count\":18446744073709551615,\"point_stride\":2,"
      "\"point_fields\":[],\"aggregate_stride\":1,\"aggregate_fields\":[]}",
      "{\"v\":1,\"encoding\":\"point_array\",\"point_count\":40,"
      "\"point_stride\":6,\"point_fields\":[],\"aggregate_stride\":17,"
      "\"aggregate_fields\":[]}",
      "{\"v\":1,\"encoding\":\"point_array\",\"point_count\":1,"
      "\"point_stride\":6,\"point_fields\":["
      "{\"name\":\"fx\",\"offset\":0,\"type\":\"i64\"},"
      "{\"name\":\"fy\",\"offset\":2,\"type\":\"i16\"},"
      "{\"name\":\"fz\",\"offset\":4,\"type\":\"i16\"}],"
      "\"aggregate_stride\":8,\"aggregate_fields\":[]}",
      "{\"v\":1,\"encoding\":\"point_array\",\"point_count\":1,"
      "\"point_stride\":6,\"point_fields\":["
      "{\"name\":\"fx\",\"offset\":5,\"type\":\"i16\"},"
      "{\"name\":\"fy\",\"offset\":2,\"type\":\"i16\"},"
      "{\"name\":\"fz\",\"offset\":4,\"type\":\"i16\"}],"
      "\"aggregate_stride\":8,\"aggregate_fields\":[]}",
      "{\"v\":1,\"encoding\":\"point_array\",\"point_count\":1,"
      "\"point_stride\":6,\"point_fields\":["
      "{\"name\":\"fx\",\"offset\":0,\"type\":\"i16\"},"
      "{\"name\":\"fy\",\"offset\":1,\"type\":\"i16\"},"
      "{\"name\":\"fz\",\"offset\":4,\"type\":\"i16\"}],"
      "\"aggregate_stride\":8,\"aggregate_fields\":[]}",
      "{\"v\":1,\"encoding\":\"point_array\",\"point_count\":1,"
      "\"point_stride\":6,\"point_fields\":["
      "{\"name\":\"fx\",\"offset\":0,\"type\":\"i16\"},"
      "{\"name\":\"fx\",\"offset\":2,\"type\":\"i16\"},"
      "{\"name\":\"fz\",\"offset\":4,\"type\":\"i16\"}],"
      "\"aggregate_stride\":8,\"aggregate_fields\":[]}",
      "{\"v\":1,\"encoding\":\"point_array\",\"point_count\":1,"
      "\"point_stride\":6,\"point_fields\":["
      "{\"name\":\"fx\",\"offset\":0,\"type\":\"i16\",\"scale\":\"x\"},"
      "{\"name\":\"fy\",\"offset\":2,\"type\":\"i16\"},"
      "{\"name\":\"fz\",\"offset\":4,\"type\":\"i16\"}],"
      "\"aggregate_stride\":8,\"aggregate_fields\":[]}",
      "{\"v\":1,\"encoding\":\"point_array\",\"point_count\":1,"
      "\"point_stride\":6,\"point_fields\":["
      "{\"name\":\"fx\",\"offset\":0,\"type\":\"i16\",\"scale\":1e400},"
      "{\"name\":\"fy\",\"offset\":2,\"type\":\"i16\"},"
      "{\"name\":\"fz\",\"offset\":4,\"type\":\"i16\"}],"
      "\"aggregate_stride\":8,\"aggregate_fields\":[]}",
  };

  for (const auto& format : invalid_formats) {
    TactileDecoder decoder;
    const auto status = decoder.configure(make_info(format));
    EXPECT_FALSE(status) << "unexpectedly accepted: " << format;
  }

  TactileDecoder missing_required;
  std::string missing = kTinyFormat;
  const auto location = missing.find("\"temperature\"");
  ASSERT_NE(location, std::string::npos);
  missing.replace(location, std::strlen("\"temperature\""), "\"other\"");
  EXPECT_FALSE(missing_required.configure(make_info(missing)));

  TactileDecoder zero_digest;
  EXPECT_FALSE(zero_digest.configure(make_info(kTinyFormat, 0)));

  for (const float rate : {0.0F, -1.0F, std::numeric_limits<float>::infinity(),
                           std::numeric_limits<float>::quiet_NaN()}) {
    auto info = make_info(kTinyFormat);
    info.rate_hz = rate;
    TactileDecoder decoder;
    EXPECT_FALSE(decoder.configure(info));
  }
}

TEST(TactileDecoder, RejectsInvalidFramesWithoutChangingOutput) {
  std::array<std::uint8_t, 14> payload{{
      0x64,
      0x00,
      0x38,
      0xff,
      0x2c,
      0x01,
      0x90,
      0x01,
      0x0c,
      0xfe,
      0x58,
      0x02,
      0xfa,
      0x00,
  }};
  TactileDecoder decoder;
  ASSERT_TRUE(decoder.configure(make_info(kTinyFormat)));
  WujiFingertipSensorData frame = make_frame(payload);
  wuji_msgs::msg::FingertipTactile output;

  const auto reject = [&](const WujiFingertipSensorData& invalid) {
    output = sentinel_output();
    EXPECT_FALSE(decoder.decode(invalid, output));
    expect_sentinel(output);
  };

  auto mismatch = frame;
  mismatch.info_digest = 0x99999999U;
  output = sentinel_output();
  const auto mismatch_status = decoder.decode(mismatch, output);
  EXPECT_FALSE(mismatch_status);
  EXPECT_NE(mismatch_status.message.find("digest"), std::string::npos);
  expect_sentinel(output);

  auto null_data = frame;
  null_data.data = nullptr;
  reject(null_data);

  auto short_data = frame;
  short_data.data_len = payload.size() - 1;
  reject(short_data);

  auto long_data = frame;
  long_data.data_len = payload.size() + 1;
  reject(long_data);

  auto zero_timestamp = frame;
  zero_timestamp.header.timestamp_us = 0;
  reject(zero_timestamp);

  auto unterminated_frame = frame;
  std::fill(std::begin(unterminated_frame.header.frame_id),
            std::end(unterminated_frame.header.frame_id), 'x');
  reject(unterminated_frame);

  TactileDecoder unconfigured;
  output = sentinel_output();
  EXPECT_FALSE(unconfigured.decode(frame, output));
  expect_sentinel(output);
}

TEST(TactileDecoder, RejectsNonfiniteDecodedValuesWithoutChangingOutput) {
  constexpr const char* float_format =
      "{\"v\":1,\"encoding\":\"point_array\",\"point_count\":1,"
      "\"point_stride\":12,\"point_fields\":["
      "{\"name\":\"fx\",\"offset\":0,\"type\":\"f32\"},"
      "{\"name\":\"fy\",\"offset\":4,\"type\":\"f32\"},"
      "{\"name\":\"fz\",\"offset\":8,\"type\":\"f32\"}],"
      "\"aggregate_stride\":16,\"aggregate_fields\":["
      "{\"name\":\"fx\",\"offset\":0,\"type\":\"f32\"},"
      "{\"name\":\"fy\",\"offset\":4,\"type\":\"f32\"},"
      "{\"name\":\"fz\",\"offset\":8,\"type\":\"f32\"},"
      "{\"name\":\"temperature\",\"offset\":12,\"type\":\"f32\"}]}";
  std::array<std::uint8_t, 28> payload{{
      0x00, 0x00, 0xc0, 0x7f, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
      0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  }};
  TactileDecoder decoder;
  ASSERT_TRUE(decoder.configure(make_info(float_format)));
  WujiFingertipSensorData frame = make_frame(payload);
  auto output = sentinel_output();

  EXPECT_FALSE(decoder.decode(frame, output));
  expect_sentinel(output);

  constexpr const char* overflow_format =
      "{\"v\":1,\"encoding\":\"point_array\",\"point_count\":1,"
      "\"point_stride\":6,\"point_fields\":["
      "{\"name\":\"fx\",\"offset\":0,\"type\":\"i16\"},"
      "{\"name\":\"fy\",\"offset\":2,\"type\":\"i16\"},"
      "{\"name\":\"fz\",\"offset\":4,\"type\":\"i16\"}],"
      "\"aggregate_stride\":8,\"aggregate_fields\":["
      "{\"name\":\"fx\",\"offset\":0,\"type\":\"i16\"},"
      "{\"name\":\"fy\",\"offset\":2,\"type\":\"i16\"},"
      "{\"name\":\"fz\",\"offset\":4,\"type\":\"i16\"},"
      "{\"name\":\"temperature\",\"offset\":6,\"type\":\"i16\","
      "\"scale\":1e40}]}";
  std::array<std::uint8_t, 14> integer_payload{{
      0x00,
      0x00,
      0x00,
      0x00,
      0x00,
      0x00,
      0x00,
      0x00,
      0x00,
      0x00,
      0x00,
      0x00,
      0xff,
      0x7f,
  }};
  TactileDecoder overflow_decoder;
  ASSERT_TRUE(overflow_decoder.configure(make_info(overflow_format)));
  auto overflow_frame = make_frame(integer_payload);
  output = sentinel_output();
  EXPECT_FALSE(overflow_decoder.decode(overflow_frame, output));
  expect_sentinel(output);
}

}  // namespace
}  // namespace wuji_hand2_driver::conversion
