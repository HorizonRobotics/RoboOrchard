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
#include <set>
#include <string>
#include <string_view>
#include <vector>

#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <wuji_msgs/msg/joint_diagnostics_array.hpp>

#include "wuji_hand2_driver/conversion/imu_conversion.hpp"
#include "wuji_hand2_driver/conversion/joint_command_conversion.hpp"
#include "wuji_hand2_driver/conversion/joint_diagnostics_conversion.hpp"
#include "wuji_hand2_driver/conversion/joint_mapping.hpp"
#include "wuji_hand2_driver/conversion/joint_state_conversion.hpp"
#include "wuji_hand2_driver/conversion/ros_header_conversion.hpp"

namespace wuji_hand2_driver::conversion {
namespace {

constexpr std::array<std::uint8_t, 20> kExpectedNids{1,  2,  3,  4,  6,  7,  8,  9,  11, 12,
                                                     13, 14, 16, 17, 18, 19, 21, 22, 23, 24};

constexpr std::array<std::string_view, 20> kExpectedBodies{
    "thumb_cmc_flex",         "thumb_cmc_abd",         "thumb_mcp",         "thumb_ip",
    "index_finger_mcp_flex",  "index_finger_mcp_abd",  "index_finger_pip",  "index_finger_dip",
    "middle_finger_mcp_flex", "middle_finger_mcp_abd", "middle_finger_pip", "middle_finger_dip",
    "ring_finger_mcp_flex",   "ring_finger_mcp_abd",   "ring_finger_pip",   "ring_finger_dip",
    "pinky_mcp_flex",         "pinky_mcp_abd",         "pinky_pip",         "pinky_dip"};

void expect_golden_mapping(HandSide side, std::string_view prefix) {
  const JointMapping mapping{side};
  std::set<std::string> names;

  for (std::size_t index = 0; index < kExpectedNids.size(); ++index) {
    const std::string expected_name = std::string{prefix} + std::string{kExpectedBodies[index]};
    ASSERT_EQ(mapping.nid_from_index(index), kExpectedNids[index]);
    ASSERT_EQ(mapping.index_from_nid(kExpectedNids[index]), index);
    ASSERT_EQ(mapping.name_from_index(index), expected_name);
    ASSERT_EQ(mapping.name_from_nid(kExpectedNids[index]), expected_name);
    ASSERT_EQ(mapping.index_from_name(expected_name), index);
    EXPECT_TRUE(names.insert(expected_name).second);
  }
  EXPECT_EQ(names.size(), 20U);
}

TEST(JointMapping, MatchesTheFullLeftAndRightGoldenTables) {
  expect_golden_mapping(HandSide::kLeft, "l_");
  expect_golden_mapping(HandSide::kRight, "r_");
}

TEST(JointMapping, MatchesSdkDenseIndexAndNidConversion) {
  // These SDK functions are pure conversions: no init or hardware access.
  const JointMapping mapping{HandSide::kRight};
  for (std::uint8_t index = 0; index < JointMapping::kJointCount; ++index) {
    std::uint8_t nid = 0;
    ASSERT_EQ(wuji_hand_2_joint_index_to_nid(index, &nid), WUJI_STATUS_OK);
    EXPECT_EQ(mapping.nid_from_index(index), nid);
    std::uint8_t converted_index = 0;
    ASSERT_EQ(wuji_hand_2_nid_to_joint_index(nid, &converted_index), WUJI_STATUS_OK);
    EXPECT_EQ(converted_index, index);
  }
}

TEST(JointMapping, RejectsUnknownNidsIndicesAndNames) {
  const JointMapping left{HandSide::kLeft};
  const JointMapping right{HandSide::kRight};

  for (const std::uint8_t nid : {0, 5, 10, 15, 20, 25, 255}) {
    EXPECT_FALSE(left.index_from_nid(nid).has_value());
    EXPECT_FALSE(left.name_from_nid(nid).has_value());
  }
  EXPECT_FALSE(left.nid_from_index(20).has_value());
  EXPECT_FALSE(left.name_from_index(20).has_value());
  EXPECT_FALSE(left.index_from_name("").has_value());
  EXPECT_FALSE(left.index_from_name("r_thumb_cmc_flex").has_value());
  EXPECT_FALSE(right.index_from_name("l_thumb_cmc_flex").has_value());
  EXPECT_FALSE(right.index_from_name("r_unknown_joint").has_value());
}

void set_header(WujiFrameHeader& header, std::uint64_t timestamp_us, const char* frame_id) {
  header = {};
  header.timestamp_us = timestamp_us;
  std::strncpy(header.frame_id, frame_id, sizeof(header.frame_id) - 1);
}

sensor_msgs::msg::JointState sentinel_joint_state() {
  sensor_msgs::msg::JointState output;
  output.header.stamp.sec = 7;
  output.header.stamp.nanosec = 8;
  output.header.frame_id = "unchanged";
  output.name = {"sentinel"};
  output.position = {1.0};
  output.velocity = {2.0};
  output.effort = {3.0};
  return output;
}

void expect_joint_state_sentinel(const sensor_msgs::msg::JointState& output) {
  EXPECT_EQ(output.header.stamp.sec, 7);
  EXPECT_EQ(output.header.stamp.nanosec, 8U);
  EXPECT_EQ(output.header.frame_id, "unchanged");
  EXPECT_EQ(output.name, std::vector<std::string>{"sentinel"});
  EXPECT_EQ(output.position, std::vector<double>{1.0});
  EXPECT_EQ(output.velocity, std::vector<double>{2.0});
  EXPECT_EQ(output.effort, std::vector<double>{3.0});
}

wuji_msgs::msg::JointDiagnosticsArray sentinel_diagnostics() {
  wuji_msgs::msg::JointDiagnosticsArray output;
  output.header.frame_id = "unchanged";
  output.joints.resize(1);
  output.joints[0].nid = 99;
  output.joints[0].name = "sentinel";
  return output;
}

sensor_msgs::msg::Imu sentinel_imu() {
  sensor_msgs::msg::Imu output;
  output.header.frame_id = "unchanged";
  output.orientation.w = 9.0;
  output.angular_velocity.x = 8.0;
  output.linear_acceleration.z = 7.0;
  return output;
}

std::array<WujiJointCommand, 20> sentinel_commands() {
  std::array<WujiJointCommand, 20> output{};
  for (auto& command : output) {
    command = WujiJointCommand{99.0F, 98.0F, 97.0F};
  }
  return output;
}

void expect_command_sentinel(const std::array<WujiJointCommand, 20>& output) {
  for (const auto& command : output) {
    EXPECT_FLOAT_EQ(command.position, 99.0F);
    EXPECT_FLOAT_EQ(command.velocity, 98.0F);
    EXPECT_FLOAT_EQ(command.effort, 97.0F);
  }
}

TEST(JointStateConversion, PreservesOnlineSubsetOrderAndAlignedArrays) {
  std::array<WujiJointStateEntry, 3> entries{{
      {24, 0.25F, 0.5F, 0.75F},
      {1, -1.0F, -2.0F, -3.0F},
      {7, 4.0F, 5.0F, 6.0F},
  }};
  WujiJointStateFrame frame{};
  set_header(frame.header, 1234567, "sdk_joint_frame");
  frame.num_joints = 3;
  frame.joints = entries.data();
  frame.joints_len = entries.size();
  sensor_msgs::msg::JointState output;

  const auto status = to_joint_state(frame, JointMapping{HandSide::kLeft}, output);

  ASSERT_TRUE(status) << status.message;
  EXPECT_EQ(output.header.stamp.sec, 1);
  EXPECT_EQ(output.header.stamp.nanosec, 234567000U);
  EXPECT_EQ(output.header.frame_id, "sdk_joint_frame");
  EXPECT_EQ(output.name, (std::vector<std::string>{"l_pinky_dip", "l_thumb_cmc_flex",
                                                   "l_index_finger_mcp_abd"}));
  EXPECT_EQ(output.position, (std::vector<double>{0.25, -1.0, 4.0}));
  EXPECT_EQ(output.velocity, (std::vector<double>{0.5, -2.0, 5.0}));
  EXPECT_EQ(output.effort, (std::vector<double>{0.75, -3.0, 6.0}));
}

TEST(JointStateConversion, RejectsMalformedFramesWithoutChangingOutput) {
  std::array<WujiJointStateEntry, 2> entries{{{1, 1.0F, 2.0F, 3.0F}, {2, 4.0F, 5.0F, 6.0F}}};
  const JointMapping mapping{HandSide::kLeft};
  sensor_msgs::msg::JointState output;
  WujiJointStateFrame frame{};
  set_header(frame.header, 1, "frame");

  const auto reject = [&](std::uint8_t count, WujiJointStateEntry* data, std::size_t length) {
    frame.num_joints = count;
    frame.joints = data;
    frame.joints_len = length;
    output = sentinel_joint_state();
    EXPECT_FALSE(to_joint_state(frame, mapping, output));
    expect_joint_state_sentinel(output);
  };

  reject(0, nullptr, 0);
  reject(1, nullptr, 1);
  reject(1, entries.data(), 2);
  reject(2, entries.data(), 1);

  std::array<WujiJointStateEntry, 21> oversized_entries{};
  frame.num_joints = 21;
  frame.joints = oversized_entries.data();
  frame.joints_len = oversized_entries.size();
  output = sentinel_joint_state();
  const auto oversized_status = to_joint_state(frame, mapping, output);
  EXPECT_FALSE(oversized_status);
  EXPECT_EQ(oversized_status.message, "joint state frame must contain 1 to 20 joints");
  expect_joint_state_sentinel(output);

  frame.joints = entries.data();
  entries[1].nid = 1;
  reject(2, entries.data(), 2);
  entries[1].nid = 5;
  reject(2, entries.data(), 2);
  entries[1].nid = 250;
  reject(2, entries.data(), 2);
  entries[1] = WujiJointStateEntry{2, 4.0F, 5.0F, 6.0F};
  entries[0].velocity = std::numeric_limits<float>::infinity();
  reject(2, entries.data(), 2);
  entries[0].velocity = 2.0F;
  entries[0].effort = std::numeric_limits<float>::quiet_NaN();
  reject(2, entries.data(), 2);
}

TEST(JointCommandConversion, MapsNamedPartialCommandInAnyOrderWithAllFields) {
  sensor_msgs::msg::JointState input;
  input.header.stamp.sec = -99;
  input.header.frame_id = "ignored";
  input.name = {"l_pinky_dip", "l_thumb_cmc_flex"};
  input.position = {1.25, -2.5};
  input.velocity = {3.75, -4.5};
  input.effort = {5.25, -6.5};
  std::array<WujiJointCommand, 20> output{};

  const auto status = to_joint_commands(input, JointMapping{HandSide::kLeft}, output);

  ASSERT_TRUE(status) << status.message;
  EXPECT_FLOAT_EQ(output[19].position, 1.25F);
  EXPECT_FLOAT_EQ(output[19].velocity, 3.75F);
  EXPECT_FLOAT_EQ(output[19].effort, 5.25F);
  EXPECT_FLOAT_EQ(output[0].position, -2.5F);
  EXPECT_FLOAT_EQ(output[0].velocity, -4.5F);
  EXPECT_FLOAT_EQ(output[0].effort, -6.5F);
  for (std::size_t index = 1; index < 19; ++index) {
    EXPECT_FLOAT_EQ(output[index].position, 0.0F);
    EXPECT_FLOAT_EQ(output[index].velocity, 0.0F);
    EXPECT_FLOAT_EQ(output[index].effort, 0.0F);
  }
}

TEST(JointCommandConversion, MapsAnonymousFullTwentyAndDefaultsOptionalFields) {
  sensor_msgs::msg::JointState input;
  for (std::size_t index = 0; index < 20; ++index) {
    input.position.push_back(static_cast<double>(index) + 0.5);
    input.effort.push_back(-static_cast<double>(index) - 0.25);
  }
  std::array<WujiJointCommand, 20> output{};

  ASSERT_TRUE(to_joint_commands(input, JointMapping{HandSide::kLeft}, output));
  for (std::size_t index = 0; index < output.size(); ++index) {
    EXPECT_FLOAT_EQ(output[index].position, static_cast<float>(index) + 0.5F);
    EXPECT_FLOAT_EQ(output[index].velocity, 0.0F);
    EXPECT_FLOAT_EQ(output[index].effort, -static_cast<float>(index) - 0.25F);
  }
}

TEST(JointCommandConversion, RejectsAnonymousCommandsWithNineteenOrTwentyOnePositions) {
  const JointMapping mapping{HandSide::kLeft};
  std::array<WujiJointCommand, 20> output;
  sensor_msgs::msg::JointState input;

  for (const std::size_t size : {19U, 21U}) {
    input.position.assign(size, 1.0);
    output = sentinel_commands();

    const Status status = to_joint_commands(input, mapping, output);

    EXPECT_FALSE(status);
    EXPECT_EQ(status.message, "joint command position length is invalid");
    expect_command_sentinel(output);
  }
}

TEST(JointCommandConversion, RejectsInvalidNamedCommandsWithoutChangingOutput) {
  const JointMapping mapping{HandSide::kLeft};
  std::array<WujiJointCommand, 20> output;
  sensor_msgs::msg::JointState input;
  const auto reject = [&](sensor_msgs::msg::JointState candidate) {
    output = sentinel_commands();
    EXPECT_FALSE(to_joint_commands(candidate, mapping, output));
    expect_command_sentinel(output);
  };

  input.name = {"l_thumb_cmc_flex"};
  reject(input);
  input.position = {1.0, 2.0};
  reject(input);
  input.position = {1.0};
  input.velocity = {1.0, 2.0};
  reject(input);
  input.velocity.clear();
  input.effort = {1.0, 2.0};
  reject(input);
  input.effort.clear();
  input.name = {"unknown"};
  reject(input);
  input.name = {"r_thumb_cmc_flex"};
  reject(input);
  input.name = {"l_thumb_cmc_flex", "l_thumb_cmc_flex"};
  input.position = {1.0, 2.0};
  reject(input);
  input.name = {"l_thumb_cmc_flex"};
  input.position = {std::numeric_limits<double>::quiet_NaN()};
  reject(input);
  input.position = {1.0};
  input.velocity = {std::numeric_limits<double>::infinity()};
  reject(input);
  input.velocity.clear();
  input.effort = {-std::numeric_limits<double>::infinity()};
  reject(input);
  input.effort.clear();
}

TEST(JointCommandConversion, AcceptsFloatBoundsAndRejectsOverflowWithoutChangingOutput) {
  const JointMapping mapping{HandSide::kLeft};
  sensor_msgs::msg::JointState input;
  input.name = {"l_thumb_cmc_flex"};
  const double lowest = static_cast<double>(std::numeric_limits<float>::lowest());
  const double highest = static_cast<double>(std::numeric_limits<float>::max());
  std::array<WujiJointCommand, 20> output{};
  for (const double value : {lowest, 0.0, highest}) {
    input.position = {value};
    input.velocity = {value};
    input.effort = {value};
    ASSERT_TRUE(to_joint_commands(input, mapping, output));
    EXPECT_FLOAT_EQ(output[0].position, static_cast<float>(value));
    EXPECT_FLOAT_EQ(output[0].velocity, static_cast<float>(value));
    EXPECT_FLOAT_EQ(output[0].effort, static_cast<float>(value));
  }
  const double infinity = std::numeric_limits<double>::infinity();
  for (const double value : {std::nextafter(lowest, -infinity),
                            std::nextafter(highest, infinity)}) {
    for (auto field : {&sensor_msgs::msg::JointState::position,
                       &sensor_msgs::msg::JointState::velocity,
                       &sensor_msgs::msg::JointState::effort}) {
      input.position = {0.0};
      input.velocity = {0.0};
      input.effort = {0.0};
      (input.*field)[0] = value;
      output = sentinel_commands();
      EXPECT_FALSE(to_joint_commands(input, mapping, output));
      expect_command_sentinel(output);
    }
  }
}

TEST(JointDiagnosticsConversion, MapsOnlyApprovedPerJointFields) {
  std::array<WujiJointDiagnosticsEntry, 2> entries{{
      {6, 0x12345678U, 1.5F, 24.25F, 36.5F, 0x2345U, 98, 1234},
      {24, 0x87654321U, -2.5F, 23.75F, 40.0F, 0x3456U, 87, 5678},
  }};
  WujiJointDiagnosticsFrame frame{};
  set_header(frame.header, 2000001, "diagnostics_frame");
  frame.num_joints = 2;
  frame.joints = entries.data();
  frame.joints_len = entries.size();
  frame.comm.age_ms = 65535;
  frame.comm.e2e_lost = 999999;
  wuji_msgs::msg::JointDiagnosticsArray output;

  const auto status = to_joint_diagnostics(frame, JointMapping{HandSide::kRight}, output);

  ASSERT_TRUE(status) << status.message;
  EXPECT_EQ(output.header.stamp.sec, 2);
  EXPECT_EQ(output.header.stamp.nanosec, 1000U);
  EXPECT_EQ(output.header.frame_id, "diagnostics_frame");
  ASSERT_EQ(output.joints.size(), 2U);
  EXPECT_EQ(output.joints[0].nid, 6);
  EXPECT_EQ(output.joints[0].name, "r_index_finger_mcp_flex");
  EXPECT_EQ(output.joints[0].status_word, 0x12345678U);
  EXPECT_FLOAT_EQ(output.joints[0].current, 1.5F);
  EXPECT_FLOAT_EQ(output.joints[0].bus_voltage, 24.25F);
  EXPECT_FLOAT_EQ(output.joints[0].mcu_temperature, 36.5F);
  EXPECT_EQ(output.joints[0].error_code, 0x2345U);
  EXPECT_EQ(output.joints[0].comm_response_rate_pct, 98);
  EXPECT_EQ(output.joints[0].comm_timeout_total, 1234U);
  EXPECT_EQ(output.joints[1].name, "r_pinky_dip");
}

TEST(ImuConversion, MapsSdkFieldsAndMarksOrientationUnavailable) {
  WujiImuData frame{};
  set_header(frame.header, 3000002, "imu/sdk");
  frame.orientation = WujiQuaternionF64{0.1, 0.2, 0.3, 0.4};
  frame.angular_velocity = WujiVector3F64{1.1, 2.2, 3.3};
  frame.linear_acceleration = WujiVector3F64{4.4, 5.5, 6.6};
  for (std::size_t index = 0; index < 9; ++index) {
    frame.orientation_covariance[index] = 10.0 + static_cast<double>(index);
    frame.angular_velocity_covariance[index] = 20.0 + static_cast<double>(index);
    frame.linear_acceleration_covariance[index] = 30.0 + static_cast<double>(index);
  }
  sensor_msgs::msg::Imu output;

  const auto status = to_imu(frame, output);

  ASSERT_TRUE(status) << status.message;
  EXPECT_EQ(output.header.stamp.sec, 3);
  EXPECT_EQ(output.header.stamp.nanosec, 2000U);
  EXPECT_EQ(output.header.frame_id, "imu/sdk");
  EXPECT_DOUBLE_EQ(output.orientation.x, 0.1);
  EXPECT_DOUBLE_EQ(output.orientation.y, 0.2);
  EXPECT_DOUBLE_EQ(output.orientation.z, 0.3);
  EXPECT_DOUBLE_EQ(output.orientation.w, 0.4);
  EXPECT_DOUBLE_EQ(output.orientation_covariance[0], -1.0);
  for (std::size_t index = 1; index < 9; ++index) {
    EXPECT_DOUBLE_EQ(output.orientation_covariance[index], 10.0 + static_cast<double>(index));
  }
  EXPECT_DOUBLE_EQ(output.angular_velocity.x, 1.1);
  EXPECT_DOUBLE_EQ(output.angular_velocity.y, 2.2);
  EXPECT_DOUBLE_EQ(output.angular_velocity.z, 3.3);
  EXPECT_DOUBLE_EQ(output.linear_acceleration.x, 4.4);
  EXPECT_DOUBLE_EQ(output.linear_acceleration.y, 5.5);
  EXPECT_DOUBLE_EQ(output.linear_acceleration.z, 6.6);
  for (std::size_t index = 0; index < 9; ++index) {
    EXPECT_DOUBLE_EQ(output.angular_velocity_covariance[index], 20.0 + static_cast<double>(index));
    EXPECT_DOUBLE_EQ(output.linear_acceleration_covariance[index],
                     30.0 + static_cast<double>(index));
  }
}

TEST(ImuConversion, RejectsNonfiniteFieldsWithoutChangingOutput) {
  WujiImuData frame{};
  set_header(frame.header, 1, "imu");
  frame.orientation.w = 1.0;
  sensor_msgs::msg::Imu output;
  const auto reject = [&]() {
    output = sentinel_imu();
    EXPECT_FALSE(to_imu(frame, output));
    EXPECT_EQ(output.header.frame_id, "unchanged");
    EXPECT_DOUBLE_EQ(output.orientation.w, 9.0);
    EXPECT_DOUBLE_EQ(output.angular_velocity.x, 8.0);
    EXPECT_DOUBLE_EQ(output.linear_acceleration.z, 7.0);
  };

  frame.orientation.x = std::numeric_limits<double>::quiet_NaN();
  reject();
  frame.orientation.x = 0.0;
  frame.orientation_covariance[8] = std::numeric_limits<double>::infinity();
  reject();
  frame.orientation_covariance[8] = 0.0;
  frame.angular_velocity.y = -std::numeric_limits<double>::infinity();
  reject();
  frame.angular_velocity.y = 0.0;
  frame.angular_velocity_covariance[2] = std::numeric_limits<double>::quiet_NaN();
  reject();
  frame.angular_velocity_covariance[2] = 0.0;
  frame.linear_acceleration.z = std::numeric_limits<double>::infinity();
  reject();
  frame.linear_acceleration.z = 0.0;
  frame.linear_acceleration_covariance[4] = std::numeric_limits<double>::quiet_NaN();
  reject();
}

}  // namespace
}  // namespace wuji_hand2_driver::conversion
