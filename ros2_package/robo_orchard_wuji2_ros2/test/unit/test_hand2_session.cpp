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
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
// THE SOFTWARE.

#include <gtest/gtest.h>
#include <wuji_sdk.h>

#include <algorithm>
#include <array>
#include <chrono>
#include <cstddef>
#include <condition_variable>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "fake_sdk/fake_hand2_sdk.hpp"
#include "wuji_hand2_driver/sdk/hand2_sdk_api.hpp"
#include "wuji_hand2_driver/sdk/hand2_session.hpp"

namespace wuji_hand2_driver::test {
namespace {

sdk::Hand2ConnectConfig endpoint_config() {
  sdk::Hand2ConnectConfig config;
  config.endpoint = "192.0.2.10:7447";
  config.connect_timeout = 2.5;
  return config;
}

sdk::Hand2ConnectConfig serial_config() {
  sdk::Hand2ConnectConfig config;
  config.serial_number = "WH2J000000000001";
  config.connect_timeout = 2.5;
  return config;
}

extern "C" void alternate_shutdown_for_test() {}

TEST(Hand2SdkApi, ProductionTablePointsDirectlyAtSdkFunctions) {
  const auto api = sdk::Hand2SdkApi::production();
  EXPECT_EQ(api.init, &wuji_init);
  EXPECT_EQ(api.shutdown, &wuji_shutdown);
  EXPECT_EQ(api.last_error, &wuji_last_error);
  EXPECT_EQ(api.connect_options_default, &wuji_connect_options_default);
  EXPECT_EQ(api.scan, &wuji_scan);
  EXPECT_EQ(api.discovered_free, &wuji_discovered_free);
  EXPECT_EQ(api.connect, &wuji_connect);
  EXPECT_EQ(api.disconnect, &wuji_dev_disconnect);
  EXPECT_EQ(api.release, &wuji_dev_release);
  EXPECT_EQ(api.device_serial_number, &wuji_dev_serial_number);
  EXPECT_EQ(api.device_model, &wuji_dev_model);
  EXPECT_EQ(api.device_is_connected, &wuji_dev_is_connected);
  EXPECT_EQ(api.sub_close, &wuji_sub_close);
  EXPECT_EQ(api.subscribe_joint_states, &wuji_hand_2_subscribe_joint_states);
  EXPECT_EQ(api.subscribe_joint_diagnostics, &wuji_hand_2_subscribe_joint_diagnostics);
  EXPECT_EQ(api.subscribe_imu, &wuji_hand_2_subscribe_imu);
  EXPECT_EQ(api.subscribe_fingertip[0], &wuji_hand_2_subscribe_fingertip_thumb_data);
  EXPECT_EQ(api.subscribe_fingertip[1], &wuji_hand_2_subscribe_fingertip_index_data);
  EXPECT_EQ(api.subscribe_fingertip[2], &wuji_hand_2_subscribe_fingertip_middle_data);
  EXPECT_EQ(api.subscribe_fingertip[3], &wuji_hand_2_subscribe_fingertip_ring_data);
  EXPECT_EQ(api.subscribe_fingertip[4], &wuji_hand_2_subscribe_fingertip_pinky_data);
  EXPECT_EQ(api.get_fingertip_info[0], &wuji_hand_2_get_fingertip_thumb_info);
  EXPECT_EQ(api.get_fingertip_info[1], &wuji_hand_2_get_fingertip_index_info);
  EXPECT_EQ(api.get_fingertip_info[2], &wuji_hand_2_get_fingertip_middle_info);
  EXPECT_EQ(api.get_fingertip_info[3], &wuji_hand_2_get_fingertip_ring_info);
  EXPECT_EQ(api.get_fingertip_info[4], &wuji_hand_2_get_fingertip_pinky_info);
  EXPECT_EQ(api.fingertip_online, &wuji_hand_2_fingertip_online);
  EXPECT_EQ(api.get_handedness, &wuji_hand_2_get_handedness);
  EXPECT_EQ(api.get_hw_version, &wuji_hand_2_get_hw_version);
  EXPECT_EQ(api.enable, &wuji_hand_2_enable);
  EXPECT_EQ(api.disable, &wuji_hand_2_disable);
  EXPECT_EQ(api.clear_fault, &wuji_hand_2_clear_fault);
  EXPECT_EQ(api.emergency_stop, &wuji_hand_2_emergency_stop);
  EXPECT_EQ(api.set_effort_limit, &wuji_hand_2_set_all_effort_limit);
  EXPECT_EQ(api.set_effort_limits, &wuji_hand_2_set_all_effort_limit_per_joint);
  EXPECT_EQ(api.set_mit_params, &wuji_hand_2_set_all_mit_params);
  EXPECT_EQ(api.calibrate_fingertip, &wuji_hand_2_calibrate_fingertip);
  EXPECT_EQ(api.get_fingertip_status[0], &wuji_hand_2_get_fingertip_thumb_status);
  EXPECT_EQ(api.get_fingertip_status[1], &wuji_hand_2_get_fingertip_index_status);
  EXPECT_EQ(api.get_fingertip_status[2], &wuji_hand_2_get_fingertip_middle_status);
  EXPECT_EQ(api.get_fingertip_status[3], &wuji_hand_2_get_fingertip_ring_status);
  EXPECT_EQ(api.get_fingertip_status[4], &wuji_hand_2_get_fingertip_pinky_status);
  EXPECT_EQ(api.open_joint_command_publisher, &wuji_hand_2_joint_command_publish);
  EXPECT_EQ(api.send_joint_commands, &wuji_joint_command_publisher_send);
  EXPECT_EQ(api.close_joint_command_publisher, &wuji_joint_command_publisher_close);
}

TEST(Hand2Session, InitializesBeforeFirstConnectAndShutsDownAfterLastSession) {
  FakeHand2Sdk fake;
  sdk::Hand2Session first(fake.api());
  sdk::Hand2Session second(fake.api());

  ASSERT_TRUE(first.connect(endpoint_config()));
  auto second_config = endpoint_config();
  second_config.endpoint = "192.0.2.11:7447";
  ASSERT_TRUE(second.connect(second_config));

  const auto first_init = std::find(fake.calls().begin(), fake.calls().end(), "init");
  const auto first_connect = std::find(fake.calls().begin(), fake.calls().end(), "connect");
  ASSERT_NE(first_init, fake.calls().end());
  ASSERT_NE(first_connect, fake.calls().end());
  EXPECT_LT(first_init, first_connect);
  EXPECT_EQ(fake.count("init"), 1U);

  first.disconnect();
  EXPECT_EQ(fake.count("shutdown"), 0U);
  second.disconnect();
  EXPECT_EQ(fake.count("shutdown"), 1U);
}

TEST(Hand2Session, ConcurrentSessionsBalanceOneProcessRuntime) {
  FakeHand2Sdk fake;
  sdk::Hand2Session first(fake.api());
  sdk::Hand2Session second(fake.api());
  const std::array<sdk::Hand2Session*, 2> sessions{&first, &second};
  std::array<sdk::Status, 2> results{sdk::Status::failure("not run"),
                                     sdk::Status::failure("not run")};
  std::mutex mutex;
  std::condition_variable condition;
  std::size_t ready{0};
  std::size_t connect_finished{0};
  bool start{false};
  bool release{false};

  const auto worker = [&](std::size_t index) {
    {
      std::unique_lock<std::mutex> lock(mutex);
      ++ready;
      condition.notify_all();
      condition.wait(lock, [&]() { return start; });
    }
    auto config = endpoint_config();
    config.endpoint = index == 0 ? "192.0.2.20:7447" : "192.0.2.21:7447";
    results[index] = sessions[index]->connect(config);
    {
      std::unique_lock<std::mutex> lock(mutex);
      ++connect_finished;
      condition.notify_all();
      condition.wait(lock, [&]() { return release; });
    }
    sessions[index]->disconnect();
  };

  std::thread first_thread(worker, 0);
  std::thread second_thread(worker, 1);
  bool both_ready{false};
  bool both_connected{false};
  {
    std::unique_lock<std::mutex> lock(mutex);
    both_ready = condition.wait_for(lock, std::chrono::seconds{2},
                                    [&]() { return ready == sessions.size(); });
    start = true;
  }
  condition.notify_all();
  {
    std::unique_lock<std::mutex> lock(mutex);
    both_connected = condition.wait_for(lock, std::chrono::seconds{2},
                                        [&]() { return connect_finished == sessions.size(); });
    release = true;
  }
  condition.notify_all();
  first_thread.join();
  second_thread.join();

  EXPECT_TRUE(both_ready);
  EXPECT_TRUE(both_connected);
  EXPECT_TRUE(results[0]);
  EXPECT_TRUE(results[1]);
  EXPECT_EQ(fake.count("init"), 1U);
  EXPECT_EQ(fake.count("connect"), 2U);
  EXPECT_EQ(fake.count("close_joint_command_publisher"), 2U);
  EXPECT_EQ(fake.count("disconnect"), 2U);
  EXPECT_EQ(fake.count("release"), 2U);
  EXPECT_EQ(fake.count("shutdown"), 1U);
}

TEST(Hand2Session, DisconnectFailureStillReleasesDeviceAndRuntime) {
  FakeHand2Sdk fake;
  fake.set_disconnect_failure(WUJI_STATUS_ERR_DISCONNECTED, "disconnect failed");
  {
    sdk::Hand2Session first(fake.api());
    ASSERT_TRUE(first.connect(endpoint_config()));
    first.disconnect();
  }
  EXPECT_EQ(fake.count("disconnect"), 1U);
  EXPECT_EQ(fake.count("release"), 1U);
  EXPECT_EQ(fake.count("shutdown"), 1U);
  fake.set_disconnect_failure(WUJI_STATUS_OK, {});

  {
    sdk::Hand2Session second(fake.api());
    ASSERT_TRUE(second.connect(endpoint_config()));
    second.disconnect();
  }

  EXPECT_EQ(fake.count("init"), 2U);
  EXPECT_EQ(fake.count("disconnect"), 2U);
  EXPECT_EQ(fake.count("release"), 2U);
  EXPECT_EQ(fake.count("shutdown"), 2U);
}

TEST(Hand2Session, EndpointTakesPriorityAndSerialNumberValidatesConnectedDevice) {
  FakeHand2Sdk fake;
  sdk::Hand2Session session(fake.api());
  auto config = endpoint_config();
  config.serial_number = "WH2J000000000001";

  ASSERT_TRUE(session.connect(config));
  ASSERT_EQ(fake.connect_calls().size(), 1U);
  EXPECT_EQ(fake.connect_calls().front().target_kind, WUJI_CONNECT_TARGET_KIND_ADDR);
  EXPECT_EQ(fake.connect_calls().front().target_value, config.endpoint);
  EXPECT_FALSE(fake.connect_calls().front().options.enable_bridge);
  EXPECT_EQ(session.serial_number(), config.serial_number);
}

TEST(Hand2Session, RejectsSerialMismatchAndReleasesConnectedDevice) {
  FakeHand2Sdk fake;
  fake.set_serial_number("WH2K000000000002");
  sdk::Hand2Session session(fake.api());
  auto config = endpoint_config();
  config.serial_number = "WH2J000000000001";

  const auto status = session.connect(config);

  EXPECT_FALSE(status);
  EXPECT_EQ(status.message,
            "connected serial_number is WH2K000000000002, expected WH2J000000000001");
  EXPECT_EQ(fake.count("open_joint_command_publisher"), 0U);
  EXPECT_EQ(fake.count("disconnect"), 1U);
  EXPECT_EQ(fake.count("release"), 1U);
}

TEST(Hand2Session, RejectsNonHand2ModelAndRollsBack) {
  FakeHand2Sdk fake;
  fake.set_model("0x0003");
  sdk::Hand2Session session(fake.api());

  const auto status = session.connect(serial_config());

  EXPECT_FALSE(status);
  EXPECT_EQ(status.message, "connected model is 0x0003, expected 0x0002");
  EXPECT_EQ(fake.count("device_model"), 3U);
  EXPECT_EQ(fake.count("open_joint_command_publisher"), 0U);
  EXPECT_EQ(fake.count("disconnect"), 1U);
  EXPECT_EQ(fake.count("release"), 1U);
}

TEST(Hand2Session, ValidatesHandSideAfterConnect) {
  FakeHand2Sdk fake;
  fake.set_handedness(WUJI_HANDEDNESS_RIGHT);
  sdk::Hand2Session session(fake.api());
  auto config = serial_config();
  config.hand_side = "left";

  const auto status = session.connect(config);

  EXPECT_FALSE(status);
  EXPECT_EQ(status.message, "connected hand_side is right, expected left");
  EXPECT_EQ(fake.count("get_handedness"), 1U);
  EXPECT_EQ(fake.count("disconnect"), 1U);
  EXPECT_EQ(fake.count("release"), 1U);
}

TEST(Hand2Session, ClosesCommandPublisherBeforeDisconnect) {
  FakeHand2Sdk fake;
  sdk::Hand2Session session(fake.api());
  ASSERT_TRUE(session.connect(serial_config()));

  session.disconnect();

  const std::vector<std::string> expected{"init",
                                          "connect_options_default",
                                          "connect",
                                          "device_serial_number",
                                          "device_serial_number",
                                          "device_serial_number",
                                          "device_model",
                                          "device_model",
                                          "device_model",
                                          "device_is_connected",
                                          "get_handedness",
                                          "open_joint_command_publisher",
                                          "close_joint_command_publisher",
                                          "disconnect",
                                          "release",
                                          "shutdown"};
  EXPECT_EQ(fake.calls(), expected);
}

TEST(Hand2Session, RollsBackPublisherReturnedByFailedOpen) {
  FakeHand2Sdk fake;
  fake.set_open_publisher_failure(WUJI_STATUS_ERR_INTERNAL, "publisher failed", true);
  sdk::Hand2Session session(fake.api());

  const auto status = session.connect(serial_config());

  EXPECT_FALSE(status);
  EXPECT_EQ(status.message,
            "wuji_hand_2_joint_command_publish failed (status -99): publisher failed");
  const auto error = std::find(fake.calls().begin(), fake.calls().end(), "last_error");
  const auto close =
      std::find(fake.calls().begin(), fake.calls().end(), "close_joint_command_publisher");
  const auto disconnect = std::find(fake.calls().begin(), fake.calls().end(), "disconnect");
  const auto release = std::find(fake.calls().begin(), fake.calls().end(), "release");
  ASSERT_NE(error, fake.calls().end());
  ASSERT_NE(close, fake.calls().end());
  ASSERT_NE(disconnect, fake.calls().end());
  ASSERT_NE(release, fake.calls().end());
  EXPECT_LT(error, close);
  EXPECT_LT(close, disconnect);
  EXPECT_LT(disconnect, release);
}

TEST(Hand2Session, SendsExactlyTwentyJointCommands) {
  FakeHand2Sdk fake;
  sdk::Hand2Session session(fake.api());
  ASSERT_TRUE(session.connect(serial_config()));
  std::array<WujiJointCommand, WUJI_HAND_2_JOINT_COUNT> commands{};
  for (std::size_t index = 0; index < commands.size(); ++index) {
    commands[index] = WujiJointCommand{static_cast<float>(index), static_cast<float>(index + 20),
                                       static_cast<float>(index + 40)};
  }

  ASSERT_TRUE(session.send(commands));

  ASSERT_EQ(fake.sent_commands().size(), 1U);
  for (std::size_t index = 0; index < commands.size(); ++index) {
    EXPECT_FLOAT_EQ(fake.sent_commands().front()[index].position, commands[index].position);
    EXPECT_FLOAT_EQ(fake.sent_commands().front()[index].velocity, commands[index].velocity);
    EXPECT_FLOAT_EQ(fake.sent_commands().front()[index].effort, commands[index].effort);
  }
}

TEST(Hand2Session, DisableAllUsesWholeHandCallAndReturnsCopiedFailure) {
  FakeHand2Sdk fake;
  fake.set_disable_failure(WUJI_STATUS_ERR_DISCONNECTED, "device left the network");
  sdk::Hand2Session session(fake.api());
  ASSERT_TRUE(session.connect(serial_config()));

  const auto status = session.disable_all();
  fake.overwrite_last_error("overwritten after return");

  EXPECT_FALSE(status);
  EXPECT_TRUE(fake.last_disable_used_whole_hand());
  EXPECT_EQ(status.message, "wuji_hand_2_disable failed (status -6): device left the network");
}

}  // namespace
}  // namespace wuji_hand2_driver::test
