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

#include <atomic>
#include <chrono>
#include <cstddef>
#include <cstdlib>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include "fake_sdk/fake_hand2_sdk.hpp"
#include "wuji_hand2_driver/hand2_node.hpp"

namespace wuji_hand2_driver {
namespace {

using namespace std::chrono_literals;
using test::FakeHand2Sdk;
using Clock = Hand2NodeHooks::Clock;

// Holds an SDK call inside the fake until release(); the test observes the entry.
class Hand2StartupTest : public testing::Test {
 protected:
  static void SetUpTestSuite() {
    int argc = 0;
    char** argv = nullptr;
    rclcpp::init(argc, argv);
  }

  static void TearDownTestSuite() {
    rclcpp::shutdown();
  }

  std::shared_ptr<Hand2Node> make_node(std::vector<rclcpp::Parameter> overrides = {}) {
    overrides.emplace_back("serial_number", "WH2J000000000001");
    rclcpp::NodeOptions options;
    options.parameter_overrides(overrides);
    Hand2NodeHooks hooks;
    hooks.now = [this]() { return now(); };
    hooks.sleep_for = [this](Clock::duration duration) { advance(duration); };
    hooks.cancelled = [this]() {
      return cancelled_.load(std::memory_order_acquire) ||
             (cancel_after_imu_subscribe_ && fake_.count("subscribe_imu") > 0U);
    };
    hooks.request_shutdown = []() {};
    return std::make_shared<Hand2Node>(fake_.api(), std::move(hooks), options);
  }

  Clock::time_point now() const {
    const std::lock_guard<std::mutex> lock(clock_mutex_);
    return now_;
  }

  void advance(Clock::duration duration) {
    const std::lock_guard<std::mutex> lock(clock_mutex_);
    now_ += duration;
  }

  FakeHand2Sdk fake_;
  mutable std::mutex clock_mutex_;
  Clock::time_point now_{Clock::now()};
  std::atomic<bool> cancelled_{false};
  bool cancel_after_imu_subscribe_{false};
};

TEST_F(Hand2StartupTest, ShutdownRequestDuringHandednessStopsStartOnceTheCallReturns) {
  fake_.set_handedness_hook([this]() { cancelled_.store(true, std::memory_order_release); });
  auto node = make_node({rclcpp::Parameter("auto_enable", true)});

  const auto status = node->start();

  EXPECT_FALSE(status);
  EXPECT_NE(status.message.find("cancelled"), std::string::npos);
  EXPECT_EQ(node->exit_code(), EXIT_SUCCESS);
  EXPECT_NE(node->exit_reason().find("startup cancelled by shutdown request"), std::string::npos);
  EXPECT_EQ(fake_.count("enable"), 0U);
  EXPECT_EQ(fake_.count("disconnect"), 1U);
  EXPECT_EQ(fake_.count("release"), 1U);
}

TEST_F(Hand2StartupTest, ShutdownRequestBeforeAutoEnableSkipsEnable) {
  cancel_after_imu_subscribe_ = true;
  auto node = make_node({rclcpp::Parameter("auto_enable", true)});

  const auto status = node->start();

  EXPECT_FALSE(status);
  EXPECT_NE(status.message.find("shutdown requested before enable"), std::string::npos);
  EXPECT_EQ(fake_.count("enable"), 0U);
  EXPECT_EQ(fake_.count("disable"), 1U);
  EXPECT_EQ(fake_.count("disconnect"), 1U);
}

TEST_F(Hand2StartupTest, ShutdownDisableWaitsForTheDeviceDespiteThePendingShutdownRequest) {
  // Ctrl+C is what brings stop() here, so the shutdown request must not cut the
  // best-effort disable short. Only the deadline may.
  std::atomic<bool> disable_finished{false};
  fake_.set_disable_hook([&disable_finished]() {
    std::this_thread::sleep_for(150ms);
    disable_finished.store(true, std::memory_order_release);
  });
  auto node = make_node();
  ASSERT_TRUE(node->start());
  cancelled_.store(true, std::memory_order_release);

  node->stop();

  EXPECT_TRUE(disable_finished.load(std::memory_order_acquire));
  EXPECT_EQ(fake_.count("disable"), 1U);
  EXPECT_EQ(fake_.count("disconnect"), 1U);
  EXPECT_EQ(fake_.count("release"), 1U);
  EXPECT_EQ(fake_.count("init"), fake_.count("shutdown"));
}

TEST_F(Hand2StartupTest, StartupFailureDisableWaitsForTheDeviceAfterAShutdownRequest) {
  fake_.set_enable_failure(WUJI_STATUS_ERR_TIMEOUT, "Operation timeout - device not responding");
  fake_.set_enable_hook([this]() { cancelled_.store(true, std::memory_order_release); });
  std::atomic<bool> disable_finished{false};
  fake_.set_disable_hook([&disable_finished]() {
    std::this_thread::sleep_for(150ms);
    disable_finished.store(true, std::memory_order_release);
  });
  auto node = make_node({rclcpp::Parameter("auto_enable", true)});

  EXPECT_FALSE(node->start());

  EXPECT_TRUE(disable_finished.load(std::memory_order_acquire));
  EXPECT_EQ(fake_.count("disconnect"), 1U);
}

TEST_F(Hand2StartupTest, AutoEnableRetriesTimeoutAndSessionErrorsThenSucceeds) {
  fake_.set_enable_failure(WUJI_STATUS_ERR_TIMEOUT, "Operation timeout - device not responding");
  std::atomic<std::size_t> enable_calls{0};
  // The fake captures the failure before running the hook, so each hook call shapes the
  // reply of the following attempt.
  fake_.set_enable_hook([this, &enable_calls]() {
    const auto call = ++enable_calls;
    if (call == 1U) {
      fake_.set_enable_failure(WUJI_STATUS_ERR_INTERNAL,
                               "Device returned error: Session not found (0x0011)");
    } else if (call == 2U) {
      fake_.set_enable_failure(WUJI_STATUS_OK, "");
    }
  });
  auto node = make_node({rclcpp::Parameter("auto_enable", true)});

  ASSERT_TRUE(node->start());

  EXPECT_EQ(fake_.count("enable"), 3U);
  EXPECT_FALSE(node->exit_requested());
  node->stop();
  EXPECT_EQ(fake_.count("disable"), 1U);
  EXPECT_EQ(fake_.count("disconnect"), fake_.count("release"));
}

TEST_F(Hand2StartupTest, AutoEnableGivesUpAfterThreeAttemptsAndStillDisables) {
  fake_.set_enable_failure(WUJI_STATUS_ERR_TIMEOUT, "Operation timeout - device not responding");
  auto node = make_node({rclcpp::Parameter("auto_enable", true)});

  EXPECT_FALSE(node->start());

  EXPECT_EQ(fake_.count("enable"), 3U);
  EXPECT_NE(node->exit_reason().find("wuji_hand_2_enable"), std::string::npos);
  EXPECT_EQ(node->exit_code(), EXIT_FAILURE);
  EXPECT_EQ(fake_.count("disable"), 1U);
  EXPECT_EQ(fake_.count("disconnect"), fake_.count("release"));
  EXPECT_EQ(fake_.count("init"), fake_.count("shutdown"));
}

TEST_F(Hand2StartupTest, AutoEnableStopsRetryingOnceTheSdkDroppedTheSession) {
  fake_.set_enable_failure(WUJI_STATUS_ERR_INTERNAL, "Session not found (0x0011)");
  fake_.set_enable_hook([this]() { fake_.set_connected(false); });
  auto node = make_node({rclcpp::Parameter("auto_enable", true)});

  EXPECT_FALSE(node->start());

  EXPECT_EQ(fake_.count("enable"), 1U);
  EXPECT_NE(node->exit_reason().find("Session not found"), std::string::npos);
}

TEST_F(Hand2StartupTest, AutoEnableDoesNotRetryNonTransientErrors) {
  fake_.set_enable_failure(WUJI_STATUS_ERR_UNSUPPORTED, "unsupported");
  auto node = make_node({rclcpp::Parameter("auto_enable", true)});

  EXPECT_FALSE(node->start());

  EXPECT_EQ(fake_.count("enable"), 1U);
}

TEST_F(Hand2StartupTest, ShutdownRequestDuringEnableRetryDelayStopsRetrying) {
  fake_.set_enable_failure(WUJI_STATUS_ERR_TIMEOUT, "Operation timeout - device not responding");
  fake_.set_enable_hook([this]() { cancelled_.store(true, std::memory_order_release); });
  auto node = make_node({rclcpp::Parameter("auto_enable", true)});

  EXPECT_FALSE(node->start());

  EXPECT_EQ(fake_.count("enable"), 1U);
  EXPECT_NE(node->exit_reason().find("shutdown requested"), std::string::npos);
  EXPECT_EQ(node->exit_code(), EXIT_SUCCESS);
  EXPECT_EQ(fake_.count("disable"), 1U);
}

}  // namespace
}  // namespace wuji_hand2_driver
