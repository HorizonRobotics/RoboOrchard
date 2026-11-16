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

#include <wuji_sdk.h>

#include <array>
#include <atomic>
#include <chrono>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <memory>
#include <string>

#include <rclcpp/executors/multi_threaded_executor.hpp>
#include <rclcpp/rclcpp.hpp>

#include "fake_sdk/fake_hand2_sdk.hpp"
#include "wuji_hand2_driver/hand2_node.hpp"

namespace {

using namespace std::chrono_literals;

struct JointSource {
  JointSource() {
    constexpr std::array<std::uint8_t, WUJI_HAND_2_JOINT_COUNT> nids{
        {1U,  2U,  3U,  4U,  6U,  7U,  8U,  9U,  11U, 12U,
         13U, 14U, 16U, 17U, 18U, 19U, 21U, 22U, 23U, 24U}};
    for (std::size_t index = 0; index < joints.size(); ++index) {
      joints[index] = WujiJointStateEntry{nids[index], 0.0F, 0.0F, 0.0F};
    }
    frame.num_joints = static_cast<std::uint8_t>(joints.size());
    frame.joints = joints.data();
    frame.joints_len = joints.size();
    std::snprintf(frame.header.frame_id, sizeof(frame.header.frame_id), "hand");
  }

  void emit(wuji_hand2_driver::test::FakeHand2Sdk& fake) {
    ++frame.header.seq;
    frame.header.timestamp_us += 10000U;
    fake.emit_joint_state(WUJI_FRAME_KIND_OK, &frame);
  }

  std::array<WujiJointStateEntry, WUJI_HAND_2_JOINT_COUNT> joints{};
  WujiJointStateFrame frame{};
};

bool terminal_mode_requested() noexcept {
  const char* mode = std::getenv("WUJI_HAND2_TEST_MODE");
  return mode != nullptr && std::strcmp(mode, "terminal") == 0;
}

void print_calls(const wuji_hand2_driver::test::FakeHand2Sdk& fake) {
  const auto calls = fake.calls_snapshot();
  std::fputs("WUJI_HAND2_TEST_TERMINAL_CALLS=", stderr);
  for (std::size_t index = 0; index < calls.size(); ++index) {
    std::fprintf(stderr, "%s%s", index == 0 ? "" : ",", calls[index].c_str());
  }
  std::fputc('\n', stderr);
}
}  // namespace

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  wuji_hand2_driver::test::FakeHand2Sdk fake;
  rclcpp::NodeOptions options;
  options.parameter_overrides({rclcpp::Parameter("serial_number", "WH2J000000000001")});
  const bool terminal_mode = terminal_mode_requested();
  auto node = std::make_shared<wuji_hand2_driver::Hand2Node>(
      fake.api(), wuji_hand2_driver::Hand2NodeHooks{}, options);
  const auto started = node->start();
  if (!started) {
    node->stop();
    rclcpp::shutdown();
    return EXIT_FAILURE;
  }

  auto source = std::make_shared<JointSource>();
  std::atomic<bool> terminal_emitted{false};
  const auto timer =
      node->create_wall_timer(10ms, [source, &fake, terminal_mode, &terminal_emitted]() {
        if (!terminal_mode) {
          source->emit(fake);
        } else if (!terminal_emitted.exchange(true, std::memory_order_acq_rel)) {
          fake.emit_joint_state(WUJI_FRAME_KIND_ERROR, nullptr);
        }
      });
  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions{}, 3U);
  executor.add_node(node);
  executor.spin();
  executor.remove_node(node);
  static_cast<void>(timer);
  node->stop();
  if (terminal_mode) {
    print_calls(fake);
  }
  if (rclcpp::ok()) {
    rclcpp::shutdown();
  }
  return node->exit_code();
}
