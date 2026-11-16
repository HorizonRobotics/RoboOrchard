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

#include <cstdlib>
#include <exception>
#include <memory>

#include <rclcpp/executors/multi_threaded_executor.hpp>
#include <rclcpp/rclcpp.hpp>

#include "wuji_hand2_driver/hand2_node.hpp"

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  const auto logger = rclcpp::get_logger("wuji_hand2_driver");
  std::shared_ptr<wuji_hand2_driver::Hand2Node> node;
  try {
    node = std::make_shared<wuji_hand2_driver::Hand2Node>();
  } catch (const rclcpp::exceptions::InvalidParameterTypeException& error) {
    RCLCPP_ERROR(logger, "invalid parameter: %s", error.what());
    rclcpp::shutdown();
    return EXIT_FAILURE;
  } catch (const std::exception& error) {
    // Ctrl+C before the node exists invalidates the context under its constructor.
    if (!rclcpp::ok()) {
      RCLCPP_INFO(logger, "startup cancelled by shutdown request");
      return EXIT_SUCCESS;
    }
    RCLCPP_ERROR(logger, "Hand2Node construction failed: %s", error.what());
    rclcpp::shutdown();
    return EXIT_FAILURE;
  }
  const auto started = node->start();
  if (!started) {
    node->stop();
    // A start cancelled by Ctrl+C or SIGTERM records EXIT_SUCCESS.
    const int exit_code = node->exit_requested() ? node->exit_code() : EXIT_FAILURE;
    rclcpp::shutdown();
    return exit_code;
  }

  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions{}, 3U);
  executor.add_node(node);
  executor.spin();
  executor.remove_node(node);
  node->stop();
  const int exit_code = node->exit_code();
  if (rclcpp::ok()) {
    rclcpp::shutdown();
  }
  return exit_code;
}
