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

#include "wuji_hand2_driver/hand2_node.hpp"

#include <wuji_sdk.h>

#include <algorithm>
#include <atomic>
#include <array>
#include <cmath>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <exception>
#include <limits>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include <rclcpp/callback_group.hpp>
#include <rclcpp/context.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/timer.hpp>

#include <wuji_driver_common/stream/stream_registry.hpp>
#include <wuji_driver_common/stream/stream_types.hpp>

#include "wuji_hand2_driver/control/control_plane.hpp"
#include "wuji_hand2_driver/control/operation_context.hpp"
#include "wuji_hand2_driver/conversion/joint_mapping.hpp"
#include "wuji_hand2_driver/driver_config.hpp"
#include "wuji_hand2_driver/lifecycle/exit_state.hpp"
#include "wuji_hand2_driver/sdk/hand2_session.hpp"
#include "wuji_hand2_driver/sdk/sdk_status.hpp"
#include "wuji_hand2_driver/stream/fingertip_stream.hpp"
#include "wuji_hand2_driver/stream/imu_stream.hpp"
#include "wuji_hand2_driver/stream/joint_diagnostics_stream.hpp"
#include "wuji_hand2_driver/stream/joint_state_stream.hpp"

namespace wuji_hand2_driver {
namespace {

using namespace std::chrono_literals;
using wuji_driver_common::stream::StreamKey;
using wuji_driver_common::stream::StreamPolicy;
using wuji_driver_common::stream::StreamSnapshot;
using wuji_driver_common::stream::StreamState;
using wuji_driver_common::stream::TerminalKind;

constexpr char kScannedHand2Model[] = "WujiHand2";
constexpr auto kDiscoveryPollInterval = 100ms;
constexpr auto kFaultBridgePeriod = 50ms;
constexpr std::size_t kEnableAttempts = 3;
constexpr auto kEnableRetryDelay = 500ms;

constexpr StreamKey kJointStateKey{0U};
constexpr StreamKey kJointDiagnosticsKey{1U};
constexpr StreamKey kImuKey{2U};
constexpr std::array<StreamKey, 5> kFingertipKeys{
    {StreamKey{3U}, StreamKey{4U}, StreamKey{5U}, StreamKey{6U}, StreamKey{7U}}};

struct Discovery {
  std::string serial_number;
  std::string address;
  WujiDeviceType device_id{WUJI_DEVICE_TYPE_UNKNOWN};
  WujiTransportType transport{};
};

std::string physical_identity(const Discovery& discovery) {
  if (!discovery.serial_number.empty()) {
    return "serial:" + discovery.serial_number;
  }
  return "fallback:" + std::to_string(static_cast<int>(discovery.transport)) + ":" +
         discovery.address + ":" + std::to_string(static_cast<int>(discovery.device_id));
}

template <std::size_t Size>
std::string fixed_string(const char (&input)[Size]) {
  const auto end = std::find(input, input + Size, '\0');
  return std::string(input, end);
}

std::string exception_message(const char* boundary) {
  try {
    throw;
  } catch (const std::exception& error) {
    return std::string{boundary} + ": " + error.what();
  } catch (...) {
    return std::string{boundary} + ": unknown exception";
  }
}

const char* stream_label(StreamKey key) noexcept {
  switch (key.value) {
    case 0U:
      return "joint_states";
    case 1U:
      return "joint_diagnostics";
    case 2U:
      return "imu";
    case 3U:
      return "fingertip_tactile/thumb";
    case 4U:
      return "fingertip_tactile/index";
    case 5U:
      return "fingertip_tactile/middle";
    case 6U:
      return "fingertip_tactile/ring";
    case 7U:
      return "fingertip_tactile/pinky";
    default:
      return "unknown";
  }
}

const char* terminal_label(TerminalKind kind) noexcept {
  return kind == TerminalKind::End ? "END" : "ERROR";
}

std::string stream_fault_message(const StreamSnapshot& snapshot) {
  std::string message = stream_label(snapshot.key);
  message += " ";
  message += terminal_label(snapshot.fault.kind);
  if (!snapshot.fault.reason.empty()) {
    message += ": " + snapshot.fault.reason;
  }
  return message;
}

class ScanRuntimeGuard {
 public:
  explicit ScanRuntimeGuard(sdk::Hand2SdkApi& sdk) noexcept : sdk_(sdk) {}

  ~ScanRuntimeGuard() noexcept {
    if (active_) {
      sdk_.shutdown();
    }
  }

  void activate() noexcept {
    active_ = true;
  }

 private:
  sdk::Hand2SdkApi& sdk_;
  bool active_{false};
};

class DiscoveredGuard {
 public:
  DiscoveredGuard(sdk::Hand2SdkApi& sdk, WujiDiscovered* discovered, std::size_t count) noexcept
      : sdk_(sdk), discovered_(discovered), count_(count) {}

  ~DiscoveredGuard() noexcept {
    if (discovered_ != nullptr) {
      sdk_.discovered_free(discovered_, count_);
    }
  }

  DiscoveredGuard(const DiscoveredGuard&) = delete;
  DiscoveredGuard& operator=(const DiscoveredGuard&) = delete;

 private:
  sdk::Hand2SdkApi& sdk_;
  WujiDiscovered* discovered_;
  std::size_t count_;
};

}  // namespace

// =============================================================================
// Hand2Node::Impl Definition
// =============================================================================

class Hand2Node::Impl {
 public:
  Impl(Hand2Node& node, sdk::Hand2SdkApi sdk_api, Hand2NodeHooks hooks);

  sdk::Status start();
  void stop() noexcept;

  bool exit_requested() const noexcept;
  int exit_code() const noexcept;
  std::string exit_reason() const;

 private:
  enum class State {
    Constructed,
    Starting,
    Running,
    Stopping,
    Stopped,
  };

  struct ConnectAttempt {
    sdk::Status status;
    WujiStatus failure_status{WUJI_STATUS_OK};
    bool budget_exhausted{false};
  };

  sdk::Status fail_start(sdk::Status status);
  sdk::Status scan_once(std::vector<Discovery>& output);
  static bool retryable_connect_status(WujiStatus status) noexcept;
  sdk::Status next_attempt_timeout(Hand2NodeHooks::Clock::time_point started_at,
                                   std::uint32_t& output) const;
  sdk::Status wait_before_retry(Hand2NodeHooks::Clock::time_point started_at);
  sdk::Hand2ConnectConfig candidate_config(const Discovery& candidate, bool validate_side) const;
  ConnectAttempt connect_attempt(sdk::Hand2ConnectConfig connection, bool open_publisher,
                                 Hand2NodeHooks::Clock::time_point started_at);
  sdk::Status connect_full_with_retry(const sdk::Hand2ConnectConfig& connection,
                                      Hand2NodeHooks::Clock::time_point started_at);
  sdk::Status select_and_connect();
  static bool retryable_enable_status(WujiStatus status) noexcept;
  sdk::Status enable_with_retry();
  sdk::Status establish_identity();
  sdk::Status register_streams();
  sdk::Status start_control_plane();
  void start_fingertip_streams() noexcept;
  void start_fault_bridge();
  void request_runtime_failure(std::string reason) noexcept;
  void on_fault_timer() noexcept;

  Hand2Node& node_;
  sdk::Hand2SdkApi sdk_;
  Hand2NodeHooks hooks_;
  sdk::Hand2Session session_;
  DriverConfig config_;
  lifecycle::ExitState exit_state_;
  control::OperationGate operation_gate_;
  std::unique_ptr<conversion::JointMapping> mapping_;
  std::unique_ptr<wuji_driver_common::stream::StreamRegistry> registry_;
  std::unique_ptr<wuji_driver_common::stream::StreamRegistry> fingertip_registry_;
  std::unique_ptr<control::ControlPlane> control_plane_;
  rclcpp::CallbackGroup::SharedPtr command_group_;
  rclcpp::CallbackGroup::SharedPtr control_group_;
  rclcpp::CallbackGroup::SharedPtr emergency_group_;
  rclcpp::CallbackGroup::SharedPtr fault_group_;
  rclcpp::CallbackGroup::SharedPtr tactile_group_;
  rclcpp::TimerBase::SharedPtr fault_timer_;
  rclcpp::TimerBase::SharedPtr fingertip_timer_;
  // Guards state_. start_fingertip_streams() holds it for the whole fingertip metadata read,
  // so code that runs while the executor spins must not wait on it.
  mutable std::mutex state_mutex_;
  State state_{State::Constructed};
  std::atomic<bool> stopping_{false};
};

// =============================================================================
// Hand2Node::Impl Implementation
// =============================================================================

// -----------------------------------------------------------------------------
// Construction and Lifecycle
// -----------------------------------------------------------------------------

Hand2Node::Impl::Impl(Hand2Node& node, sdk::Hand2SdkApi sdk_api, Hand2NodeHooks hooks)
    : node_(node), sdk_(std::move(sdk_api)), hooks_(std::move(hooks)), session_(sdk_) {
  const auto context = node_.get_node_base_interface()->get_context();
  if (!hooks_.now) {
    hooks_.now = []() { return Hand2NodeHooks::Clock::now(); };
  }
  if (!hooks_.sleep_for) {
    hooks_.sleep_for = [](Hand2NodeHooks::Clock::duration duration) {
      std::this_thread::sleep_for(duration);
    };
  }
  if (!hooks_.cancelled) {
    hooks_.cancelled = [context]() { return !context->is_valid(); };
  }
  if (!hooks_.request_shutdown) {
    hooks_.request_shutdown = [context]() { rclcpp::shutdown(context); };
  }
  command_group_ = node_.create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  control_group_ = node_.create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  emergency_group_ = node_.create_callback_group(rclcpp::CallbackGroupType::Reentrant);
  fault_group_ = node_.create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  tactile_group_ = node_.create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
}

sdk::Status Hand2Node::Impl::start() {
  {
    const std::lock_guard<std::mutex> lock(state_mutex_);
    if (state_ == State::Running) {
      return sdk::Status::success();
    }
    if (state_ != State::Constructed) {
      return sdk::Status::failure("Hand2Node cannot restart after stopping");
    }
    state_ = State::Starting;
  }

  try {
    sdk::Status status = read_driver_config(node_, config_);
    if (!status) {
      return fail_start(std::move(status));
    }
    status = select_and_connect();
    if (!status) {
      return fail_start(std::move(status));
    }
    status = establish_identity();
    if (!status) {
      return fail_start(std::move(status));
    }
    status = register_streams();
    if (!status) {
      return fail_start(std::move(status));
    }
    status = registry_->start_required();
    if (!status) {
      return fail_start(std::move(status));
    }
    status = start_control_plane();
    if (!status) {
      return fail_start(std::move(status));
    }
    status = registry_->start_optional();
    if (!status) {
      return fail_start(std::move(status));
    }
    start_fault_bridge();
    // SDK calls are not cancellable, so a Ctrl+C that arrived during one of them is seen
    // here and must not lead to enabling the motors.
    if (hooks_.cancelled()) {
      return fail_start(sdk::Status::failure("shutdown requested before enable"));
    }
    if (config_.auto_enable) {
      status = enable_with_retry();
      if (!status) {
        return fail_start(std::move(status));
      }
    }
    if (fingertip_registry_) {
      // Fingertip metadata reads are device round-trips the SDK does not always bound, so
      // they run on an executor thread once spinning starts instead of blocking startup.
      fingertip_timer_ =
          node_.create_wall_timer(0ms, [this]() { start_fingertip_streams(); }, tactile_group_);
    }
  } catch (...) {
    return fail_start(sdk::Status::failure(exception_message("Hand2Node start")));
  }

  {
    const std::lock_guard<std::mutex> lock(state_mutex_);
    state_ = State::Running;
  }
  RCLCPP_INFO(node_.get_logger(), "Hand2Node started");
  return sdk::Status::success();
}

void Hand2Node::Impl::stop() noexcept {
  {
    const std::lock_guard<std::mutex> lock(state_mutex_);
    if (state_ == State::Stopping || state_ == State::Stopped) {
      return;
    }
    state_ = State::Stopping;
  }

  try {
    if (control_plane_) {
      control_plane_->stop();
    } else {
      operation_gate_.stop();
    }
    stopping_.store(true, std::memory_order_release);
    fault_timer_.reset();
    fingertip_timer_.reset();
    if (fingertip_registry_) {
      fingertip_registry_->stop_all();
    }
    if (registry_) {
      registry_->stop_all();
    }
    session_.close_command_publisher();
    if (session_.device() != nullptr) {
      const auto disabled = session_.disable_all();
      if (!disabled) {
        RCLCPP_WARN(node_.get_logger(), "best-effort disable failed: %s", disabled.message.c_str());
      }
    }
    session_.disconnect();
    control_plane_.reset();
    registry_.reset();
    fingertip_registry_.reset();
    mapping_.reset();
  } catch (...) {
    RCLCPP_ERROR(node_.get_logger(), "Hand2Node stop contained an exception");
    try {
      session_.disconnect();
    } catch (...) {
    }
  }

  const std::lock_guard<std::mutex> lock(state_mutex_);
  state_ = State::Stopped;
}

bool Hand2Node::Impl::exit_requested() const noexcept {
  return exit_state_.requested();
}

int Hand2Node::Impl::exit_code() const noexcept {
  return exit_state_.code();
}

std::string Hand2Node::Impl::exit_reason() const {
  return exit_state_.reason();
}

sdk::Status Hand2Node::Impl::fail_start(sdk::Status status) {
  // Once Ctrl+C or SIGTERM arrived, the failure is a consequence of the stop the user asked for.
  if (hooks_.cancelled()) {
    exit_state_.request(EXIT_SUCCESS, "startup cancelled by shutdown request: " + status.message);
    RCLCPP_INFO(node_.get_logger(), "Hand2Node startup cancelled by shutdown request: %s",
                status.message.c_str());
  } else {
    exit_state_.request(EXIT_FAILURE, status.message);
    RCLCPP_ERROR(node_.get_logger(), "Hand2Node startup failed: %s", status.message.c_str());
  }
  stop();
  return status;
}

// -----------------------------------------------------------------------------
// Discovery and Connection
// -----------------------------------------------------------------------------

sdk::Status Hand2Node::Impl::scan_once(std::vector<Discovery>& output) {
  ScanRuntimeGuard runtime(sdk_);
  const auto initialized = sdk_.init(nullptr);
  sdk::Status status = sdk::sdk_status(sdk_, "wuji_init", initialized);
  if (!status) {
    return status;
  }
  runtime.activate();

  WujiDiscovered* discovered = nullptr;
  std::size_t count = 0;
  const auto scanned = sdk_.scan(&discovered, &count);
  DiscoveredGuard discovered_guard(sdk_, discovered, count);
  status = sdk::sdk_status(sdk_, "wuji_scan", scanned);
  if (!status) {
    return status;
  }
  if (count != 0U && discovered == nullptr) {
    return sdk::Status::failure("wuji_scan returned a null array with nonzero size");
  }

  std::vector<Discovery> hand2;
  hand2.reserve(count);
  for (std::size_t index = 0; index < count; ++index) {
    if (fixed_string(discovered[index].model) == kScannedHand2Model) {
      hand2.push_back(Discovery{fixed_string(discovered[index].serial_number),
                                fixed_string(discovered[index].address),
                                discovered[index].device_id, discovered[index].transport});
    }
  }
  std::sort(hand2.begin(), hand2.end(), [](const Discovery& lhs, const Discovery& rhs) {
    const std::string lhs_identity = physical_identity(lhs);
    const std::string rhs_identity = physical_identity(rhs);
    if (lhs_identity != rhs_identity) {
      return lhs_identity < rhs_identity;
    }
    return lhs.address < rhs.address;
  });
  hand2.erase(std::unique(hand2.begin(), hand2.end(),
                          [](const Discovery& lhs, const Discovery& rhs) {
                            return physical_identity(lhs) == physical_identity(rhs);
                          }),
              hand2.end());
  output = std::move(hand2);
  return sdk::Status::success();
}

bool Hand2Node::Impl::retryable_connect_status(WujiStatus status) noexcept {
  return status == WUJI_STATUS_ERR_TIMEOUT || status == WUJI_STATUS_ERR_NOT_FOUND;
}

sdk::Status Hand2Node::Impl::next_attempt_timeout(Hand2NodeHooks::Clock::time_point started_at,
                                                  std::uint32_t& output) const {
  if (hooks_.cancelled()) {
    return sdk::Status::failure("device connection was cancelled");
  }
  if (config_.connect_timeout == 0.0) {
    output = 1000U;
    return sdk::Status::success();
  }

  const double elapsed = std::chrono::duration<double>(hooks_.now() - started_at).count();
  const double remaining = config_.connect_timeout - elapsed;
  if (remaining <= 0.0) {
    return sdk::Status::failure("device selection exceeded connect_timeout");
  }
  const double remaining_ms = std::ceil(remaining * 1000.0);
  output = static_cast<std::uint32_t>(std::clamp(
      remaining_ms, 1.0, static_cast<double>(std::numeric_limits<std::uint16_t>::max())));
  output = std::min(output, 1000U);
  return sdk::Status::success();
}

sdk::Status Hand2Node::Impl::wait_before_retry(Hand2NodeHooks::Clock::time_point started_at) {
  std::uint32_t remaining_attempt_ms = 0;
  sdk::Status status = next_attempt_timeout(started_at, remaining_attempt_ms);
  static_cast<void>(remaining_attempt_ms);
  if (!status) {
    return status;
  }
  auto sleep_duration =
      std::chrono::duration_cast<Hand2NodeHooks::Clock::duration>(kDiscoveryPollInterval);
  if (config_.connect_timeout > 0.0) {
    const double elapsed = std::chrono::duration<double>(hooks_.now() - started_at).count();
    const double remaining = config_.connect_timeout - elapsed;
    sleep_duration =
        std::min(sleep_duration, std::chrono::duration_cast<Hand2NodeHooks::Clock::duration>(
                                     std::chrono::duration<double>{remaining}));
  }
  hooks_.sleep_for(sleep_duration);
  return sdk::Status::success();
}

sdk::Hand2ConnectConfig Hand2Node::Impl::candidate_config(const Discovery& candidate,
                                                          bool validate_side) const {
  sdk::Hand2ConnectConfig connection;
  if (validate_side) {
    connection.hand_side = config_.hand_side;
  }
  if (!candidate.serial_number.empty()) {
    connection.serial_number = candidate.serial_number;
  } else {
    connection.endpoint = candidate.address;
  }
  return connection;
}

Hand2Node::Impl::ConnectAttempt Hand2Node::Impl::connect_attempt(
    sdk::Hand2ConnectConfig connection, bool open_publisher,
    Hand2NodeHooks::Clock::time_point started_at) {
  std::uint32_t timeout_ms = 0;
  sdk::Status budget = next_attempt_timeout(started_at, timeout_ms);
  if (!budget) {
    return ConnectAttempt{std::move(budget), WUJI_STATUS_OK, true};
  }
  connection.connect_timeout = static_cast<double>(timeout_ms) / 1000.0;
  connection.retry_count = 0U;

  WujiStatus failure_status = WUJI_STATUS_OK;
  sdk::Status status = session_.connect_device(connection, &failure_status);
  if (!status || !open_publisher) {
    return ConnectAttempt{std::move(status), failure_status, false};
  }
  std::uint32_t publisher_budget_ms = 0;
  budget = next_attempt_timeout(started_at, publisher_budget_ms);
  static_cast<void>(publisher_budget_ms);
  if (!budget) {
    session_.disconnect();
    return ConnectAttempt{std::move(budget), failure_status, true};
  }
  status = session_.open_command_publisher();
  if (!status) {
    session_.disconnect();
    return ConnectAttempt{std::move(status), failure_status, false};
  }

  std::uint32_t post_publisher_budget_ms = 0;
  budget = next_attempt_timeout(started_at, post_publisher_budget_ms);
  static_cast<void>(post_publisher_budget_ms);
  if (!budget) {
    session_.close_command_publisher();
    session_.disconnect();
    return ConnectAttempt{std::move(budget), failure_status, true};
  }
  return ConnectAttempt{std::move(status), failure_status, false};
}

sdk::Status Hand2Node::Impl::connect_full_with_retry(const sdk::Hand2ConnectConfig& connection,
                                                     Hand2NodeHooks::Clock::time_point started_at) {
  for (;;) {
    ConnectAttempt attempt = connect_attempt(connection, true, started_at);
    if (attempt.status || attempt.budget_exhausted) {
      return attempt.status;
    }
    if (!retryable_connect_status(attempt.failure_status)) {
      return attempt.status;
    }
    sdk::Status wait_status = wait_before_retry(started_at);
    if (!wait_status) {
      return wait_status;
    }
  }
}

sdk::Status Hand2Node::Impl::select_and_connect() {
  const auto started_at = hooks_.now();
  if (!config_.endpoint.empty() || !config_.serial_number.empty()) {
    sdk::Hand2ConnectConfig connection;
    connection.endpoint = config_.endpoint;
    connection.serial_number = config_.serial_number;
    connection.hand_side = config_.hand_side;
    return connect_full_with_retry(connection, started_at);
  }

  for (;;) {
    std::uint32_t unused_timeout_ms = 0;
    sdk::Status budget_status = next_attempt_timeout(started_at, unused_timeout_ms);
    if (!budget_status) {
      return budget_status;
    }

    std::vector<Discovery> candidates;
    sdk::Status status = scan_once(candidates);
    if (!status) {
      return status;
    }
    if (candidates.empty()) {
      status = wait_before_retry(started_at);
      if (!status) {
        return status;
      }
      continue;
    }

    if (config_.hand_side.empty()) {
      if (candidates.size() != 1U) {
        return sdk::Status::failure(
            "multiple Wuji Hand 2 devices found; set endpoint, serial_number, or hand_side");
      }
      return connect_full_with_retry(candidate_config(candidates.front(), false), started_at);
    }

    std::vector<Discovery> matches;
    bool unresolved_candidate = false;
    bool has_operational_failure = false;
    bool has_unusable_candidate = false;
    sdk::Status operational_failure = sdk::Status::success();
    sdk::Status unusable_candidate = sdk::Status::success();
    for (const Discovery& candidate : candidates) {
      sdk::Hand2ConnectConfig probe = candidate_config(candidate, false);
      if (probe.serial_number.empty() && probe.endpoint.empty()) {
        if (!has_unusable_candidate) {
          unusable_candidate =
              sdk::Status::failure("discovered Hand 2 has no serial number or endpoint");
          has_unusable_candidate = true;
        }
        continue;
      }
      ConnectAttempt attempt = connect_attempt(std::move(probe), false, started_at);
      if (attempt.budget_exhausted) {
        return attempt.status;
      }
      if (!attempt.status) {
        if (retryable_connect_status(attempt.failure_status)) {
          unresolved_candidate = true;
          continue;
        }
        if (!has_operational_failure) {
          operational_failure = std::move(attempt.status);
          has_operational_failure = true;
        }
        continue;
      }

      const bool side_matches = session_.hand_side() == config_.hand_side;
      session_.disconnect();
      if (side_matches) {
        matches.push_back(candidate);
      }
    }

    if (matches.size() > 1U) {
      return sdk::Status::failure("multiple Wuji Hand 2 devices match hand_side " +
                                  config_.hand_side);
    }
    if (matches.size() == 1U && !unresolved_candidate && !has_operational_failure) {
      return connect_full_with_retry(candidate_config(matches.front(), true), started_at);
    }
    if (has_operational_failure) {
      return operational_failure;
    }
    if (has_unusable_candidate) {
      return unusable_candidate;
    }

    status = wait_before_retry(started_at);
    if (!status) {
      return status;
    }
  }
}

// -----------------------------------------------------------------------------
// Enable
// -----------------------------------------------------------------------------

bool Hand2Node::Impl::retryable_enable_status(WujiStatus status) noexcept {
  return status == WUJI_STATUS_ERR_TIMEOUT || status == WUJI_STATUS_ERR_PROTOCOL ||
         status == WUJI_STATUS_ERR_INTERNAL;
}

sdk::Status Hand2Node::Impl::enable_with_retry() {
  for (std::size_t attempt = 1;; ++attempt) {
    WujiStatus failure_status = WUJI_STATUS_OK;
    sdk::Status status = session_.enable_all(&failure_status);
    if (status) {
      return status;
    }
    if (attempt >= kEnableAttempts || !retryable_enable_status(failure_status) ||
        !session_.is_connected()) {
      return status;
    }
    RCLCPP_WARN(node_.get_logger(), "%s; retrying enable (%zu/%zu)", status.message.c_str(),
                attempt + 1U, kEnableAttempts);
    hooks_.sleep_for(kEnableRetryDelay);
    if (hooks_.cancelled()) {
      return sdk::Status::failure("wuji_hand_2_enable retry abandoned: shutdown requested");
    }
  }
}

// -----------------------------------------------------------------------------
// Identity, Streams, and Control
// -----------------------------------------------------------------------------

sdk::Status Hand2Node::Impl::establish_identity() {
  const std::string side = session_.hand_side();
  if (side == "left") {
    mapping_ = std::make_unique<conversion::JointMapping>(conversion::HandSide::kLeft);
  } else if (side == "right") {
    mapping_ = std::make_unique<conversion::JointMapping>(conversion::HandSide::kRight);
  } else {
    return sdk::Status::failure("connected Hand 2 has no valid hand side");
  }

  return sdk::Status::success();
}

sdk::Status Hand2Node::Impl::register_streams() {
  control::OperationContext context{sdk_, session_, *mapping_, operation_gate_,
                                    node_, node_.get_logger()};
  control_plane_ = std::make_unique<control::ControlPlane>(
      context, control::TactileCalibrationTiming::production(), config_);
  registry_ = std::make_unique<wuji_driver_common::stream::StreamRegistry>();
  sdk::Status status = registry_->add(std::make_unique<stream::JointStateStream>(
      kJointStateKey, session_.device(), sdk_, node_, *mapping_, control_plane_->feedback_observer()));
  if (!status) {
    return status;
  }
  if (config_.publish_joint_diagnostics) {
    auto diagnostics = std::make_unique<stream::JointDiagnosticsStream>(
        kJointDiagnosticsKey, session_.device(), sdk_, node_, *mapping_);
    status = registry_->add(std::move(diagnostics));
    if (!status) {
      return status;
    }
  }
  if (config_.publish_imu) {
    status = registry_->add(
        std::make_unique<stream::ImuStream>(kImuKey, session_.device(), sdk_, node_));
    if (!status) {
      return status;
    }
  }
  if (config_.publish_fingertip_tactile) {
    fingertip_registry_ = std::make_unique<wuji_driver_common::stream::StreamRegistry>();
  }
  return sdk::Status::success();
}

void Hand2Node::Impl::start_fingertip_streams() noexcept {
  try {
    // Held for the whole callback: stop() takes this lock first, so it waits for the
    // fingertip startup to finish before it releases the registry and the device.
    const std::lock_guard<std::mutex> lock(state_mutex_);
    if (state_ != State::Running) {
      return;
    }
    fingertip_timer_->cancel();
    bool online[WUJI_HAND_2_FINGER_COUNT] = {};
    sdk::Status status = sdk::sdk_status(sdk_, "wuji_hand_2_fingertip_online",
                                         sdk_.fingertip_online(session_.device(), online));
    for (std::size_t index = 0; status && index < kFingertipKeys.size(); ++index) {
      if (!online[index]) {
        RCLCPP_INFO(node_.get_logger(), "%s skipped: tactile module offline",
                    stream_label(kFingertipKeys[index]));
        continue;
      }
      status = fingertip_registry_->add(std::make_unique<stream::FingertipStream>(
          kFingertipKeys[index], session_.device(), sdk_, node_,
          static_cast<stream::Fingertip>(index), sdk_.subscribe_fingertip[index]));
    }
    // The registry has no required streams, so this only moves it to the optional phase.
    if (status) {
      status = fingertip_registry_->start_required();
    }
    if (status) {
      status = fingertip_registry_->start_optional(hooks_.cancelled);
    }
    if (!status) {
      RCLCPP_WARN(node_.get_logger(), "fingertip tactile streams skipped: %s",
                  status.message.c_str());
    }
  } catch (...) {
    auto message = exception_message("fingertip stream startup");
    RCLCPP_WARN(node_.get_logger(), "%s", message.c_str());
  }
}

sdk::Status Hand2Node::Impl::start_control_plane() {
  return control_plane_->start(command_group_, control_group_, emergency_group_, tactile_group_);
}

// -----------------------------------------------------------------------------
// Stream Fault Handling
// -----------------------------------------------------------------------------

void Hand2Node::Impl::start_fault_bridge() {
  fault_timer_ =
      node_.create_wall_timer(kFaultBridgePeriod, [this]() { on_fault_timer(); }, fault_group_);
}

void Hand2Node::Impl::request_runtime_failure(std::string reason) noexcept {
  try {
    if (!exit_state_.request(EXIT_FAILURE, std::move(reason))) {
      return;
    }
    operation_gate_.stop();
    try {
      hooks_.request_shutdown();
    } catch (...) {
      RCLCPP_ERROR(node_.get_logger(), "shutdown request failed");
    }
  } catch (...) {
    RCLCPP_ERROR(node_.get_logger(), "runtime failure handling contained an exception");
  }
}

void Hand2Node::Impl::on_fault_timer() noexcept {
  try {
    if (stopping_.load(std::memory_order_acquire) || exit_state_.requested()) {
      return;
    }
    for (auto* const registry : {registry_.get(), fingertip_registry_.get()}) {
      if (registry == nullptr) {
        continue;
      }
      for (const auto& snapshot : registry->snapshots()) {
        if (snapshot.fault.kind == TerminalKind::None) {
          continue;
        }
        if (snapshot.policy == StreamPolicy::Required) {
          auto message = stream_fault_message(snapshot);
          RCLCPP_ERROR(node_.get_logger(), "required stream fault: %s", message.c_str());
          request_runtime_failure(std::move(message));
          return;
        }
        if (snapshot.state == StreamState::Stopping || snapshot.state == StreamState::Stopped) {
          continue;
        }
        const auto message = stream_fault_message(snapshot);
        RCLCPP_WARN(node_.get_logger(), "optional stream fault: %s", message.c_str());
        registry->stop(snapshot.key);
      }
    }
  } catch (...) {
    auto message = exception_message("stream fault bridge");
    RCLCPP_ERROR(node_.get_logger(), "%s", message.c_str());
    request_runtime_failure(std::move(message));
  }
}

// =============================================================================
// Hand2Node
// =============================================================================

Hand2Node::Hand2Node(const rclcpp::NodeOptions& options)
    : Hand2Node(sdk::Hand2SdkApi::production(), Hand2NodeHooks{}, options) {}

Hand2Node::Hand2Node(sdk::Hand2SdkApi sdk_api, Hand2NodeHooks hooks,
                     const rclcpp::NodeOptions& options)
    : rclcpp::Node("wuji_hand2_driver", options) {
  declare_driver_parameters(*this);
  impl_ = std::make_unique<Impl>(*this, std::move(sdk_api), std::move(hooks));
}

Hand2Node::~Hand2Node() noexcept {
  stop();
}

sdk::Status Hand2Node::start() {
  return impl_->start();
}

void Hand2Node::stop() noexcept {
  if (impl_) {
    impl_->stop();
  }
}

bool Hand2Node::exit_requested() const noexcept {
  return impl_->exit_requested();
}

int Hand2Node::exit_code() const noexcept {
  return impl_->exit_code();
}

std::string Hand2Node::exit_reason() const {
  return impl_->exit_reason();
}

}  // namespace wuji_hand2_driver
