// Project RoboOrchard
//
// Copyright (c) 2024-2026 Horizon Robotics. All Rights Reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//       http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or
// implied. See the License for the specific language governing
// permissions and limitations under the License.

#ifndef MARVIN_RESET_TIMING_HPP_
#define MARVIN_RESET_TIMING_HPP_

#include <chrono>
#include <optional>

namespace robo_orchard_marvin_ros2
{

class ResetTrajectoryTiming
{
public:
  using Clock = std::chrono::steady_clock;
  using TimePoint = Clock::time_point;

  explicit ResetTrajectoryTiming(TimePoint started_at)
  : last_update_at_(started_at)
  {
  }

  double candidate_elapsed_s(TimePoint now) const
  {
    if (send_buffer_busy_since_.has_value()) {
      return trajectory_elapsed_s_;
    }
    return trajectory_elapsed_s_ +
           std::chrono::duration<double>(now - last_update_at_).count();
  }

  void defer_send(TimePoint now)
  {
    last_update_at_ = now;
    if (!send_buffer_busy_since_.has_value()) {
      send_buffer_busy_since_ = now;
    }
  }

  void commit_send(TimePoint now)
  {
    trajectory_elapsed_s_ = candidate_elapsed_s(now);
    last_update_at_ = now;
    send_buffer_busy_since_.reset();
  }

  bool send_buffer_busy() const
  {
    return send_buffer_busy_since_.has_value();
  }

  double send_buffer_busy_duration_s(TimePoint now) const
  {
    if (!send_buffer_busy_since_.has_value()) {
      return 0.0;
    }
    return std::chrono::duration<double>(now - *send_buffer_busy_since_).count();
  }

  bool send_buffer_busy_timed_out(TimePoint now, double timeout_s) const
  {
    return send_buffer_busy_duration_s(now) >= timeout_s;
  }

  double trajectory_elapsed_s() const
  {
    return trajectory_elapsed_s_;
  }

private:
  TimePoint last_update_at_{};
  std::optional<TimePoint> send_buffer_busy_since_;
  double trajectory_elapsed_s_{0.0};
};

}  // namespace robo_orchard_marvin_ros2

#endif  // MARVIN_RESET_TIMING_HPP_
