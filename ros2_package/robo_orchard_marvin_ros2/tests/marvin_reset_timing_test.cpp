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

#include "marvin_reset_timing.hpp"

#include <gtest/gtest.h>

#include <chrono>

namespace
{

using robo_orchard_marvin_ros2::ResetTrajectoryTiming;

constexpr double kTolerance = 1e-12;

}  // namespace

TEST(MarvinResetTimingTest, AdvancesAfterSuccessfulSends)
{
  const ResetTrajectoryTiming::TimePoint started_at{};
  ResetTrajectoryTiming timing(started_at);

  const auto first_send = started_at + std::chrono::milliseconds(10);
  EXPECT_NEAR(timing.candidate_elapsed_s(first_send), 0.010, kTolerance);
  timing.commit_send(first_send);
  EXPECT_NEAR(timing.trajectory_elapsed_s(), 0.010, kTolerance);

  EXPECT_NEAR(
    timing.candidate_elapsed_s(started_at + std::chrono::milliseconds(25)),
    0.025, kTolerance);
}

TEST(MarvinResetTimingTest, PausesAcrossTransientBusyState)
{
  const ResetTrajectoryTiming::TimePoint started_at{};
  ResetTrajectoryTiming timing(started_at);
  timing.commit_send(started_at + std::chrono::milliseconds(10));

  timing.defer_send(started_at + std::chrono::milliseconds(15));
  EXPECT_TRUE(timing.send_buffer_busy());
  EXPECT_NEAR(
    timing.candidate_elapsed_s(started_at + std::chrono::milliseconds(20)),
    0.010, kTolerance);

  timing.commit_send(started_at + std::chrono::milliseconds(20));
  EXPECT_FALSE(timing.send_buffer_busy());
  EXPECT_NEAR(timing.trajectory_elapsed_s(), 0.010, kTolerance);
  EXPECT_NEAR(
    timing.candidate_elapsed_s(started_at + std::chrono::milliseconds(25)),
    0.015, kTolerance);
}

TEST(MarvinResetTimingTest, TimesOutOnlyAfterContinuousBusyState)
{
  const ResetTrajectoryTiming::TimePoint started_at{};
  ResetTrajectoryTiming timing(started_at);
  constexpr double kTimeoutS = 0.1;

  timing.defer_send(started_at + std::chrono::milliseconds(5));
  EXPECT_FALSE(
    timing.send_buffer_busy_timed_out(
      started_at + std::chrono::milliseconds(104), kTimeoutS));
  EXPECT_TRUE(
    timing.send_buffer_busy_timed_out(
      started_at + std::chrono::milliseconds(105), kTimeoutS));

  timing.commit_send(started_at + std::chrono::milliseconds(110));
  EXPECT_FALSE(timing.send_buffer_busy());
  EXPECT_FALSE(
    timing.send_buffer_busy_timed_out(
      started_at + std::chrono::seconds(1), kTimeoutS));
}

TEST(MarvinResetTimingTest, KeepsArmTimingIndependent)
{
  const ResetTrajectoryTiming::TimePoint started_at{};
  ResetTrajectoryTiming left(started_at);
  ResetTrajectoryTiming right(started_at);

  left.defer_send(started_at + std::chrono::milliseconds(5));
  right.commit_send(started_at + std::chrono::milliseconds(10));

  EXPECT_TRUE(left.send_buffer_busy());
  EXPECT_NEAR(left.trajectory_elapsed_s(), 0.0, kTolerance);
  EXPECT_FALSE(right.send_buffer_busy());
  EXPECT_NEAR(right.trajectory_elapsed_s(), 0.010, kTolerance);
}
