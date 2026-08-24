// Copyright 2025 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#pragma once

// Timer handle and the free set_period().

#include <rclcpp/exceptions/exceptions.hpp>
#include <rclcpp/rclcpp.hpp>

#include <rcl/timer.h>

#include <chrono>
#include <cstdint>
#include <memory>
#include <stdexcept>
#include <utility>

#ifdef USE_AGNOCAST_ENABLED

#include <agnocast/agnocast.hpp>

namespace autoware::agnocast_wrapper
{

/// @brief Type-erased timer handle for the Agnocast build.
///
/// Backed by AgnocastTimer or ROS2Timer depending on whether Agnocast is
/// active at runtime. Must be obtained via Node::create_wall_timer() or the
/// free create_timer() — do not construct directly. set_period() is
/// intentionally private; use the free set_period(Timer::SharedPtr, ...)
/// instead, which is also available in non-Agnocast builds.
///
/// Cross-build portability: in non-Agnocast builds AUTOWARE_TIMER_PTR
/// resolves to rclcpp::TimerBase::SharedPtr and exposes the full rclcpp API;
/// in Agnocast builds it resolves to this wrapper, which only exposes
/// cancel/reset/is_canceled/time_until_trigger and the free set_period().
/// Calling any other rclcpp::TimerBase method compiles in non-Agnocast builds
/// but breaks once Agnocast is enabled. Stay within the wrapper's surface to
/// remain portable.
///
/// Exception contract: the exception type and "throw vs no-op" behavior of
/// these methods is not normalized across backends — the rclcpp backend tends
/// to throw rclcpp::exceptions::RCLError on rcl failure, while the Agnocast
/// backend may throw std::runtime_error or silently return a sentinel. This
/// asymmetry exists across the wrapper as a whole (not Timer-specific) and is
/// expected to be normalized in a follow-up.
class Timer
{
public:
  using SharedPtr = std::shared_ptr<Timer>;
  virtual ~Timer() = default;

  virtual void cancel() = 0;
  virtual void reset() = 0;
  virtual bool is_canceled() = 0;
  virtual std::chrono::nanoseconds time_until_trigger() = 0;

private:
  // Private so callers must use the free set_period() function, which also works in the
  // non-agnocast build (where AUTOWARE_TIMER_PTR is a plain rclcpp::TimerBase, no set_period).
  virtual void set_period(std::chrono::nanoseconds period) = 0;
  friend void set_period(const SharedPtr & timer, std::chrono::nanoseconds period);
};

class AgnocastTimer : public Timer
{
  std::shared_ptr<agnocast::TimerBase> timer_;

public:
  explicit AgnocastTimer(std::shared_ptr<agnocast::TimerBase> timer) : timer_(std::move(timer)) {}

  void cancel() override { timer_->cancel(); }
  void reset() override { timer_->reset(); }
  bool is_canceled() override { return timer_->is_canceled(); }
  std::chrono::nanoseconds time_until_trigger() override { return timer_->time_until_trigger(); }

private:
  void set_period(std::chrono::nanoseconds period) override { timer_->set_period(period); }
};

class ROS2Timer : public Timer
{
  rclcpp::TimerBase::SharedPtr timer_;

public:
  explicit ROS2Timer(rclcpp::TimerBase::SharedPtr timer) : timer_(std::move(timer)) {}

  void cancel() override { timer_->cancel(); }
  void reset() override { timer_->reset(); }
  bool is_canceled() override { return timer_->is_canceled(); }
  std::chrono::nanoseconds time_until_trigger() override { return timer_->time_until_trigger(); }

private:
  // rclcpp::TimerBase does not expose a set_period API; fall back to the rcl C API and
  // convert the rcl_ret_t to an rclcpp::exceptions::RCLError (matching the throw style used
  // by the other rclcpp timer methods such as cancel/reset/time_until_trigger).
  void set_period(std::chrono::nanoseconds period) override
  {
    int64_t old_period = 0;
    const rcl_ret_t ret =
      rcl_timer_exchange_period(timer_->get_timer_handle().get(), period.count(), &old_period);
    if (ret != RCL_RET_OK) {
      rclcpp::exceptions::throw_from_rcl_error(ret, "Failed to set timer period");
    }
  }
};

/// @brief Set the timer period.
///
/// Provided as a free function so the same call site compiles in both builds:
/// in non-Agnocast builds rclcpp::TimerBase has no set_period member, so a
/// free overload is the only portable form. Timer::set_period is private to
/// prevent member-style calls that would not survive the non-Agnocast build.
///
/// @throws std::invalid_argument if period is negative or equal to
///   std::chrono::nanoseconds::max() (mirrors rclcpp::create_wall_timer's
///   precondition 0 <= period < nanoseconds::max()).
/// @throws rclcpp::exceptions::RCLError on rcl-level failure when the ROS 2
///   backend is active (see Timer's class-level note on exception asymmetry
///   between backends).
inline void set_period(const Timer::SharedPtr & timer, std::chrono::nanoseconds period)
{
  if (period < std::chrono::nanoseconds::zero()) {
    throw std::invalid_argument{"timer period cannot be negative"};
  }
  if (period == std::chrono::nanoseconds::max()) {
    throw std::invalid_argument{"timer period must be less than std::chrono::nanoseconds::max()"};
  }
  timer->set_period(period);
}

}  // namespace autoware::agnocast_wrapper

#else

namespace autoware::agnocast_wrapper
{

/// @brief Set the timer period (non-Agnocast build).
///
/// rclcpp::TimerBase has no set_period member, so we provide a free overload
/// that falls back to the rcl C API. Mirrors the Agnocast-build overload on
/// Timer::SharedPtr so the same call site works in both builds.
///
/// @throws std::invalid_argument if period is negative or equal to
///   std::chrono::nanoseconds::max() (mirrors rclcpp::create_wall_timer's
///   precondition 0 <= period < nanoseconds::max()).
/// @throws rclcpp::exceptions::RCLError on rcl-level failure.
inline void set_period(const rclcpp::TimerBase::SharedPtr & timer, std::chrono::nanoseconds period)
{
  if (period < std::chrono::nanoseconds::zero()) {
    throw std::invalid_argument{"timer period cannot be negative"};
  }
  if (period == std::chrono::nanoseconds::max()) {
    throw std::invalid_argument{"timer period must be less than std::chrono::nanoseconds::max()"};
  }
  int64_t old_period = 0;
  const rcl_ret_t ret =
    rcl_timer_exchange_period(timer->get_timer_handle().get(), period.count(), &old_period);
  if (ret != RCL_RET_OK) {
    rclcpp::exceptions::throw_from_rcl_error(ret, "Failed to set timer period");
  }
}

}  // namespace autoware::agnocast_wrapper

#endif
