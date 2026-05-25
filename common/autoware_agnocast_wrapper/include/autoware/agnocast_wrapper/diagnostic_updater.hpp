// Copyright 2026 TIER IV, Inc.
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

// cspell:ignore hwid

#pragma once

#include "autoware/agnocast_wrapper/node.hpp"

#include <diagnostic_updater/diagnostic_updater.hpp>

#include <cstdarg>
#include <cstdio>
#include <memory>
#include <string>
#include <variant>

#ifdef USE_AGNOCAST_ENABLED

#include <agnocast/node/diagnostic_updater/diagnostic_updater.hpp>

namespace autoware::agnocast_wrapper
{

/// @brief Wrapper Updater that dispatches between diagnostic_updater::Updater (rclcpp mode)
///        and agnocast::Updater (agnocast mode) at runtime, depending on whether the given
///        autoware::agnocast_wrapper::Node is running in agnocast mode.
///
/// Constructor signature mirrors `diagnostic_updater::Updater updater_{this};` so nodes
/// inheriting from autoware::agnocast_wrapper::Node can use the same idiom in both modes.
class Updater
{
public:
  using RclcppImpl = std::unique_ptr<diagnostic_updater::Updater>;
  using AgnocastImpl = std::unique_ptr<agnocast::Updater>;

  explicit Updater(autoware::agnocast_wrapper::Node * node, double period = 1.0)
  : logger_(node->get_logger()),
    impl_(
      use_agnocast()
        ? decltype(impl_)(
            std::in_place_type<AgnocastImpl>,
            std::make_unique<agnocast::Updater>(*node->get_agnocast_node(), period))
        : decltype(impl_)(
            std::in_place_type<RclcppImpl>,
            std::make_unique<diagnostic_updater::Updater>(node->get_rclcpp_node(), period))),
    verbose_(std::visit([](auto & impl) -> bool & { return impl->verbose_; }, impl_))
  {
  }

  void add(const std::string & name, diagnostic_updater::TaskFunction f)
  {
    std::visit([&](auto & impl) { impl->add(name, f); }, impl_);
  }

  void add(diagnostic_updater::DiagnosticTask & task)
  {
    std::visit([&](auto & impl) { impl->add(task); }, impl_);
  }

  template <class T>
  void add(
    const std::string name, T * c, void (T::*f)(diagnostic_updater::DiagnosticStatusWrapper &))
  {
    std::visit([&](auto & impl) { impl->add(name, c, f); }, impl_);
  }

  bool removeByName(const std::string name)
  {
    return std::visit([&](auto & impl) { return impl->removeByName(name); }, impl_);
  }

  auto getPeriod() const
  {
    return std::visit([](const auto & impl) { return impl->getPeriod(); }, impl_);
  }

  void setPeriod(rclcpp::Duration period)
  {
    std::visit([&](auto & impl) { impl->setPeriod(period); }, impl_);
  }

  void setPeriod(double period)
  {
    std::visit([&](auto & impl) { impl->setPeriod(period); }, impl_);
  }

  void force_update()
  {
    std::visit([&](auto & impl) { impl->force_update(); }, impl_);
  }

  void broadcast(unsigned char lvl, const std::string msg)
  {
    std::visit([&](auto & impl) { impl->broadcast(lvl, msg); }, impl_);
  }

  void setHardwareID(const std::string & hwid)
  {
    std::visit([&](auto & impl) { impl->setHardwareID(hwid); }, impl_);
  }

  void setHardwareIDf(const char * format, ...)
  {
    va_list va;
    constexpr int kBufferSize = 1000;
    char buff[kBufferSize];
    va_start(va, format);
    if (vsnprintf(buff, kBufferSize, format, va) >= kBufferSize) {
      RCLCPP_DEBUG(logger_, "Really long string in diagnostic_updater::setHardwareIDf.");
    }
    va_end(va);
    setHardwareID(std::string(buff));
  }

  Updater(const Updater &) = delete;
  Updater & operator=(const Updater &) = delete;
  Updater(Updater &&) = delete;
  Updater & operator=(Updater &&) = delete;

private:
  rclcpp::Logger logger_;
  std::variant<RclcppImpl, AgnocastImpl> impl_;

public:
  // Mirrors `diagnostic_updater::Updater::verbose_` / `agnocast::Updater::verbose_`.
  // Bound by reference so `updater.verbose_ = true;` writes through to the underlying impl.
  bool & verbose_;
};

}  // namespace autoware::agnocast_wrapper

#else

namespace autoware::agnocast_wrapper
{
using Updater = diagnostic_updater::Updater;
}  // namespace autoware::agnocast_wrapper

#endif
