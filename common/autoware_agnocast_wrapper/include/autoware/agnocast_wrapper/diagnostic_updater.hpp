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

#ifdef USE_AGNOCAST_ENABLED

#include <agnocast/node/diagnostic_updater/diagnostic_updater.hpp>

#include <cstdarg>
#include <cstdio>
#include <memory>
#include <string>
#include <variant>

namespace autoware::agnocast_wrapper
{
namespace diagnostic_updater
{

/// @brief Wrapper Updater that dispatches between ::diagnostic_updater::Updater
///        (rclcpp mode) and ::agnocast::Updater (agnocast mode) at runtime,
///        depending on whether the given autoware::agnocast_wrapper::Node is
///        running in agnocast mode.
///
/// Constructor signature mirrors `::diagnostic_updater::Updater updater_{this};`
/// so nodes inheriting from autoware::agnocast_wrapper::Node can use the same
/// idiom in both modes. The `diagnostic_updater.period` and
/// `diagnostic_updater.use_fqn` ros2 parameters are declared by the underlying
/// impl identically in both backends, so observed behavior is consistent.
///
/// @invariant The backend variant is selected from use_agnocast() at construction
///            and never changes. impl_ holds the active backend through unique_ptr,
///            so the underlying impl object's address stays stable for the
///            wrapper's lifetime — `verbose_` is bound by reference to that
///            stable address.
///
/// @code
/// #include <autoware/agnocast_wrapper/diagnostic_updater.hpp>
///
/// class MyNode : public autoware::agnocast_wrapper::Node
/// {
/// public:
///   explicit MyNode(const rclcpp::NodeOptions & options)
///   : Node("my_node", options), updater_(this)
///   {
///     updater_.setHardwareID("my_hardware");
///     updater_.add("status", this, &MyNode::diagnose);
///   }
///
/// private:
///   void diagnose(diagnostic_updater::DiagnosticStatusWrapper & stat) {
///     stat.summary(diagnostic_msgs::msg::DiagnosticStatus::OK, "running");
///   }
///
///   autoware::agnocast_wrapper::diagnostic_updater::Updater updater_;
/// };
/// @endcode
class Updater
{
public:
  /// @brief Heap-allocated rclcpp-backed implementation held inside impl_.
  using RclcppImpl = std::unique_ptr<::diagnostic_updater::Updater>;

  /// @brief Heap-allocated agnocast-backed implementation held inside impl_.
  using AgnocastImpl = std::unique_ptr<::agnocast::Updater>;

  /// @brief Construct an Updater bound to a wrapper Node.
  ///
  /// Selects the backend from use_agnocast() at construction; the choice is
  /// fixed for the wrapper's lifetime.
  ///
  /// @pre The given Node must outlive this Updater. ::agnocast::Updater stores
  ///      `agnocast::Node &` by reference (initialized from
  ///      `*node->get_agnocast_node()`), and ::diagnostic_updater::Updater holds
  ///      interface pointers extracted from the rclcpp::Node. Outliving the
  ///      wrapper Node will dangle in both modes.
  ///
  /// @param node   Wrapper node providing access to either an agnocast::Node or
  ///               an rclcpp::Node.
  /// @param period Default update period in seconds; overridden by the
  ///               `diagnostic_updater.period` ros2 parameter if set.
  explicit Updater(autoware::agnocast_wrapper::Node * node, double period = 1.0)
  : logger_(node->get_logger()),
    impl_(
      // unique_ptr indirection is required because both ::diagnostic_updater::Updater
      // and ::agnocast::Updater are non-movable (their constructors register internal
      // callbacks that capture `this`). std::variant alternatives must be at least
      // move-constructible, so we hold each impl through unique_ptr to satisfy that
      // constraint while keeping the underlying impl's address stable.
      use_agnocast()
        ? decltype(impl_)(
            std::in_place_type<AgnocastImpl>,
            std::make_unique<::agnocast::Updater>(*node->get_agnocast_node(), period))
        : decltype(impl_)(
            std::in_place_type<RclcppImpl>,
            std::make_unique<::diagnostic_updater::Updater>(node->get_rclcpp_node(), period))),
    verbose_(std::visit([](auto & impl) -> bool & { return impl->verbose_; }, impl_))
  {
  }

  /// @brief Register a diagnostic task by name and callable.
  ///
  /// Dispatches to ::diagnostic_updater::Updater::add() or ::agnocast::Updater::add()
  /// depending on the active backend.
  ///
  /// @param name Diagnostic task name surfaced in the published DiagnosticStatus.
  /// @param f    Callable invoked each update cycle to fill in a DiagnosticStatusWrapper.
  void add(const std::string & name, ::diagnostic_updater::TaskFunction f)
  {
    std::visit([&](auto & impl) { impl->add(name, f); }, impl_);
  }

  /// @brief Register a diagnostic task object by reference.
  ///
  /// The task is stored by reference inside the underlying Updater; the caller
  /// must ensure the task outlives this Updater. Useful for adding pre-built
  /// task types such as FrequencyStatus, TimeStampStatus, or Heartbeat.
  ///
  /// @param task DiagnosticTask subclass instance.
  void add(::diagnostic_updater::DiagnosticTask & task)
  {
    std::visit([&](auto & impl) { impl->add(task); }, impl_);
  }

  /// @brief Register a diagnostic task by name and member function pointer.
  ///
  /// @tparam T   Class type owning the diagnostic method.
  /// @param name Diagnostic task name.
  /// @param c    Pointer to the owning instance; must outlive this Updater.
  /// @param f    Member function called each update cycle.
  template <class T>
  void add(
    const std::string name, T * c,
    void (T::*f)(::diagnostic_updater::DiagnosticStatusWrapper &))
  {
    std::visit([&](auto & impl) { impl->add(name, c, f); }, impl_);
  }

  /// @brief Remove a previously added task by name.
  /// @param name Task name passed to a prior add() call.
  /// @return true if a task with that name was found and removed; false otherwise.
  bool removeByName(const std::string name)
  {
    return std::visit([&](auto & impl) { return impl->removeByName(name); }, impl_);
  }

  /// @brief Get the current update period as rclcpp::Duration.
  auto getPeriod() const
  {
    return std::visit([](const auto & impl) { return impl->getPeriod(); }, impl_);
  }

  /// @brief Set the update period from rclcpp::Duration.
  ///
  /// Resets the internal timer to the new period.
  void setPeriod(rclcpp::Duration period)
  {
    std::visit([&](auto & impl) { impl->setPeriod(period); }, impl_);
  }

  /// @brief Set the update period in seconds.
  ///
  /// Convenience overload that converts to rclcpp::Duration internally.
  void setPeriod(double period)
  {
    std::visit([&](auto & impl) { impl->setPeriod(period); }, impl_);
  }

  /// @brief Force an immediate update of all known DiagnosticStatus tasks,
  ///        bypassing the period interval.
  ///
  /// Useful when something drastic happens (shutdown, self-test) and the latest
  /// status must be published immediately rather than waiting for the next tick.
  void force_update()
  {
    std::visit([&](auto & impl) { impl->force_update(); }, impl_);
  }

  /// @brief Publish a single status with the given level and message across all
  ///        known DiagnosticStatus tasks.
  ///
  /// @param lvl Diagnostic level
  ///            (diagnostic_msgs::msg::DiagnosticStatus::OK / WARN / ERROR / STALE).
  /// @param msg Status message attached to every task in the broadcast.
  void broadcast(unsigned char lvl, const std::string msg)
  {
    std::visit([&](auto & impl) { impl->broadcast(lvl, msg); }, impl_);
  }

  /// @brief Set the hardware ID embedded in every published DiagnosticStatus.
  /// @param hwid Hardware identifier string (free-form).
  void setHardwareID(const std::string & hwid)
  {
    std::visit([&](auto & impl) { impl->setHardwareID(hwid); }, impl_);
  }

  /// @brief printf-style variant of setHardwareID, with truncation reported via
  ///        RCLCPP_DEBUG when the formatted string overflows the internal buffer.
  ///
  /// We pre-format here (rather than forwarding the varargs to impl->setHardwareIDf)
  /// so we can warn on truncation; C-style varargs (`...`) cannot be forwarded
  /// cleanly across a std::visit boundary, and neither upstream impl exposes a
  /// `vsetHardwareID(va_list)` overload. After formatting, storage is delegated to
  /// setHardwareID -> impl, so the underlying hwid_ field stays in sync with the
  /// active backend.
  ///
  /// @param format printf-style format string.
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

  // Non-copyable and non-movable: `verbose_` is a reference bound at construction
  // to impl_'s active alternative's `verbose_` field. References cannot be reseated,
  // so neither copy-assign nor move (which would require rebinding) is meaningful.
  Updater(const Updater &) = delete;
  Updater & operator=(const Updater &) = delete;
  Updater(Updater &&) = delete;
  Updater & operator=(Updater &&) = delete;

private:
  rclcpp::Logger logger_;
  std::variant<RclcppImpl, AgnocastImpl> impl_;

public:
  /// Mirrors ::diagnostic_updater::Updater::verbose_ / ::agnocast::Updater::verbose_.
  /// Bound by reference so `updater.verbose_ = true;` writes through to the
  /// underlying impl, matching the field-access idiom of both upstream Updaters.
  bool & verbose_;
};

}  // namespace diagnostic_updater
}  // namespace autoware::agnocast_wrapper

#else  // !USE_AGNOCAST_ENABLED

namespace autoware::agnocast_wrapper
{
namespace diagnostic_updater
{

/// @brief Pass-through Updater for the non-Agnocast build.
///
/// Inherits from ::diagnostic_updater::Updater and re-declares only the
/// `Updater(Node*, double)` constructor; base-class constructors are not
/// implicitly inherited in C++, so the upstream template and interface-pointer
/// constructors stay hidden. This keeps the supported signature identical to
/// the agnocast-enabled build. All other public members are inherited as-is.
class Updater : public ::diagnostic_updater::Updater
{
public:
  /// @brief Construct from a wrapper Node and update period.
  ///
  /// In this build, autoware::agnocast_wrapper::Node is an alias for rclcpp::Node,
  /// so the call forwards directly to ::diagnostic_updater::Updater(NodeT, double).
  ///
  /// @param node   Wrapper node (alias for rclcpp::Node in this build).
  /// @param period Default update period in seconds; overridden by the
  ///               `diagnostic_updater.period` ros2 parameter if set.
  explicit Updater(autoware::agnocast_wrapper::Node * node, double period = 1.0)
  : ::diagnostic_updater::Updater(node, period)
  {
  }
};

}  // namespace diagnostic_updater
}  // namespace autoware::agnocast_wrapper

#endif
