// Copyright 2026 Autoware Foundation
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

#ifndef HARNESS__DIAGNOSTICS_CAPTURE_HPP_
#define HARNESS__DIAGNOSTICS_CAPTURE_HPP_

#include <rclcpp/rclcpp.hpp>

#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <diagnostic_msgs/msg/diagnostic_status.hpp>

#include <algorithm>
#include <limits>
#include <map>
#include <mutex>
#include <optional>
#include <string>
#include <utility>
#include <vector>

namespace ndt_test
{

/// @brief Records everything published on `/diagnostics`, indexed by `status[0].name`.
///
/// `ndt_scan_matcher` writes nearly every intermediate decision of its callbacks into
/// diagnostics key-values, which makes `/diagnostics` the primary observable surface for
/// characterizing the node. Two properties of `DiagnosticsInterface` are load-bearing here
/// and are relied upon by the assertions:
///   - values are stringified with `std::to_string`, and bools become "True"/"False";
///   - `values` preserves *insertion* order, so the key order witnesses the code path taken.
///
/// The subscription uses `KeepAll().reliable()` because every `DiagnosticsInterface`
/// publishes with a volatile `QoS(10)`; a shallower or later subscription silently loses
/// messages.
class DiagnosticsCapture
{
public:
  /// @brief One captured `DiagnosticArray` (the node only ever puts one status in each).
  class Record
  {
  public:
    Record() = default;
    explicit Record(
      diagnostic_msgs::msg::DiagnosticStatus status, builtin_interfaces::msg::Time stamp)
    : status_(std::move(status)), stamp_(stamp)
    {
    }

    /// @brief Keys in the order `DiagnosticsInterface::add_key_value` inserted them.
    [[nodiscard]] std::vector<std::string> keys_in_order() const
    {
      std::vector<std::string> keys;
      keys.reserve(status_.values.size());
      for (const auto & kv : status_.values) {
        keys.push_back(kv.key);
      }
      return keys;
    }

    [[nodiscard]] bool has_key(const std::string & key) const
    {
      return std::any_of(status_.values.begin(), status_.values.end(), [&](const auto & kv) {
        return kv.key == key;
      });
    }

    /// @brief The raw (stringified) value, or "" when the key is absent.
    [[nodiscard]] std::string value(const std::string & key) const
    {
      const auto it = std::find_if(
        status_.values.begin(), status_.values.end(),
        [&](const auto & kv) { return kv.key == key; });
      return (it == status_.values.end()) ? std::string{} : it->value;
    }

    /// @brief Parsed value, or NaN when the key is absent, so a missing key fails the comparison
    /// it was used in rather than throwing out of `std::stod`.
    [[nodiscard]] double value_as_double(const std::string & key) const
    {
      return has_key(key) ? std::stod(value(key)) : std::numeric_limits<double>::quiet_NaN();
    }

    [[nodiscard]] int8_t level() const { return status_.level; }
    [[nodiscard]] const std::string & message() const { return status_.message; }
    [[nodiscard]] const builtin_interfaces::msg::Time & stamp() const { return stamp_; }

  private:
    diagnostic_msgs::msg::DiagnosticStatus status_{};
    builtin_interfaces::msg::Time stamp_{};
  };

  /// @param observer Node that owns the subscription. Must be spun by the caller.
  explicit DiagnosticsCapture(rclcpp::Node * observer)
  {
    subscription_ = observer->create_subscription<diagnostic_msgs::msg::DiagnosticArray>(
      "/diagnostics", rclcpp::QoS(rclcpp::KeepAll()).reliable(),
      [this](diagnostic_msgs::msg::DiagnosticArray::ConstSharedPtr msg) { on_message(*msg); });
  }

  /// @brief Readiness predicate: each `DiagnosticsInterface` creates its own `/diagnostics`
  /// publisher, so the node under test contributes a known, fixed number of them. Waiting on
  /// this count is an exact discovery gate that needs no sleeping.
  [[nodiscard]] size_t publisher_count() const { return subscription_->get_publisher_count(); }

  [[nodiscard]] std::vector<Record> records(const std::string & status_name) const
  {
    const std::lock_guard<std::mutex> lock(records_mutex_);
    const auto it = records_.find(status_name);
    return (it == records_.end()) ? std::vector<Record>{} : it->second;
  }

  [[nodiscard]] size_t count(const std::string & status_name) const
  {
    const std::lock_guard<std::mutex> lock(records_mutex_);
    const auto it = records_.find(status_name);
    return (it == records_.end()) ? 0U : it->second.size();
  }

  /// @brief Remember the current record count, so `newest_since_mark` can pick up only what
  /// arrives afterwards. Needed for the three service/timer statuses, which are stamped with
  /// `now()` and therefore cannot be correlated by an input stamp.
  void mark(const std::string & status_name) { marks_[status_name] = count(status_name); }

  [[nodiscard]] std::optional<Record> newest_since_mark(const std::string & status_name) const
  {
    const size_t mark = marks_.count(status_name) ? marks_.at(status_name) : 0U;
    const auto all = records(status_name);
    if (all.size() <= mark) {
      return std::nullopt;
    }
    return all.back();
  }

  /// @brief Find the record whose `header.stamp` equals `stamp`.
  ///
  /// `scan_matching_status`, `initial_pose_subscriber_status` and
  /// `regularization_pose_subscriber_status` are published with the *input message* stamp, so a
  /// unique input stamp identifies its resulting diagnostics record exactly. This is what lets
  /// the suite correlate cause and effect instead of waiting a fixed duration and hoping.
  [[nodiscard]] std::optional<Record> find_by_stamp(
    const std::string & status_name, const builtin_interfaces::msg::Time & stamp) const
  {
    for (const auto & record : records(status_name)) {
      if (record.stamp().sec == stamp.sec && record.stamp().nanosec == stamp.nanosec) {
        return record;
      }
    }
    return std::nullopt;
  }

private:
  void on_message(const diagnostic_msgs::msg::DiagnosticArray & msg)
  {
    const std::lock_guard<std::mutex> lock(records_mutex_);
    for (const auto & status : msg.status) {
      records_[status.name].emplace_back(status, msg.header.stamp);
    }
  }

  rclcpp::Subscription<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr subscription_;
  mutable std::mutex records_mutex_;
  std::map<std::string, std::vector<Record>> records_;
  std::map<std::string, size_t> marks_;
};

}  // namespace ndt_test

#endif  // HARNESS__DIAGNOSTICS_CAPTURE_HPP_
