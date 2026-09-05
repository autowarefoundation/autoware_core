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

#ifndef HARNESS__TOPIC_CAPTURE_HPP_
#define HARNESS__TOPIC_CAPTURE_HPP_

#include <rclcpp/rclcpp.hpp>

#include <cstddef>
#include <mutex>
#include <optional>
#include <string>
#include <vector>

namespace ndt_test
{

/// @brief Records every message seen on one topic.
///
/// `KeepAll().reliable()` is deliberate: several assertions count messages (for example
/// "published exactly once per scan"), and every publisher in `ndt_scan_matcher` is reliable
/// with a shallow depth, so a `KeepLast` subscription would drop bursts and turn a counting
/// assertion into a flake.
///
/// Absence assertions ("this topic was NOT published") are only meaningful once the
/// subscription exists, so captures must be created *before* the stimulus that could publish.
template <typename MsgT>
class TopicCapture
{
public:
  /// @param qos Not defaulted: `NdtHarness::capture` owns the default and always forwards it, so a
  /// second default here would be unreachable and free to drift out of step with it.
  TopicCapture(rclcpp::Node * observer, const std::string & topic, const rclcpp::QoS & qos)
  {
    subscription_ =
      observer->create_subscription<MsgT>(topic, qos, [this](typename MsgT::ConstSharedPtr msg) {
        const std::lock_guard<std::mutex> lock(mutex_);
        messages_.push_back(*msg);
      });
  }

  [[nodiscard]] size_t count() const
  {
    const std::lock_guard<std::mutex> lock(mutex_);
    return messages_.size();
  }

  /// @brief How many publishers this subscription has matched.
  ///
  /// The discovery gate for absence assertions, and the counterpart of
  /// `DiagnosticsCapture::publisher_count`. "This topic was not published" only means anything once
  /// the subscription has matched the node's publisher; before that it is a statement about
  /// discovery, not about the node. The node creates every publisher in its constructor, so in
  /// practice this is already non-zero by the time a test looks — waiting on it removes the
  /// assumption rather than a delay.
  [[nodiscard]] size_t publisher_count() const { return subscription_->get_publisher_count(); }

  [[nodiscard]] std::vector<MsgT> messages() const
  {
    const std::lock_guard<std::mutex> lock(mutex_);
    return messages_;
  }

  /// @brief A copy of the first message received, or nullopt.
  ///
  /// Returned **by value** while the lock is held, because handing out a reference into
  /// `messages_` would race the callback that appends to it. Callers must therefore bind the
  /// result to a value:
  ///
  /// @code
  ///   const auto published = capture->first();      // right
  ///   ASSERT_TRUE(published.has_value());
  ///   const auto & field = published->some.field;
  ///
  ///   const auto & field = capture->first()->some.field;   // dangles
  /// @endcode
  ///
  /// Lifetime extension does not save the second form: it applies to a reference bound to a
  /// temporary or to a subobject reached through member access, and `optional::operator->` is a
  /// function call, which ends the chain. The temporary dies at the end of the full-expression.
  [[nodiscard]] std::optional<MsgT> first() const
  {
    const std::lock_guard<std::mutex> lock(mutex_);
    if (messages_.empty()) {
      return std::nullopt;
    }
    return messages_.front();
  }

private:
  // `subscription_` is declared last so that it is destroyed *first*: members go in reverse
  // declaration order, and a callback still running would otherwise append to a `messages_` that
  // has already been destroyed. The suite pumps the observer from the test thread, so this cannot
  // bite today -- but this header is meant to be reused, and the mutex above only makes sense if
  // concurrent callbacks are assumed possible.
  mutable std::mutex mutex_;
  std::vector<MsgT> messages_;
  typename rclcpp::Subscription<MsgT>::SharedPtr subscription_;
};

}  // namespace ndt_test

#endif  // HARNESS__TOPIC_CAPTURE_HPP_
