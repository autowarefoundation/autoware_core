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

/// Records every message seen on one topic.
///
/// `KeepAll().reliable()` is deliberate: assertions count messages, and the node's publishers are
/// reliable with a shallow depth. Create the capture before the input that could publish.
template <typename MsgT>
class TopicCapture
{
public:
  /// `qos` is not defaulted, so it cannot drift from `NdtHarness::capture`, which always passes it.
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

  /// Publisher count, the discovery gate: silence means nothing until a publisher is connected.
  [[nodiscard]] size_t publisher_count() const { return subscription_->get_publisher_count(); }

  [[nodiscard]] std::vector<MsgT> messages() const
  {
    const std::lock_guard<std::mutex> lock(mutex_);
    return messages_;
  }

  /// A copy of the first message, or nullopt. Returned by value under the lock, so bind it to a
  /// value: `capture->first()->field` dangles.
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
