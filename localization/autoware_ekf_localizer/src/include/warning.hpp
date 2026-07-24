// Copyright 2022 Autoware Foundation
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

#ifndef WARNING_HPP_
#define WARNING_HPP_

#include <autoware/agnocast_wrapper/node.hpp>
#include <rclcpp/rclcpp.hpp>

#include <chrono>
#include <cstddef>
#include <string>

namespace autoware::ekf_localizer
{

class Warning
{
public:
  explicit Warning(autoware::agnocast_wrapper::Node * node) : node_(node) {}

  // Additive seam: an explicitly-requested no-op logger constructed without a node, so EKFModule
  // can be unit-tested outside of a ROS runtime. When node_ is null, warn/warn_throttle silently
  // do nothing. The std::nullptr_t parameter forces callers to opt in deliberately
  // (e.g. Warning(nullptr)); production code cannot accidentally default-construct a Warning and
  // silently drop safety-relevant logs, because there is no default constructor.
  explicit Warning(std::nullptr_t) : node_(nullptr) {}

  void warn(const std::string & message) const
  {
    if (node_ == nullptr) {
      return;
    }
    RCLCPP_WARN(node_->get_logger(), "%s", message.c_str());
  }

  void warn_throttle(const std::string & message, const int duration_milliseconds) const
  {
    if (node_ == nullptr) {
      return;
    }
    RCLCPP_WARN_THROTTLE(
      node_->get_logger(), *(node_->get_clock()),
      std::chrono::milliseconds(duration_milliseconds).count(), "%s", message.c_str());
  }

private:
  autoware::agnocast_wrapper::Node * node_;
};

}  // namespace autoware::ekf_localizer

#endif  // WARNING_HPP_
