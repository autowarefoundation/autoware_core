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

// Compile-time coverage for the policy-aware PollingSubscriber wrapper. The take_data() bodies live
// in class templates, so they are only type-checked when instantiated; the explicit instantiations
// below force the compiler to check every polling-policy branch (Latest / Newest / All) for both the
// Agnocast and ROS 2 backends without needing an Agnocast runtime. The static_asserts pin down the
// policy-dependent return types (single message vs. vector).

#include <autoware/agnocast_wrapper/node.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>

#include <gtest/gtest.h>

#include <memory>
#include <type_traits>
#include <vector>

namespace autoware::agnocast_wrapper
{
namespace
{
using Msg = geometry_msgs::msg::PoseStamped;
namespace pp = autoware_utils_rclcpp::polling_policy;
}  // namespace

#ifdef USE_AGNOCAST_ENABLED
// Force instantiation of all take_data() branches for both backends.
template class AgnocastPollingSubscriber<Msg, pp::Latest>;
template class AgnocastPollingSubscriber<Msg, pp::Newest>;
template class AgnocastPollingSubscriber<Msg, pp::All>;
template class ROS2PollingSubscriber<Msg, pp::Latest>;
template class ROS2PollingSubscriber<Msg, pp::Newest>;
template class ROS2PollingSubscriber<Msg, pp::All>;
#endif

}  // namespace autoware::agnocast_wrapper

TEST(PollingSubscriberCompile, ReturnTypes)
{
  using autoware::agnocast_wrapper::Msg;
  namespace pp = autoware_utils_rclcpp::polling_policy;

  // Latest / Newest yield a single message; All yields a vector of messages.
  static_assert(
    std::is_same_v<
      autoware::agnocast_wrapper::polling_take_result_t<Msg, pp::Latest>,
      AUTOWARE_MESSAGE_SHARED_PTR(const Msg)>,
    "Latest take_data() must return a single message");
  static_assert(
    std::is_same_v<
      autoware::agnocast_wrapper::polling_take_result_t<Msg, pp::Newest>,
      AUTOWARE_MESSAGE_SHARED_PTR(const Msg)>,
    "Newest take_data() must return a single message");
  static_assert(
    std::is_same_v<
      autoware::agnocast_wrapper::polling_take_result_t<Msg, pp::All>,
      std::vector<AUTOWARE_MESSAGE_SHARED_PTR(const Msg)>>,
    "All take_data() must return a vector of messages");

  SUCCEED();
}

TEST(PollingSubscriberCompile, ToRos2SharedYieldsStdSharedPtr)
{
  using autoware::agnocast_wrapper::to_ros2_shared;
  using Msg = autoware::agnocast_wrapper::Msg;

  // In both builds, to_ros2_shared() must normalize a polled message into a plain
  // std::shared_ptr<const Msg> (and a vector thereof), so downstream ROS-typed code is build-mode
  // agnostic.
  AUTOWARE_MESSAGE_SHARED_PTR(const Msg) scalar{};
  std::vector<AUTOWARE_MESSAGE_SHARED_PTR(const Msg)> vec{};
  static_assert(
    std::is_same_v<decltype(to_ros2_shared(scalar)), std::shared_ptr<const Msg>>,
    "to_ros2_shared(scalar) must return std::shared_ptr<const Msg>");
  static_assert(
    std::is_same_v<
      decltype(to_ros2_shared(std::move(vec))), std::vector<std::shared_ptr<const Msg>>>,
    "to_ros2_shared(vector) must return std::vector<std::shared_ptr<const Msg>>");

  SUCCEED();
}
