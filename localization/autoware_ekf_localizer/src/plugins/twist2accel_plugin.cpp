// Copyright 2025 Autoware Foundation
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

#include "plugins/twist2accel_plugin.hpp"

#include <memory>
#include <vector>

namespace autoware::ekf_localizer::plugin
{

void Twist2AccelPlugin::set_up_params()
{
  auto * node = get_node_ptr();
  const double accel_lowpass_gain =
    node->declare_parameter<double>("twist2accel.accel_lowpass_gain");
  use_odom_ = node->declare_parameter<bool>("twist2accel.use_odom");
  processor_ = std::make_unique<autoware::twist2accel::Twist2AccelProcessor>(accel_lowpass_gain);

  accel_pub_ = node->create_publisher<geometry_msgs::msg::AccelWithCovarianceStamped>(
    "output/acceleration", 1);
  enabled_ = node->get_parameter("use_twist2accel").as_bool();
}

void Twist2AccelPlugin::on_parameter(const std::vector<rclcpp::Parameter> & parameters)
{
  for (const auto & param : parameters) {
    if (param.get_name() == "use_twist2accel") {
      enabled_ = param.as_bool();
    } else if (param.get_name() == "twist2accel.accel_lowpass_gain") {
      processor_ = std::make_unique<autoware::twist2accel::Twist2AccelProcessor>(param.as_double());
    } else if (param.get_name() == "twist2accel.use_odom") {
      use_odom_ = param.as_bool();
    }
  }
}

void Twist2AccelPlugin::process(FusionData & data)
{
  if (!enabled_ || !data.has_kinematic_state) {
    return;
  }

  geometry_msgs::msg::TwistStamped twist;
  if (use_odom_) {
    twist.header = data.kinematic_state.header;
    twist.twist = data.kinematic_state.twist.twist;
  } else {
    if (!data.has_twist_estimator) {
      return;
    }
    twist.header = data.twist_estimator.header;
    twist.twist = data.twist_estimator.twist.twist;
  }

  accel_pub_->publish(processor_->estimate_accel(twist));
}

}  // namespace autoware::ekf_localizer::plugin

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(
  autoware::ekf_localizer::plugin::Twist2AccelPlugin,
  autoware::ekf_localizer::plugin::PostEstimatePluginBase)
