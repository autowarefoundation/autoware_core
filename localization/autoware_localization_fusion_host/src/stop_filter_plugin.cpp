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

#include "include/stop_filter_plugin.hpp"

#include <memory>
#include <vector>

namespace autoware::localization_fusion_host::plugin
{

void StopFilterPlugin::set_up_params()
{
  auto * node = get_node_ptr();
  const double vx_threshold = node->declare_parameter<double>("stop_filter.vx_threshold");
  const double wz_threshold = node->declare_parameter<double>("stop_filter.wz_threshold");
  processor_ =
    std::make_unique<autoware::stop_filter::StopFilterProcessor>(vx_threshold, wz_threshold);

  kinematic_state_pub_ =
    node->create_publisher<nav_msgs::msg::Odometry>("output/kinematic_state", 1);
  stop_flag_pub_ =
    node->create_publisher<autoware_internal_debug_msgs::msg::BoolStamped>("debug/stop_flag", 1);
  intermediate_odom_pub_ =
    node->create_publisher<nav_msgs::msg::Odometry>("debug/ekf_kinematic_state", 1);

  enabled_ = node->get_parameter("use_stop_filter").as_bool();
  publish_intermediate_outputs_ = node->get_parameter("publish_intermediate_outputs").as_bool();
}

void StopFilterPlugin::on_parameter(const std::vector<rclcpp::Parameter> & parameters)
{
  auto * node = get_node_ptr();
  for (const auto & param : parameters) {
    if (param.get_name() == "use_stop_filter") {
      enabled_ = param.as_bool();
    } else if (param.get_name() == "publish_intermediate_outputs") {
      publish_intermediate_outputs_ = param.as_bool();
    } else if (param.get_name() == "stop_filter.vx_threshold") {
      processor_ = std::make_unique<autoware::stop_filter::StopFilterProcessor>(
        param.as_double(), node->get_parameter("stop_filter.wz_threshold").as_double());
    } else if (param.get_name() == "stop_filter.wz_threshold") {
      processor_ = std::make_unique<autoware::stop_filter::StopFilterProcessor>(
        node->get_parameter("stop_filter.vx_threshold").as_double(), param.as_double());
    }
  }
}

void StopFilterPlugin::process(FusionData & data)
{
  if (!enabled_ || !data.has_ekf_estimate) {
    return;
  }

  const auto filtered = processor_->create_filtered_msg(data.ekf_odom);
  data.kinematic_state = filtered;
  data.has_kinematic_state = true;

  stop_flag_pub_->publish(processor_->create_stop_flag_msg(data.ekf_odom));
  kinematic_state_pub_->publish(filtered);

  if (publish_intermediate_outputs_) {
    intermediate_odom_pub_->publish(data.ekf_odom);
  }
}

}  // namespace autoware::localization_fusion_host::plugin

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(
  autoware::localization_fusion_host::plugin::StopFilterPlugin,
  autoware::localization_fusion_host::plugin::LocalizationFusionPluginBase)
