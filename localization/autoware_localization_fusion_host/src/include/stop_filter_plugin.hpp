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

#ifndef STOP_FILTER_PLUGIN_HPP_
#define STOP_FILTER_PLUGIN_HPP_

#include "localization_fusion_plugin_base.hpp"

#include <autoware/stop_filter/stop_filter_processor.hpp>
#include <rclcpp/rclcpp.hpp>

#include <autoware_internal_debug_msgs/msg/bool_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include <memory>
#include <vector>

namespace autoware::localization_fusion_host::plugin
{

class StopFilterPlugin : public LocalizationFusionPluginBase
{
public:
  void set_up_params() override;
  void on_parameter(const std::vector<rclcpp::Parameter> & parameters) override;
  void process(FusionData & data) override;

private:
  std::unique_ptr<autoware::stop_filter::StopFilterProcessor> processor_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr kinematic_state_pub_;
  rclcpp::Publisher<autoware_internal_debug_msgs::msg::BoolStamped>::SharedPtr stop_flag_pub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr intermediate_odom_pub_;

  bool enabled_{true};
  bool publish_intermediate_outputs_{false};
};

}  // namespace autoware::localization_fusion_host::plugin

#endif  // STOP_FILTER_PLUGIN_HPP_
