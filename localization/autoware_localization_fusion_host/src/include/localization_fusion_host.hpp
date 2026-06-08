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

#ifndef LOCALIZATION_FUSION_HOST_HPP_
#define LOCALIZATION_FUSION_HOST_HPP_

#include "ekf_fusion_core.hpp"
#include "fusion_data.hpp"
#include "localization_fusion_plugin_base.hpp"

#include <pluginlib/class_loader.hpp>
#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/twist_with_covariance_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include <memory>
#include <string>
#include <vector>

namespace autoware::localization_fusion_host
{

class LocalizationFusionHost : public rclcpp::Node
{
public:
  explicit LocalizationFusionHost(const rclcpp::NodeOptions & options);

  void run_downstream_pipeline(
    const nav_msgs::msg::Odometry & ekf_odom,
    const geometry_msgs::msg::TwistWithCovarianceStamped & ekf_twist_cov);

  FusionData & get_fusion_data() { return fusion_data_; }

private:
  void set_up_params();
  void load_plugins();
  rcl_interfaces::msg::SetParametersResult on_parameter(
    const std::vector<rclcpp::Parameter> & parameters);

  void on_twist_estimator(geometry_msgs::msg::TwistWithCovarianceStamped::ConstSharedPtr msg);

  std::unique_ptr<EkfFusionCore> ekf_core_;
  std::unique_ptr<pluginlib::ClassLoader<plugin::LocalizationFusionPluginBase>> plugin_loader_;
  std::vector<std::shared_ptr<plugin::LocalizationFusionPluginBase>> pipeline_plugins_;

  rclcpp::Subscription<geometry_msgs::msg::TwistWithCovarianceStamped>::SharedPtr
    twist_estimator_sub_;

  FusionData fusion_data_;

  OnSetParametersCallbackHandle::SharedPtr set_param_res_;

  bool use_stop_filter_{true};
  bool use_twist2accel_{true};
  bool publish_intermediate_outputs_{false};
};

}  // namespace autoware::localization_fusion_host

#endif  // LOCALIZATION_FUSION_HOST_HPP_
