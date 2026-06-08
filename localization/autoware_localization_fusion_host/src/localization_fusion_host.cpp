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

#include "include/localization_fusion_host.hpp"

#include <functional>
#include <memory>
#include <string>
#include <vector>

namespace autoware::localization_fusion_host
{

LocalizationFusionHost::LocalizationFusionHost(const rclcpp::NodeOptions & options)
: Node("localization_fusion_host", options),
  plugin_loader_(
    std::make_unique<pluginlib::ClassLoader<plugin::LocalizationFusionPluginBase>>(
      "autoware_localization_fusion_host",
      "autoware::localization_fusion_host::plugin::LocalizationFusionPluginBase"))
{
  set_up_params();

  ekf_core_ = std::make_unique<EkfFusionCore>(
    this, [this](
            const nav_msgs::msg::Odometry & odom,
            const geometry_msgs::msg::TwistWithCovarianceStamped & twist_cov) {
      run_downstream_pipeline(odom, twist_cov);
    });
  ekf_core_->configure(publish_intermediate_outputs_, use_stop_filter_);

  load_plugins();

  set_param_res_ = add_on_set_parameters_callback(
    std::bind(&LocalizationFusionHost::on_parameter, this, std::placeholders::_1));

  twist_estimator_sub_ = create_subscription<geometry_msgs::msg::TwistWithCovarianceStamped>(
    "input/twist_estimator", 10,
    std::bind(&LocalizationFusionHost::on_twist_estimator, this, std::placeholders::_1));
}

void LocalizationFusionHost::set_up_params()
{
  const std::vector<std::string> default_plugins = {
    "autoware::localization_fusion_host::plugin::StopFilterPlugin",
    "autoware::localization_fusion_host::plugin::Twist2AccelPlugin",
  };
  declare_parameter("plugin_names", default_plugins);
  use_stop_filter_ = declare_parameter<bool>("use_stop_filter", true);
  use_twist2accel_ = declare_parameter<bool>("use_twist2accel", true);
  publish_intermediate_outputs_ = declare_parameter<bool>("publish_intermediate_outputs", false);
}

void LocalizationFusionHost::load_plugins()
{
  const auto plugin_names = get_parameter("plugin_names").as_string_array();

  for (const auto & plugin_name : plugin_names) {
    try {
      auto plugin = plugin_loader_->createSharedInstance(plugin_name);
      plugin->initialize(plugin_name, this, this);
      pipeline_plugins_.push_back(plugin);
      RCLCPP_INFO_STREAM(get_logger(), "Loaded plugin: " << plugin_name);
    } catch (const pluginlib::CreateClassException & e) {
      RCLCPP_ERROR_STREAM(
        get_logger(), "Failed to load plugin '" << plugin_name << "': " << e.what());
    }
  }
}

rcl_interfaces::msg::SetParametersResult LocalizationFusionHost::on_parameter(
  const std::vector<rclcpp::Parameter> & parameters)
{
  for (const auto & param : parameters) {
    if (param.get_name() == "use_stop_filter") {
      use_stop_filter_ = param.as_bool();
    } else if (param.get_name() == "use_twist2accel") {
      use_twist2accel_ = param.as_bool();
    } else if (param.get_name() == "publish_intermediate_outputs") {
      publish_intermediate_outputs_ = param.as_bool();
    }
  }

  if (ekf_core_) {
    ekf_core_->on_parameter(parameters);
  }

  for (auto & plugin : pipeline_plugins_) {
    plugin->on_parameter(parameters);
  }

  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  result.reason = "success";
  return result;
}

void LocalizationFusionHost::on_twist_estimator(
  geometry_msgs::msg::TwistWithCovarianceStamped::ConstSharedPtr msg)
{
  fusion_data_.twist_estimator = *msg;
  fusion_data_.has_twist_estimator = true;
}

void LocalizationFusionHost::run_downstream_pipeline(
  const nav_msgs::msg::Odometry & ekf_odom,
  const geometry_msgs::msg::TwistWithCovarianceStamped & ekf_twist_cov)
{
  fusion_data_.ekf_odom = ekf_odom;
  fusion_data_.ekf_twist_cov = ekf_twist_cov;
  fusion_data_.has_ekf_estimate = true;
  fusion_data_.has_kinematic_state = false;

  if (!use_stop_filter_) {
    fusion_data_.kinematic_state = ekf_odom;
    fusion_data_.has_kinematic_state = true;
  }

  for (auto & plugin : pipeline_plugins_) {
    plugin->process(fusion_data_);
  }
}

}  // namespace autoware::localization_fusion_host

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(autoware::localization_fusion_host::LocalizationFusionHost)
