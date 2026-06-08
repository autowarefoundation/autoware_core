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

#ifndef PLUGINS__TWIST2ACCEL_PLUGIN_HPP_
#define PLUGINS__TWIST2ACCEL_PLUGIN_HPP_

#include "include/post_estimate_plugin_base.hpp"

#include <autoware/twist2accel/twist2accel_processor.hpp>
#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/accel_with_covariance_stamped.hpp>

#include <memory>
#include <vector>

namespace autoware::ekf_localizer::plugin
{

class Twist2AccelPlugin : public PostEstimatePluginBase
{
public:
  void set_up_params() override;
  void on_parameter(const std::vector<rclcpp::Parameter> & parameters) override;
  void process(FusionData & data) override;

private:
  std::unique_ptr<autoware::twist2accel::Twist2AccelProcessor> processor_;
  rclcpp::Publisher<geometry_msgs::msg::AccelWithCovarianceStamped>::SharedPtr accel_pub_;

  bool enabled_{true};
  bool use_odom_{true};
};

}  // namespace autoware::ekf_localizer::plugin

#endif  // PLUGINS__TWIST2ACCEL_PLUGIN_HPP_
