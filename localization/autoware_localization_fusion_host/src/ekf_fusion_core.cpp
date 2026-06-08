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

#include "include/ekf_fusion_core.hpp"

#include <utility>
#include <vector>

namespace autoware::localization_fusion_host
{

EkfFusionCore::EkfFusionCore(rclcpp::Node * node, PostEstimateCallback callback)
: core_(std::make_unique<autoware::ekf_localizer::EKFLocalizerCore>(node))
{
  core_->set_post_estimate_callback(std::move(callback));
}

void EkfFusionCore::configure(bool publish_intermediate_outputs, bool use_stop_filter)
{
  publish_intermediate_outputs_ = publish_intermediate_outputs;
  use_stop_filter_ = use_stop_filter;
  update_publish_ekf_odom_topic();
}

void EkfFusionCore::on_parameter(const std::vector<rclcpp::Parameter> & parameters)
{
  for (const auto & param : parameters) {
    if (param.get_name() == "use_stop_filter") {
      use_stop_filter_ = param.as_bool();
    } else if (param.get_name() == "publish_intermediate_outputs") {
      publish_intermediate_outputs_ = param.as_bool();
    }
  }
  update_publish_ekf_odom_topic();
}

void EkfFusionCore::update_publish_ekf_odom_topic()
{
  core_->set_publish_ekf_odom_topic(publish_intermediate_outputs_ || !use_stop_filter_);
}

}  // namespace autoware::localization_fusion_host
