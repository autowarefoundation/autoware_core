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

#ifndef EKF_FUSION_CORE_HPP_
#define EKF_FUSION_CORE_HPP_

#include <autoware/ekf_localizer/ekf_localizer_core.hpp>
#include <rclcpp/rclcpp.hpp>

#include <memory>
#include <vector>

namespace autoware::localization_fusion_host
{

class EkfFusionCore
{
public:
  using PostEstimateCallback = autoware::ekf_localizer::EKFLocalizerCore::PostEstimateCallback;

  explicit EkfFusionCore(rclcpp::Node * node, PostEstimateCallback callback);

  void configure(bool publish_intermediate_outputs, bool use_stop_filter);
  void on_parameter(const std::vector<rclcpp::Parameter> & parameters);

private:
  void update_publish_ekf_odom_topic();

  std::unique_ptr<autoware::ekf_localizer::EKFLocalizerCore> core_;
  bool publish_intermediate_outputs_{false};
  bool use_stop_filter_{true};
};

}  // namespace autoware::localization_fusion_host

#endif  // EKF_FUSION_CORE_HPP_
