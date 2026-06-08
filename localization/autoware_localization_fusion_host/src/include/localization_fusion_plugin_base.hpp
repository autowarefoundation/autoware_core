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

#ifndef LOCALIZATION_FUSION_PLUGIN_BASE_HPP_
#define LOCALIZATION_FUSION_PLUGIN_BASE_HPP_

#include "fusion_data.hpp"

#include <rclcpp/rclcpp.hpp>

#include <string>
#include <vector>

namespace autoware::localization_fusion_host
{

class LocalizationFusionHost;

namespace plugin
{

class LocalizationFusionPluginBase
{
public:
  LocalizationFusionPluginBase() = default;
  virtual ~LocalizationFusionPluginBase() = default;

  virtual void initialize(
    const std::string & name, rclcpp::Node * node_ptr, LocalizationFusionHost * host_ptr)
  {
    name_ = name;
    node_ptr_ = node_ptr;
    host_ptr_ = host_ptr;
    set_up_params();
  }

  virtual void set_up_params() {}

  virtual void on_parameter(const std::vector<rclcpp::Parameter> & /*parameters*/) {}

  /// Runs after EKF estimate is available (downstream pipeline stages).
  virtual void process(FusionData & data) = 0;

  std::string get_name() const { return name_; }

protected:
  rclcpp::Node * get_node_ptr() const { return node_ptr_; }
  LocalizationFusionHost * get_host_ptr() const { return host_ptr_; }

private:
  std::string name_{"unnamed_plugin"};
  rclcpp::Node * node_ptr_{nullptr};
  LocalizationFusionHost * host_ptr_{nullptr};
};

}  // namespace plugin
}  // namespace autoware::localization_fusion_host

#endif  // LOCALIZATION_FUSION_PLUGIN_BASE_HPP_
