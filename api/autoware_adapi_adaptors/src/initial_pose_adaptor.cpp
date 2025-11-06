// Copyright 2022 TIER IV, Inc.
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

#include "initial_pose_adaptor.hpp"

#include <autoware/qos_utils/qos_compatibility.hpp>

#include <memory>
#include <string>
#include <vector>

namespace autoware::adapi_adaptors
{
template <class ServiceT>
using Future = typename rclcpp::Client<ServiceT>::SharedFuture;

std::array<double, 36> get_default_covariance()
{
  // Return identity covariance matrix (6x6 in row-major order)
  std::array<double, 36> covariance{};
  covariance.fill(0.0);
  // Set diagonal elements to 1.0 for position (x, y, z) and orientation (roll, pitch, yaw)
  for (size_t i = 0; i < 6; ++i) {
    covariance[i * 6 + i] = 1.0;
  }
  return covariance;
}

std::array<double, 36> get_covariance_parameter(rclcpp::Node * node, const std::string & name)
{
  try {
    const auto vector = node->declare_parameter<std::vector<double>>(name);
    if (vector.size() != 36) {
      RCLCPP_ERROR(
        node->get_logger(),
        "The covariance parameter '%s' size is %zu, expected 36. Using default identity "
        "covariance.",
        name.c_str(), vector.size());
      return get_default_covariance();
    }
    std::array<double, 36> array;
    std::copy_n(vector.begin(), array.size(), array.begin());
    return array;
  } catch (const rclcpp::exceptions::ParameterNotDeclaredException & e) {
    RCLCPP_WARN(
      node->get_logger(), "Parameter '%s' not declared: %s. Using default identity covariance.",
      name.c_str(), e.what());
    return get_default_covariance();
  } catch (const std::exception & e) {
    RCLCPP_ERROR(
      node->get_logger(), "Error retrieving parameter '%s': %s. Using default identity covariance.",
      name.c_str(), e.what());
    return get_default_covariance();
  }
}

InitialPoseAdaptor::InitialPoseAdaptor(const rclcpp::NodeOptions & options)
: Node("autoware_initial_pose_adaptor", options), fitter_(this)
{
  try {
    rviz_particle_covariance_ = get_covariance_parameter(this, "initial_pose_particle_covariance");
    sub_initial_pose_ = create_subscription<PoseWithCovarianceStamped>(
      "~/initialpose", rclcpp::QoS(1),
      std::bind(&InitialPoseAdaptor::on_initial_pose, this, std::placeholders::_1));

    cli_initialize_ =
      create_client<Initialize::Service>(Initialize::name, AUTOWARE_DEFAULT_SERVICES_QOS_PROFILE());

    RCLCPP_INFO(get_logger(), "InitialPoseAdaptor initialized successfully");
  } catch (const std::exception & e) {
    RCLCPP_ERROR(
      get_logger(), "Failed to initialize InitialPoseAdaptor: %s. Node may not function properly.",
      e.what());
    // Don't rethrow - allow node to start with degraded functionality
  }
}

void InitialPoseAdaptor::on_initial_pose(const PoseWithCovarianceStamped::ConstSharedPtr msg)
{
  PoseWithCovarianceStamped pose = *msg;
  const auto fitted = fitter_.fit(pose.pose.pose.position, pose.header.frame_id);
  if (fitted) {
    pose.pose.pose.position = fitted.value();
  }
  pose.pose.covariance = rviz_particle_covariance_;

  const auto req = std::make_shared<Initialize::Service::Request>();
  req->pose.push_back(pose);
  cli_initialize_->async_send_request(req);
}

}  // namespace autoware::adapi_adaptors

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(autoware::adapi_adaptors::InitialPoseAdaptor)
