// Copyright 2024 Autoware Foundation
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

#ifndef AUTOWARE__NDT_SCAN_MATCHER__HYPER_PARAMETERS_LOADER_HPP_
#define AUTOWARE__NDT_SCAN_MATCHER__HYPER_PARAMETERS_LOADER_HPP_

#include "hyper_parameters.hpp"

#include <autoware/agnocast_wrapper/node.hpp>

namespace autoware::ndt_scan_matcher
{

/**
 * \brief Declare every ROS parameter the node takes, and return them as plain values.
 *
 * The ROS-facing half of `HyperParameters`: it is the one place that knows the struct's fields
 * correspond to parameter names, so that holding the values costs no dependency on rclcpp. Only
 * the node calls it, once, at construction.
 *
 * `ndt.num_threads` is clamped up to 1, since a non-positive thread count would leave the NDT
 * unable to align anything.
 *
 * \param node the node to declare the parameters on.
 *
 * \throws std::runtime_error if `initial_pose_offset_model_x` and `initial_pose_offset_model_y`
 *         differ in length, since the two are read as coordinate pairs.
 */
HyperParameters declare_hyper_parameters(autoware::agnocast_wrapper::Node * node);

}  // namespace autoware::ndt_scan_matcher

#endif  // AUTOWARE__NDT_SCAN_MATCHER__HYPER_PARAMETERS_LOADER_HPP_
