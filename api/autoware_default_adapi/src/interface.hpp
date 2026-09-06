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

#ifndef INTERFACE_HPP_
#define INTERFACE_HPP_

#include <autoware/adapi_specs/interface.hpp>
#include <autoware/agnocast_wrapper/node.hpp>
#include <autoware/component_interface_utils/rclcpp.hpp>
#include <rclcpp/rclcpp.hpp>

namespace autoware::default_adapi
{

class InterfaceNode : public autoware::agnocast_wrapper::Node
{
public:
  explicit InterfaceNode(const rclcpp::NodeOptions & options);

private:
  // NodeAdaptor deduces its constructor argument separately from NodeT, so the node type has to
  // be named explicitly here and on every endpoint below.
  using NodeT = autoware::agnocast_wrapper::Node;
  using Version = autoware::adapi_specs::interface::Version;
  autoware::component_interface_utils::NodeAdaptor<NodeT> adaptor_{this};
  autoware::component_interface_utils::Service<Version, NodeT>::SharedPtr srv_;
};

}  // namespace autoware::default_adapi

#endif  // INTERFACE_HPP_
