// Copyright 2021 TierIV
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

/*
 * Copyright 2019 Autoware Foundation. All rights reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 * Authors: Simon Thompson, Ryohsuke Mitsudome
 *
 */

#ifndef LANELET2_MAP_LOADER__LANELET2_MAP_LOADER_NODE_HPP_
#define LANELET2_MAP_LOADER__LANELET2_MAP_LOADER_NODE_HPP_

#include <autoware/component_interface_specs/map.hpp>
#include <rclcpp/rclcpp.hpp>

#include <memory>

namespace autoware::map_loader
{

class Lanelet2SelectedMapLoaderModule;

class Lanelet2MapLoaderNode : public rclcpp::Node
{
public:
  explicit Lanelet2MapLoaderNode(const rclcpp::NodeOptions & options);
  ~Lanelet2MapLoaderNode() override;

private:
  using MapProjectorInfo = autoware::component_interface_specs::map::MapProjectorInfo;
  using VectorMap = autoware::component_interface_specs::map::VectorMap;

  void on_map_projector_info(const MapProjectorInfo::Message::ConstSharedPtr msg);

  rclcpp::Subscription<MapProjectorInfo::Message>::SharedPtr sub_map_projector_info_;
  rclcpp::Publisher<VectorMap::Message>::SharedPtr pub_map_bin_;
  std::unique_ptr<Lanelet2SelectedMapLoaderModule> selected_map_loader_module_;
};

}  // namespace autoware::map_loader
#endif  // LANELET2_MAP_LOADER__LANELET2_MAP_LOADER_NODE_HPP_
