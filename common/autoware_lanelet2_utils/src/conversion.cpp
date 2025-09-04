// Copyright 2025 TIER IV, Inc.
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

#include <autoware/lanelet2_utils/conversion.hpp>
#include <autoware_lanelet2_extension/projection/mgrs_projector.hpp>
#include <autoware_lanelet2_extension/utility/utilities.hpp>

#include <boost/archive/binary_iarchive.hpp>
#include <boost/archive/binary_oarchive.hpp>

#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_core/primitives/Lanelet.h>
#include <lanelet2_io/Io.h>
#include <lanelet2_io/Projection.h>
#include <lanelet2_io/io_handlers/Serialize.h>
#include <lanelet2_routing/RoutingGraph.h>

#include <iostream>
#include <sstream>
#include <string>
#include <utility>

namespace autoware::experimental::lanelet2_utils
{

lanelet::LaneletMapConstPtr load_mgrs_coordinate_map(
  const std::string & path, const double centerline_resolution)
{
  lanelet::ErrorMessages errors{};
  lanelet::projection::MGRSProjector projector;
  auto lanelet_map_ptr_mut = lanelet::load(path, projector, &errors);

  for (auto & lanelet_obj : lanelet_map_ptr_mut->laneletLayer) {
    if (lanelet_obj.hasCustomCenterline()) {
      const auto & centerline = lanelet_obj.centerline();
      lanelet_obj.setAttribute("waypoints", centerline.id());
    }
    const auto fine_center_line =
      lanelet::utils::generateFineCenterline(lanelet_obj, centerline_resolution);
    lanelet_obj.setCenterline(fine_center_line);
  }
  return lanelet::LaneletMapConstPtr{std::move(lanelet_map_ptr_mut)};
}

std::pair<lanelet::routing::RoutingGraphConstPtr, lanelet::traffic_rules::TrafficRulesPtr>
instantiate_routing_graph_and_traffic_rules(
  lanelet::LaneletMapConstPtr lanelet_map, const char * location, const char * participant)
{
  auto traffic_rules = lanelet::traffic_rules::TrafficRulesFactory::create(location, participant);
  return {
    lanelet::routing::RoutingGraph::build(*lanelet_map, *traffic_rules), std::move(traffic_rules)};
}

void to_bin_msg(const lanelet::LaneletMapPtr & map, autoware_map_msgs::msg::LaneletMapBin * msg)
{
  if (msg == nullptr) {
    std::cerr << __FUNCTION__ << ": msg is null pointer!";
    return;
  }

  std::stringstream ss;
  boost::archive::binary_oarchive oa(ss);
  oa << *map;
  auto id_counter = lanelet::utils::getId();
  oa << id_counter;

  std::string data_str(ss.str());

  msg->data.clear();
  msg->data.assign(data_str.begin(), data_str.end());
}

void from_bin_msg(const autoware_map_msgs::msg::LaneletMapBin & msg, lanelet::LaneletMapPtr map)
{
  if (!map) {
    std::cerr << __FUNCTION__ << ": map is null pointer!";
    return;
  }

  std::string data_str;
  data_str.assign(msg.data.begin(), msg.data.end());

  std::stringstream ss;
  ss << data_str;
  boost::archive::binary_iarchive ia(ss);
  ia >> *map;
  lanelet::Id id_counter = 0;
  ia >> id_counter;
  lanelet::utils::registerId(id_counter);
}

void from_bin_msg(
  const autoware_map_msgs::msg::LaneletMapBin & msg, const lanelet::LaneletMapPtr map,
  lanelet::traffic_rules::TrafficRulesPtr * traffic_rules,
  lanelet::routing::RoutingGraphPtr * routing_graph)
{
  from_bin_msg(msg, map);
  *traffic_rules = lanelet::traffic_rules::TrafficRulesFactory::create(
    lanelet::Locations::Germany, lanelet::Participants::Vehicle);
  *routing_graph = lanelet::routing::RoutingGraph::build(*map, **traffic_rules);
}

void to_geom_msg_pt(const geometry_msgs::msg::Point32 & src, geometry_msgs::msg::Point * dst)
{
  if (dst == nullptr) {
    std::cerr << __FUNCTION__ << "pointer is null!";
    return;
  }
  dst->x = src.x;
  dst->y = src.y;
  dst->z = src.z;
}
void to_geom_msg_pt(const Eigen::Vector3d & src, geometry_msgs::msg::Point * dst)
{
  if (dst == nullptr) {
    std::cerr << __FUNCTION__ << "pointer is null!";
    return;
  }
  dst->x = src.x();
  dst->y = src.y();
  dst->z = src.z();
}
void to_geom_msg_pt(const lanelet::ConstPoint3d & src, geometry_msgs::msg::Point * dst)
{
  if (dst == nullptr) {
    std::cerr << __FUNCTION__ << "pointer is null!";
    return;
  }
  dst->x = src.x();
  dst->y = src.y();
  dst->z = src.z();
}
void to_geom_msg_pt(const lanelet::ConstPoint2d & src, geometry_msgs::msg::Point * dst)
{
  if (dst == nullptr) {
    std::cerr << __FUNCTION__ << "pointer is null!" << std::endl;
    return;
  }
  dst->x = src.x();
  dst->y = src.y();
  dst->z = 0;
}

void to_geom_msg_pt32(const Eigen::Vector3d & src, geometry_msgs::msg::Point32 * dst)
{
  if (dst == nullptr) {
    std::cerr << __FUNCTION__ << "pointer is null!" << std::endl;
    return;
  }
  dst->x = static_cast<float>(src.x());
  dst->y = static_cast<float>(src.y());
  dst->z = static_cast<float>(src.z());
}

geometry_msgs::msg::Point to_geom_msg_pt(const geometry_msgs::msg::Point32 & src)
{
  geometry_msgs::msg::Point dst;
  to_geom_msg_pt(src, &dst);
  return dst;
}
geometry_msgs::msg::Point to_geom_msg_pt(const Eigen::Vector3d & src)
{
  geometry_msgs::msg::Point dst;
  to_geom_msg_pt(src, &dst);
  return dst;
}
geometry_msgs::msg::Point to_geom_msg_pt(const lanelet::ConstPoint3d & src)
{
  geometry_msgs::msg::Point dst;
  to_geom_msg_pt(src, &dst);
  return dst;
}
geometry_msgs::msg::Point to_geom_msg_pt(const lanelet::ConstPoint2d & src)
{
  geometry_msgs::msg::Point dst;
  to_geom_msg_pt(src, &dst);
  return dst;
}

void to_lanelet_point(const geometry_msgs::msg::Point & src, lanelet::ConstPoint3d * dst)
{
  *dst = lanelet::Point3d(lanelet::InvalId, src.x, src.y, src.z);
}

lanelet::ConstPoint3d to_lanelet_point(const geometry_msgs::msg::Point & src)
{
  lanelet::ConstPoint3d dst;
  to_lanelet_point(src, &dst);
  return dst;
}

void to_geom_msg_poly(
  const lanelet::ConstPolygon3d & ll_poly, geometry_msgs::msg::Polygon * geom_poly)
{
  geom_poly->points.clear();
  geom_poly->points.reserve(ll_poly.size());
  for (const auto & ll_pt : ll_poly) {
    geometry_msgs::msg::Point32 geom_pt32;
    to_geom_msg_pt32(ll_pt.basicPoint(), &geom_pt32);
    geom_poly->points.push_back(geom_pt32);
  }
}

}  // namespace autoware::experimental::lanelet2_utils
