// Copyright 2026 TIER IV, Inc.
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

#include "lanelet2_map_visualization.hpp"

#include <autoware_lanelet2_extension/regulatory_elements/autoware_traffic_light.hpp>
#include <autoware_lanelet2_extension/utility/query.hpp>
#include <autoware_lanelet2_extension/visualization/visualization.hpp>
#include <autoware_utils_visualization/marker_helper.hpp>

#include <std_msgs/msg/color_rgba.hpp>

#include <lanelet2_core/LaneletMap.h>

#include <vector>

namespace autoware::lanelet2_map_visualizer
{
visualization_msgs::msg::MarkerArray create_lanelet_map_marker_array(
  const lanelet::LaneletMapConstPtr & viz_lanelet_map, const bool viz_centerline)
{
  // get lanelets etc to visualize
  lanelet::ConstLanelets all_lanelets = lanelet::utils::query::laneletLayer(viz_lanelet_map);
  lanelet::ConstLanelets road_lanelets = lanelet::utils::query::roadLanelets(all_lanelets);
  lanelet::ConstLanelets shoulder_lanelets = lanelet::utils::query::shoulderLanelets(all_lanelets);
  lanelet::ConstLanelets crosswalk_lanelets =
    lanelet::utils::query::crosswalkLanelets(all_lanelets);
  lanelet::ConstLanelets bicycle_lane_lanelets =
    lanelet::utils::query::bicycleLaneLanelets(all_lanelets);
  lanelet::ConstLineStrings3d partitions = lanelet::utils::query::getAllPartitions(viz_lanelet_map);
  lanelet::ConstLineStrings3d road_borders =
    lanelet::utils::query::getAllLinestringsWithType(viz_lanelet_map, "road_border");
  lanelet::ConstLineStrings3d pedestrian_polygon_markings =
    lanelet::utils::query::getAllPedestrianPolygonMarkings(viz_lanelet_map);
  lanelet::ConstLineStrings3d pedestrian_line_markings =
    lanelet::utils::query::getAllPedestrianLineMarkings(viz_lanelet_map);
  lanelet::ConstLanelets walkway_lanelets = lanelet::utils::query::walkwayLanelets(all_lanelets);
  std::vector<lanelet::ConstLineString3d> stop_lines =
    lanelet::utils::query::stopLinesLanelets(road_lanelets);
  std::vector<lanelet::AutowareTrafficLightConstPtr> aw_tl_reg_elems =
    lanelet::utils::query::autowareTrafficLights(all_lanelets);
  std::vector<lanelet::DetectionAreaConstPtr> da_reg_elems =
    lanelet::utils::query::detectionAreas(all_lanelets);
  std::vector<lanelet::NoStoppingAreaConstPtr> no_reg_elems =
    lanelet::utils::query::noStoppingAreas(all_lanelets);
  std::vector<lanelet::SpeedBumpConstPtr> sb_reg_elems =
    lanelet::utils::query::speedBumps(all_lanelets);
  std::vector<lanelet::CrosswalkConstPtr> cw_reg_elems =
    lanelet::utils::query::crosswalks(all_lanelets);
  lanelet::ConstLineStrings3d parking_spaces =
    lanelet::utils::query::getAllParkingSpaces(viz_lanelet_map);
  lanelet::ConstPolygons3d parking_lots = lanelet::utils::query::getAllParkingLots(viz_lanelet_map);
  lanelet::ConstPolygons3d obstacle_polygons =
    lanelet::utils::query::getAllObstaclePolygons(viz_lanelet_map);
  lanelet::ConstPolygons3d no_obstacle_segmentation_area =
    lanelet::utils::query::getAllPolygonsByType(viz_lanelet_map, "no_obstacle_segmentation_area");
  lanelet::ConstPolygons3d no_obstacle_segmentation_area_for_run_out =
    lanelet::utils::query::getAllPolygonsByType(
      viz_lanelet_map, "no_obstacle_segmentation_area_for_run_out");
  lanelet::ConstPolygons3d hatched_road_markings_area =
    lanelet::utils::query::getAllPolygonsByType(viz_lanelet_map, "hatched_road_markings");
  lanelet::ConstPolygons3d intersection_areas =
    lanelet::utils::query::getAllPolygonsByType(viz_lanelet_map, "intersection_area");
  std::vector<lanelet::NoParkingAreaConstPtr> no_parking_reg_elems =
    lanelet::utils::query::noParkingAreas(all_lanelets);
  lanelet::ConstLineStrings3d curbstones = lanelet::utils::query::curbstones(viz_lanelet_map);
  std::vector<lanelet::BusStopAreaConstPtr> bus_stop_reg_elems =
    lanelet::utils::query::busStopAreas(all_lanelets);
  lanelet::ConstLineStrings3d waypoints = lanelet::utils::query::getAllWaypoints(viz_lanelet_map);
  lanelet::ConstPolygons3d obstacle_removal_areas =
    lanelet::utils::query::getAllPolygonsByType(viz_lanelet_map, "obstacle_removal_area");

  std::vector<lanelet::ConstArea> lanelet_areas;
  lanelet_areas.reserve(viz_lanelet_map->areaLayer.size());
  for (const auto & area : viz_lanelet_map->areaLayer) {
    lanelet_areas.push_back(area);
  }

  const std_msgs::msg::ColorRGBA cl_road =
    autoware_utils_visualization::create_marker_color(0.27, 0.27, 0.27, 0.999);
  const std_msgs::msg::ColorRGBA cl_shoulder =
    autoware_utils_visualization::create_marker_color(0.15, 0.15, 0.15, 0.999);
  const std_msgs::msg::ColorRGBA cl_cross =
    autoware_utils_visualization::create_marker_color(0.27, 0.3, 0.27, 0.5);
  const std_msgs::msg::ColorRGBA cl_partitions =
    autoware_utils_visualization::create_marker_color(0.25, 0.25, 0.25, 0.999);
  const std_msgs::msg::ColorRGBA cl_road_borders =
    autoware_utils_visualization::create_marker_color(0.3, 0.25, 0.3, 0.999);
  const std_msgs::msg::ColorRGBA cl_pedestrian_markings =
    autoware_utils_visualization::create_marker_color(0.5, 0.5, 0.5, 0.999);
  const std_msgs::msg::ColorRGBA cl_ll_borders =
    autoware_utils_visualization::create_marker_color(0.5, 0.5, 0.5, 0.999);
  const std_msgs::msg::ColorRGBA cl_shoulder_borders =
    autoware_utils_visualization::create_marker_color(0.2, 0.2, 0.2, 0.999);
  const std_msgs::msg::ColorRGBA cl_stoplines =
    autoware_utils_visualization::create_marker_color(0.5, 0.5, 0.5, 0.999);
  const std_msgs::msg::ColorRGBA cl_trafficlights =
    autoware_utils_visualization::create_marker_color(0.5, 0.5, 0.5, 0.8);
  const std_msgs::msg::ColorRGBA cl_detection_areas =
    autoware_utils_visualization::create_marker_color(0.27, 0.27, 0.37, 0.5);
  const std_msgs::msg::ColorRGBA cl_speed_bumps =
    autoware_utils_visualization::create_marker_color(0.56, 0.40, 0.27, 0.5);
  const std_msgs::msg::ColorRGBA cl_crosswalks =
    autoware_utils_visualization::create_marker_color(0.80, 0.80, 0.0, 0.5);
  const std_msgs::msg::ColorRGBA cl_parking_lots =
    autoware_utils_visualization::create_marker_color(1.0, 1.0, 1.0, 0.2);
  const std_msgs::msg::ColorRGBA cl_parking_spaces =
    autoware_utils_visualization::create_marker_color(1.0, 1.0, 1.0, 0.3);
  const std_msgs::msg::ColorRGBA cl_lanelet_id =
    autoware_utils_visualization::create_marker_color(0.5, 0.5, 0.5, 0.999);
  const std_msgs::msg::ColorRGBA cl_obstacle_polygons =
    autoware_utils_visualization::create_marker_color(0.4, 0.27, 0.27, 0.5);
  const std_msgs::msg::ColorRGBA cl_no_stopping_areas =
    autoware_utils_visualization::create_marker_color(0.37, 0.37, 0.37, 0.5);
  const std_msgs::msg::ColorRGBA cl_no_obstacle_segmentation_area =
    autoware_utils_visualization::create_marker_color(0.37, 0.37, 0.27, 0.5);
  const std_msgs::msg::ColorRGBA cl_no_obstacle_segmentation_area_for_run_out =
    autoware_utils_visualization::create_marker_color(0.37, 0.7, 0.27, 0.5);
  const std_msgs::msg::ColorRGBA cl_hatched_road_markings_area =
    autoware_utils_visualization::create_marker_color(0.3, 0.3, 0.3, 0.5);
  const std_msgs::msg::ColorRGBA cl_hatched_road_markings_line =
    autoware_utils_visualization::create_marker_color(0.5, 0.5, 0.5, 0.999);
  const std_msgs::msg::ColorRGBA cl_no_parking_areas =
    autoware_utils_visualization::create_marker_color(0.42, 0.42, 0.42, 0.5);
  const std_msgs::msg::ColorRGBA cl_curbstones =
    autoware_utils_visualization::create_marker_color(0.1, 0.1, 0.2, 0.999);
  const std_msgs::msg::ColorRGBA cl_intersection_area =
    autoware_utils_visualization::create_marker_color(0.16, 1.0, 0.69, 0.5);
  const std_msgs::msg::ColorRGBA cl_bus_stop_area =
    autoware_utils_visualization::create_marker_color(0.863, 0.863, 0.863, 0.5);
  const std_msgs::msg::ColorRGBA cl_bicycle_lane =
    autoware_utils_visualization::create_marker_color(0.0, 0.3843, 0.6274, 0.5);
  const std_msgs::msg::ColorRGBA cl_waypoints =
    autoware_utils_visualization::create_marker_color(0.6, 0.4, 0.3, 0.999);
  const std_msgs::msg::ColorRGBA cl_obstacle_removal_area =
    autoware_utils_visualization::create_marker_color(0.2, 0.2, 0.5, 0.5);
  const std_msgs::msg::ColorRGBA cl_lanelet_routing_area =
    autoware_utils_visualization::create_marker_color(0.8, 0.53, 0.25, 0.35);
  const std_msgs::msg::ColorRGBA cl_lanelet_routing_area_outline =
    autoware_utils_visualization::create_marker_color(0.45, 0.28, 0.12, 0.95);

  visualization_msgs::msg::MarkerArray map_marker_array;

  autoware_utils_visualization::append_marker_array(
    lanelet::visualization::lineStringsAsMarkerArray(stop_lines, "stop_lines", cl_stoplines, 0.5),
    &map_marker_array);
  autoware_utils_visualization::append_marker_array(
    lanelet::visualization::lineStringsAsMarkerArray(partitions, "partitions", cl_partitions, 0.1),
    &map_marker_array);
  autoware_utils_visualization::append_marker_array(
    lanelet::visualization::lineStringsAsMarkerArray(
      road_borders, "road_borders", cl_road_borders, 0.2),
    &map_marker_array);
  autoware_utils_visualization::append_marker_array(
    lanelet::visualization::laneletDirectionAsMarkerArray(shoulder_lanelets, "shoulder_"),
    &map_marker_array);
  autoware_utils_visualization::append_marker_array(
    lanelet::visualization::laneletDirectionAsMarkerArray(road_lanelets), &map_marker_array);
  autoware_utils_visualization::append_marker_array(
    lanelet::visualization::laneletsAsTriangleMarkerArray(
      "crosswalk_lanelets", crosswalk_lanelets, cl_cross),
    &map_marker_array);
  autoware_utils_visualization::append_marker_array(
    lanelet::visualization::pedestrianPolygonMarkingsAsMarkerArray(
      pedestrian_polygon_markings, cl_pedestrian_markings),
    &map_marker_array);

  autoware_utils_visualization::append_marker_array(
    lanelet::visualization::pedestrianLineMarkingsAsMarkerArray(
      pedestrian_line_markings, cl_pedestrian_markings),
    &map_marker_array);
  autoware_utils_visualization::append_marker_array(
    lanelet::visualization::laneletsAsTriangleMarkerArray(
      "walkway_lanelets", walkway_lanelets, cl_cross),
    &map_marker_array);
  autoware_utils_visualization::append_marker_array(
    lanelet::visualization::obstaclePolygonsAsMarkerArray(obstacle_polygons, cl_obstacle_polygons),
    &map_marker_array);
  autoware_utils_visualization::append_marker_array(
    lanelet::visualization::detectionAreasAsMarkerArray(da_reg_elems, cl_detection_areas),
    &map_marker_array);
  autoware_utils_visualization::append_marker_array(
    lanelet::visualization::noStoppingAreasAsMarkerArray(no_reg_elems, cl_no_stopping_areas),
    &map_marker_array);
  autoware_utils_visualization::append_marker_array(
    lanelet::visualization::speedBumpsAsMarkerArray(sb_reg_elems, cl_speed_bumps),
    &map_marker_array);
  autoware_utils_visualization::append_marker_array(
    lanelet::visualization::crosswalkAreasAsMarkerArray(cw_reg_elems, cl_crosswalks),
    &map_marker_array);
  autoware_utils_visualization::append_marker_array(
    lanelet::visualization::parkingLotsAsMarkerArray(parking_lots, cl_parking_lots),
    &map_marker_array);
  autoware_utils_visualization::append_marker_array(
    lanelet::visualization::parkingSpacesAsMarkerArray(parking_spaces, cl_parking_spaces),
    &map_marker_array);
  autoware_utils_visualization::append_marker_array(
    lanelet::visualization::laneletsBoundaryAsMarkerArray(
      shoulder_lanelets, cl_shoulder_borders, viz_centerline, "shoulder_"),
    &map_marker_array);
  autoware_utils_visualization::append_marker_array(
    lanelet::visualization::laneletsBoundaryAsMarkerArray(
      road_lanelets, cl_ll_borders, viz_centerline),
    &map_marker_array);
  autoware_utils_visualization::append_marker_array(
    lanelet::visualization::autowareTrafficLightsAsMarkerArray(aw_tl_reg_elems, cl_trafficlights),
    &map_marker_array);
  autoware_utils_visualization::append_marker_array(
    lanelet::visualization::generateTrafficLightRegulatoryElementIdMaker(
      road_lanelets, cl_trafficlights),
    &map_marker_array);
  autoware_utils_visualization::append_marker_array(
    lanelet::visualization::generateTrafficLightRegulatoryElementIdMaker(
      crosswalk_lanelets, cl_trafficlights),
    &map_marker_array);
  autoware_utils_visualization::append_marker_array(
    lanelet::visualization::generateTrafficLightIdMaker(aw_tl_reg_elems, cl_trafficlights),
    &map_marker_array);
  autoware_utils_visualization::append_marker_array(
    lanelet::visualization::generateLaneletIdMarker(shoulder_lanelets, cl_lanelet_id),
    &map_marker_array);
  autoware_utils_visualization::append_marker_array(
    lanelet::visualization::generateLaneletIdMarker(road_lanelets, cl_lanelet_id),
    &map_marker_array);
  autoware_utils_visualization::append_marker_array(
    lanelet::visualization::generateLaneletIdMarker(
      crosswalk_lanelets, cl_lanelet_id, "crosswalk_lanelet_id"),
    &map_marker_array);
  autoware_utils_visualization::append_marker_array(
    lanelet::visualization::laneletsAsTriangleMarkerArray(
      "shoulder_road_lanelets", shoulder_lanelets, cl_shoulder),
    &map_marker_array);
  autoware_utils_visualization::append_marker_array(
    lanelet::visualization::laneletsAsTriangleMarkerArray("road_lanelets", road_lanelets, cl_road),
    &map_marker_array);
  autoware_utils_visualization::append_marker_array(
    lanelet::visualization::noObstacleSegmentationAreaAsMarkerArray(
      no_obstacle_segmentation_area, cl_no_obstacle_segmentation_area),
    &map_marker_array);
  autoware_utils_visualization::append_marker_array(
    lanelet::visualization::noObstacleSegmentationAreaForRunOutAsMarkerArray(
      no_obstacle_segmentation_area_for_run_out, cl_no_obstacle_segmentation_area_for_run_out),
    &map_marker_array);

  autoware_utils_visualization::append_marker_array(
    lanelet::visualization::hatchedRoadMarkingsAreaAsMarkerArray(
      hatched_road_markings_area, cl_hatched_road_markings_area, cl_hatched_road_markings_line),
    &map_marker_array);

  autoware_utils_visualization::append_marker_array(
    lanelet::visualization::noParkingAreasAsMarkerArray(no_parking_reg_elems, cl_no_parking_areas),
    &map_marker_array);

  autoware_utils_visualization::append_marker_array(
    lanelet::visualization::lineStringsAsMarkerArray(curbstones, "curbstone", cl_curbstones, 0.2),
    &map_marker_array);

  autoware_utils_visualization::append_marker_array(
    lanelet::visualization::intersectionAreaAsMarkerArray(intersection_areas, cl_intersection_area),
    &map_marker_array);

  autoware_utils_visualization::append_marker_array(
    lanelet::visualization::busStopAreasAsMarkerArray(bus_stop_reg_elems, cl_bus_stop_area),
    &map_marker_array);

  autoware_utils_visualization::append_marker_array(
    lanelet::visualization::laneletDirectionAsMarkerArray(bicycle_lane_lanelets, "bicycle_lane_"),
    &map_marker_array);
  autoware_utils_visualization::append_marker_array(
    lanelet::visualization::laneletsBoundaryAsMarkerArray(
      bicycle_lane_lanelets, cl_ll_borders /* use ll_border color */, viz_centerline,
      "bicycle_lane_"),
    &map_marker_array);
  autoware_utils_visualization::append_marker_array(
    lanelet::visualization::generateLaneletIdMarker(
      bicycle_lane_lanelets, cl_lanelet_id /* use lanelet_id color */),
    &map_marker_array);
  autoware_utils_visualization::append_marker_array(
    lanelet::visualization::laneletsAsTriangleMarkerArray(
      "bicycle_lane_lanelets", bicycle_lane_lanelets, cl_bicycle_lane),
    &map_marker_array);

  autoware_utils_visualization::append_marker_array(
    lanelet::visualization::lineStringsAsMarkerArray(waypoints, "waypoints", cl_waypoints, 0.02),
    &map_marker_array);
  autoware_utils_visualization::append_marker_array(
    lanelet::visualization::obstacleRemovalAreaAsMarkerArray(
      obstacle_removal_areas, cl_obstacle_removal_area),
    &map_marker_array);
  autoware_utils_visualization::append_marker_array(
    lanelet::visualization::laneletAreasAsMarkerArray(
      lanelet_areas, cl_lanelet_routing_area, cl_lanelet_routing_area_outline),
    &map_marker_array);

  return map_marker_array;
}
}  // namespace autoware::lanelet2_map_visualizer
