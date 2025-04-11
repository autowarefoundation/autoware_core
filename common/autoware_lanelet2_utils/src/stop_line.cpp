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

#include <autoware_lanelet2_utils/stop_line.hpp>

#include <set>
#include <string>
#include <utility>
#include <vector>

namespace autoware::lanelet2_utils
{
std::vector<lanelet::ConstLineString3d> get_stop_lines_from_stop_sign(
  const lanelet::ConstLanelets & lanelets, const std::string & stop_sign_id)
{
  std::vector<lanelet::ConstLineString3d> stoplines;
  std::set<lanelet::Id> checklist;

  for (const auto & ll : lanelets) {
    std::vector<std::shared_ptr<const lanelet::TrafficSign>> traffic_sign_reg_elems =
      ll.regulatoryElementsAs<const lanelet::TrafficSign>();

    if (!traffic_sign_reg_elems.empty()) {
      for (const auto & ts : traffic_sign_reg_elems) {
        if (ts->type() != stop_sign_id) {
          continue;
        }

        lanelet::ConstLineStrings3d traffic_sign_stoplines = ts->refLines();
        if (!traffic_sign_stoplines.empty()) {
          auto id = traffic_sign_stoplines.front().id();
          if (checklist.find(id) == checklist.end()) {
            checklist.insert(id);
            stoplines.push_back(traffic_sign_stoplines.front());
          }
        }
      }
    }
  }
  return stoplines;
}

std::optional<lanelet::ConstLineString3d> get_stop_line_from_road_marking(
  const lanelet::Id lane_id, const lanelet::LaneletMapPtr & lanelet_map_ptr,
  const std::string & attribute_name, bool check_id_match)
{
  lanelet::ConstLanelet lanelet = lanelet_map_ptr->laneletLayer.get(lane_id);
  const auto road_markings = lanelet.regulatoryElementsAs<lanelet::autoware::RoadMarking>();
  lanelet::ConstLineStrings3d stop_line;

  for (const auto & road_marking : road_markings) {
    const std::string type =
      road_marking->roadMarking().attributeOr(lanelet::AttributeName::Type, "none");

    if (type == lanelet::AttributeValueString::StopLine) {
      if (check_id_match) {
        const int target_id = road_marking->roadMarking().attributeOr(attribute_name, 0);
        if (target_id != lane_id) {
          continue;
        }
      }
      stop_line.push_back(road_marking->roadMarking());
      break;
    }
  }

  if (stop_line.empty()) {
    return {};
  }
  return stop_line.front();
}

std::vector<lanelet::ConstLineString3d> get_stop_lines_from_no_stopping_area(
  const lanelet::ConstLanelets & lanelets)
{
  std::vector<lanelet::ConstLineString3d> stopLines;
  std::set<lanelet::Id> checklist;

  for (const auto & ll : lanelets) {
    std::vector<std::shared_ptr<const lanelet::autoware::NoStoppingArea>> no_stopping_elems =
      ll.regulatoryElementsAs<lanelet::autoware::NoStoppingArea>();

    if (!no_stopping_elems.empty()) {
      for (const auto & no_stopping_area : no_stopping_elems) {
        if (auto opt_stop_line = no_stopping_area->stopLine()) {
          const lanelet::ConstLineString3d stopLine = *opt_stop_line;
          const lanelet::Id id = stopLine.id();
          if (checklist.find(id) == checklist.end()) {
            checklist.insert(id);
            stopLines.push_back(stopLine);
          }
        }
      }
    }
  }
  return stopLines;
}

std::vector<lanelet::ConstLineString3d> get_stop_lines_from_detection_area(
  const lanelet::ConstLanelets & lanelets)
{
  std::vector<lanelet::ConstLineString3d> stopLines;
  std::set<lanelet::Id> checklist;

  for (const auto & ll : lanelets) {
    std::vector<std::shared_ptr<const lanelet::autoware::DetectionArea>> detection_areas =
      ll.regulatoryElementsAs<lanelet::autoware::DetectionArea>();

    if (!detection_areas.empty()) {
      for (const auto & detection_area : detection_areas) {
        const lanelet::ConstLineString3d stopLine = detection_area->stopLine();
        const lanelet::Id id = stopLine.id();
        if (checklist.find(id) == checklist.end()) {
          checklist.insert(id);
          stopLines.push_back(stopLine);
        }
      }
    }
  }

  return stopLines;
}

}  // namespace autoware::lanelet2_utils
