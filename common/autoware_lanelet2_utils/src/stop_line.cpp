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

#include <autoware/lanelet2_utils/stop_line.hpp>
#include <autoware_lanelet2_extension/regulatory_elements/detection_area.hpp>
#include <autoware_lanelet2_extension/regulatory_elements/no_stopping_area.hpp>
#include <autoware_lanelet2_extension/regulatory_elements/road_marking.hpp>

#include <lanelet2_core/LaneletMap.h>

#include <set>
#include <string>
#include <utility>
#include <vector>

namespace autoware::experimental::lanelet2_utils
{
std::optional<lanelet::ConstLineString3d> get_stop_line_from_deprecated_crosswalk(
  const lanelet::ConstLanelet & lanelet)
{
  std::vector<std::shared_ptr<const lanelet::autoware::RoadMarking>> road_markings =
    lanelet.regulatoryElementsAs<lanelet::autoware::RoadMarking>();

  if (!road_markings.empty()) {
    for (const auto & road_marking : road_markings) {
      const std::string type =
        road_marking->roadMarking().attributeOr(lanelet::AttributeName::Type, "none");
      if (type == lanelet::AttributeValueString::StopLine) {
        lanelet::ConstLineString3d stop_line = road_marking->roadMarking();
        return stop_line;
      }
    }
  }
  return std::nullopt;
}

std::optional<lanelet::ConstLineString3d> get_stop_lines_from_no_stopping_area(
  const lanelet::ConstLanelet & lanelet)
{
  std::vector<std::shared_ptr<const lanelet::autoware::NoStoppingArea>> no_stopping_elems =
    lanelet.regulatoryElementsAs<lanelet::autoware::NoStoppingArea>();

  if (!no_stopping_elems.empty()) {
    for (const auto & no_stopping_area : no_stopping_elems) {
      if (auto opt_stop_line = no_stopping_area->stopLine()) {
        const lanelet::ConstLineString3d stopLine = *opt_stop_line;
        return stopLine;
      }
    }
  }
  return std::nullopt;
}

std::optional<lanelet::ConstLineString3d> get_stop_lines_from_detection_area(
  const lanelet::ConstLanelet & lanelet)
{
  std::vector<std::shared_ptr<const lanelet::autoware::DetectionArea>> detection_areas =
    lanelet.regulatoryElementsAs<lanelet::autoware::DetectionArea>();

  if (!detection_areas.empty()) {
    for (const auto & detection_area : detection_areas) {
      const lanelet::ConstLineString3d stopLine = detection_area->stopLine();
      return stopLine;
    }
  }
  return std::nullopt;
}

std::optional<lanelet::ConstLineString3d> get_stop_line_from_intersection_marking(
  const lanelet::ConstLanelet & lanelet)
{
  std::vector<std::shared_ptr<const lanelet::autoware::RoadMarking>> road_markings =
    lanelet.regulatoryElementsAs<lanelet::autoware::RoadMarking>();

  if (!road_markings.empty()) {
    for (const auto & road_marking : road_markings) {
      const std::string type =
        road_marking->roadMarking().attributeOr(lanelet::AttributeName::Type, "none");
      if (type == lanelet::AttributeValueString::StopLine) {
        lanelet::ConstLineString3d stop_line = road_marking->roadMarking();
        return stop_line;
      }
    }
  }
  return std::nullopt;
}

std::optional<lanelet::ConstLineString3d> get_stop_lines_from_stop_sign(
  const lanelet::ConstLanelet & lanelet)
{
  std::vector<std::shared_ptr<const lanelet::TrafficSign>> traffic_sign_reg_elems =
    lanelet.regulatoryElementsAs<const lanelet::TrafficSign>();

  if (!traffic_sign_reg_elems.empty()) {
    for (const auto & ts : traffic_sign_reg_elems) {
      if (ts->type() != "stop_sign") {
        continue;
      }
      lanelet::ConstLineStrings3d traffic_sign_stoplines = ts->refLines();
      if (!traffic_sign_stoplines.empty()) {
        return traffic_sign_stoplines.front();
      }
    }
  }
  return std::nullopt;
}

}  // namespace autoware::experimental::lanelet2_utils
