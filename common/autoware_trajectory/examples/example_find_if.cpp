// Copyright 2026 Tier IV, Inc.
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

#include "autoware/trajectory/path_point_with_lane_id.hpp"
#include "autoware/trajectory/utils/find_if.hpp"

#include <autoware/pyplot/pyplot.hpp>
#include <autoware_utils_geometry/geometry.hpp>

#include <pybind11/embed.h>
#include <pybind11/stl.h>

#include <iostream>
#include <vector>

using autoware_internal_planning_msgs::msg::PathPointWithLaneId;
using Trajectory = autoware::experimental::trajectory::Trajectory<
  autoware_internal_planning_msgs::msg::PathPointWithLaneId>;

PathPointWithLaneId path_point_with_lane_id(double x, double y, uint8_t lane_id)
{
  PathPointWithLaneId point;
  point.point.pose.position.x = x;
  point.point.pose.position.y = y;
  point.lane_ids.emplace_back(lane_id);
  return point;
}

int main()
{
  try {
    pybind11::scoped_interpreter guard{};
    auto plt = autoware::pyplot::import();

    {
      const std::vector points{
        path_point_with_lane_id(0.41, 0.69, 0), path_point_with_lane_id(0.66, 1.09, 0),
        path_point_with_lane_id(0.93, 1.41, 0), path_point_with_lane_id(1.26, 1.71, 0),
        path_point_with_lane_id(1.62, 1.90, 0), path_point_with_lane_id(1.96, 1.98, 0),
        path_point_with_lane_id(2.48, 1.96, 1), path_point_with_lane_id(3.02, 1.87, 1),
        path_point_with_lane_id(3.56, 1.82, 1), path_point_with_lane_id(4.14, 2.02, 1),
        path_point_with_lane_id(4.56, 2.36, 1), path_point_with_lane_id(4.89, 2.72, 1),
        path_point_with_lane_id(5.27, 3.15, 1), path_point_with_lane_id(5.71, 3.69, 1),
        path_point_with_lane_id(6.09, 4.02, 0), path_point_with_lane_id(6.54, 4.16, 0),
        path_point_with_lane_id(6.79, 3.92, 0), path_point_with_lane_id(7.11, 3.60, 0),
        path_point_with_lane_id(7.42, 3.01, 0)};

      const auto trajectory = Trajectory::Builder{}.build(points);

      if (!trajectory) {
        return 1;
      }

      const auto constraint =
        [](const autoware_internal_planning_msgs::msg::PathPointWithLaneId & point) {
          return point.lane_ids[0] == 1;
        };

      const auto first_index =
        autoware::experimental::trajectory::find_first_index_if(*trajectory, constraint);

      const auto last_index =
        autoware::experimental::trajectory::find_last_index_if(*trajectory, constraint);

      if (!first_index || !last_index) {
        std::cerr << "Expected an index, but got none" << std::endl;
        return 1;
      }

      const auto first_point = trajectory->compute(*first_index).point.pose.position;
      const auto last_point = trajectory->compute(*last_index).point.pose.position;

      std::vector<double> x_all;
      std::vector<double> y_all;
      std::vector<double> x_id0;
      std::vector<double> y_id0;
      std::vector<double> x_id1;
      std::vector<double> y_id1;

      for (auto s = 0.0; s < trajectory->length(); s += 0.01) {
        const auto p = trajectory->compute(s);
        x_all.push_back(p.point.pose.position.x);
        y_all.push_back(p.point.pose.position.y);
      }

      for (const auto & point : points) {
        if (point.lane_ids[0] == 0) {
          x_id0.push_back(point.point.pose.position.x);
          y_id0.push_back(point.point.pose.position.y);
        } else {
          x_id1.push_back(point.point.pose.position.x);
          y_id1.push_back(point.point.pose.position.y);
        }
      }

      plt.plot(Args(x_all, y_all), Kwargs("color"_a = "blue"));
      plt.scatter(Args(x_id0, y_id0), Kwargs("color"_a = "blue", "label"_a = "lane_id = 0"));
      plt.scatter(Args(x_id1, y_id1), Kwargs("color"_a = "green", "label"_a = "lane_id = 1"));
      plt.scatter(
        Args(first_point.x, first_point.y), Kwargs(
                                              "color"_a = "red", "marker"_a = "<", "zorder"_a = 3,
                                              "label"_a = "first point where lane_id = 1"));
      plt.scatter(
        Args(last_point.x, last_point.y), Kwargs(
                                            "color"_a = "red", "marker"_a = ">", "zorder"_a = 3,
                                            "label"_a = "last point where lane_id = 1"));
      plt.grid();
      plt.legend(Args(), Kwargs("loc"_a = "upper left"));
      plt.show();
    }

    {
      const std::vector points{
        path_point_with_lane_id(-3.0, 0.0, 0), path_point_with_lane_id(-2.0, 0.0, 0),
        path_point_with_lane_id(-1.0, 0.0, 0), path_point_with_lane_id(0.0, 0.0, 0),
        path_point_with_lane_id(1.0, 0.0, 0),  path_point_with_lane_id(2.0, 0.0, 0),
        path_point_with_lane_id(3.0, 0.0, 0)};

      const auto trajectory = Trajectory::Builder{}.build(points);
      if (!trajectory) {
        return 1;
      }

      geometry_msgs::msg::Point base_point;
      base_point.x = 0.0;
      base_point.y = 1.0;

      const auto constraint =
        [&](const autoware_internal_planning_msgs::msg::PathPointWithLaneId & point) {
          return autoware_utils_geometry::calc_distance2d(point.point.pose.position, base_point) <
                 2.0;
        };

      const auto first_index =
        autoware::experimental::trajectory::find_first_index_if(*trajectory, constraint, 10);

      const auto last_index =
        autoware::experimental::trajectory::find_last_index_if(*trajectory, constraint, 10);

      if (!first_index || !last_index) {
        std::cerr << "Expected an index, but got none" << std::endl;
        return 1;
      }

      std::cout << "First index: " << *first_index << ", Last index: " << *last_index << std::endl;

      const auto first_point = trajectory->compute(*first_index).point.pose.position;
      const auto last_point = trajectory->compute(*last_index).point.pose.position;

      std::vector<double> x_original;
      std::vector<double> y_original;
      std::vector<double> x;
      std::vector<double> y;
      std::vector<double> x_circle;
      std::vector<double> y_circle;

      for (const auto & point : points) {
        x_original.push_back(point.point.pose.position.x);
        y_original.push_back(point.point.pose.position.y);
      }

      for (auto s = 0.0; s < trajectory->length(); s += 0.01) {
        const auto p = trajectory->compute(s);
        x.push_back(p.point.pose.position.x);
        y.push_back(p.point.pose.position.y);
      }

      for (auto theta = 0.0; theta <= 2.0 * M_PI; theta += 0.01) {
        x_circle.push_back(base_point.x + 2.0 * std::cos(theta));
        y_circle.push_back(base_point.y + 2.0 * std::sin(theta));
      }

      plt.scatter(
        Args(x_original, y_original), Kwargs("color"_a = "blue", "label"_a = "original points"));
      plt.plot(Args(x, y), Kwargs("color"_a = "blue"));
      plt.plot(Args(x_circle, y_circle), Kwargs("color"_a = "green", "label"_a = "distance < 2.0"));
      plt.scatter(
        Args(first_point.x, first_point.y),
        Kwargs(
          "color"_a = "red", "marker"_a = "<", "zorder"_a = 3,
          "label"_a = "first point where distance < 2.0 from center"));
      plt.scatter(
        Args(last_point.x, last_point.y),
        Kwargs(
          "color"_a = "red", "marker"_a = ">", "zorder"_a = 3,
          "label"_a = "last point where distance < 2.0 from center"));
      plt.axis(Args("equal"));
      plt.grid();
      plt.legend(Args(), Kwargs("loc"_a = "upper left"));
      plt.show();
    }
  } catch (const std::exception & e) {
    std::cerr << "Error: " << e.what() << std::endl;
    return 1;
  }

  return 0;
}
