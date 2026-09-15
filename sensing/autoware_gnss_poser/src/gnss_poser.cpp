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

#include "gnss_poser.hpp"

#include <autoware/geography_utils/height.hpp>
#include <autoware/geography_utils/projection.hpp>
#include <tf2/LinearMath/Quaternion.hpp>
#include <tf2/LinearMath/Transform.hpp>

#include <geographic_msgs/msg/geo_point.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <sensor_msgs/msg/nav_sat_status.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <algorithm>
#include <cmath>
#include <numeric>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace autoware::gnss_poser
{
namespace
{
/// True when the receiver reports at least STATUS_FIX.
bool is_fixed(const sensor_msgs::msg::NavSatStatus & nav_sat_status_msg)
{
  return nav_sat_status_msg.status >= sensor_msgs::msg::NavSatStatus::STATUS_FIX;
}

/// True when the receiver reports a covariance type other than UNKNOWN.
bool can_get_covariance(const sensor_msgs::msg::NavSatFix & nav_sat_fix_msg)
{
  return nav_sat_fix_msg.position_covariance_type >
         sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_UNKNOWN;
}

geometry_msgs::msg::Point project_to_map(
  const sensor_msgs::msg::NavSatFix & fix,
  const autoware_map_msgs::msg::MapProjectorInfo & projector_info)
{
  geographic_msgs::msg::GeoPoint gps_point;
  gps_point.latitude = fix.latitude;
  gps_point.longitude = fix.longitude;
  gps_point.altitude = fix.altitude;
  geometry_msgs::msg::Point position =
    autoware::geography_utils::project_forward(gps_point, projector_info);
  position.z = autoware::geography_utils::convert_height(
    position.z, gps_point.latitude, gps_point.longitude,
    autoware_map_msgs::msg::MapProjectorInfo::WGS84, projector_info.vertical_datum);
  return position;
}

geometry_msgs::msg::Pose compose_base_link_pose(
  const geometry_msgs::msg::Pose & antenna_pose,
  const geometry_msgs::msg::Transform & antenna_to_base_link)
{
  tf2::Transform tf_map2gnss_antenna{};
  tf2::fromMsg(antenna_pose, tf_map2gnss_antenna);
  tf2::Transform tf_gnss_antenna2base_link{};
  tf2::fromMsg(antenna_to_base_link, tf_gnss_antenna2base_link);

  // transform pose from gnss_antenna(in map frame) to base_link(in map frame)
  const tf2::Transform tf_map2base_link = tf_map2gnss_antenna * tf_gnss_antenna2base_link;
  geometry_msgs::msg::Pose base_link_pose;
  tf2::toMsg(tf_map2base_link, base_link_pose);
  return base_link_pose;
}

std::array<double, 36> make_pose_covariance(
  const sensor_msgs::msg::NavSatFix & fix, const std::array<double, 3> & rotation_variances,
  const std::array<double, 3> & unknown_position_variances)
{
  std::array<double, 36> covariance{};
  constexpr std::size_t diagonal_stride = 7;
  covariance[diagonal_stride * 0] =
    can_get_covariance(fix) ? fix.position_covariance[0] : unknown_position_variances[0];
  covariance[diagonal_stride * 1] =
    can_get_covariance(fix) ? fix.position_covariance[4] : unknown_position_variances[1];
  covariance[diagonal_stride * 2] =
    can_get_covariance(fix) ? fix.position_covariance[8] : unknown_position_variances[2];
  covariance[diagonal_stride * 3] = rotation_variances[0];
  covariance[diagonal_stride * 4] = rotation_variances[1];
  covariance[diagonal_stride * 5] = rotation_variances[2];
  return covariance;
}
}  // namespace

GnssPoser::GnssPoser(
  const GnssPoserParams & params, TransformLookup lookup_antenna_to_base_link,
  const GnssPoserCovarianceDefaults & covariance_defaults)
: params_(params),
  lookup_antenna_to_base_link_(std::move(lookup_antenna_to_base_link)),
  covariance_defaults_(covariance_defaults)
{
  if (params.buff_epoch < 1) {
    throw std::invalid_argument(
      "buff_epoch must be at least 1, got " + std::to_string(params.buff_epoch));
  }
  position_buffer_.set_capacity(params.buff_epoch);

  // Stand-in until the first INS message arrives (not to publish zero value covariances).
  const auto placeholder_rmse = static_cast<float>(covariance_defaults_.ins_placeholder_rmse);
  ins_orientation_.rmse_rotation_x = placeholder_rmse;
  ins_orientation_.rmse_rotation_y = placeholder_rmse;
  ins_orientation_.rmse_rotation_z = placeholder_rmse;
}

void GnssPoser::set_projector_info(const autoware_map_msgs::msg::MapProjectorInfo & projector_info)
{
  projector_info_ = projector_info;
  received_map_projector_info_ = true;
}

void GnssPoser::set_ins_orientation(
  const autoware_sensing_msgs::msg::GnssInsOrientation & orientation)
{
  ins_orientation_ = orientation;
}

GnssPoser::Result GnssPoser::input_fix(const sensor_msgs::msg::NavSatFix & fix)
{
  // Return immediately if map_projector_info has not been received yet.
  if (!received_map_projector_info_) {
    return {Outcome::NoProjectorInfo, std::nullopt};
  }

  if (projector_info_.projector_type == autoware_map_msgs::msg::MapProjectorInfo::LOCAL) {
    return {Outcome::LocalProjector, std::nullopt};
  }

  // check fixed topic
  if (!is_fixed(fix.status)) {
    return {Outcome::NotFixed, std::nullopt};
  }

  // get position
  const geometry_msgs::msg::Point position = project_to_map(fix, projector_info_);

  geometry_msgs::msg::Pose gnss_antenna_pose{};

  // publish pose immediately
  if (params_.gnss_pose_pub_method == GnssPosePubMethod::Instant) {
    gnss_antenna_pose.position = position;
  } else {
    // fill position buffer
    position_buffer_.push_front(position);
    if (!position_buffer_.full()) {
      return {Outcome::Buffering, std::nullopt};
    }
    // publish average pose or median pose of position buffer
    gnss_antenna_pose.position = (params_.gnss_pose_pub_method == GnssPosePubMethod::Average)
                                   ? get_average_position(position_buffer_)
                                   : get_median_position(position_buffer_);
  }

  // calc gnss antenna orientation
  geometry_msgs::msg::Quaternion orientation;
  if (params_.use_gnss_ins_orientation) {
    orientation = ins_orientation_.orientation;
  } else {
    if (!has_prev_position_) {
      prev_position_ = gnss_antenna_pose.position;
      has_prev_position_ = true;
    }
    orientation = get_quaternion_by_position_difference(gnss_antenna_pose.position, prev_position_);
    prev_position_ = gnss_antenna_pose.position;
  }

  gnss_antenna_pose.orientation = orientation;

  // get TF from gnss_antenna to base_link. If it cannot be obtained, the antenna pose is published
  // as the base_link pose, i.e. the identity transform is used (a default-constructed Transform has
  // zero translation and rotation w = 1).
  const geometry_msgs::msg::Transform antenna_to_base_link =
    lookup_antenna_to_base_link_(fix.header.frame_id, fix.header.stamp)
      .value_or(geometry_msgs::msg::Transform{});

  std::array<double, 3> rotation_variances{};
  if (params_.use_gnss_ins_orientation) {
    rotation_variances[0] = std::pow(ins_orientation_.rmse_rotation_x, 2);
    rotation_variances[1] = std::pow(ins_orientation_.rmse_rotation_y, 2);
    rotation_variances[2] = std::pow(ins_orientation_.rmse_rotation_z, 2);
  } else {
    rotation_variances = covariance_defaults_.motion_rotation_variances;
  }

  geometry_msgs::msg::PoseWithCovariance gnss_base_pose_with_covariance;
  gnss_base_pose_with_covariance.pose =
    compose_base_link_pose(gnss_antenna_pose, antenna_to_base_link);
  gnss_base_pose_with_covariance.covariance =
    make_pose_covariance(fix, rotation_variances, covariance_defaults_.unknown_position_variances);
  return {Outcome::Published, gnss_base_pose_with_covariance};
}

geometry_msgs::msg::Point get_median_position(
  const boost::circular_buffer<geometry_msgs::msg::Point> & position_buffer)
{
  auto get_median = [](std::vector<double> array) {
    std::sort(std::begin(array), std::end(array));
    const size_t median_index = array.size() / 2;
    double median = (array.size() % 2)
                      ? (array.at(median_index))
                      : ((array.at(median_index) + array.at(median_index - 1)) / 2);
    return median;
  };

  std::vector<double> array_x;
  std::vector<double> array_y;
  std::vector<double> array_z;
  for (const auto & position : position_buffer) {
    array_x.push_back(position.x);
    array_y.push_back(position.y);
    array_z.push_back(position.z);
  }

  geometry_msgs::msg::Point median_point;
  median_point.x = get_median(array_x);
  median_point.y = get_median(array_y);
  median_point.z = get_median(array_z);
  return median_point;
}

geometry_msgs::msg::Point get_average_position(
  const boost::circular_buffer<geometry_msgs::msg::Point> & position_buffer)
{
  std::vector<double> array_x;
  std::vector<double> array_y;
  std::vector<double> array_z;
  for (const auto & position : position_buffer) {
    array_x.push_back(position.x);
    array_y.push_back(position.y);
    array_z.push_back(position.z);
  }

  geometry_msgs::msg::Point average_point;
  average_point.x =
    std::reduce(array_x.begin(), array_x.end()) / static_cast<double>(array_x.size());
  average_point.y =
    std::reduce(array_y.begin(), array_y.end()) / static_cast<double>(array_y.size());
  average_point.z =
    std::reduce(array_z.begin(), array_z.end()) / static_cast<double>(array_z.size());
  return average_point;
}

geometry_msgs::msg::Quaternion get_quaternion_by_position_difference(
  const geometry_msgs::msg::Point & point, const geometry_msgs::msg::Point & prev_point)
{
  const double yaw = std::atan2(point.y - prev_point.y, point.x - prev_point.x);
  tf2::Quaternion quaternion;
  quaternion.setRPY(0, 0, yaw);
  return tf2::toMsg(quaternion);
}

}  // namespace autoware::gnss_poser
