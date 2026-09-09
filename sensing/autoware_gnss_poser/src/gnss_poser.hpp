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
#ifndef GNSS_POSER_HPP_
#define GNSS_POSER_HPP_

#include <builtin_interfaces/msg/time.hpp>

#include <autoware_map_msgs/msg/map_projector_info.hpp>
#include <autoware_sensing_msgs/msg/gnss_ins_orientation.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_with_covariance.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <geometry_msgs/msg/transform.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>

#include <boost/circular_buffer.hpp>

#include <array>
#include <functional>
#include <optional>
#include <string>
#include <utility>

// The rclcpp-free part of the GNSS poser: it turns GNSS fixes into base_link poses. Everything
// here takes and returns message types but needs no node, clock, TF listener or publisher.
namespace autoware::gnss_poser
{
// How the published position is derived from the incoming fixes: the latest one as-is, or the
// average / component-wise median of the last `buff_epoch` ones.
enum class GnssPosePubMethod { Instant = 0, Average = 1, Median = 2 };

/// \brief Configuration of GnssPoser, taken from the node parameters of the same names.
struct GnssPoserParams
{
  GnssPosePubMethod gnss_pose_pub_method = GnssPosePubMethod::Instant;
  int buff_epoch = 1;
  bool use_gnss_ins_orientation = true;
};

/// \brief Turns GNSS fixes into base_link poses in the map frame.
///
/// Feed the map projector info and, when configured, the INS orientation through the setters, then
/// call input_fix() for every NavSatFix. The result says whether the fix produced a pose or why it
/// did not; what to publish and log for each outcome is the caller's decision.
class GnssPoser
{
public:
  /// \brief Resolves the transform from the antenna frame named in the fix header to base_link at
  /// the fix header stamp, or std::nullopt when it cannot be resolved.
  ///
  /// input_fix() calls it only for a fix that passed the gates and the buffering, right before the
  /// pose is composed. Obtaining the transform (TF lookup, logging) is the caller's business; what
  /// to do when there is none is decided here.
  using TransformLookup = std::function<std::optional<geometry_msgs::msg::Transform>(
    const std::string & antenna_frame, const builtin_interfaces::msg::Time & stamp)>;

  /// \param lookup_antenna_to_base_link see TransformLookup; kept for the lifetime of the object.
  /// \throw std::invalid_argument when params.buff_epoch is smaller than 1.
  GnssPoser(const GnssPoserParams & params, TransformLookup lookup_antenna_to_base_link);

  /// \brief What input_fix() did with the fix.
  enum class Outcome {
    NoProjectorInfo,  ///< no map projector info received yet; nothing was derived from the fix
    LocalProjector,   ///< the map uses a local projector, so a GNSS fix cannot be converted
    NotFixed,         ///< the receiver reports no fix; its status is known but no pose is computed
    Buffering,        ///< the position buffer is not full yet (average and median methods)
    Published,        ///< a pose was computed, see Result::pose_with_covariance
  };

  struct Result
  {
    Outcome outcome;
    /// The base_link pose in the map frame with its 6x6 covariance (only the diagonal is filled).
    /// Set exactly when outcome is Outcome::Published.
    std::optional<geometry_msgs::msg::PoseWithCovariance> pose_with_covariance;
  };

  void set_projector_info(const autoware_map_msgs::msg::MapProjectorInfo & projector_info);

  /// \brief Latch the INS orientation used while use_gnss_ins_orientation is true. Until the first
  /// call, an identity orientation with an rmse of 1.0 rad per axis stands in for it.
  void set_ins_orientation(const autoware_sensing_msgs::msg::GnssInsOrientation & orientation);

  /// \brief Process one fix: gate, project, buffer, orient, compose with the antenna transform and
  /// attach the covariance.
  Result input_fix(const sensor_msgs::msg::NavSatFix & fix);

private:
  GnssPoserParams params_;
  TransformLookup lookup_antenna_to_base_link_;
  autoware_map_msgs::msg::MapProjectorInfo projector_info_;
  bool received_map_projector_info_ = false;
  boost::circular_buffer<geometry_msgs::msg::Point> position_buffer_;
  // Previous antenna position used to derive orientation from motion.
  geometry_msgs::msg::Point prev_position_;
  bool has_prev_position_ = false;
  autoware_sensing_msgs::msg::GnssInsOrientation ins_orientation_;
};

// Utility functions of a few specific steps, declared here only because the unit tests that came
// with the package call them directly.

/// \brief Component-wise median of the buffered positions. The buffer must be non-empty.
geometry_msgs::msg::Point get_median_position(
  const boost::circular_buffer<geometry_msgs::msg::Point> & position_buffer);

/// \brief Component-wise mean of the buffered positions. The buffer must be non-empty.
geometry_msgs::msg::Point get_average_position(
  const boost::circular_buffer<geometry_msgs::msg::Point> & position_buffer);

/// \brief Yaw-only orientation pointing from `prev_point` to `point` (atan2 of the xy
/// displacement); identity when the two points coincide.
geometry_msgs::msg::Quaternion get_quaternion_by_position_difference(
  const geometry_msgs::msg::Point & point, const geometry_msgs::msg::Point & prev_point);

}  // namespace autoware::gnss_poser

#endif  // GNSS_POSER_HPP_
