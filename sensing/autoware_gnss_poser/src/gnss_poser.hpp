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

#include <autoware_internal_debug_msgs/msg/bool_stamped.hpp>
#include <autoware_map_msgs/msg/map_projector_info.hpp>
#include <autoware_sensing_msgs/msg/gnss_ins_orientation.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/transform.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>

#include <boost/circular_buffer.hpp>

#include <array>
#include <chrono>
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
  /// frame_id of the published pose and of the transform to broadcast.
  std::string map_frame = "map";
  /// child_frame_id of the transform to broadcast.
  std::string gnss_base_frame = "gnss_base_link";
};

// Covariance values the poser claims when its input carries none. gnss_poser has used them
// unchanged since 2020 and no rationale for any of them is recorded; the maintainer wrote in
// autoware.universe issue #800 that "the default values were set without any verification". They
// are named here so that they can be seen and discussed in one place rather than read out of the
// arithmetic, and so that a test can state which of them it exercises.
constexpr std::array<double, 3> default_unknown_position_variances = {10.0, 10.0, 10.0};  // [m^2]
constexpr std::array<double, 3> default_motion_rotation_variances = {0.1, 0.1, 1.0};      // [rad^2]
constexpr double default_ins_placeholder_rmse = 1.0;                                      // [rad]

// How long the antenna transform may be unavailable before it is an issue: right after startup
// /tf_static may arrive after the first fixes. Measured by the fix header stamps, so that it
// follows the data, e.g. a rosbag replayed at another rate.
constexpr std::chrono::seconds antenna_transform_grace_period{5};
// A step between the header stamps of consecutive fixes that goes backward, or forward by more than
// this, is a jump of the clock that stamps the fixes rather than time that passed (e.g. a receiver
// switching to GNSS time, or a rosbag replayed in a loop); the grace period starts over there.
// Receivers publish at 1 Hz or faster, so this allows for one missing fix at 1 Hz.
constexpr std::chrono::seconds fix_stamp_jump_threshold{2};

/// \brief The covariance values above, as GnssPoser takes them.
///
/// Not node parameters: GnssPoserNode constructs GnssPoser without them, so the values are the
/// ones above. They are injectable so that a test can show which value reaches the output, and so
/// that turning any of them into a parameter later needs no change here.
struct GnssPoserCovarianceDefaults
{
  /// Position variances used when the fix reports COVARIANCE_TYPE_UNKNOWN. [m^2]
  std::array<double, 3> unknown_position_variances = default_unknown_position_variances;
  /// Roll, pitch and yaw variances used while the orientation is derived from motion. [rad^2]
  std::array<double, 3> motion_rotation_variances = default_motion_rotation_variances;
  /// Stand-in rmse per axis until the first INS orientation arrives. It is squared into the
  /// rotation variances like a real one, so 1.0 rad becomes 1.0 rad^2. [rad]
  double ins_placeholder_rmse = default_ins_placeholder_rmse;
};

/// \brief Turns GNSS fixes into base_link poses in the map frame.
///
/// Feed the map projector info and, when configured, the INS orientation through the setters, then
/// call input_fix() for every NavSatFix. The result says whether the fix produced a pose or why it
/// did not. Whether an input has been missing for longer than expected, or cannot be used at all,
/// is a separate question, which input_fix() answers by calling the IssueRaisedCallback for each
/// such issue when it arises.
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

  /// \brief An input that should be usable by now but is not.
  struct Issue
  {
    enum class Cause {
      /// TransformLookup has given no transform for longer than antenna_transform_grace_period.
      /// The transform may still be published.
      TransformUnavailable,
    };

    Cause cause;
    /// The fix's header.frame_id that the transform was looked up for.
    std::string antenna_frame;
  };

  /// \brief Receives each issue when it is raised, from within the input that raised it. An issue
  /// that persists is not passed again; one that is resolved and then raised again is.
  using IssueRaisedCallback = std::function<void(const Issue & issue)>;

  /// \param lookup_antenna_to_base_link see TransformLookup; kept for the lifetime of the object.
  /// \param on_issue_raised see IssueRaisedCallback; may be empty when the caller does not need the
  /// issues.
  /// \param covariance_defaults see GnssPoserCovarianceDefaults; the node leaves it at its
  /// defaults.
  /// \throw std::invalid_argument when params.buff_epoch is smaller than 1.
  GnssPoser(
    const GnssPoserParams & params, TransformLookup lookup_antenna_to_base_link,
    IssueRaisedCallback on_issue_raised = {},
    const GnssPoserCovarianceDefaults & covariance_defaults = {});

  /// \brief What input_fix() did with the fix.
  enum class Outcome {
    NoProjectorInfo,  ///< no map projector info received yet; nothing was derived from the fix
    LocalProjector,   ///< the map uses a local projector, so a GNSS fix cannot be converted
    NotFixed,         ///< the receiver reports no fix; its status is known but no pose is computed
    Buffering,        ///< the position buffer is not full yet (average and median methods)
    Published,        ///< a pose was computed, see Result::gnss_pose
  };

  /// \brief What the fix produced, as the messages to publish.
  ///
  /// The messages are filled in completely, headers included, so that the caller only has to hand
  /// them to a publisher: which frame a pose is in and which stamp it carries is part of the pose
  /// computation, not of the plumbing. What is empty is not published, so the caller never has to
  /// know which outcome produces which output; `outcome` is there to be logged and reported.
  struct Result
  {
    Outcome outcome;
    /// The receiver's fix status. Empty for a fix that was rejected before the fixed check,
    /// which says nothing about the receiver.
    std::optional<autoware_internal_debug_msgs::msg::BoolStamped> gnss_fixed;
    /// The base_link pose in the map frame.
    std::optional<geometry_msgs::msg::PoseStamped> gnss_pose;
    /// The same pose with its 6x6 covariance (only the diagonal is filled).
    std::optional<geometry_msgs::msg::PoseWithCovarianceStamped> gnss_pose_cov;
    /// The same pose as a transform, map_frame -> gnss_base_frame, ready to broadcast.
    std::optional<geometry_msgs::msg::TransformStamped> transform;
  };

  void set_projector_info(const autoware_map_msgs::msg::MapProjectorInfo & projector_info);

  /// \brief Latch the INS orientation used while use_gnss_ins_orientation is true. Until the first
  /// call, an identity orientation with an rmse of 1.0 rad per axis stands in for it.
  void set_ins_orientation(const autoware_sensing_msgs::msg::GnssInsOrientation & orientation);

  /// \brief Process one fix: gate, project, buffer, orient, compose with the antenna transform and
  /// attach the covariance.
  /// How long an input has been missing is measured by the fix header stamps.
  Result input_fix(const sensor_msgs::msg::NavSatFix & fix);

private:
  // A fix header stamp as the time since the epoch of the clock that stamped it.
  using StampTime = std::chrono::nanoseconds;

  // input_fix() without the stamp continuity and the issue update.
  Result process_fix(const sensor_msgs::msg::NavSatFix & fix);

  // Stamp continuity. Only this function touches previous_stamp_.
  // Whether the fix stamped `stamp` continues the stamps of the previous fix, whether or not either
  // reaches the lookup: false for the first fix and where the stamps jump, which is no error.
  bool update_stamp_continuity(const builtin_interfaces::msg::Time & stamp);

  // Issue management. Only these functions touch transform_lookup_failure_.
  // The transform for `frame` could not be looked up for the fix stamped `stamp`, so the failure
  // now lasts until `stamp`. A failure that continues keeps its start, so that the grace period
  // runs from there.
  void record_lookup_failure(
    const std::string & frame, const builtin_interfaces::msg::Time & stamp);
  // Forget the failure: the transform could be looked up, or the stamps jumped, so that how long
  // the failure has lasted is not known. The next fix that cannot get the transform starts a new
  // failure, which is raised again once it lasts beyond the grace period.
  void clear_lookup_failure();
  // Raise transform_lookup_failure_ as an issue through on_issue_raised_ once it has lasted beyond
  // the grace period, from its start to its latest failure. As only a fix that reaches the lookup
  // extends the failure, only such a fix can make it an issue.
  void update_issues();

  GnssPoserParams params_;
  TransformLookup lookup_antenna_to_base_link_;
  IssueRaisedCallback on_issue_raised_;
  GnssPoserCovarianceDefaults covariance_defaults_;
  autoware_map_msgs::msg::MapProjectorInfo projector_info_;
  bool received_map_projector_info_ = false;
  boost::circular_buffer<geometry_msgs::msg::Point> position_buffer_;
  // Previous antenna position used to derive orientation from motion.
  geometry_msgs::msg::Point prev_position_;
  bool has_prev_position_ = false;
  autoware_sensing_msgs::msg::GnssInsOrientation ins_orientation_;
  // The antenna transform the latest fix to reach the lookup could not get: for which frame, from
  // which fix to which that has been so, and whether it has been raised as an issue, so that it is
  // raised once. Kept across fixes that do not reach the lookup, as they say nothing about it.
  struct TransformLookupFailure
  {
    std::string frame;
    StampTime since;
    StampTime last;
    bool raised = false;
  };
  std::optional<TransformLookupFailure> transform_lookup_failure_;
  // The header stamp of the previous fix, to tell a jump of the stamps from time that passed.
  std::optional<StampTime> previous_stamp_;
};

}  // namespace autoware::gnss_poser

#endif  // GNSS_POSER_HPP_
