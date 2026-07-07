// Copyright 2024 Autoware Foundation
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

// Roadmap foundation (plan/ndt_in_rust_next.md → Phase 0/1): the opaque Rust node handle
// (`AwNdtScanMatcher`) + its RAII C++ owner, plus the param conversion that crosses the FFI once at
// construction. In legacy builds this header provides an empty owner so the core node layout can
// name the Rust handle unconditionally without linking to FFI symbols.

#ifndef AUTOWARE__NDT_SCAN_MATCHER__NDT_SCAN_MATCHER_RS_HPP_
#define AUTOWARE__NDT_SCAN_MATCHER__NDT_SCAN_MATCHER_RS_HPP_

#ifdef NDT_USE_RUST

#include "autoware/ndt_scan_matcher/hyper_parameters.hpp"

#include "autoware_ndt_scan_matcher_rs.h"

#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <stdexcept>

namespace autoware::ndt_scan_matcher
{

/// Build the FFI param struct from the node's `HyperParameters`. The covariance offset-model vectors
/// are borrowed by pointer for the duration of the `_new` call only (Rust copies them), so the
/// referenced `HyperParameters` must outlive that call. The fixed `min_points` / `eig_mult` /
/// `outlier_ratio` mirror the legacy C++ engine constants used by the Rust-owned engine.
inline AwNdtParams make_aw_ndt_params(const HyperParameters & p)
{
  AwNdtParams out{};
  out.resolution = static_cast<double>(p.ndt.resolution);
  out.min_points = 6;
  out.eig_mult = 0.01;
  out.trans_epsilon = p.ndt.trans_epsilon;
  out.step_size = p.ndt.step_size;
  out.max_iterations = p.ndt.max_iterations;
  out.outlier_ratio = 0.55;
  out.num_threads = p.ndt.num_threads;
  out.converged_param_type = static_cast<int32_t>(p.score_estimation.converged_param_type);
  out.converged_param_transform_probability =
    p.score_estimation.converged_param_transform_probability;
  out.converged_param_nearest_voxel_transformation_likelihood =
    p.score_estimation.converged_param_nearest_voxel_transformation_likelihood;
  out.covariance_estimation_type =
    static_cast<int32_t>(p.covariance.covariance_estimation.covariance_estimation_type);
  out.covariance_scale_factor = p.covariance.covariance_estimation.scale_factor;
  out.covariance_temperature = p.covariance.covariance_estimation.temperature;
  std::copy(
    p.covariance.output_pose_covariance.begin(), p.covariance.output_pose_covariance.end(),
    out.output_pose_covariance);
  out.initial_pose_offset_model_x =
    p.covariance.covariance_estimation.initial_pose_offset_model_x.data();
  out.initial_pose_offset_model_x_len =
    p.covariance.covariance_estimation.initial_pose_offset_model_x.size();
  out.initial_pose_offset_model_y =
    p.covariance.covariance_estimation.initial_pose_offset_model_y.data();
  out.initial_pose_offset_model_y_len =
    p.covariance.covariance_estimation.initial_pose_offset_model_y.size();
  // Regularization pose buffer: enabled flag + the SmartPoseBuffer tolerances the C++ node uses
  // (`value_as_unlimited = 1000.0` for both — validation effectively off).
  out.regularization_enable = p.ndt_regularization_enable;
  out.regularization_pose_timeout_sec = 1000.0;
  out.regularization_pose_distance_tolerance_m = 1000.0;
  // Initial pose buffer: the expected frame + the SmartPoseBuffer tolerances. `map_frame` is borrowed
  // for the `_new` call only (Rust copies it), so `p` must outlive the call.
  out.map_frame = reinterpret_cast<const std::uint8_t *>(p.frame.map_frame.data());
  out.map_frame_len = p.frame.map_frame.size();
  out.initial_pose_timeout_sec = p.validation.initial_pose_timeout_sec;
  out.initial_pose_distance_tolerance_m = p.validation.initial_pose_distance_tolerance_m;
  // Sensor-points prologue: base frame + delay/distance thresholds (borrowed `base_frame` bytes).
  out.base_frame = reinterpret_cast<const std::uint8_t *>(p.frame.base_frame.data());
  out.base_frame_len = p.frame.base_frame.size();
  out.sensor_points_timeout_sec = p.sensor_points.timeout_sec;
  out.sensor_points_required_distance = p.sensor_points.required_distance;
  return out;
}

/// Build the borrowed FFI view of a `PointCloud2` (stamp + frame + layout + data + xyz field offsets).
/// Valid only for the duration of the FFI call. The xyz datatype is taken from the `x` field.
inline AwPointCloud2View make_pointcloud2_view(const sensor_msgs::msg::PointCloud2 & msg)
{
  AwPointCloud2View v{};
  v.stamp_ns = static_cast<rclcpp::Time>(msg.header.stamp).nanoseconds();
  v.frame_id.ptr = reinterpret_cast<const std::uint8_t *>(msg.header.frame_id.data());
  v.frame_id.len = msg.header.frame_id.size();
  v.width = msg.width;
  v.height = msg.height;
  v.point_step = msg.point_step;
  v.row_step = msg.row_step;
  v.data = msg.data.data();
  v.data_len = msg.data.size();
  for (const auto & field : msg.fields) {
    if (field.name == "x") {
      v.x_offset = field.offset;
      v.datatype = field.datatype;
    } else if (field.name == "y") {
      v.y_offset = field.offset;
    } else if (field.name == "z") {
      v.z_offset = field.offset;
    }
  }
  v.is_bigendian = msg.is_bigendian;
  return v;
}

/// Build the borrowed FFI view of a `PoseWithCovarianceStamped` (stamp + pose + row-major 6x6 cov).
/// Valid only for the duration of the FFI call (Rust copies what it retains).
inline AwPoseWithCovarianceStampedView make_pose_with_cov_view(
  const geometry_msgs::msg::PoseWithCovarianceStamped & msg)
{
  AwPoseWithCovarianceStampedView v{};
  v.stamp_ns = static_cast<rclcpp::Time>(msg.header.stamp).nanoseconds();
  const auto & position = msg.pose.pose.position;
  const auto & orientation = msg.pose.pose.orientation;
  v.position[0] = position.x;
  v.position[1] = position.y;
  v.position[2] = position.z;
  v.orientation[0] = orientation.x;
  v.orientation[1] = orientation.y;
  v.orientation[2] = orientation.z;
  v.orientation[3] = orientation.w;
  std::copy(msg.pose.covariance.begin(), msg.pose.covariance.end(), v.covariance);
  v.frame_id = reinterpret_cast<const std::uint8_t *>(msg.header.frame_id.data());
  v.frame_id_len = msg.header.frame_id.size();
  return v;
}

/// RAII owner of the opaque Rust node handle (`AwNdtScanMatcher *`). Initializes via `_new`
/// (throwing on a null result), frees via `_free`. Non-copyable, non-movable — a single owner per
/// node.
class NDTScanMatcherRS
{
public:
  NDTScanMatcherRS() = default;
  explicit NDTScanMatcherRS(const AwNdtParams & params) { initialize(params); }

  ~NDTScanMatcherRS() { autoware_ndt_scan_matcher_rs_free(handle_); }

  NDTScanMatcherRS(const NDTScanMatcherRS &) = delete;
  NDTScanMatcherRS & operator=(const NDTScanMatcherRS &) = delete;
  NDTScanMatcherRS(NDTScanMatcherRS &&) = delete;
  NDTScanMatcherRS & operator=(NDTScanMatcherRS &&) = delete;

  void initialize(const AwNdtParams & params)
  {
    autoware_ndt_scan_matcher_rs_free(handle_);
    handle_ = autoware_ndt_scan_matcher_rs_new(&params);
    if (handle_ == nullptr) {
      throw std::runtime_error("failed to create Rust NdtScanMatcherRs");
    }
  }

  AwNdtScanMatcher * raw() { return handle_; }
  const AwNdtEngine * engine_raw() const { return autoware_ndt_scan_matcher_rs_engine(handle_); }

private:
  AwNdtScanMatcher * handle_{nullptr};
};

}  // namespace autoware::ndt_scan_matcher

#else  // NDT_USE_RUST

namespace autoware::ndt_scan_matcher
{

// Empty legacy-build owner. The Rust-selected translation units provide the real FFI-backed
// implementation above; OFF builds keep this as inert node layout state only.
class NDTScanMatcherRS
{
public:
  NDTScanMatcherRS() = default;
  ~NDTScanMatcherRS() = default;

  NDTScanMatcherRS(const NDTScanMatcherRS &) = delete;
  NDTScanMatcherRS & operator=(const NDTScanMatcherRS &) = delete;
  NDTScanMatcherRS(NDTScanMatcherRS &&) = delete;
  NDTScanMatcherRS & operator=(NDTScanMatcherRS &&) = delete;
};

}  // namespace autoware::ndt_scan_matcher

#endif  // NDT_USE_RUST

#endif  // AUTOWARE__NDT_SCAN_MATCHER__NDT_SCAN_MATCHER_RS_HPP_
