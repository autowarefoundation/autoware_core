// Copyright 2015-2019 Autoware Foundation
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

#include "ndt_scan_matcher_helper.hpp"

#include <autoware/localization_util/matrix_type.hpp>
#include <autoware/localization_util/tree_structured_parzen_estimator.hpp>
#include <autoware/localization_util/util_func.hpp>
#include <autoware/ndt_scan_matcher/ndt_omp/estimate_covariance.hpp>
#include <autoware/ndt_scan_matcher/ndt_scan_matcher_core.hpp>
#include <autoware/ndt_scan_matcher/particle.hpp>
#include <autoware/qos_utils/qos_compatibility.hpp>
#include <autoware_utils_geometry/geometry.hpp>
#include <autoware_utils_pcl/transforms.hpp>

#include <pcl_conversions/pcl_conversions.h>

#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <tuple>
#include <vector>

#ifdef ROS_DISTRO_GALACTIC
#include <tf2_eigen/tf2_eigen.h>
#else
#include <tf2_eigen/tf2_eigen.hpp>
#endif

#include <algorithm>
#include <cmath>
#include <functional>
#include <iomanip>
#include <thread>

namespace autoware::ndt_scan_matcher
{
using autoware::localization_util::exchange_color_crc;
using autoware::localization_util::matrix4f_to_pose;
using autoware::localization_util::pose_to_matrix4f;

using autoware::localization_util::SmartPoseBuffer;
using autoware::localization_util::TreeStructuredParzenEstimator;
using autoware_utils_diagnostics::DiagnosticsInterface;

#ifdef NDT_USE_RUST
// Phase N4c: engine-result + cloud marshaling lifted from NdtRustAdapter (deleted in N4e) so the
// sensor callback drives the engine handle (`ndt_ptr_->raw_handle()`) directly via the C ABI instead
// of the adapter's pclomp-shaped methods. These are the adapter's `to_flat` / `getResult` bodies.
static std::vector<float> cloud_to_flat(const pcl::PointCloud<pcl::PointXYZ> & cloud)
{
  std::vector<float> f;
  f.reserve(cloud.size() * 3);
  for (const auto & p : cloud) {
    f.push_back(p.x);
    f.push_back(p.y);
    f.push_back(p.z);
  }
  return f;
}

static pclomp::NdtResult ndt_result_from_engine(const AwNdtEngine * handle)
{
  pclomp::NdtResult r;
  std::array<float, 16> pose{};
  std::int32_t iter = 0;
  float tp = 0.0F;
  float nvl = 0.0F;
  std::array<double, 36> hess{};
  constexpr std::uint32_t kCap = 256;
  std::vector<float> ta(static_cast<size_t>(kCap) * 16);
  std::uint32_t count = 0;
  AwNdtAlignOutput out{};
  out.pose = pose.data();
  out.iteration_num = &iter;
  out.transform_probability = &tp;
  out.nearest_voxel_likelihood = &nvl;
  out.hessian = hess.data();
  out.transformation_array = ta.data();
  out.transforms_cap = kCap;
  out.transforms_count = &count;
  autoware_ndt_scan_matcher_rs_ndt_engine_get_result(handle, &out);

  for (int rr = 0; rr < 4; ++rr) {
    for (int cc = 0; cc < 4; ++cc) {
      r.pose(rr, cc) = pose[(rr * 4) + cc];
    }
  }
  r.iteration_num = iter;
  r.transform_probability = tp;
  r.nearest_voxel_transformation_likelihood = nvl;
  for (int rr = 0; rr < 6; ++rr) {
    for (int cc = 0; cc < 6; ++cc) {
      r.hessian(rr, cc) = hess[(rr * 6) + cc];
    }
  }
  const std::uint32_t n = std::min(count, kCap);
  r.transformation_array.resize(n);
  for (std::uint32_t k = 0; k < n; ++k) {
    Eigen::Matrix4f m;
    for (int rr = 0; rr < 4; ++rr) {
      for (int cc = 0; cc < 4; ++cc) {
        m(rr, cc) = ta[(static_cast<size_t>(k) * 16) + (rr * 4) + cc];
      }
    }
    r.transformation_array[k] = m;
  }

  std::vector<float> tps(kCap);
  std::vector<float> nvls(kCap);
  std::uint32_t scount = 0;
  autoware_ndt_scan_matcher_rs_ndt_engine_get_score_arrays(
    handle, tps.data(), nvls.data(), kCap, &scount);
  const std::uint32_t sn = std::min(scount, kCap);
  r.transform_probability_array.assign(tps.begin(), tps.begin() + sn);
  r.nearest_voxel_transformation_likelihood_array.assign(nvls.begin(), nvls.begin() + sn);
  return r;
}
#endif

autoware_internal_debug_msgs::msg::Float32Stamped make_float32_stamped(
  const builtin_interfaces::msg::Time & stamp, const float data)
{
  using T = autoware_internal_debug_msgs::msg::Float32Stamped;
  return autoware_internal_debug_msgs::build<T>().stamp(stamp).data(data);
}

autoware_internal_debug_msgs::msg::Int32Stamped make_int32_stamped(
  const builtin_interfaces::msg::Time & stamp, const int32_t data)
{
  using T = autoware_internal_debug_msgs::msg::Int32Stamped;
  return autoware_internal_debug_msgs::build<T>().stamp(stamp).data(data);
}

NDTScanMatcher::NDTScanMatcher(const rclcpp::NodeOptions & options)
: Node("ndt_scan_matcher", options),
  tf2_broadcaster_(*this),
  tf2_buffer_(this->get_clock()),
  tf2_listener_(tf2_buffer_),
#ifndef NDT_USE_RUST
  is_activated_(false),
#endif
  param_(this)
#ifdef NDT_USE_RUST
  ,
  rs_(make_aw_ndt_params(param_))
#endif
{
  timer_callback_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  rclcpp::CallbackGroup::SharedPtr initial_pose_callback_group =
    this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  rclcpp::CallbackGroup::SharedPtr sensor_callback_group =
    this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

  auto initial_pose_sub_opt = rclcpp::SubscriptionOptions();
  initial_pose_sub_opt.callback_group = initial_pose_callback_group;
  auto sensor_sub_opt = rclcpp::SubscriptionOptions();
  sensor_sub_opt.callback_group = sensor_callback_group;

  constexpr double map_update_dt = 1.0;
  constexpr auto period_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::duration<double>(map_update_dt));
  map_update_timer_ = rclcpp::create_timer(
    this, this->get_clock(), period_ns, std::bind(&NDTScanMatcher::callback_timer, this),
    timer_callback_group_);
  initial_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
    "ekf_pose_with_covariance", 10,
    std::bind(&NDTScanMatcher::callback_initial_pose, this, std::placeholders::_1),
    initial_pose_sub_opt);
  sensor_points_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
    "points_raw", rclcpp::SensorDataQoS().keep_last(1),
    std::bind(&NDTScanMatcher::callback_sensor_points, this, std::placeholders::_1),
    sensor_sub_opt);

  // Only if regularization is enabled, subscribe to the regularization base pose
  if (param_.ndt_regularization_enable) {
    // NOTE: The reason that the regularization subscriber does not belong to the
    // sensor_callback_group is to ensure that the regularization callback is called even if
    // sensor_callback takes long time to process.
    // Both callback_initial_pose and callback_regularization_pose must not miss receiving data for
    // proper interpolation.
    regularization_pose_sub_ =
      this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
        "regularization_pose_with_covariance", 10,
        std::bind(&NDTScanMatcher::callback_regularization_pose, this, std::placeholders::_1),
        initial_pose_sub_opt);
#ifndef NDT_USE_RUST
    const double value_as_unlimited = 1000.0;
    regularization_pose_buffer_ =
      std::make_unique<SmartPoseBuffer>(this->get_logger(), value_as_unlimited, value_as_unlimited);
#endif

    diagnostics_regularization_pose_ =
      std::make_unique<DiagnosticsInterface>(this, "regularization_pose_subscriber_status");
  }

  sensor_aligned_pose_pub_ =
    this->create_publisher<sensor_msgs::msg::PointCloud2>("points_aligned", 10);
  no_ground_points_aligned_pose_pub_ =
    this->create_publisher<sensor_msgs::msg::PointCloud2>("points_aligned_no_ground", 10);
  ndt_pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("ndt_pose", 10);
  ndt_pose_with_covariance_pub_ =
    this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
      "ndt_pose_with_covariance", 10);
  initial_pose_with_covariance_pub_ =
    this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
      "initial_pose_with_covariance", 10);
  multi_ndt_pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseArray>("multi_ndt_pose", 10);
  multi_initial_pose_pub_ =
    this->create_publisher<geometry_msgs::msg::PoseArray>("multi_initial_pose", 10);
  exe_time_pub_ =
    this->create_publisher<autoware_internal_debug_msgs::msg::Float32Stamped>("exe_time_ms", 10);
  transform_probability_pub_ =
    this->create_publisher<autoware_internal_debug_msgs::msg::Float32Stamped>(
      "transform_probability", 10);
  nearest_voxel_transformation_likelihood_pub_ =
    this->create_publisher<autoware_internal_debug_msgs::msg::Float32Stamped>(
      "nearest_voxel_transformation_likelihood", 10);
  voxel_score_points_pub_ =
    this->create_publisher<sensor_msgs::msg::PointCloud2>("voxel_score_points", 10);
  no_ground_transform_probability_pub_ =
    this->create_publisher<autoware_internal_debug_msgs::msg::Float32Stamped>(
      "no_ground_transform_probability", 10);
  no_ground_nearest_voxel_transformation_likelihood_pub_ =
    this->create_publisher<autoware_internal_debug_msgs::msg::Float32Stamped>(
      "no_ground_nearest_voxel_transformation_likelihood", 10);
  iteration_num_pub_ =
    this->create_publisher<autoware_internal_debug_msgs::msg::Int32Stamped>("iteration_num", 10);
  initial_to_result_relative_pose_pub_ =
    this->create_publisher<geometry_msgs::msg::PoseStamped>("initial_to_result_relative_pose", 10);
  initial_to_result_distance_pub_ =
    this->create_publisher<autoware_internal_debug_msgs::msg::Float32Stamped>(
      "initial_to_result_distance", 10);
  initial_to_result_distance_old_pub_ =
    this->create_publisher<autoware_internal_debug_msgs::msg::Float32Stamped>(
      "initial_to_result_distance_old", 10);
  initial_to_result_distance_new_pub_ =
    this->create_publisher<autoware_internal_debug_msgs::msg::Float32Stamped>(
      "initial_to_result_distance_new", 10);
  ndt_marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("ndt_marker", 10);
  ndt_monte_carlo_initial_pose_marker_pub_ =
    this->create_publisher<visualization_msgs::msg::MarkerArray>(
      "monte_carlo_initial_pose_marker", 10);

  service_ =
    this->create_service<autoware_internal_localization_msgs::srv::PoseWithCovarianceStamped>(
      "ndt_align_srv",
      std::bind(
        &NDTScanMatcher::service_ndt_align, this, std::placeholders::_1, std::placeholders::_2),
      AUTOWARE_DEFAULT_SERVICES_QOS_PROFILE(), sensor_callback_group);
  service_trigger_node_ = this->create_service<std_srvs::srv::SetBool>(
    "trigger_node_srv",
    std::bind(
      &NDTScanMatcher::service_trigger_node, this, std::placeholders::_1, std::placeholders::_2),
    AUTOWARE_DEFAULT_SERVICES_QOS_PROFILE(), sensor_callback_group);

  ndt_ptr_.with([&](const auto & ndt_ptr) { ndt_ptr->setParams(param_.ndt); });

#ifndef NDT_USE_RUST
  // Under NDT_USE_RUST the initial-pose buffer lives on the Rust handle (constructed from params in
  // `rs_`); only the non-Rust path builds the C++ buffer here.
  initial_pose_buffer_ = std::make_unique<SmartPoseBuffer>(
    this->get_logger(), param_.validation.initial_pose_timeout_sec,
    param_.validation.initial_pose_distance_tolerance_m);
#endif

  map_update_module_ = std::make_unique<MapUpdateModule>(
    this, ndt_ptr_, param_.dynamic_map_loading
#ifdef NDT_USE_RUST
    ,
    rs_.raw()
#endif
  );

  diagnostics_scan_points_ = std::make_unique<DiagnosticsInterface>(this, "scan_matching_status");
  diagnostics_initial_pose_ =
    std::make_unique<DiagnosticsInterface>(this, "initial_pose_subscriber_status");
  diagnostics_map_update_ = std::make_unique<DiagnosticsInterface>(this, "map_update_status");
  diagnostics_ndt_align_ = std::make_unique<DiagnosticsInterface>(this, "ndt_align_service_status");
  diagnostics_trigger_node_ =
    std::make_unique<DiagnosticsInterface>(this, "trigger_node_service_status");

  logger_configure_ = std::make_unique<autoware_utils_logging::LoggerLevelConfigure>(this);
}

#ifdef NDT_USE_RUST
// Forward declaration: the pose callbacks below build an AwDiagnostics via this helper, whose
// definition (with the diag_* trampolines) lives next to make_host further down the file.
namespace
{
AwDiagnostics make_diagnostics(DiagnosticsInterface * d);
}  // namespace
#endif

void NDTScanMatcher::callback_timer()
{
  const rclcpp::Time ros_time_now = this->now();

  diagnostics_map_update_->clear();

  diagnostics_map_update_->add_key_value("timer_callback_time_stamp", ros_time_now.nanoseconds());

#ifdef NDT_USE_RUST
  // Activation + latest-EKF-position are Rust-owned (Phase 1 slice B); read them over the FFI.
  const bool node_is_activated = autoware_ndt_scan_matcher_rs_is_activated(rs_.raw());
  std::optional<geometry_msgs::msg::Point> latest_ekf_position;
  std::array<double, 3> ekf_xyz{};
  if (autoware_ndt_scan_matcher_rs_latest_ekf_position(rs_.raw(), ekf_xyz.data())) {
    geometry_msgs::msg::Point point;
    point.x = ekf_xyz[0];
    point.y = ekf_xyz[1];
    point.z = ekf_xyz[2];
    latest_ekf_position = point;
  }
#else
  const bool node_is_activated = is_activated_;
  const auto latest_ekf_position = latest_ekf_position_.with([](const auto & pos) { return pos; });
#endif
  map_update_module_->callback_timer(node_is_activated, latest_ekf_position, diagnostics_map_update_);

  diagnostics_map_update_->publish(ros_time_now);
}

void NDTScanMatcher::callback_initial_pose(
  const geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr initial_pose_msg_ptr)
{
#ifdef NDT_USE_RUST
  // Callback-level: the whole body — diagnostics + activation/frame gates + buffer push +
  // latest-EKF-position — runs in Rust, driving the Rust-owned state on the handle (Phase 1 slice B).
  // The C++ shell only builds the diagnostics vtable + the pose view.
  const AwDiagnostics diag = make_diagnostics(diagnostics_initial_pose_.get());
  const AwPoseWithCovarianceStampedView view = make_pose_with_cov_view(*initial_pose_msg_ptr);
  autoware_ndt_scan_matcher_rs_node_on_initial_pose(rs_.raw(), &diag, &view);
#else
  diagnostics_initial_pose_->clear();

  callback_initial_pose_main(initial_pose_msg_ptr);

  diagnostics_initial_pose_->publish(initial_pose_msg_ptr->header.stamp);
#endif
}

#ifndef NDT_USE_RUST
// The C++ baseline body (OFF only). Under NDT_USE_RUST the whole callback runs in Rust (see the
// forwarder above), so this is not compiled.
void NDTScanMatcher::callback_initial_pose_main(
  const geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr initial_pose_msg_ptr)
{
  diagnostics_initial_pose_->add_key_value(
    "topic_time_stamp",
    static_cast<rclcpp::Time>(initial_pose_msg_ptr->header.stamp).nanoseconds());

  // check is_activated
  diagnostics_initial_pose_->add_key_value("is_activated", static_cast<bool>(is_activated_));
  if (!is_activated_) {
    std::stringstream message;
    message << "Node is not activated.";
    diagnostics_initial_pose_->update_level_and_message(
      diagnostic_msgs::msg::DiagnosticStatus::WARN, message.str());
    return;
  }

  // check is_expected_frame_id
  const bool is_expected_frame_id =
    (initial_pose_msg_ptr->header.frame_id == param_.frame.map_frame);
  diagnostics_initial_pose_->add_key_value("is_expected_frame_id", is_expected_frame_id);
  if (!is_expected_frame_id) {
    std::stringstream message;
    message << "Received initial pose message with frame_id "
            << initial_pose_msg_ptr->header.frame_id << ", but expected " << param_.frame.map_frame
            << ". Please check the frame_id in the input topic and ensure it is correct.";
    diagnostics_initial_pose_->update_level_and_message(
      diagnostic_msgs::msg::DiagnosticStatus::ERROR, message.str());
    return;
  }

  initial_pose_buffer_->push_back(initial_pose_msg_ptr);

  latest_ekf_position_.with([&](auto & pos) { pos = initial_pose_msg_ptr->pose.pose.position; });
}
#endif

void NDTScanMatcher::callback_regularization_pose(
  geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr pose_conv_msg_ptr)
{
#ifdef NDT_USE_RUST
  // Callback-level: the whole body (diagnostics + buffer push) runs in Rust. The pose now crosses as
  // a value view and is pushed into the Rust-owned regularization buffer on the handle (Phase 1
  // slice A) — no host vtable needed here.
  const AwDiagnostics diag = make_diagnostics(diagnostics_regularization_pose_.get());
  const AwPoseWithCovarianceStampedView view = make_pose_with_cov_view(*pose_conv_msg_ptr);
  autoware_ndt_scan_matcher_rs_node_on_regularization_pose(rs_.raw(), &diag, &view);
#else
  diagnostics_regularization_pose_->clear();

  diagnostics_regularization_pose_->add_key_value(
    "topic_time_stamp", static_cast<rclcpp::Time>(pose_conv_msg_ptr->header.stamp).nanoseconds());

  regularization_pose_buffer_->push_back(pose_conv_msg_ptr);

  diagnostics_regularization_pose_->publish(pose_conv_msg_ptr->header.stamp);
#endif
}

void NDTScanMatcher::callback_sensor_points(
  sensor_msgs::msg::PointCloud2::ConstSharedPtr sensor_points_msg_in_sensor_frame)
{
  // clear diagnostics
  diagnostics_scan_points_->clear();

  // scan matching
  const bool is_succeed_scan_matching =
    callback_sensor_points_main(sensor_points_msg_in_sensor_frame);

  // check skipping_publish_num
#ifdef NDT_USE_RUST
  const bool node_is_activated = autoware_ndt_scan_matcher_rs_is_activated(rs_.raw());
#else
  const bool node_is_activated = is_activated_;
#endif
  static int64_t skipping_publish_num = 0;
  skipping_publish_num =
    ((is_succeed_scan_matching || !node_is_activated) ? 0 : (skipping_publish_num + 1));
  diagnostics_scan_points_->add_key_value("skipping_publish_num", skipping_publish_num);
  if (skipping_publish_num >= param_.validation.skipping_publish_num) {
    std::stringstream message;
    message << "skipping_publish_num exceed limit (" << skipping_publish_num << " times).";
    diagnostics_scan_points_->update_level_and_message(
      diagnostic_msgs::msg::DiagnosticStatus::WARN, message.str());
  }

  diagnostics_scan_points_->publish(sensor_points_msg_in_sensor_frame->header.stamp);
}

bool NDTScanMatcher::callback_sensor_points_main(
  sensor_msgs::msg::PointCloud2::ConstSharedPtr sensor_points_msg_in_sensor_frame)
{
  const auto exe_start_time = std::chrono::system_clock::now();

  const rclcpp::Time sensor_ros_time = sensor_points_msg_in_sensor_frame->header.stamp;

#ifdef NDT_USE_RUST
  return ndt_ptr_.with([&](const auto & ndt_ptr) {
    const AwHost host = make_host();
    const AwDiagnostics diag = make_diagnostics(diagnostics_scan_points_.get());
    const AwPointCloud2View view = make_pointcloud2_view(*sensor_points_msg_in_sensor_frame);
    const AwSensorPointsMatchParams match_params{
      param_.dynamic_map_loading.lidar_radius,
      param_.dynamic_map_loading.map_radius,
      param_.validation.initial_to_result_distance_tolerance_m,
      param_.ndt.regularization_scale_factor,
      param_.score_estimation.no_ground_points.enable,
      param_.score_estimation.no_ground_points.z_margin_for_ground_removal};

    bool is_converged = false;
    const int32_t status = autoware_ndt_scan_matcher_rs_node_on_sensor_points(
      rs_.raw(), ndt_ptr->raw_handle(), &host, &diag, &match_params, &view, &is_converged);
    if (status != 0) {
      return false;
    }

    const auto exe_end_time = std::chrono::system_clock::now();
    const auto duration_micro_sec =
      std::chrono::duration_cast<std::chrono::microseconds>(exe_end_time - exe_start_time).count();
    const auto exe_time = static_cast<float>(duration_micro_sec) / 1000.0f;
    diagnostics_scan_points_->add_key_value("execution_time", exe_time);
    if (exe_time > param_.validation.critical_upper_bound_exe_time_ms) {
      std::stringstream message;
      message << "NDT exe time is too long (took " << exe_time << " [ms]).";
      diagnostics_scan_points_->update_level_and_message(
        diagnostic_msgs::msg::DiagnosticStatus::WARN, message.str());
    }
    exe_time_pub_->publish(make_float32_stamped(sensor_ros_time, exe_time));
    return is_converged;
  });
#else

  pcl::shared_ptr<pcl::PointCloud<PointSource>> sensor_points_in_baselink_frame(
    new pcl::PointCloud<PointSource>);

  // check topic_time_stamp
  diagnostics_scan_points_->add_key_value("topic_time_stamp", sensor_ros_time.nanoseconds());

  // check sensor_points_size
  const size_t sensor_points_size = sensor_points_msg_in_sensor_frame->width;
  diagnostics_scan_points_->add_key_value("sensor_points_size", sensor_points_size);
  if (sensor_points_size == 0) {
    std::stringstream message;
    message << "Sensor points is empty.";
    diagnostics_scan_points_->update_level_and_message(
      diagnostic_msgs::msg::DiagnosticStatus::WARN, message.str());
    return false;
  }

  // check sensor_points_delay_time_sec
  const double sensor_points_delay_time_sec =
    (this->now() - sensor_points_msg_in_sensor_frame->header.stamp).seconds();
  diagnostics_scan_points_->add_key_value(
    "sensor_points_delay_time_sec", sensor_points_delay_time_sec);
  if (sensor_points_delay_time_sec > param_.sensor_points.timeout_sec) {
    std::stringstream message;
    message << "sensor points is experiencing latency."
            << "The delay time is " << sensor_points_delay_time_sec << "[sec] "
            << "(the tolerance is " << param_.sensor_points.timeout_sec << "[sec]).";
    diagnostics_scan_points_->update_level_and_message(
      diagnostic_msgs::msg::DiagnosticStatus::WARN, message.str());

    // If the delay time of the LiDAR topic exceeds the delay compensation time of ekf_localizer,
    // even if further processing continues, the estimated result will be rejected by ekf_localizer.
    // Therefore, it would be acceptable to exit the function here.
    // However, for now, we will continue the processing as it is.

    // return false;
  }

  // preprocess input pointcloud
  pcl::shared_ptr<pcl::PointCloud<PointSource>> sensor_points_in_sensor_frame(
    new pcl::PointCloud<PointSource>);
  const std::string & sensor_frame = sensor_points_msg_in_sensor_frame->header.frame_id;

  pcl::fromROSMsg(*sensor_points_msg_in_sensor_frame, *sensor_points_in_sensor_frame);

  // transform sensor points from sensor-frame to base_link
  try {
    transform_sensor_measurement(
      sensor_frame, param_.frame.base_frame, sensor_points_in_sensor_frame,
      sensor_points_in_baselink_frame);
  } catch (const std::exception & ex) {
    std::stringstream message;
    message << ex.what() << ". Please publish TF " << sensor_frame << " to "
            << param_.frame.base_frame;
    diagnostics_scan_points_->update_level_and_message(
      diagnostic_msgs::msg::DiagnosticStatus::ERROR, message.str());
    RCLCPP_ERROR_STREAM_THROTTLE(this->get_logger(), *this->get_clock(), 1000, message.str());
    diagnostics_scan_points_->add_key_value("is_succeed_transform_sensor_points", false);
    return false;
  }
  diagnostics_scan_points_->add_key_value("is_succeed_transform_sensor_points", true);

  // check sensor_points_max_distance
  double max_distance = 0.0;
  for (const auto & point : sensor_points_in_baselink_frame->points) {
    const double distance = std::hypot(point.x, point.y, point.z);
    max_distance = std::max(max_distance, distance);
  }

  diagnostics_scan_points_->add_key_value("sensor_points_max_distance", max_distance);
  if (max_distance < param_.sensor_points.required_distance) {
    std::stringstream message;
    message << "Max distance of sensor points = " << std::fixed << std::setprecision(3)
            << max_distance << " [m] < " << param_.sensor_points.required_distance << " [m]";
    diagnostics_scan_points_->update_level_and_message(
      diagnostic_msgs::msg::DiagnosticStatus::WARN, message.str());
    return false;
  }

  return ndt_ptr_.with([&](const auto & ndt_ptr) {
    // store sensor points for ndt alignment
    sensor_points_in_baselink_frame_ = sensor_points_in_baselink_frame;

    // The still-C++ cloud publishers read the aligned pose; the return feeds `skipping_publish_num`.
    pclomp::NdtResult ndt_result;
    bool is_converged = false;
    SmartPoseBuffer::InterpolateResult interpolation_result;
    geometry_msgs::msg::Pose result_pose_msg;
    std::vector<geometry_msgs::msg::Pose> transformation_msg_array;
    std::array<double, 36> ndt_covariance{};

    // check is_activated
    const bool node_is_activated = is_activated_;
    diagnostics_scan_points_->add_key_value("is_activated", node_is_activated);
    if (!node_is_activated) {
      std::stringstream message;
      message << "Node is not activated.";
      diagnostics_scan_points_->update_level_and_message(
        diagnostic_msgs::msg::DiagnosticStatus::WARN, message.str());
      return false;
    }

    // calculate initial pose
    std::optional<SmartPoseBuffer::InterpolateResult> interpolation_result_opt =
      initial_pose_buffer_->interpolate(sensor_ros_time);

    // check is_succeed_interpolate_initial_pose
    const bool is_succeed_interpolate_initial_pose = (interpolation_result_opt != std::nullopt);
    diagnostics_scan_points_->add_key_value(
      "is_succeed_interpolate_initial_pose", is_succeed_interpolate_initial_pose);
    if (!is_succeed_interpolate_initial_pose) {
      std::stringstream message;
      message << "Couldn't interpolate pose. Please verify that "
                 "(1) the initial pose topic (primarily come from the EKF) is being published, and "
                 "(2) the timestamps of the sensor PCD messages and pose messages are synchronized "
                 "correctly.";
      diagnostics_scan_points_->update_level_and_message(
        diagnostic_msgs::msg::DiagnosticStatus::WARN, message.str());
      return false;
    }

    initial_pose_buffer_->pop_old(sensor_ros_time);
    interpolation_result = interpolation_result_opt.value();

    // if regularization is enabled and available, set pose to NDT for regularization
    if (param_.ndt_regularization_enable) {
      add_regularization_pose(sensor_ros_time, *ndt_ptr);
    }

    // Warn if the lidar has gone out of the map range
    if (map_update_module_->out_of_map_range(
          interpolation_result.interpolated_pose.pose.pose.position)) {
      std::stringstream msg;

      msg << "Lidar has gone out of the map range";
      diagnostics_scan_points_->update_level_and_message(
        diagnostic_msgs::msg::DiagnosticStatus::WARN, msg.str());

      RCLCPP_WARN_STREAM_THROTTLE(this->get_logger(), *this->get_clock(), 1000, msg.str());
    }

    // check is_set_map_points
    const bool is_set_map_points = ndt_ptr->hasTarget();
    diagnostics_scan_points_->add_key_value("is_set_map_points", is_set_map_points);
    if (!is_set_map_points) {
      std::stringstream message;
      message << "Map points is not set.";
      diagnostics_scan_points_->update_level_and_message(
        diagnostic_msgs::msg::DiagnosticStatus::WARN, message.str());
      return false;
    }

    // perform ndt scan matching
    const Eigen::Matrix4f initial_pose_matrix =
      pose_to_matrix4f(interpolation_result.interpolated_pose.pose.pose);
    auto output_cloud = std::make_shared<pcl::PointCloud<PointSource>>();
    ndt_ptr->align(*output_cloud, initial_pose_matrix, sensor_points_in_baselink_frame_);

    ndt_result = ndt_ptr->getResult();

    result_pose_msg = matrix4f_to_pose(ndt_result.pose);
    for (const auto & pose_matrix : ndt_result.transformation_array) {
      geometry_msgs::msg::Pose pose_ros = matrix4f_to_pose(pose_matrix);
      transformation_msg_array.push_back(pose_ros);
    }

    // --- convergence decision ---
    constexpr int oscillation_num_threshold = 10;
    const int max_iterations = ndt_ptr->getMaximumIterations();
    const int oscillation_num = count_oscillation(transformation_msg_array);
    const bool is_ok_iteration_num = (ndt_result.iteration_num < max_iterations);
    const bool is_local_optimal_solution_oscillation =
      (oscillation_num > oscillation_num_threshold);
    bool valid_param_type = false;
    bool is_ok_score = false;
    double score = 0.0;
    double score_threshold = 0.0;
    if (param_.score_estimation.converged_param_type == ConvergedParamType::TRANSFORM_PROBABILITY) {
      valid_param_type = true;
      score = ndt_result.transform_probability;
      score_threshold = param_.score_estimation.converged_param_transform_probability;
    } else if (
      param_.score_estimation.converged_param_type ==
      ConvergedParamType::NEAREST_VOXEL_TRANSFORMATION_LIKELIHOOD) {
      valid_param_type = true;
      score = ndt_result.nearest_voxel_transformation_likelihood;
      score_threshold =
        param_.score_estimation.converged_param_nearest_voxel_transformation_likelihood;
    }
    if (valid_param_type) {
      is_ok_score = (score > score_threshold);
      is_converged = (is_ok_iteration_num || is_local_optimal_solution_oscillation) && is_ok_score;
    }

    // check iteration_num
    diagnostics_scan_points_->add_key_value("iteration_num", ndt_result.iteration_num);
    if (!is_ok_iteration_num) {
      std::stringstream message;
      message << "The number of iterations has reached its upper limit. The number of iterations: "
              << ndt_result.iteration_num << ", Limit: " << max_iterations << ".";
      diagnostics_scan_points_->update_level_and_message(
        diagnostic_msgs::msg::DiagnosticStatus::WARN, message.str());
    }

    // check local_optimal_solution_oscillation_num
    diagnostics_scan_points_->add_key_value(
      "local_optimal_solution_oscillation_num", oscillation_num);
    if (is_local_optimal_solution_oscillation) {
      std::stringstream message;
      message << "There is a possibility of oscillation in a local minimum";
      diagnostics_scan_points_->update_level_and_message(
        diagnostic_msgs::msg::DiagnosticStatus::WARN, message.str());
    }

    // check score
    diagnostics_scan_points_->add_key_value(
      "transform_probability", ndt_result.transform_probability);
    diagnostics_scan_points_->add_key_value(
      "nearest_voxel_transformation_likelihood",
      ndt_result.nearest_voxel_transformation_likelihood);
    if (!valid_param_type) {
      std::stringstream message;
      message
        << "Unknown converged param type. Please check `score_estimation.converged_param_type`";
      diagnostics_scan_points_->update_level_and_message(
        diagnostic_msgs::msg::DiagnosticStatus::ERROR, message.str());
      return false;
    }

    // check score diff
    const std::vector<float> & tp_array = ndt_result.transform_probability_array;
    if (static_cast<int>(tp_array.size()) != ndt_result.iteration_num + 1) {
      // only publish warning to /diagnostics, not skip publishing pose
      std::stringstream message;
      message << "transform_probability_array size is not equal to iteration_num + 1."
              << " transform_probability_array size: " << tp_array.size()
              << ", iteration_num: " << ndt_result.iteration_num;
      diagnostics_scan_points_->update_level_and_message(
        diagnostic_msgs::msg::DiagnosticStatus::WARN, message.str());
    } else {
      const float diff = tp_array.back() - tp_array.front();
      diagnostics_scan_points_->add_key_value("transform_probability_diff", diff);
      diagnostics_scan_points_->add_key_value("transform_probability_before", tp_array.front());
    }
    const std::vector<float> & nvtl_array =
      ndt_result.nearest_voxel_transformation_likelihood_array;
    if (static_cast<int>(nvtl_array.size()) != ndt_result.iteration_num + 1) {
      // only publish warning to /diagnostics, not skip publishing pose
      std::stringstream message;
      message
        << "nearest_voxel_transformation_likelihood_array size is not equal to iteration_num + 1."
        << " nearest_voxel_transformation_likelihood_array size: " << nvtl_array.size()
        << ", iteration_num: " << ndt_result.iteration_num;
      diagnostics_scan_points_->update_level_and_message(
        diagnostic_msgs::msg::DiagnosticStatus::WARN, message.str());
    } else {
      const float diff = nvtl_array.back() - nvtl_array.front();
      diagnostics_scan_points_->add_key_value("nearest_voxel_transformation_likelihood_diff", diff);
      diagnostics_scan_points_->add_key_value(
        "nearest_voxel_transformation_likelihood_before", nvtl_array.front());
    }

    if (!is_ok_score) {
      std::stringstream message;
      message << "Score is below the threshold. Score: " << score
              << ", Threshold: " << score_threshold;
      diagnostics_scan_points_->update_level_and_message(
        diagnostic_msgs::msg::DiagnosticStatus::WARN, message.str());
      RCLCPP_WARN_STREAM_THROTTLE(this->get_logger(), *this->get_clock(), 1000, message.str());
    }

    // covariance estimation
    const Eigen::Quaterniond map_to_base_link_quat = Eigen::Quaterniond(
      result_pose_msg.orientation.w, result_pose_msg.orientation.x, result_pose_msg.orientation.y,
      result_pose_msg.orientation.z);
    const Eigen::Matrix3d map_to_base_link_rotation =
      map_to_base_link_quat.normalized().toRotationMatrix();

    ndt_covariance =
      rotate_covariance(param_.covariance.output_pose_covariance, map_to_base_link_rotation);
    if (
      param_.covariance.covariance_estimation.covariance_estimation_type !=
      CovarianceEstimationType::FIXED_VALUE) {
      const Eigen::Matrix2d estimated_covariance_2d =
        estimate_covariance(ndt_result, initial_pose_matrix, sensor_ros_time, *ndt_ptr);
      const Eigen::Matrix2d estimated_covariance_2d_scaled =
        estimated_covariance_2d * param_.covariance.covariance_estimation.scale_factor;
      const double default_cov_xx = param_.covariance.output_pose_covariance[0];
      const double default_cov_yy = param_.covariance.output_pose_covariance[7];
      const Eigen::Matrix2d estimated_covariance_2d_adj = pclomp::adjust_diagonal_covariance(
        estimated_covariance_2d_scaled, ndt_result.pose, default_cov_xx, default_cov_yy);
      ndt_covariance[0 + 6 * 0] = estimated_covariance_2d_adj(0, 0);
      ndt_covariance[1 + 6 * 1] = estimated_covariance_2d_adj(1, 1);
      ndt_covariance[1 + 6 * 0] = estimated_covariance_2d_adj(1, 0);
      ndt_covariance[0 + 6 * 1] = estimated_covariance_2d_adj(0, 1);
    }

    // check distance_initial_to_result
    const auto distance_initial_to_result = static_cast<double>(autoware::localization_util::norm(
      interpolation_result.interpolated_pose.pose.pose.position, result_pose_msg.position));
    diagnostics_scan_points_->add_key_value(
      "distance_initial_to_result", distance_initial_to_result);
    if (distance_initial_to_result > param_.validation.initial_to_result_distance_tolerance_m) {
      std::stringstream message;
      message << "distance_initial_to_result is too large (" << distance_initial_to_result
              << " [m]).";
      diagnostics_scan_points_->update_level_and_message(
        diagnostic_msgs::msg::DiagnosticStatus::WARN, message.str());
    }

    // check execution_time
    const auto exe_end_time = std::chrono::system_clock::now();
    const auto duration_micro_sec =
      std::chrono::duration_cast<std::chrono::microseconds>(exe_end_time - exe_start_time).count();
    const auto exe_time = static_cast<float>(duration_micro_sec) / 1000.0f;
    diagnostics_scan_points_->add_key_value("execution_time", exe_time);
    if (exe_time > param_.validation.critical_upper_bound_exe_time_ms) {
      std::stringstream message;
      message << "NDT exe time is too long (took " << exe_time << " [ms]).";
      diagnostics_scan_points_->update_level_and_message(
        diagnostic_msgs::msg::DiagnosticStatus::WARN, message.str());
    }

    // publish legacy C++ outputs. exe_time wraps the prologue + middle.
    exe_time_pub_->publish(make_float32_stamped(sensor_ros_time, exe_time));
    initial_pose_with_covariance_pub_->publish(interpolation_result.interpolated_pose);
    transform_probability_pub_->publish(
      make_float32_stamped(sensor_ros_time, ndt_result.transform_probability));
    nearest_voxel_transformation_likelihood_pub_->publish(
      make_float32_stamped(sensor_ros_time, ndt_result.nearest_voxel_transformation_likelihood));
    iteration_num_pub_->publish(make_int32_stamped(sensor_ros_time, ndt_result.iteration_num));
    publish_tf(sensor_ros_time, result_pose_msg);
    publish_pose(sensor_ros_time, result_pose_msg, ndt_covariance, is_converged);
    publish_marker(sensor_ros_time, transformation_msg_array, ndt_ptr->getMaximumIterations());
    publish_initial_to_result(
      sensor_ros_time, result_pose_msg, interpolation_result.interpolated_pose,
      interpolation_result.old_pose, interpolation_result.new_pose);

    pcl::shared_ptr<pcl::PointCloud<PointSource>> sensor_points_in_map_ptr(
      new pcl::PointCloud<PointSource>);
    autoware_utils_pcl::transform_pointcloud(
      *sensor_points_in_baselink_frame, *sensor_points_in_map_ptr, ndt_result.pose);
    publish_point_cloud(sensor_ros_time, param_.frame.map_frame, sensor_points_in_map_ptr);

    // check each of point score
    const float lower_nvs = 1.0f;
    const float upper_nvs = 3.5f;
    if (voxel_score_points_pub_->get_subscription_count() > 0) {
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr nvs_points_in_map_ptr_rgb{
        new pcl::PointCloud<pcl::PointXYZRGB>};
      nvs_points_in_map_ptr_rgb =
        visualize_point_score(sensor_points_in_map_ptr, lower_nvs, upper_nvs, *ndt_ptr);
      sensor_msgs::msg::PointCloud2 nvs_points_msg_in_map;
      pcl::toROSMsg(*nvs_points_in_map_ptr_rgb, nvs_points_msg_in_map);
      nvs_points_msg_in_map.header.stamp = sensor_ros_time;
      nvs_points_msg_in_map.header.frame_id = param_.frame.map_frame;
      voxel_score_points_pub_->publish(nvs_points_msg_in_map);
    }

    // whether use no ground points to calculate score
    if (param_.score_estimation.no_ground_points.enable) {
      // remove ground
      pcl::shared_ptr<pcl::PointCloud<PointSource>> no_ground_points_in_map_ptr(
        new pcl::PointCloud<PointSource>);
      no_ground_points_in_map_ptr->points.reserve(sensor_points_in_map_ptr->size());
      // The aligned pose z is constant over the loop; the translation z of the 4x4 matrix equals
      // matrix4f_to_pose(ndt_result.pose).position.z. Hoist it to avoid rebuilding a full Pose
      // (including a quaternion extraction) for every point in the scan.
      const double result_pose_z = ndt_result.pose(2, 3);
      for (std::size_t i = 0; i < sensor_points_in_map_ptr->size(); i++) {
        const float point_z = sensor_points_in_map_ptr->points[i].z;  // NOLINT
        if (
          point_z - result_pose_z >
          param_.score_estimation.no_ground_points.z_margin_for_ground_removal) {
          no_ground_points_in_map_ptr->points.push_back(sensor_points_in_map_ptr->points[i]);
        }
      }
      // pub remove-ground points
      sensor_msgs::msg::PointCloud2 no_ground_points_msg_in_map;
      pcl::toROSMsg(*no_ground_points_in_map_ptr, no_ground_points_msg_in_map);
      no_ground_points_msg_in_map.header.stamp = sensor_ros_time;
      no_ground_points_msg_in_map.header.frame_id = param_.frame.map_frame;
      no_ground_points_aligned_pose_pub_->publish(no_ground_points_msg_in_map);
      // calculate score
      const auto no_ground_transform_probability = static_cast<float>(
        ndt_ptr->calculateTransformationProbability(*no_ground_points_in_map_ptr));
      const auto no_ground_nearest_voxel_transformation_likelihood = static_cast<float>(
        ndt_ptr->calculateNearestVoxelTransformationLikelihood(*no_ground_points_in_map_ptr));
      // pub score
      no_ground_transform_probability_pub_->publish(
        make_float32_stamped(sensor_ros_time, no_ground_transform_probability));
      no_ground_nearest_voxel_transformation_likelihood_pub_->publish(
        make_float32_stamped(sensor_ros_time, no_ground_nearest_voxel_transformation_likelihood));
    }

    return is_converged;
  });
#endif
}

void NDTScanMatcher::transform_sensor_measurement(
  const std::string & source_frame, const std::string & target_frame,
  const pcl::shared_ptr<pcl::PointCloud<PointSource>> & sensor_points_input_ptr,
  pcl::shared_ptr<pcl::PointCloud<PointSource>> & sensor_points_output_ptr)
{
  if (source_frame == target_frame) {
    sensor_points_output_ptr = sensor_points_input_ptr;
    return;
  }

  geometry_msgs::msg::TransformStamped transform;
  try {
    transform = tf2_buffer_.lookupTransform(target_frame, source_frame, tf2::TimePointZero);
  } catch (const tf2::TransformException & ex) {
    throw;
  }

  const geometry_msgs::msg::PoseStamped target_to_source_pose_stamped =
    autoware_utils_geometry::transform2pose(transform);
  const Eigen::Matrix4f base_to_sensor_matrix =
    pose_to_matrix4f(target_to_source_pose_stamped.pose);
  autoware_utils_pcl::transform_pointcloud(
    *sensor_points_input_ptr, *sensor_points_output_ptr, base_to_sensor_matrix);
}

void NDTScanMatcher::publish_tf(
  const rclcpp::Time & sensor_ros_time, const geometry_msgs::msg::Pose & result_pose_msg)
{
  geometry_msgs::msg::PoseStamped result_pose_stamped_msg;
  result_pose_stamped_msg.header.stamp = sensor_ros_time;
  result_pose_stamped_msg.header.frame_id = param_.frame.map_frame;
  result_pose_stamped_msg.pose = result_pose_msg;
  tf2_broadcaster_.sendTransform(
    autoware_utils_geometry::pose2transform(result_pose_stamped_msg, param_.frame.ndt_base_frame));
}

void NDTScanMatcher::publish_pose(
  const rclcpp::Time & sensor_ros_time, const geometry_msgs::msg::Pose & result_pose_msg,
  const std::array<double, 36> & ndt_covariance, const bool is_converged)
{
  geometry_msgs::msg::PoseStamped result_pose_stamped_msg;
  result_pose_stamped_msg.header.stamp = sensor_ros_time;
  result_pose_stamped_msg.header.frame_id = param_.frame.map_frame;
  result_pose_stamped_msg.pose = result_pose_msg;

  geometry_msgs::msg::PoseWithCovarianceStamped result_pose_with_cov_msg;
  result_pose_with_cov_msg.header.stamp = sensor_ros_time;
  result_pose_with_cov_msg.header.frame_id = param_.frame.map_frame;
  result_pose_with_cov_msg.pose.pose = result_pose_msg;
  result_pose_with_cov_msg.pose.covariance = ndt_covariance;

  if (is_converged) {
    ndt_pose_pub_->publish(result_pose_stamped_msg);
    ndt_pose_with_covariance_pub_->publish(result_pose_with_cov_msg);
  }
}

void NDTScanMatcher::publish_point_cloud(
  const rclcpp::Time & sensor_ros_time, const std::string & frame_id,
  const pcl::shared_ptr<pcl::PointCloud<PointSource>> & sensor_points_in_map_ptr)
{
  sensor_msgs::msg::PointCloud2 sensor_points_msg_in_map;
  pcl::toROSMsg(*sensor_points_in_map_ptr, sensor_points_msg_in_map);
  sensor_points_msg_in_map.header.stamp = sensor_ros_time;
  sensor_points_msg_in_map.header.frame_id = frame_id;
  sensor_aligned_pose_pub_->publish(sensor_points_msg_in_map);
}

void NDTScanMatcher::publish_marker(
  const rclcpp::Time & sensor_ros_time, const std::vector<geometry_msgs::msg::Pose> & pose_array,
  int max_iterations)
{
  visualization_msgs::msg::MarkerArray marker_array;
  visualization_msgs::msg::Marker marker;
  marker.header.stamp = sensor_ros_time;
  marker.header.frame_id = param_.frame.map_frame;
  marker.type = visualization_msgs::msg::Marker::ARROW;
  marker.action = visualization_msgs::msg::Marker::ADD;
  marker.scale = autoware_utils_visualization::create_marker_scale(0.3, 0.1, 0.1);
  int i = 0;
  marker.ns = "result_pose_matrix_array";
  marker.action = visualization_msgs::msg::Marker::ADD;
  for (const auto & pose_msg : pose_array) {
    marker.id = i++;
    marker.pose = pose_msg;
    marker.color = exchange_color_crc((1.0 * i) / 15.0);
    marker_array.markers.push_back(marker);
  }

  // TODO(Tier IV): delete old marker
  for (; i < max_iterations + 2;) {
    marker.id = i++;
    marker.pose = geometry_msgs::msg::Pose();
    marker.color = exchange_color_crc(0);
    marker_array.markers.push_back(marker);
  }
  ndt_marker_pub_->publish(marker_array);
}

void NDTScanMatcher::publish_initial_to_result(
  const rclcpp::Time & sensor_ros_time, const geometry_msgs::msg::Pose & result_pose_msg,
  const geometry_msgs::msg::PoseWithCovarianceStamped & initial_pose_cov_msg,
  const geometry_msgs::msg::PoseWithCovarianceStamped & initial_pose_old_msg,
  const geometry_msgs::msg::PoseWithCovarianceStamped & initial_pose_new_msg)
{
  geometry_msgs::msg::PoseStamped initial_to_result_relative_pose_stamped;
  initial_to_result_relative_pose_stamped.pose = autoware_utils_geometry::inverse_transform_pose(
    result_pose_msg, initial_pose_cov_msg.pose.pose);
  initial_to_result_relative_pose_stamped.header.stamp = sensor_ros_time;
  initial_to_result_relative_pose_stamped.header.frame_id = param_.frame.map_frame;
  initial_to_result_relative_pose_pub_->publish(initial_to_result_relative_pose_stamped);

  const auto initial_to_result_distance = static_cast<float>(autoware::localization_util::norm(
    initial_pose_cov_msg.pose.pose.position, result_pose_msg.position));
  initial_to_result_distance_pub_->publish(
    make_float32_stamped(sensor_ros_time, initial_to_result_distance));

  const auto initial_to_result_distance_old = static_cast<float>(autoware::localization_util::norm(
    initial_pose_old_msg.pose.pose.position, result_pose_msg.position));
  initial_to_result_distance_old_pub_->publish(
    make_float32_stamped(sensor_ros_time, initial_to_result_distance_old));

  const auto initial_to_result_distance_new = static_cast<float>(autoware::localization_util::norm(
    initial_pose_new_msg.pose.pose.position, result_pose_msg.position));
  initial_to_result_distance_new_pub_->publish(
    make_float32_stamped(sensor_ros_time, initial_to_result_distance_new));
}

int NDTScanMatcher::count_oscillation(
  const std::vector<geometry_msgs::msg::Pose> & result_pose_msg_array)
{
  return autoware::ndt_scan_matcher::count_oscillation(result_pose_msg_array);
}

#ifndef NDT_USE_RUST
// Phase N4b: under NDT_USE_RUST the covariance is computed by the Rust orchestrator
// (autoware_ndt_scan_matcher_rs_node_estimate_pose_covariance), so this method — and its references
// to the templated estimate_xy_covariance_by_multi_ndt[_score] / propose_poses_to_search /
// adjust_diagonal_covariance — is compiled only for the pure-C++ (OFF) baseline.
Eigen::Matrix2d NDTScanMatcher::estimate_covariance(
  const pclomp::NdtResult & ndt_result, const Eigen::Matrix4f & initial_pose_matrix,
  const rclcpp::Time & sensor_ros_time, NormalDistributionsTransform & ndt_ref)
{
  geometry_msgs::msg::PoseArray multi_ndt_result_msg;
  geometry_msgs::msg::PoseArray multi_initial_pose_msg;
  multi_ndt_result_msg.header.stamp = sensor_ros_time;
  multi_ndt_result_msg.header.frame_id = param_.frame.map_frame;
  multi_initial_pose_msg.header.stamp = sensor_ros_time;
  multi_initial_pose_msg.header.frame_id = param_.frame.map_frame;
  multi_ndt_result_msg.poses.push_back(matrix4f_to_pose(ndt_result.pose));
  multi_initial_pose_msg.poses.push_back(matrix4f_to_pose(initial_pose_matrix));

  if (
    param_.covariance.covariance_estimation.covariance_estimation_type ==
    CovarianceEstimationType::LAPLACE_APPROXIMATION) {
    return pclomp::estimate_xy_covariance_by_laplace_approximation(ndt_result.hessian);
  } else if (
    param_.covariance.covariance_estimation.covariance_estimation_type ==
    CovarianceEstimationType::MULTI_NDT) {
    const std::vector<Eigen::Matrix4f> poses_to_search = pclomp::propose_poses_to_search(
      ndt_result, param_.covariance.covariance_estimation.initial_pose_offset_model_x,
      param_.covariance.covariance_estimation.initial_pose_offset_model_y);
    const pclomp::ResultOfMultiNdtCovarianceEstimation result_of_multi_ndt_covariance_estimation =
      estimate_xy_covariance_by_multi_ndt(
        ndt_result, ndt_ref, poses_to_search, sensor_points_in_baselink_frame_);
    for (size_t i = 0; i < result_of_multi_ndt_covariance_estimation.ndt_initial_poses.size();
         i++) {
      multi_ndt_result_msg.poses.push_back(
        matrix4f_to_pose(result_of_multi_ndt_covariance_estimation.ndt_results[i].pose));
      multi_initial_pose_msg.poses.push_back(
        matrix4f_to_pose(result_of_multi_ndt_covariance_estimation.ndt_initial_poses[i]));
    }
    multi_ndt_pose_pub_->publish(multi_ndt_result_msg);
    multi_initial_pose_pub_->publish(multi_initial_pose_msg);
    return result_of_multi_ndt_covariance_estimation.covariance;
  } else if (
    param_.covariance.covariance_estimation.covariance_estimation_type ==
    CovarianceEstimationType::MULTI_NDT_SCORE) {
    const std::vector<Eigen::Matrix4f> poses_to_search = pclomp::propose_poses_to_search(
      ndt_result, param_.covariance.covariance_estimation.initial_pose_offset_model_x,
      param_.covariance.covariance_estimation.initial_pose_offset_model_y);
    const pclomp::ResultOfMultiNdtCovarianceEstimation
      result_of_multi_ndt_score_covariance_estimation = estimate_xy_covariance_by_multi_ndt_score(
        ndt_result, ndt_ref, poses_to_search, sensor_points_in_baselink_frame_,
        param_.covariance.covariance_estimation.temperature);
    for (const auto & sub_initial_pose_matrix : poses_to_search) {
      multi_initial_pose_msg.poses.push_back(matrix4f_to_pose(sub_initial_pose_matrix));
    }
    multi_initial_pose_pub_->publish(multi_initial_pose_msg);
    return result_of_multi_ndt_score_covariance_estimation.covariance;
  } else {
    return Eigen::Matrix2d::Identity() * param_.covariance.output_pose_covariance[0 + 6 * 0];
  }
}
#endif  // NDT_USE_RUST

pcl::PointCloud<pcl::PointXYZRGB>::Ptr NDTScanMatcher::visualize_point_score(
  const pcl::shared_ptr<pcl::PointCloud<PointSource>> & sensor_points_in_map_ptr,
  const float & lower_nvs, const float & upper_nvs, NormalDistributionsTransform & ndt_ref)
{
  pcl::PointCloud<pcl::PointXYZI> nvs_points_in_map_ptr_i;
#ifdef NDT_USE_RUST
  // N4c: per-point nearest-voxel score via the engine FFI (the adapter's calculateNearestVoxelScore-
  // EachPoint body); include only points that found a neighbor (score > 0), as the C++ output does.
  {
    const std::vector<float> flat = cloud_to_flat(*sensor_points_in_map_ptr);
    std::vector<float> scores(sensor_points_in_map_ptr->size(), 0.0F);
    autoware_ndt_scan_matcher_rs_ndt_engine_calc_nearest_voxel_score_each_point(
      ndt_ref.raw_handle(), flat.data(), sensor_points_in_map_ptr->size(), scores.data());
    for (std::size_t i = 0; i < sensor_points_in_map_ptr->size(); ++i) {
      if (scores[i] > 0.0F) {
        pcl::PointXYZI p;
        p.x = sensor_points_in_map_ptr->points[i].x;  // NOLINT
        p.y = sensor_points_in_map_ptr->points[i].y;  // NOLINT
        p.z = sensor_points_in_map_ptr->points[i].z;  // NOLINT
        p.intensity = scores[i];
        nvs_points_in_map_ptr_i.points.push_back(p);
      }
    }
  }
#else
  nvs_points_in_map_ptr_i = ndt_ref.calculateNearestVoxelScoreEachPoint(*sensor_points_in_map_ptr);
#endif
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr nvs_points_in_map_ptr_rgb{
    new pcl::PointCloud<pcl::PointXYZRGB>};

  const float range = upper_nvs - lower_nvs;
  for (std::size_t i = 0; i < nvs_points_in_map_ptr_i.size(); i++) {
    pcl::PointXYZRGB point;
    point.x = nvs_points_in_map_ptr_i.points[i].x;
    point.y = nvs_points_in_map_ptr_i.points[i].y;
    point.z = nvs_points_in_map_ptr_i.points[i].z;
    std_msgs::msg::ColorRGBA color =
      exchange_color_crc((nvs_points_in_map_ptr_i.points[i].intensity - lower_nvs) / range);
    point.r = static_cast<std::uint8_t>(color.r * 255);
    point.g = static_cast<std::uint8_t>(color.g * 255);
    point.b = static_cast<std::uint8_t>(color.b * 255);
    nvs_points_in_map_ptr_rgb->points.push_back(point);
  }
  return nvs_points_in_map_ptr_rgb;
}

void NDTScanMatcher::add_regularization_pose(
  const rclcpp::Time & sensor_ros_time, NormalDistributionsTransform & ndt_ref)
{
#ifdef NDT_USE_RUST
  // N4c: regularization set/unset via the engine FFI (scale == 0 disables), driving the handle.
  autoware_ndt_scan_matcher_rs_ndt_engine_set_regularization(ndt_ref.raw_handle(), 0.0F, 0.0F, 0.0F);
  // The regularization buffer is Rust-owned (Phase 1 slice A): interpolate (+ pop_old) over the FFI.
  AwInterpolatedPose interpolated{};
  const int64_t stamp_ns = static_cast<rclcpp::Time>(sensor_ros_time).nanoseconds();
  if (!autoware_ndt_scan_matcher_rs_regularization_interpolate(rs_.raw(), stamp_ns, &interpolated)) {
    return;
  }
  autoware_ndt_scan_matcher_rs_ndt_engine_set_regularization(
    ndt_ref.raw_handle(), static_cast<float>(interpolated.position[0]),
    static_cast<float>(interpolated.position[1]), param_.ndt.regularization_scale_factor);
#else
  ndt_ref.unsetRegularizationPose();
  std::optional<SmartPoseBuffer::InterpolateResult> interpolation_result_opt =
    regularization_pose_buffer_->interpolate(sensor_ros_time);
  if (!interpolation_result_opt) {
    return;
  }
  regularization_pose_buffer_->pop_old(sensor_ros_time);
  const SmartPoseBuffer::InterpolateResult & interpolation_result =
    interpolation_result_opt.value();
  const Eigen::Matrix4f pose = pose_to_matrix4f(interpolation_result.interpolated_pose.pose.pose);
  ndt_ref.setRegularizationPose(pose);
#endif
}

#ifdef NDT_USE_RUST
namespace
{
// Diagnostics host-interface trampolines: cast the opaque handle back to DiagnosticsInterface* and
// forward. Keys/messages arrive as (ptr, len) UTF-8 (not NUL-terminated). A migrated Rust callback
// emits its /diagnostics through these, preserving the C++ key order/values.
DiagnosticsInterface * as_diag(void * d)
{
  return static_cast<DiagnosticsInterface *>(d);
}
std::string as_str(const std::uint8_t * p, std::size_t len)
{
  return std::string(reinterpret_cast<const char *>(p), len);
}
void diag_clear(void * d)
{
  as_diag(d)->clear();
}
void diag_add_key_value_bool(void * d, const std::uint8_t * key, std::size_t key_len, bool v)
{
  as_diag(d)->add_key_value(as_str(key, key_len), v);
}
void diag_add_key_value_i64(void * d, const std::uint8_t * key, std::size_t key_len, std::int64_t v)
{
  as_diag(d)->add_key_value(as_str(key, key_len), v);
}
void diag_add_key_value_f64(void * d, const std::uint8_t * key, std::size_t key_len, double v)
{
  as_diag(d)->add_key_value(as_str(key, key_len), v);
}
void diag_add_key_value_str(
  void * d, const std::uint8_t * key, std::size_t key_len, const std::uint8_t * val,
  std::size_t val_len)
{
  as_diag(d)->add_key_value(as_str(key, key_len), as_str(val, val_len));
}
void diag_update_level_and_message(
  void * d, std::int8_t level, const std::uint8_t * msg, std::size_t msg_len)
{
  as_diag(d)->update_level_and_message(level, as_str(msg, msg_len));
}
void diag_publish(void * d, std::int64_t stamp_ns)
{
  as_diag(d)->publish(rclcpp::Time(stamp_ns));
}
AwDiagnostics make_diagnostics(DiagnosticsInterface * d)
{
  return AwDiagnostics{
    d,
    diag_clear,
    diag_add_key_value_bool,
    diag_add_key_value_i64,
    diag_add_key_value_f64,
    diag_add_key_value_str,
    diag_update_level_and_message,
    diag_publish};
}
}  // namespace

// Phase 5: the AwHost side-effects vtable trampolines (ctx == this) + make_host. Side-effects only
// (clock / logging / TF); node state stays Rust-owned on the handle.
int64_t NDTScanMatcher::host_now_ns(void * ctx)
{
  return static_cast<NDTScanMatcher *>(ctx)->now().nanoseconds();
}
void NDTScanMatcher::host_log(
  void * ctx, int32_t level, const std::uint8_t * msg, std::size_t msg_len)
{
  auto * self = static_cast<NDTScanMatcher *>(ctx);
  const std::string text(reinterpret_cast<const char *>(msg), msg_len);
  if (level >= 2) {
    RCLCPP_ERROR_STREAM_THROTTLE(self->get_logger(), *self->get_clock(), 1000, text);
  } else {
    RCLCPP_WARN_STREAM_THROTTLE(self->get_logger(), *self->get_clock(), 1000, text);
  }
}
bool NDTScanMatcher::host_lookup_transform(
  void * ctx, AwStr target, AwStr source, float * out_matrix4x4_row_major)
{
  auto * self = static_cast<NDTScanMatcher *>(ctx);
  const std::string target_frame(reinterpret_cast<const char *>(target.ptr), target.len);
  const std::string source_frame(reinterpret_cast<const char *>(source.ptr), source.len);
  try {
    const geometry_msgs::msg::TransformStamped transform =
      self->tf2_buffer_.lookupTransform(target_frame, source_frame, tf2::TimePointZero);
    const geometry_msgs::msg::PoseStamped pose_stamped =
      autoware_utils_geometry::transform2pose(transform);
    const Eigen::Matrix4f matrix = pose_to_matrix4f(pose_stamped.pose);
    for (int row = 0; row < 4; ++row) {
      for (int col = 0; col < 4; ++col) {
        out_matrix4x4_row_major[(row * 4) + col] = matrix(row, col);
      }
    }
    return true;
  } catch (const tf2::TransformException &) {
    return false;
  }
}
// AwPose (position + [x,y,z,w] quaternion) → geometry_msgs::Pose, for the publish trampolines.
static geometry_msgs::msg::Pose aw_pose_to_msg(const AwPose & p)
{
  geometry_msgs::msg::Pose pose;
  pose.position.x = p.position[0];
  pose.position.y = p.position[1];
  pose.position.z = p.position[2];
  pose.orientation.x = p.orientation[0];
  pose.orientation.y = p.orientation[1];
  pose.orientation.z = p.orientation[2];
  pose.orientation.w = p.orientation[3];
  return pose;
}
void NDTScanMatcher::host_publish_pose(
  void * ctx, AwPoseTopic topic, int64_t stamp_ns, const AwPose * pose, const double * cov)
{
  auto * self = static_cast<NDTScanMatcher *>(ctx);
  try {
    const rclcpp::Time stamp(stamp_ns);
    const geometry_msgs::msg::Pose pose_msg = aw_pose_to_msg(*pose);
    if (topic == AwPoseTopic::NdtPose) {
      geometry_msgs::msg::PoseStamped msg;
      msg.header.stamp = stamp;
      msg.header.frame_id = self->param_.frame.map_frame;
      msg.pose = pose_msg;
      self->ndt_pose_pub_->publish(msg);
    } else {
      geometry_msgs::msg::PoseWithCovarianceStamped msg;
      msg.header.stamp = stamp;
      msg.header.frame_id = self->param_.frame.map_frame;
      msg.pose.pose = pose_msg;
      if (cov != nullptr) {
        std::copy_n(cov, 36, msg.pose.covariance.begin());
      }
      if (topic == AwPoseTopic::NdtPoseWithCovariance) {
        self->ndt_pose_with_covariance_pub_->publish(msg);
      } else {
        self->initial_pose_with_covariance_pub_->publish(msg);
      }
    }
  } catch (...) {
    RCLCPP_ERROR_STREAM_THROTTLE(
      self->get_logger(), *self->get_clock(), 1000, "publish_pose failed");
  }
}
void NDTScanMatcher::host_publish_pose_array(
  void * ctx, AwPoseArrayTopic topic, int64_t stamp_ns, const AwPose * poses, std::size_t n)
{
  auto * self = static_cast<NDTScanMatcher *>(ctx);
  try {
    geometry_msgs::msg::PoseArray msg;
    msg.header.stamp = rclcpp::Time(stamp_ns);
    msg.header.frame_id = self->param_.frame.map_frame;
    for (std::size_t i = 0; i < n; ++i) {
      msg.poses.push_back(aw_pose_to_msg(poses[i]));  // NOLINT
    }
    if (topic == AwPoseArrayTopic::MultiNdtPose) {
      self->multi_ndt_pose_pub_->publish(msg);
    } else {
      self->multi_initial_pose_pub_->publish(msg);
    }
  } catch (...) {
    RCLCPP_ERROR_STREAM_THROTTLE(
      self->get_logger(), *self->get_clock(), 1000, "publish_pose_array failed");
  }
}
void NDTScanMatcher::host_publish_marker(
  void * ctx, int64_t stamp_ns, const AwPose * poses, std::size_t n, int32_t max_iterations)
{
  auto * self = static_cast<NDTScanMatcher *>(ctx);
  try {
    std::vector<geometry_msgs::msg::Pose> pose_array;
    pose_array.reserve(n);
    for (std::size_t i = 0; i < n; ++i) {
      pose_array.push_back(aw_pose_to_msg(poses[i]));  // NOLINT
    }
    self->publish_marker(rclcpp::Time(stamp_ns), pose_array, max_iterations);
  } catch (...) {
    RCLCPP_ERROR_STREAM_THROTTLE(
      self->get_logger(), *self->get_clock(), 1000, "publish_marker failed");
  }
}
void NDTScanMatcher::host_publish_float32(
  void * ctx, AwFloat32Topic topic, int64_t stamp_ns, float value)
{
  auto * self = static_cast<NDTScanMatcher *>(ctx);
  try {
    const auto msg = make_float32_stamped(rclcpp::Time(stamp_ns), value);
    if (topic == AwFloat32Topic::TransformProbability) {
      self->transform_probability_pub_->publish(msg);
    } else if (topic == AwFloat32Topic::NearestVoxelTransformationLikelihood) {
      self->nearest_voxel_transformation_likelihood_pub_->publish(msg);
    } else if (topic == AwFloat32Topic::NoGroundTransformProbability) {
      self->no_ground_transform_probability_pub_->publish(msg);
    } else {
      self->no_ground_nearest_voxel_transformation_likelihood_pub_->publish(msg);
    }
  } catch (...) {
    RCLCPP_ERROR_STREAM_THROTTLE(
      self->get_logger(), *self->get_clock(), 1000, "publish_float32 failed");
  }
}
void NDTScanMatcher::host_publish_int32(
  void * ctx, AwInt32Topic /*topic*/, int64_t stamp_ns, int32_t value)
{
  auto * self = static_cast<NDTScanMatcher *>(ctx);
  try {
    self->iteration_num_pub_->publish(make_int32_stamped(rclcpp::Time(stamp_ns), value));
  } catch (...) {
    RCLCPP_ERROR_STREAM_THROTTLE(
      self->get_logger(), *self->get_clock(), 1000, "publish_int32 failed");
  }
}
void NDTScanMatcher::host_publish_tf(void * ctx, int64_t stamp_ns, const AwPose * pose)
{
  auto * self = static_cast<NDTScanMatcher *>(ctx);
  try {
    self->publish_tf(rclcpp::Time(stamp_ns), aw_pose_to_msg(*pose));
  } catch (...) {
    RCLCPP_ERROR_STREAM_THROTTLE(
      self->get_logger(), *self->get_clock(), 1000, "publish_tf failed");
  }
}
void NDTScanMatcher::host_publish_initial_to_result(
  void * ctx, int64_t stamp_ns, const AwPose * result, const AwPose * initial,
  const double * old_pos, const double * new_pos)
{
  auto * self = static_cast<NDTScanMatcher *>(ctx);
  try {
    geometry_msgs::msg::PoseWithCovarianceStamped initial_msg;
    initial_msg.pose.pose = aw_pose_to_msg(*initial);
    geometry_msgs::msg::PoseWithCovarianceStamped old_msg;
    old_msg.pose.pose.position.x = old_pos[0];
    old_msg.pose.pose.position.y = old_pos[1];
    old_msg.pose.pose.position.z = old_pos[2];
    geometry_msgs::msg::PoseWithCovarianceStamped new_msg;
    new_msg.pose.pose.position.x = new_pos[0];
    new_msg.pose.pose.position.y = new_pos[1];
    new_msg.pose.pose.position.z = new_pos[2];
    self->publish_initial_to_result(
      rclcpp::Time(stamp_ns), aw_pose_to_msg(*result), initial_msg, old_msg, new_msg);
  } catch (...) {
    RCLCPP_ERROR_STREAM_THROTTLE(
      self->get_logger(), *self->get_clock(), 1000, "publish_initial_to_result failed");
  }
}

static pcl::PointCloud<pcl::PointXYZ> aw_xyz_slice_to_cloud(AwPoint3fSlice points);

void NDTScanMatcher::host_store_sensor_points_base_link(void * ctx, AwPoint3fSlice points)
{
  auto * self = static_cast<NDTScanMatcher *>(ctx);
  try {
    auto cloud = pcl::make_shared<pcl::PointCloud<PointSource>>(aw_xyz_slice_to_cloud(points));
    self->sensor_points_in_baselink_frame_ = cloud;
  } catch (...) {
    RCLCPP_ERROR_STREAM_THROTTLE(
      self->get_logger(), *self->get_clock(), 1000, "store_sensor_points_base_link failed");
  }
}

bool NDTScanMatcher::host_pointcloud_has_subscribers(void * ctx, AwPointCloudTopic topic)
{
  auto * self = static_cast<NDTScanMatcher *>(ctx);
  if (topic == AwPointCloudTopic::VoxelScorePoints) {
    return self->voxel_score_points_pub_->get_subscription_count() > 0;
  }
  return true;
}

static pcl::PointCloud<pcl::PointXYZ> aw_xyz_slice_to_cloud(AwPoint3fSlice points)
{
  pcl::PointCloud<pcl::PointXYZ> cloud;
  if (points.ptr == nullptr && points.len > 0) {
    return cloud;
  }
  cloud.points.reserve(points.len);
  for (std::size_t i = 0; i < points.len; ++i) {
    const std::size_t base = i * 3;
    cloud.points.emplace_back(points.ptr[base], points.ptr[base + 1], points.ptr[base + 2]);
  }
  cloud.width = static_cast<std::uint32_t>(cloud.points.size());
  cloud.height = 1;
  cloud.is_dense = true;
  return cloud;
}

void NDTScanMatcher::host_publish_pointcloud_xyz(
  void * ctx, AwPointCloudTopic topic, int64_t stamp_ns, AwPoint3fSlice points)
{
  auto * self = static_cast<NDTScanMatcher *>(ctx);
  try {
    const pcl::PointCloud<pcl::PointXYZ> cloud = aw_xyz_slice_to_cloud(points);
    sensor_msgs::msg::PointCloud2 msg;
    pcl::toROSMsg(cloud, msg);
    msg.header.stamp = rclcpp::Time(stamp_ns);
    msg.header.frame_id = self->param_.frame.map_frame;
    if (topic == AwPointCloudTopic::PointsAlignedNoGround) {
      self->no_ground_points_aligned_pose_pub_->publish(msg);
    } else {
      self->sensor_aligned_pose_pub_->publish(msg);
    }
  } catch (...) {
    RCLCPP_ERROR_STREAM_THROTTLE(
      self->get_logger(), *self->get_clock(), 1000, "publish_pointcloud_xyz failed");
  }
}

void NDTScanMatcher::host_publish_voxel_score_points(
  void * ctx, int64_t stamp_ns, AwPoint3fSlice points, const float * scores, std::size_t scores_len)
{
  auto * self = static_cast<NDTScanMatcher *>(ctx);
  try {
    if ((points.ptr == nullptr && points.len > 0) || scores == nullptr || scores_len != points.len) {
      return;
    }
    constexpr float lower_nvs = 1.0f;
    constexpr float upper_nvs = 3.5f;
    constexpr float range = upper_nvs - lower_nvs;
    pcl::PointCloud<pcl::PointXYZRGB> cloud;
    cloud.points.reserve(points.len);
    for (std::size_t i = 0; i < points.len; ++i) {
      const std::size_t base = i * 3;
      pcl::PointXYZRGB point;
      point.x = points.ptr[base];
      point.y = points.ptr[base + 1];
      point.z = points.ptr[base + 2];
      const std_msgs::msg::ColorRGBA color = exchange_color_crc((scores[i] - lower_nvs) / range);
      point.r = static_cast<std::uint8_t>(color.r * 255);
      point.g = static_cast<std::uint8_t>(color.g * 255);
      point.b = static_cast<std::uint8_t>(color.b * 255);
      cloud.points.push_back(point);
    }
    cloud.width = static_cast<std::uint32_t>(cloud.points.size());
    cloud.height = 1;
    cloud.is_dense = true;
    sensor_msgs::msg::PointCloud2 msg;
    pcl::toROSMsg(cloud, msg);
    msg.header.stamp = rclcpp::Time(stamp_ns);
    msg.header.frame_id = self->param_.frame.map_frame;
    self->voxel_score_points_pub_->publish(msg);
  } catch (...) {
    RCLCPP_ERROR_STREAM_THROTTLE(
      self->get_logger(), *self->get_clock(), 1000, "publish_voxel_score_points failed");
  }
}
AwHost NDTScanMatcher::make_host()
{
  return AwHost{
    this,
    &NDTScanMatcher::host_now_ns,
    &NDTScanMatcher::host_log,
    &NDTScanMatcher::host_lookup_transform,
    &NDTScanMatcher::host_publish_pose,
    &NDTScanMatcher::host_publish_pose_array,
    &NDTScanMatcher::host_publish_marker,
    &NDTScanMatcher::host_publish_float32,
    &NDTScanMatcher::host_publish_int32,
    &NDTScanMatcher::host_publish_tf,
    &NDTScanMatcher::host_publish_initial_to_result,
    &NDTScanMatcher::host_store_sensor_points_base_link,
    &NDTScanMatcher::host_pointcloud_has_subscribers,
    &NDTScanMatcher::host_publish_pointcloud_xyz,
    &NDTScanMatcher::host_publish_voxel_score_points};
}
#endif

void NDTScanMatcher::service_trigger_node(
  const std_srvs::srv::SetBool::Request::SharedPtr req,
  std_srvs::srv::SetBool::Response::SharedPtr res)
{
  const rclcpp::Time ros_time_now = this->now();

#ifdef NDT_USE_RUST
  // Callback-level: the whole body — diagnostics + activation + buffer clear — runs in Rust, driving
  // the handle's Rust-owned state (Phase 1 slice B). The C++ shell only builds the diagnostics vtable
  // and assigns res->success.
  const AwDiagnostics diag = make_diagnostics(diagnostics_trigger_node_.get());
  res->success = autoware_ndt_scan_matcher_rs_node_on_trigger(
    rs_.raw(), &diag, req->data, ros_time_now.nanoseconds());
#else
  diagnostics_trigger_node_->clear();
  diagnostics_trigger_node_->add_key_value("service_call_time_stamp", ros_time_now.nanoseconds());

  is_activated_ = req->data;
  if (is_activated_) {
    initial_pose_buffer_->clear();
  }
  res->success = true;

  diagnostics_trigger_node_->add_key_value("is_activated", static_cast<bool>(is_activated_));
  diagnostics_trigger_node_->add_key_value("is_succeed_service", res->success);
  diagnostics_trigger_node_->publish(ros_time_now);
#endif
}

void NDTScanMatcher::service_ndt_align(
  const autoware_internal_localization_msgs::srv::PoseWithCovarianceStamped::Request::SharedPtr req,
  autoware_internal_localization_msgs::srv::PoseWithCovarianceStamped::Response::SharedPtr res)
{
  const rclcpp::Time ros_time_now = this->now();

  diagnostics_ndt_align_->clear();

  diagnostics_ndt_align_->add_key_value("service_call_time_stamp", ros_time_now.nanoseconds());

  service_ndt_align_main(req, res);

  // check is_succeed_service
  bool is_succeed_service = res->success;
  diagnostics_ndt_align_->add_key_value("is_succeed_service", is_succeed_service);
  if (!is_succeed_service) {
    std::stringstream message;
    message << "ndt_align_service is failed.";
    diagnostics_ndt_align_->update_level_and_message(
      diagnostic_msgs::msg::DiagnosticStatus::WARN, message.str());
  }

  diagnostics_ndt_align_->publish(ros_time_now);
}

void NDTScanMatcher::service_ndt_align_main(
  const autoware_internal_localization_msgs::srv::PoseWithCovarianceStamped::Request::SharedPtr req,
  autoware_internal_localization_msgs::srv::PoseWithCovarianceStamped::Response::SharedPtr res)
{
  // get TF from pose_frame to map_frame
  const std::string & target_frame = param_.frame.map_frame;
  const std::string & source_frame = req->pose_with_covariance.header.frame_id;

  geometry_msgs::msg::TransformStamped transform_s2t;
  try {
    transform_s2t = tf2_buffer_.lookupTransform(target_frame, source_frame, tf2::TimePointZero);
  } catch (tf2::TransformException & ex) {
    // Note: Up to AWSIMv1.1.0, there is a known bug where the GNSS frame_id is incorrectly set to
    // "gnss_link" instead of "map". The ndt_align is designed to return identity when this issue
    // occurs. However, in the future, converting to a non-existent frame_id should be prohibited.

    diagnostics_ndt_align_->add_key_value("is_succeed_transform_initial_pose", false);

    std::stringstream message;
    message << "Please publish TF " << target_frame.c_str() << " to " << source_frame.c_str();
    diagnostics_ndt_align_->update_level_and_message(
      diagnostic_msgs::msg::DiagnosticStatus::ERROR, message.str());
    RCLCPP_ERROR_STREAM_THROTTLE(this->get_logger(), *this->get_clock(), 1000, message.str());
    res->success = false;
    return;
  }
  diagnostics_ndt_align_->add_key_value("is_succeed_transform_initial_pose", true);

  // transform pose_frame to map_frame
  auto initial_pose_msg_in_map_frame =
    autoware::localization_util::transform(req->pose_with_covariance, transform_s2t);
  initial_pose_msg_in_map_frame.header.stamp = req->pose_with_covariance.header.stamp;
  map_update_module_->update_map(
    initial_pose_msg_in_map_frame.pose.pose.position, diagnostics_ndt_align_);

  ndt_ptr_.with([&](auto & ndt_ptr) {
    // check is_set_map_points
#ifdef NDT_USE_RUST
    bool is_set_map_points =
      autoware_ndt_scan_matcher_rs_ndt_engine_has_target(ndt_ptr->raw_handle());
#else
    bool is_set_map_points = ndt_ptr->hasTarget();
#endif
    diagnostics_ndt_align_->add_key_value("is_set_map_points", is_set_map_points);
    if (!is_set_map_points) {
      std::stringstream message;
      message << "No InputTarget. Please check the map file and the map_loader service";
      diagnostics_ndt_align_->update_level_and_message(
        diagnostic_msgs::msg::DiagnosticStatus::WARN, message.str());
      RCLCPP_WARN_STREAM_THROTTLE(this->get_logger(), *this->get_clock(), 1000, message.str());
      res->success = false;
      return;
    }

    // check is_set_sensor_points
    bool is_set_sensor_points = (sensor_points_in_baselink_frame_ != nullptr);
    diagnostics_ndt_align_->add_key_value("is_set_sensor_points", is_set_sensor_points);
    if (!is_set_sensor_points) {
      std::stringstream message;
      message << "No InputSource. Please check the input lidar topic";
      diagnostics_ndt_align_->update_level_and_message(
        diagnostic_msgs::msg::DiagnosticStatus::WARN, message.str());
      RCLCPP_WARN_STREAM_THROTTLE(this->get_logger(), *this->get_clock(), 1000, message.str());
      res->success = false;
      return;
    }

    // estimate initial pose
    const auto [pose_with_covariance, score] = align_pose(initial_pose_msg_in_map_frame, *ndt_ptr);

    // check reliability of initial pose result
    res->reliable =
      (param_.score_estimation.converged_param_nearest_voxel_transformation_likelihood < score);
    if (!res->reliable) {
      RCLCPP_WARN_STREAM(
        this->get_logger(), "Initial Pose Estimation is Unstable. Score is " << score);
    }
    res->success = true;
    res->pose_with_covariance = pose_with_covariance;
    res->pose_with_covariance.pose.covariance = req->pose_with_covariance.pose.covariance;
  });
}

std::tuple<geometry_msgs::msg::PoseWithCovarianceStamped, double> NDTScanMatcher::align_pose(
  const geometry_msgs::msg::PoseWithCovarianceStamped & initial_pose_with_cov,
  NormalDistributionsTransform & ndt_ref)
{
  autoware::localization_util::output_pose_with_cov_to_log(
    get_logger(), "align_pose_input", initial_pose_with_cov);

  const auto base_rpy = autoware::localization_util::get_rpy(initial_pose_with_cov);
  const Eigen::Map<const autoware::localization_util::RowMatrixXd> covariance = {
    initial_pose_with_cov.pose.covariance.data(), 6, 6};
  const double stddev_x = std::sqrt(covariance(0, 0));
  const double stddev_y = std::sqrt(covariance(1, 1));
  const double stddev_z = std::sqrt(covariance(2, 2));
  const double stddev_roll = std::sqrt(covariance(3, 3));
  const double stddev_pitch = std::sqrt(covariance(4, 4));

  // Since only yaw is uniformly sampled, we define the mean and standard deviation for the others.
  const std::vector<double> sample_mean{
    initial_pose_with_cov.pose.pose.position.x,  // trans_x
    initial_pose_with_cov.pose.pose.position.y,  // trans_y
    initial_pose_with_cov.pose.pose.position.z,  // trans_z
    base_rpy.x,                                  // angle_x
    base_rpy.y                                   // angle_y
  };
  const std::vector<double> sample_stddev{stddev_x, stddev_y, stddev_z, stddev_roll, stddev_pitch};

  // Optimizing (x, y, z, roll, pitch, yaw) 6 dimensions.
  TreeStructuredParzenEstimator tpe(
    TreeStructuredParzenEstimator::Direction::MAXIMIZE,
    param_.initial_pose_estimation.n_startup_trials, sample_mean, sample_stddev);

  std::vector<Particle> particle_array;
  auto output_cloud = std::make_shared<pcl::PointCloud<PointSource>>();
#ifdef NDT_USE_RUST
  // N4e: the TPE loop aligns the (constant) source from each candidate via the engine handle; flatten
  // once. `output_cloud` is only used by the #else pclomp align below.
  const std::vector<float> source_flat = cloud_to_flat(*sensor_points_in_baselink_frame_);
#endif

  // publish the estimated poses in 20 times to see the progress and to avoid dropping data
  visualization_msgs::msg::MarkerArray marker_array;
  constexpr int64_t publish_num = 20;
  const int64_t publish_interval =
    std::max<int64_t>(param_.initial_pose_estimation.particles_num / publish_num, 1);

  for (int64_t i = 0; i < param_.initial_pose_estimation.particles_num; i++) {
    const TreeStructuredParzenEstimator::Input input = tpe.get_next_input();

    geometry_msgs::msg::Pose initial_pose;
    initial_pose.position.x = input[0];
    initial_pose.position.y = input[1];
    initial_pose.position.z = input[2];
    geometry_msgs::msg::Vector3 init_rpy;
    init_rpy.x = input[3];
    init_rpy.y = input[4];
    init_rpy.z = input[5];
    tf2::Quaternion tf_quaternion;
    tf_quaternion.setRPY(init_rpy.x, init_rpy.y, init_rpy.z);
    initial_pose.orientation = tf2::toMsg(tf_quaternion);

    const Eigen::Matrix4f initial_pose_matrix = pose_to_matrix4f(initial_pose);
#ifdef NDT_USE_RUST
    std::array<float, 16> guess16{};
    for (int r = 0; r < 4; ++r) {
      for (int c = 0; c < 4; ++c) {
        guess16[(r * 4) + c] = initial_pose_matrix(r, c);
      }
    }
    autoware_ndt_scan_matcher_rs_ndt_engine_align(
      ndt_ref.raw_handle(), guess16.data(), source_flat.data(),
      sensor_points_in_baselink_frame_->size());
    const pclomp::NdtResult ndt_result = ndt_result_from_engine(ndt_ref.raw_handle());
#else
    ndt_ref.align(*output_cloud, initial_pose_matrix, sensor_points_in_baselink_frame_);
    const pclomp::NdtResult ndt_result = ndt_ref.getResult();
#endif

    Particle particle(
      initial_pose, matrix4f_to_pose(ndt_result.pose),
      ndt_result.nearest_voxel_transformation_likelihood, ndt_result.iteration_num);
    particle_array.push_back(particle);
    push_debug_markers(marker_array, get_clock()->now(), param_.frame.map_frame, particle, i);

    if (
      (i + 1) % publish_interval == 0 || (i + 1) == param_.initial_pose_estimation.particles_num) {
      ndt_monte_carlo_initial_pose_marker_pub_->publish(marker_array);
      marker_array.markers.clear();
    }

    const geometry_msgs::msg::Pose pose = matrix4f_to_pose(ndt_result.pose);
    const geometry_msgs::msg::Vector3 rpy = autoware::localization_util::get_rpy(pose);

    TreeStructuredParzenEstimator::Input result(6);
    result[0] = pose.position.x;
    result[1] = pose.position.y;
    result[2] = pose.position.z;
    result[3] = rpy.x;
    result[4] = rpy.y;
    result[5] = rpy.z;
    tpe.add_trial(TreeStructuredParzenEstimator::Trial{result, ndt_result.transform_probability});

    auto sensor_points_in_map_ptr = std::make_shared<pcl::PointCloud<PointSource>>();
    autoware_utils_pcl::transform_pointcloud(
      *sensor_points_in_baselink_frame_, *sensor_points_in_map_ptr, ndt_result.pose);
    publish_point_cloud(
      initial_pose_with_cov.header.stamp, param_.frame.map_frame, sensor_points_in_map_ptr);
  }

  auto best_particle_ptr = std::max_element(
    std::begin(particle_array), std::end(particle_array),
    [](const Particle & lhs, const Particle & rhs) { return lhs.score < rhs.score; });

  geometry_msgs::msg::PoseWithCovarianceStamped result_pose_with_cov_msg;
  result_pose_with_cov_msg.header.stamp = initial_pose_with_cov.header.stamp;
  result_pose_with_cov_msg.header.frame_id = param_.frame.map_frame;
  result_pose_with_cov_msg.pose.pose = best_particle_ptr->result_pose;

  autoware::localization_util::output_pose_with_cov_to_log(
    get_logger(), "align_pose_output", result_pose_with_cov_msg);
  diagnostics_ndt_align_->add_key_value("best_particle_score", best_particle_ptr->score);

  return std::make_tuple(result_pose_with_cov_msg, best_particle_ptr->score);
}

}  // namespace autoware::ndt_scan_matcher

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(autoware::ndt_scan_matcher::NDTScanMatcher)
