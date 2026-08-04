// Copyright 2026 Autoware Foundation
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

#ifndef HARNESS__NDT_HARNESS_HPP_
#define HARNESS__NDT_HARNESS_HPP_

#include "diagnostics_capture.hpp"
#include "stimulus.hpp"
#include "stub_map_loader.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <autoware/ndt_scan_matcher/ndt_scan_matcher_core.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/static_transform_broadcaster.hpp>

#include <autoware_internal_localization_msgs/srv/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_srvs/srv/set_bool.hpp>

#include <rcl_yaml_param_parser/parser.h>

#include <atomic>
#include <chrono>
#include <functional>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>
#include <vector>

namespace ndt_test
{

using namespace std::chrono_literals;  // NOLINT(build/namespaces)

/// @brief Where a scan's bracketing initial poses should sit, and how to tell them apart.
struct InitialPoseSpec
{
  /// Position of the *older* pose.
  double x{map_center_x};
  double y{map_center_y};
  std::string frame_id{map_frame};
};

/// @brief One scan-driving attempt's parameters.
struct ScanDrive
{
  /// Builds the cloud for a given stamp. Defaults to the standard half-cubic scan.
  std::function<sensor_msgs::msg::PointCloud2(const builtin_interfaces::msg::Time &)> make_cloud =
    [](const builtin_interfaces::msg::Time & stamp) { return make_scan_at(stamp); };
  /// When set, two poses bracketing the scan stamp are published and confirmed first.
  std::optional<InitialPoseSpec> initial_pose{};
  /// Runs after the bracketing poses are confirmed and before the scan is published. This is the
  /// seam for tests that need to disturb buffer state mid-sequence.
  std::function<void()> before_scan{};
  /// Shifts the scan stamp relative to "now". A negative offset makes the scan late on purpose;
  /// it applies to every attempt, so retries stay equally late and a latency assertion cannot
  /// silently pass for the wrong reason.
  std::chrono::nanoseconds stamp_offset{0};
  std::chrono::nanoseconds timeout{20s};
  int attempts{3};
};

struct ScanOutcome
{
  builtin_interfaces::msg::Time stamp{};
  DiagnosticsCapture::Record diag{};
};

/// @brief Drives a real `NDTScanMatcher` and observes everything it emits.
///
/// ## Executor topology
///
/// Three executors, deliberately:
///
/// | What                                        | Executor                 | Thread        |
/// |---------------------------------------------|--------------------------|---------------|
/// | `NDTScanMatcher`                            | `MultiThreadedExecutor`  | dedicated     |
/// | `StubMapLoader`                             | `SingleThreadedExecutor` | dedicated     |
/// | observer node (captures, stimulus, clients) | `SingleThreadedExecutor` | test thread   |
///
/// The observer side is pumped with `spin_some` from the test thread, which makes waiting
/// deterministic: nothing is observed unless the test asks for it. Tests never call
/// `rclcpp::shutdown()`; the destructor owns teardown.
class NdtHarness
{
public:
  using AlignService = autoware_internal_localization_msgs::srv::PoseWithCovarianceStamped;
  using Record = DiagnosticsCapture::Record;

  /// @param overrides Appended *after* the shipped yaml. `NodeOptions::parameter_overrides()`
  /// is applied in vector order into a map, so the last write wins and these take effect.
  explicit NdtHarness(std::vector<rclcpp::Parameter> overrides = {})
  : overrides_(std::move(overrides))
  {
    node_ = std::make_shared<autoware::ndt_scan_matcher::NDTScanMatcher>(build_node_options());

    observer_ = std::make_shared<rclcpp::Node>("ndt_test_observer_" + unique_suffix());
    diagnostics_ = std::make_unique<DiagnosticsCapture>(observer_.get());

    initial_pose_pub_ = observer_->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
      "/ekf_pose_with_covariance", 10);
    // Reliable publisher into the node's best-effort `SensorDataQoS` subscription: a compatible
    // match that maximizes the chance of delivery. Drops are still possible, which is why
    // `drive_one_scan` retries on a fresh time window.
    scan_pub_ = observer_->create_publisher<sensor_msgs::msg::PointCloud2>("/points_raw", 10);

    trigger_client_ = observer_->create_client<std_srvs::srv::SetBool>("/trigger_node_srv");
    align_client_ = observer_->create_client<AlignService>("/ndt_align_srv");

    observer_executor_ = std::make_unique<rclcpp::executors::SingleThreadedExecutor>();
    observer_executor_->add_node(observer_);

    broadcast_sensor_tf();

    node_executor_ =
      std::make_unique<rclcpp::executors::MultiThreadedExecutor>(rclcpp::ExecutorOptions{}, 4);
    node_executor_->add_node(node_);
    node_thread_ = std::thread([this] { node_executor_->spin(); });

    map_loader_ = std::make_shared<StubMapLoader>();
    loader_executor_ = std::make_unique<rclcpp::executors::SingleThreadedExecutor>();
    loader_executor_->add_node(map_loader_);
    loader_thread_ = std::thread([this] { loader_executor_->spin(); });
  }

  ~NdtHarness()
  {
    // The node stops before the loader: its map-update timer can be inside `update_ndt` waiting on
    // `pcd_loader_service`, and `cancel()` only returns once that callback finishes, so the wait
    // needs the loader still running to complete.
    if (node_executor_) {
      node_executor_->cancel();
    }
    if (node_thread_.joinable()) {
      node_thread_.join();
    }
    if (loader_executor_) {
      loader_executor_->cancel();
    }
    if (loader_thread_.joinable()) {
      loader_thread_.join();
    }
    if (observer_executor_) {
      observer_executor_->remove_node(observer_);
    }
    map_loader_.reset();
    node_.reset();
    diagnostics_.reset();
    observer_.reset();
  }

  NdtHarness(const NdtHarness &) = delete;
  NdtHarness & operator=(const NdtHarness &) = delete;
  NdtHarness(NdtHarness &&) = delete;
  NdtHarness & operator=(NdtHarness &&) = delete;

  // ---------------------------------------------------------------- accessors

  [[nodiscard]] DiagnosticsCapture & diag() const { return *diagnostics_; }
  [[nodiscard]] rclcpp::Time now() const { return observer_->now(); }

  // ------------------------------------------------------------------ pumping

  /// @brief Pump until `predicate` holds, or the timeout expires.
  bool wait_until(const std::function<bool()> & predicate, const std::chrono::nanoseconds timeout)
  {
    const auto deadline = std::chrono::steady_clock::now() + timeout;
    while (rclcpp::ok()) {
      pump();
      if (predicate()) {
        return true;
      }
      if (std::chrono::steady_clock::now() >= deadline) {
        return false;
      }
      std::this_thread::sleep_for(2ms);
    }
    return false;
  }

  /// @brief Wait until the node has advertised all of its `/diagnostics` publishers.
  ///
  /// Each `DiagnosticsInterface` creates its own publisher, so the count is exact: five normally,
  /// six when `ndt.regularization.enable` is true. This is the discovery gate; no stimulus may be
  /// sent before it returns true, or the resulting diagnostics are lost.
  bool wait_for_diagnostics_ready(
    const size_t expected_publishers = 5, const std::chrono::nanoseconds timeout = 10s)
  {
    return wait_until(
      [&] { return diagnostics_->publisher_count() >= expected_publishers; }, timeout);
  }

  // -------------------------------------------------------- diagnostics waits

  /// @brief Wait for the record whose stamp equals `stamp` (subscriber statuses only).
  std::optional<Record> wait_for_diag_stamp(
    const std::string & status_name, const builtin_interfaces::msg::Time & stamp,
    const std::chrono::nanoseconds timeout = 10s)
  {
    std::optional<Record> found;
    wait_until(
      [&] {
        found = diagnostics_->find_by_stamp(status_name, stamp);
        return found.has_value();
      },
      timeout);
    return found;
  }

  /// @brief Wait for any record of `status_name` satisfying `predicate`.
  std::optional<Record> wait_for_diag(
    const std::string & status_name, const std::function<bool(const Record &)> & predicate,
    const std::chrono::nanoseconds timeout = 10s)
  {
    std::optional<Record> found;
    wait_until(
      [&] {
        for (const auto & record : diagnostics_->records(status_name)) {
          if (predicate(record)) {
            found = record;
            return true;
          }
        }
        return false;
      },
      timeout);
    return found;
  }

  // ----------------------------------------------------------------- stimulus

  /// @brief Activate the node, by calling `trigger_node_srv` (`std_srvs/SetBool`) with `true`.
  ///
  /// The service sets `is_activated_`, which gates both the scan-matching hot path and the
  /// initial-pose subscriber, and clears the initial-pose buffer as a side effect. In the real
  /// system `autoware_pose_initializer` drives it: `false` while the initial pose is being
  /// estimated, `true` once `ndt_align_srv` has answered.
  ///
  /// Returns the response's `success`, or nullopt on timeout.
  std::optional<bool> activate(const std::chrono::nanoseconds timeout = 10s)
  {
    if (!trigger_client_->wait_for_service(5s)) {
      return std::nullopt;
    }
    auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
    request->data = true;
    auto future = trigger_client_->async_send_request(request);
    if (!wait_until([&] { return future.wait_for(0s) == std::future_status::ready; }, timeout)) {
      return std::nullopt;
    }
    return future.get()->success;
  }

  /// @brief Call `ndt_align_srv`. The timeout is generous because the handler synchronously loads
  /// the map and then runs one NDT alignment per particle.
  std::optional<AlignService::Response> call_ndt_align(
    const geometry_msgs::msg::PoseWithCovarianceStamped & pose,
    const std::chrono::nanoseconds timeout = 60s)
  {
    if (!align_client_->wait_for_service(5s)) {
      return std::nullopt;
    }
    auto request = std::make_shared<AlignService::Request>();
    request->pose_with_covariance = pose;
    auto future = align_client_->async_send_request(request);
    if (!wait_until([&] { return future.wait_for(0s) == std::future_status::ready; }, timeout)) {
      return std::nullopt;
    }
    return *future.get();
  }

  /// @brief Publish one initial pose and wait until the node has reported receiving it.
  ///
  /// Waiting on the stamp-correlated diagnostic is what makes the buffer state deterministic
  /// before a scan is published, instead of sleeping and hoping. It works whether or not the pose
  /// is accepted, because the diagnostic is published either way.
  bool publish_initial_pose_and_confirm(
    const geometry_msgs::msg::PoseWithCovarianceStamped & pose,
    const std::chrono::nanoseconds timeout = 10s)
  {
    initial_pose_pub_->publish(pose);
    return wait_for_diag_stamp(initial_pose_status, pose.header.stamp, timeout).has_value();
  }

  void publish_initial_pose(const geometry_msgs::msg::PoseWithCovarianceStamped & pose)
  {
    initial_pose_pub_->publish(pose);
  }

  void publish_scan(const sensor_msgs::msg::PointCloud2 & cloud) { scan_pub_->publish(cloud); }

  /// @brief Wait until the node's subscriptions have matched our publishers.
  /// @brief Wait until the node has matched every publisher a test drives it through.
  ///
  /// `/tf_static` is included because a scan that arrives before the node's listener holds
  /// `base_link -> sensor_frame` fails its transform with an ERROR, which looks exactly like a
  /// behavior change. Matching is necessary but not sufficient -- the node still has to run the
  /// callback -- so `drive_one_scan` also retries that specific startup race.
  bool wait_for_stimulus_discovery(const std::chrono::nanoseconds timeout = 10s)
  {
    return wait_until(
      [&] {
        return scan_pub_->get_subscription_count() >= 1 &&
               initial_pose_pub_->get_subscription_count() >= 1 &&
               observer_->count_subscribers("/tf_static") >= 1;
      },
      timeout);
  }

  // -------------------------------------------------------- composite drivers

  /// @brief Publish one scan, with optional bracketing initial poses, and return the
  /// `scan_matching_status` record it produced.
  ///
  /// Each attempt uses a fresh, strictly later time window. Re-publishing the same stamp would
  /// not work: `pop_old` drops the older bracketing pose after a successful interpolation, so a
  /// retry on the same window would find fewer than two poses and produce *different*
  /// diagnostics. Retries are necessary because `points_raw` is best-effort with `keep_last(1)`.
  std::optional<ScanOutcome> drive_one_scan(const ScanDrive & drive)
  {
    const rclcpp::Time base = now() + rclcpp::Duration(drive.stamp_offset);
    for (int attempt = 0; attempt < drive.attempts; ++attempt) {
      const rclcpp::Time target = base + rclcpp::Duration(std::chrono::seconds(attempt));

      if (drive.initial_pose.has_value()) {
        const auto & spec = drive.initial_pose.value();
        const auto older =
          make_pose_at(target - rclcpp::Duration(100ms), spec.x, spec.y, spec.frame_id);
        const auto newer =
          make_pose_at(target + rclcpp::Duration(100ms), spec.x, spec.y, spec.frame_id);
        if (!publish_initial_pose_and_confirm(older) || !publish_initial_pose_and_confirm(newer)) {
          continue;
        }
      }

      if (drive.before_scan) {
        drive.before_scan();
      }

      publish_scan(drive.make_cloud(target));
      if (const auto record = wait_for_diag_stamp(scan_matching_status, target, drive.timeout)) {
        // The node can service the scan before it has processed the retained `/tf_static` sample,
        // and then the transform into `base_link` fails. That is a startup race, not an outcome, so
        // it is retried on a fresh window. Narrowed to the frame we actually broadcast, so a test
        // that deliberately uses an unknown frame still sees its own failure.
        const bool lost_tf_race = record->value("is_succeed_transform_sensor_points") == "False" &&
                                  record->message().find(sensor_frame) != std::string::npos;
        if (lost_tf_race) {
          continue;
        }
        return ScanOutcome{target, record.value()};
      }
    }
    return std::nullopt;
  }

private:
  /// @brief Process observer-side work that is currently ready.
  void pump() { observer_executor_->spin_some(5ms); }

  /// @brief Load the shipped parameter file, then append the test's overrides.
  ///
  /// Loading the real yaml keeps the shipped defaults under test (an undeclared parameter throws
  /// at construction) while the appended overrides pin whatever the assertions depend on.
  [[nodiscard]] rclcpp::NodeOptions build_node_options() const
  {
    const std::string yaml_path =
      ament_index_cpp::get_package_share_directory("autoware_ndt_scan_matcher") +
      "/config/ndt_scan_matcher.param.yaml";

    rcl_params_t * params_st = rcl_yaml_node_struct_init(rcl_get_default_allocator());
    if (!rcl_parse_yaml_file(yaml_path.c_str(), params_st)) {
      rcl_yaml_node_struct_fini(params_st);
      throw std::runtime_error("Failed to parse " + yaml_path);
    }
    const rclcpp::ParameterMap parameter_map = rclcpp::parameter_map_from(params_st, "");
    rcl_yaml_node_struct_fini(params_st);

    rclcpp::NodeOptions node_options;
    for (const auto & name_and_parameters : parameter_map) {
      for (const auto & parameter : name_and_parameters.second) {
        node_options.parameter_overrides().push_back(parameter);
      }
    }
    for (const auto & parameter : overrides_) {
      node_options.parameter_overrides().push_back(parameter);
    }
    return node_options;
  }

  /// @brief Publish the static `base_link -> sensor_frame` transform the node needs to bring a
  /// scan into the base frame.
  ///
  /// The broadcaster lives on the observer node, not on the node under test, so that the test
  /// thread owns every piece of stimulus. `/tf_static` is transient-local, so the node's listener
  /// still receives it even though it subscribes later.
  void broadcast_sensor_tf()
  {
    static_tf_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(observer_);
    geometry_msgs::msg::TransformStamped transform;
    transform.header.stamp = rclcpp::Time(0, 0);
    transform.header.frame_id = base_frame;
    transform.child_frame_id = sensor_frame;
    transform.transform.rotation.w = 1.0;
    static_tf_broadcaster_->sendTransform(transform);
  }

  /// @brief Distinct observer node names, so a lingering node from a previous test cannot be
  /// confused with this one during teardown.
  static std::string unique_suffix()
  {
    static std::atomic<int> counter{0};
    return std::to_string(counter++);
  }

  std::vector<rclcpp::Parameter> overrides_;

  std::shared_ptr<autoware::ndt_scan_matcher::NDTScanMatcher> node_;
  std::shared_ptr<rclcpp::Node> observer_;
  std::shared_ptr<StubMapLoader> map_loader_;

  std::unique_ptr<DiagnosticsCapture> diagnostics_;
  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> static_tf_broadcaster_;

  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr initial_pose_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr scan_pub_;
  rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr trigger_client_;
  rclcpp::Client<AlignService>::SharedPtr align_client_;

  std::unique_ptr<rclcpp::executors::MultiThreadedExecutor> node_executor_;
  std::unique_ptr<rclcpp::executors::SingleThreadedExecutor> loader_executor_;
  std::unique_ptr<rclcpp::executors::SingleThreadedExecutor> observer_executor_;
  std::thread node_thread_;
  std::thread loader_thread_;
};

}  // namespace ndt_test

#endif  // HARNESS__NDT_HARNESS_HPP_
