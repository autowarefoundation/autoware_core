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

/// @file
/// @brief Characterization tests for `NDTScanMatcher`.
///
/// These tests exist to make a refactor provable, not to specify desirable behavior. They pin
/// what the node does *today*, through its ROS surface (`/diagnostics`, output topics, services),
/// so that extracting the decision logic into a ROS-free core can be shown to preserve it.
///
/// Rules that keep them trustworthy:
///
///  - **Assert current behavior, even when it looks wrong.** Cases marked `SUSPICIOUS` pin
///    behavior that reads like a bug. They are deliberately frozen; changing them is a separate,
///    explicit decision, not a side effect of a refactor.
///  - **Never assert NDT numerics.** Alignment runs under OpenMP and the initial-pose search
///    draws from a process-global RNG, so pose/score values are not reproducible. Assert
///    decisions, key sets, levels and message text instead.
///  - **Override every parameter an assertion depends on**, even when it already matches the
///    shipped yaml, so a config change cannot silently flip a test.
///  - **Prefer `absent(key)` to witness ordering.** The diagnostics key set is the only evidence
///    of which gate short-circuited which.
///
/// One binary, one node per test: each test builds its own `NdtHarness`, whose destructor tears
/// the node down in a specific order. Tests never call `rclcpp::shutdown()`.

#include "harness/ndt_harness.hpp"
#include "harness/stimulus.hpp"

#include <builtin_interfaces/msg/time.hpp>
#include <rclcpp/rclcpp.hpp>

#include <autoware_internal_debug_msgs/msg/float32_stamped.hpp>
#include <autoware_internal_debug_msgs/msg/int32_stamped.hpp>
#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2_msgs/msg/tf_message.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <functional>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace
{

using ndt_test::InitialPoseSpec;
using ndt_test::NdtHarness;
using ndt_test::ScanDrive;
using ndt_test::ScanOutcome;
using ndt_test::TopicCapture;

using ndt_test::base_link_frame;
using ndt_test::map_center_x;
using ndt_test::map_center_y;
using ndt_test::map_frame;
using ndt_test::ndt_base_link_frame;

using ndt_test::initial_pose_status;
using ndt_test::map_update_status;
using ndt_test::ndt_align_status;
using ndt_test::scan_matching_status;

using ndt_test::make_empty_scan;
using ndt_test::make_near_field_scan;
using ndt_test::make_pose_at;

using Float32Stamped = autoware_internal_debug_msgs::msg::Float32Stamped;
using Int32Stamped = autoware_internal_debug_msgs::msg::Int32Stamped;

using namespace std::chrono_literals;  // NOLINT(build/namespaces)

constexpr int8_t level_ok = diagnostic_msgs::msg::DiagnosticStatus::OK;
constexpr int8_t level_warn = diagnostic_msgs::msg::DiagnosticStatus::WARN;
constexpr int8_t level_error = diagnostic_msgs::msg::DiagnosticStatus::ERROR;

/// @brief Overrides that make the initial-pose search cheap enough to run in a test.
///
/// `particles_num` is one `ndt->align` per particle, so 200 -> 10 is the twentyfold saving.
/// `n_startup_trials` has to come down with it: the TPE samples randomly until it has that many
/// trials, so leaving it at 100 would make all ten random anyway. At 10/10 they are all random --
/// the adaptive half of the search never runs here, and no case depends on it.
std::vector<rclcpp::Parameter> fast_align_overrides()
{
  return {
    rclcpp::Parameter("initial_pose_estimation.particles_num", 10),
    rclcpp::Parameter("initial_pose_estimation.n_startup_trials", 10),
    rclcpp::Parameter("ndt.num_threads", 1),  // removes OpenMP reduction nondeterminism
  };
}

/// @brief Build a harness and wait until it can be driven deterministically.
///
/// Both waits are mandatory: publishing before the node's `/diagnostics` publishers and its
/// subscriptions have been discovered loses the message. Throws rather than handing back an
/// unusable harness, so a broken environment is reported at its cause instead of as a downstream
/// timeout that reads like a behavior change. gtest catches it and fails only that case.
std::unique_ptr<NdtHarness> make_ready_harness(std::vector<rclcpp::Parameter> overrides = {})
{
  auto harness = std::make_unique<NdtHarness>(std::move(overrides));
  if (!harness->wait_for_diagnostics_ready()) {
    throw std::runtime_error("the node's /diagnostics publishers never appeared");
  }
  if (!harness->wait_for_stimulus_discovery()) {
    throw std::runtime_error("the node never subscribed to our stimulus");
  }
  return harness;
}

bool contains(const std::string & haystack, const std::string & needle)
{
  return haystack.find(needle) != std::string::npos;
}

/// Returns the keys sorted, so a comparison checks the set and the count but not the order.
std::vector<std::string> sorted_keys(std::vector<std::string> keys)
{
  std::sort(keys.begin(), keys.end());
  return keys;
}

/// The standard input: the corner scan, with two initial poses around it at the map center.
ScanDrive default_drive()
{
  ScanDrive drive;
  drive.initial_pose = InitialPoseSpec{};
  return drive;
}

template <typename MsgT>
void expect_published_once(
  NdtHarness & harness, const std::shared_ptr<TopicCapture<MsgT>> & capture,
  const ScanOutcome & outcome)
{
  ASSERT_TRUE(harness.wait_until([&] { return capture->count() >= 1; }, 5s));
  EXPECT_EQ(capture->count(), 1U) << "scan drive attempt was " << outcome.attempt;
}

/// Returns the `map_update_status` records that match `predicate`, in the order they arrived.
std::vector<NdtHarness::Record> map_update_records(
  NdtHarness & harness, const std::function<bool(const NdtHarness::Record &)> & predicate)
{
  std::vector<NdtHarness::Record> matching;
  for (const auto & record : harness.diag().records(map_update_status)) {
    if (predicate(record)) {
      matching.push_back(record);
    }
  }
  return matching;
}

/// Waits until at least `count` `map_update_status` records match `predicate`, and returns them.
std::vector<NdtHarness::Record> wait_for_map_update_records(
  NdtHarness & harness, const std::function<bool(const NdtHarness::Record &)> & predicate,
  const size_t count, const std::chrono::nanoseconds timeout)
{
  std::vector<NdtHarness::Record> matching;
  harness.wait_until(
    [&] {
      matching = map_update_records(harness, predicate);
      return matching.size() >= count;
    },
    timeout);
  return matching;
}

/// Did this timer tick call the loader? Only records from such a tick have `is_need_rebuild`.
bool is_loader_query(const NdtHarness::Record & record)
{
  return record.has_key("is_need_rebuild");
}

/// Runs `action` when it goes out of scope, so cleanup happens even after a failed assertion.
class ScopeExit
{
public:
  explicit ScopeExit(std::function<void()> action) : action_(std::move(action)) {}
  ~ScopeExit()
  {
    try {
      action_();
    } catch (const std::exception & e) {
      ADD_FAILURE() << "cleanup threw: " << e.what();
    } catch (...) {
      ADD_FAILURE() << "cleanup threw a non-standard exception";
    }
  }
  ScopeExit(const ScopeExit &) = delete;
  ScopeExit & operator=(const ScopeExit &) = delete;
  ScopeExit(ScopeExit &&) = delete;
  ScopeExit & operator=(ScopeExit &&) = delete;

private:
  std::function<void()> action_;
};

/// Deactivates the node and drives one scan, which resets the skip counter shared by all nodes.
void reset_skip_counter_via_deactivation(NdtHarness & harness)
{
  if (harness.deactivate() != std::optional<bool>(true)) {
    ADD_FAILURE() << "could not deactivate the node to reset the skip counter";
    return;
  }
  // No initial pose is needed: the activation check rejects the scan before one would matter.
  ScanDrive reset_drive;
  const auto reset_outcome = harness.drive_one_scan(reset_drive);
  if (!reset_outcome.has_value()) {
    ADD_FAILURE() << "the deactivated scan produced no scan_matching_status";
    return;
  }
  EXPECT_EQ(reset_outcome->diag.value("skipping_publish_num"), "0");
}

template <typename... Captures>
bool wait_for_capture_discovery(NdtHarness & harness, const Captures &... captures)
{
  return harness.wait_until([&] { return (... && (captures->publisher_count() >= 1)); }, 10s);
}

/// Did the node broadcast `map -> ndt_base_link` on `/tf`?
bool has_ndt_base_link_transform(const TopicCapture<tf2_msgs::msg::TFMessage> & capture)
{
  for (const auto & message : capture.messages()) {
    const bool found =
      std::any_of(message.transforms.begin(), message.transforms.end(), [](const auto & transform) {
        return transform.child_frame_id == ndt_base_link_frame &&
               transform.header.frame_id == map_frame;
      });
    if (found) {
      return true;
    }
  }
  return false;
}

// ---------------------------------------------------------------------------------------------
// Sensor-points gates: which check runs first, and which of them abort.
// ---------------------------------------------------------------------------------------------

/// An empty cloud is rejected with a WARN.
TEST(NdtScanMatcherCharacteristics, EmptyScanIsRejectedWithAWarning)
{
  // Arrange
  auto harness = make_ready_harness();

  ScanDrive drive;
  drive.make_cloud = [](const builtin_interfaces::msg::Time & stamp) {
    return make_empty_scan(stamp);  // empty cloud
  };

  // Act
  const auto outcome = harness->drive_one_scan(drive);

  // Assert
  ASSERT_TRUE(outcome.has_value());
  const auto & diag = outcome->diag;

  EXPECT_EQ(diag.level(), level_warn);
  EXPECT_TRUE(contains(diag.message(), "Sensor points is empty."))
    << "message was: " << diag.message();
}

/// SUSPICIOUS — a late scan only warns; the early return is commented out on purpose. Continuing
/// is the behavior pinned here, witnessed by `is_succeed_transform_sensor_points`.
TEST(NdtScanMatcherCharacteristics, StaleScanWarnsButProcessingContinues)
{
  // Arrange
  constexpr double timeout_sec = 1.0;
  auto harness = make_ready_harness({rclcpp::Parameter("sensor_points.timeout_sec", timeout_sec)});

  ScanDrive drive;
  drive.stamp_offset = std::chrono::seconds(-5);  // far beyond timeout_sec

  // Act
  const auto outcome = harness->drive_one_scan(drive);

  // Assert
  ASSERT_TRUE(outcome.has_value());
  const auto & diag = outcome->diag;

  EXPECT_GT(diag.value_as_double("sensor_points_delay_time_sec"), timeout_sec);
  EXPECT_TRUE(contains(diag.message(), "sensor points is experiencing latency."))
    << "message was: " << diag.message();

  // Processing continued past the latency gate. Whether it *should* is genuinely open -- the
  // production comment argues either way -- so this records today's answer, not a preference.
  EXPECT_EQ(diag.value("is_succeed_transform_sensor_points"), "True")
    << "the latency gate now aborts, where today it only warns. keys: "
    << ::testing::PrintToString(diag.keys_in_order());
}

/// A scan whose frame has no transform to `base_link` is an ERROR, and the callback stops there —
/// `sensor_points_max_distance` is absent.
TEST(NdtScanMatcherCharacteristics, ScanWithoutATransformIsAnError)
{
  // Arrange
  auto harness = make_ready_harness();

  ScanDrive drive;
  drive.make_cloud = [](const builtin_interfaces::msg::Time & stamp) {
    auto cloud = ndt_test::make_scan_at(stamp);
    cloud.header.frame_id = "no_such_frame";  // nothing broadcasts a transform for it
    return cloud;
  };

  // Act
  const auto outcome = harness->drive_one_scan(drive);

  // Assert
  ASSERT_TRUE(outcome.has_value());
  const auto & diag = outcome->diag;

  EXPECT_EQ(diag.value("is_succeed_transform_sensor_points"), "False");
  EXPECT_EQ(diag.level(), level_error) << "message was: " << diag.message();
  EXPECT_FALSE(diag.has_key("sensor_points_max_distance"))
    << "the callback ran past a failed transform. keys: "
    << ::testing::PrintToString(diag.keys_in_order());
}

/// The near-field gate runs *before* the activation check; the absent `is_activated` is the only
/// evidence of that order.
TEST(NdtScanMatcherCharacteristics, NearFieldScanIsRejectedBeforeActivationCheck)
{
  // Arrange
  // `make_near_field_scan` reaches ~0.866 m, so any required distance above that trips the gate.
  constexpr double required_distance = 10.0;
  auto harness =
    make_ready_harness({rclcpp::Parameter("sensor_points.required_distance", required_distance)});

  ScanDrive drive;
  drive.make_cloud = [](const builtin_interfaces::msg::Time & stamp) {
    return make_near_field_scan(stamp);  // near field cloud
  };

  // Act
  const auto outcome = harness->drive_one_scan(drive);

  // Assert
  ASSERT_TRUE(outcome.has_value());
  const auto & diag = outcome->diag;

  EXPECT_LT(diag.value_as_double("sensor_points_max_distance"), required_distance);
  EXPECT_EQ(diag.level(), level_warn);
  EXPECT_FALSE(diag.has_key("is_activated"))
    << "the distance gate no longer precedes the activation gate. keys: "
    << ::testing::PrintToString(diag.keys_in_order());
}

/// SUSPICIOUS — the scan is stored one line *before* the activation gate rejects it. Moving it
/// below the gate breaks initialization: `ndt_align_srv` needs a stored scan, and the node is not
/// activated while the initial pose is estimated.
TEST(NdtScanMatcherCharacteristics, SensorPointsAreStoredEvenWhileDeactivated)
{
  // Arrange
  auto harness = make_ready_harness(fast_align_overrides());

  // Deliberately do not activate: the scan must be rejected, yet still be retained.
  const auto outcome = harness->drive_one_scan(ScanDrive{});
  ASSERT_TRUE(outcome.has_value());
  ASSERT_EQ(outcome->diag.value("is_activated"), "False");

  // Act
  ASSERT_EQ(harness->activate(), std::optional<bool>(true));

  harness->diag().mark(ndt_align_status);
  const auto response =
    harness->call_ndt_align(make_pose_at(harness->now(), map_center_x, map_center_y));

  // Assert
  ASSERT_TRUE(response.has_value());

  // Waited for, not sampled: the service response can outrun the diagnostics its handler published.
  const auto diag = harness->wait_for_diag_since_mark(ndt_align_status);
  ASSERT_TRUE(diag.has_value());
  EXPECT_EQ(diag->value("is_set_sensor_points"), "True");
  EXPECT_TRUE(response->success);
}

/// Without two bracketing poses the scan aborts before the map is consulted, so a stalled startup
/// names the cause and not "Map points is not set." — `is_set_map_points` is absent.
TEST(NdtScanMatcherCharacteristics, MissingInitialPoseAbortsBeforeMapCheck)
{
  // Arrange
  auto harness = make_ready_harness();
  ASSERT_EQ(harness->activate(), std::optional<bool>(true));

  // Act
  const auto outcome = harness->drive_one_scan(ScanDrive{});

  // Assert
  ASSERT_TRUE(outcome.has_value());
  const auto & diag = outcome->diag;

  EXPECT_EQ(diag.value("is_succeed_interpolate_initial_pose"), "False");
  EXPECT_EQ(diag.level(), level_warn);
  EXPECT_FALSE(diag.has_key("is_set_map_points"))
    << "interpolation no longer short-circuits the map check. keys: "
    << ::testing::PrintToString(diag.keys_in_order());
}

/// With no map loaded the scan aborts before alignment — the absent `iteration_num` witnesses
/// that `ndt_ptr->align` was never called.
TEST(NdtScanMatcherCharacteristics, MissingMapAbortsBeforeAlignment)
{
  // Arrange
  auto harness = make_ready_harness();
  ASSERT_EQ(harness->activate(), std::optional<bool>(true));

  ScanDrive drive;
  drive.initial_pose = InitialPoseSpec{-100.0, -100.0};

  // Act
  const auto outcome = harness->drive_one_scan(drive);

  // Assert
  ASSERT_TRUE(outcome.has_value());
  const auto & diag = outcome->diag;

  EXPECT_EQ(diag.value("is_succeed_interpolate_initial_pose"), "True");
  EXPECT_EQ(diag.value("is_set_map_points"), "False");
  EXPECT_EQ(diag.level(), level_warn);
  EXPECT_FALSE(diag.has_key("iteration_num"))
    << "alignment ran without a map. keys: " << ::testing::PrintToString(diag.keys_in_order());
}

// ---------------------------------------------------------------------------------------------
// The same path past alignment. Driven by the 1 Hz map-update timer, not `ndt_align_srv`, so no
// `TreeStructuredParzenEstimator` is built and the shared random generator is untouched.
// ---------------------------------------------------------------------------------------------

/// Thresholds outside anything the node can produce: one disables a check, one always fails it.
constexpr double never_exceeded = 1.0e9;

constexpr double never_reached = 1.0e9;

/// A limit every measurement passes: the distances and times it guards are never negative.
constexpr double always_exceeded = -1.0;

/// The diagonal of `covariance.output_pose_covariance`, which the covariance case reads back.
constexpr double param_variance_xyz = 0.0225;

constexpr double param_variance_angular = 0.000625;

/// The shipped `output_pose_covariance`, rebuilt from the two constants above.
std::vector<double> output_pose_covariance()
{
  std::vector<double> covariance(36, 0.0);
  covariance[0] = covariance[7] = covariance[14] = param_variance_xyz;
  covariance[21] = covariance[28] = covariance[35] = param_variance_angular;
  return covariance;
}

/// Overrides that make a converged scan repeatable. `extra` is appended, and later entries win.
std::vector<rclcpp::Parameter> converged_hot_path_overrides(
  std::vector<rclcpp::Parameter> extra = {})
{
  std::vector<rclcpp::Parameter> overrides{
    rclcpp::Parameter("ndt.num_threads", 1),  // removes OpenMP reduction nondeterminism
    rclcpp::Parameter("ndt.max_iterations", 30),
    // These three decide whether this scene converges. The measured NVTL is about 3.2 against the
    // 2.3 threshold below, so the margin is small, and `ndt.resolution` affects it most.
    rclcpp::Parameter("ndt.resolution", 2.0),
    rclcpp::Parameter("ndt.step_size", 0.1),
    rclcpp::Parameter("ndt.trans_epsilon", 0.01),
    // All three are read by assertions: `has_ndt_base_link_transform` checks the first two,
    // `map_frame` is also the frame of `/ndt_pose`, and the sensor TF points at `base_link_frame`.
    rclcpp::Parameter("frame.ndt_base_frame", ndt_base_link_frame),
    rclcpp::Parameter("frame.map_frame", map_frame),
    rclcpp::Parameter("frame.base_frame", base_link_frame),
    rclcpp::Parameter("score_estimation.converged_param_type", 1),  // NVTL
    rclcpp::Parameter(
      "score_estimation.converged_param_nearest_voxel_transformation_likelihood", 2.3),
    rclcpp::Parameter("score_estimation.no_ground_points.enable", false),
    rclcpp::Parameter("covariance.output_pose_covariance", output_pose_covariance()),
    rclcpp::Parameter("covariance.covariance_estimation.covariance_estimation_type", 0),
    rclcpp::Parameter("validation.critical_upper_bound_exe_time_ms", never_exceeded),
    rclcpp::Parameter("validation.initial_to_result_distance_tolerance_m", never_exceeded),
    rclcpp::Parameter("validation.skipping_publish_num", 1000000),
    // Both checks run before everything else. `required_distance` is geometry: a 28.3 m cloud
    // against 10 m. `timeout_sec` is wall clock, and the delay includes two blocking initial-pose
    // round trips, so it is relaxed here. The stale-scan test checks it instead.
    rclcpp::Parameter("sensor_points.timeout_sec", never_exceeded),
    rclcpp::Parameter("sensor_points.required_distance", 10.0),
    // Both must hold: `drive_one_scan` brackets the scan stamp +/-100 ms, up to `delta_x` apart.
    rclcpp::Parameter("validation.initial_pose_timeout_sec", 1.0),
    rclcpp::Parameter("validation.initial_pose_distance_tolerance_m", 10.0),
    // The range check warns once the lidar radius reaches past the loaded radius.
    // `update_distance` decides whether moving `delta_x` triggers a second load.
    rclcpp::Parameter("dynamic_map_loading.map_radius", 150.0),
    rclcpp::Parameter("dynamic_map_loading.lidar_radius", 100.0),
    rclcpp::Parameter("dynamic_map_loading.update_distance", 20.0),
    // Regularization would add `add_regularization_pose` to this path, interpolate a buffer
    // nothing here fills, and create a sixth `/diagnostics` publisher the readiness check misses.
    rclcpp::Parameter("ndt.regularization.enable", false),
  };
  for (auto & parameter : extra) {
    overrides.push_back(std::move(parameter));
  }
  return overrides;
}

/// An unknown `converged_param_type` aligns, then discards the result: ERROR, nothing published.
TEST(NdtScanMatcherCharacteristics, UnknownConvergedParamTypeIsAnErrorAfterAligning)
{
  // Arrange
  auto harness = make_ready_harness(converged_hot_path_overrides(
    {rclcpp::Parameter("score_estimation.converged_param_type", 2)}));  // 0 and 1 are the types

  auto ndt_pose = harness->capture<geometry_msgs::msg::PoseStamped>("/ndt_pose");
  auto points_aligned = harness->capture<sensor_msgs::msg::PointCloud2>("/points_aligned");
  ASSERT_TRUE(wait_for_capture_discovery(*harness, ndt_pose, points_aligned));

  ASSERT_TRUE(harness->ensure_map_loaded());

  const ScopeExit reset_skip_counter([&] { reset_skip_counter_via_deactivation(*harness); });

  // Act
  const auto outcome = harness->drive_one_scan(default_drive());

  // Assert
  ASSERT_TRUE(outcome.has_value());
  const auto & diag = outcome->diag;

  EXPECT_TRUE(diag.has_key("iteration_num"))
    << "alignment did not run. keys: " << ::testing::PrintToString(diag.keys_in_order());
  EXPECT_FALSE(diag.has_key("transform_probability_diff"))
    << "the callback ran past the type check.";
  EXPECT_EQ(diag.level(), level_error);
  EXPECT_TRUE(contains(diag.message(), "Unknown converged param type"))
    << "message was: " << diag.message();
  EXPECT_GT(diag.value_as_double("skipping_publish_num"), 0.0);

  // The record above is published after the callback returned, so any publish came first.
  EXPECT_EQ(ndt_pose->count(), 0U);
  EXPECT_EQ(points_aligned->count(), 0U);
}

/// SUSPICIOUS — a non-converged scan withholds the pose but still broadcasts the TF: the
/// convergence gate sits inside `publish_pose`, and `publish_tf` has none. Both repairs harm.
TEST(NdtScanMatcherCharacteristics, NonConvergedScanSuppressesPoseButStillBroadcastsTf)
{
  // Arrange
  // Convergence fails on the score, not on the iteration count.
  auto harness = make_ready_harness(converged_hot_path_overrides({rclcpp::Parameter(
    "score_estimation.converged_param_nearest_voxel_transformation_likelihood", never_reached)}));

  auto ndt_pose = harness->capture<geometry_msgs::msg::PoseStamped>("/ndt_pose");
  auto ndt_pose_with_cov =
    harness->capture<geometry_msgs::msg::PoseWithCovarianceStamped>("/ndt_pose_with_covariance");
  auto points_aligned = harness->capture<sensor_msgs::msg::PointCloud2>("/points_aligned");
  auto tf = harness->capture<tf2_msgs::msg::TFMessage>("/tf");

  // These two captures must stay empty. That is the point of this test.
  ASSERT_TRUE(wait_for_capture_discovery(*harness, ndt_pose, ndt_pose_with_cov));

  ASSERT_TRUE(harness->ensure_map_loaded());

  // After the map load, so a failure there is not hidden by a cleanup with nothing to undo.
  const ScopeExit reset_skip_counter([&] { reset_skip_counter_via_deactivation(*harness); });

  // Act
  const auto outcome = harness->drive_one_scan(default_drive());

  // Assert
  ASSERT_TRUE(outcome.has_value());
  const auto & diag = outcome->diag;

  EXPECT_EQ(diag.level(), level_warn);
  EXPECT_TRUE(contains(diag.message(), "Score is below the threshold. Score: "))
    << "message was: " << diag.message();

  // `points_aligned` is the last unconditional publish, so it proves the callback passed
  // `publish_pose`.
  ASSERT_TRUE(
    harness->wait_until([&] { return points_aligned->count() >= 1 && tf->count() >= 1; }, 5s));

  EXPECT_TRUE(has_ndt_base_link_transform(*tf));
  EXPECT_EQ(ndt_pose->count(), 0U);
  EXPECT_EQ(ndt_pose_with_cov->count(), 0U);

  // Different from `ConvergedScanResetsTheSkipCounter`, which exits early at the distance check.
  // This one reaches the final `return is_converged`.
  EXPECT_GT(diag.value_as_double("skipping_publish_num"), 0.0);
}

/// The iteration limit withholds the pose even when the score is fine.
TEST(NdtScanMatcherCharacteristics, IterationLimitAloneSuppressesTheConvergedPose)
{
  // Arrange
  // `iteration_num < max_iterations` is false on the first reported iteration.
  auto harness =
    make_ready_harness(converged_hot_path_overrides({rclcpp::Parameter("ndt.max_iterations", 1)}));

  auto ndt_pose = harness->capture<geometry_msgs::msg::PoseStamped>("/ndt_pose");
  auto points_aligned = harness->capture<sensor_msgs::msg::PointCloud2>("/points_aligned");

  ASSERT_TRUE(wait_for_capture_discovery(*harness, ndt_pose));

  ASSERT_TRUE(harness->ensure_map_loaded());

  // This scan does not converge while activated, so it advances the shared skip counter.
  const ScopeExit reset_skip_counter([&] { reset_skip_counter_via_deactivation(*harness); });

  // Act
  const auto outcome = harness->drive_one_scan(default_drive());

  // Assert
  ASSERT_TRUE(outcome.has_value());
  const auto & diag = outcome->diag;

  EXPECT_EQ(diag.value("iteration_num"), "1");
  EXPECT_EQ(diag.value("local_optimal_solution_oscillation_num"), "0");
  EXPECT_EQ(diag.level(), level_warn);
  EXPECT_TRUE(contains(diag.message(), "The number of iterations has reached its upper limit."))
    << "message was: " << diag.message();
  // ASSERT, not EXPECT: if the score check also failed, the next assertion proves nothing.
  ASSERT_FALSE(contains(diag.message(), "Score is below the threshold."))
    << "the score check also failed, so this test no longer isolates the iteration check: "
    << diag.message();

  ASSERT_TRUE(harness->wait_until([&] { return points_aligned->count() >= 1; }, 5s));

  EXPECT_EQ(ndt_pose->count(), 0U);
}

/// Drives one scan and checks that the WARN it raised did not withhold the pose.
void expect_scan_warns_but_still_publishes(
  NdtHarness & harness, const std::string & expected_message)
{
  auto ndt_pose = harness.capture<geometry_msgs::msg::PoseStamped>("/ndt_pose");
  ASSERT_TRUE(harness.ensure_map_loaded());

  const auto outcome = harness.drive_one_scan(default_drive());

  ASSERT_TRUE(outcome.has_value());
  const auto & diag = outcome->diag;

  EXPECT_EQ(diag.level(), level_warn);
  EXPECT_TRUE(contains(diag.message(), expected_message)) << "message was: " << diag.message();
  EXPECT_EQ(diag.value("skipping_publish_num"), "0");

  expect_published_once(harness, ndt_pose, *outcome);
}

/// `distance_initial_to_result` over its tolerance is a WARN, and the pose still goes out.
TEST(NdtScanMatcherCharacteristics, InitialToResultDistanceOverToleranceWarnsButStillPublishes)
{
  auto harness = make_ready_harness(converged_hot_path_overrides(
    {rclcpp::Parameter("validation.initial_to_result_distance_tolerance_m", always_exceeded)}));

  expect_scan_warns_but_still_publishes(*harness, "distance_initial_to_result is too large");
}

/// `execution_time` over its bound is a WARN, and the pose still goes out.
TEST(NdtScanMatcherCharacteristics, ExecutionTimeOverBoundWarnsButStillPublishes)
{
  auto harness = make_ready_harness(converged_hot_path_overrides(
    {rclcpp::Parameter("validation.critical_upper_bound_exe_time_ms", always_exceeded)}));

  expect_scan_warns_but_still_publishes(*harness, "NDT exe time is too long");
}

/// Reaching `validation.skipping_publish_num` appends the "exceed limit" WARN — the comparison is
/// `>=`, so the threshold value itself warns. The counter is a `static` shared by the binary.
TEST(NdtScanMatcherCharacteristics, SkipCounterWarnsWhenItReachesTheThreshold)
{
  // Arrange
  // `required_distance` is what rejects the near-field scan below, so it is pinned alongside.
  auto harness = make_ready_harness(
    {rclcpp::Parameter("validation.skipping_publish_num", 1),
     rclcpp::Parameter("sensor_points.required_distance", 10.0)});

  ScanDrive near_field;
  near_field.make_cloud = [](const builtin_interfaces::msg::Time & stamp) {
    return make_near_field_scan(stamp);
  };
  const auto zeroed = harness->drive_one_scan(near_field);
  ASSERT_TRUE(zeroed.has_value());
  ASSERT_EQ(zeroed->diag.value("skipping_publish_num"), "0");

  ASSERT_EQ(harness->activate(), std::optional<bool>(true));

  // Act
  const auto outcome = harness->drive_one_scan(near_field);

  // Assert
  ASSERT_TRUE(outcome.has_value());
  const auto & diag = outcome->diag;

  EXPECT_EQ(diag.value("skipping_publish_num"), "1");
  EXPECT_EQ(diag.level(), level_warn);
  EXPECT_TRUE(contains(diag.message(), "skipping_publish_num exceed limit"))
    << "message was: " << diag.message();
}

/// A converged scan reports exactly these nineteen diagnostics keys.
TEST(NdtScanMatcherCharacteristics, ScanMatchingStatusEmitsExactlyTheseNineteenKeys)
{
  // Arrange
  auto harness = make_ready_harness(converged_hot_path_overrides());
  ASSERT_TRUE(harness->ensure_map_loaded());

  // Act
  const auto outcome = harness->drive_one_scan(default_drive());

  // Assert
  ASSERT_TRUE(outcome.has_value());
  const auto & diag = outcome->diag;

  const std::vector<std::string> expected_keys{
    "topic_time_stamp",
    "sensor_points_size",
    "sensor_points_delay_time_sec",
    "is_succeed_transform_sensor_points",
    "sensor_points_max_distance",
    "is_activated",
    "is_succeed_interpolate_initial_pose",
    "is_set_map_points",
    "iteration_num",
    "local_optimal_solution_oscillation_num",
    "transform_probability",
    "nearest_voxel_transformation_likelihood",
    "transform_probability_diff",
    "transform_probability_before",
    "nearest_voxel_transformation_likelihood_diff",
    "nearest_voxel_transformation_likelihood_before",
    "distance_initial_to_result",
    "execution_time",
    "skipping_publish_num",
  };
  // Sorted, so the count is checked but the positions are free.
  EXPECT_EQ(sorted_keys(diag.keys_in_order()), sorted_keys(expected_keys));

  // The message and hardware id are not checked: `DiagnosticsInterface` builds both from the level
  // and the node name, so checking them would test that package instead of this node.
  EXPECT_EQ(diag.level(), level_ok) << "message was: " << diag.message();
}

/// Which topics one converged scan publishes, and which stay silent.
TEST(NdtScanMatcherCharacteristics, ConvergedScanPublishesTheseTopicsAndNotThose)
{
  // Arrange
  auto harness = make_ready_harness(converged_hot_path_overrides());

  // Captures must exist before the input, or checking for silence proves nothing.
  auto ndt_pose = harness->capture<geometry_msgs::msg::PoseStamped>("/ndt_pose");
  auto ndt_pose_with_cov =
    harness->capture<geometry_msgs::msg::PoseWithCovarianceStamped>("/ndt_pose_with_covariance");
  auto initial_pose_with_cov = harness->capture<geometry_msgs::msg::PoseWithCovarianceStamped>(
    "/initial_pose_with_covariance");
  auto points_aligned = harness->capture<sensor_msgs::msg::PointCloud2>("/points_aligned");
  auto exe_time = harness->capture<Float32Stamped>("/exe_time_ms");
  auto transform_probability = harness->capture<Float32Stamped>("/transform_probability");
  auto nvtl = harness->capture<Float32Stamped>("/nearest_voxel_transformation_likelihood");
  auto iteration_num = harness->capture<Int32Stamped>("/iteration_num");
  auto ndt_marker = harness->capture<visualization_msgs::msg::MarkerArray>("/ndt_marker");
  auto relative_pose =
    harness->capture<geometry_msgs::msg::PoseStamped>("/initial_to_result_relative_pose");
  auto distance = harness->capture<Float32Stamped>("/initial_to_result_distance");
  auto distance_old = harness->capture<Float32Stamped>("/initial_to_result_distance_old");
  auto distance_new = harness->capture<Float32Stamped>("/initial_to_result_distance_new");
  auto tf = harness->capture<tf2_msgs::msg::TFMessage>("/tf");

  auto no_ground_points =
    harness->capture<sensor_msgs::msg::PointCloud2>("/points_aligned_no_ground");
  auto no_ground_tp = harness->capture<Float32Stamped>("/no_ground_transform_probability");
  auto no_ground_nvtl =
    harness->capture<Float32Stamped>("/no_ground_nearest_voxel_transformation_likelihood");
  auto multi_ndt_pose = harness->capture<geometry_msgs::msg::PoseArray>("/multi_ndt_pose");
  auto multi_initial_pose = harness->capture<geometry_msgs::msg::PoseArray>("/multi_initial_pose");

  // They must also have found a publisher, or silence only means discovery has not finished.
  ASSERT_TRUE(wait_for_capture_discovery(
    *harness, no_ground_points, no_ground_tp, no_ground_nvtl, multi_ndt_pose, multi_initial_pose));

  ASSERT_TRUE(harness->ensure_map_loaded());

  // Act
  const auto outcome = harness->drive_one_scan(default_drive());

  // Assert
  ASSERT_TRUE(outcome.has_value());
  ASSERT_EQ(outcome->diag.level(), level_ok)
    << "scan did not converge: " << outcome->diag.message();

  // The observer is a separate node on a separate executor, so the publish order inside the
  // callback says nothing about the arrival order. The silence checks rely on the discovery wait
  // and on the diagnostics record, which is published after the callback returned. That is not
  // proof of delivery, because DDS does not order messages across writers.
  ASSERT_TRUE(harness->wait_until(
    [&] {
      return ndt_pose->count() >= 1 && ndt_pose_with_cov->count() >= 1 &&
             initial_pose_with_cov->count() >= 1 && exe_time->count() >= 1 &&
             transform_probability->count() >= 1 && nvtl->count() >= 1 &&
             iteration_num->count() >= 1 && ndt_marker->count() >= 1 &&
             relative_pose->count() >= 1 && distance->count() >= 1 && distance_old->count() >= 1 &&
             distance_new->count() >= 1 && tf->count() >= 1 && points_aligned->count() >= 1;
    },
    5s))
    << "not every expected publication arrived";

  // A retry runs alignment twice, so every count below would read 2. `attempt` tells that apart
  // from the node publishing twice. Retrying is still worth it: the scan uses best-effort
  // `SensorDataQoS` and can be dropped, while only a lost reliable status would double-count.
  EXPECT_EQ(ndt_pose->count(), 1U) << "scan drive attempt was " << outcome->attempt;
  EXPECT_EQ(ndt_pose_with_cov->count(), 1U);
  EXPECT_EQ(initial_pose_with_cov->count(), 1U);
  EXPECT_EQ(points_aligned->count(), 1U);
  EXPECT_EQ(exe_time->count(), 1U);
  EXPECT_EQ(transform_probability->count(), 1U);
  EXPECT_EQ(nvtl->count(), 1U);
  EXPECT_EQ(iteration_num->count(), 1U);
  EXPECT_EQ(ndt_marker->count(), 1U);
  EXPECT_EQ(relative_pose->count(), 1U);
  EXPECT_EQ(distance->count(), 1U);
  EXPECT_EQ(distance_old->count(), 1U);
  EXPECT_EQ(distance_new->count(), 1U);

  const auto published_pose = ndt_pose->first();
  ASSERT_TRUE(published_pose.has_value());
  EXPECT_EQ(published_pose->header.frame_id, map_frame);
  // `first()` is the pose from the earliest attempt, but `outcome->stamp` is the last attempt's
  // window, so a retry would look here like the node stamping its output wrongly.
  EXPECT_EQ(published_pose->header.stamp, outcome->stamp)
    << "scan drive attempt was " << outcome->attempt;

  EXPECT_TRUE(has_ndt_base_link_transform(*tf));

  EXPECT_EQ(no_ground_points->count(), 0U);
  EXPECT_EQ(no_ground_tp->count(), 0U);
  EXPECT_EQ(no_ground_nvtl->count(), 0U);
  EXPECT_EQ(multi_ndt_pose->count(), 0U);
  EXPECT_EQ(multi_initial_pose->count(), 0U);
}

/// SUSPICIOUS — the estimate overwrites only 4 of the 36 covariance entries, and the two
/// off-diagonal writes are transposed. Harmless only while estimators return symmetric matrices.
TEST(NdtScanMatcherCharacteristics, EstimatedCovarianceOverwritesOnlyFourOfThirtySixEntries)
{
  // Arrange
  constexpr double scale_factor = 1.0e6;

  // LAPLACE_APPROXIMATION: an estimate that needs no extra alignments.
  auto harness = make_ready_harness(converged_hot_path_overrides(
    {rclcpp::Parameter("covariance.covariance_estimation.covariance_estimation_type", 1),
     rclcpp::Parameter("covariance.covariance_estimation.scale_factor", scale_factor)}));

  auto ndt_pose_with_cov =
    harness->capture<geometry_msgs::msg::PoseWithCovarianceStamped>("/ndt_pose_with_covariance");

  ASSERT_TRUE(harness->ensure_map_loaded());

  // Act
  const auto outcome = harness->drive_one_scan(default_drive());

  // Assert
  ASSERT_TRUE(outcome.has_value());
  ASSERT_EQ(outcome->diag.level(), level_ok)
    << "scan did not converge: " << outcome->diag.message();

  ASSERT_TRUE(harness->wait_until([&] { return ndt_pose_with_cov->count() >= 1; }, 5s));
  const auto published = ndt_pose_with_cov->first();
  ASSERT_TRUE(published.has_value());
  const auto & covariance = published->pose.covariance;

  constexpr double tolerance = 1e-12;

  // Untouched by the rotation and by the four-index overwrite.
  EXPECT_NEAR(covariance[14], param_variance_xyz, tolerance) << "z variance was overwritten";
  EXPECT_NEAR(covariance[21], param_variance_angular, tolerance) << "roll variance was overwritten";
  EXPECT_NEAR(covariance[28], param_variance_angular, tolerance)
    << "pitch variance was overwritten";
  EXPECT_NEAR(covariance[35], param_variance_angular, tolerance) << "yaw variance was overwritten";

  // Overwritten by the scaled estimate, which `scale_factor` lifts well above the floor. If the
  // estimation branch were skipped, these would still read exactly `param_variance_xyz`.
  EXPECT_GT(covariance[0], param_variance_xyz * 10.0) << "the x variance was not overwritten";
  EXPECT_GT(covariance[7], param_variance_xyz * 10.0) << "the y variance was not overwritten";
  // Magnitude first. Symmetry alone cannot catch the realistic mistake: dropping *both*
  // off-diagonal writes leaves the two entries equal, at the tiny value the rotation leaves
  // behind, and the loop below skips indices 1 and 6. The estimate here is about -0.04 after
  // scaling, so this floor is far above that leftover value and far below the estimate.
  EXPECT_GT(std::abs(covariance[1]), 1.0e-6) << "the xy cross terms were never written";
  // Then symmetry, which catches one of the two writes being dropped. It cannot catch the
  // transpose above, which needs an asymmetric input and so a unit test on the extracted function.
  EXPECT_NEAR(covariance[1], covariance[6], std::abs(covariance[1]) * 1e-9 + tolerance);

  // Every other entry stays zero: the parameter matrix is diagonal and the estimate only touches
  // the four indices above.
  for (size_t i = 0; i < 36; ++i) {
    if (i == 0 || i == 1 || i == 6 || i == 7 || i == 14 || i == 21 || i == 28 || i == 35) {
      continue;
    }
    EXPECT_NEAR(covariance[i], 0.0, 1e-12) << "unexpected non-zero at covariance[" << i << "]";
  }
}

/// The pose `align` starts from is the interpolated midpoint, not either surrounding pose.
TEST(NdtScanMatcherCharacteristics, PublishedInitialPoseIsTheInterpolatedMidpoint)
{
  // Arrange
  auto harness = make_ready_harness(converged_hot_path_overrides());

  auto initial_pose_with_cov = harness->capture<geometry_msgs::msg::PoseWithCovarianceStamped>(
    "/initial_pose_with_covariance");

  ASSERT_TRUE(harness->ensure_map_loaded());

  constexpr double newer_pose_delta_x = 2.0;

  auto drive = default_drive();
  // So the interpolated position differs from both endpoints.
  drive.initial_pose->delta_x = newer_pose_delta_x;

  // Act
  const auto outcome = harness->drive_one_scan(drive);

  // Assert
  ASSERT_TRUE(outcome.has_value());
  ASSERT_EQ(outcome->diag.value("is_succeed_interpolate_initial_pose"), "True");
  // Convergence is checked even though this test is about the interpolated position, because the
  // non-converged test's cleanup depends on it: every converged test except that one resets the
  // shared skip counter by matching successfully. If this test stopped converging, it would start
  // leaving the counter above zero without saying so.
  ASSERT_EQ(outcome->diag.level(), level_ok)
    << "scan did not converge: " << outcome->diag.message();

  ASSERT_TRUE(harness->wait_until([&] { return initial_pose_with_cov->count() >= 1; }, 5s));
  const auto published = initial_pose_with_cov->first();
  ASSERT_TRUE(published.has_value());
  const auto & interpolated = *published;

  // The scan stamp sits exactly between the two poses.
  EXPECT_GT(interpolated.pose.pose.position.x, map_center_x);
  EXPECT_LT(interpolated.pose.pose.position.x, map_center_x + newer_pose_delta_x);
  EXPECT_NEAR(interpolated.pose.pose.position.x, map_center_x + newer_pose_delta_x / 2.0, 1e-6);
}

/// `initial_pose_distance_tolerance_m` applies to the gap between the two surrounding poses.
TEST(NdtScanMatcherCharacteristics, InitialPoseDistanceToleranceReachesTheInterpolationBuffer)
{
  // Arrange
  auto harness = make_ready_harness(converged_hot_path_overrides(
    {rclcpp::Parameter("validation.initial_pose_distance_tolerance_m", 5.0)}));
  ASSERT_EQ(harness->activate(), std::optional<bool>(true));

  // Rejected while activated, so the shared skip counter advances.
  const ScopeExit reset_skip_counter([&] { reset_skip_counter_via_deactivation(*harness); });

  // Act
  auto drive = default_drive();
  drive.initial_pose->delta_x = 6.0;

  const auto outcome = harness->drive_one_scan(drive);

  // Assert
  ASSERT_TRUE(outcome.has_value());
  const auto & diag = outcome->diag;

  EXPECT_EQ(diag.value("is_succeed_interpolate_initial_pose"), "False");
  EXPECT_EQ(diag.level(), level_warn);
  EXPECT_FALSE(diag.has_key("is_set_map_points"))
    << "interpolation accepted poses 6 m apart against a 5 m tolerance.";
}

/// A converged scan resets the skip counter, checked after a rejected scan raised it.
TEST(NdtScanMatcherCharacteristics, ConvergedScanResetsTheSkipCounter)
{
  // Arrange
  auto harness = make_ready_harness(converged_hot_path_overrides());
  ASSERT_TRUE(harness->ensure_map_loaded());

  auto rejected = default_drive();
  rejected.make_cloud = [](const builtin_interfaces::msg::Time & stamp) {
    return make_near_field_scan(stamp);
  };
  const auto advanced = harness->drive_one_scan(rejected);
  ASSERT_TRUE(advanced.has_value());
  ASSERT_GT(advanced->diag.value_as_double("skipping_publish_num"), 0.0);

  // Act
  const auto outcome = harness->drive_one_scan(default_drive());

  // Assert
  ASSERT_TRUE(outcome.has_value());
  ASSERT_EQ(outcome->diag.level(), level_ok)
    << "scan did not converge: " << outcome->diag.message();
  EXPECT_EQ(outcome->diag.value("skipping_publish_num"), "0");
}

/// With TRANSFORM_PROBABILITY selected, its own threshold decides convergence.
TEST(NdtScanMatcherCharacteristics, TransformProbabilityTypeIsJudgedByItsOwnThreshold)
{
  // Arrange
  auto harness = make_ready_harness(converged_hot_path_overrides(
    {rclcpp::Parameter("score_estimation.converged_param_type", 0),  // TRANSFORM_PROBABILITY
     rclcpp::Parameter("score_estimation.converged_param_transform_probability", never_reached)}));

  auto ndt_pose = harness->capture<geometry_msgs::msg::PoseStamped>("/ndt_pose");
  auto points_aligned = harness->capture<sensor_msgs::msg::PointCloud2>("/points_aligned");
  ASSERT_TRUE(wait_for_capture_discovery(*harness, ndt_pose));

  ASSERT_TRUE(harness->ensure_map_loaded());

  const ScopeExit reset_skip_counter([&] { reset_skip_counter_via_deactivation(*harness); });

  // Act
  const auto outcome = harness->drive_one_scan(default_drive());

  // Assert
  ASSERT_TRUE(outcome.has_value());
  const auto & diag = outcome->diag;

  EXPECT_EQ(diag.level(), level_warn);
  EXPECT_TRUE(contains(diag.message(), "Score is below the threshold. Score: "))
    << "message was: " << diag.message();

  ASSERT_TRUE(harness->wait_until([&] { return points_aligned->count() >= 1; }, 5s));
  EXPECT_EQ(ndt_pose->count(), 0U);
}

/// Out of map range is a WARN on the scan and an ERROR on the timer, and the pose still goes out.
TEST(NdtScanMatcherCharacteristics, OutOfMapRangeIsAWarnOnTheScanAndAnErrorOnTheTimer)
{
  // Arrange
  auto harness = make_ready_harness(converged_hot_path_overrides(
    {rclcpp::Parameter("dynamic_map_loading.lidar_radius", 151.0)}));  // `map_radius` is 150

  // Act and Assert, scan side: the shared warn-and-continue shape.
  ASSERT_NO_FATAL_FAILURE(
    expect_scan_warns_but_still_publishes(*harness, "Lidar has gone out of the map range"));

  // Timer side. The vehicle has not moved `update_distance`, so the timer only reports. A missing
  // `is_need_rebuild` key shows no rebuild was attempted.
  const auto timer = harness->wait_for_diag(
    map_update_status,
    [](const NdtHarness::Record & record) { return record.level() == level_error; },
    std::chrono::seconds(5));
  ASSERT_TRUE(timer.has_value());
  EXPECT_TRUE(contains(timer->message(), "Dynamic map loading is not keeping up"))
    << "message was: " << timer->message();
  EXPECT_FALSE(timer->has_key("is_need_rebuild")) << "the timer went on to update the map.";

  // Side effect of the ERROR: past `update_distance`, the next load rebuilds instead of adding.
  ASSERT_TRUE(harness->publish_initial_pose_and_confirm(
    make_pose_at(harness->now(), map_center_x + 25.0, map_center_y)));
  const auto loads =
    wait_for_map_update_records(*harness, is_loader_query, 2U, std::chrono::seconds(5));
  ASSERT_GE(loads.size(), 2U) << "the timer never loaded again after the move";
  EXPECT_EQ(loads.back().value("is_need_rebuild"), "True");
  EXPECT_EQ(loads.back().value("is_updated_map"), "True");
}

/// Activating the node clears the initial-pose buffer — a side effect nothing else observes. The
/// control arm proves the sequence would otherwise have interpolated successfully.
TEST(NdtScanMatcherCharacteristics, ActivatingClearsTheInitialPoseBuffer)
{
  {
    // Arrange
    // Control arm: same sequence without the second activation.
    auto harness = make_ready_harness();
    ASSERT_EQ(harness->activate(), std::optional<bool>(true));

    ScanDrive drive;
    drive.initial_pose = InitialPoseSpec{};

    // Act
    const auto outcome = harness->drive_one_scan(drive);

    // Assert
    ASSERT_TRUE(outcome.has_value());
    ASSERT_EQ(outcome->diag.value("is_succeed_interpolate_initial_pose"), "True");
  }
  {
    // Arrange
    // Experiment: re-activate between the poses and the scan.
    auto harness = make_ready_harness();
    ASSERT_EQ(harness->activate(), std::optional<bool>(true));

    ScanDrive drive;
    drive.initial_pose = InitialPoseSpec{};
    drive.before_scan = [&] { harness->activate(); };

    // Act
    const auto outcome = harness->drive_one_scan(drive);

    // Assert
    ASSERT_TRUE(outcome.has_value());
    EXPECT_EQ(outcome->diag.value("is_succeed_interpolate_initial_pose"), "False");
  }
}

// ---------------------------------------------------------------------------------------------
// Initial-pose subscriber: validation order and severity.
// ---------------------------------------------------------------------------------------------

/// An initial pose arriving while deactivated is dropped before its frame is checked, so startup
/// shows "Node is not activated." and not a frame-id ERROR — `is_expected_frame_id` is absent.
TEST(NdtScanMatcherCharacteristics, InitialPoseIsRejectedBeforeTheFrameCheckWhenNotActivated)
{
  // Arrange
  auto harness = make_ready_harness();

  // Deliberately do not activate, and use a *valid* frame so the frame check would have passed.
  const auto pose = make_pose_at(harness->now(), map_center_x, map_center_y, map_frame);

  // Act
  ASSERT_TRUE(harness->publish_initial_pose_and_confirm(pose));

  // Assert
  const auto diag = harness->diag().find_by_stamp(initial_pose_status, pose.header.stamp);
  ASSERT_TRUE(diag.has_value());

  EXPECT_EQ(diag->value("is_activated"), "False");
  EXPECT_EQ(diag->level(), level_warn);
  EXPECT_FALSE(diag->has_key("is_expected_frame_id"))
    << "the activation check no longer precedes the frame check. keys: "
    << ::testing::PrintToString(diag->keys_in_order());
}

/// A wrong `frame_id` on the initial pose is an ERROR, not a WARN: severity splits by whether the
/// condition can resolve itself, and a publisher using the wrong frame keeps using it.
TEST(NdtScanMatcherCharacteristics, WrongFrameIdOnInitialPoseIsErrorNotWarn)
{
  // Arrange
  auto harness = make_ready_harness();
  ASSERT_EQ(harness->activate(), std::optional<bool>(true));

  const auto pose = make_pose_at(harness->now(), 0.0, 0.0, base_link_frame);

  // Act
  ASSERT_TRUE(harness->publish_initial_pose_and_confirm(pose));

  // Assert
  const auto diag = harness->diag().find_by_stamp(initial_pose_status, pose.header.stamp);
  ASSERT_TRUE(diag.has_value());

  EXPECT_EQ(diag->level(), level_error) << "severity was downgraded; message: " << diag->message();
  EXPECT_EQ(diag->value("is_expected_frame_id"), "False");
}

/// A rejected initial pose updates neither the interpolation buffer nor the map anchor — two side
/// effects behind one early return, and nothing else observes either.
TEST(NdtScanMatcherCharacteristics, RejectedInitialPoseUpdatesNeitherBufferNorMapAnchor)
{
  // Arrange
  auto harness = make_ready_harness();
  ASSERT_EQ(harness->activate(), std::optional<bool>(true));

  // Only wrong-frame poses are ever published.
  ScanDrive drive;
  drive.initial_pose = InitialPoseSpec{};
  drive.initial_pose->frame_id = base_link_frame;

  // Act
  const auto outcome = harness->drive_one_scan(drive);

  // Assert
  // The buffer stayed empty.
  ASSERT_TRUE(outcome.has_value());
  EXPECT_EQ(outcome->diag.value("is_succeed_interpolate_initial_pose"), "False");

  // The map anchor stayed unset, so the timer cannot even try to load.
  const auto diag = harness->wait_for_diag(
    map_update_status,
    [](const NdtHarness::Record & record) {
      return record.value("is_activated") == "True" &&
             record.has_key("is_set_last_update_position");
    },
    std::chrono::seconds(15));
  ASSERT_TRUE(diag.has_value());

  EXPECT_EQ(diag->value("is_set_last_update_position"), "False");
  EXPECT_EQ(diag->level(), level_warn);
}
}  // namespace

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  ::testing::InitGoogleTest(&argc, argv);
  const int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
