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
///
/// **Run this through ctest, not by invoking the binary.** `ament_add_ros_isolated_gtest` hands the
/// target an unused `ROS_DOMAIN_ID`; run directly, it lands on the default domain and shares topic
/// and service names with anything else there. Two concurrent instances are enough: the
/// `count() == 1` assertions see the other instance's publications, and its
/// `/ekf_pose_with_covariance` and `/trigger_node_srv` traffic drives this instance's node, so
/// interpolation and map state come out wrong. Measured across two- and four-way runs, anywhere
/// from a few to sixteen assertions fail per process, varying widely between runs — and every one
/// of them reads like a defect in the node. A node built from patched sources on the same domain
/// can break the absence assertions too, so a mutation experiment running next door is enough —
/// though only some runs get that far, because the level assertion ahead of them aborts the body
/// first. Debugging a single case directly is fine; set `ROS_DOMAIN_ID` yourself first.
///
/// **The counter values in these comments assume declaration order.** `skipping_publish_num` is a
/// function-local `static`, so one counter is shared by every node this binary builds, and the
/// trajectory the cases below describe -- reaching 5 by
/// `RejectedInitialPoseUpdatesNeitherBufferNorMapAnchor` -- is the one declaration order produces.
/// `--gtest_shuffle` and `--gtest_repeat` change it, and `--gtest_filter` to a subset drops the
/// cases that reset it, so those numbers stop describing what happens.
///
/// Shuffling has been measured to pass, over a dozen seeds. What survives it is the assertions:
/// reaching the shipped threshold of 5 appends an "exceed limit" WARN, and nothing here compares a
/// whole message or expects OK from a case that already warns. So this is a note about which
/// numbers to trust while reading, and a latent hazard for whoever adds a case that does compare a
/// whole message -- not a reason to avoid the flag.

#include "harness/ndt_harness.hpp"
#include "harness/stimulus.hpp"

#include <builtin_interfaces/msg/time.hpp>
#include <rclcpp/rclcpp.hpp>

#include <autoware_internal_debug_msgs/msg/float32_stamped.hpp>
#include <autoware_internal_debug_msgs/msg/int32_stamped.hpp>
#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2_msgs/msg/tf_message.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
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

/// @brief Runs `action` on destruction, so cleanup happens on the failing paths too.
///
/// A failed `ASSERT_*` returns from the test body immediately, which would skip any cleanup written
/// as ordinary trailing statements.
///
/// `action` runs inside a destructor, which is implicitly `noexcept`, so anything it lets escape
/// would call `std::terminate`. Cleanup here reaches into rclcpp, which throws, and it runs exactly
/// when a case has already failed — so an unguarded throw would take the gtest report down with it,
/// which is the opposite of the point. Failures are reported instead.
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

/// @brief Deactivate the node and drive one more scan, which resets the skip counter.
///
/// `skipping_publish_num` is a function-local `static` shared by every node this binary builds, so
/// a case that leaves it advanced hands the next one a value it does not expect. The
/// `!is_activated_` branch is one of the counter's two resets, and driving a scan through it both
/// pins that branch and leaves the counter clean. Called from a `ScopeExit` so a failed assertion
/// cannot skip it.
///
/// `ADD_FAILURE` plus an explicit return rather than `ASSERT_*`, because `ASSERT_*` compiles only
/// in a function returning void and this is called from a lambda that may not stay one.
void reset_skip_counter_via_deactivation(NdtHarness & harness)
{
  if (harness.deactivate() != std::optional<bool>(true)) {
    ADD_FAILURE() << "could not deactivate the node to reset the skip counter";
    return;
  }
  // No initial pose: the activation gate rejects the scan before one would matter.
  ScanDrive reset_drive;
  const auto reset_outcome = harness.drive_one_scan(reset_drive);
  if (!reset_outcome.has_value()) {
    ADD_FAILURE() << "the deactivated scan produced no scan_matching_status";
    return;
  }
  EXPECT_EQ(reset_outcome->diag.value("skipping_publish_num"), "0")
    << "deactivating no longer resets the skip counter";
}

/// @brief Wait until each capture has matched the node's publisher.
///
/// Only needed for the captures an absence assertion reads. For the ones a test expects to receive,
/// arrival is itself proof that discovery finished; for "this was never published", nothing else
/// distinguishes a silent node from a subscription that had not matched yet. This is the same gate
/// `wait_for_diagnostics_ready` applies to `/diagnostics`, which the suite already treats as
/// mandatory.
template <typename... Captures>
bool wait_for_capture_discovery(NdtHarness & harness, const Captures &... captures)
{
  return harness.wait_until([&] { return (... && (captures->publisher_count() >= 1)); }, 10s);
}

/// @brief Did the node broadcast `map -> ndt_base_link` on `/tf`?
///
/// `/tf` is read as raw `TFMessage` rather than through a `tf2_ros::Buffer` because the question is
/// whether a message was *sent*, which a buffer's time-based lookup would obscure.
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
  // And it stopped there. Without this the case pins only the warning, so dropping the gate's
  // `return false;` and letting an empty cloud run on would keep it green -- measured.
  // `sensor_points_delay_time_sec` is the next key the callback adds.
  EXPECT_FALSE(diag.has_key("sensor_points_delay_time_sec"))
    << "an empty cloud was processed past its gate";
}

/// SUSPICIOUS — the early return for a late scan is commented out on purpose.
///
/// `callback_sensor_points_main` reports the latency as a WARN and then *keeps going*; the
/// `return false;` sits commented out under a four-line comment explaining the choice. Any
/// reimplementation of "detect the timeout and report it" returns instead, and then NDT stops
/// publishing exactly when the LiDAR is late — the moment localization matters most.
///
/// The witness that execution continued is `is_succeed_transform_sensor_points`, the next key the
/// callback adds after the latency check.
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

/// A scan whose frame has no transform to `base_link` is an ERROR, and the callback stops there.
///
/// The "missing TF" that `WrongFrameIdOnInitialPoseIsErrorNotWarn`'s severity split names, pinned
/// on the scan side. The lookup is `TimePointZero` with no timeout, so an unknown frame fails at
/// once. `absent("sensor_points_max_distance")` is the witness for the stop.
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

/// The near-field gate runs *before* the activation check.
///
/// Hoisting the cheap `is_activated_` boolean above the O(n) distance scan is a tempting
/// optimization, and it is behavior-changing because of
/// `SensorPointsAreStoredEvenWhileDeactivated` below. `absent("is_activated")` is the only
/// evidence of the current order.
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

/// SUSPICIOUS — the scan is stored one line *before* the activation gate rejects it.
///
/// `sensor_points_in_baselink_frame_` is assigned inside the `ndt_ptr_` lock and immediately
/// before `if (!is_activated_) return false;`. In the deactivated path that assignment looks like
/// dead work, so the natural "gate first, then mutate state" extraction moves it below the gate.
///
/// That would break vehicle initialization: `service_ndt_align_main` requires a stored scan, and
/// the node is *not* activated while the initial pose is being estimated. Nothing else in the
/// repository guards this ordering — the pre-existing tests all activate first.
TEST(NdtScanMatcherCharacteristics, SensorPointsAreStoredEvenWhileDeactivated)
{
  // Arrange
  auto harness = make_ready_harness(fast_align_overrides());

  // Deliberately do not activate: the scan must be rejected, yet still be retained.
  const auto outcome = harness->drive_one_scan(ScanDrive{});
  ASSERT_TRUE(outcome.has_value());
  ASSERT_EQ(outcome->diag.value("is_activated"), "False");
  // The gate also stopped the callback: dropping its `return false;` lets a deactivated scan reach
  // interpolation, which nothing else here would notice -- measured.
  EXPECT_FALSE(outcome->diag.has_key("is_succeed_interpolate_initial_pose"))
    << "a scan was processed past the activation gate";

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

/// Without two bracketing poses the scan aborts before the map is consulted.
///
/// Nothing but the scan is published, so *both* gates would fail: with no initial pose the map
/// anchor stays unset, and the 1 Hz timer therefore never loads a map either. Only the order
/// decides which diagnostic a stalled startup shows — today "Couldn't interpolate pose.", which
/// names the cause (no EKF), rather than "Map points is not set.", which names a consequence.
///
/// Hoisting the cheap `hasTarget()` above the interpolation is the tempting rewrite, and
/// `absent("is_set_map_points")` is the only witness of the current order.
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

/// With no map loaded, the scan aborts before any alignment happens.
///
/// The poses sit at (-100, -100), where `StubMapLoader` answers with nothing. The load fails once;
/// a failed load still records the position (`map_update_module.cpp:176`), so the timer does not
/// try again until the vehicle moves `update_distance`. `absent("iteration_num")` is the witness
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

/// Activating the node clears the initial-pose buffer.
///
/// `service_trigger_node` reaches into buffer state that the extraction will move into the core
/// object, and this `clear()` is a side effect nothing else in the world observes. The control
/// arm proves the sequence would otherwise have interpolated successfully, so the assertion
/// cannot pass for an unrelated reason.
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

/// Reaching `validation.skipping_publish_num` appends the "exceed limit" WARN, and the comparison
/// is inclusive.
///
/// The counter is a function-local `static` shared by every node this binary builds, so the case
/// zeroes it first: a rejected scan while deactivated takes the `!is_activated_` arm. One rejected
/// scan while activated then reads 1, which against a threshold of 1 is the boundary -- `>=` warns
/// where `>` would not.
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

// ---------------------------------------------------------------------------------------------
// Initial-pose subscriber: validation order and severity.
// ---------------------------------------------------------------------------------------------

/// An initial pose arriving while the node is deactivated is dropped before its frame is checked.
///
/// The ordering is pure convention and reverses trivially in a rewrite, and it decides which
/// diagnostic an operator sees during startup: "Node is not activated." (WARN) rather than a
/// frame-id ERROR. `absent("is_expected_frame_id")` is the only witness.
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

/// A wrong `frame_id` on the initial pose is an ERROR, not a WARN.
///
/// Severity splits by whether the condition can resolve itself: WARN for transient states -- not
/// activated, no pose to interpolate, no map yet, a poor score -- and ERROR for configuration that
/// never will, a missing TF and this. (`map_update_status` has one that does not fit.) Whoever
/// publishes `ekf_pose_with_covariance` in the wrong frame keeps doing so, so ERROR is consistent,
/// not an outlier. The split is nowhere stated in code, so normalizing severities into one helper
/// would flatten it -- and WARN silently disarms whatever supervises the node.
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

/// A rejected initial pose updates neither the interpolation buffer nor the map anchor.
///
/// `push_back` and the `latest_ekf_position_` write are two side effects sitting behind one early
/// return. An extraction that computes "should I buffer this?" separately from "where should the
/// map be centered?" can easily hoist one of them above the frame check, and nothing else
/// observes either.
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

// ---------------------------------------------------------------------------------------------
// The converged hot path. Reached via the 1 Hz map-update timer, not `ndt_align_srv`, so no
// `TreeStructuredParzenEstimator` is constructed and the process-global RNG stays untouched.
// ---------------------------------------------------------------------------------------------

/// The diagonal of `covariance.output_pose_covariance`, which the covariance case reads back.
constexpr double param_variance_xyz = 0.0225;
constexpr double param_variance_angular = 0.000625;

/// @brief The shipped `output_pose_covariance`, rebuilt from the two constants above.
///
/// Pinned rather than trusted so that the numbers the covariance case asserts are *definitions*
/// instead of a copy of the yaml that a config change could silently invalidate.
std::vector<double> output_pose_covariance()
{
  std::vector<double> covariance(36, 0.0);
  covariance[0] = covariance[7] = covariance[14] = param_variance_xyz;
  covariance[21] = covariance[28] = covariance[35] = param_variance_angular;
  return covariance;
}

/// @brief Overrides that make a converged scan deterministic.
///
/// Five differ from the shipped values. `ndt.num_threads` is one; the other four disable WARNs that
/// are not this path's subject and would otherwise follow CI load -- `1e9` on
/// `critical_upper_bound_exe_time_ms`, `initial_to_result_distance_tolerance_m` and
/// `sensor_points.timeout_sec`, and a `skipping_publish_num` no run can reach, its counter being a
/// function-local `static` shared by every node this binary builds.
std::vector<rclcpp::Parameter> converged_hot_path_overrides()
{
  return {
    rclcpp::Parameter("ndt.num_threads", 1),  // removes OpenMP reduction nondeterminism
    rclcpp::Parameter("ndt.max_iterations", 30),
    // These three decide whether this scene converges. The measured NVTL is about 3.2 against the
    // 2.3 threshold below, so the margin is under one point, and `ndt.resolution` moves it most.
    rclcpp::Parameter("ndt.resolution", 2.0),
    rclcpp::Parameter("ndt.step_size", 0.1),
    rclcpp::Parameter("ndt.trans_epsilon", 0.01),
    // All three are read by assertions: the first two are the child and parent
    // `has_ndt_base_link_transform` matches, `map_frame` is also the `/ndt_pose` `header.frame_id`,
    // and `base_link_frame` is what the harness's static sensor transform targets.
    rclcpp::Parameter("frame.ndt_base_frame", ndt_base_link_frame),
    rclcpp::Parameter("frame.map_frame", map_frame),
    rclcpp::Parameter("frame.base_frame", base_link_frame),
    rclcpp::Parameter("score_estimation.converged_param_type", 1),  // NVTL
    rclcpp::Parameter(
      "score_estimation.converged_param_nearest_voxel_transformation_likelihood", 2.3),
    rclcpp::Parameter("score_estimation.no_ground_points.enable", false),
    rclcpp::Parameter("covariance.output_pose_covariance", output_pose_covariance()),
    rclcpp::Parameter("covariance.covariance_estimation.covariance_estimation_type", 0),
    rclcpp::Parameter("validation.critical_upper_bound_exe_time_ms", 1.0e9),
    rclcpp::Parameter("validation.initial_to_result_distance_tolerance_m", 1.0e9),
    rclcpp::Parameter("validation.skipping_publish_num", 1000000),
    // Both gates run ahead of everything else. `required_distance` is geometry -- a 28.3 m cloud
    // against 10 m. `timeout_sec` is wall clock, and the delay it measures includes the two
    // blocking initial-pose round-trips `drive_one_scan` makes after taking the scan stamp, so it
    // is relaxed instead; `StaleScanWarnsButProcessingContinues` covers the latency WARN itself.
    rclcpp::Parameter("sensor_points.timeout_sec", 1.0e9),
    rclcpp::Parameter("sensor_points.required_distance", 10.0),
    // The bracketing poses `drive_one_scan` sends sit +/-100 ms around the scan stamp and up to
    // `delta_x` apart, so both of these have to hold for interpolation to succeed.
    rclcpp::Parameter("validation.initial_pose_timeout_sec", 1.0),
    rclcpp::Parameter("validation.initial_pose_distance_tolerance_m", 10.0),
    // The map-range check appends a WARN once the lidar radius reaches past the loaded radius, and
    // `update_distance` decides whether moving `delta_x` triggers a second map load -- which would
    // reset `ndt_ptr` mid-case.
    rclcpp::Parameter("dynamic_map_loading.map_radius", 150.0),
    rclcpp::Parameter("dynamic_map_loading.lidar_radius", 100.0),
    rclcpp::Parameter("dynamic_map_loading.update_distance", 20.0),
    // Enabling regularization puts `add_regularization_pose` into the hot path, interpolating a
    // second buffer nothing here feeds, and adds a sixth `/diagnostics` publisher -- which
    // `wait_for_diagnostics_ready` survives, testing `>= expected_publishers`, but only by matching
    // five of six.
    rclcpp::Parameter("ndt.regularization.enable", false),
  };
}

/// What the node records about its own reasoning: nineteen keys, these names.
///
/// No other case observes them, so an extraction that groups the decisions into a result struct can
/// drop or rename one and stay green everywhere else.
///
/// Insertion order is deliberately not asserted, because nothing consumes it: the aggregator
/// matches on `status.name` and copies `values` through untouched, and no other package names these
/// keys. The literal below is in insertion order anyway, to read alongside the callback.
TEST(NdtScanMatcherCharacteristics, ScanMatchingStatusEmitsExactlyTheseNineteenKeys)
{
  // Arrange
  auto harness = make_ready_harness(converged_hot_path_overrides());
  ASSERT_TRUE(harness->ensure_map_loaded()) << "the stub map never loaded";

  // Act
  ScanDrive drive;
  drive.initial_pose = InitialPoseSpec{};

  const auto outcome = harness->drive_one_scan(drive);

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
  // Sorted, so the count stays pinned and only the positions are free.
  auto sorted = [](std::vector<std::string> keys) {
    std::sort(keys.begin(), keys.end());
    return keys;
  };
  EXPECT_EQ(sorted(diag.keys_in_order()), sorted(expected_keys));

  // Neither the message nor the hardware id is asserted, because neither could fail here.
  // `create_diagnostics_array` rewrites the message to "OK" whenever the level is OK, and
  // `DiagnosticsInterface` builds `name` as `get_name() + ": " + diagnostic_name` alongside
  // `hardware_id = get_name()` — so a record found under `scan_matching_status` can only carry
  // "ndt_scan_matcher". Both would be testing `autoware_utils_diagnostics`, not this node.
  EXPECT_EQ(diag.level(), level_ok) << "message was: " << diag.message();
}

/// The node's whole output surface for one converged scan.
///
/// The "absent" half carries most of the value, and its two halves are not equal.
/// `score_estimation.no_ground_points.enable` withholds three topics and is the one conditional
/// here nothing else reaches: forcing it true fails this case.
///
/// The two `multi_*` assertions are weaker. They pin "at FIXED_VALUE the multi-search arrays stay
/// silent", not the `!= FIXED_VALUE` gate: measured, removing that gate keeps every case green,
/// because `estimate_covariance` then falls to its final `else`, and only MULTI_NDT and
/// MULTI_NDT_SCORE publish those arrays. Catching the gate needs a case that selects one of them.
///
/// `voxel_score_points` is deliberately not captured. The node only publishes it when its
/// publisher already has a subscriber, so subscribing in order to assert absence would create the
/// very condition being tested.
TEST(NdtScanMatcherCharacteristics, ConvergedScanPublishesTheseTopicsAndNotThose)
{
  // Arrange
  auto harness = make_ready_harness(converged_hot_path_overrides());

  // Captures must exist before the stimulus, or the absence assertions prove nothing.
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

  // The five captures the absence assertions read must have matched before the stimulus, or those
  // assertions say nothing about the node.
  ASSERT_TRUE(wait_for_capture_discovery(
    *harness, no_ground_points, no_ground_tp, no_ground_nvtl, multi_ndt_pose, multi_initial_pose))
    << "a capture never matched the node's publisher; the absence assertions would be vacuous";

  ASSERT_TRUE(harness->ensure_map_loaded()) << "the stub map never loaded";

  // Act
  ScanDrive drive;
  drive.initial_pose = InitialPoseSpec{};

  const auto outcome = harness->drive_one_scan(drive);

  // Assert
  ASSERT_TRUE(outcome.has_value());
  ASSERT_EQ(outcome->diag.level(), level_ok)
    << "scan did not converge: " << outcome->diag.message();

  // Every expected publication, not a couple of early ones: the observer is a separate node on a
  // separate executor, so publish order inside the callback says nothing about arrival order, and
  // waiting on TF and `ndt_pose` alone would leave the counts below racing the publisher.
  //
  // The absence assertions rest elsewhere -- on the discovery gate above, and on the diagnostics
  // record `drive_one_scan` returned, which `scan_matching_status` publishes after
  // `callback_sensor_points_main` returns, so every publish call for this scan has happened. That
  // is not proof of delivery, since DDS orders nothing across writers; the guarantee is empirical,
  // from forcing `no_ground_points.enable` true and watching this case fail.
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

  // A retry inside `drive_one_scan` can drive alignment twice, which would make every count below
  // read 2 as if the node had published twice. `attempt` is how a failure here says which happened.
  //
  // `attempts = 1` would rule it out and is deliberately not used: of the three retry paths only a
  // lost `scan_matching_status` can follow a completed `align`, and that record travels a reliable
  // `KeepAll` subscription while the scan itself goes into best-effort `SensorDataQoS`. Dropping
  // the retry trades a rare double count for a common hard failure on a dropped scan.
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
  // The retry hazard bites hardest here: `first()` is the earliest attempt's pose while
  // `outcome->stamp` is the last attempt's window, so a retry makes this read as the node stamping
  // its output wrongly. `attempt` separates the two readings.
  EXPECT_EQ(published_pose->header.stamp, outcome->stamp)
    << "scan drive attempt was " << outcome->attempt;

  EXPECT_TRUE(has_ndt_base_link_transform(*tf))
    << "map -> ndt_base_link was not broadcast for a converged scan";

  EXPECT_EQ(no_ground_points->count(), 0U) << "no_ground scoring is disabled";
  EXPECT_EQ(no_ground_tp->count(), 0U);
  EXPECT_EQ(no_ground_nvtl->count(), 0U);
  EXPECT_EQ(multi_ndt_pose->count(), 0U) << "covariance_estimation_type is FIXED_VALUE";
  EXPECT_EQ(multi_initial_pose->count(), 0U);
}

/// SUSPICIOUS — a non-converged scan suppresses the pose but still broadcasts the TF.
///
/// `publish_tf` is called unconditionally while `publish_pose` gates on `is_converged` *inside*
/// itself. Lifting that gate to the call site — the obvious cleanup — also gates the TF, and the
/// vehicle then silently loses `map -> ndt_base_link` whenever NDT scores poorly. Moving the gate
/// the other way feeds a bad pose to the EKF. Neither direction is caught by anything else.
TEST(NdtScanMatcherCharacteristics, NonConvergedScanSuppressesPoseButStillBroadcastsTf)
{
  // Arrange
  auto overrides = converged_hot_path_overrides();
  // Unreachable score threshold: convergence is forced to fail on the score, not on iterations.
  overrides.emplace_back(
    "score_estimation.converged_param_nearest_voxel_transformation_likelihood", 1.0e9);
  auto harness = make_ready_harness(std::move(overrides));

  auto ndt_pose = harness->capture<geometry_msgs::msg::PoseStamped>("/ndt_pose");
  auto ndt_pose_with_cov =
    harness->capture<geometry_msgs::msg::PoseWithCovarianceStamped>("/ndt_pose_with_covariance");
  auto points_aligned = harness->capture<sensor_msgs::msg::PointCloud2>("/points_aligned");
  auto exe_time = harness->capture<Float32Stamped>("/exe_time_ms");
  auto nvtl = harness->capture<Float32Stamped>("/nearest_voxel_transformation_likelihood");
  auto iteration_num = harness->capture<Int32Stamped>("/iteration_num");
  auto ndt_marker = harness->capture<visualization_msgs::msg::MarkerArray>("/ndt_marker");
  auto distance = harness->capture<Float32Stamped>("/initial_to_result_distance");
  auto tf = harness->capture<tf2_msgs::msg::TFMessage>("/tf");

  // The two captures whose emptiness is the point of this case.
  ASSERT_TRUE(wait_for_capture_discovery(*harness, ndt_pose, ndt_pose_with_cov))
    << "a capture never matched the node's publisher; the absence assertions would be vacuous";

  ASSERT_TRUE(harness->ensure_map_loaded()) << "the stub map never loaded";

  // Declared after the map is loaded on purpose: nothing above drives a scan, so the counter cannot
  // have advanced yet, and a failure in `ensure_map_loaded` would otherwise bury its own cause
  // under a pointless cleanup. This scan leaves the counter advanced, so the guard resets it
  // through the
  // `!is_activated_` branch -- which also pins that branch, since asserting `"0"` on a converged
  // scan would pin nothing.
  const ScopeExit reset_skip_counter([&] { reset_skip_counter_via_deactivation(*harness); });

  // Act
  ScanDrive drive;
  drive.initial_pose = InitialPoseSpec{};

  const auto outcome = harness->drive_one_scan(drive);

  // Assert
  ASSERT_TRUE(outcome.has_value());
  const auto & diag = outcome->diag;

  EXPECT_EQ(diag.level(), level_warn);
  EXPECT_TRUE(contains(diag.message(), "Score is below the threshold. Score: "))
    << "message was: " << diag.message();

  // Every expected publication, for the reason the converged case gives: publish *order* inside the
  // callback says nothing about arrival order at a separate node on a separate executor. Waiting
  // for `points_aligned` because it is published last would be relying on exactly the inference
  // that case rejects — and a small `Float32Stamped` overtaking a 1,323-point cloud is the easy
  // direction for that to go wrong.
  ASSERT_TRUE(harness->wait_until(
    [&] {
      return points_aligned->count() >= 1 && tf->count() >= 1 && exe_time->count() >= 1 &&
             nvtl->count() >= 1 && iteration_num->count() >= 1 && ndt_marker->count() >= 1 &&
             distance->count() >= 1;
    },
    5s))
    << "the alignment outputs never arrived";

  // The alignment ran and everything else was published ...
  EXPECT_EQ(points_aligned->count(), 1U) << "scan drive attempt was " << outcome->attempt;
  EXPECT_EQ(exe_time->count(), 1U);
  EXPECT_EQ(nvtl->count(), 1U);
  EXPECT_EQ(iteration_num->count(), 1U);
  EXPECT_EQ(ndt_marker->count(), 1U);
  EXPECT_EQ(distance->count(), 1U);
  // ... including the TF, which is not gated on convergence ...
  EXPECT_TRUE(has_ndt_base_link_transform(*tf))
    << "map -> ndt_base_link is now gated on convergence; downstream TF consumers would break";
  // ... but the pose itself was withheld.
  EXPECT_EQ(ndt_pose->count(), 0U) << "a non-converged pose reached ndt_pose";
  EXPECT_EQ(ndt_pose_with_cov->count(), 0U)
    << "a non-converged pose reached ndt_pose_with_covariance";

  // The other half of the counter's contract: a scan rejected while the node is activated advances
  // it. `reset_skip_counter` at the top of this case pins the reset, and cleans it up.
  // `value_as_double` rather than comparing the string: `value()` returns "" for a missing key, and
  // "" != "0" would pass if the key disappeared altogether. NaN fails this comparison.
  EXPECT_GT(diag.value_as_double("skipping_publish_num"), 0.0)
    << "a scan rejected while activated no longer advances the skip counter";
}

/// The other arm of `is_converged`'s `||`: hitting the iteration limit withholds the pose even when
/// the score is fine.
///
/// `is_converged` is `(is_ok_iteration_num || is_local_optimal_solution_oscillation) &&
/// is_ok_score`. The case above drives the `is_ok_score` half; without this one, rewriting the
/// whole expression to `is_converged = is_ok_score` passes every case in the file, which is
/// measured and was the state of this suite before this case existed. That rewrite is exactly what
/// extracting the judgement into a helper invites.
///
/// `ndt.max_iterations: 1` is what makes the arm reachable through the node: the alignment reports
/// one iteration, so `iteration_num < max_iterations` is false, while the geometry still scores
/// above the threshold. `count_oscillation` needs three poses before it can even look at a
/// reversal, so the oscillation arm stays false here.
///
/// The remaining combination -- oscillation rescuing a hit limit, two bad signals reading as
/// converged -- needs eleven consecutive step reversals and is not reachable from this stimulus.
/// Pinning it needs `decide_convergence` moved into `ndt_scan_matcher_helper`, where a truth table
/// reaches it directly. That is the next step in the plan.
TEST(NdtScanMatcherCharacteristics, IterationLimitAloneSuppressesTheConvergedPose)
{
  // Arrange
  auto overrides = converged_hot_path_overrides();
  overrides.emplace_back("ndt.max_iterations", 1);
  auto harness = make_ready_harness(std::move(overrides));

  auto ndt_pose = harness->capture<geometry_msgs::msg::PoseStamped>("/ndt_pose");
  auto points_aligned = harness->capture<sensor_msgs::msg::PointCloud2>("/points_aligned");

  ASSERT_TRUE(wait_for_capture_discovery(*harness, ndt_pose))
    << "a capture never matched the node's publisher; the absence assertion would be vacuous";

  ASSERT_TRUE(harness->ensure_map_loaded()) << "the stub map never loaded";

  // Non-converging while activated, so this case advances the shared skip counter too.
  const ScopeExit reset_skip_counter([&] { reset_skip_counter_via_deactivation(*harness); });

  // Act
  ScanDrive drive;
  drive.initial_pose = InitialPoseSpec{};

  const auto outcome = harness->drive_one_scan(drive);

  // Assert
  ASSERT_TRUE(outcome.has_value());
  const auto & diag = outcome->diag;

  // The limit was reached, and it alone decided the outcome: the score arm is satisfied here.
  EXPECT_EQ(diag.value("iteration_num"), "1");
  EXPECT_EQ(diag.value("local_optimal_solution_oscillation_num"), "0");
  EXPECT_EQ(diag.level(), level_warn);
  EXPECT_TRUE(contains(diag.message(), "The number of iterations has reached its upper limit."))
    << "message was: " << diag.message();
  EXPECT_FALSE(contains(diag.message(), "Score is below the threshold."))
    << "the score arm also failed, so this case no longer isolates the iteration arm: "
    << diag.message();

  ASSERT_TRUE(harness->wait_until([&] { return points_aligned->count() >= 1; }, 5s))
    << "the alignment never ran";
  EXPECT_EQ(ndt_pose->count(), 0U) << "a pose that hit the iteration limit reached ndt_pose";
}

/// A converged scan resets the skip counter; that reset is a branch of its own.
///
/// `skipping_publish_num` is assigned `(is_succeed_scan_matching || !is_activated_) ? 0 : n + 1`.
/// The deactivation arm is pinned by the two non-converging cases, which reset through it. Nothing
/// pinned the success arm: an assertion that a converged scan reports "0" is unconditionally true
/// while the arm exists, so it says nothing, and dropping the arm passed every other case --
/// measured. What makes it observable is arriving at the converged scan with the counter already
/// advanced.
///
/// The near-field scan is what advances it: rejected at the distance gate while the node is
/// activated, which is the increment condition, and rejected deterministically rather than by
/// timing.
TEST(NdtScanMatcherCharacteristics, ConvergedScanResetsTheSkipCounter)
{
  // Arrange
  auto harness = make_ready_harness(converged_hot_path_overrides());
  ASSERT_TRUE(harness->ensure_map_loaded()) << "the stub map never loaded";

  ScanDrive rejected;
  rejected.initial_pose = InitialPoseSpec{};
  rejected.make_cloud = [](const builtin_interfaces::msg::Time & stamp) {
    return make_near_field_scan(stamp);
  };
  const auto advanced = harness->drive_one_scan(rejected);
  ASSERT_TRUE(advanced.has_value());
  ASSERT_GT(advanced->diag.value_as_double("skipping_publish_num"), 0.0)
    << "the near-field scan did not advance the counter, so the reset below proves nothing";

  // Act
  ScanDrive drive;
  drive.initial_pose = InitialPoseSpec{};

  const auto outcome = harness->drive_one_scan(drive);

  // Assert
  ASSERT_TRUE(outcome.has_value());
  ASSERT_EQ(outcome->diag.level(), level_ok)
    << "scan did not converge: " << outcome->diag.message();
  EXPECT_EQ(outcome->diag.value("skipping_publish_num"), "0")
    << "a converged scan no longer resets the skip counter";
}

/// SUSPICIOUS — an estimated covariance overwrites exactly four of the thirty-six entries.
///
/// The published covariance is the *parameter* matrix rotated into the map frame, with only indices
/// 0, 7, 1 and 6 replaced by the estimate. Those four hand-written indices beg to be folded into a
/// loop, and the `[1] <- (1,0)` / `[6] <- (0,1)` pairing invites a transpose slip. Getting it wrong
/// mis-weights NDT in the EKF forever and fires no diagnostic.
///
/// The untouched entries are exact for two separate reasons: `rotate_covariance` only rotates the
/// 3x3 position block, so 21, 28 and 35 pass through verbatim, and that block is isotropic here, so
/// rotating it is an identity operation and 14 does too.
///
/// `scale_factor` is far above 1 on purpose. `adjust_diagonal_covariance` floors the diagonal at
/// the parameter variance, and at the default scale the estimate for this scene lands below it --
/// the floor wins and "written" becomes indistinguishable from "never written". Measured, this
/// scale puts the diagonal near 18: 800x the floor, 80x the bound below. A failure there means
/// re-measuring, not relaxing the bound.
///
/// It does *not* pin the multiply-then-floor order: swapping them keeps every case green, because
/// flooring first still gives `0.0225 * scale_factor`. Catching that needs an upper bound on a
/// published variance, which is an NDT number.
TEST(NdtScanMatcherCharacteristics, EstimatedCovarianceOverwritesOnlyFourOfThirtySixEntries)
{
  // Arrange
  constexpr double scale_factor = 1.0e6;

  auto overrides = converged_hot_path_overrides();
  // LAPLACE_APPROXIMATION: an estimate that needs no extra alignments.
  overrides.emplace_back("covariance.covariance_estimation.covariance_estimation_type", 1);
  overrides.emplace_back("covariance.covariance_estimation.scale_factor", scale_factor);
  auto harness = make_ready_harness(std::move(overrides));

  auto ndt_pose_with_cov =
    harness->capture<geometry_msgs::msg::PoseWithCovarianceStamped>("/ndt_pose_with_covariance");

  ASSERT_TRUE(harness->ensure_map_loaded()) << "the stub map never loaded";

  // Act
  ScanDrive drive;
  drive.initial_pose = InitialPoseSpec{};

  const auto outcome = harness->drive_one_scan(drive);

  // Assert
  ASSERT_TRUE(outcome.has_value());
  ASSERT_EQ(outcome->diag.level(), level_ok)
    << "scan did not converge: " << outcome->diag.message();

  ASSERT_TRUE(harness->wait_until([&] { return ndt_pose_with_cov->count() >= 1; }, 5s));
  const auto published = ndt_pose_with_cov->first();
  ASSERT_TRUE(published.has_value());
  const auto & covariance = published->pose.covariance;

  constexpr double tolerance = 1e-12;

  // Untouched by both the rotation and the four-index overwrite.
  EXPECT_NEAR(covariance[14], param_variance_xyz, tolerance) << "z variance was overwritten";
  EXPECT_NEAR(covariance[21], param_variance_angular, tolerance) << "roll variance was overwritten";
  EXPECT_NEAR(covariance[28], param_variance_angular, tolerance)
    << "pitch variance was overwritten";
  EXPECT_NEAR(covariance[35], param_variance_angular, tolerance) << "yaw variance was overwritten";

  // Overwritten by the scaled estimate, which `scale_factor` puts well clear of the floor. If the
  // estimation branch were skipped these would still read exactly `param_variance_xyz`.
  EXPECT_GT(covariance[0], param_variance_xyz * 10.0)
    << "the x variance still looks like the parameter value; was the estimate written at all?";
  EXPECT_GT(covariance[7], param_variance_xyz * 10.0)
    << "the y variance still looks like the parameter value; was the estimate written at all?";
  // The magnitude first, because symmetry alone cannot see the mutation that a loop conversion
  // actually produces: dropping *both* off-diagonal writes leaves the two entries equal, at the
  // ~1e-27 dust the rotation leaves behind, and the sweep below skips indices 1 and 6. Measured for
  // this scene the estimate is about -0.04, so this floor sits 21 orders of magnitude above the
  // dust and 4 to 5 below the estimate -- clear of both, without asserting an NDT number.
  EXPECT_GT(std::abs(covariance[1]), 1.0e-6) << "the xy cross terms were never written";
  // Then symmetry, which catches one of the two writes being dropped. It cannot catch a
  // transposition: the Laplace estimate is symmetric, so `(0,1)` and `(1,0)` are equal at the
  // source. Only a unit test on the extracted function can feed it an asymmetric input.
  EXPECT_NEAR(covariance[1], covariance[6], std::abs(covariance[1]) * 1e-9 + tolerance)
    << "only one of covariance[1] / covariance[6] was written";

  // Every other entry stays zero: the parameter matrix is diagonal and the estimate only reaches
  // the four indices above.
  for (size_t i = 0; i < 36; ++i) {
    if (i == 0 || i == 1 || i == 6 || i == 7 || i == 14 || i == 21 || i == 28 || i == 35) {
      continue;
    }
    EXPECT_NEAR(covariance[i], 0.0, 1e-12) << "unexpected non-zero at covariance[" << i << "]";
  }
}

/// The published initial pose carries the *older* pose's covariance, never an interpolated one.
///
/// `SmartPoseBuffer::interpolate` interpolates the position and then assigns
/// `old_pose.covariance` verbatim, with a comment saying so. Anyone reimplementing interpolation
/// will interpolate the covariance too, because that is the obvious thing to do, and nothing else
/// observes the difference.
TEST(NdtScanMatcherCharacteristics, PublishedInitialPoseCarriesOldPoseCovarianceNotInterpolated)
{
  // Arrange
  auto harness = make_ready_harness(converged_hot_path_overrides());

  auto initial_pose_with_cov = harness->capture<geometry_msgs::msg::PoseWithCovarianceStamped>(
    "/initial_pose_with_covariance");

  ASSERT_TRUE(harness->ensure_map_loaded()) << "the stub map never loaded";

  // Neither value is `make_pose_at`'s default, which `ensure_map_loaded` above has already put into
  // the buffer. Reusing the default here would let the assertion pass on the wrong pose's
  // covariance.
  constexpr double older_pose_variance = 0.09;
  constexpr double newer_pose_variance = 4.0;
  constexpr double newer_pose_delta_x = 2.0;

  ScanDrive drive;
  InitialPoseSpec spec;
  spec.delta_x = newer_pose_delta_x;  // so the interpolated position differs from both endpoints
  spec.old_variance_xy = older_pose_variance;
  spec.new_variance_xy = newer_pose_variance;
  drive.initial_pose = spec;

  // Act
  const auto outcome = harness->drive_one_scan(drive);

  // Assert
  ASSERT_TRUE(outcome.has_value());
  ASSERT_EQ(outcome->diag.value("is_succeed_interpolate_initial_pose"), "True");
  // Convergence is asserted even though this case is about the covariance, because the
  // non-converged case's cleanup reasoning depends on it: every converged-path case except that one
  // resets the process-global skip counter by matching successfully. Left implicit, this case could
  // stop converging and start leaving the counter advanced without saying so.
  ASSERT_EQ(outcome->diag.level(), level_ok)
    << "scan did not converge: " << outcome->diag.message();

  ASSERT_TRUE(harness->wait_until([&] { return initial_pose_with_cov->count() >= 1; }, 5s));
  const auto published = initial_pose_with_cov->first();
  ASSERT_TRUE(published.has_value());
  const auto & interpolated = *published;

  EXPECT_DOUBLE_EQ(interpolated.pose.covariance[0], older_pose_variance)
    << "the covariance is now interpolated instead of copied from the older pose";
  EXPECT_DOUBLE_EQ(interpolated.pose.covariance[7], older_pose_variance);

  // The position *is* interpolated: the scan stamp sits exactly between the two poses.
  EXPECT_GT(interpolated.pose.pose.position.x, map_center_x);
  EXPECT_LT(interpolated.pose.pose.position.x, map_center_x + newer_pose_delta_x);
  EXPECT_NEAR(interpolated.pose.pose.position.x, map_center_x + newer_pose_delta_x / 2.0, 1e-6);
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
