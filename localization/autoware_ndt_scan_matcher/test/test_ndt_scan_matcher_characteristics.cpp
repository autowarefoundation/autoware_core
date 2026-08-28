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

#include <rclcpp/rclcpp.hpp>

#include <diagnostic_msgs/msg/diagnostic_status.hpp>

#include <gtest/gtest.h>

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

using ndt_test::base_link_frame;
using ndt_test::map_center_x;
using ndt_test::map_center_y;
using ndt_test::map_frame;

using ndt_test::initial_pose_status;
using ndt_test::map_update_status;
using ndt_test::ndt_align_status;
using ndt_test::scan_matching_status;

using ndt_test::make_empty_scan;
using ndt_test::make_near_field_scan;
using ndt_test::make_pose_at;

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
}  // namespace

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  ::testing::InitGoogleTest(&argc, argv);
  const int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
