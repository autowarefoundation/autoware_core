// Copyright 2026 The Autoware Contributors
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

#include "../src/gnss_module.hpp"
#include "../src/localization_module.hpp"
#include "../src/localization_trigger_module.hpp"
#include "../src/pose_error_check_module.hpp"
#include "../src/pose_initializer_core.hpp"
#include "../src/stop_check_module.hpp"

#include <rclcpp/rclcpp.hpp>

#include <autoware_adapi_v1_msgs/msg/localization_initialization_state.hpp>
#include <autoware_internal_localization_msgs/srv/pose_with_covariance_stamped.hpp>
#include <autoware_localization_msgs/srv/initialize_localization.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/twist_with_covariance_stamped.hpp>
#include <std_srvs/srv/set_bool.hpp>

#include <gtest/gtest.h>

#include <atomic>
#include <condition_variable>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

using autoware::pose_initializer::PoseInitializer;
using InitializeLocalization = autoware_localization_msgs::srv::InitializeLocalization;
using RequestPoseAlignment = autoware_internal_localization_msgs::srv::PoseWithCovarianceStamped;
using PoseWithCovarianceStamped = geometry_msgs::msg::PoseWithCovarianceStamped;
using TwistWithCovarianceStamped = geometry_msgs::msg::TwistWithCovarianceStamped;
using InitializationState = autoware_adapi_v1_msgs::msg::LocalizationInitializationState;
using SetBool = std_srvs::srv::SetBool;

namespace
{
// Floating point tolerance at EXPECT_NEAR and similar checks
constexpr float near_tol = 1e-2F;

// The node declares its parameters without defaults, so a test has to supply the whole set.
rclcpp::NodeOptions make_node_options(
  bool user_defined_initial_pose_enable = false,
  const std::vector<double> & user_defined_initial_pose = {0, 0, 0, 0, 0, 0, 1})
{
  rclcpp::NodeOptions options;
  options.append_parameter_override("ekf_enabled", true);
  options.append_parameter_override("gnss_enabled", true);
  options.append_parameter_override("ndt_enabled", true);
  options.append_parameter_override("yabloc_enabled", false);
  options.append_parameter_override("stop_check_enabled", true);
  options.append_parameter_override("stop_check_duration", 0.5);
  options.append_parameter_override("gnss_pose_timeout", 3.0);
  options.append_parameter_override("pose_error_check_enabled", true);
  options.append_parameter_override("pose_error_threshold", 5.0);
  options.append_parameter_override(
    "user_defined_initial_pose.enable", user_defined_initial_pose_enable);
  options.append_parameter_override("map_height_fitter.target", "vector_map");
  options.append_parameter_override("map_height_fitter.map_loader_name", "/map/vector_map_loader");
  options.append_parameter_override("user_defined_initial_pose.pose", user_defined_initial_pose);

  const std::vector<double> cov(36, 0.01);  // Satisfy the 36-element array requirement
  options.append_parameter_override("output_pose_covariance", cov);
  options.append_parameter_override("gnss_particle_covariance", cov);
  return options;
}
}  // namespace

class PoseInitializerNodeIntegrationTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    // rclcpp init once for whole test binary via RosEnv below.

    node_ = std::make_shared<PoseInitializer>(make_node_options());
    harness_ = std::make_shared<rclcpp::Node>("test_harness");

    // Harness pub/sub
    pub_gnss_ = harness_->create_publisher<PoseWithCovarianceStamped>("gnss_pose_cov", 1);
    pub_twist_ = harness_->create_publisher<TwistWithCovarianceStamped>("stop_check_twist", 1);

    sub_reset_ = harness_->create_subscription<PoseWithCovarianceStamped>(
      "pose_reset", 1, [this](PoseWithCovarianceStamped::ConstSharedPtr msg) {
        {
          std::lock_guard<std::mutex> lk(reset_mtx_);
          last_reset_pose_ = msg;
          got_reset_ = true;
        }
        reset_cv_.notify_one();
      });
    sub_state_ = harness_->create_subscription<InitializationState>(
      "/localization/initialization_state", 10,
      [this](InitializationState::ConstSharedPtr msg) { last_state_ = msg; });

    // Harness service mocks
    srv_ndt_align_ = harness_->create_service<RequestPoseAlignment>(
      "ndt_align", [this](
                     const std::shared_ptr<RequestPoseAlignment::Request> req,
                     std::shared_ptr<RequestPoseAlignment::Response> res) {
        res->success = mock_align_success_;
        res->pose_with_covariance = req->pose_with_covariance;
        res->pose_with_covariance.pose.pose.position.x += 1.0;  // Simulate an alignment shift
      });

    auto trigger_callback = [this](
                              const std::shared_ptr<SetBool::Request> /*req*/,
                              std::shared_ptr<SetBool::Response> res) {
      trigger_calls_++;
      res->success = mock_trigger_success_;
    };
    srv_ekf_trigger_ = harness_->create_service<SetBool>("ekf_trigger_node", trigger_callback);
    srv_ndt_trigger_ = harness_->create_service<SetBool>("ndt_trigger_node", trigger_callback);

    // Force executor to use 8 threads to prevent future.get() deadlocks
    exec_ =
      std::make_shared<rclcpp::executors::MultiThreadedExecutor>(rclcpp::ExecutorOptions(), 8);
    exec_->add_node(node_);
    exec_->add_node(harness_);
    exec_thread_ = std::thread([this]() { exec_->spin(); });

    // Allow DDS discovery to fully register mock services
    std::this_thread::sleep_for(std::chrono::milliseconds(200));

    // Ensure trigger services are discoverable to avoid races where node
    // blocks waiting for them inside its service callback. Wait up to 2s.
    auto cli_ekf = harness_->create_client<SetBool>("ekf_trigger_node");
    auto cli_ndt = harness_->create_client<SetBool>("ndt_trigger_node");
    ASSERT_TRUE(cli_ekf->wait_for_service(std::chrono::seconds(2)));
    ASSERT_TRUE(cli_ndt->wait_for_service(std::chrono::seconds(2)));

    // Reset notification state
    {
      std::lock_guard<std::mutex> lk(reset_mtx_);
      got_reset_ = false;
      last_reset_pose_ = nullptr;
    }
  }

  void TearDown() override
  {
    // Cancel and join executor thread before shutdown. Global RosEnv handles
    // rclcpp::shutdown() at process exit.
    if (exec_) {
      exec_->cancel();
    }
    if (exec_thread_.joinable()) {
      exec_thread_.join();
    }
  }

  // Helper to publish stopped twist data for a while
  // Simulates stationary ego
  void simulate_vehicle_stopped(double duration_sec)
  {
    const rclcpp::Time start_time = harness_->now();
    rclcpp::Rate rate(10);  // 10 Hz
    while ((harness_->now() - start_time).seconds() < duration_sec) {
      TwistWithCovarianceStamped twist;
      twist.header.stamp = harness_->now();
      twist.header.frame_id = "base_link";
      twist.twist.twist.linear.x = 0.0;  // Stationary
      pub_twist_->publish(twist);
      rate.sleep();
    }
  }

  std::shared_ptr<PoseInitializer> node_;
  std::shared_ptr<rclcpp::Node> harness_;
  std::shared_ptr<rclcpp::executors::MultiThreadedExecutor> exec_;
  std::thread exec_thread_;

  rclcpp::Publisher<PoseWithCovarianceStamped>::SharedPtr pub_gnss_;
  rclcpp::Publisher<TwistWithCovarianceStamped>::SharedPtr pub_twist_;
  rclcpp::Subscription<PoseWithCovarianceStamped>::SharedPtr sub_reset_;
  rclcpp::Subscription<InitializationState>::SharedPtr sub_state_;

  rclcpp::Service<RequestPoseAlignment>::SharedPtr srv_ndt_align_;
  rclcpp::Service<SetBool>::SharedPtr srv_ekf_trigger_;
  rclcpp::Service<SetBool>::SharedPtr srv_ndt_trigger_;

  PoseWithCovarianceStamped::ConstSharedPtr last_reset_pose_ = nullptr;
  InitializationState::ConstSharedPtr last_state_ = nullptr;

  std::atomic<bool> mock_align_success_{true};
  std::atomic<bool> mock_trigger_success_{true};
  std::atomic<int> trigger_calls_{0};

  // Synchronization for receiving reset poses deterministically in tests
  std::mutex reset_mtx_;
  std::condition_variable reset_cv_;
  bool got_reset_{false};

  // Helper to publish GNSS and handle DDS delay
  void publish_gnss_pose(double time_offset_sec = 0.0, double x_pos = 0.0)
  {
    PoseWithCovarianceStamped msg;
    msg.header.stamp = harness_->now() + rclcpp::Duration::from_seconds(time_offset_sec);
    msg.header.frame_id = "map";
    msg.pose.pose.position.x = x_pos;
    pub_gnss_->publish(msg);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }

  // Helper to do lots of stuffs one shot
  InitializeLocalization::Response::SharedPtr call_init_service(
    InitializeLocalization::Request::SharedPtr req)
  {
    auto cli_init = harness_->create_client<InitializeLocalization>("/localization/initialize");
    EXPECT_TRUE(cli_init->wait_for_service(std::chrono::seconds(2)));

    simulate_vehicle_stopped(0.5);

    auto future = cli_init->async_send_request(req);
    EXPECT_EQ(future.wait_for(std::chrono::seconds(2)), std::future_status::ready);
    return future.get();
  }

  // Wait for a reset pose to arrive (published to `pose_reset`) within timeout.
  bool wait_for_reset(std::chrono::milliseconds timeout = std::chrono::milliseconds(100))
  {
    std::unique_lock<std::mutex> lk(reset_mtx_);
    return reset_cv_.wait_for(lk, timeout, [this]() { return got_reset_; });
  }
};

// ================================ TESTING AREA HERE ================================

// TEST 1. Confirms that sending DIRECT req will:
// - Bypass NDT and EKF aligners.
// - Publish req pose directly to node.
// - Toggles localizer trigger (4 calls).
TEST_F(PoseInitializerNodeIntegrationTest, DirectInitBypassAligners)
{
  auto req = std::make_shared<InitializeLocalization::Request>();
  req->method = InitializeLocalization::Request::DIRECT;

  PoseWithCovarianceStamped initial_pose;
  initial_pose.header.frame_id = "map";
  initial_pose.pose.pose.position.x = 100.0;
  initial_pose.pose.pose.orientation.w = 1.0;
  req->pose_with_covariance.push_back(initial_pose);

  auto res = call_init_service(req);

  EXPECT_TRUE(res->status.success);

  // Wait for pub to register output
  ASSERT_TRUE(wait_for_reset(std::chrono::milliseconds(500))) << "Timed out waiting for reset pose";
  ASSERT_NE(last_reset_pose_, nullptr) << "Failed to publish reset pose!";
  EXPECT_DOUBLE_EQ(last_reset_pose_->pose.pose.position.x, 100.0);

  // Expect trigger calls
  // (deactivate + activate) x 2 for both EKF and NDT = 4 calls
  EXPECT_EQ(trigger_calls_.load(), 4);
}

// TEST 2. Confirms missing pose arrays will fail fast with error message.
TEST_F(PoseInitializerNodeIntegrationTest, DirectInitEmptyPoseFailsFast)
{
  auto req = std::make_shared<InitializeLocalization::Request>();
  req->method = InitializeLocalization::Request::DIRECT;

  auto res = call_init_service(req);

  EXPECT_FALSE(res->status.success);
  EXPECT_TRUE(res->status.message.find("No input pose_with_covariance") != std::string::npos);
}

// TEST 3. Confirms unknown method ID will fail fast with error message.
TEST_F(PoseInitializerNodeIntegrationTest, UnknownMethodFailsFast)
{
  auto req = std::make_shared<InitializeLocalization::Request>();
  req->method = 99;  // Invalid method ID

  auto res = call_init_service(req);

  EXPECT_FALSE(res->status.success);
  EXPECT_TRUE(res->status.message.find("Unknown method type") != std::string::npos);
}

// TEST 4. Confirms AUTO init without GNSS will fail fast with error message.
TEST_F(PoseInitializerNodeIntegrationTest, AutoInitNoGnssFailsFast)
{
  auto req = std::make_shared<InitializeLocalization::Request>();
  req->method = InitializeLocalization::Request::AUTO;

  auto res = call_init_service(req);

  EXPECT_FALSE(res->status.success);
  EXPECT_EQ(res->status.message, "The GNSS pose has not arrived.");
}

// TEST 5. Confirms AUTO init with stale GNSS (lagged 5 sec > 3 sec) will fail fast with error
// message.
TEST_F(PoseInitializerNodeIntegrationTest, AutoInitStaleGnssFailsFast)
{
  // Publish GNSS pose with 5 sec old (timeout = 3 sec)
  publish_gnss_pose(-5.0);

  auto req = std::make_shared<InitializeLocalization::Request>();
  req->method = InitializeLocalization::Request::AUTO;

  auto res = call_init_service(req);

  EXPECT_FALSE(res->status.success);
  EXPECT_EQ(res->status.message, "The GNSS pose is out of date.");
}

// TEST 6. Confirms AUTO init with NDT alignment failure will return error.
TEST_F(PoseInitializerNodeIntegrationTest, AutoInitNdtAlignFailsReturnsEstError)
{
  mock_align_success_ = false;  // Force mock NDT server to fail

  publish_gnss_pose();

  auto req = std::make_shared<InitializeLocalization::Request>();
  req->method = InitializeLocalization::Request::AUTO;

  auto res = call_init_service(req);

  EXPECT_FALSE(res->status.success);
  EXPECT_EQ(res->status.message, "align server failed.");
}

// TEST 7. Confirms AUTO init with trigger failure will return error.
TEST_F(PoseInitializerNodeIntegrationTest, AutoInitTriggerFailsReturnsEstError)
{
  mock_trigger_success_ = false;  // Force mock trigger to fail

  publish_gnss_pose();

  auto req = std::make_shared<InitializeLocalization::Request>();
  req->method = InitializeLocalization::Request::AUTO;

  auto res = call_init_service(req);

  EXPECT_FALSE(res->status.success);
  EXPECT_TRUE(res->status.message.find("failed") != std::string::npos);
}

// TEST 8. Confirms AUTO init with large pose error will still succeed but issue a warning.
// For details please see comments inside this func.
TEST_F(PoseInitializerNodeIntegrationTest, AutoInitLargePoseErrSucceedsWithWarn)
{
  // 1. Publish GNSS at X = 0
  publish_gnss_pose();

  auto req = std::make_shared<InitializeLocalization::Request>();
  req->method = InitializeLocalization::Request::AUTO;

  // 2. Pass request pose at X = 10.
  // Node gonna use this instead of GNSS for alignment.
  // Mock aligns it to X = 11.
  // Error check compares X = 11 against GNSS(X = 0).
  // Distance 11.0 > 5.0m threshold.
  // Issues warn, still returns success.
  PoseWithCovarianceStamped req_pose;
  req_pose.header.frame_id = "map";
  req_pose.pose.pose.position.x = 10.0;
  req->pose_with_covariance.push_back(req_pose);

  auto res = call_init_service(req);

  // Still succeed but there will be warning
  EXPECT_TRUE(res->status.success);

  // Wait for reset pose to be published and received
  ASSERT_TRUE(wait_for_reset(std::chrono::milliseconds(500))) << "Timed out waiting for reset pose";
  // Reset pose is now aligned pose (X = 11.0)
  ASSERT_NE(last_reset_pose_, nullptr);
  EXPECT_NEAR(last_reset_pose_->pose.pose.position.x, 11.0, near_tol);
}

// ============================ USER DEFINED INITIAL POSE HERE ============================

// Fixture for the startup path, where the node initializes itself from
// `user_defined_initial_pose` instead of from an Initialize request.
class PoseInitializerUserDefinedInitialPoseTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    harness_ = std::make_shared<rclcpp::Node>("test_harness");

    // Harness sub
    sub_reset_ = harness_->create_subscription<PoseWithCovarianceStamped>(
      "pose_reset", 1, [this](PoseWithCovarianceStamped::ConstSharedPtr msg) {
        {
          std::lock_guard<std::mutex> lk(startup_mtx_);
          last_reset_pose_ = msg;
        }
        startup_cv_.notify_one();
      });
    sub_state_ = harness_->create_subscription<InitializationState>(
      "/localization/initialization_state", 10, [this](InitializationState::ConstSharedPtr msg) {
        {
          std::lock_guard<std::mutex> lk(startup_mtx_);
          state_history_.push_back(msg->state);
        }
        startup_cv_.notify_one();
      });

    // Harness service mocks
    auto trigger_callback = [this](
                              const std::shared_ptr<SetBool::Request> /*req*/,
                              std::shared_ptr<SetBool::Response> res) {
      trigger_calls_++;
      res->success = mock_trigger_success_;
    };
    srv_ekf_trigger_ = harness_->create_service<SetBool>("ekf_trigger_node", trigger_callback);
    srv_ndt_trigger_ = harness_->create_service<SetBool>("ndt_trigger_node", trigger_callback);
  }

  void TearDown() override
  {
    if (exec_) {
      exec_->cancel();
    }
    if (exec_thread_.joinable()) {
      exec_thread_.join();
    }
  }

  void start_node(const std::vector<double> & initial_pose)
  {
    node_ = std::make_shared<PoseInitializer>(make_node_options(true, initial_pose));

    // `pose_reset` is volatile and the node publishes on it milliseconds into the spin, so the
    // harness has to be matched before then or the sample is dropped.
    ASSERT_TRUE(wait_for_matched(std::chrono::seconds(2))) << "Timed out matching the node";

    exec_ =
      std::make_shared<rclcpp::executors::MultiThreadedExecutor>(rclcpp::ExecutorOptions(), 8);
    exec_->add_node(node_);
    exec_->add_node(harness_);
    exec_thread_ = std::thread([this]() { exec_->spin(); });
  }

  // Endpoint matching happens in the middleware, so this works before the executor is spinning.
  bool wait_for_matched(std::chrono::milliseconds timeout)
  {
    const auto deadline = std::chrono::steady_clock::now() + timeout;
    while (std::chrono::steady_clock::now() < deadline) {
      if (sub_reset_->get_publisher_count() > 0 && sub_state_->get_publisher_count() > 0) {
        return true;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    return false;
  }

  std::shared_ptr<PoseInitializer> node_;
  std::shared_ptr<rclcpp::Node> harness_;
  std::shared_ptr<rclcpp::executors::MultiThreadedExecutor> exec_;
  std::thread exec_thread_;

  rclcpp::Subscription<PoseWithCovarianceStamped>::SharedPtr sub_reset_;
  rclcpp::Subscription<InitializationState>::SharedPtr sub_state_;

  rclcpp::Service<SetBool>::SharedPtr srv_ekf_trigger_;
  rclcpp::Service<SetBool>::SharedPtr srv_ndt_trigger_;

  PoseWithCovarianceStamped::ConstSharedPtr last_reset_pose_ = nullptr;

  // The whole sequence, because the constructor publishes UNINITIALIZED before the startup path
  // runs and the failure case ends on that same value.
  std::vector<InitializationState::_state_type> state_history_;

  std::atomic<bool> mock_trigger_success_{true};
  std::atomic<int> trigger_calls_{0};

  // Guards last_reset_pose_ and state_history_
  std::mutex startup_mtx_;
  std::condition_variable startup_cv_;

  template <typename Predicate>
  bool wait_for(Predicate predicate, std::chrono::milliseconds timeout)
  {
    std::unique_lock<std::mutex> lk(startup_mtx_);
    return startup_cv_.wait_for(lk, timeout, predicate);
  }
};

// TEST 9. Confirms that enabling user_defined_initial_pose will:
// - Publish the configured pose on startup, with no init req involved.
// - Toggle both localizer triggers (4 calls).
// - Reach INITIALIZED state.
TEST_F(PoseInitializerUserDefinedInitialPoseTest, UserDefinedInitOnStartupSucceeds)
{
  start_node({1.0, 2.0, 3.0, 0.0, 0.0, 0.0, 1.0});

  ASSERT_TRUE(wait_for(
    [this]() {
      return last_reset_pose_ && !state_history_.empty() &&
             state_history_.back() == InitializationState::INITIALIZED;
    },
    std::chrono::seconds(2)))
    << "Timed out waiting for startup initialization";

  std::lock_guard<std::mutex> lk(startup_mtx_);
  EXPECT_EQ(last_reset_pose_->header.frame_id, "map");
  EXPECT_DOUBLE_EQ(last_reset_pose_->pose.pose.position.x, 1.0);
  EXPECT_DOUBLE_EQ(last_reset_pose_->pose.pose.position.y, 2.0);
  EXPECT_DOUBLE_EQ(last_reset_pose_->pose.pose.position.z, 3.0);
  EXPECT_DOUBLE_EQ(last_reset_pose_->pose.pose.orientation.w, 1.0);

  // (deactivate + activate) x 2 for both EKF and NDT = 4 calls
  EXPECT_EQ(trigger_calls_.load(), 4);
}

// TEST 10. Confirms that a trigger failure during startup falls back to UNINITIALIZED without
// publishing a pose, and that the exception does not escape the timer callback.
TEST_F(PoseInitializerUserDefinedInitialPoseTest, UserDefinedInitTriggerFailsStaysUninitialized)
{
  mock_trigger_success_ = false;

  start_node({1.0, 2.0, 3.0, 0.0, 0.0, 0.0, 1.0});

  ASSERT_TRUE(wait_for(
    [this]() {
      const auto size = state_history_.size();
      return size >= 2 && state_history_[size - 2] == InitializationState::INITIALIZING &&
             state_history_[size - 1] == InitializationState::UNINITIALIZED;
    },
    std::chrono::seconds(2)))
    << "Timed out waiting for the node to fall back to UNINITIALIZED";

  std::lock_guard<std::mutex> lk(startup_mtx_);
  EXPECT_EQ(last_reset_pose_, nullptr) << "Published a reset pose despite the failure!";

  // Gives up on the first deactivation, so NDT is never asked
  EXPECT_EQ(trigger_calls_.load(), 1);
}

// TEST 11. Confirms that a pose of the wrong size is rejected at construction.
TEST_F(PoseInitializerUserDefinedInitialPoseTest, RejectsPoseOfWrongSize)
{
  EXPECT_THROW(
    std::make_shared<PoseInitializer>(make_node_options(true, {1.0, 2.0, 3.0})),
    std::invalid_argument);
}

// TEST 12. Confirms that a zero quaternion is rejected at construction.
TEST_F(PoseInitializerUserDefinedInitialPoseTest, RejectsZeroQuaternion)
{
  EXPECT_THROW(
    std::make_shared<PoseInitializer>(make_node_options(true, {1.0, 2.0, 3.0, 0.0, 0.0, 0.0, 0.0})),
    std::invalid_argument);
}

// Initialize/shutdown rclcpp once for entire test binary to avoid repeated
// init/shutdown races across tests.
class RosEnv : public ::testing::Environment
{
public:
  void SetUp() override { rclcpp::init(0, nullptr); }
  void TearDown() override
  {
    if (rclcpp::ok()) rclcpp::shutdown();
  }
};

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  ::testing::AddGlobalTestEnvironment(new RosEnv());
  return RUN_ALL_TESTS();
}
