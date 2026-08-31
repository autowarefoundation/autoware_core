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

#include "gyro_odometer_node.hpp"

#include <builtin_interfaces/msg/time.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/static_transform_broadcaster.hpp>

#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <geometry_msgs/msg/twist_with_covariance_stamped.hpp>
#include <rosgraph_msgs/msg/clock.hpp>
#include <sensor_msgs/msg/imu.hpp>

#include <gtest/gtest.h>

#include <chrono>
#include <functional>
#include <map>
#include <memory>
#include <optional>
#include <string>

namespace autoware::gyro_odometer
{
namespace
{

using diagnostic_msgs::msg::DiagnosticArray;
using geometry_msgs::msg::TwistStamped;
using geometry_msgs::msg::TwistWithCovarianceStamped;
using sensor_msgs::msg::Imu;
using COV_IDX_XYZ = autoware_utils_geometry::xyz_covariance_index::XYZ_COV_IDX;
using COV_IDX_XYZRPY = autoware_utils_geometry::xyzrpy_covariance_index::XYZRPY_COV_IDX;

// The diagnostics timer period. Advancing the simulated clock by this much is what makes the node
// emit one diagnostics message.
constexpr std::chrono::milliseconds diagnostics_period{100};
// Wall-clock budget for the loopback delivery of a single published message. Only one message is
// ever in flight, so this only has to cover the delivery itself.
constexpr std::chrono::milliseconds delivery_budget{50};
// Wall-clock budget for anything the test actively waits on.
constexpr std::chrono::milliseconds wait_budget{2000};

// ---------------------------------------------------------------------------
// Input builders
//
// These describe the messages only, with no notion of how they reach the fusion. A suite that
// drives the fusion directly instead of over topics can build its inputs with the same calls.
// ---------------------------------------------------------------------------

builtin_interfaces::msg::Time make_stamp(int32_t sec, uint32_t nanosec)
{
  builtin_interfaces::msg::Time stamp;
  stamp.sec = sec;
  stamp.nanosec = nanosec;
  return stamp;
}

Imu make_imu(
  const builtin_interfaces::msg::Time & stamp, const std::string & frame_id, double wx, double wy,
  double wz, double cov_xx, double cov_yy, double cov_zz)
{
  Imu imu;
  imu.header.stamp = stamp;
  imu.header.frame_id = frame_id;
  imu.angular_velocity.x = wx;
  imu.angular_velocity.y = wy;
  imu.angular_velocity.z = wz;
  imu.angular_velocity_covariance[COV_IDX_XYZ::X_X] = cov_xx;
  imu.angular_velocity_covariance[COV_IDX_XYZ::Y_Y] = cov_yy;
  imu.angular_velocity_covariance[COV_IDX_XYZ::Z_Z] = cov_zz;
  return imu;
}

TwistWithCovarianceStamped make_vehicle_twist(
  const builtin_interfaces::msg::Time & stamp, double vx, double cov_xx)
{
  TwistWithCovarianceStamped twist;
  twist.header.stamp = stamp;
  twist.header.frame_id = "base_link";
  twist.twist.twist.linear.x = vx;
  twist.twist.covariance[COV_IDX_XYZRPY::X_X] = cov_xx;
  return twist;
}

// ---------------------------------------------------------------------------
// Observations
//
// FusedOutput mirrors the four messages one fusion produces. DiagnosticsSnapshot keeps the reported
// values as the strings they are put on the wire as, so that a test parses only the entries it
// actually pins -- which matters because some of them are read before ever being assigned.
// ---------------------------------------------------------------------------

struct FusedOutput
{
  TwistStamped twist_raw;
  TwistWithCovarianceStamped twist_with_covariance_raw;
  TwistStamped twist;
  TwistWithCovarianceStamped twist_with_covariance;
};

struct DiagnosticsSnapshot
{
  std::map<std::string, std::string> values;
  int8_t level{0};
  std::string message;
};

bool reported_flag(const DiagnosticsSnapshot & snapshot, const std::string & key)
{
  const auto it = snapshot.values.find(key);
  EXPECT_NE(it, snapshot.values.end()) << "diagnostics has no entry named " << key;
  if (it == snapshot.values.end()) {
    return false;
  }
  return it->second == "True";
}

int32_t reported_int(const DiagnosticsSnapshot & snapshot, const std::string & key)
{
  const auto it = snapshot.values.find(key);
  EXPECT_NE(it, snapshot.values.end()) << "diagnostics has no entry named " << key;
  if (it == snapshot.values.end()) {
    return 0;
  }
  return static_cast<int32_t>(std::stol(it->second));
}

// Drives the node over its real topics while keeping every step of the scenario ordered.
//
// There is no background spin: the test thread itself pumps the executor, and only ever one
// message is in flight, so the node cannot observe the two input topics in an order the scenario
// did not ask for. Time is simulated and only moves when the scenario moves it, which is also what
// makes the diagnostics timer fire at a point the scenario chooses.
class GyroOdometerNodeCharacterization : public ::testing::Test
{
protected:
  void SetUp() override
  {
    rclcpp::init(0, nullptr);

    executor_ = std::make_unique<rclcpp::executors::SingleThreadedExecutor>();
    driver_node_ = std::make_shared<rclcpp::Node>("gyro_odometer_characterization_driver");

    clock_pub_ =
      driver_node_->create_publisher<rosgraph_msgs::msg::Clock>("/clock", rclcpp::ClockQoS());
    imu_pub_ = driver_node_->create_publisher<Imu>("imu", rclcpp::QoS{10});
    vehicle_twist_pub_ = driver_node_->create_publisher<TwistWithCovarianceStamped>(
      "vehicle/twist_with_covariance", rclcpp::QoS{10});

    twist_raw_sub_ = driver_node_->create_subscription<TwistStamped>(
      "twist_raw", rclcpp::QoS{10},
      [this](const TwistStamped::SharedPtr msg) { twist_raw_ = *msg; });
    twist_with_covariance_raw_sub_ = driver_node_->create_subscription<TwistWithCovarianceStamped>(
      "twist_with_covariance_raw", rclcpp::QoS{10},
      [this](const TwistWithCovarianceStamped::SharedPtr msg) {
        twist_with_covariance_raw_ = *msg;
      });
    twist_sub_ = driver_node_->create_subscription<TwistStamped>(
      "twist", rclcpp::QoS{10}, [this](const TwistStamped::SharedPtr msg) { twist_ = *msg; });
    twist_with_covariance_sub_ = driver_node_->create_subscription<TwistWithCovarianceStamped>(
      "twist_with_covariance", rclcpp::QoS{10},
      [this](const TwistWithCovarianceStamped::SharedPtr msg) { twist_with_covariance_ = *msg; });

    diagnostics_sub_ = driver_node_->create_subscription<DiagnosticArray>(
      "/diagnostics", rclcpp::QoS{10}, [this](const DiagnosticArray::SharedPtr msg) {
        for (const auto & status : msg->status) {
          if (status.name.find("gyro_odometer_status") == std::string::npos) {
            continue;
          }
          DiagnosticsSnapshot snapshot;
          snapshot.level = status.level;
          snapshot.message = status.message;
          for (const auto & value : status.values) {
            snapshot.values.emplace(value.key, value.value);
          }
          latest_diagnostics_ = snapshot;
          ++diagnostics_count_;
        }
      });

    static_transform_broadcaster_ =
      std::make_unique<tf2_ros::StaticTransformBroadcaster>(driver_node_);

    executor_->add_node(driver_node_);
  }

  void TearDown() override
  {
    if (gyro_odometer_node_) {
      executor_->remove_node(gyro_odometer_node_->get_node_base_interface());
    }
    executor_->remove_node(driver_node_);
    rclcpp::shutdown();
  }

  // Bring up the node under test and put the simulated clock at a known point.
  void start_node(const std::string & output_frame, double message_timeout_sec)
  {
    rclcpp::NodeOptions node_options;
    node_options.append_parameter_override("output_frame", output_frame);
    node_options.append_parameter_override("message_timeout_sec", message_timeout_sec);
    node_options.append_parameter_override("use_sim_time", true);

    gyro_odometer_node_ = std::make_shared<GyroOdometerNode>(node_options);
    executor_->add_node(gyro_odometer_node_->get_node_base_interface());

    set_now(rclcpp::Time(100, 0, RCL_ROS_TIME));
  }

  // Move the simulated clock and do not return until the node has taken it up.
  void set_now(const rclcpp::Time & now)
  {
    sim_now_ = now;

    rosgraph_msgs::msg::Clock clock_msg;
    clock_msg.clock = now;
    clock_pub_->publish(clock_msg);

    const bool taken_up = pump_until(
      [this, now]() { return gyro_odometer_node_->get_clock()->now() >= now; }, wait_budget);
    ASSERT_TRUE(taken_up) << "the node did not take up the simulated clock";
  }

  rclcpp::Time sim_now() const { return sim_now_; }

  void send_vehicle_twist(const TwistWithCovarianceStamped & vehicle_twist)
  {
    vehicle_twist_pub_->publish(vehicle_twist);
    pump(delivery_budget);
  }

  void send_imu(const Imu & imu)
  {
    imu_pub_->publish(imu);
    pump(delivery_budget);
  }

  // Make a frame pair resolvable, with the rotation the node will apply to the queued samples.
  void broadcast_static_transform(
    const std::string & frame_id, const std::string & child_frame_id, double qx, double qy,
    double qz, double qw)
  {
    geometry_msgs::msg::TransformStamped transform;
    transform.header.stamp = sim_now_;
    transform.header.frame_id = frame_id;
    transform.child_frame_id = child_frame_id;
    transform.transform.rotation.x = qx;
    transform.transform.rotation.y = qy;
    transform.transform.rotation.z = qz;
    transform.transform.rotation.w = qw;
    static_transform_broadcaster_->sendTransform(transform);
    pump(delivery_budget);
  }

  // Consume whatever fusion output has been received since the last call.
  std::optional<FusedOutput> take_output()
  {
    if (!twist_raw_ || !twist_with_covariance_raw_ || !twist_ || !twist_with_covariance_) {
      return std::nullopt;
    }
    const FusedOutput output{
      *twist_raw_, *twist_with_covariance_raw_, *twist_, *twist_with_covariance_};
    twist_raw_.reset();
    twist_with_covariance_raw_.reset();
    twist_.reset();
    twist_with_covariance_.reset();
    return output;
  }

  // Let the diagnostics timer fire once and return what it reported.
  DiagnosticsSnapshot take_diagnostics()
  {
    const uint64_t before = diagnostics_count_;
    set_now(sim_now_ + rclcpp::Duration(diagnostics_period));
    const bool reported =
      pump_until([this, before]() { return diagnostics_count_ > before; }, wait_budget);
    EXPECT_TRUE(reported) << "the node did not report diagnostics";
    return latest_diagnostics_;
  }

private:
  void pump(std::chrono::milliseconds duration)
  {
    const auto deadline = std::chrono::steady_clock::now() + duration;
    while (std::chrono::steady_clock::now() < deadline) {
      executor_->spin_some(std::chrono::milliseconds(1));
    }
  }

  bool pump_until(const std::function<bool()> & predicate, std::chrono::milliseconds timeout)
  {
    const auto deadline = std::chrono::steady_clock::now() + timeout;
    while (std::chrono::steady_clock::now() < deadline) {
      if (predicate()) {
        return true;
      }
      executor_->spin_some(std::chrono::milliseconds(1));
    }
    return predicate();
  }

  std::unique_ptr<rclcpp::executors::SingleThreadedExecutor> executor_;
  rclcpp::Node::SharedPtr driver_node_;
  std::shared_ptr<GyroOdometerNode> gyro_odometer_node_;

  rclcpp::Publisher<rosgraph_msgs::msg::Clock>::SharedPtr clock_pub_;
  rclcpp::Publisher<Imu>::SharedPtr imu_pub_;
  rclcpp::Publisher<TwistWithCovarianceStamped>::SharedPtr vehicle_twist_pub_;

  rclcpp::Subscription<TwistStamped>::SharedPtr twist_raw_sub_;
  rclcpp::Subscription<TwistWithCovarianceStamped>::SharedPtr twist_with_covariance_raw_sub_;
  rclcpp::Subscription<TwistStamped>::SharedPtr twist_sub_;
  rclcpp::Subscription<TwistWithCovarianceStamped>::SharedPtr twist_with_covariance_sub_;
  rclcpp::Subscription<DiagnosticArray>::SharedPtr diagnostics_sub_;
  std::unique_ptr<tf2_ros::StaticTransformBroadcaster> static_transform_broadcaster_;

  std::optional<TwistStamped> twist_raw_;
  std::optional<TwistWithCovarianceStamped> twist_with_covariance_raw_;
  std::optional<TwistStamped> twist_;
  std::optional<TwistWithCovarianceStamped> twist_with_covariance_;

  DiagnosticsSnapshot latest_diagnostics_;
  uint64_t diagnostics_count_{0};

  rclcpp::Time sim_now_{0, 0, RCL_ROS_TIME};
};

}  // namespace

// With nothing published, the node reports that neither input has arrived.
//
// Only the two arrival flags are pinned here. The reported level and the rest of the message are
// left alone on purpose: the transform result and both message ages are reported before any input
// has ever set them, so what they contribute to the message is not something to hold the node to.
TEST_F(GyroOdometerNodeCharacterization, NoInputReportsNeitherMessageArrived)
{
  start_node("base_link", 10.0);

  const DiagnosticsSnapshot diagnostics = take_diagnostics();

  EXPECT_FALSE(reported_flag(diagnostics, "is_arrived_first_vehicle_twist"));
  EXPECT_FALSE(reported_flag(diagnostics, "is_arrived_first_imu"));
  EXPECT_NE(diagnostics.message.find("Twist msg has not been arrived yet."), std::string::npos);
  EXPECT_NE(diagnostics.message.find("IMU msg has not been arrived yet."), std::string::npos);
}

// Vehicle twists that arrive while no IMU sample is queued accumulate, and the IMU sample that
// completes the pair fuses against all of them at once: the reported longitudinal velocity is
// their mean and the reported variance is their mean variance divided by how many there were.
TEST_F(GyroOdometerNodeCharacterization, AccumulatedVehicleTwistsAreAveragedIntoOneFusion)
{
  start_node("base_link", 10.0);
  const auto stamp = make_stamp(100, 0);

  // The first message of each kind only marks its side as arrived, so the queues have to be primed
  // before a scenario can put a known number of messages in them.
  send_vehicle_twist(make_vehicle_twist(stamp, 0.0, 0.0));
  send_imu(make_imu(stamp, "base_link", 0.0, 0.0, 0.0, 0.0, 0.0, 0.0));
  send_vehicle_twist(make_vehicle_twist(stamp, 0.0, 0.0));
  ASSERT_TRUE(take_output().has_value()) << "priming did not reach a first fusion";

  send_vehicle_twist(make_vehicle_twist(stamp, 1.0, 4.0));
  send_vehicle_twist(make_vehicle_twist(stamp, 3.0, 4.0));
  EXPECT_FALSE(take_output().has_value()) << "fused before an IMU sample completed the pair";

  send_imu(make_imu(stamp, "base_link", 0.1, 0.2, 0.3, 0.01, 0.02, 0.03));

  const auto output = take_output();
  ASSERT_TRUE(output.has_value());
  const auto & fused = output->twist_with_covariance_raw;

  EXPECT_DOUBLE_EQ(fused.twist.twist.linear.x, 2.0);
  EXPECT_DOUBLE_EQ(fused.twist.covariance[COV_IDX_XYZRPY::X_X], 2.0);
  EXPECT_DOUBLE_EQ(fused.twist.twist.angular.x, 0.1);
  EXPECT_DOUBLE_EQ(fused.twist.twist.angular.y, 0.2);
  EXPECT_DOUBLE_EQ(fused.twist.twist.angular.z, 0.3);

  const DiagnosticsSnapshot diagnostics = take_diagnostics();
  EXPECT_EQ(reported_int(diagnostics, "vehicle_twist_queue_size"), 2);
  EXPECT_EQ(reported_int(diagnostics, "imu_queue_size"), 1);
}

// A completed pair puts the longitudinal velocity of the vehicle twist and the angular velocity of
// the IMU on all four output topics.
TEST_F(GyroOdometerNodeCharacterization, CompletedPairIsPublishedOnAllFourTopics)
{
  start_node("base_link", 10.0);
  const auto stamp = make_stamp(100, 0);

  send_vehicle_twist(make_vehicle_twist(stamp, 1.0, 4.0));
  send_imu(make_imu(stamp, "base_link", 0.1, 0.2, 0.3, 0.01, 0.02, 0.03));
  send_vehicle_twist(make_vehicle_twist(stamp, 1.0, 4.0));

  const auto output = take_output();
  ASSERT_TRUE(output.has_value());

  EXPECT_DOUBLE_EQ(output->twist_raw.twist.linear.x, 1.0);
  EXPECT_DOUBLE_EQ(output->twist_raw.twist.angular.z, 0.3);
  EXPECT_DOUBLE_EQ(output->twist.twist.linear.x, 1.0);
  EXPECT_DOUBLE_EQ(output->twist.twist.angular.z, 0.3);

  const auto & raw = output->twist_with_covariance_raw;
  EXPECT_EQ(raw.header.frame_id, "base_link");
  EXPECT_DOUBLE_EQ(raw.twist.twist.linear.x, 1.0);
  EXPECT_DOUBLE_EQ(raw.twist.twist.angular.x, 0.1);
  EXPECT_DOUBLE_EQ(raw.twist.twist.angular.y, 0.2);
  EXPECT_DOUBLE_EQ(raw.twist.twist.angular.z, 0.3);
  // The lateral and vertical velocities are not estimated, and say so through a large variance.
  EXPECT_DOUBLE_EQ(raw.twist.twist.linear.y, 0.0);
  EXPECT_DOUBLE_EQ(raw.twist.twist.linear.z, 0.0);
  EXPECT_DOUBLE_EQ(raw.twist.covariance[COV_IDX_XYZRPY::Y_Y], 100000.0);
  EXPECT_DOUBLE_EQ(raw.twist.covariance[COV_IDX_XYZRPY::Z_Z], 100000.0);
  // The angular variance is the largest of the reported axes, applied to all three.
  EXPECT_DOUBLE_EQ(raw.twist.covariance[COV_IDX_XYZRPY::ROLL_ROLL], 0.03);
  EXPECT_DOUBLE_EQ(raw.twist.covariance[COV_IDX_XYZRPY::PITCH_PITCH], 0.03);
  EXPECT_DOUBLE_EQ(raw.twist.covariance[COV_IDX_XYZRPY::YAW_YAW], 0.03);

  EXPECT_EQ(output->twist_with_covariance.twist.covariance, raw.twist.covariance);
}

// IMU samples alone never fuse, however many arrive.
TEST_F(GyroOdometerNodeCharacterization, ImuAloneNeverFuses)
{
  start_node("base_link", 10.0);
  const auto stamp = make_stamp(100, 0);

  send_imu(make_imu(stamp, "base_link", 0.1, 0.2, 0.3, 0.01, 0.02, 0.03));
  send_imu(make_imu(stamp, "base_link", 0.1, 0.2, 0.3, 0.01, 0.02, 0.03));
  send_imu(make_imu(stamp, "base_link", 0.1, 0.2, 0.3, 0.01, 0.02, 0.03));

  EXPECT_FALSE(take_output().has_value());

  const DiagnosticsSnapshot diagnostics = take_diagnostics();
  EXPECT_TRUE(reported_flag(diagnostics, "is_arrived_first_imu"));
  EXPECT_FALSE(reported_flag(diagnostics, "is_arrived_first_vehicle_twist"));
  EXPECT_NE(diagnostics.message.find("Twist msg has not been arrived yet."), std::string::npos);
}

// Vehicle twists alone never fuse, however many arrive.
TEST_F(GyroOdometerNodeCharacterization, VehicleTwistAloneNeverFuses)
{
  start_node("base_link", 10.0);
  const auto stamp = make_stamp(100, 0);

  send_vehicle_twist(make_vehicle_twist(stamp, 1.0, 4.0));
  send_vehicle_twist(make_vehicle_twist(stamp, 1.0, 4.0));
  send_vehicle_twist(make_vehicle_twist(stamp, 1.0, 4.0));

  EXPECT_FALSE(take_output().has_value());

  const DiagnosticsSnapshot diagnostics = take_diagnostics();
  EXPECT_TRUE(reported_flag(diagnostics, "is_arrived_first_vehicle_twist"));
  EXPECT_FALSE(reported_flag(diagnostics, "is_arrived_first_imu"));
  EXPECT_NE(diagnostics.message.find("IMU msg has not been arrived yet."), std::string::npos);
}

// IMU samples that arrive while no vehicle twist is queued accumulate, and the vehicle twist that
// completes the pair fuses against their mean angular velocity.
TEST_F(GyroOdometerNodeCharacterization, AccumulatedImuSamplesAreAveragedIntoOneFusion)
{
  start_node("base_link", 10.0);
  const auto stamp = make_stamp(100, 0);

  send_vehicle_twist(make_vehicle_twist(stamp, 0.0, 0.0));
  send_imu(make_imu(stamp, "base_link", 0.0, 0.0, 0.0, 0.0, 0.0, 0.0));
  send_vehicle_twist(make_vehicle_twist(stamp, 0.0, 0.0));
  ASSERT_TRUE(take_output().has_value()) << "priming did not reach a first fusion";

  send_imu(make_imu(stamp, "base_link", 0.0, 0.0, 0.2, 0.01, 0.01, 0.01));
  send_imu(make_imu(stamp, "base_link", 0.0, 0.0, 0.4, 0.01, 0.01, 0.01));
  EXPECT_FALSE(take_output().has_value()) << "fused before a vehicle twist completed the pair";

  send_vehicle_twist(make_vehicle_twist(stamp, 1.0, 4.0));

  const auto output = take_output();
  ASSERT_TRUE(output.has_value());
  EXPECT_DOUBLE_EQ(output->twist_with_covariance_raw.twist.twist.angular.z, 0.3);
  EXPECT_DOUBLE_EQ(
    output->twist_with_covariance_raw.twist.covariance[COV_IDX_XYZRPY::YAW_YAW], 0.005);

  const DiagnosticsSnapshot diagnostics = take_diagnostics();
  EXPECT_EQ(reported_int(diagnostics, "vehicle_twist_queue_size"), 1);
  EXPECT_EQ(reported_int(diagnostics, "imu_queue_size"), 2);
}

// The output carries the later of the two input stamps, whichever side it comes from.
TEST_F(GyroOdometerNodeCharacterization, OutputCarriesTheLaterVehicleTwistStamp)
{
  start_node("base_link", 10.0);
  const auto priming_stamp = make_stamp(100, 0);

  send_vehicle_twist(make_vehicle_twist(priming_stamp, 0.0, 0.0));
  send_imu(make_imu(priming_stamp, "base_link", 0.0, 0.0, 0.0, 0.0, 0.0, 0.0));
  send_vehicle_twist(make_vehicle_twist(priming_stamp, 0.0, 0.0));
  ASSERT_TRUE(take_output().has_value()) << "priming did not reach a first fusion";

  send_imu(make_imu(make_stamp(98, 0), "base_link", 0.1, 0.2, 0.3, 0.01, 0.01, 0.01));
  send_vehicle_twist(make_vehicle_twist(make_stamp(99, 0), 1.0, 4.0));

  const auto output = take_output();
  ASSERT_TRUE(output.has_value());
  EXPECT_EQ(rclcpp::Time(output->twist_with_covariance_raw.header.stamp).seconds(), 99.0);
}

// Mirror of the above: this time the IMU sample is the later of the two.
TEST_F(GyroOdometerNodeCharacterization, OutputCarriesTheLaterImuStamp)
{
  start_node("base_link", 10.0);
  const auto priming_stamp = make_stamp(100, 0);

  send_vehicle_twist(make_vehicle_twist(priming_stamp, 0.0, 0.0));
  send_imu(make_imu(priming_stamp, "base_link", 0.0, 0.0, 0.0, 0.0, 0.0, 0.0));
  send_vehicle_twist(make_vehicle_twist(priming_stamp, 0.0, 0.0));
  ASSERT_TRUE(take_output().has_value()) << "priming did not reach a first fusion";

  send_vehicle_twist(make_vehicle_twist(make_stamp(98, 0), 1.0, 4.0));
  send_imu(make_imu(make_stamp(99, 0), "base_link", 0.1, 0.2, 0.3, 0.01, 0.01, 0.01));

  const auto output = take_output();
  ASSERT_TRUE(output.has_value());
  EXPECT_EQ(rclcpp::Time(output->twist_with_covariance_raw.header.stamp).seconds(), 99.0);
}

// A vehicle twist older than the tolerance drops the pending data instead of fusing it, and says
// so through the diagnostics.
TEST_F(GyroOdometerNodeCharacterization, VehicleTwistOlderThanToleranceDropsPendingData)
{
  start_node("base_link", 1.0);
  const auto stamp = make_stamp(100, 0);

  send_vehicle_twist(make_vehicle_twist(stamp, 0.0, 0.0));
  send_imu(make_imu(stamp, "base_link", 0.0, 0.0, 0.0, 0.0, 0.0, 0.0));
  send_vehicle_twist(make_vehicle_twist(stamp, 0.0, 0.0));
  ASSERT_TRUE(take_output().has_value()) << "priming did not reach a first fusion";

  send_vehicle_twist(make_vehicle_twist(stamp, 1.0, 4.0));
  set_now(rclcpp::Time(105, 0, RCL_ROS_TIME));
  send_imu(make_imu(make_stamp(105, 0), "base_link", 0.1, 0.2, 0.3, 0.01, 0.01, 0.01));

  EXPECT_FALSE(take_output().has_value());

  const DiagnosticsSnapshot diagnostics = take_diagnostics();
  EXPECT_EQ(diagnostics.level, diagnostic_msgs::msg::DiagnosticStatus::ERROR);
  EXPECT_NE(
    diagnostics.message.find("Vehicle twist msg is timeout. vehicle_twist_dt: 5[sec]"),
    std::string::npos)
    << "reported message was: " << diagnostics.message;
}

// Mirror of the above: this time the IMU sample is the one older than the tolerance.
TEST_F(GyroOdometerNodeCharacterization, ImuOlderThanToleranceDropsPendingData)
{
  start_node("base_link", 1.0);
  const auto stamp = make_stamp(100, 0);

  send_vehicle_twist(make_vehicle_twist(stamp, 0.0, 0.0));
  send_imu(make_imu(stamp, "base_link", 0.0, 0.0, 0.0, 0.0, 0.0, 0.0));
  send_vehicle_twist(make_vehicle_twist(stamp, 0.0, 0.0));
  ASSERT_TRUE(take_output().has_value()) << "priming did not reach a first fusion";

  send_imu(make_imu(stamp, "base_link", 0.1, 0.2, 0.3, 0.01, 0.01, 0.01));
  set_now(rclcpp::Time(105, 0, RCL_ROS_TIME));
  send_vehicle_twist(make_vehicle_twist(make_stamp(105, 0), 1.0, 4.0));

  EXPECT_FALSE(take_output().has_value());

  const DiagnosticsSnapshot diagnostics = take_diagnostics();
  EXPECT_EQ(diagnostics.level, diagnostic_msgs::msg::DiagnosticStatus::ERROR);
  EXPECT_NE(diagnostics.message.find("IMU msg is timeout. imu_dt: 5[sec]"), std::string::npos)
    << "reported message was: " << diagnostics.message;
}

// An IMU frame that cannot be resolved into the output frame drops the pending data instead of
// fusing it, and says so through the diagnostics.
TEST_F(GyroOdometerNodeCharacterization, UnresolvableImuFrameDropsPendingData)
{
  start_node("base_link", 10.0);
  const auto stamp = make_stamp(100, 0);

  send_vehicle_twist(make_vehicle_twist(stamp, 1.0, 4.0));
  send_imu(make_imu(stamp, "imu_link", 0.1, 0.2, 0.3, 0.01, 0.01, 0.01));
  send_vehicle_twist(make_vehicle_twist(stamp, 1.0, 4.0));

  EXPECT_FALSE(take_output().has_value());

  const DiagnosticsSnapshot diagnostics = take_diagnostics();
  EXPECT_FALSE(reported_flag(diagnostics, "is_succeed_transform_imu"));
  // A sample that never became usable does not count as the IMU side having arrived.
  EXPECT_FALSE(reported_flag(diagnostics, "is_arrived_first_imu"));
  EXPECT_EQ(diagnostics.level, diagnostic_msgs::msg::DiagnosticStatus::ERROR);
  EXPECT_NE(
    diagnostics.message.find("Please publish TF from base_link to frame of IMU."),
    std::string::npos)
    << "reported message was: " << diagnostics.message;
}

// An IMU sample whose frame cannot be resolved must never have its rate reach an output, not even
// one that resolvable samples complete afterwards. It is kept out of the queue rather than fused,
// so the resolvable sample that follows it is free to be fused on its own.
TEST_F(GyroOdometerNodeCharacterization, UnresolvableImuKeepsItsRateOutOfEveryOutput)
{
  start_node("base_link", 10.0);
  const auto stamp = make_stamp(100, 0);

  send_vehicle_twist(make_vehicle_twist(stamp, 0.0, 0.0));
  send_imu(make_imu(stamp, "base_link", 0.0, 0.0, 0.0, 0.0, 0.0, 0.0));
  send_vehicle_twist(make_vehicle_twist(stamp, 0.0, 0.0));
  ASSERT_TRUE(take_output().has_value()) << "priming did not reach a first fusion";

  // Unresolvable, and carrying a rate nothing else in this scenario could account for.
  send_imu(make_imu(stamp, "unresolvable_link", 10.0, 0.0, 0.0, 0.0, 0.0, 0.0));
  // Resolvable, and the only sample a fusion is allowed to draw on.
  send_imu(make_imu(stamp, "base_link", 0.0, 0.0, 0.3, 0.01, 0.01, 0.01));
  send_vehicle_twist(make_vehicle_twist(stamp, 1.0, 4.0));

  const auto output = take_output();
  ASSERT_TRUE(output.has_value());
  const auto & fused = output->twist_with_covariance_raw;

  EXPECT_DOUBLE_EQ(fused.twist.twist.angular.x, 0.0)
    << "the unresolvable sample was fused: its rate is showing up in the output";
  EXPECT_DOUBLE_EQ(fused.twist.twist.angular.z, 0.3);

  const DiagnosticsSnapshot diagnostics = take_diagnostics();
  EXPECT_EQ(reported_int(diagnostics, "imu_queue_size"), 1)
    << "the unresolvable sample was queued alongside the resolvable one";
}

// A fusion that completes leaves nothing for the diagnostics to complain about.
TEST_F(GyroOdometerNodeCharacterization, CompletedFusionReportsOk)
{
  start_node("base_link", 10.0);
  const auto stamp = make_stamp(100, 0);

  send_vehicle_twist(make_vehicle_twist(stamp, 1.0, 4.0));
  send_imu(make_imu(stamp, "base_link", 0.1, 0.2, 0.3, 0.01, 0.01, 0.01));
  send_vehicle_twist(make_vehicle_twist(stamp, 1.0, 4.0));
  ASSERT_TRUE(take_output().has_value());

  const DiagnosticsSnapshot diagnostics = take_diagnostics();
  EXPECT_EQ(diagnostics.level, diagnostic_msgs::msg::DiagnosticStatus::OK);
  EXPECT_EQ(diagnostics.message, "OK");
  EXPECT_TRUE(reported_flag(diagnostics, "is_succeed_transform_imu"));
}

// An IMU sample in an unresolvable frame is reported as a transform failure; being too old on top
// of that is not reported, because the sample never reaches the staleness judgment.
TEST_F(GyroOdometerNodeCharacterization, StaleAndUnresolvableImuReportsTheTransformFailure)
{
  start_node("base_link", 1.0);
  const auto stamp = make_stamp(100, 0);

  send_vehicle_twist(make_vehicle_twist(stamp, 0.0, 0.0));
  send_imu(make_imu(stamp, "base_link", 0.0, 0.0, 0.0, 0.0, 0.0, 0.0));
  send_vehicle_twist(make_vehicle_twist(stamp, 0.0, 0.0));
  ASSERT_TRUE(take_output().has_value()) << "priming did not reach a first fusion";

  send_vehicle_twist(make_vehicle_twist(stamp, 1.0, 4.0));
  set_now(rclcpp::Time(105, 0, RCL_ROS_TIME));
  send_imu(make_imu(make_stamp(105, 0), "imu_link", 0.1, 0.2, 0.3, 0.01, 0.01, 0.01));

  EXPECT_FALSE(take_output().has_value());

  const DiagnosticsSnapshot diagnostics = take_diagnostics();
  EXPECT_FALSE(reported_flag(diagnostics, "is_succeed_transform_imu"));
  EXPECT_EQ(diagnostics.message, "Please publish TF from base_link to frame of IMU.");
}

// An unresolvable IMU sample arriving before any vehicle twist has to be survivable: there is no
// vehicle twist yet whose age could be worked out, and the node still has to carry on reporting.
TEST_F(GyroOdometerNodeCharacterization, UnresolvableImuBeforeAnyVehicleTwistIsSurvivable)
{
  start_node("base_link", 10.0);
  const auto stamp = make_stamp(100, 0);

  send_imu(make_imu(stamp, "imu_link", 0.1, 0.2, 0.3, 0.01, 0.01, 0.01));

  const DiagnosticsSnapshot diagnostics = take_diagnostics();
  EXPECT_FALSE(reported_flag(diagnostics, "is_arrived_first_imu"));
  EXPECT_FALSE(reported_flag(diagnostics, "is_arrived_first_vehicle_twist"));
  EXPECT_FALSE(reported_flag(diagnostics, "is_succeed_transform_imu"));
}

// The transform status describes the IMU sample that arrived most recently, whether or not there
// was anything to fuse it against.
TEST_F(GyroOdometerNodeCharacterization, ImuAloneStillReportsItsTransformStatus)
{
  start_node("base_link", 10.0);
  const auto stamp = make_stamp(100, 0);

  send_imu(make_imu(stamp, "base_link", 0.1, 0.2, 0.3, 0.01, 0.01, 0.01));
  send_imu(make_imu(stamp, "base_link", 0.1, 0.2, 0.3, 0.01, 0.01, 0.01));

  const DiagnosticsSnapshot diagnostics = take_diagnostics();
  EXPECT_TRUE(reported_flag(diagnostics, "is_succeed_transform_imu"));
  EXPECT_EQ(diagnostics.message, "Twist msg has not been arrived yet.");
}

// With the IMU frame resolvable, the queued angular velocity is rotated by the looked-up transform
// before it is fused.
TEST_F(GyroOdometerNodeCharacterization, ResolvableImuFrameRotatesTheAngularVelocity)
{
  start_node("base_link", 10.0);
  const auto stamp = make_stamp(100, 0);

  // A quarter turn about the vertical axis, which sends (x, y, z) to (-y, x, z).
  constexpr double quarter_turn = 0.7071067811865476;
  broadcast_static_transform("imu_link", "base_link", 0.0, 0.0, quarter_turn, quarter_turn);

  send_vehicle_twist(make_vehicle_twist(stamp, 1.0, 4.0));
  send_imu(make_imu(stamp, "imu_link", 0.1, 0.2, 0.3, 0.01, 0.02, 0.03));
  send_vehicle_twist(make_vehicle_twist(stamp, 1.0, 4.0));

  const auto output = take_output();
  ASSERT_TRUE(output.has_value());
  const auto & raw = output->twist_with_covariance_raw;

  EXPECT_NEAR(raw.twist.twist.angular.x, -0.2, 1e-9);
  EXPECT_NEAR(raw.twist.twist.angular.y, 0.1, 1e-9);
  EXPECT_NEAR(raw.twist.twist.angular.z, 0.3, 1e-9);
  // The samples are relabelled as belonging to the output frame.
  EXPECT_EQ(raw.header.frame_id, "base_link");

  const DiagnosticsSnapshot diagnostics = take_diagnostics();
  EXPECT_TRUE(reported_flag(diagnostics, "is_succeed_transform_imu"));
}

// Staleness is judged between the two inputs' stamps, not against the node clock: a pair whose
// stamps agree with each other fuses however far the clock has moved past them, and the ages the
// diagnostics report are measured between the stamps as well.
TEST_F(GyroOdometerNodeCharacterization, MutuallyFreshPairFusesRegardlessOfTheClock)
{
  start_node("base_link", 1.0);
  const auto stamp = make_stamp(100, 0);

  send_vehicle_twist(make_vehicle_twist(stamp, 0.0, 0.0));
  send_imu(make_imu(stamp, "base_link", 0.0, 0.0, 0.0, 0.0, 0.0, 0.0));
  send_vehicle_twist(make_vehicle_twist(stamp, 0.0, 0.0));
  ASSERT_TRUE(take_output().has_value()) << "priming did not reach a first fusion";

  set_now(rclcpp::Time(200, 0, RCL_ROS_TIME));
  send_vehicle_twist(make_vehicle_twist(make_stamp(100, 500000000), 1.0, 4.0));
  send_imu(make_imu(make_stamp(100, 500000000), "base_link", 0.1, 0.2, 0.3, 0.01, 0.01, 0.01));

  const auto output = take_output();
  ASSERT_TRUE(output.has_value());
  EXPECT_DOUBLE_EQ(output->twist_with_covariance_raw.twist.twist.linear.x, 1.0);

  const DiagnosticsSnapshot diagnostics = take_diagnostics();
  EXPECT_EQ(diagnostics.level, diagnostic_msgs::msg::DiagnosticStatus::OK);
}

// The staleness judgment depends only on each side's most recent stamp: stamps seen earlier leave
// no residue, so a mutually consistent pair fuses regardless of what was fused before it.
TEST_F(GyroOdometerNodeCharacterization, StalenessDependsOnlyOnTheLatestStamps)
{
  start_node("base_link", 1.0);

  send_vehicle_twist(make_vehicle_twist(make_stamp(100, 0), 0.0, 0.0));
  send_imu(make_imu(make_stamp(100, 0), "base_link", 0.0, 0.0, 0.0, 0.0, 0.0, 0.0));
  send_vehicle_twist(make_vehicle_twist(make_stamp(100, 0), 0.0, 0.0));
  ASSERT_TRUE(take_output().has_value()) << "priming did not reach a first fusion";

  send_vehicle_twist(make_vehicle_twist(make_stamp(10, 0), 3.0, 4.0));
  send_imu(make_imu(make_stamp(10, 0), "base_link", 0.1, 0.2, 0.3, 0.01, 0.01, 0.01));
  send_vehicle_twist(make_vehicle_twist(make_stamp(10, 0), 3.0, 4.0));

  const auto output = take_output();
  ASSERT_TRUE(output.has_value());
  EXPECT_DOUBLE_EQ(output->twist_with_covariance_raw.twist.twist.linear.x, 3.0);
}

// A vehicle twist waiting for its IMU counterpart survives an unresolvable sample in between: the
// next resolvable sample fuses with it as if the unresolvable one had never been published.
TEST_F(GyroOdometerNodeCharacterization, PendingDataSurvivesAnUnresolvableImu)
{
  start_node("base_link", 10.0);
  const auto stamp = make_stamp(100, 0);

  send_vehicle_twist(make_vehicle_twist(stamp, 0.0, 0.0));
  send_imu(make_imu(stamp, "base_link", 0.0, 0.0, 0.0, 0.0, 0.0, 0.0));
  send_vehicle_twist(make_vehicle_twist(stamp, 0.0, 0.0));
  ASSERT_TRUE(take_output().has_value()) << "priming did not reach a first fusion";

  send_vehicle_twist(make_vehicle_twist(stamp, 7.0, 4.0));
  send_imu(make_imu(stamp, "unresolvable_link", 10.0, 0.0, 0.0, 0.0, 0.0, 0.0));
  send_imu(make_imu(stamp, "base_link", 0.0, 0.0, 0.3, 0.01, 0.01, 0.01));

  const auto output = take_output();
  ASSERT_TRUE(output.has_value());
  EXPECT_DOUBLE_EQ(output->twist_with_covariance_raw.twist.twist.linear.x, 7.0);
  EXPECT_DOUBLE_EQ(output->twist_with_covariance_raw.twist.twist.angular.x, 0.0);
}

// At a standstill the compensated pair reports no rotation at all, while the raw pair keeps what
// the IMU measured.
TEST_F(GyroOdometerNodeCharacterization, StandstillClearsAngularVelocityInTheCompensatedOutput)
{
  start_node("base_link", 10.0);
  const auto stamp = make_stamp(100, 0);

  send_vehicle_twist(make_vehicle_twist(stamp, 0.0, 0.0));
  send_imu(make_imu(stamp, "base_link", 0.5, 0.6, 0.0, 0.01, 0.01, 0.01));
  send_vehicle_twist(make_vehicle_twist(stamp, 0.0, 4.0));

  const auto output = take_output();
  ASSERT_TRUE(output.has_value());

  EXPECT_DOUBLE_EQ(output->twist_with_covariance_raw.twist.twist.angular.x, 0.5);
  EXPECT_DOUBLE_EQ(output->twist_with_covariance_raw.twist.twist.angular.y, 0.6);
  EXPECT_DOUBLE_EQ(output->twist_raw.twist.angular.x, 0.5);

  EXPECT_DOUBLE_EQ(output->twist_with_covariance.twist.twist.angular.x, 0.0);
  EXPECT_DOUBLE_EQ(output->twist_with_covariance.twist.twist.angular.y, 0.0);
  EXPECT_DOUBLE_EQ(output->twist_with_covariance.twist.twist.angular.z, 0.0);
  EXPECT_DOUBLE_EQ(output->twist.twist.angular.x, 0.0);
  EXPECT_DOUBLE_EQ(output->twist.twist.angular.y, 0.0);
}

}  // namespace autoware::gyro_odometer
