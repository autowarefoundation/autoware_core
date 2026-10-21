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

// cspell:ignore SBAS GBAS

// =====================================================================================
// Integration test of autoware::gnss_poser::GnssPoserNode, the ROS layer of the GNSS poser.
//
// The pose computation itself (GnssPoser) is unit-tested in test_gnss_poser.cpp without ROS. What
// is checked here, over real topics, is what only the node does: declare the parameters, offer the
// topics, hand the inputs and the parameters to the logic, publish the right topics for each
// outcome with the right headers, and look up the antenna transform.
//
// The cases come from the characterization suite that pinned the node before its logic was
// separated, and from the node test that came with the package. Where the pinned behavior looks
// like a latent defect it is marked "NOTE(characterization)", so that a later change can update
// the corresponding case on purpose.
//
// Test naming: <Aspect>_<Condition>_<ObservedBehavior>
//
// Each case is written as Arrange / Act / Assert: the inputs that set the scene, the one input
// whose effect is checked, and the checks.
// =====================================================================================

#include "gnss_poser_node.hpp"

#include <autoware/geography_utils/height.hpp>
#include <autoware/geography_utils/projection.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2/LinearMath/Matrix3x3.hpp>
#include <tf2/LinearMath/Quaternion.hpp>
#include <tf2_ros/buffer.hpp>
#include <tf2_ros/static_transform_broadcaster.hpp>
#include <tf2_ros/transform_broadcaster.hpp>
#include <tf2_ros/transform_listener.hpp>

#include <autoware_internal_debug_msgs/msg/bool_stamped.hpp>
#include <autoware_map_msgs/msg/map_projector_info.hpp>
#include <autoware_sensing_msgs/msg/gnss_ins_orientation_stamped.hpp>
#include <geographic_msgs/msg/geo_point.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_msgs/msg/tf_message.hpp>

#include <gtest/gtest.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <exception>
#include <functional>
#include <memory>
#include <ostream>
#include <stdexcept>
#include <string>
#include <thread>
#include <tuple>
#include <utility>
#include <vector>

namespace
{
using autoware_internal_debug_msgs::msg::BoolStamped;
using autoware_map_msgs::msg::MapProjectorInfo;
using autoware_sensing_msgs::msg::GnssInsOrientationStamped;
using geometry_msgs::msg::Point;
using geometry_msgs::msg::PoseStamped;
using geometry_msgs::msg::PoseWithCovarianceStamped;
using geometry_msgs::msg::Quaternion;
using geometry_msgs::msg::TransformStamped;
using sensor_msgs::msg::NavSatFix;
using sensor_msgs::msg::NavSatStatus;
using tf2_msgs::msg::TFMessage;
using std::chrono_literals::operator""ms;
using std::chrono_literals::operator""s;

// ---------------------------------------------------------------------------------------
// Reference input used throughout (same point as autoware_geography_utils' own tests).
// ---------------------------------------------------------------------------------------
constexpr double reference_latitude = 35.62426;
constexpr double reference_longitude = 139.74252;
constexpr double reference_altitude = 10.0;
constexpr const char * reference_mgrs_grid = "54SUE";
// Cases that need several fixes offset them by multiples of 0.001 degrees: about 111 m in latitude
// and 90 m in longitude at this latitude, so the fixes project to clearly distinct positions and
// every output can be matched to the fix it came from.

// Golden values: MGRS(54SUE) projection of (reference_latitude, reference_longitude,
// reference_altitude) as observed from the node output (identity antenna->base TF, WGS84 vertical
// datum). Recorded on 2026-09-03 with the GeographicLib/lanelet2 versions bundled in the autoware
// core-devel jazzy image.
constexpr double golden_x = 86128.181788819958;
constexpr double golden_y = 43002.610125367064;
constexpr double golden_z = reference_altitude;
// Tolerances for computed values. A behavior change worth catching moves a position by centimeters
// or more and a heading by about 0.001 rad or more; the floating-point noise of the pipeline is
// many orders of magnitude below both. Values the node copies are compared exactly instead.
constexpr double position_tolerance = 1e-4;  // [m]
constexpr double angle_tolerance = 1e-6;     // [rad]

// A deterministic, clearly artificial header stamp so that a stamp copied from the input can be
// told apart from one taken from a clock.
constexpr int32_t fix_stamp_sec = 1700000000;
constexpr uint32_t fix_stamp_nanosec = 123456789U;

// Wall-clock budget for the loopback delivery of one published message. The test thread pumps the
// executor itself and only ever one input is in flight, so this only has to cover the delivery.
// It also bounds the window in which an output that is *not* expected would have shown up.
constexpr std::chrono::milliseconds delivery_budget{200};
// Wall-clock budget for anything the test actively waits on (an output that must arrive).
constexpr std::chrono::milliseconds wait_budget{3000};
// Wall-clock budget for pub/sub discovery between the peer and the node under test.
constexpr std::chrono::milliseconds discovery_budget{10000};

struct NodeParams
{
  std::string base_frame = "base_link";
  std::string gnss_base_frame = "gnss_base_link";
  std::string map_frame = "map";
  bool use_gnss_ins_orientation = true;
  int gnss_pose_pub_method = 0;
  int buff_epoch = 1;

  // The six parameters the node declares, in declaration order.
  static const std::vector<std::string> & names()
  {
    static const std::vector<std::string> parameter_names = {
      "base_frame",           "gnss_base_frame", "map_frame", "use_gnss_ins_orientation",
      "gnss_pose_pub_method", "buff_epoch"};
    return parameter_names;
  }

  // Parameter overrides for the node under test. `skip` leaves that one parameter unset.
  [[nodiscard]] rclcpp::NodeOptions to_options(const std::string & skip = "") const
  {
    rclcpp::NodeOptions options;
    const auto add = [&](const std::string & name, const auto & value) {
      if (name != skip) {
        options.append_parameter_override(name, value);
      }
    };
    add("base_frame", base_frame);
    add("gnss_base_frame", gnss_base_frame);
    add("map_frame", map_frame);
    add("use_gnss_ins_orientation", use_gnss_ins_orientation);
    add("gnss_pose_pub_method", gnss_pose_pub_method);
    add("buff_epoch", buff_epoch);
    return options;
  }
};

builtin_interfaces::msg::Time make_stamp(int32_t sec, uint32_t nanosec)
{
  builtin_interfaces::msg::Time stamp;
  stamp.sec = sec;
  stamp.nanosec = nanosec;
  return stamp;
}

builtin_interfaces::msg::Time fix_stamp()
{
  return make_stamp(fix_stamp_sec, fix_stamp_nanosec);
}

NavSatFix make_fix(
  double latitude, double longitude, double altitude,
  NavSatStatus::_status_type status = NavSatStatus::STATUS_FIX,
  const std::string & frame_id = "gnss")
{
  NavSatFix msg;
  msg.header.stamp = fix_stamp();
  msg.header.frame_id = frame_id;
  msg.status.status = status;
  msg.status.service = NavSatStatus::SERVICE_GPS;
  msg.latitude = latitude;
  msg.longitude = longitude;
  msg.altitude = altitude;
  msg.position_covariance = {1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0};
  msg.position_covariance_type = NavSatFix::COVARIANCE_TYPE_DIAGONAL_KNOWN;
  return msg;
}

NavSatFix make_reference_fix(
  NavSatStatus::_status_type status = NavSatStatus::STATUS_FIX,
  const std::string & frame_id = "gnss")
{
  return make_fix(reference_latitude, reference_longitude, reference_altitude, status, frame_id);
}

// Fixes of the average cases: about 100 m apart (see the note at the reference constants).
std::vector<NavSatFix> make_average_fixes()
{
  return {
    make_fix(reference_latitude, reference_longitude, reference_altitude),
    make_fix(reference_latitude + 0.001, reference_longitude + 0.002, reference_altitude + 20.0),
    make_fix(reference_latitude + 0.002, reference_longitude + 0.001, reference_altitude + 10.0),
    make_fix(reference_latitude + 0.003, reference_longitude + 0.003, reference_altitude + 30.0)};
}

MapProjectorInfo make_mgrs_projector_info(
  const std::string & grid = reference_mgrs_grid,
  const std::string & vertical_datum = MapProjectorInfo::WGS84)
{
  MapProjectorInfo msg;
  msg.projector_type = MapProjectorInfo::MGRS;
  msg.vertical_datum = vertical_datum;
  msg.mgrs_grid = grid;
  return msg;
}

MapProjectorInfo make_local_projector_info()
{
  MapProjectorInfo msg;
  msg.projector_type = MapProjectorInfo::LOCAL;
  msg.vertical_datum = MapProjectorInfo::WGS84;
  return msg;
}

Quaternion yaw_to_quaternion(double yaw)
{
  tf2::Quaternion q;
  q.setRPY(0.0, 0.0, yaw);
  return tf2::toMsg(q);
}

double yaw_of(const Quaternion & quaternion)
{
  tf2::Quaternion q;
  tf2::fromMsg(quaternion, q);
  double roll = 0.0;
  double pitch = 0.0;
  double yaw = 0.0;
  tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
  return yaw;
}

GnssInsOrientationStamped make_orientation(
  double yaw, double rmse_x = 0.1, double rmse_y = 0.2, double rmse_z = 0.3)
{
  GnssInsOrientationStamped msg;
  msg.header.stamp = make_stamp(1, 0);  // deliberately unrelated to the fix stamp
  msg.header.frame_id = "ins";
  msg.orientation.orientation = yaw_to_quaternion(yaw);
  msg.orientation.rmse_rotation_x = rmse_x;
  msg.orientation.rmse_rotation_y = rmse_y;
  msg.orientation.rmse_rotation_z = rmse_z;
  return msg;
}

// Antenna position the node is expected to derive from a NavSatFix, computed with the same
// library the node uses (autoware_geography_utils). Used to build exact expectations for the
// buffering / orientation / TF composition logic, which is what these tests characterize.
Point project_antenna(const NavSatFix & fix, const MapProjectorInfo & projector_info)
{
  geographic_msgs::msg::GeoPoint geo_point;
  geo_point.latitude = fix.latitude;
  geo_point.longitude = fix.longitude;
  geo_point.altitude = fix.altitude;
  Point position = autoware::geography_utils::project_forward(geo_point, projector_info);
  position.z = autoware::geography_utils::convert_height(
    position.z, geo_point.latitude, geo_point.longitude, MapProjectorInfo::WGS84,
    projector_info.vertical_datum);
  return position;
}

Point make_point(double x, double y, double z)
{
  Point p;
  p.x = x;
  p.y = y;
  p.z = z;
  return p;
}

void expect_point_near(
  const Point & actual, const Point & expected, double tol = position_tolerance)
{
  EXPECT_NEAR(actual.x, expected.x, tol);
  EXPECT_NEAR(actual.y, expected.y, tol);
  EXPECT_NEAR(actual.z, expected.z, tol);
}

// Params of the buffering cases: gnss_pose_pub_method and buff_epoch, the rest at their defaults.
NodeParams make_buffer_params(int gnss_pose_pub_method, int buff_epoch)
{
  NodeParams params;
  params.gnss_pose_pub_method = gnss_pose_pub_method;
  params.buff_epoch = buff_epoch;
  return params;
}

// Antenna position of a fix under the MGRS projector info of make_mgrs_projector_info().
Point project_mgrs(const NavSatFix & fix)
{
  return project_antenna(fix, make_mgrs_projector_info());
}

// The reference fix in the antenna frame of the TF cases, stamped `stamp`.
NavSatFix make_antenna_fix_at(const builtin_interfaces::msg::Time & stamp)
{
  auto fix = make_reference_fix(NavSatStatus::STATUS_FIX, "my_antenna");
  fix.header.stamp = stamp;
  return fix;
}

void expect_qos(
  const rclcpp::TopicEndpointInfo & endpoint, rclcpp::ReliabilityPolicy reliability,
  rclcpp::DurabilityPolicy durability)
{
  EXPECT_EQ(endpoint.qos_profile().reliability(), reliability) << endpoint.topic_type();
  EXPECT_EQ(endpoint.qos_profile().durability(), durability) << endpoint.topic_type();
}

// ---------------------------------------------------------------------------------------
// Test double: a plain rclcpp node that plays the role of every peer of gnss_poser
// (map_projection_loader, GNSS driver, INS driver, robot_state_publisher, and the consumers).
// ---------------------------------------------------------------------------------------
class PeerNode : public rclcpp::Node
{
public:
  PeerNode()
  : rclcpp::Node("gnss_poser_integration_peer"),
    tf_buffer_(get_clock()),
    tf_listener_(tf_buffer_, this, /*spin_thread=*/false)
  {
    projector_pub_ = create_publisher<MapProjectorInfo>(
      "/map/map_projector_info", rclcpp::QoS{1}.transient_local());
    fix_pub_ = create_publisher<NavSatFix>("fix", rclcpp::QoS{1});
    orientation_pub_ =
      create_publisher<GnssInsOrientationStamped>("autoware_orientation", rclcpp::QoS{1});

    pose_sub_ = create_subscription<PoseStamped>(
      "gnss_pose", rclcpp::QoS{100},
      [this](const PoseStamped::ConstSharedPtr msg) { poses.push_back(*msg); });
    pose_cov_sub_ = create_subscription<PoseWithCovarianceStamped>(
      "gnss_pose_cov", rclcpp::QoS{100},
      [this](const PoseWithCovarianceStamped::ConstSharedPtr msg) {
        pose_cov_msgs.push_back(*msg);
      });
    fixed_sub_ = create_subscription<BoolStamped>(
      "gnss_fixed", rclcpp::QoS{100},
      [this](const BoolStamped::ConstSharedPtr msg) { fixed_flags.push_back(*msg); });
    tf_sub_ = create_subscription<TFMessage>(
      "/tf", rclcpp::QoS{100}, [this](const TFMessage::ConstSharedPtr msg) {
        for (const auto & t : msg->transforms) {
          // Only keep what gnss_poser broadcasts; this peer publishes on /tf as well.
          if (t.child_frame_id != own_dynamic_tf_child_) {
            broadcast_tfs.push_back(t);
          }
        }
      });

    static_tf_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(*this);
    dynamic_tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(*this);
  }

  [[nodiscard]] bool all_endpoints_matched() const
  {
    return projector_pub_->get_subscription_count() >= 1 &&
           fix_pub_->get_subscription_count() >= 1 &&
           orientation_pub_->get_subscription_count() >= 1 &&
           pose_sub_->get_publisher_count() >= 1 && pose_cov_sub_->get_publisher_count() >= 1 &&
           fixed_sub_->get_publisher_count() >= 1 &&
           // this peer's own dynamic broadcaster + gnss_poser's broadcaster
           tf_sub_->get_publisher_count() >= 2;
  }

  rclcpp::Publisher<MapProjectorInfo>::SharedPtr projector_pub_;
  rclcpp::Publisher<NavSatFix>::SharedPtr fix_pub_;
  rclcpp::Publisher<GnssInsOrientationStamped>::SharedPtr orientation_pub_;
  rclcpp::Subscription<PoseStamped>::SharedPtr pose_sub_;
  rclcpp::Subscription<PoseWithCovarianceStamped>::SharedPtr pose_cov_sub_;
  rclcpp::Subscription<BoolStamped>::SharedPtr fixed_sub_;
  rclcpp::Subscription<TFMessage>::SharedPtr tf_sub_;
  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> static_tf_broadcaster_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> dynamic_tf_broadcaster_;
  // Observer buffer: used only to know that a TF we broadcast has propagated. It is filled by the
  // test thread pumping this node, not by a dedicated thread, so it must be asked without a
  // timeout: the overloads that take one refuse to answer at all without such a thread.
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  std::string own_dynamic_tf_child_;

  std::vector<PoseStamped> poses;
  std::vector<PoseWithCovarianceStamped> pose_cov_msgs;
  std::vector<BoolStamped> fixed_flags;
  std::vector<TransformStamped> broadcast_tfs;
};

// Drives the node over its real topics from the test thread. There is no background spin on the
// test side: the executor holding both the peer and the node is pumped only from here, and only one
// input is ever in flight, so a scenario's steps reach the node in the order written. (The node's
// own TF listener runs its usual dedicated thread.)
class GnssPoserNodeIntegration : public ::testing::Test
{
protected:
  void SetUp() override
  {
    peer_ = std::make_shared<PeerNode>();
    executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
    executor_->add_node(peer_);
  }

  void TearDown() override
  {
    if (node_) {
      executor_->remove_node(node_->get_node_base_interface());
    }
    executor_->remove_node(peer_);
    node_.reset();
    executor_.reset();
    peer_.reset();
  }

  // Creates the node under test and waits for pub/sub discovery to complete.
  void build_node(const NodeParams & params)
  {
    node_ = std::make_shared<autoware::gnss_poser::GnssPoserNode>(params.to_options());
    executor_->add_node(node_->get_node_base_interface());
    ASSERT_TRUE(pump_until([this] { return peer_->all_endpoints_matched(); }, discovery_budget))
      << "pub/sub discovery between the peer and gnss_poser did not complete";
  }

  // Processes whatever is ready on both nodes for `duration` of wall-clock time.
  void pump(std::chrono::milliseconds duration)
  {
    const auto deadline = std::chrono::steady_clock::now() + duration;
    while (std::chrono::steady_clock::now() < deadline) {
      executor_->spin_some();
      std::this_thread::sleep_for(1ms);
    }
  }

  // Pumps until `predicate` holds or `timeout` expires; returns the predicate's final value.
  bool pump_until(
    const std::function<bool()> & predicate, std::chrono::milliseconds timeout = wait_budget)
  {
    const auto deadline = std::chrono::steady_clock::now() + timeout;
    while (std::chrono::steady_clock::now() < deadline) {
      executor_->spin_some();
      if (predicate()) {
        return true;
      }
      std::this_thread::sleep_for(1ms);
    }
    return predicate();
  }

  // Inputs. Each helper publishes one message and pumps for the delivery budget, so that the next
  // step of a scenario starts with this input delivered; none of them has an acknowledgement.
  void send_projector_info(const MapProjectorInfo & info)
  {
    peer_->projector_pub_->publish(info);
    pump(delivery_budget);
  }

  void send_orientation(const GnssInsOrientationStamped & msg)
  {
    peer_->orientation_pub_->publish(msg);
    pump(delivery_budget);
  }

  void send_fix(const NavSatFix & fix)
  {
    peer_->fix_pub_->publish(fix);
    pump(delivery_budget);
  }

  // Waits until `gnss_fixed` has been received `count` times. The node publishes it for every fix
  // that passes the projector gates, fixed or not, so it marks that fix as processed. Then pumps
  // for one more delivery budget so that the remaining outputs of the same callback, if any, have
  // landed before a count is asserted.
  void wait_for_gnss_fixed(std::size_t count)
  {
    ASSERT_TRUE(pump_until([this, count] { return peer_->fixed_flags.size() >= count; }))
      << "expected " << count << " gnss_fixed messages, got " << peer_->fixed_flags.size();
    pump(delivery_budget);
  }

  // Waits until every output of an accepted fix (gnss_fixed, gnss_pose, gnss_pose_cov and the
  // TF broadcast) has been received at least `count` times. The four topics arrive in no
  // particular order, so they are waited for together.
  void wait_for_outputs(std::size_t count)
  {
    ASSERT_TRUE(pump_until([this, count] {
      return peer_->fixed_flags.size() >= count && peer_->poses.size() >= count &&
             peer_->pose_cov_msgs.size() >= count && peer_->broadcast_tfs.size() >= count;
    }))
      << "expected " << count << " of each output, got gnss_fixed=" << peer_->fixed_flags.size()
      << " gnss_pose=" << peer_->poses.size() << " gnss_pose_cov=" << peer_->pose_cov_msgs.size()
      << " tf=" << peer_->broadcast_tfs.size();
  }

  // Exact number of messages received so far on each output.
  void expect_output_counts(
    std::size_t fixed, std::size_t pose, std::size_t pose_cov, std::size_t tf) const
  {
    EXPECT_EQ(peer_->fixed_flags.size(), fixed) << "gnss_fixed";
    EXPECT_EQ(peer_->poses.size(), pose) << "gnss_pose";
    EXPECT_EQ(peer_->pose_cov_msgs.size(), pose_cov) << "gnss_pose_cov";
    EXPECT_EQ(peer_->broadcast_tfs.size(), tf) << "/tf";
  }

  TransformStamped make_tf(
    const std::string & parent, const std::string & child, const Point & translation,
    const Quaternion & rotation, const builtin_interfaces::msg::Time & stamp)
  {
    TransformStamped tf;
    tf.header.stamp = stamp;
    tf.header.frame_id = parent;
    tf.child_frame_id = child;
    tf.transform.translation.x = translation.x;
    tf.transform.translation.y = translation.y;
    tf.transform.translation.z = translation.z;
    tf.transform.rotation = rotation;
    return tf;
  }

  void broadcast_static_tf(
    const std::string & parent, const std::string & child, const Point & translation,
    const Quaternion & rotation)
  {
    peer_->static_tf_broadcaster_->sendTransform(
      make_tf(parent, child, translation, rotation, peer_->now()));
    ASSERT_TRUE(
      pump_until([&] { return peer_->tf_buffer_.canTransform(parent, child, tf2::TimePointZero); }))
      << "static TF " << parent << "->" << child << " did not propagate";
    pump(delivery_budget);  // margin for the node's own (threaded) listener
  }

  // Publishes one time-stamped transform on /tf (as opposed to /tf_static), so that the node's
  // lookup time becomes observable.
  void broadcast_timed_tf(
    const std::string & parent, const std::string & child, const Point & translation,
    const Quaternion & rotation, const builtin_interfaces::msg::Time & stamp)
  {
    peer_->own_dynamic_tf_child_ = child;
    peer_->dynamic_tf_broadcaster_->sendTransform(
      make_tf(parent, child, translation, rotation, stamp));
    ASSERT_TRUE(pump_until(
      [&] { return peer_->tf_buffer_.canTransform(parent, child, tf2_ros::fromMsg(stamp)); }))
      << "timed TF " << parent << "->" << child << " did not propagate";
    pump(delivery_budget);
  }

  // True once every topic of the node is visible with its message type.
  bool all_topics_discovered()
  {
    const auto topics = peer_->get_topic_names_and_types();
    const auto has = [&](const std::string & name, const std::string & type) {
      const auto it = topics.find(name);
      return it != topics.end() &&
             std::find(it->second.begin(), it->second.end(), type) != it->second.end();
    };
    return has("/fix", "sensor_msgs/msg/NavSatFix") &&
           has("/autoware_orientation", "autoware_sensing_msgs/msg/GnssInsOrientationStamped") &&
           has("/map/map_projector_info", "autoware_map_msgs/msg/MapProjectorInfo") &&
           has("/gnss_pose", "geometry_msgs/msg/PoseStamped") &&
           has("/gnss_pose_cov", "geometry_msgs/msg/PoseWithCovarianceStamped") &&
           has("/gnss_fixed", "autoware_internal_debug_msgs/msg/BoolStamped") &&
           has("/tf", "tf2_msgs/msg/TFMessage");
  }

  // build_node() followed by the MGRS projector info.
  void build_node_with_mgrs_projector(const NodeParams & params)
  {
    ASSERT_NO_FATAL_FAILURE(build_node(params));
    send_projector_info(make_mgrs_projector_info());
  }

  // Sends each fix and waits until the node has processed it (its gnss_fixed has arrived), so the
  // next step starts with all of them processed.
  void send_fixes_and_wait(const std::vector<NavSatFix> & fixes)
  {
    for (const auto & fix : fixes) {
      const std::size_t processed = peer_->fixed_flags.size();
      send_fix(fix);
      ASSERT_NO_FATAL_FAILURE(wait_for_gnss_fixed(processed + 1));
    }
  }

  // A node whose base_frame is my_base, with the INS orientation yaw pi/2 and the static TF
  // my_antenna -> my_base: translation (1, 2, 0.5), no rotation.
  void build_node_with_static_antenna_tf()
  {
    NodeParams params;
    params.gnss_pose_pub_method = 0;
    params.use_gnss_ins_orientation = true;
    params.base_frame = "my_base";
    ASSERT_NO_FATAL_FAILURE(build_node_with_mgrs_projector(params));
    send_orientation(make_orientation(M_PI / 2.0));
    ASSERT_NO_FATAL_FAILURE(broadcast_static_tf(
      "my_antenna", "my_base", make_point(1.0, 2.0, 0.5), yaw_to_quaternion(0.0)));
  }

  // A node whose base_frame is my_base, with the INS orientation yaw 0 and the TF
  // my_antenna -> my_base, translation (1, 0, 0), published on /tf at 1000 s only.
  void build_node_with_timed_antenna_tf()
  {
    NodeParams params;
    params.gnss_pose_pub_method = 0;
    params.use_gnss_ins_orientation = true;
    params.base_frame = "my_base";
    ASSERT_NO_FATAL_FAILURE(build_node_with_mgrs_projector(params));
    send_orientation(make_orientation(0.0));
    ASSERT_NO_FATAL_FAILURE(broadcast_timed_tf(
      "my_antenna", "my_base", make_point(1.0, 0.0, 0.0), yaw_to_quaternion(0.0),
      make_stamp(1000, 0)));
  }

  // A node whose base_frame is my_base, with the INS orientation yaw 0 and a TF
  // my_antenna -> my_base on /tf that changes between 1000 s and 1002 s.
  void build_node_with_changing_antenna_tf()
  {
    NodeParams params;
    params.gnss_pose_pub_method = 0;
    params.use_gnss_ins_orientation = true;
    params.base_frame = "my_base";
    ASSERT_NO_FATAL_FAILURE(build_node_with_mgrs_projector(params));
    send_orientation(make_orientation(0.0));
    // The relation moves 1 m along x and turns by 90 degrees between t0 and t1.
    ASSERT_NO_FATAL_FAILURE(broadcast_timed_tf(
      "my_antenna", "my_base", make_point(1.0, 0.0, 0.0), yaw_to_quaternion(0.0),
      make_stamp(1000, 0)));
    ASSERT_NO_FATAL_FAILURE(broadcast_timed_tf(
      "my_antenna", "my_base", make_point(2.0, 0.0, 0.0), yaw_to_quaternion(M_PI / 2.0),
      make_stamp(1002, 0)));
  }

  const PoseStamped & last_pose() const { return peer_->poses.back(); }
  const PoseWithCovarianceStamped & last_pose_cov() const { return peer_->pose_cov_msgs.back(); }
  const BoolStamped & last_fixed() const { return peer_->fixed_flags.back(); }
  const TransformStamped & last_tf() const { return peer_->broadcast_tfs.back(); }

  std::shared_ptr<PeerNode> peer_;
  std::shared_ptr<autoware::gnss_poser::GnssPoserNode> node_;
  std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> executor_;
};

// The endpoint gnss_poser owns among `infos`; throws when there is none.
rclcpp::TopicEndpointInfo gnss_poser_endpoint(const std::vector<rclcpp::TopicEndpointInfo> & infos)
{
  for (const auto & info : infos) {
    if (info.node_name() == "gnss_poser") {
      return info;
    }
  }
  throw std::runtime_error("no endpoint owned by gnss_poser");
}

// A parameter name for an int value: "Minus1" for -1, "3" for 3.
std::string int_param_name(int value)
{
  return (value < 0 ? "Minus" : "") + std::to_string(std::abs(value));
}

class GnssPoserMissingParameter : public GnssPoserNodeIntegration,
                                  public ::testing::WithParamInterface<std::string>
{
};

// A parameter and an override of the wrong type for it.
struct WrongTypedParameter
{
  std::string name;
  rclcpp::ParameterValue value;
};

std::ostream & operator<<(std::ostream & os, const WrongTypedParameter & parameter)
{
  return os << parameter.name << " = " << rclcpp::to_string(parameter.value);
}

class GnssPoserWrongParameterType : public GnssPoserNodeIntegration,
                                    public ::testing::WithParamInterface<WrongTypedParameter>
{
};

class GnssPoserUnknownPubMethod : public GnssPoserNodeIntegration,
                                  public ::testing::WithParamInterface<int>
{
};

class GnssPoserBuffEpochBelowOne : public GnssPoserNodeIntegration,
                                   public ::testing::WithParamInterface<std::tuple<int, int>>
{
};

}  // namespace

// =======================================================================================
// 1. Construction and ROS interface
// =======================================================================================

// The node is named `gnss_poser` in the root namespace and declares the six parameters the README
// documents; every override is read back unchanged.
TEST_F(GnssPoserNodeIntegration, Construct_WithAllParameters_DeclaresThemAndIsNamedGnssPoser)
{
  // Arrange
  NodeParams params;
  params.base_frame = "my_base";
  params.gnss_base_frame = "my_gnss_base";
  params.map_frame = "my_map";
  params.use_gnss_ins_orientation = false;
  params.gnss_pose_pub_method = 2;
  params.buff_epoch = 7;

  // Act
  ASSERT_NO_FATAL_FAILURE(build_node(params));

  // Assert
  EXPECT_STREQ(node_->get_name(), "gnss_poser");
  EXPECT_STREQ(node_->get_namespace(), "/");

  EXPECT_EQ(node_->get_parameter("base_frame").as_string(), "my_base");
  EXPECT_EQ(node_->get_parameter("gnss_base_frame").as_string(), "my_gnss_base");
  EXPECT_EQ(node_->get_parameter("map_frame").as_string(), "my_map");
  EXPECT_EQ(node_->get_parameter("use_gnss_ins_orientation").as_bool(), false);
  EXPECT_EQ(node_->get_parameter("gnss_pose_pub_method").as_int(), 2);
  EXPECT_EQ(node_->get_parameter("buff_epoch").as_int(), 7);
}

// The node declares all six parameters without a built-in default: a configuration that lacks any
// of them makes the node fail to start (constructing the component throws) instead of running with
// a default. The values README.md lists as defaults live in config/gnss_poser.param.yaml, which the
// next case pins. Only "fails to start" is pinned; which exception rclcpp throws is not part of the
// contract.
TEST_P(GnssPoserMissingParameter, Construct_MissingAnyRequiredParameter_FailsToStart)
{
  // Arrange
  const rclcpp::NodeOptions options = NodeParams{}.to_options(GetParam());

  // Act & Assert
  EXPECT_THROW(std::make_shared<autoware::gnss_poser::GnssPoserNode>(options), std::exception);
}

INSTANTIATE_TEST_SUITE_P(
  EveryParameter, GnssPoserMissingParameter, ::testing::ValuesIn(NodeParams::names()),
  [](const ::testing::TestParamInfo<std::string> & info) { return info.param; });

// The node declares each parameter with a type: an override of the wrong type (an integer for the
// strings, a string for the boolean and for the integers) makes the node fail to start. As above,
// only "fails to start" is pinned, not the exception rclcpp throws.
TEST_P(GnssPoserWrongParameterType, Construct_WrongParameterType_FailsToStart)
{
  // Arrange
  rclcpp::NodeOptions options = NodeParams{}.to_options(GetParam().name);
  options.append_parameter_override(GetParam().name, GetParam().value);

  // Act & Assert
  EXPECT_THROW(std::make_shared<autoware::gnss_poser::GnssPoserNode>(options), std::exception);
}

INSTANTIATE_TEST_SUITE_P(
  EveryParameter, GnssPoserWrongParameterType,
  ::testing::Values(
    WrongTypedParameter{"base_frame", rclcpp::ParameterValue(123)},
    WrongTypedParameter{"gnss_base_frame", rclcpp::ParameterValue(123)},
    WrongTypedParameter{"map_frame", rclcpp::ParameterValue(123)},
    WrongTypedParameter{"use_gnss_ins_orientation", rclcpp::ParameterValue("not_a_bool")},
    WrongTypedParameter{"gnss_pose_pub_method", rclcpp::ParameterValue("not_an_int")},
    WrongTypedParameter{"buff_epoch", rclcpp::ParameterValue("not_an_int")}),
  [](const ::testing::TestParamInfo<WrongTypedParameter> & info) { return info.param.name; });

// `gnss_pose_pub_method` accepts exactly 0 (instant), 1 (average) and 2 (median). Any other value
// makes the node fail to start instead of being silently treated as one of them (before this
// validation, every non-zero value buffered and every value other than 1 selected the median).
TEST_P(GnssPoserUnknownPubMethod, Construct_UnknownPubMethod_FailsToStart)
{
  // Arrange
  NodeParams params;
  params.gnss_pose_pub_method = GetParam();
  params.buff_epoch = 3;

  // Act & Assert
  EXPECT_THROW(
    std::make_shared<autoware::gnss_poser::GnssPoserNode>(params.to_options()), std::exception);
}

INSTANTIATE_TEST_SUITE_P(
  OutOfRange, GnssPoserUnknownPubMethod, ::testing::Values(-1, 3, 42),
  [](const ::testing::TestParamInfo<int> & info) { return int_param_name(info.param); });

// `buff_epoch` must be at least 1 whatever the method: 0 and negative values make the node fail to
// start. Before this validation a zero-capacity buffer counted as "full", so the average published
// NaN coordinates and the median threw inside the callback.
TEST_P(GnssPoserBuffEpochBelowOne, Construct_BuffEpochBelowOne_FailsToStart)
{
  // Arrange
  NodeParams params;
  params.gnss_pose_pub_method = std::get<0>(GetParam());
  params.buff_epoch = std::get<1>(GetParam());

  // Act & Assert
  EXPECT_THROW(
    std::make_shared<autoware::gnss_poser::GnssPoserNode>(params.to_options()), std::exception);
}

INSTANTIATE_TEST_SUITE_P(
  EveryMethod, GnssPoserBuffEpochBelowOne,
  ::testing::Combine(::testing::Values(0, 1, 2), ::testing::Values(0, -1)),
  [](const ::testing::TestParamInfo<std::tuple<int, int>> & info) {
    return "Method" + int_param_name(std::get<0>(info.param)) + "_BuffEpoch" +
           int_param_name(std::get<1>(info.param));
  });

// The node has no built-in defaults: the values README.md and the schema document as defaults live
// in config/gnss_poser.param.yaml, which the launch file loads. That file must keep constructing
// the node and read back exactly as documented.
TEST_F(GnssPoserNodeIntegration, Construct_WithShippedParamFile_MatchesDocumentedDefaults)
{
  // Arrange
  rclcpp::NodeOptions options;
  options.arguments(
    {"--ros-args", "--params-file", GNSS_POSER_CONFIG_DIR "/gnss_poser.param.yaml"});

  // Act
  const auto node = std::make_shared<autoware::gnss_poser::GnssPoserNode>(options);

  // Assert
  EXPECT_EQ(node->get_parameter("base_frame").as_string(), "base_link");
  EXPECT_EQ(node->get_parameter("gnss_base_frame").as_string(), "gnss_base_link");
  EXPECT_EQ(node->get_parameter("map_frame").as_string(), "map");
  EXPECT_TRUE(node->get_parameter("use_gnss_ins_orientation").as_bool());
  EXPECT_EQ(node->get_parameter("gnss_pose_pub_method").as_int(), 0);
  EXPECT_EQ(node->get_parameter("buff_epoch").as_int(), 1);
}

// The node's ROS surface: three subscriptions, three publishers and the /tf broadcast, with their
// message types, reliability and durability. `/map/map_projector_info` is the one transient-local
// subscription.
//
// Queue depths are not pinned: they are not visible through discovery.
TEST_F(GnssPoserNodeIntegration, Interface_TopicsAndQos)
{
  // Arrange
  ASSERT_NO_FATAL_FAILURE(build_node({}));

  ASSERT_TRUE(pump_until([this] { return all_topics_discovered(); }, 10s));

  // Act
  // Subscriptions
  const auto fix_sub = gnss_poser_endpoint(peer_->get_subscriptions_info_by_topic("/fix"));
  const auto orientation_sub =
    gnss_poser_endpoint(peer_->get_subscriptions_info_by_topic("/autoware_orientation"));
  const auto projector_sub =
    gnss_poser_endpoint(peer_->get_subscriptions_info_by_topic("/map/map_projector_info"));
  // Publishers
  const auto pose_pub = gnss_poser_endpoint(peer_->get_publishers_info_by_topic("/gnss_pose"));
  const auto pose_cov_pub =
    gnss_poser_endpoint(peer_->get_publishers_info_by_topic("/gnss_pose_cov"));
  const auto fixed_pub = gnss_poser_endpoint(peer_->get_publishers_info_by_topic("/gnss_fixed"));

  // Assert
  using rclcpp::DurabilityPolicy;
  using rclcpp::ReliabilityPolicy;
  expect_qos(fix_sub, ReliabilityPolicy::Reliable, DurabilityPolicy::Volatile);
  expect_qos(orientation_sub, ReliabilityPolicy::Reliable, DurabilityPolicy::Volatile);
  expect_qos(projector_sub, ReliabilityPolicy::Reliable, DurabilityPolicy::TransientLocal);
  expect_qos(pose_pub, ReliabilityPolicy::Reliable, DurabilityPolicy::Volatile);
  expect_qos(pose_cov_pub, ReliabilityPolicy::Reliable, DurabilityPolicy::Volatile);
  expect_qos(fixed_pub, ReliabilityPolicy::Reliable, DurabilityPolicy::Volatile);
}

// =======================================================================================
// 2. Gating: when does a NavSatFix produce output at all?
// =======================================================================================

// Until a usable projector has been received, a fix produces nothing at all, not even `gnss_fixed`.
// The arrival of a projector publishes nothing by itself either: dropped fixes are not replayed,
// the next fix is simply the first one processed.
TEST_F(GnssPoserNodeIntegration, Gate_BeforeMapProjectorInfo_PublishesNothing)
{
  // Arrange
  ASSERT_NO_FATAL_FAILURE(build_node({}));

  // Act
  // No projector info yet: the fix is dropped, and not even gnss_fixed is published.
  send_fix(make_reference_fix());

  // Assert
  expect_output_counts(0, 0, 0, 0);
}

TEST_F(GnssPoserNodeIntegration, Gate_FixUnderLocalProjector_PublishesNothing)
{
  // Arrange
  ASSERT_NO_FATAL_FAILURE(build_node({}));
  // No projector info yet: the fix is dropped, and not even gnss_fixed is published.
  send_fix(make_reference_fix());
  // A LOCAL projector arriving changes nothing, neither by itself nor for the next fix.
  send_projector_info(make_local_projector_info());

  // Act
  send_fix(make_reference_fix());

  // Assert
  expect_output_counts(0, 0, 0, 0);
}

TEST_F(GnssPoserNodeIntegration, Gate_MgrsProjectorArrival_DoesNotReplayDroppedFixes)
{
  // Arrange
  ASSERT_NO_FATAL_FAILURE(build_node({}));
  // No projector info yet: the fix is dropped, and not even gnss_fixed is published.
  send_fix(make_reference_fix());
  // A LOCAL projector arriving changes nothing, neither by itself nor for the next fix.
  send_projector_info(make_local_projector_info());
  send_fix(make_reference_fix());

  // Act
  // A usable projector arriving publishes nothing by itself: dropped fixes are not replayed.
  send_projector_info(make_mgrs_projector_info());

  // Assert
  expect_output_counts(0, 0, 0, 0);
}

TEST_F(GnssPoserNodeIntegration, Gate_FirstFixAfterMgrsProjector_PublishesOneOfEachOutput)
{
  // Arrange
  ASSERT_NO_FATAL_FAILURE(build_node({}));
  // No projector info yet: the fix is dropped, and not even gnss_fixed is published.
  send_fix(make_reference_fix());
  // A LOCAL projector arriving changes nothing, neither by itself nor for the next fix.
  send_projector_info(make_local_projector_info());
  send_fix(make_reference_fix());
  // A usable projector arriving publishes nothing by itself: dropped fixes are not replayed.
  send_projector_info(make_mgrs_projector_info());

  // Act
  // The next fix is processed: exactly one of each output.
  send_fix(make_reference_fix());

  // Assert
  ASSERT_NO_FATAL_FAILURE(wait_for_outputs(1));
  expect_output_counts(1, 1, 1, 1);
}

// A fix without a position solution (STATUS_NO_FIX) publishes `gnss_fixed = false` and nothing
// else.
TEST_F(GnssPoserNodeIntegration, Gate_StatusNoFix_PublishesOnlyGnssFixedFalse)
{
  // Arrange
  ASSERT_NO_FATAL_FAILURE(build_node({}));
  send_projector_info(make_mgrs_projector_info());

  // Act
  send_fix(make_reference_fix(NavSatStatus::STATUS_NO_FIX));

  // Assert
  ASSERT_NO_FATAL_FAILURE(wait_for_gnss_fixed(1));
  expect_output_counts(1, 0, 0, 0);
  EXPECT_FALSE(last_fixed().data);
}

// `gnss_fixed` carries the header stamp of the fix it reports on, like every other output.
TEST_F(GnssPoserNodeIntegration, GnssFixed_StampIsFixHeaderStamp)
{
  // Arrange
  ASSERT_NO_FATAL_FAILURE(build_node({}));
  send_projector_info(make_mgrs_projector_info());

  // Act
  send_fix(make_reference_fix(NavSatStatus::STATUS_NO_FIX));

  // Assert
  ASSERT_NO_FATAL_FAILURE(wait_for_gnss_fixed(1));
  EXPECT_EQ(last_fixed().stamp, fix_stamp());
}

// =======================================================================================
// 3. Position pipeline with gnss_pose_pub_method = 0 (instantaneous)
// =======================================================================================

// With gnss_pose_pub_method = 0, `gnss_pose` is the MGRS projection of the fix as-is: header stamp
// copied from the fix, `frame_id` = map_frame, `z` = altitude (WGS84 -> WGS84 is the identity), and
// `gnss_pose_cov` carries the same header and pose. The coordinates are pinned twice: as golden
// values recorded from the current implementation, and as the output of the same library call the
// node makes.
//
// No antenna -> base_link TF is available here, so the antenna position is published unchanged;
// that fallback, and the composition when a TF exists, are pinned by the TF cases. Orientation is
// left to the orientation cases.
TEST_F(GnssPoserNodeIntegration, MethodInstant_PublishesProjectedAntennaPosition)
{
  // Arrange
  NodeParams params;
  params.gnss_pose_pub_method = 0;
  params.map_frame = "my_map";  // non-default, so that frame_id is shown to come from the parameter
  ASSERT_NO_FATAL_FAILURE(build_node(params));
  const auto projector = make_mgrs_projector_info();
  send_projector_info(projector);

  const auto fix = make_reference_fix();
  const auto expected_antenna = project_antenna(fix, projector);

  // Act
  send_fix(fix);

  // Assert
  ASSERT_NO_FATAL_FAILURE(wait_for_outputs(1));
  const auto & pose = last_pose();
  EXPECT_EQ(pose.header.frame_id, "my_map");
  EXPECT_EQ(pose.header.stamp, fix.header.stamp);
  // Golden values recorded from the current implementation: a change in the library shows here.
  expect_point_near(pose.pose.position, make_point(golden_x, golden_y, golden_z));
  // The same library call the node composes: a change in how the node calls it shows here.
  expect_point_near(pose.pose.position, expected_antenna);
  // z is the altitude passed through (MGRS ignores it, WGS84 -> WGS84 is the identity).
  EXPECT_NEAR(pose.pose.position.z, reference_altitude, position_tolerance);

  // gnss_pose_cov carries a copy of the same header and pose, hence exact equality.
  const auto & pose_cov = last_pose_cov();
  EXPECT_EQ(pose_cov.header, pose.header);
  EXPECT_EQ(pose_cov.pose.pose, pose.pose);
}

// =======================================================================================
// 4. Position buffering (gnss_pose_pub_method = 1 average / 2 median)
// =======================================================================================

// With method 1 nothing is published until `buff_epoch` fixes have been buffered (each of them
// still publishes `gnss_fixed`); from then on every fix publishes the mean of the last `buff_epoch`
// positions as a sliding window.
TEST_F(GnssPoserNodeIntegration, MethodAverage_BufferNotFull_PublishesOnlyGnssFixed)
{
  // Arrange
  ASSERT_NO_FATAL_FAILURE(build_node_with_mgrs_projector(make_buffer_params(1, 3)));
  const auto fixes = make_average_fixes();
  // Fixes 1 and 2 only fill the buffer: each publishes gnss_fixed, neither publishes a pose.
  ASSERT_NO_FATAL_FAILURE(send_fixes_and_wait({fixes[0]}));

  // Act
  send_fix(fixes[1]);

  // Assert
  ASSERT_NO_FATAL_FAILURE(wait_for_gnss_fixed(2));
  expect_output_counts(2, 0, 0, 0);
}

// =======================================================================================
// 5. TF handling (antenna -> base_link) and TF broadcast (map -> gnss_base_link)
// =======================================================================================

// The published pose is map->base = map->antenna * antenna->base: the antenna orientation rotates
// the lever arm and the yaws add up. Non-default frame names pin which parameter names which frame.
// The /tf broadcast map_frame -> gnss_base_frame mirrors the pose exactly at the fix stamp, and one
// processed fix yields exactly one of each output.
TEST_F(GnssPoserNodeIntegration, Tf_StaticAntennaToBaseTransform_IsComposedAndBroadcast)
{
  // Arrange
  NodeParams params;
  params.gnss_pose_pub_method = 0;
  params.use_gnss_ins_orientation = true;
  params.base_frame = "my_base";
  params.gnss_base_frame = "my_gnss_base";
  params.map_frame = "my_map";
  ASSERT_NO_FATAL_FAILURE(build_node(params));
  const auto projector = make_mgrs_projector_info();
  send_projector_info(projector);
  send_orientation(make_orientation(M_PI / 2.0));

  // TF: my_antenna -> my_base, translation (1, 2, 0.5), rotation yaw +90deg.
  ASSERT_NO_FATAL_FAILURE(broadcast_static_tf(
    "my_antenna", "my_base", make_point(1.0, 2.0, 0.5), yaw_to_quaternion(M_PI / 2.0)));

  const auto fix = make_reference_fix(NavSatStatus::STATUS_FIX, "my_antenna");
  // map->base = map->antenna * antenna->base. Antenna yaw 90deg rotates (1,2,0.5) into
  // (-2, 1, 0.5); yaws add up to 180deg.
  const auto antenna = project_antenna(fix, projector);
  const auto expected_position = make_point(antenna.x - 2.0, antenna.y + 1.0, antenna.z + 0.5);

  // Act
  send_fix(fix);

  // Assert
  ASSERT_NO_FATAL_FAILURE(wait_for_outputs(1));
  const auto & pose = last_pose();
  expect_point_near(pose.pose.position, expected_position);
  EXPECT_NEAR(std::abs(yaw_of(pose.pose.orientation)), M_PI, angle_tolerance);
  EXPECT_EQ(pose.header.frame_id, "my_map");

  // The broadcast TF mirrors the pose exactly: my_map -> my_gnss_base at the fix stamp.
  const auto & tf = last_tf();
  EXPECT_EQ(tf.header.frame_id, "my_map");
  EXPECT_EQ(tf.child_frame_id, "my_gnss_base");
  EXPECT_EQ(tf.header.stamp, fix.header.stamp);
  // The transform is a field-by-field copy of the pose, hence exact equality on every component.
  EXPECT_EQ(tf.transform.translation.x, pose.pose.position.x);
  EXPECT_EQ(tf.transform.translation.y, pose.pose.position.y);
  EXPECT_EQ(tf.transform.translation.z, pose.pose.position.z);
  EXPECT_EQ(tf.transform.rotation, pose.pose.orientation);

  // Exactly one of each output per processed fix.
  EXPECT_EQ(peer_->fixed_flags.size(), 1U);
  EXPECT_EQ(peer_->poses.size(), 1U);
  EXPECT_EQ(peer_->pose_cov_msgs.size(), 1U);
  EXPECT_EQ(peer_->broadcast_tfs.size(), 1U);
}

// The antenna frame is the fix's `header.frame_id`. A frame with no transform to base_frame, or a
// frame equal to base_frame, yields the antenna pose unchanged (identity fallback); a known frame
// gets the transform applied.
//
// NOTE(characterization): the unknown-frame case is a configuration error (the driver's frame_id
// does not match any TF frame) that never resolves by waiting, yet the node keeps publishing the
// antenna position as base_link, with the receiver's covariance and only a throttled warning. The
// node does not distinguish this from a transform that is merely not available yet. The
// frame_id == base_frame shortcut, by contrast, is a legitimate configuration.
TEST_F(GnssPoserNodeIntegration, Tf_AntennaFrameComesFromFixHeader_UnknownFrameFallsBackToIdentity)
{
  // Arrange
  ASSERT_NO_FATAL_FAILURE(build_node_with_static_antenna_tf());
  const auto fix_unknown = make_reference_fix(NavSatStatus::STATUS_FIX, "some_other_antenna");

  // Act
  // Frame from the header is what gets looked up; a frame without TF yields the antenna pose.
  send_fix(fix_unknown);

  // Assert
  ASSERT_NO_FATAL_FAILURE(wait_for_outputs(1));
  expect_point_near(last_pose().pose.position, project_mgrs(fix_unknown));
  EXPECT_NEAR(yaw_of(last_pose().pose.orientation), M_PI / 2.0, angle_tolerance);
}

TEST_F(GnssPoserNodeIntegration, Tf_AntennaFrameComesFromFixHeader_BaseFrameUsesIdentity)
{
  // Arrange
  ASSERT_NO_FATAL_FAILURE(build_node_with_static_antenna_tf());
  const auto fix_base = make_reference_fix(NavSatStatus::STATUS_FIX, "my_base");

  // Act
  // Same frame as base_frame: no lookup, identity.
  send_fix(fix_base);

  // Assert
  ASSERT_NO_FATAL_FAILURE(wait_for_outputs(1));
  expect_point_near(last_pose().pose.position, project_mgrs(fix_base));
}

// The antenna -> base transform is looked up at the fix's header stamp, not at "latest". With a
// transform published on /tf (time-stamped, unlike /tf_static) at exactly t0: a fix stamped t0 gets
// it applied, a fix stamped one second later needs extrapolation, which tf2 refuses, so the node
// falls back to identity, and a fix with a zero stamp means "latest" to tf2 and gets it applied
// again.
//
// The antenna sits rigidly on the vehicle and is normally published on /tf_static, where time is
// ignored; a time-stamped transform is the only way to observe which time the node asks for. What
// this pins is the lookup policy, which decides how the node reacts when the receiver's clock and
// the TF clock disagree.
TEST_F(GnssPoserNodeIntegration, Tf_LookupIsAtFixHeaderStamp_FixAtTransformStampAppliesIt)
{
  // Arrange
  ASSERT_NO_FATAL_FAILURE(build_node_with_timed_antenna_tf());
  const auto projected = project_mgrs(make_reference_fix());

  // Act
  // Fix stamped exactly at t0: transform found and applied.
  send_fix(make_antenna_fix_at(make_stamp(1000, 0)));

  // Assert
  ASSERT_NO_FATAL_FAILURE(wait_for_outputs(1));
  expect_point_near(
    last_pose().pose.position, make_point(projected.x + 1.0, projected.y, projected.z));
}

TEST_F(GnssPoserNodeIntegration, Tf_LookupIsAtFixHeaderStamp_LaterFixFallsBackToIdentity)
{
  // Arrange
  ASSERT_NO_FATAL_FAILURE(build_node_with_timed_antenna_tf());
  const auto projected = project_mgrs(make_reference_fix());

  // Act
  // Fix stamped 1 s later: lookup needs extrapolation, fails, and falls back to identity.
  send_fix(make_antenna_fix_at(make_stamp(1001, 0)));

  // Assert
  ASSERT_NO_FATAL_FAILURE(wait_for_outputs(1));
  expect_point_near(last_pose().pose.position, projected);
  EXPECT_EQ(last_pose().header.stamp, make_stamp(1001, 0));
}

// When the antenna -> base relation changes over time, each pose follows the relation at its own
// fix stamp, interpolated between the two neighboring transforms: a fix halfway between two
// transforms 1 m apart is offset by the midpoint, and the yaw is interpolated as well.
//
// The relation is rigid in practice; this pins that the node does not cache or latch a transform
// but resolves it per fix, so a moving relation (a calibration rig, a trailer) would be honored.
TEST_F(GnssPoserNodeIntegration, Tf_ChangingRelationIsInterpolatedAtEachFixStamp)
{
  // Arrange
  ASSERT_NO_FATAL_FAILURE(build_node_with_changing_antenna_tf());
  const auto projected = project_mgrs(make_reference_fix());

  // Act
  // Halfway between, tf2 interpolates: translation 1.5 m, yaw 45 degrees. Going back in time is
  // fine, the buffer keeps both transforms.
  // First stamp: make_stamp(1000, 0), second stamp: make_stamp(1002, 0).
  // Here `make_stamp(1001, 0)` is halfway between the first and second stamps.
  // See `build_node_with_changing_antenna_tf` for details.
  send_fix(make_antenna_fix_at(make_stamp(1001, 0)));

  // Assert
  ASSERT_NO_FATAL_FAILURE(wait_for_outputs(1));
  expect_point_near(
    last_pose().pose.position, make_point(projected.x + 1.5, projected.y, projected.z));
  EXPECT_NEAR(yaw_of(last_pose().pose.orientation), M_PI / 4.0, angle_tolerance);
}

// =======================================================================================
// 6. Cases kept from the node test that came with the package
// =======================================================================================

// The orientation source selected by use_gnss_ins_orientation = false reaches the logic: moving
// north yields a yaw of about pi/2.
TEST_F(GnssPoserNodeIntegration, TestOrientationSources)
{
  // Arrange
  NodeParams params;
  params.use_gnss_ins_orientation = false;
  params.buff_epoch = 1;
  params.gnss_pose_pub_method = 0;
  ASSERT_NO_FATAL_FAILURE(build_node_with_mgrs_projector(params));
  ASSERT_NO_FATAL_FAILURE(send_fixes_and_wait({make_fix(35.681236, 139.767125, 41.0)}));

  // Act
  // Publish second position point (moving north)
  send_fix(make_fix(35.682236, 139.767125, 41.0));

  // Assert
  ASSERT_NO_FATAL_FAILURE(wait_for_outputs(2));
  // Moving north should result in yaw angle of approximately 90 degrees (PI/2)
  EXPECT_NEAR(std::abs(yaw_of(last_pose().pose.orientation)), M_PI / 2.0, 0.1);
}

// The median method and buff_epoch reach the logic: the pose is published once five fixes are
// buffered.
TEST_F(GnssPoserNodeIntegration, TestMedianPosition)
{
  // Arrange
  NodeParams params;
  params.use_gnss_ins_orientation = true;
  params.buff_epoch = 5;            // Use 5 points
  params.gnss_pose_pub_method = 2;  // Use median method
  ASSERT_NO_FATAL_FAILURE(build_node_with_mgrs_projector(params));
  send_orientation(make_orientation(0.0, 0.01, 0.01, 0.01));
  // Use deliberately placed outlier points to test if median calculation works
  ASSERT_NO_FATAL_FAILURE(send_fixes_and_wait(
    {make_fix(35.681236, 139.767125, 41.0), make_fix(35.681237, 139.767127, 41.1),
     make_fix(35.681240, 139.767130, 41.5), make_fix(35.681230, 139.767120, 40.8)}));

  // Act
  send_fix(make_fix(35.681235, 139.767123, 41.2));

  // Assert
  // Wait for position message (should be published after buffer is filled)
  ASSERT_NO_FATAL_FAILURE(wait_for_outputs(1));
  EXPECT_NE(last_pose().pose.position.x, 0.0);
  EXPECT_NE(last_pose().pose.position.y, 0.0);
}

// The covariance the logic derives reaches gnss_pose_cov: the fix diagonal for DIAGONAL_KNOWN.
TEST_F(GnssPoserNodeIntegration, TestCovarianceProcessing_DiagonalKnown)
{
  // Arrange
  ASSERT_NO_FATAL_FAILURE(build_node_with_mgrs_projector({}));
  send_orientation(make_orientation(0.0, 0.01, 0.01, 0.01));
  auto nav_sat_fix = make_fix(35.681236, 139.767125, 41.0);
  nav_sat_fix.position_covariance_type = NavSatFix::COVARIANCE_TYPE_DIAGONAL_KNOWN;
  nav_sat_fix.position_covariance = {2.0, 0.0, 0.0, 0.0, 3.0, 0.0, 0.0, 0.0, 4.0};

  // Act
  send_fix(nav_sat_fix);

  // Assert
  ASSERT_NO_FATAL_FAILURE(wait_for_outputs(1));
  // Verify covariance values are correctly passed
  EXPECT_NEAR(last_pose_cov().pose.covariance[0], 2.0, 1e-6);
  EXPECT_NEAR(last_pose_cov().pose.covariance[7], 3.0, 1e-6);
  EXPECT_NEAR(last_pose_cov().pose.covariance[14], 4.0, 1e-6);
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  const int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
