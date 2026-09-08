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
// Characterization tests for autoware::gnss_poser::GNSSPoser.
//
// These tests pin down the *currently observable* behavior of the node as seen through its
// public ROS interface (parameters, topics, TF) so that a later refactoring can be verified
// to preserve it. They deliberately describe what the node does today, not what it should
// do. Where the observed behavior looks like a latent defect it is still asserted as-is and
// marked with "NOTE(characterization)" so that a follow-up phase can decide whether to keep
// or change it, and update the corresponding test on purpose.
//
// Test naming: <Aspect>_<Condition>_<ObservedBehavior>
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
#include <cassert>
#include <chrono>
#include <cmath>
#include <exception>
#include <functional>
#include <memory>
#include <stdexcept>
#include <string>
#include <thread>
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
// EGM2008 geoid height at the reference point, from GeographicLib's GeoidEval on the egm2008-1
// dataset: independent of the node and of autoware_geography_utils.
constexpr double reference_geoid_height = 36.12;  // [m]
// The library and GeoidEval interpolate the same grid; 1 cm covers their difference while any
// missing (0 m) or wrongly signed (+36 m) conversion is far outside.
constexpr double geoid_height_tolerance = 0.01;  // [m]

// A deterministic, clearly artificial header stamp so that "stamp copied from input" and
// "stamp taken from the node clock" can be told apart.
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

// GnssInsOrientation::rmse_rotation_* are float32, so rmse^2 carries single-precision error.
constexpr double rmse_squared_tolerance = 1e-6;

// Covariance index helpers (6x6 row-major).
constexpr std::size_t cov_xx = 0;
constexpr std::size_t cov_yy = 7;
constexpr std::size_t cov_zz = 14;
constexpr std::size_t cov_roll_roll = 21;
constexpr std::size_t cov_pitch_pitch = 28;
constexpr std::size_t cov_yaw_yaw = 35;

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

MapProjectorInfo make_local_cartesian_utm_projector_info(
  double origin_lat, double origin_lon, double origin_alt)
{
  MapProjectorInfo msg;
  msg.projector_type = MapProjectorInfo::LOCAL_CARTESIAN_UTM;
  msg.vertical_datum = MapProjectorInfo::WGS84;
  msg.map_origin.latitude = origin_lat;
  msg.map_origin.longitude = origin_lon;
  msg.map_origin.altitude = origin_alt;
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

// Two orientations agree when the rotation angle between them, 2 * acos(|q1 . q2|), is negligible.
void expect_same_rotation(
  const Quaternion & actual, const Quaternion & expected, double tol = angle_tolerance)
{
  const double dot =
    actual.x * expected.x + actual.y * expected.y + actual.z * expected.z + actual.w * expected.w;
  const double angle = 2.0 * std::acos(std::min(1.0, std::abs(dot)));
  EXPECT_LE(angle, tol) << "actual=(" << actual.x << "," << actual.y << "," << actual.z << ","
                        << actual.w << ") expected=(" << expected.x << "," << expected.y << ","
                        << expected.z << "," << expected.w << ")";
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

Point mean_of(const std::vector<Point> & points)
{
  assert(!points.empty() && "mean_of requires at least one point");
  Point mean = make_point(0.0, 0.0, 0.0);
  for (const auto & p : points) {
    mean.x += p.x;
    mean.y += p.y;
    mean.z += p.z;
  }
  const auto n = static_cast<double>(points.size());
  return make_point(mean.x / n, mean.y / n, mean.z / n);
}

double median_of(std::vector<double> values)
{
  assert(!values.empty() && "median_of requires at least one value");
  std::sort(values.begin(), values.end());
  const std::size_t mid = values.size() / 2;
  return (values.size() % 2 == 1) ? values[mid] : (values[mid] + values[mid - 1]) / 2.0;
}

// Component-wise median (this is what the node computes, NOT the median sample).
Point component_wise_median_of(const std::vector<Point> & points)
{
  std::vector<double> xs;
  std::vector<double> ys;
  std::vector<double> zs;
  for (const auto & p : points) {
    xs.push_back(p.x);
    ys.push_back(p.y);
    zs.push_back(p.z);
  }
  return make_point(median_of(xs), median_of(ys), median_of(zs));
}

void expect_point_near(
  const Point & actual, const Point & expected, double tol = position_tolerance)
{
  EXPECT_NEAR(actual.x, expected.x, tol);
  EXPECT_NEAR(actual.y, expected.y, tol);
  EXPECT_NEAR(actual.z, expected.z, tol);
}

bool is_egm2008_dataset_available()
{
  try {
    autoware::geography_utils::convert_wgs84_to_egm2008(0.0, 0.0, 0.0);
    return true;
  } catch (const std::runtime_error &) {
    return false;
  }
}

// ---------------------------------------------------------------------------------------
// Test double: a plain rclcpp node that plays the role of every peer of gnss_poser
// (map_projection_loader, GNSS driver, INS driver, robot_state_publisher, and the consumers).
// ---------------------------------------------------------------------------------------
class PeerNode : public rclcpp::Node
{
public:
  PeerNode()
  : rclcpp::Node("gnss_poser_characterization_peer"),
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
class GnssPoserCharacterization : public ::testing::Test
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
    node_ = std::make_shared<autoware::gnss_poser::GNSSPoser>(params.to_options());
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

  const PoseStamped & last_pose() const { return peer_->poses.back(); }
  const PoseWithCovarianceStamped & last_pose_cov() const { return peer_->pose_cov_msgs.back(); }
  const BoolStamped & last_fixed() const { return peer_->fixed_flags.back(); }
  const TransformStamped & last_tf() const { return peer_->broadcast_tfs.back(); }

  std::shared_ptr<PeerNode> peer_;
  std::shared_ptr<autoware::gnss_poser::GNSSPoser> node_;
  std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> executor_;
};

}  // namespace

// =======================================================================================
// 1. Construction and ROS interface
// =======================================================================================

// The node is named `gnss_poser` in the root namespace and declares the six parameters the README
// documents; every override is read back unchanged.
TEST_F(GnssPoserCharacterization, Construct_WithAllParameters_DeclaresThemAndIsNamedGnssPoser)
{
  NodeParams params;
  params.base_frame = "my_base";
  params.gnss_base_frame = "my_gnss_base";
  params.map_frame = "my_map";
  params.use_gnss_ins_orientation = false;
  params.gnss_pose_pub_method = 2;
  params.buff_epoch = 7;
  ASSERT_NO_FATAL_FAILURE(build_node(params));

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
TEST_F(GnssPoserCharacterization, Construct_MissingAnyRequiredParameter_FailsToStart)
{
  const NodeParams params;
  for (const auto & missing : NodeParams::names()) {
    EXPECT_THROW(
      std::make_shared<autoware::gnss_poser::GNSSPoser>(params.to_options(missing)), std::exception)
      << "missing parameter: " << missing;
  }
}

// The node declares each parameter with a type: an override of the wrong type (an integer for the
// strings, a string for the boolean and for the integers) makes the node fail to start. As above,
// only "fails to start" is pinned, not the exception rclcpp throws.
TEST_F(GnssPoserCharacterization, Construct_WrongParameterType_FailsToStart)
{
  const std::vector<std::pair<std::string, rclcpp::ParameterValue>> wrong_typed = {
    {"base_frame", rclcpp::ParameterValue(123)},
    {"gnss_base_frame", rclcpp::ParameterValue(123)},
    {"map_frame", rclcpp::ParameterValue(123)},
    {"use_gnss_ins_orientation", rclcpp::ParameterValue("not_a_bool")},
    {"gnss_pose_pub_method", rclcpp::ParameterValue("not_an_int")},
    {"buff_epoch", rclcpp::ParameterValue("not_an_int")},
  };
  for (const auto & [name, value] : wrong_typed) {
    rclcpp::NodeOptions options = NodeParams{}.to_options(name);
    options.append_parameter_override(name, value);
    EXPECT_THROW(std::make_shared<autoware::gnss_poser::GNSSPoser>(options), std::exception)
      << "parameter: " << name;
  }
}

// The node has no built-in defaults: the values README.md and the schema document as defaults live
// in config/gnss_poser.param.yaml, which the launch file loads. That file must keep constructing
// the node and read back exactly as documented.
TEST_F(GnssPoserCharacterization, Construct_WithShippedParamFile_MatchesDocumentedDefaults)
{
  rclcpp::NodeOptions options;
  options.arguments(
    {"--ros-args", "--params-file", GNSS_POSER_CONFIG_DIR "/gnss_poser.param.yaml"});
  const auto node = std::make_shared<autoware::gnss_poser::GNSSPoser>(options);

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
TEST_F(GnssPoserCharacterization, Interface_TopicsAndQos)
{
  ASSERT_NO_FATAL_FAILURE(build_node({}));

  const auto topics_ready = [this] {
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
  };
  ASSERT_TRUE(pump_until(topics_ready, 10s));

  const auto node_endpoint = [](const std::vector<rclcpp::TopicEndpointInfo> & infos) {
    for (const auto & info : infos) {
      if (info.node_name() == "gnss_poser") {
        return info;
      }
    }
    throw std::runtime_error("no endpoint owned by gnss_poser");
  };

  // Subscriptions
  {
    const auto sub = node_endpoint(peer_->get_subscriptions_info_by_topic("/fix"));
    EXPECT_EQ(sub.qos_profile().reliability(), rclcpp::ReliabilityPolicy::Reliable);
    EXPECT_EQ(sub.qos_profile().durability(), rclcpp::DurabilityPolicy::Volatile);
  }
  {
    const auto sub = node_endpoint(peer_->get_subscriptions_info_by_topic("/autoware_orientation"));
    EXPECT_EQ(sub.qos_profile().reliability(), rclcpp::ReliabilityPolicy::Reliable);
    EXPECT_EQ(sub.qos_profile().durability(), rclcpp::DurabilityPolicy::Volatile);
  }
  {
    const auto sub =
      node_endpoint(peer_->get_subscriptions_info_by_topic("/map/map_projector_info"));
    EXPECT_EQ(sub.qos_profile().reliability(), rclcpp::ReliabilityPolicy::Reliable);
    EXPECT_EQ(sub.qos_profile().durability(), rclcpp::DurabilityPolicy::TransientLocal);
  }
  // Publishers
  for (const auto & topic : {"/gnss_pose", "/gnss_pose_cov", "/gnss_fixed"}) {
    const auto pub = node_endpoint(peer_->get_publishers_info_by_topic(topic));
    EXPECT_EQ(pub.qos_profile().reliability(), rclcpp::ReliabilityPolicy::Reliable) << topic;
    EXPECT_EQ(pub.qos_profile().durability(), rclcpp::DurabilityPolicy::Volatile) << topic;
  }
}

// =======================================================================================
// 2. Gating: when does a NavSatFix produce output at all?
// =======================================================================================

// Until a usable projector has been received, a fix produces nothing at all, not even `gnss_fixed`.
// The arrival of a projector publishes nothing by itself either: dropped fixes are not replayed,
// the next fix is simply the first one processed.
TEST_F(GnssPoserCharacterization, Gate_BeforeMapProjectorInfo_PublishesNothing)
{
  ASSERT_NO_FATAL_FAILURE(build_node({}));

  // No projector info yet: the fix is dropped, and not even gnss_fixed is published.
  send_fix(make_reference_fix());
  expect_output_counts(0, 0, 0, 0);

  // A LOCAL projector arriving changes nothing, neither by itself nor for the next fix.
  send_projector_info(make_local_projector_info());
  expect_output_counts(0, 0, 0, 0);
  send_fix(make_reference_fix());
  expect_output_counts(0, 0, 0, 0);

  // A usable projector arriving publishes nothing by itself: dropped fixes are not replayed.
  send_projector_info(make_mgrs_projector_info());
  expect_output_counts(0, 0, 0, 0);

  // The next fix is processed: exactly one of each output.
  send_fix(make_reference_fix());
  ASSERT_NO_FATAL_FAILURE(wait_for_outputs(1));
  expect_output_counts(1, 1, 1, 1);
}

// A LOCAL projector switches processing off, and the latest projector info always wins: MGRS after
// LOCAL switches it back on, LOCAL after MGRS off again, and neither arrival publishes anything by
// itself.
//
// NOTE(characterization): unlike the not-fixed case below, a fix dropped under LOCAL does not
// publish `gnss_fixed` either, so a consumer of that topic sees no update at all.
TEST_F(GnssPoserCharacterization, Gate_LocalProjector_PublishesNothingNotEvenGnssFixed)
{
  ASSERT_NO_FATAL_FAILURE(build_node({}));
  send_projector_info(make_mgrs_projector_info());
  send_fix(make_reference_fix());
  ASSERT_NO_FATAL_FAILURE(wait_for_outputs(1));
  expect_output_counts(1, 1, 1, 1);

  // The latest projector info wins: LOCAL disables processing. Its arrival publishes nothing,
  // and the next fix is dropped entirely.
  send_projector_info(make_local_projector_info());
  expect_output_counts(1, 1, 1, 1);
  send_fix(make_reference_fix());
  expect_output_counts(1, 1, 1, 1);

  // Switching back re-enables processing, again without replaying the dropped fix.
  send_projector_info(make_mgrs_projector_info());
  expect_output_counts(1, 1, 1, 1);
  send_fix(make_reference_fix());
  ASSERT_NO_FATAL_FAILURE(wait_for_outputs(2));
  expect_output_counts(2, 2, 2, 2);
}

// A fix without a position solution (STATUS_NO_FIX) publishes `gnss_fixed = false` and nothing
// else.
TEST_F(GnssPoserCharacterization, Gate_StatusNoFix_PublishesOnlyGnssFixedFalse)
{
  ASSERT_NO_FATAL_FAILURE(build_node({}));
  send_projector_info(make_mgrs_projector_info());

  send_fix(make_reference_fix(NavSatStatus::STATUS_NO_FIX));
  ASSERT_NO_FATAL_FAILURE(wait_for_gnss_fixed(1));
  expect_output_counts(1, 0, 0, 0);
  EXPECT_FALSE(last_fixed().data);
}

// `is_fixed` is `status >= STATUS_FIX`: FIX, SBAS_FIX and GBAS_FIX all pass the gate and each
// produces one of every output, while NO_FIX adds a `gnss_fixed = false` and nothing else.
TEST_F(GnssPoserCharacterization, Gate_StatusFixSbasGbas_AllPass)
{
  ASSERT_NO_FATAL_FAILURE(build_node({}));
  send_projector_info(make_mgrs_projector_info());

  // is_fixed(status) <=> status >= STATUS_FIX (0)
  const std::vector<NavSatStatus::_status_type> fixed_statuses = {
    NavSatStatus::STATUS_FIX, NavSatStatus::STATUS_SBAS_FIX, NavSatStatus::STATUS_GBAS_FIX};
  std::size_t expected_count = 0;
  for (const auto status : fixed_statuses) {
    send_fix(make_reference_fix(status));
    ++expected_count;
    ASSERT_NO_FATAL_FAILURE(wait_for_outputs(expected_count));
    EXPECT_TRUE(last_fixed().data) << "status=" << static_cast<int>(status);
  }
  expect_output_counts(3, 3, 3, 3);

  send_fix(make_reference_fix(NavSatStatus::STATUS_NO_FIX));
  ASSERT_NO_FATAL_FAILURE(wait_for_gnss_fixed(4));
  EXPECT_FALSE(last_fixed().data);
  expect_output_counts(4, 3, 3, 3);
}

// `gnss_fixed` is stamped with the node clock, not with the header stamp of the fix it reports on.
//
// NOTE(characterization): every other output copies the input stamp; this one alone carries
// `now()`. Only the interval is pinned (between the moments before and after the fix was sent), not
// a value.
TEST_F(GnssPoserCharacterization, GnssFixed_StampIsNodeClockNotFixHeaderStamp)
{
  ASSERT_NO_FATAL_FAILURE(build_node({}));
  send_projector_info(make_mgrs_projector_info());

  const rclcpp::Time before = peer_->now();
  send_fix(make_reference_fix(NavSatStatus::STATUS_NO_FIX));
  ASSERT_NO_FATAL_FAILURE(wait_for_gnss_fixed(1));
  const rclcpp::Time after = peer_->now();

  const rclcpp::Time stamp(last_fixed().stamp, RCL_ROS_TIME);
  EXPECT_NE(last_fixed().stamp, fix_stamp());
  EXPECT_GE(stamp.nanoseconds(), before.nanoseconds());
  EXPECT_LE(stamp.nanoseconds(), after.nanoseconds());
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
TEST_F(GnssPoserCharacterization, MethodInstant_PublishesProjectedAntennaPosition)
{
  NodeParams params;
  params.gnss_pose_pub_method = 0;
  params.map_frame = "my_map";  // non-default, so that frame_id is shown to come from the parameter
  ASSERT_NO_FATAL_FAILURE(build_node(params));
  const auto projector = make_mgrs_projector_info();
  send_projector_info(projector);

  const auto fix = make_reference_fix();
  send_fix(fix);
  ASSERT_NO_FATAL_FAILURE(wait_for_outputs(1));

  const auto & pose = last_pose();
  EXPECT_EQ(pose.header.frame_id, "my_map");
  EXPECT_EQ(pose.header.stamp, fix.header.stamp);
  // Golden values recorded from the current implementation: a change in the library shows here.
  expect_point_near(pose.pose.position, make_point(golden_x, golden_y, golden_z));
  // The same library call the node composes: a change in how the node calls it shows here.
  expect_point_near(pose.pose.position, project_antenna(fix, projector));
  // z is the altitude passed through (MGRS ignores it, WGS84 -> WGS84 is the identity).
  EXPECT_NEAR(pose.pose.position.z, reference_altitude, position_tolerance);

  // gnss_pose_cov carries a copy of the same header and pose, hence exact equality.
  const auto & pose_cov = last_pose_cov();
  EXPECT_EQ(pose_cov.header, pose.header);
  EXPECT_EQ(pose_cov.pose.pose, pose.pose);
}

// With method 0 the buffer is never used: whatever `buff_epoch` says, every fix yields its own
// instantaneous position.
TEST_F(GnssPoserCharacterization, MethodInstant_LargeBuffEpoch_StillPublishesEveryFixInstantly)
{
  NodeParams params;
  params.gnss_pose_pub_method = 0;
  params.buff_epoch = 5;  // ignored for method 0
  ASSERT_NO_FATAL_FAILURE(build_node(params));
  const auto projector = make_mgrs_projector_info();
  send_projector_info(projector);

  // Three fixes about 100 m apart (see the note at the reference constants), so that each output
  // can be checked against its own fix and not a neighbor.
  const std::vector<NavSatFix> fixes = {
    make_fix(reference_latitude, reference_longitude, reference_altitude),
    make_fix(reference_latitude + 0.001, reference_longitude, reference_altitude),
    make_fix(reference_latitude + 0.002, reference_longitude + 0.001, reference_altitude + 1.0)};
  for (std::size_t i = 0; i < fixes.size(); ++i) {
    send_fix(fixes[i]);
    ASSERT_NO_FATAL_FAILURE(wait_for_outputs(i + 1));
    expect_point_near(last_pose().pose.position, project_antenna(fixes[i], projector));
  }
  EXPECT_EQ(peer_->poses.size(), 3U);
}

// The projector's `vertical_datum` is honored: with EGM2008 the WGS84 ellipsoidal altitude is
// converted to an orthometric height, i.e. lowered by the geoid height (about 36 m at the reference
// point), while x and y are the MGRS coordinates as before. The expectation is independent ground
// truth (the golden coordinates and GeoidEval's geoid height). Skipped when the geoid dataset is
// not installed.
TEST_F(GnssPoserCharacterization, MethodInstant_Egm2008VerticalDatum_ConvertsHeightFromWgs84)
{
  if (!is_egm2008_dataset_available()) {
    GTEST_SKIP() << "egm2008-1 geoid dataset is not installed";
  }
  NodeParams params;
  params.gnss_pose_pub_method = 0;
  ASSERT_NO_FATAL_FAILURE(build_node(params));
  const auto projector = make_mgrs_projector_info(reference_mgrs_grid, MapProjectorInfo::EGM2008);
  send_projector_info(projector);

  const auto fix = make_reference_fix();
  send_fix(fix);
  ASSERT_NO_FATAL_FAILURE(wait_for_outputs(1));

  expect_point_near(
    last_pose().pose.position,
    make_point(golden_x, golden_y, reference_altitude - reference_geoid_height),
    geoid_height_tolerance);
}

// Projector info is forwarded to the projection library as-is: with LOCAL_CARTESIAN_UTM the map
// origin is honored, so a fix at the origin lands at (0, 0, altitude - origin altitude).
TEST_F(GnssPoserCharacterization, MethodInstant_LocalCartesianUtmProjector_UsesMapOrigin)
{
  NodeParams params;
  params.gnss_pose_pub_method = 0;
  ASSERT_NO_FATAL_FAILURE(build_node(params));
  // Origin == reference point => antenna at (0, 0, alt - origin_alt).
  const auto projector =
    make_local_cartesian_utm_projector_info(reference_latitude, reference_longitude, -10.0);
  send_projector_info(projector);

  const auto fix = make_reference_fix();
  send_fix(fix);
  ASSERT_NO_FATAL_FAILURE(wait_for_outputs(1));

  // Literal expectation: the fix sits on the origin, so only the altitude offset remains.
  expect_point_near(last_pose().pose.position, make_point(0.0, 0.0, reference_altitude - (-10.0)));
}

// =======================================================================================
// 4. Position buffering (gnss_pose_pub_method = 1 average / 2 median / other)
// =======================================================================================

// With method 1 nothing is published until `buff_epoch` fixes have been buffered (each of them
// still publishes `gnss_fixed`); from then on every fix publishes the mean of the last `buff_epoch`
// positions as a sliding window.
TEST_F(GnssPoserCharacterization, MethodAverage_WaitsForFullBufferThenSlidingWindowMean)
{
  NodeParams params;
  params.gnss_pose_pub_method = 1;
  params.buff_epoch = 3;
  ASSERT_NO_FATAL_FAILURE(build_node(params));
  const auto projector = make_mgrs_projector_info();
  send_projector_info(projector);

  const std::vector<NavSatFix> fixes = {
    make_fix(reference_latitude, reference_longitude, reference_altitude),
    make_fix(reference_latitude + 0.001, reference_longitude + 0.002, reference_altitude + 20.0),
    make_fix(reference_latitude + 0.002, reference_longitude + 0.001, reference_altitude + 10.0),
    make_fix(reference_latitude + 0.003, reference_longitude + 0.003, reference_altitude + 30.0)};
  std::vector<Point> antenna;
  for (const auto & fix : fixes) {
    antenna.push_back(project_antenna(fix, projector));
  }

  // Fixes 1 and 2 only fill the buffer: each publishes gnss_fixed, neither publishes a pose.
  send_fix(fixes[0]);
  ASSERT_NO_FATAL_FAILURE(wait_for_gnss_fixed(1));
  send_fix(fixes[1]);
  ASSERT_NO_FATAL_FAILURE(wait_for_gnss_fixed(2));
  expect_output_counts(2, 0, 0, 0);

  // Fix 3 completes the buffer: the third gnss_fixed comes with the first pose, the mean of 1..3.
  send_fix(fixes[2]);
  ASSERT_NO_FATAL_FAILURE(wait_for_outputs(1));
  expect_output_counts(3, 1, 1, 1);
  expect_point_near(last_pose().pose.position, mean_of({antenna[0], antenna[1], antenna[2]}));

  // Fix 4 slides the window: the mean of 2..4, the oldest sample dropped.
  send_fix(fixes[3]);
  ASSERT_NO_FATAL_FAILURE(wait_for_outputs(2));
  expect_output_counts(4, 2, 2, 2);
  expect_point_near(last_pose().pose.position, mean_of({antenna[1], antenna[2], antenna[3]}));
}

// With method 2 the median is taken per coordinate, so the result is in general not any of the
// input samples. The full-buffer gate and the sliding window apply as for the mean.
TEST_F(GnssPoserCharacterization, MethodMedian_OddBuffer_ComponentWiseMedian)
{
  NodeParams params;
  params.gnss_pose_pub_method = 2;
  params.buff_epoch = 3;
  ASSERT_NO_FATAL_FAILURE(build_node(params));
  const auto projector = make_mgrs_projector_info();
  send_projector_info(projector);

  // Chosen so that the median of x, of y and of z each come from a different sample, which is what
  // makes the case discriminating: a "median sample" implementation would return one of the inputs.
  const std::vector<NavSatFix> fixes = {
    make_fix(reference_latitude, reference_longitude, reference_altitude + 20.0),
    make_fix(reference_latitude + 0.001, reference_longitude + 0.002, reference_altitude),
    make_fix(reference_latitude + 0.002, reference_longitude + 0.001, reference_altitude + 10.0),
    make_fix(reference_latitude + 0.003, reference_longitude + 0.003, reference_altitude + 30.0)};
  std::vector<Point> antenna;
  for (const auto & fix : fixes) {
    antenna.push_back(project_antenna(fix, projector));
  }
  const auto expected_123 = component_wise_median_of({antenna[0], antenna[1], antenna[2]});
  for (const auto & sample : antenna) {
    ASSERT_FALSE(
      std::abs(sample.x - expected_123.x) < position_tolerance &&
      std::abs(sample.y - expected_123.y) < position_tolerance &&
      std::abs(sample.z - expected_123.z) < position_tolerance)
      << "fixture data: the component-wise median must not coincide with a sample";
  }

  // Fixes 1 and 2 only fill the buffer: each publishes gnss_fixed, neither publishes a pose.
  send_fix(fixes[0]);
  ASSERT_NO_FATAL_FAILURE(wait_for_gnss_fixed(1));
  send_fix(fixes[1]);
  ASSERT_NO_FATAL_FAILURE(wait_for_gnss_fixed(2));
  expect_output_counts(2, 0, 0, 0);

  // Fix 3 completes the buffer: the component-wise median of 1..3.
  send_fix(fixes[2]);
  ASSERT_NO_FATAL_FAILURE(wait_for_outputs(1));
  expect_output_counts(3, 1, 1, 1);
  expect_point_near(last_pose().pose.position, expected_123);

  // Fix 4 slides the window: the median of 2..4.
  send_fix(fixes[3]);
  ASSERT_NO_FATAL_FAILURE(wait_for_outputs(2));
  expect_output_counts(4, 2, 2, 2);
  expect_point_near(
    last_pose().pose.position, component_wise_median_of({antenna[1], antenna[2], antenna[3]}));
}

// An even buffer size averages the two central values of each coordinate: `get_median_position`
// has a separate branch for it. The formula itself is covered by the helper-level unit test in
// test_gnss_poser_node.cpp; this case pins that an even `buff_epoch` reaches that branch through
// the node, which keeps the behavior guarded while the helper is moved out of the node.
TEST_F(GnssPoserCharacterization, MethodMedian_EvenBuffer_AveragesTwoCentralValues)
{
  NodeParams params;
  params.gnss_pose_pub_method = 2;
  params.buff_epoch = 4;
  ASSERT_NO_FATAL_FAILURE(build_node(params));
  const auto projector = make_mgrs_projector_info();
  send_projector_info(projector);

  const std::vector<NavSatFix> fixes = {
    make_fix(reference_latitude, reference_longitude, reference_altitude),
    make_fix(reference_latitude + 0.003, reference_longitude + 0.001, reference_altitude + 30.0),
    make_fix(reference_latitude + 0.001, reference_longitude + 0.003, reference_altitude + 10.0),
    make_fix(reference_latitude + 0.002, reference_longitude + 0.002, reference_altitude + 20.0)};
  std::vector<Point> antenna;
  for (const auto & fix : fixes) {
    antenna.push_back(project_antenna(fix, projector));
  }

  for (std::size_t i = 0; i < 3; ++i) {
    send_fix(fixes[i]);
    ASSERT_NO_FATAL_FAILURE(wait_for_gnss_fixed(i + 1));
  }
  EXPECT_TRUE(peer_->poses.empty());

  send_fix(fixes[3]);
  ASSERT_NO_FATAL_FAILURE(wait_for_outputs(1));
  expect_point_near(last_pose().pose.position, component_wise_median_of(antenna));
}

// A `gnss_pose_pub_method` outside the documented 0..2 is accepted: any non-zero value enables
// buffering and any value other than 1 selects the median.
//
// NOTE(characterization): the schema documents the range but the node never validates it. Frozen
// here so that adding validation is an explicit decision (and a change to this case), not a side
// effect of the refactoring.
TEST_F(GnssPoserCharacterization, MethodUnknown_BehavesLikeMedian)
{
  NodeParams params;
  params.gnss_pose_pub_method = 3;
  params.buff_epoch = 3;
  ASSERT_NO_FATAL_FAILURE(build_node(params));
  const auto projector = make_mgrs_projector_info();
  send_projector_info(projector);

  const std::vector<NavSatFix> fixes = {
    make_fix(reference_latitude, reference_longitude, reference_altitude + 20.0),
    make_fix(reference_latitude + 0.001, reference_longitude + 0.002, reference_altitude),
    make_fix(reference_latitude + 0.002, reference_longitude + 0.001, reference_altitude + 10.0)};
  std::vector<Point> antenna;
  for (const auto & fix : fixes) {
    antenna.push_back(project_antenna(fix, projector));
  }

  send_fix(fixes[0]);
  ASSERT_NO_FATAL_FAILURE(wait_for_gnss_fixed(1));
  send_fix(fixes[1]);
  ASSERT_NO_FATAL_FAILURE(wait_for_gnss_fixed(2));
  EXPECT_TRUE(peer_->poses.empty());  // buffering is active (unlike method 0)

  send_fix(fixes[2]);
  ASSERT_NO_FATAL_FAILURE(wait_for_outputs(1));
  expect_point_near(last_pose().pose.position, component_wise_median_of(antenna));
}

// Non-fixed messages neither fill nor clear the position buffer: two accepted fixes around one
// NO_FIX still produce the mean of exactly those two.
TEST_F(GnssPoserCharacterization, Buffer_IsNotAffectedByNonFixedMessages)
{
  NodeParams params;
  params.gnss_pose_pub_method = 1;
  params.buff_epoch = 2;
  ASSERT_NO_FATAL_FAILURE(build_node(params));
  const auto projector = make_mgrs_projector_info();
  send_projector_info(projector);

  const auto fix1 = make_fix(reference_latitude, reference_longitude, reference_altitude);
  const auto no_fix = make_fix(
    reference_latitude + 0.01, reference_longitude + 0.01, reference_altitude + 100.0,
    NavSatStatus::STATUS_NO_FIX);
  const auto fix2 =
    make_fix(reference_latitude + 0.001, reference_longitude + 0.001, reference_altitude + 10.0);

  send_fix(fix1);
  ASSERT_NO_FATAL_FAILURE(wait_for_gnss_fixed(1));
  send_fix(no_fix);  // published gnss_fixed=false, nothing else
  ASSERT_NO_FATAL_FAILURE(wait_for_gnss_fixed(2));
  EXPECT_TRUE(peer_->poses.empty());

  send_fix(fix2);
  ASSERT_NO_FATAL_FAILURE(wait_for_outputs(1));
  expect_point_near(
    last_pose().pose.position,
    mean_of({project_antenna(fix1, projector), project_antenna(fix2, projector)}));
}

// Fixes dropped at the projector gates (no projector info yet, or LOCAL) never reach the position
// buffer: after two dropped fixes, the first accepted one still fills only one of the two slots.
// Pins the order "gates first, then buffer", which a refactoring could plausibly swap.
TEST_F(GnssPoserCharacterization, Buffer_IsNotAffectedByGatedFixes)
{
  NodeParams params;
  params.gnss_pose_pub_method = 1;
  params.buff_epoch = 2;
  ASSERT_NO_FATAL_FAILURE(build_node(params));
  const auto projector = make_mgrs_projector_info();

  const auto dropped_no_info =
    make_fix(reference_latitude + 0.01, reference_longitude + 0.01, reference_altitude + 100.0);
  const auto dropped_local =
    make_fix(reference_latitude - 0.01, reference_longitude - 0.01, reference_altitude - 100.0);
  const auto fix1 = make_fix(reference_latitude, reference_longitude, reference_altitude);
  const auto fix2 =
    make_fix(reference_latitude + 0.001, reference_longitude + 0.001, reference_altitude + 10.0);

  send_fix(dropped_no_info);  // no projector info yet
  send_projector_info(make_local_projector_info());
  send_fix(dropped_local);  // LOCAL projector
  expect_output_counts(0, 0, 0, 0);

  send_projector_info(projector);
  send_fix(fix1);
  ASSERT_NO_FATAL_FAILURE(wait_for_gnss_fixed(1));
  expect_output_counts(1, 0, 0, 0);  // one slot filled, not full

  send_fix(fix2);
  ASSERT_NO_FATAL_FAILURE(wait_for_outputs(1));
  expect_point_near(
    last_pose().pose.position,
    mean_of({project_antenna(fix1, projector), project_antenna(fix2, projector)}));
}

// `buff_epoch = 0` with the average publishes NaN coordinates on every output: a zero-capacity
// buffer is always "full", and the mean of nothing is 0/0. The node does not notice: `gnss_fixed`
// is true, `gnss_pose_cov` still carries the receiver's position covariance next to the NaN
// position, and the TF broadcast is NaN as well.
//
// NOTE(characterization): this is not a behavior to keep, it is one to be aware of. README says
// buff_epoch = 0 makes the method lose effect; it does not, and no consumer is told. The median
// counterpart (method 2, buff_epoch 0) throws std::out_of_range inside the callback and is
// deliberately not pinned. Rejecting buff_epoch = 0 at construction is a refactoring-phase change.
TEST_F(GnssPoserCharacterization, MethodAverage_BuffEpochZero_PublishesNaNPosition)
{
  NodeParams params;
  params.gnss_pose_pub_method = 1;
  params.buff_epoch = 0;
  ASSERT_NO_FATAL_FAILURE(build_node(params));
  send_projector_info(make_mgrs_projector_info());

  send_fix(make_reference_fix());
  ASSERT_NO_FATAL_FAILURE(wait_for_outputs(1));
  expect_output_counts(1, 1, 1, 1);
  EXPECT_TRUE(last_fixed().data);
  const auto is_nan_point = [](const auto & p) {
    return std::isnan(p.x) && std::isnan(p.y) && std::isnan(p.z);
  };
  EXPECT_TRUE(is_nan_point(last_pose().pose.position));
  EXPECT_TRUE(is_nan_point(last_pose_cov().pose.pose.position));
  EXPECT_TRUE(is_nan_point(last_tf().transform.translation));
  // The covariance still claims the receiver's accuracy for a position that is NaN.
  EXPECT_DOUBLE_EQ(last_pose_cov().pose.covariance[cov_xx], 1.0);
}

// `buff_epoch = 0` with method 0 is harmless, because the buffer is never touched.
TEST_F(GnssPoserCharacterization, MethodInstant_BuffEpochZero_IsHarmless)
{
  NodeParams params;
  params.gnss_pose_pub_method = 0;
  params.buff_epoch = 0;
  ASSERT_NO_FATAL_FAILURE(build_node(params));
  send_projector_info(make_mgrs_projector_info());

  const auto fix = make_reference_fix();
  send_fix(fix);
  ASSERT_NO_FATAL_FAILURE(wait_for_outputs(1));
  expect_point_near(last_pose().pose.position, project_antenna(fix, make_mgrs_projector_info()));
}

// =======================================================================================
// 5. Orientation
// =======================================================================================

// With use_gnss_ins_orientation = true the pose carries the orientation of the latest
// `autoware_orientation` message, and the orientation covariance diagonal is its RMSE squared
// (float32 precision). A newer message replaces the previous one; its header is irrelevant.
TEST_F(GnssPoserCharacterization, InsOrientation_UsesLatestMessageAndSquaredRmseAsCovariance)
{
  NodeParams params;
  params.gnss_pose_pub_method = 0;
  params.use_gnss_ins_orientation = true;
  ASSERT_NO_FATAL_FAILURE(build_node(params));
  send_projector_info(make_mgrs_projector_info());

  send_orientation(make_orientation(M_PI / 2.0, 0.1, 0.2, 0.3));
  send_fix(make_reference_fix());
  ASSERT_NO_FATAL_FAILURE(wait_for_outputs(1));
  EXPECT_NEAR(yaw_of(last_pose().pose.orientation), M_PI / 2.0, angle_tolerance);
  expect_same_rotation(last_pose().pose.orientation, yaw_to_quaternion(M_PI / 2.0));
  EXPECT_NEAR(last_pose_cov().pose.covariance[cov_roll_roll], 0.01, rmse_squared_tolerance);
  EXPECT_NEAR(last_pose_cov().pose.covariance[cov_pitch_pitch], 0.04, rmse_squared_tolerance);
  EXPECT_NEAR(last_pose_cov().pose.covariance[cov_yaw_yaw], 0.09, rmse_squared_tolerance);

  // A newer orientation message replaces the previous one; its header is irrelevant.
  send_orientation(make_orientation(-M_PI / 4.0, 0.5, 0.6, 0.7));
  send_fix(make_reference_fix());
  ASSERT_NO_FATAL_FAILURE(wait_for_outputs(2));
  EXPECT_NEAR(yaw_of(last_pose().pose.orientation), -M_PI / 4.0, angle_tolerance);
  EXPECT_NEAR(last_pose_cov().pose.covariance[cov_roll_roll], 0.25, rmse_squared_tolerance);
  EXPECT_NEAR(last_pose_cov().pose.covariance[cov_pitch_pitch], 0.36, rmse_squared_tolerance);
  EXPECT_NEAR(last_pose_cov().pose.covariance[cov_yaw_yaw], 0.49, rmse_squared_tolerance);
}

// With use_gnss_ins_orientation = true the node does not wait for an orientation message: before
// the first one it publishes the message-default orientation (identity, i.e. heading east in `map`)
// with the constructor's placeholder RMSE of 1.0 squared, and the antenna position unchanged.
//
// NOTE(characterization): a consumer cannot tell this placeholder from a measured orientation whose
// RMSE happens to be 1.0, and the identity heading also enters the antenna -> base_link lever-arm
// correction. Whether to gate on the first orientation (as on projector info) is a decision for the
// refactoring phase, not for this test.
TEST_F(GnssPoserCharacterization, InsOrientation_NoMessageYet_IdentityAndUnitCovariance)
{
  NodeParams params;
  params.gnss_pose_pub_method = 0;
  params.use_gnss_ins_orientation = true;
  ASSERT_NO_FATAL_FAILURE(build_node(params));
  const auto projector = make_mgrs_projector_info();
  send_projector_info(projector);

  const auto fix = make_reference_fix();
  send_fix(fix);
  ASSERT_NO_FATAL_FAILURE(wait_for_outputs(1));
  EXPECT_TRUE(last_fixed().data);
  expect_point_near(last_pose().pose.position, project_antenna(fix, projector));
  expect_same_rotation(last_pose().pose.orientation, yaw_to_quaternion(0.0));
  EXPECT_DOUBLE_EQ(last_pose_cov().pose.covariance[cov_roll_roll], 1.0);
  EXPECT_DOUBLE_EQ(last_pose_cov().pose.covariance[cov_pitch_pitch], 1.0);
  EXPECT_DOUBLE_EQ(last_pose_cov().pose.covariance[cov_yaw_yaw], 1.0);
  // Position covariance is unaffected.
  EXPECT_DOUBLE_EQ(last_pose_cov().pose.covariance[cov_xx], 1.0);
}

// With use_gnss_ins_orientation = false the heading comes from motion, and only from the last two
// accepted positions: the first fix has no previous position and gets the identity; every later fix
// gets yaw = atan2 of its displacement from the previous fix (roll = pitch = 0), whatever the
// heading before was. The orientation covariance is the constant 0.1 / 0.1 / 1.0 regardless of the
// displacement, and INS messages are ignored in this mode.
//
// NOTE(characterization): standing still gives a zero displacement and atan2(0, 0) = 0, so the
// heading snaps back to east instead of keeping the last one.
TEST_F(GnssPoserCharacterization, MotionOrientation_FirstFixIdentity_ThenYawFromDisplacement)
{
  NodeParams params;
  params.gnss_pose_pub_method = 0;
  params.use_gnss_ins_orientation = false;
  ASSERT_NO_FATAL_FAILURE(build_node(params));
  const auto projector = make_mgrs_projector_info();
  send_projector_info(projector);
  // INS orientation messages are ignored in this mode.
  send_orientation(make_orientation(M_PI / 2.0, 0.1, 0.2, 0.3));

  const auto fix1 = make_fix(reference_latitude, reference_longitude, reference_altitude);
  const auto fix2 = make_fix(
    reference_latitude + 0.001, reference_longitude + 0.001,
    reference_altitude);  // north-east of fix1
  const auto fix3 = make_fix(
    reference_latitude + 0.001, reference_longitude - 0.001,
    reference_altitude);  // west of fix2, north of fix1
  const auto p1 = project_antenna(fix1, projector);
  const auto p2 = project_antenna(fix2, projector);
  const auto p3 = project_antenna(fix3, projector);

  // First fix: no previous position => yaw = atan2(0, 0) = 0 (identity).
  send_fix(fix1);
  ASSERT_NO_FATAL_FAILURE(wait_for_outputs(1));
  expect_same_rotation(last_pose().pose.orientation, yaw_to_quaternion(0.0));
  EXPECT_DOUBLE_EQ(last_pose_cov().pose.covariance[cov_roll_roll], 0.1);
  EXPECT_DOUBLE_EQ(last_pose_cov().pose.covariance[cov_pitch_pitch], 0.1);
  EXPECT_DOUBLE_EQ(last_pose_cov().pose.covariance[cov_yaw_yaw], 1.0);

  // Second fix: yaw of the displacement in the map frame (roll = pitch = 0).
  send_fix(fix2);
  ASSERT_NO_FATAL_FAILURE(wait_for_outputs(2));
  const double expected_yaw = std::atan2(p2.y - p1.y, p2.x - p1.x);
  EXPECT_NEAR(yaw_of(last_pose().pose.orientation), expected_yaw, angle_tolerance);
  expect_same_rotation(last_pose().pose.orientation, yaw_to_quaternion(expected_yaw));

  // Third fix: the heading follows the latest displacement only. It is atan2(p3 - p2), the heading
  // of the step just made, and not atan2(p3 - p1) or anything carried over from the previous
  // heading.
  send_fix(fix3);
  ASSERT_NO_FATAL_FAILURE(wait_for_outputs(3));
  const double yaw_step = std::atan2(p3.y - p2.y, p3.x - p2.x);
  const double yaw_from_start = std::atan2(p3.y - p1.y, p3.x - p1.x);
  ASSERT_GT(std::abs(yaw_step - yaw_from_start), 0.5) << "fixture data must tell the two apart";
  EXPECT_NEAR(yaw_of(last_pose().pose.orientation), yaw_step, angle_tolerance);

  // Standing still: displacement is zero => yaw snaps back to 0, not "keep last heading".
  send_fix(fix3);
  ASSERT_NO_FATAL_FAILURE(wait_for_outputs(4));
  expect_same_rotation(last_pose().pose.orientation, yaw_to_quaternion(0.0));
}

// The previous position used for the heading is not reset by a NO_FIX message in between: the
// heading is still the displacement between the two accepted fixes.
TEST_F(GnssPoserCharacterization, MotionOrientation_PreviousPositionSurvivesNonFixedMessages)
{
  NodeParams params;
  params.gnss_pose_pub_method = 0;
  params.use_gnss_ins_orientation = false;
  ASSERT_NO_FATAL_FAILURE(build_node(params));
  const auto projector = make_mgrs_projector_info();
  send_projector_info(projector);

  const auto fix1 = make_fix(reference_latitude, reference_longitude, reference_altitude);
  const auto no_fix = make_fix(
    reference_latitude - 0.01, reference_longitude - 0.01, reference_altitude,
    NavSatStatus::STATUS_NO_FIX);
  const auto fix2 = make_fix(reference_latitude + 0.001, reference_longitude, reference_altitude);
  const auto p1 = project_antenna(fix1, projector);
  const auto p2 = project_antenna(fix2, projector);

  send_fix(fix1);
  ASSERT_NO_FATAL_FAILURE(wait_for_outputs(1));
  send_fix(no_fix);
  ASSERT_NO_FATAL_FAILURE(wait_for_gnss_fixed(2));
  send_fix(fix2);
  ASSERT_NO_FATAL_FAILURE(wait_for_outputs(2));
  EXPECT_NEAR(
    yaw_of(last_pose().pose.orientation), std::atan2(p2.y - p1.y, p2.x - p1.x), angle_tolerance);
}

// When a buffer is in use, the heading is derived from consecutive *averaged* positions, not from
// the raw antenna positions.
TEST_F(GnssPoserCharacterization, MotionOrientation_WithBuffer_UsesFilteredPositions)
{
  NodeParams params;
  params.use_gnss_ins_orientation = false;
  params.gnss_pose_pub_method = 1;
  params.buff_epoch = 2;
  ASSERT_NO_FATAL_FAILURE(build_node(params));
  const auto projector = make_mgrs_projector_info();
  send_projector_info(projector);

  const std::vector<NavSatFix> fixes = {
    make_fix(reference_latitude, reference_longitude, reference_altitude),
    make_fix(reference_latitude + 0.001, reference_longitude, reference_altitude),
    make_fix(reference_latitude + 0.001, reference_longitude + 0.002, reference_altitude)};
  std::vector<Point> antenna;
  for (const auto & fix : fixes) {
    antenna.push_back(project_antenna(fix, projector));
  }
  const auto m12 = mean_of({antenna[0], antenna[1]});
  const auto m23 = mean_of({antenna[1], antenna[2]});

  send_fix(fixes[0]);
  ASSERT_NO_FATAL_FAILURE(wait_for_gnss_fixed(1));
  send_fix(fixes[1]);
  ASSERT_NO_FATAL_FAILURE(wait_for_outputs(1));
  expect_same_rotation(last_pose().pose.orientation, yaw_to_quaternion(0.0));

  send_fix(fixes[2]);
  ASSERT_NO_FATAL_FAILURE(wait_for_outputs(2));
  // Heading is derived from consecutive *averaged* positions, not raw antenna positions.
  EXPECT_NEAR(
    yaw_of(last_pose().pose.orientation), std::atan2(m23.y - m12.y, m23.x - m12.x),
    angle_tolerance);
}

// =======================================================================================
// 6. TF handling (antenna -> base_link) and TF broadcast (map -> gnss_base_link)
// =======================================================================================

// The published pose is map->base = map->antenna * antenna->base: the antenna orientation rotates
// the lever arm and the yaws add up. Non-default frame names pin which parameter names which frame.
// The /tf broadcast map_frame -> gnss_base_frame mirrors the pose exactly at the fix stamp, and one
// processed fix yields exactly one of each output.
TEST_F(GnssPoserCharacterization, Tf_StaticAntennaToBaseTransform_IsComposedAndBroadcast)
{
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
  send_fix(fix);
  ASSERT_NO_FATAL_FAILURE(wait_for_outputs(1));

  // map->base = map->antenna * antenna->base. Antenna yaw 90deg rotates (1,2,0.5) into
  // (-2, 1, 0.5); yaws add up to 180deg.
  const auto antenna = project_antenna(fix, projector);
  const auto expected_position = make_point(antenna.x - 2.0, antenna.y + 1.0, antenna.z + 0.5);
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
TEST_F(GnssPoserCharacterization, Tf_AntennaFrameComesFromFixHeader_UnknownFrameFallsBackToIdentity)
{
  NodeParams params;
  params.gnss_pose_pub_method = 0;
  params.use_gnss_ins_orientation = true;
  params.base_frame = "my_base";
  ASSERT_NO_FATAL_FAILURE(build_node(params));
  const auto projector = make_mgrs_projector_info();
  send_projector_info(projector);
  send_orientation(make_orientation(M_PI / 2.0));
  ASSERT_NO_FATAL_FAILURE(broadcast_static_tf(
    "my_antenna", "my_base", make_point(1.0, 2.0, 0.5), yaw_to_quaternion(0.0)));

  // Frame from the header is what gets looked up; a frame without TF yields the antenna pose.
  const auto fix_unknown = make_reference_fix(NavSatStatus::STATUS_FIX, "some_other_antenna");
  send_fix(fix_unknown);
  ASSERT_NO_FATAL_FAILURE(wait_for_outputs(1));
  expect_point_near(last_pose().pose.position, project_antenna(fix_unknown, projector));
  EXPECT_NEAR(yaw_of(last_pose().pose.orientation), M_PI / 2.0, angle_tolerance);

  // Same frame as base_frame: no lookup, identity.
  const auto fix_base = make_reference_fix(NavSatStatus::STATUS_FIX, "my_base");
  send_fix(fix_base);
  ASSERT_NO_FATAL_FAILURE(wait_for_outputs(2));
  expect_point_near(last_pose().pose.position, project_antenna(fix_base, projector));

  // Known frame: the TF is applied (positive control).
  const auto fix_known = make_reference_fix(NavSatStatus::STATUS_FIX, "my_antenna");
  send_fix(fix_known);
  ASSERT_NO_FATAL_FAILURE(wait_for_outputs(3));
  const auto antenna = project_antenna(fix_known, projector);
  expect_point_near(
    last_pose().pose.position, make_point(antenna.x - 2.0, antenna.y + 1.0, antenna.z + 0.5));
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
TEST_F(GnssPoserCharacterization, Tf_LookupIsAtFixHeaderStamp)
{
  NodeParams params;
  params.gnss_pose_pub_method = 0;
  params.use_gnss_ins_orientation = true;
  params.base_frame = "my_base";
  ASSERT_NO_FATAL_FAILURE(build_node(params));
  const auto projector = make_mgrs_projector_info();
  send_projector_info(projector);
  send_orientation(make_orientation(0.0));

  const auto t0 = make_stamp(1000, 0);
  ASSERT_NO_FATAL_FAILURE(broadcast_timed_tf(
    "my_antenna", "my_base", make_point(1.0, 0.0, 0.0), yaw_to_quaternion(0.0), t0));

  const auto projected = project_antenna(make_reference_fix(), projector);

  // Fix stamped exactly at t0: transform found and applied.
  auto fix_at_t0 = make_reference_fix(NavSatStatus::STATUS_FIX, "my_antenna");
  fix_at_t0.header.stamp = t0;
  send_fix(fix_at_t0);
  ASSERT_NO_FATAL_FAILURE(wait_for_outputs(1));
  expect_point_near(
    last_pose().pose.position, make_point(projected.x + 1.0, projected.y, projected.z));

  // Fix stamped 1 s later: lookup needs extrapolation, fails, and falls back to identity.
  auto fix_later = fix_at_t0;
  fix_later.header.stamp = make_stamp(1001, 0);
  send_fix(fix_later);
  ASSERT_NO_FATAL_FAILURE(wait_for_outputs(2));
  expect_point_near(last_pose().pose.position, projected);
  EXPECT_EQ(last_pose().header.stamp, fix_later.header.stamp);

  // Fix with a zero stamp: tf2 treats time 0 as "latest", so the transform is applied again.
  auto fix_zero = fix_at_t0;
  fix_zero.header.stamp = make_stamp(0, 0);
  send_fix(fix_zero);
  ASSERT_NO_FATAL_FAILURE(wait_for_outputs(3));
  expect_point_near(
    last_pose().pose.position, make_point(projected.x + 1.0, projected.y, projected.z));
}

// When the antenna -> base relation changes over time, each pose follows the relation at its own
// fix stamp, interpolated between the two neighboring transforms: a fix halfway between two
// transforms 1 m apart is offset by the midpoint, and the yaw is interpolated as well.
//
// The relation is rigid in practice; this pins that the node does not cache or latch a transform
// but resolves it per fix, so a moving relation (a calibration rig, a trailer) would be honored.
TEST_F(GnssPoserCharacterization, Tf_ChangingRelationIsInterpolatedAtEachFixStamp)
{
  NodeParams params;
  params.gnss_pose_pub_method = 0;
  params.use_gnss_ins_orientation = true;
  params.base_frame = "my_base";
  ASSERT_NO_FATAL_FAILURE(build_node(params));
  const auto projector = make_mgrs_projector_info();
  send_projector_info(projector);
  send_orientation(make_orientation(0.0));

  // The relation moves 1 m along x and turns by 90 degrees between t0 and t1.
  const auto t0 = make_stamp(1000, 0);
  const auto t1 = make_stamp(1002, 0);
  const auto t_mid = make_stamp(1001, 0);
  ASSERT_NO_FATAL_FAILURE(broadcast_timed_tf(
    "my_antenna", "my_base", make_point(1.0, 0.0, 0.0), yaw_to_quaternion(0.0), t0));
  ASSERT_NO_FATAL_FAILURE(broadcast_timed_tf(
    "my_antenna", "my_base", make_point(2.0, 0.0, 0.0), yaw_to_quaternion(M_PI / 2.0), t1));

  const auto projected = project_antenna(make_reference_fix(), projector);
  auto fix = make_reference_fix(NavSatStatus::STATUS_FIX, "my_antenna");

  // At t0 the pose uses the first relation.
  fix.header.stamp = t0;
  send_fix(fix);
  ASSERT_NO_FATAL_FAILURE(wait_for_outputs(1));
  expect_point_near(
    last_pose().pose.position, make_point(projected.x + 1.0, projected.y, projected.z));
  EXPECT_NEAR(yaw_of(last_pose().pose.orientation), 0.0, angle_tolerance);

  // At t1 it uses the second.
  fix.header.stamp = t1;
  send_fix(fix);
  ASSERT_NO_FATAL_FAILURE(wait_for_outputs(2));
  expect_point_near(
    last_pose().pose.position, make_point(projected.x + 2.0, projected.y, projected.z));
  EXPECT_NEAR(yaw_of(last_pose().pose.orientation), M_PI / 2.0, angle_tolerance);

  // Halfway between, tf2 interpolates: translation 1.5 m, yaw 45 degrees. Going back in time is
  // fine, the buffer keeps both transforms.
  fix.header.stamp = t_mid;
  send_fix(fix);
  ASSERT_NO_FATAL_FAILURE(wait_for_outputs(3));
  expect_point_near(
    last_pose().pose.position, make_point(projected.x + 1.5, projected.y, projected.z));
  EXPECT_NEAR(yaw_of(last_pose().pose.orientation), M_PI / 4.0, angle_tolerance);
}

// =======================================================================================
// 7. Covariance
// =======================================================================================

// The position covariance diagonal is copied from the fix for APPROXIMATED, DIAGONAL_KNOWN and
// KNOWN, and replaced by 10.0 for UNKNOWN; off-diagonal input entries are never propagated and
// every off-diagonal output entry stays 0. The orientation covariance is the INS RMSE squared.
TEST_F(GnssPoserCharacterization, Covariance_PositionDiagonalFromFixUnlessTypeUnknown)
{
  NodeParams params;
  params.gnss_pose_pub_method = 0;
  params.use_gnss_ins_orientation = true;
  ASSERT_NO_FATAL_FAILURE(build_node(params));
  send_projector_info(make_mgrs_projector_info());
  send_orientation(make_orientation(0.0, 0.1, 0.2, 0.3));

  auto fix = make_reference_fix();
  // Off-diagonal entries of the input are never propagated.
  fix.position_covariance = {2.0, 0.5, 0.6, 0.5, 3.0, 0.7, 0.6, 0.7, 4.0};

  const std::vector<NavSatFix::_position_covariance_type_type> known_types = {
    NavSatFix::COVARIANCE_TYPE_APPROXIMATED, NavSatFix::COVARIANCE_TYPE_DIAGONAL_KNOWN,
    NavSatFix::COVARIANCE_TYPE_KNOWN};
  std::size_t count = 0;
  for (const auto type : known_types) {
    fix.position_covariance_type = type;
    send_fix(fix);
    ASSERT_NO_FATAL_FAILURE(wait_for_outputs(++count));
    const auto & cov = last_pose_cov().pose.covariance;
    EXPECT_DOUBLE_EQ(cov[cov_xx], 2.0) << "type=" << static_cast<int>(type);
    EXPECT_DOUBLE_EQ(cov[cov_yy], 3.0) << "type=" << static_cast<int>(type);
    EXPECT_DOUBLE_EQ(cov[cov_zz], 4.0) << "type=" << static_cast<int>(type);
  }

  fix.position_covariance_type = NavSatFix::COVARIANCE_TYPE_UNKNOWN;
  send_fix(fix);
  ASSERT_NO_FATAL_FAILURE(wait_for_outputs(++count));
  {
    const auto & cov = last_pose_cov().pose.covariance;
    EXPECT_DOUBLE_EQ(cov[cov_xx], 10.0);
    EXPECT_DOUBLE_EQ(cov[cov_yy], 10.0);
    EXPECT_DOUBLE_EQ(cov[cov_zz], 10.0);
    EXPECT_NEAR(cov[cov_roll_roll], 0.01, rmse_squared_tolerance);
    EXPECT_NEAR(cov[cov_pitch_pitch], 0.04, rmse_squared_tolerance);
    EXPECT_NEAR(cov[cov_yaw_yaw], 0.09, rmse_squared_tolerance);
    // Everything off the diagonal is zero.
    for (std::size_t i = 0; i < cov.size(); ++i) {
      if (i % 7 != 0) {
        EXPECT_DOUBLE_EQ(cov[i], 0.0) << "index " << i;
      }
    }
  }
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  const int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
