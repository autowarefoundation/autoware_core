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

#include <rclcpp/rclcpp.hpp>

#include <autoware_internal_debug_msgs/msg/bool_stamped.hpp>
#include <autoware_map_msgs/msg/map_projector_info.hpp>
#include <autoware_sensing_msgs/msg/gnss_ins_orientation_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <tf2_msgs/msg/tf_message.hpp>

#include <gtest/gtest.h>

#include <algorithm>
#include <chrono>
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
using geometry_msgs::msg::PoseStamped;
using geometry_msgs::msg::PoseWithCovarianceStamped;
using geometry_msgs::msg::TransformStamped;
using sensor_msgs::msg::NavSatFix;
using sensor_msgs::msg::NavSatStatus;
using tf2_msgs::msg::TFMessage;
using std::chrono_literals::operator""ms;
using std::chrono_literals::operator""s;

// ---------------------------------------------------------------------------------------
// Reference input used throughout (same point as autoware_geography_utils' own tests).
// ---------------------------------------------------------------------------------------
constexpr double kLat = 35.62426;
constexpr double kLon = 139.74252;
constexpr double kAlt = 10.0;
constexpr const char * kMgrsGrid = "54SUE";

// A deterministic, clearly artificial header stamp so that "stamp copied from input" and
// "stamp taken from the node clock" can be told apart.
constexpr int32_t kFixStampSec = 1700000000;
constexpr uint32_t kFixStampNanosec = 123456789U;

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
    static const std::vector<std::string> kNames = {
      "base_frame",           "gnss_base_frame", "map_frame", "use_gnss_ins_orientation",
      "gnss_pose_pub_method", "buff_epoch"};
    return kNames;
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
  return make_stamp(kFixStampSec, kFixStampNanosec);
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
  return make_fix(kLat, kLon, kAlt, status, frame_id);
}

MapProjectorInfo make_mgrs_projector_info(
  const std::string & grid = kMgrsGrid,
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

// ---------------------------------------------------------------------------------------
// Test double: a plain rclcpp node that plays the role of every peer of gnss_poser
// (map_projection_loader, GNSS driver, INS driver, robot_state_publisher, and the consumers).
// ---------------------------------------------------------------------------------------
class PeerNode : public rclcpp::Node
{
public:
  PeerNode() : rclcpp::Node("gnss_poser_characterization_peer")
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
      [this](const PoseWithCovarianceStamped::ConstSharedPtr msg) { pose_covs.push_back(*msg); });
    fixed_sub_ = create_subscription<BoolStamped>(
      "gnss_fixed", rclcpp::QoS{100},
      [this](const BoolStamped::ConstSharedPtr msg) { fixed_flags.push_back(*msg); });
    tf_sub_ = create_subscription<TFMessage>(
      "/tf", rclcpp::QoS{100}, [this](const TFMessage::ConstSharedPtr msg) {
        for (const auto & t : msg->transforms) {
          broadcast_tfs.push_back(t);
        }
      });
  }

  [[nodiscard]] bool all_endpoints_matched() const
  {
    return projector_pub_->get_subscription_count() >= 1 &&
           fix_pub_->get_subscription_count() >= 1 &&
           orientation_pub_->get_subscription_count() >= 1 &&
           pose_sub_->get_publisher_count() >= 1 && pose_cov_sub_->get_publisher_count() >= 1 &&
           fixed_sub_->get_publisher_count() >= 1 && tf_sub_->get_publisher_count() >= 1;
  }

  rclcpp::Publisher<MapProjectorInfo>::SharedPtr projector_pub_;
  rclcpp::Publisher<NavSatFix>::SharedPtr fix_pub_;
  rclcpp::Publisher<GnssInsOrientationStamped>::SharedPtr orientation_pub_;
  rclcpp::Subscription<PoseStamped>::SharedPtr pose_sub_;
  rclcpp::Subscription<PoseWithCovarianceStamped>::SharedPtr pose_cov_sub_;
  rclcpp::Subscription<BoolStamped>::SharedPtr fixed_sub_;
  rclcpp::Subscription<TFMessage>::SharedPtr tf_sub_;

  std::vector<PoseStamped> poses;
  std::vector<PoseWithCovarianceStamped> pose_covs;
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
             peer_->pose_covs.size() >= count && peer_->broadcast_tfs.size() >= count;
    }))
      << "expected " << count << " of each output, got gnss_fixed=" << peer_->fixed_flags.size()
      << " gnss_pose=" << peer_->poses.size() << " gnss_pose_cov=" << peer_->pose_covs.size()
      << " tf=" << peer_->broadcast_tfs.size();
  }

  // Exact number of messages received so far on each output.
  void expect_output_counts(
    std::size_t fixed, std::size_t pose, std::size_t pose_cov, std::size_t tf) const
  {
    EXPECT_EQ(peer_->fixed_flags.size(), fixed) << "gnss_fixed";
    EXPECT_EQ(peer_->poses.size(), pose) << "gnss_pose";
    EXPECT_EQ(peer_->pose_covs.size(), pose_cov) << "gnss_pose_cov";
    EXPECT_EQ(peer_->broadcast_tfs.size(), tf) << "/tf";
  }

  const PoseStamped & last_pose() const { return peer_->poses.back(); }
  const BoolStamped & last_fixed() const { return peer_->fixed_flags.back(); }

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

// None of the six parameters has a built-in default: leaving any one of them unset aborts
// construction.
//
// The observed exception is `rclcpp::ParameterTypeException` ("expected [<type>] got [not set]"),
// raised when the declared-but-unset value is read back with the requested type; that type is what
// is pinned.
TEST_F(GnssPoserCharacterization, Construct_MissingAnyRequiredParameter_Throws)
{
  const NodeParams params;
  for (const auto & missing : NodeParams::names()) {
    EXPECT_THROW(
      std::make_shared<autoware::gnss_poser::GNSSPoser>(params.to_options(missing)),
      rclcpp::ParameterTypeException)
      << "missing parameter: " << missing;
  }
}

// An override of the wrong type aborts construction for every one of the six parameters: an
// integer for the strings, a string for the boolean and for the integers.
TEST_F(GnssPoserCharacterization, Construct_WrongParameterType_Throws)
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
    EXPECT_THROW(
      std::make_shared<autoware::gnss_poser::GNSSPoser>(options),
      rclcpp::exceptions::InvalidParameterTypeException)
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

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  const int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
