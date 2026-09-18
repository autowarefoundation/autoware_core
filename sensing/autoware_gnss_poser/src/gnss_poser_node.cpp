// Copyright 2020 Tier IV, Inc.
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

#include "gnss_poser_node.hpp"

#include <autoware_sensing_msgs/msg/gnss_ins_orientation_stamped.hpp>

#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <utility>

namespace autoware::gnss_poser
{
namespace
{
GnssPosePubMethod to_gnss_pose_pub_method(const int value)
{
  switch (value) {
    case 0:
      return GnssPosePubMethod::Instant;
    case 1:
      return GnssPosePubMethod::Average;
    case 2:
      return GnssPosePubMethod::Median;
    default:
      throw std::invalid_argument(
        "gnss_pose_pub_method must be 0 (instant), 1 (average) or 2 (median), got " +
        std::to_string(value));
  }
}
}  // namespace

GnssPoserNode::GnssPoserNode(const rclcpp::NodeOptions & node_options)
: autoware::agnocast_wrapper::Node("gnss_poser", node_options),
  tf2_listener_(tf2_buffer_, *this),
  tf2_broadcaster_(*this),
  base_frame_(declare_parameter<std::string>("base_frame")),
  gnss_poser_(
    declare_gnss_poser_params(),
    [this](const std::string & gnss_frame, const builtin_interfaces::msg::Time & stamp) {
      return get_static_transform(gnss_frame, base_frame_, stamp);
    })
{
  // Subscribe to map_projector_info topic
  sub_map_projector_info_ = create_subscription<autoware_map_msgs::msg::MapProjectorInfo>(
    "/map/map_projector_info", rclcpp::QoS{1}.transient_local(),
    std::bind(&GnssPoserNode::callback_map_projector_info, this, std::placeholders::_1));

  // Set subscribers and publishers
  nav_sat_fix_sub_ = create_subscription<sensor_msgs::msg::NavSatFix>(
    "fix", rclcpp::QoS{1},
    std::bind(&GnssPoserNode::callback_nav_sat_fix, this, std::placeholders::_1));
  autoware_orientation_sub_ =
    create_subscription<autoware_sensing_msgs::msg::GnssInsOrientationStamped>(
      "autoware_orientation", rclcpp::QoS{1},
      std::bind(
        &GnssPoserNode::callback_gnss_ins_orientation_stamped, this, std::placeholders::_1));

  pose_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>("gnss_pose", rclcpp::QoS{1});
  pose_cov_pub_ = create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
    "gnss_pose_cov", rclcpp::QoS{1});
  fixed_pub_ =
    create_publisher<autoware_internal_debug_msgs::msg::BoolStamped>("gnss_fixed", rclcpp::QoS{1});
}

GnssPoserParams GnssPoserNode::declare_gnss_poser_params()
{
  GnssPoserParams params;
  params.gnss_base_frame = declare_parameter<std::string>("gnss_base_frame");
  params.map_frame = declare_parameter<std::string>("map_frame");
  params.use_gnss_ins_orientation = declare_parameter<bool>("use_gnss_ins_orientation");
  params.gnss_pose_pub_method =
    to_gnss_pose_pub_method(declare_parameter<int>("gnss_pose_pub_method"));
  params.buff_epoch = declare_parameter<int>("buff_epoch");
  return params;
}

void GnssPoserNode::callback_map_projector_info(
  const AUTOWARE_MESSAGE_CONST_SHARED_PTR(autoware_map_msgs::msg::MapProjectorInfo) & msg)
{
  gnss_poser_.set_projector_info(*msg);
}

void GnssPoserNode::callback_nav_sat_fix(
  const AUTOWARE_MESSAGE_CONST_SHARED_PTR(sensor_msgs::msg::NavSatFix) & nav_sat_fix_msg_ptr)
{
  const GnssPoser::Result result = gnss_poser_.input_fix(*nav_sat_fix_msg_ptr);

  switch (result.outcome) {
    case GnssPoser::Outcome::NoProjectorInfo:
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), std::chrono::milliseconds(1000).count(),
        "map_projector_info has not been received yet. Check if the map_projection_loader is "
        "successfully launched.");
      break;

    case GnssPoser::Outcome::LocalProjector:
      RCLCPP_ERROR_THROTTLE(
        this->get_logger(), *this->get_clock(), std::chrono::milliseconds(5000).count(),
        "map_projector_info is local projector type. Unable to convert GNSS pose.");
      break;

    case GnssPoser::Outcome::NotFixed:
      RCLCPP_WARN_STREAM_THROTTLE(
        this->get_logger(), *this->get_clock(), std::chrono::milliseconds(1000).count(),
        "Not Fixed Topic. Skipping Calculate.");
      break;

    case GnssPoser::Outcome::Buffering:
      RCLCPP_WARN_STREAM_THROTTLE(
        this->get_logger(), *this->get_clock(), std::chrono::milliseconds(1000).count(),
        "Buffering Position. Output Skipped.");
      break;

    case GnssPoser::Outcome::Published:
      break;
  }

  publish_data(result);
}

// The pose computation decides what a fix produces; the node only has to know where each output
// goes.
void GnssPoserNode::publish_data(const GnssPoser::Result & result)
{
  if (result.gnss_fixed) {
    fixed_pub_->publish(*result.gnss_fixed);
  }
  if (result.gnss_pose) {
    pose_pub_->publish(*result.gnss_pose);
  }
  if (result.gnss_pose_cov) {
    pose_cov_pub_->publish(*result.gnss_pose_cov);
  }
  if (result.transform) {
    tf2_broadcaster_.sendTransform(*result.transform);
  }
}

void GnssPoserNode::callback_gnss_ins_orientation_stamped(
  const AUTOWARE_MESSAGE_CONST_SHARED_PTR(autoware_sensing_msgs::msg::GnssInsOrientationStamped) &
  msg)
{
  gnss_poser_.set_ins_orientation(msg->orientation);
}

std::optional<geometry_msgs::msg::Transform> GnssPoserNode::get_static_transform(
  const std::string & target_frame, const std::string & source_frame,
  const builtin_interfaces::msg::Time & stamp)
{
  if (target_frame == source_frame) {
    return geometry_msgs::msg::Transform{};  // identity: zero translation, rotation w = 1
  }

  try {
    return tf2_buffer_
      .lookupTransform(
        target_frame, source_frame,
        tf2::TimePoint(std::chrono::seconds(stamp.sec) + std::chrono::nanoseconds(stamp.nanosec)))
      .transform;
  } catch (tf2::TransformException & ex) {
    RCLCPP_WARN_STREAM_THROTTLE(
      this->get_logger(), *this->get_clock(), std::chrono::milliseconds(1000).count(), ex.what());
    RCLCPP_WARN_STREAM_THROTTLE(
      this->get_logger(), *this->get_clock(), std::chrono::milliseconds(1000).count(),
      "Please publish TF " << target_frame.c_str() << " to " << source_frame.c_str());
    return std::nullopt;
  }
}

}  // namespace autoware::gnss_poser

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(autoware::gnss_poser::GnssPoserNode)
