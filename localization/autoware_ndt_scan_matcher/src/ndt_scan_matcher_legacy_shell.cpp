// Copyright 2015-2019 Autoware Foundation
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

#include <autoware/ndt_scan_matcher/ndt_scan_matcher_core.hpp>

#include <optional>
#include <sstream>

namespace autoware::ndt_scan_matcher
{

bool NDTScanMatcher::is_node_activated()
{
  return is_activated_;
}

void NDTScanMatcher::initialize_mode_specific_state()
{
  if (param_.ndt_regularization_enable) {
    const double value_as_unlimited = 1000.0;
    regularization_pose_buffer_ =
      std::make_unique<autoware::localization_util::SmartPoseBuffer>(this->get_logger(), value_as_unlimited, value_as_unlimited);
  }

  initial_pose_buffer_ = std::make_unique<autoware::localization_util::SmartPoseBuffer>(
    this->get_logger(), param_.validation.initial_pose_timeout_sec,
    param_.validation.initial_pose_distance_tolerance_m);
}

void NDTScanMatcher::create_map_update_module()
{
  map_update_module_ =
    std::make_unique<MapUpdateModule>(this, ndt_ptr_, param_.dynamic_map_loading);
}

void NDTScanMatcher::callback_timer()
{
  const rclcpp::Time ros_time_now = this->now();

  diagnostics_map_update_->clear();

  diagnostics_map_update_->add_key_value("timer_callback_time_stamp", ros_time_now.nanoseconds());

  const bool node_is_activated = is_activated_;
  const auto latest_ekf_position = latest_ekf_position_.with([](const auto & pos) { return pos; });
  map_update_module_->callback_timer(node_is_activated, latest_ekf_position, diagnostics_map_update_);

  diagnostics_map_update_->publish(ros_time_now);
}

void NDTScanMatcher::callback_initial_pose(
  const geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr initial_pose_msg_ptr)
{
  diagnostics_initial_pose_->clear();

  callback_initial_pose_main(initial_pose_msg_ptr);

  diagnostics_initial_pose_->publish(initial_pose_msg_ptr->header.stamp);
}

// The C++ baseline body (OFF only). Under NDT_USE_RUST the whole callback runs in Rust.
void NDTScanMatcher::callback_initial_pose_main(
  const geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr initial_pose_msg_ptr)
{
  diagnostics_initial_pose_->add_key_value(
    "topic_time_stamp",
    static_cast<rclcpp::Time>(initial_pose_msg_ptr->header.stamp).nanoseconds());

  // check is_activated
  diagnostics_initial_pose_->add_key_value("is_activated", static_cast<bool>(is_activated_));
  if (!is_activated_) {
    std::stringstream message;
    message << "Node is not activated.";
    diagnostics_initial_pose_->update_level_and_message(
      diagnostic_msgs::msg::DiagnosticStatus::WARN, message.str());
    return;
  }

  // check is_expected_frame_id
  const bool is_expected_frame_id =
    (initial_pose_msg_ptr->header.frame_id == param_.frame.map_frame);
  diagnostics_initial_pose_->add_key_value("is_expected_frame_id", is_expected_frame_id);
  if (!is_expected_frame_id) {
    std::stringstream message;
    message << "Received initial pose message with frame_id "
            << initial_pose_msg_ptr->header.frame_id << ", but expected " << param_.frame.map_frame
            << ". Please check the frame_id in the input topic and ensure it is correct.";
    diagnostics_initial_pose_->update_level_and_message(
      diagnostic_msgs::msg::DiagnosticStatus::ERROR, message.str());
    return;
  }

  initial_pose_buffer_->push_back(initial_pose_msg_ptr);

  latest_ekf_position_.with([&](auto & pos) { pos = initial_pose_msg_ptr->pose.pose.position; });
}

void NDTScanMatcher::callback_regularization_pose(
  geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr pose_conv_msg_ptr)
{
  diagnostics_regularization_pose_->clear();

  diagnostics_regularization_pose_->add_key_value(
    "topic_time_stamp", static_cast<rclcpp::Time>(pose_conv_msg_ptr->header.stamp).nanoseconds());

  regularization_pose_buffer_->push_back(pose_conv_msg_ptr);

  diagnostics_regularization_pose_->publish(pose_conv_msg_ptr->header.stamp);
}

void NDTScanMatcher::service_trigger_node(
  const std_srvs::srv::SetBool::Request::SharedPtr req,
  std_srvs::srv::SetBool::Response::SharedPtr res)
{
  const rclcpp::Time ros_time_now = this->now();

  diagnostics_trigger_node_->clear();
  diagnostics_trigger_node_->add_key_value("service_call_time_stamp", ros_time_now.nanoseconds());

  is_activated_ = req->data;
  if (is_activated_) {
    initial_pose_buffer_->clear();
  }
  res->success = true;

  diagnostics_trigger_node_->add_key_value("is_activated", static_cast<bool>(is_activated_));
  diagnostics_trigger_node_->add_key_value("is_succeed_service", res->success);
  diagnostics_trigger_node_->publish(ros_time_now);
}

}  // namespace autoware::ndt_scan_matcher
