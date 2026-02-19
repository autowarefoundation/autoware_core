// Copyright 2025 Autoware Foundation
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

#include "autoware/unified_localization_core/fusion_pipeline.hpp"
#include "autoware/unified_localization_core/types.hpp"

#include <autoware/component_interface_specs/localization.hpp>
#include <geometry_msgs/msg/accel_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/twist_with_covariance_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>

#include <cmath>
#include <memory>
#include <mutex>
#include <optional>
#include <string>

namespace autoware::unified_localization
{

using geometry_msgs::msg::AccelWithCovarianceStamped;
using geometry_msgs::msg::PoseWithCovarianceStamped;
using geometry_msgs::msg::TwistWithCovarianceStamped;
using nav_msgs::msg::Odometry;

static void msg_to_pose_core(
  const PoseWithCovarianceStamped & msg,
  autoware::unified_localization_core::PoseWithCovariance & out)
{
  out.timestamp_sec = rclcpp::Time(msg.header.stamp).seconds();
  out.position_x = msg.pose.pose.position.x;
  out.position_y = msg.pose.pose.position.y;
  out.position_z = msg.pose.pose.position.z;
  out.orientation_x = msg.pose.pose.orientation.x;
  out.orientation_y = msg.pose.pose.orientation.y;
  out.orientation_z = msg.pose.pose.orientation.z;
  out.orientation_w = msg.pose.pose.orientation.w;
  for (size_t i = 0; i < 36u; ++i) {
    out.covariance[i] = msg.pose.covariance[i];
  }
}

static void msg_to_twist_core(
  const TwistWithCovarianceStamped & msg,
  autoware::unified_localization_core::TwistWithCovariance & out)
{
  out.timestamp_sec = rclcpp::Time(msg.header.stamp).seconds();
  out.linear_x = msg.twist.twist.linear.x;
  out.linear_y = msg.twist.twist.linear.y;
  out.linear_z = msg.twist.twist.linear.z;
  out.angular_x = msg.twist.twist.angular.x;
  out.angular_y = msg.twist.twist.angular.y;
  out.angular_z = msg.twist.twist.angular.z;
  for (size_t i = 0; i < 36u; ++i) {
    out.covariance[i] = msg.twist.covariance[i];
  }
}

class LocalizationNode : public rclcpp::Node
{
public:
  explicit LocalizationNode(const rclcpp::NodeOptions & options)
  : rclcpp::Node("localization_node", options)
  {
    load_params();
    pipeline_ = std::make_unique<unified_localization_core::FusionPipeline>(params_);

    const double rate = params_.ekf.predict_frequency;
    timer_ = create_wall_timer(
      std::chrono::duration<double>(1.0 / rate),
      std::bind(&LocalizationNode::timer_callback, this));

    pub_kinematic_state_ =
      create_publisher<Odometry>("kinematic_state", rclcpp::QoS(1));
    pub_acceleration_ =
      create_publisher<AccelWithCovarianceStamped>("acceleration", rclcpp::QoS(1));
    tf_br_ = std::make_shared<tf2_ros::TransformBroadcaster>(*this);

    sub_initial_pose_ = create_subscription<PoseWithCovarianceStamped>(
      "initialpose", rclcpp::QoS(1),
      std::bind(&LocalizationNode::on_initial_pose, this, std::placeholders::_1));
    sub_pose_ = create_subscription<PoseWithCovarianceStamped>(
      "in_pose_with_covariance", rclcpp::QoS(1),
      std::bind(&LocalizationNode::on_pose, this, std::placeholders::_1));
    sub_twist_ = create_subscription<TwistWithCovarianceStamped>(
      "in_twist_with_covariance", rclcpp::QoS(1),
      std::bind(&LocalizationNode::on_twist, this, std::placeholders::_1));

    using Initialize = autoware::component_interface_specs::localization::Initialize;
    srv_initialize_ = create_service<Initialize::Service>(
      Initialize::name,
      std::bind(&LocalizationNode::on_initialize_service, this, std::placeholders::_1, std::placeholders::_2));
  }

private:
  void load_params()
  {
    params_.ekf.predict_frequency = declare_parameter<double>("node.predict_frequency", 50.0);
    params_.ekf.ekf_dt = 1.0 / std::max(params_.ekf.predict_frequency, 0.1);
    params_.ekf.enable_yaw_bias_estimation =
      declare_parameter<bool>("node.enable_yaw_bias_estimation", true);
    params_.ekf.extend_state_step =
      static_cast<size_t>(declare_parameter<int>("node.extend_state_step", 50));

    params_.ekf.pose_additional_delay =
      declare_parameter<double>("pose_measurement.pose_additional_delay", 0.0);
    params_.ekf.pose_gate_dist =
      declare_parameter<double>("pose_measurement.pose_gate_dist", 49.5);
    params_.ekf.pose_smoothing_steps =
      static_cast<size_t>(declare_parameter<int>("pose_measurement.pose_smoothing_steps", 5));
    params_.ekf.max_pose_queue_size =
      static_cast<size_t>(declare_parameter<int>("pose_measurement.max_pose_queue_size", 5));

    params_.ekf.twist_additional_delay =
      declare_parameter<double>("twist_measurement.twist_additional_delay", 0.0);
    params_.ekf.twist_gate_dist =
      declare_parameter<double>("twist_measurement.twist_gate_dist", 46.1);
    params_.ekf.twist_smoothing_steps =
      static_cast<size_t>(declare_parameter<int>("twist_measurement.twist_smoothing_steps", 2));
    params_.ekf.max_twist_queue_size =
      static_cast<size_t>(declare_parameter<int>("twist_measurement.max_twist_queue_size", 2));

    params_.ekf.proc_stddev_vx_c =
      declare_parameter<double>("process_noise.proc_stddev_vx_c", 10.0);
    params_.ekf.proc_stddev_wz_c =
      declare_parameter<double>("process_noise.proc_stddev_wz_c", 5.0);
    params_.ekf.proc_stddev_yaw_c =
      declare_parameter<double>("process_noise.proc_stddev_yaw_c", 0.005);

    params_.ekf.z_filter_proc_dev =
      declare_parameter<double>("simple_1d_filter_parameters.z_filter_proc_dev", 5.0);
    params_.ekf.roll_filter_proc_dev =
      declare_parameter<double>("simple_1d_filter_parameters.roll_filter_proc_dev", 0.1);
    params_.ekf.pitch_filter_proc_dev =
      declare_parameter<double>("simple_1d_filter_parameters.pitch_filter_proc_dev", 0.1);

    params_.stop_filter.linear_x_threshold =
      declare_parameter<double>("stop_filter.linear_x_threshold", 0.1);
    params_.stop_filter.angular_z_threshold =
      declare_parameter<double>("stop_filter.angular_z_threshold", 0.02);

    params_.twist2accel.accel_lowpass_gain =
      declare_parameter<double>("twist2accel.accel_lowpass_gain", 0.2);

    pose_frame_id_ = declare_parameter<std::string>("pose_frame_id", "map");
    child_frame_id_ = declare_parameter<std::string>("child_frame_id", "base_link");
  }

  void on_initial_pose(const PoseWithCovarianceStamped::SharedPtr msg)
  {
    unified_localization_core::PoseWithCovariance pose_core;
    msg_to_pose_core(*msg, pose_core);
    pipeline_->initialize(pose_core);
    RCLCPP_INFO(get_logger(), "Initial pose set (Adapter).");
  }

  void on_pose(const PoseWithCovarianceStamped::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(meas_mutex_);
    last_pose_ = std::make_shared<unified_localization_core::PoseWithCovariance>();
    msg_to_pose_core(*msg, *last_pose_);
  }

  void on_twist(const TwistWithCovarianceStamped::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(meas_mutex_);
    last_twist_ = std::make_shared<unified_localization_core::TwistWithCovariance>();
    msg_to_twist_core(*msg, *last_twist_);
  }

  void on_initialize_service(
    const autoware::component_interface_specs::localization::Initialize::Service::Request::SharedPtr req,
    const autoware::component_interface_specs::localization::Initialize::Service::Response::SharedPtr res)
  {
    using Initialize = autoware::component_interface_specs::localization::Initialize;
    if (req->method != Initialize::Service::Request::DIRECT) {
      res->status.success = false;
      res->status.code = 1;  // PARAMETER_ERROR or similar
      res->status.message = "Only DIRECT method is supported (pose_with_covariance in request).";
      RCLCPP_WARN(get_logger(), "%s", res->status.message.c_str());
      return;
    }
    if (req->pose_with_covariance.empty()) {
      res->status.success = false;
      res->status.code = 1;
      res->status.message = "DIRECT method requires at least one pose_with_covariance.";
      RCLCPP_WARN(get_logger(), "%s", res->status.message.c_str());
      return;
    }
    const PoseWithCovarianceStamped & msg = req->pose_with_covariance.front();
    unified_localization_core::PoseWithCovariance pose_core;
    msg_to_pose_core(msg, pose_core);
    pipeline_->initialize(pose_core);
    res->status.success = true;
    res->status.message = "";
    RCLCPP_INFO(get_logger(), "Initial pose set via /localization/initialize (DIRECT).");
  }

  void timer_callback()
  {
    const rclcpp::Time now = this->now();
    const double t_sec = now.seconds();

    if (!pipeline_->is_initialized()) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Pipeline not initialized. Send initialpose (e.g. from pose_initializer).");
      return;
    }

    double dt_sec = params_.ekf.ekf_dt;
    if (last_timer_time_) {
      dt_sec = (now - *last_timer_time_).seconds();
      if (dt_sec <= 0.0 || dt_sec > 10.0) {
        dt_sec = params_.ekf.ekf_dt;
      }
    }
    last_timer_time_ = now;

    const unified_localization_core::PoseWithCovariance * pose_ptr = nullptr;
    const unified_localization_core::TwistWithCovariance * twist_ptr = nullptr;
    {
      std::lock_guard<std::mutex> lock(meas_mutex_);
      if (last_pose_) pose_ptr = last_pose_.get();
      if (last_twist_) twist_ptr = last_twist_.get();
    }

    pipeline_->step(t_sec, dt_sec, pose_ptr, twist_ptr);

    unified_localization_core::OdometryOutput odom;
    pipeline_->get_odometry(t_sec, odom);

    // Publish kinematic_state (Odometry)
    Odometry odom_msg;
    odom_msg.header.stamp = now;
    odom_msg.header.frame_id = pose_frame_id_;
    odom_msg.child_frame_id = child_frame_id_;
    odom_msg.pose.pose.position.x = odom.position_x;
    odom_msg.pose.pose.position.y = odom.position_y;
    odom_msg.pose.pose.position.z = odom.position_z;
    odom_msg.pose.pose.orientation.x = odom.orientation_x;
    odom_msg.pose.pose.orientation.y = odom.orientation_y;
    odom_msg.pose.pose.orientation.z = odom.orientation_z;
    odom_msg.pose.pose.orientation.w = odom.orientation_w;
    for (size_t i = 0; i < 36u; ++i) {
      odom_msg.pose.covariance[i] = odom.pose_covariance[i];
    }
    odom_msg.twist.twist.linear.x = odom.linear_x;
    odom_msg.twist.twist.linear.y = odom.linear_y;
    odom_msg.twist.twist.linear.z = odom.linear_z;
    odom_msg.twist.twist.angular.x = odom.angular_x;
    odom_msg.twist.twist.angular.y = odom.angular_y;
    odom_msg.twist.twist.angular.z = odom.angular_z;
    for (size_t i = 0; i < 36u; ++i) {
      odom_msg.twist.covariance[i] = odom.twist_covariance[i];
    }
    pub_kinematic_state_->publish(odom_msg);

    // Publish acceleration (from previous and current twist)
    unified_localization_core::AccelerationOutput acc_core;
    if (prev_odom_) {
      unified_localization_core::Vector3 prev_lin{
        prev_odom_->linear_x, prev_odom_->linear_y, prev_odom_->linear_z};
      unified_localization_core::Vector3 prev_ang{
        prev_odom_->angular_x, prev_odom_->angular_y, prev_odom_->angular_z};
      unified_localization_core::Vector3 curr_lin{
        odom.linear_x, odom.linear_y, odom.linear_z};
      unified_localization_core::Vector3 curr_ang{
        odom.angular_x, odom.angular_y, odom.angular_z};
      pipeline_->get_acceleration(
        prev_odom_->timestamp_sec, odom.timestamp_sec,
        prev_lin, prev_ang, curr_lin, curr_ang, acc_core);
      AccelWithCovarianceStamped acc_msg;
      acc_msg.header.stamp = now;
      acc_msg.header.frame_id = child_frame_id_;
      acc_msg.accel.accel.linear.x = acc_core.linear_x;
      acc_msg.accel.accel.linear.y = acc_core.linear_y;
      acc_msg.accel.accel.linear.z = acc_core.linear_z;
      acc_msg.accel.accel.angular.x = acc_core.angular_x;
      acc_msg.accel.accel.angular.y = acc_core.angular_y;
      acc_msg.accel.accel.angular.z = acc_core.angular_z;
      acc_msg.accel.covariance.fill(0.0);
      pub_acceleration_->publish(acc_msg);
    }
    prev_odom_ = odom;

    // Publish TF (map -> base_link)
    geometry_msgs::msg::TransformStamped tf;
    tf.header.stamp = now;
    tf.header.frame_id = pose_frame_id_;
    tf.child_frame_id = child_frame_id_;
    tf.transform.translation.x = odom.position_x;
    tf.transform.translation.y = odom.position_y;
    tf.transform.translation.z = odom.position_z;
    tf.transform.rotation.x = odom.orientation_x;
    tf.transform.rotation.y = odom.orientation_y;
    tf.transform.rotation.z = odom.orientation_z;
    tf.transform.rotation.w = odom.orientation_w;
    tf_br_->sendTransform(tf);
  }

  unified_localization_core::FusionPipelineParams params_;
  std::unique_ptr<unified_localization_core::FusionPipeline> pipeline_;
  std::string pose_frame_id_;
  std::string child_frame_id_;

  rclcpp::TimerBase::SharedPtr timer_;
  std::optional<rclcpp::Time> last_timer_time_;

  rclcpp::Publisher<Odometry>::SharedPtr pub_kinematic_state_;
  rclcpp::Publisher<AccelWithCovarianceStamped>::SharedPtr pub_acceleration_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_br_;

  rclcpp::Subscription<PoseWithCovarianceStamped>::SharedPtr sub_initial_pose_;
  rclcpp::Subscription<PoseWithCovarianceStamped>::SharedPtr sub_pose_;
  rclcpp::Subscription<TwistWithCovarianceStamped>::SharedPtr sub_twist_;

  rclcpp::Service<autoware::component_interface_specs::localization::Initialize::Service>::SharedPtr
    srv_initialize_;

  std::mutex meas_mutex_;
  std::shared_ptr<unified_localization_core::PoseWithCovariance> last_pose_;
  std::shared_ptr<unified_localization_core::TwistWithCovariance> last_twist_;
  std::optional<unified_localization_core::OdometryOutput> prev_odom_;
};

}  // namespace autoware::unified_localization

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  auto node = std::make_shared<autoware::unified_localization::LocalizationNode>(options);
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
