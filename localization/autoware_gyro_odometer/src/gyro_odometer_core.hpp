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

#ifndef GYRO_ODOMETER_CORE_HPP_
#define GYRO_ODOMETER_CORE_HPP_

#include <autoware/agnocast_wrapper/node.hpp>
#include <autoware/agnocast_wrapper/tf2.hpp>
#include <autoware_utils_diagnostics/diagnostics_interface.hpp>
#include <autoware_utils_geometry/msg/covariance.hpp>
#include <autoware_utils_logging/logger_level_configure.hpp>
#include <autoware_utils_tf/transform_listener.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2/transform_datatypes.hpp>

#include <geometry_msgs/msg/twist_stamped.hpp>
#include <geometry_msgs/msg/twist_with_covariance_stamped.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <deque>
#include <memory>
#include <string>

namespace autoware::gyro_odometer
{

class GyroOdometerNode : public autoware::agnocast_wrapper::Node
{
private:
  using COV_IDX = autoware_utils_geometry::xyz_covariance_index::XYZ_COV_IDX;

public:
  explicit GyroOdometerNode(const rclcpp::NodeOptions & node_options);

private:
  void callback_vehicle_twist(
    const AUTOWARE_MESSAGE_CONST_SHARED_PTR(geometry_msgs::msg::TwistWithCovarianceStamped)
      vehicle_twist_msg_ptr);
  void callback_imu(const AUTOWARE_MESSAGE_CONST_SHARED_PTR(sensor_msgs::msg::Imu) imu_msg_ptr);
  void concat_gyro_and_odometer();
  void publish_data(const geometry_msgs::msg::TwistWithCovarianceStamped & twist_with_cov_raw);
  void publish_diagnostics();

  AUTOWARE_SUBSCRIPTION_PTR(geometry_msgs::msg::TwistWithCovarianceStamped) vehicle_twist_sub_;
  AUTOWARE_SUBSCRIPTION_PTR(sensor_msgs::msg::Imu) imu_sub_;

  AUTOWARE_PUBLISHER_PTR(geometry_msgs::msg::TwistStamped) twist_raw_pub_;
  AUTOWARE_PUBLISHER_PTR(geometry_msgs::msg::TwistWithCovarianceStamped)
  twist_with_covariance_raw_pub_;

  AUTOWARE_PUBLISHER_PTR(geometry_msgs::msg::TwistStamped) twist_pub_;
  AUTOWARE_PUBLISHER_PTR(geometry_msgs::msg::TwistWithCovarianceStamped)
  twist_with_covariance_pub_;

  AUTOWARE_TIMER_PTR timer_;

  using TransformListener = autoware_utils_tf::TransformListenerT<
    autoware::agnocast_wrapper::Node, autoware::agnocast_wrapper::Buffer,
    autoware::agnocast_wrapper::TransformListener>;
  std::shared_ptr<TransformListener> transform_listener_;
  std::unique_ptr<
    autoware_utils_logging::BasicLoggerLevelConfigure<autoware::agnocast_wrapper::Node>>
    logger_configure_;

  std::string output_frame_;
  double message_timeout_sec_;

  bool vehicle_twist_arrived_;
  bool imu_arrived_;
  bool is_succeed_transform_imu_;
  rclcpp::Time latest_vehicle_twist_ros_time_;
  rclcpp::Time latest_imu_ros_time_;
  double latest_vehicle_twist_dt_;
  double latest_imu_dt_;
  int32_t latest_vehicle_twist_queue_size_ = 0;
  int32_t latest_imu_queue_size_ = 0;
  std::deque<geometry_msgs::msg::TwistWithCovarianceStamped> vehicle_twist_queue_;
  std::deque<sensor_msgs::msg::Imu> gyro_queue_;

  std::unique_ptr<
    autoware_utils_diagnostics::BasicDiagnosticsInterface<autoware::agnocast_wrapper::Node>>
    diagnostics_;
};

}  // namespace autoware::gyro_odometer

#endif  // GYRO_ODOMETER_CORE_HPP_
