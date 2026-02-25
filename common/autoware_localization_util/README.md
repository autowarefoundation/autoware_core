# autoware_localization_util

## Overview

`autoware_localization_util` is a collection of localization utility packages. It contains 5 individual library that used by autoware localization nodes.

- `covariance_ellipse` 2d covariance visualization wrapper.
- `smart_pose_buffer` pose buffer management library which contains interpolate and data validation.
- `tree_structured_parzen_estimator` A Tree Structured Parzen Estimator library.
- `util_func` A tool library which contains several function for localization nodes.

## Design

- `covariance_ellipse` Translate `geometry_msgs::msg::PoseWithCovariance` message into ellipse visual marker to demonstrate covariance distribution.
- `smart_pose_buffer` A buffer library which implements pose message buffering, pose interpolate and pose validation.
- `tree_structured_parzen_estimator` A Probability Estimator library that includes estimator and log likelihood ratio calculation.
- `util_func` Tool function collection.

## Usage

### covariance_ellipse

Include header file to use:

```cpp
#include "autoware/localization_util/covariance_ellipse.hpp"
```

calculate ellipse and visualize

```cpp
autoware::localization_util::Ellipse ellipse_ = autoware::localization_util::calculate_xy_ellipse(input_msg->pose, scale_);

  const auto ellipse_marker = autoware::localization_util::create_ellipse_marker(
    ellipse_, input_msg->header, input_msg->pose);
```

### smart_pose_buffer

buffer init

```cpp
#include "autoware/localization_util/smart_pose_buffer.hpp"

using autoware::localization_util::SmartPoseBuffer;

std::unique_ptr<autoware::localization_util::SmartPoseBuffer> initial_pose_buffer_;
initial_pose_buffer_ = std::make_unique<SmartPoseBuffer>(
    this->get_logger(), param_.validation.initial_pose_timeout_sec,
    param_.validation.initial_pose_distance_tolerance_m);
```

interpolate and pop out old pose message

```cpp
std::optional<SmartPoseBuffer::InterpolateResult> interpolation_result_opt =
initial_pose_buffer_->interpolate(sensor_ros_time);

...

initial_pose_buffer_->pop_old(sensor_ros_time);
const SmartPoseBuffer::InterpolateResult & interpolation_result =
interpolation_result_opt.value();
```

clear buffer

```cpp
initial_pose_buffer_->clear();
```

### tree_structured_parzen_estimator

init the estimator.
n_startup_trials -- The number of initial random trials in the TPE (Tree-Structured Parzen Estimator). This value should be equal to or less than 'initial_estimate_particles_num' and more than 0. If it is equal to 'initial_estimate_particles_num', the search will be the same as a full random search.

```cpp
#include "autoware/localization_util/tree_structured_parzen_estimator.hpp"

using autoware::localization_util::TreeStructuredParzenEstimator;

TreeStructuredParzenEstimator tpe(
TreeStructuredParzenEstimator::Direction::MAXIMIZE,
param_.initial_pose_estimation.n_startup_trials, sample_mean, sample_stddev);
```

get estimation result

```cpp
const TreeStructuredParzenEstimator::Input input = tpe.get_next_input();
```

add new data to the estimator

```cpp
TreeStructuredParzenEstimator::Input result(6);
    result[0] = pose.position.x;
    result[1] = pose.position.y;
    result[2] = pose.position.z;
    result[3] = rpy.x;
    result[4] = rpy.y;
    result[5] = rpy.z;
tpe.add_trial(TreeStructuredParzenEstimator::Trial{result, ndt_result.transform_probability});
```

### util_func

include header file to use

```cpp
#include "autoware/localization_util/util_func.hpp"

using autoware::localization_util::exchange_color_crc;
using autoware::localization_util::matrix4f_to_pose;
using autoware::localization_util::point_to_vector3d;
using autoware::localization_util::pose_to_matrix4f;
```

list of useful function

```cpp
std_msgs::msg::ColorRGBA exchange_color_crc(double x);
double calc_diff_for_radian(const double lhs_rad, const double rhs_rad);
geometry_msgs::msg::Vector3 get_rpy(const geometry_msgs::msg::Pose & pose);
geometry_msgs::msg::Vector3 get_rpy(const geometry_msgs::msg::PoseStamped & pose);
geometry_msgs::msg::Vector3 get_rpy(const geometry_msgs::msg::PoseWithCovarianceStamped & pose);
geometry_msgs::msg::Quaternion rpy_rad_to_quaternion(
  const double r_rad, const double p_rad, const double y_rad);
geometry_msgs::msg::Quaternion rpy_deg_to_quaternion(
  const double r_deg, const double p_deg, const double y_deg);
geometry_msgs::msg::Twist calc_twist(
  const geometry_msgs::msg::PoseStamped & pose_a, const geometry_msgs::msg::PoseStamped & pose_b);
geometry_msgs::msg::PoseStamped interpolate_pose(
  const geometry_msgs::msg::PoseStamped & pose_a, const geometry_msgs::msg::PoseStamped & pose_b,
  const rclcpp::Time & time_stamp);
geometry_msgs::msg::PoseStamped interpolate_pose(
  const geometry_msgs::msg::PoseWithCovarianceStamped & pose_a,
  const geometry_msgs::msg::PoseWithCovarianceStamped & pose_b, const rclcpp::Time & time_stamp);
Eigen::Affine3d pose_to_affine3d(const geometry_msgs::msg::Pose & ros_pose);
Eigen::Matrix4f pose_to_matrix4f(const geometry_msgs::msg::Pose & ros_pose);
geometry_msgs::msg::Pose matrix4f_to_pose(const Eigen::Matrix4f & eigen_pose_matrix);
Eigen::Vector3d point_to_vector3d(const geometry_msgs::msg::Point & ros_pos);
template <class T>
T transform(const T & input, const geometry_msgs::msg::TransformStamped & transform);double norm(const geometry_msgs::msg::Point & p1, const geometry_msgs::msg::Point & p2);

void output_pose_with_cov_to_log(
  const rclcpp::Logger & logger, const std::string & prefix,
  const geometry_msgs::msg::PoseWithCovarianceStamped & pose_with_cov);
```
