// Copyright(c) 2025 AutoCore Technology (Nanjing) Co., Ltd. All rights reserved.
//
// Copyright 2025 TIER IV, Inc.
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

#ifndef AUTOWARE__CROP_BOX_FILTER__CROP_BOX_FILTER_NODE_HPP_
#define AUTOWARE__CROP_BOX_FILTER__CROP_BOX_FILTER_NODE_HPP_

#include <autoware/point_types/types.hpp>
#include <autoware_utils_debug/debug_publisher.hpp>
#include <autoware_utils_debug/published_time_publisher.hpp>
#include <autoware_utils_system/stop_watch.hpp>
#include <autoware_utils_tf/transform_listener.hpp>

#include <geometry_msgs/msg/polygon_stamped.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

#include <pcl_conversions/pcl_conversions.h>

#include <memory>
#include <string>
#include <utility>
#include <vector>

using PointCloud2 = sensor_msgs::msg::PointCloud2;
using PointCloud2ConstPtr = sensor_msgs::msg::PointCloud2::ConstSharedPtr;

using PointCloud = pcl::PointCloud<pcl::PointXYZ>;
using PointCloudPtr = PointCloud::Ptr;
using PointCloudConstPtr = PointCloud::ConstPtr;

namespace autoware::crop_box_filter
{

class CropBoxFilter : public rclcpp::Node
{
private:
  // member variable declaration & definitions *************************************

  /** \brief The managed transform buffer. */
  std::unique_ptr<autoware_utils_tf::TransformListener> transform_listener_{nullptr};

  /** \brief The input TF frame the data should be transformed into,
   * if input.header.frame_id is different. */
  std::string tf_input_frame_;

  /** \brief The original data input TF frame. */
  std::string tf_input_orig_frame_;

  /** \brief The output TF frame the data should be transformed into,
   * if input.header.frame_id is different. */
  std::string tf_output_frame_;

  /** \brief The maximum queue size (default: 3). */
  size_t max_queue_size_ = 3;

  /** \brief Internal mutex. */
  std::mutex mutex_;

  bool need_preprocess_transform_ = false;
  bool need_postprocess_transform_ = false;

  Eigen::Matrix4f eigen_transform_preprocess_ = Eigen::Matrix4f::Identity(4, 4);
  Eigen::Matrix4f eigen_transform_postprocess_ = Eigen::Matrix4f::Identity(4, 4);

  struct CropBoxParam
  {
    float min_x;
    float max_x;
    float min_y;
    float max_y;
    float min_z;
    float max_z;
    bool negative{false};
  } param_;

  /** \brief Parameter service callback result : needed to be hold */
  OnSetParametersCallbackHandle::SharedPtr set_param_res_;

  // publisher and subscriber declaration *********************

  /** \brief The input PointCloud2 subscriber. */
  rclcpp::Subscription<PointCloud2>::SharedPtr sub_input_;

  rclcpp::Publisher<PointCloud2>::SharedPtr pub_output_;
  rclcpp::Publisher<geometry_msgs::msg::PolygonStamped>::SharedPtr crop_box_polygon_pub_;

  /** \brief processing time publisher. **/
  std::unique_ptr<autoware_utils_system::StopWatch<std::chrono::milliseconds>> stop_watch_ptr_;
  std::unique_ptr<autoware_utils_debug::DebugPublisher> debug_publisher_;
  std::unique_ptr<autoware_utils_debug::PublishedTimePublisher> published_time_publisher_;

  // function declaration *************************************

  void publish_crop_box_polygon();

  void pointcloud_callback(const PointCloud2ConstPtr cloud);

  /** \brief Parameter service callback */
  rcl_interfaces::msg::SetParametersResult param_callback(const std::vector<rclcpp::Parameter> & p);

  /** \brief Return whether the input PointCloud2 data has the same layout than PointXYZI. That is
   * to say whether you can memcpy from the PointCloud2 data buffer to a PointXYZI */
  bool is_data_layout_compatible_with_point_xyzi(const PointCloud2 & input);

  /** \brief Return whether the input PointCloud2 data has the same layout than PointXYZIRC. That is
   * to say whether you can memcpy from the PointCloud2 data buffer to a PointXYZIRC */
  bool is_data_layout_compatible_with_point_xyzirc(const PointCloud2 & input);

  /** \brief Return whether the input PointCloud2 data has the same layout than PointXYZIRADRT. That
   * is to say whether you can memcpy from the PointCloud2 data buffer to a PointXYZIRADRT */
  bool is_data_layout_compatible_with_point_xyziradrt(const PointCloud2 & input);

  /** \brief Return whether the input PointCloud2 data has the same layout than PointXYZIRCAEDT.
   * That is to say whether you can memcpy from the PointCloud2 data buffer to a PointXYZIRCAEDT */
  bool is_data_layout_compatible_with_point_xyzircaedt(const PointCloud2 & input);

  bool is_valid(const PointCloud2ConstPtr & cloud);

  /** \brief For parameter service callback */
  template <typename T>
  bool get_param(const std::vector<rclcpp::Parameter> & p, const std::string & name, T & value)
  {
    auto it = std::find_if(p.cbegin(), p.cend(), [&name](const rclcpp::Parameter & parameter) {
      return parameter.get_name() == name;
    });
    if (it != p.cend()) {
      value = it->template get_value<T>();
      return true;
    }
    return false;
  }

public:
  PCL_MAKE_ALIGNED_OPERATOR_NEW
  explicit CropBoxFilter(const rclcpp::NodeOptions & options);
  void filter_pointcloud(const PointCloud2ConstPtr & cloud, PointCloud2 & output);
};
}  // namespace autoware::crop_box_filter

#endif  // AUTOWARE__CROP_BOX_FILTER__CROP_BOX_FILTER_NODE_HPP_
