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

#ifndef CROP_BOX_FILTER_NODE_HPP_
#define CROP_BOX_FILTER_NODE_HPP_

#include "crop_box_filter.hpp"

#include <autoware_utils_debug/debug_publisher.hpp>
#include <autoware_utils_debug/published_time_publisher.hpp>
#include <autoware_utils_system/stop_watch.hpp>
#include <autoware_utils_tf/transform_listener.hpp>

#include <memory>
#include <optional>
#include <string>
#include <vector>

namespace autoware::crop_box_filter
{

class CropBoxFilterNode : public rclcpp::Node
{
private:
  // member variable declaration & definitions *************************************

  /** \brief The managed transform buffer. */
  std::unique_ptr<autoware_utils_tf::TransformListener> transform_listener_{nullptr};

  /** \brief The input TF frame the data should be transformed into,
   * if input.header.frame_id is different. */
  std::string tf_input_frame_;

  /** \brief Internal mutex. */
  std::mutex mutex_;

  CropBoxFilterConfig config_;
  std::optional<CropBoxFilter> crop_box_filter_;

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

  void pointcloud_callback(const PointCloud2ConstPtr cloud);

  /** \brief Parameter service callback */
  rcl_interfaces::msg::SetParametersResult param_callback(const std::vector<rclcpp::Parameter> & p);

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
  explicit CropBoxFilterNode(const rclcpp::NodeOptions & node_options);
};
}  // namespace autoware::crop_box_filter

#endif  // CROP_BOX_FILTER_NODE_HPP_
