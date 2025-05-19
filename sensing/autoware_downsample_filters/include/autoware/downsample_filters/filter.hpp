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

/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2009, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 * $Id: filter.h 35876 2011-02-09 01:04:36Z rusu $
 *
 */

#ifndef AUTOWARE__DOWNSAMPLE_FILTERS__FILTER_HPP_
#define AUTOWARE__DOWNSAMPLE_FILTERS__FILTER_HPP_

#include "autoware/downsample_filters/transform_info.hpp"

#include <boost/thread/mutex.hpp>

#include <memory>
#include <string>
#include <vector>

// PCL includes
#include <pcl/filters/filter.h>
#include <pcl/pcl_base.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_msgs/msg/model_coefficients.h>
#include <sensor_msgs/msg/point_cloud2.h>

// Include tier4 autoware utils
#include <autoware_utils_debug/debug_publisher.hpp>
#include <autoware_utils_debug/published_time_publisher.hpp>
#include <autoware_utils_system/stop_watch.hpp>
#include <autoware_utils_tf/transform_listener.hpp>
namespace autoware::downsample_filters
{
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

/** \brief @b Filter represents the base filter class. Some generic 3D operations that are
 * applicable to all filters are defined here as static methods. \author Radu Bogdan Rusu
 */
class Filter : public rclcpp::Node
{
public:
  using PointCloud2 = sensor_msgs::msg::PointCloud2;
  using PointCloud2ConstPtr = sensor_msgs::msg::PointCloud2::ConstSharedPtr;

  PCL_MAKE_ALIGNED_OPERATOR_NEW
  explicit Filter(
    const std::string & filter_name = "autoware_downsample_filter",
    const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

protected:
  /** \brief The input PointCloud2 subscriber. */
  rclcpp::Subscription<PointCloud2>::SharedPtr sub_input_;

  /** \brief The output PointCloud2 publisher. */
  rclcpp::Publisher<PointCloud2>::SharedPtr pub_output_;

  /** \brief The input TF frame the data should be transformed into,
   * if input.header.frame_id is different. */
  std::string tf_input_frame_;

  /** \brief The original data input TF frame. */
  std::string tf_input_orig_frame_;

  /** \brief The output TF frame the data should be transformed into,
   * if input.header.frame_id is different. */
  std::string tf_output_frame_;

  /** \brief Internal mutex. */
  std::mutex mutex_;

  /** \brief processing time publisher. **/
  std::unique_ptr<autoware_utils_system::StopWatch<std::chrono::milliseconds>> stop_watch_ptr_;
  std::unique_ptr<autoware_utils_debug::DebugPublisher> debug_publisher_;
  std::unique_ptr<autoware_utils_debug::PublishedTimePublisher> published_time_publisher_;

  /** \brief Virtual abstract filter method. To be implemented by every child.
   * \param input the input point cloud dataset.
   * \param output the resultant filtered PointCloud2
   */
  virtual void filter(const PointCloud2ConstPtr & input, PointCloud2 & output) = 0;

  // TODO(sykwer): Temporary Implementation: Remove this interface when all the filter nodes conform
  // to new API. It's not pure virtual function so that a child class does not have to implement
  // this function.
  virtual void faster_filter(
    const PointCloud2ConstPtr & input, PointCloud2 & output,
    const TransformInfo & transform_info);  // != 0

  /** \brief Call the child filter () method, optionally transform the result, and publish it.
   * \param input the input point cloud dataset.
   */
  virtual void compute_publish(const PointCloud2ConstPtr & input);
  /** \brief PointCloud2 data callback. */
  virtual void input_callback(const PointCloud2ConstPtr cloud);
  virtual bool convert_output_costly(std::unique_ptr<PointCloud2> & output);

  /** \brief The maximum queue size (default: 3). */
  size_t max_queue_size_ = 3;

  std::unique_ptr<autoware_utils_tf::TransformListener> transform_listener_{nullptr};

  inline bool isValid(
    const PointCloud2ConstPtr & cloud, const std::string & /*topic_name*/ = "input")
  {
    if (cloud->width * cloud->height * cloud->point_step != cloud->data.size()) {
      RCLCPP_WARN(
        this->get_logger(),
        "Invalid PointCloud (data = %zu, width = %d, height = %d, step = %d) with stamp %f, "
        "and frame %s received!",
        cloud->data.size(), cloud->width, cloud->height, cloud->point_step,
        rclcpp::Time(cloud->header.stamp).seconds(), cloud->header.frame_id.c_str());
      return false;
    }
    return true;
  }

private:
  /** \brief Get a matrix for conversion from the original frame to the target frame */
  bool calculate_transform_matrix(
    const std::string & target_frame, const sensor_msgs::msg::PointCloud2 & from,
    TransformInfo & transform_info /*output*/);

  // TODO(sykwer): Temporary Implementation: Remove this interface when all the filter nodes conform
  // to new API.
  void faster_input_callback(const PointCloud2ConstPtr cloud);

  void setupTF();
};
}  // namespace autoware::downsample_filters

#endif  // AUTOWARE__DOWNSAMPLE_FILTERS__FILTER_HPP_
