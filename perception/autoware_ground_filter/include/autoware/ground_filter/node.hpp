// Copyright 2021 Tier IV, Inc. All rights reserved.
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

#ifndef AUTOWARE__GROUND_FILTER__NODE_HPP_
#define AUTOWARE__GROUND_FILTER__NODE_HPP_

#include "autoware/ground_filter/data.hpp"
#include "autoware/ground_filter/ground_filter.hpp"

#include <autoware_utils_debug/time_keeper.hpp>
#include <autoware_vehicle_info_utils/vehicle_info.hpp>

#include <sensor_msgs/msg/point_cloud2.hpp>

#include <boost/thread/mutex.hpp>

// PCL includes
#include <tf2/transform_datatypes.hpp>

#include <pcl_msgs/msg/point_indices.hpp>

#include <pcl/filters/extract_indices.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/pcl_base.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

// PCL includes
#if __has_include(<message_filters/subscriber.hpp>)
#include <message_filters/subscriber.hpp>
#include <message_filters/sync_policies/approximate_time.hpp>
#include <message_filters/sync_policies/exact_time.hpp>
#include <message_filters/synchronizer.hpp>
#else
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/synchronizer.h>
#endif

// Include tier4 autoware utils
#include <autoware_utils_debug/debug_publisher.hpp>
#include <autoware_utils_debug/published_time_publisher.hpp>
#include <autoware_utils_system/stop_watch.hpp>
#include <autoware_utils_tf/transform_listener.hpp>
#include <tf2_eigen/tf2_eigen.hpp>

#include <tf2_ros/transform_listener.h>

#include <algorithm>
#include <memory>
#include <string>
#include <vector>

class GroundFilterTest;

namespace autoware::ground_filter
{
/**
 * This holds the coordinate transformation information of the point cloud.
 * Usage example:
 *   \code
 *   if (transform_info.need_transform) {
 *       point = transform_info.eigen_transform * point;
 *   }
 *   \endcode
 */
struct TransformInfo
{
  TransformInfo() : eigen_transform(Eigen::Matrix4f::Identity(4, 4)), need_transform(false) {}

  Eigen::Matrix4f eigen_transform;
  bool need_transform;
};

class GroundFilterComponent : public rclcpp::Node
{
private:
  // classified point label
  // (0: not classified, 1: ground, 2: not ground, 3: follow previous point,
  //  4: unknown(currently not used), 5: virtual ground)
  enum class PointLabel : uint16_t {
    INIT = 0,
    GROUND,
    NON_GROUND,
    POINT_FOLLOW,
    UNKNOWN,
    VIRTUAL_GROUND,
    OUT_OF_RANGE
  };

  struct PointData
  {
    float radius;  // cylindrical coords on XY Plane
    PointLabel point_state{PointLabel::INIT};
    uint16_t grid_id;   // id of grid in vertical
    size_t data_index;  // index of this point data in the source pointcloud
  };
  using PointCloudVector = std::vector<PointData>;

  struct PointsCentroid
  {
    float radius_sum;
    float height_sum;
    float radius_avg;
    float height_avg;
    float height_max;
    float height_min;
    uint32_t point_num;
    uint16_t grid_id;
    std::vector<size_t> pcl_indices;
    std::vector<float> height_list;
    std::vector<float> radius_list;

    PointsCentroid()
    : radius_sum(0.0f),
      height_sum(0.0f),
      radius_avg(0.0f),
      height_avg(0.0f),
      height_max(-10.0f),
      height_min(10.0f),
      point_num(0),
      grid_id(0)
    {
    }

    void initialize()
    {
      radius_sum = 0.0f;
      height_sum = 0.0f;
      radius_avg = 0.0f;
      height_avg = 0.0f;
      height_max = -10.0f;
      height_min = 10.0f;
      point_num = 0;
      grid_id = 0;
      pcl_indices.clear();
      height_list.clear();
    }

    void addPoint(const float radius, const float height)
    {
      radius_sum += radius;
      height_sum += height;
      ++point_num;
      radius_avg = radius_sum / point_num;
      height_avg = height_sum / point_num;
      height_max = height_max < height ? height : height_max;
      height_min = height_min > height ? height : height_min;
    }

    void addPoint(const float radius, const float height, const size_t index)
    {
      pcl_indices.push_back(index);
      height_list.push_back(height);
      addPoint(radius, height);
    }

    float getAverageSlope() const { return std::atan2(height_avg, radius_avg); }
    float getAverageHeight() const { return height_avg; }
    float getAverageRadius() const { return radius_avg; }
    float getMaxHeight() const { return height_max; }
    float getMinHeight() const { return height_min; }
    const std::vector<size_t> & getIndicesRef() const { return pcl_indices; }
    const std::vector<float> & getHeightListRef() const { return height_list; }
  };

  /** \brief Lazy transport subscribe routine. */
  void subscribe();

  void filter(
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr & input, const pcl::IndicesPtr & indices,
    const sensor_msgs::msg::PointCloud2 & output) const;

  // TODO(taisa1): Temporary Implementation: Remove this interface when all the filter nodes
  // conform to new API
  void faster_filter(
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr & input,
    [[maybe_unused]] const pcl::IndicesPtr & indices, sensor_msgs::msg::PointCloud2 & output,
    [[maybe_unused]] const TransformInfo & transform_info);

  // data accessor
  PclDataAccessor data_accessor_;

  const uint16_t ground_grid_continual_thresh_ = 3;
  bool elevation_grid_mode_;
  float non_ground_height_threshold_;
  float low_priority_region_x_;
  float center_pcl_shift_;  // virtual center of pcl to center mass

  // common parameters
  float radial_divider_angle_rad_;  // distance in rads between dividers
  size_t radial_dividers_num_;
  autoware::vehicle_info_utils::VehicleInfo vehicle_info_;

  // common thresholds
  float global_slope_max_angle_rad_;  // radians
  float local_slope_max_angle_rad_;   // radians
  float global_slope_max_ratio_;
  float local_slope_max_ratio_;
  float split_points_distance_tolerance_;  // distance in meters between concentric divisions

  // non-grid mode parameters
  bool use_virtual_ground_point_;
  float                      // minimum height threshold regardless the slope,
    split_height_distance_;  // useful for close points

  // grid mode parameters
  bool use_recheck_ground_cluster_;  // to enable recheck ground cluster
  bool use_lowest_point_;  // to select lowest point for reference in recheck ground cluster,
                           // otherwise select middle point
  float detection_range_z_max_;

  // grid parameters
  float grid_size_m_;
  float grid_mode_switch_radius_;  // non linear grid size switching distance
  uint16_t ground_grid_buffer_size_;
  float virtual_lidar_z_;

  // pointcloud parameters
  std::string tf_input_frame_;
  std::string tf_output_frame_;
  std::size_t max_queue_size_;
  bool use_indices_;
  bool latched_indices_;
  bool approximate_sync_;

  // grid ground filter processor
  std::unique_ptr<GroundFilter> ground_filter_ptr_;

  // time keeper related
  rclcpp::Publisher<autoware_utils_debug::ProcessingTimeDetail>::SharedPtr
    detailed_processing_time_publisher_;
  std::shared_ptr<autoware_utils_debug::TimeKeeper> time_keeper_;

  /*!
   * Output transformed PointCloud from in_cloud_ptr->header.frame_id to in_target_frame
   * @param[in] in_target_frame Coordinate system to perform transform
   * @param[in] in_cloud_ptr PointCloud to perform transform
   * @param[out] out_cloud_ptr Resulting transformed PointCloud
   * @retval true transform succeeded
   * @retval false transform failed
   */

  /*!
   * Convert sensor_msgs::msg::PointCloud2 to sorted PointCloudVector
   * @param[in] in_cloud Input Point Cloud to be organized in radial segments
   * @param[out] out_radial_ordered_points Vector of Points Clouds,
   *     each element will contain the points ordered
   */
  void convertPointcloud(
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr & in_cloud,
    std::vector<PointCloudVector> & out_radial_ordered_points) const;

  /*!
   * Output ground center of front wheels as the virtual ground point
   * @param[out] point Virtual ground origin point
   */
  void calcVirtualGroundOrigin(pcl::PointXYZ & point) const;

  /*!
   * Classifies Points in the PointCloud as Ground and Not Ground
   * @param in_radial_ordered_clouds Vector of an Ordered PointsCloud
   *     ordered by radial distance from the origin
   * @param out_no_ground_indices Returns the indices of the points
   *     classified as not ground in the original PointCloud
   */
  void classifyPointCloud(
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr & in_cloud,
    const std::vector<PointCloudVector> & in_radial_ordered_clouds,
    pcl::PointIndices & out_no_ground_indices) const;
  /*!
   * Returns the resulting complementary PointCloud, one with the points kept
   * and the other removed as indicated in the indices
   * @param in_cloud_ptr Input PointCloud to which the extraction will be performed
   * @param in_indices Indices of the points to be both removed and kept
   * @param out_object_cloud Resulting PointCloud with the indices kept
   */
  void extractObjectPoints(
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr & in_cloud_ptr,
    const pcl::PointIndices & in_indices, sensor_msgs::msg::PointCloud2 & out_object_cloud) const;

  /** \brief Parameter service callback result : needed to be hold */
  rclcpp::Node::OnSetParametersCallbackHandle::SharedPtr set_param_res_;

  /** \brief Parameter service callback */
  rcl_interfaces::msg::SetParametersResult onParameter(
    const std::vector<rclcpp::Parameter> & param);

  // debugger
  std::unique_ptr<autoware_utils_system::StopWatch<std::chrono::milliseconds>> stop_watch_ptr_{
    nullptr};
  std::unique_ptr<autoware_utils_debug::DebugPublisher> debug_publisher_ptr_{nullptr};

  // For pointcloud

  /** \brief Get a matrix for conversion from the original frame to the target frame */
  /** \brief Synchronized input, and indices.*/
  std::shared_ptr<message_filters::Synchronizer<message_filters::sync_policies::ApproximateTime<
    sensor_msgs::msg::PointCloud2, pcl_msgs::msg::PointIndices>>>
    sync_input_indices_a_;
  std::shared_ptr<message_filters::Synchronizer<message_filters::sync_policies::ExactTime<
    sensor_msgs::msg::PointCloud2, pcl_msgs::msg::PointIndices>>>
    sync_input_indices_e_;

  bool calculate_transform_matrix(
    const std::string & target_frame, const sensor_msgs::msg::PointCloud2 & from,
    TransformInfo & transform_info /*output*/);
  bool convert_output_costly(std::unique_ptr<sensor_msgs::msg::PointCloud2> & output);
  void faster_input_indices_callback(
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr cloud,
    const pcl_msgs::msg::PointIndices::ConstSharedPtr indices);

  void setupTF();

protected:
  /** \brief The original TF frame of the input pointcloud. */
  std::string tf_input_orig_frame_;

  /** \brief Internal mutex for thread safe parameter setting */
  std::mutex mutex_;

  /** \brief The input PointCloud2 subscriber. */
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_input_;

  /** \brief The output PointCloud2 publisher. */
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_output_;

  /** \brief The message filter subscriber for PointCloud2. */
  message_filters::Subscriber<sensor_msgs::msg::PointCloud2> sub_input_filter_;

  /** \brief The message filter subscriber for PointIndices. */
  message_filters::Subscriber<pcl_msgs::msg::PointIndices> sub_indices_filter_;

  std::unique_ptr<autoware_utils_tf::TransformListener> transform_listener_{nullptr};

  std::unique_ptr<autoware_utils_debug::PublishedTimePublisher> published_time_publisher_;

  // To validate if the pointcloud is valid
  inline bool isValid(
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr & cloud,
    const std::string & /*topic_name*/ = "input") const
  {
    // Ensure non-null
    if (cloud == nullptr) {
      RCLCPP_WARN(this->get_logger(), "Received null PointCloud");
      return false;
    }

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

  inline bool isValid(
    [[maybe_unused]] const pcl_msgs::msg::PointIndices::ConstSharedPtr & indices,
    const std::string & /*topic_name*/ = "indices") const
  {
    return true;
  }

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  explicit GroundFilterComponent(const rclcpp::NodeOptions & options);

  // for test
  friend GroundFilterTest;
};
}  // namespace autoware::ground_filter

#endif  // AUTOWARE__GROUND_FILTER__NODE_HPP_
