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

#include "autoware/ground_filter/node.hpp"

#include "autoware/ground_filter/ground_filter.hpp"
#include "autoware/ground_filter/sanity_check.hpp"

#include <autoware_utils_geometry/geometry.hpp>
#include <autoware_utils_math/normalization.hpp>
#include <autoware_utils_math/unit_conversion.hpp>
#include <autoware_utils_tf/transform_listener.hpp>
#include <autoware_vehicle_info_utils/vehicle_info_utils.hpp>
#include <pcl_ros/transforms.hpp>
#include <rclcpp/rclcpp.hpp>

#include <memory>
#include <string>
#include <utility>
#include <vector>

namespace
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
}  // namespace

namespace autoware::ground_filter
{
// For PointCloud2
using PointCloud2 = sensor_msgs::msg::PointCloud2;
using PointCloud2ConstPtr = sensor_msgs::msg::PointCloud2::ConstSharedPtr;

using autoware::vehicle_info_utils::VehicleInfoUtils;
using autoware_utils_debug::ScopedTimeTrack;
using autoware_utils_geometry::calc_distance3d;
using autoware_utils_math::deg2rad;
using autoware_utils_math::normalize_degree;
using autoware_utils_math::normalize_radian;

GroundFilterComponent::GroundFilterComponent(const rclcpp::NodeOptions & options)
: rclcpp::Node("GroundFilter", options)
{
  // set initial parameters
  {
    // modes
    elevation_grid_mode_ = rclcpp::Node::declare_parameter<bool>("elevation_grid_mode");

    // common parameters
    radial_divider_angle_rad_ = static_cast<float>(
      deg2rad(rclcpp::Node::declare_parameter<double>("radial_divider_angle_deg")));
    radial_dividers_num_ = std::ceil(2.0 * M_PI / radial_divider_angle_rad_);

    // common thresholds
    global_slope_max_angle_rad_ = static_cast<float>(
      deg2rad(rclcpp::Node::declare_parameter<double>("global_slope_max_angle_deg")));
    local_slope_max_angle_rad_ = static_cast<float>(
      deg2rad(rclcpp::Node::declare_parameter<double>("local_slope_max_angle_deg")));
    global_slope_max_ratio_ = std::tan(global_slope_max_angle_rad_);
    local_slope_max_ratio_ = std::tan(local_slope_max_angle_rad_);
    split_points_distance_tolerance_ = static_cast<float>(
      rclcpp::Node::declare_parameter<double>("split_points_distance_tolerance"));

    // vehicle info
    vehicle_info_ = VehicleInfoUtils(*this).getVehicleInfo();

    // non-grid parameters
    use_virtual_ground_point_ = rclcpp::Node::declare_parameter<bool>("use_virtual_ground_point");
    split_height_distance_ =
      static_cast<float>(rclcpp::Node::declare_parameter<double>("split_height_distance"));

    // grid mode parameters
    use_recheck_ground_cluster_ =
      rclcpp::Node::declare_parameter<bool>("use_recheck_ground_cluster");
    use_lowest_point_ = rclcpp::Node::declare_parameter<bool>("use_lowest_point");
    detection_range_z_max_ =
      static_cast<float>(rclcpp::Node::declare_parameter<double>("detection_range_z_max"));
    low_priority_region_x_ =
      static_cast<float>(rclcpp::Node::declare_parameter<double>("low_priority_region_x"));
    center_pcl_shift_ =
      static_cast<float>(rclcpp::Node::declare_parameter<double>("center_pcl_shift"));
    non_ground_height_threshold_ =
      static_cast<float>(rclcpp::Node::declare_parameter<double>("non_ground_height_threshold"));

    // grid parameters
    grid_size_m_ = static_cast<float>(rclcpp::Node::declare_parameter<double>("grid_size_m"));
    grid_mode_switch_radius_ =
      static_cast<float>(rclcpp::Node::declare_parameter<double>("grid_mode_switch_radius"));
    ground_grid_buffer_size_ = rclcpp::Node::declare_parameter<int>("ground_grid_buffer_size");
    virtual_lidar_z_ = vehicle_info_.vehicle_height_m;

    // initialize grid filter
    {
      GroundFilterParameter param;
      param.global_slope_max_angle_rad = global_slope_max_angle_rad_;
      param.local_slope_max_angle_rad = local_slope_max_angle_rad_;
      param.radial_divider_angle_rad = radial_divider_angle_rad_;

      param.use_recheck_ground_cluster = use_recheck_ground_cluster_;
      param.use_lowest_point = use_lowest_point_;
      param.detection_range_z_max = detection_range_z_max_;
      param.non_ground_height_threshold = non_ground_height_threshold_;

      param.grid_size_m = grid_size_m_;
      param.grid_mode_switch_radius = grid_mode_switch_radius_;
      param.ground_grid_buffer_size = ground_grid_buffer_size_;
      param.virtual_lidar_x = vehicle_info_.wheel_base_m / 2.0f + center_pcl_shift_;
      param.virtual_lidar_y = 0.0f;
      param.virtual_lidar_z = virtual_lidar_z_;

      ground_filter_ptr_ = std::make_unique<GroundFilter>(param);
    }
  }

  using std::placeholders::_1;
  set_param_res_ =
    this->add_on_set_parameters_callback(std::bind(&GroundFilterComponent::onParameter, this, _1));

  // initialize debug tool
  {
    stop_watch_ptr_ =
      std::make_unique<autoware_utils_system::StopWatch<std::chrono::milliseconds>>();
    debug_publisher_ptr_ =
      std::make_unique<autoware_utils_debug::DebugPublisher>(this, "ground_filter");
    stop_watch_ptr_->tic("cyclic_time");
    stop_watch_ptr_->tic("processing_time");

    bool use_time_keeper = rclcpp::Node::declare_parameter<bool>("publish_processing_time_detail");
    if (use_time_keeper) {
      detailed_processing_time_publisher_ =
        this->create_publisher<autoware_utils_debug::ProcessingTimeDetail>(
          "~/debug/processing_time_detail_ms", 1);
      auto time_keeper = autoware_utils_debug::TimeKeeper(detailed_processing_time_publisher_);
      time_keeper_ = std::make_shared<autoware_utils_debug::TimeKeeper>(time_keeper);

      // set time keeper to grid
      ground_filter_ptr_->setTimeKeeper(time_keeper_);
    }
  }

  // pointcloud parameters
  tf_input_frame_ = static_cast<std::string>(declare_parameter("input_frame", ""));
  tf_output_frame_ = static_cast<std::string>(declare_parameter("output_frame", ""));
  max_queue_size_ = static_cast<std::size_t>(declare_parameter("max_queue_size", 5));
  use_indices_ = static_cast<bool>(declare_parameter("use_indices", false));
  latched_indices_ = static_cast<bool>(declare_parameter("latched_indices", false));
  approximate_sync_ = static_cast<bool>(declare_parameter("approximate_sync", false));

  RCLCPP_DEBUG_STREAM(
    this->get_logger(),
    "Filter (as Component) successfully created with the following parameters:"
      << std::endl
      << " - approximate_sync : " << (approximate_sync_ ? "true" : "false") << std::endl
      << " - use_indices      : " << (use_indices_ ? "true" : "false") << std::endl
      << " - latched_indices  : " << (latched_indices_ ? "true" : "false") << std::endl
      << " - max_queue_size   : " << max_queue_size_);

  // Set publisher
  {
    rclcpp::PublisherOptions pub_options;
    pub_options.qos_overriding_options = rclcpp::QosOverridingOptions::with_default_policies();
    pub_output_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
      "output", rclcpp::SensorDataQoS().keep_last(max_queue_size_), pub_options);
  }

  subscribe();

  // Set tf_listener, tf_buffer.
  setupTF();

  published_time_publisher_ = std::make_unique<autoware_utils_debug::PublishedTimePublisher>(this);
  RCLCPP_DEBUG(this->get_logger(), "[Filter Constructor] successfully created.");
}

void GroundFilterComponent::setupTF()
{
  transform_listener_ = std::make_unique<autoware_utils_tf::TransformListener>(this);
}

void GroundFilterComponent::subscribe()
{
  if (use_indices_) {
    // Subscribe to the input using a filter
    sub_input_filter_.subscribe(
      this, "input", rclcpp::SensorDataQoS().keep_last(max_queue_size_).get_rmw_qos_profile());
    sub_indices_filter_.subscribe(
      this, "indices", rclcpp::SensorDataQoS().keep_last(max_queue_size_).get_rmw_qos_profile());

    if (approximate_sync_) {
      sync_input_indices_a_ = std::make_shared<
        message_filters::Synchronizer<message_filters::sync_policies::ApproximateTime<
          sensor_msgs::msg::PointCloud2, pcl_msgs::msg::PointIndices>>>(max_queue_size_);
      sync_input_indices_a_->connectInput(sub_input_filter_, sub_indices_filter_);
      sync_input_indices_a_->registerCallback(std::bind(
        &GroundFilterComponent::faster_input_indices_callback, this, std::placeholders::_1,
        std::placeholders::_2));
    } else {
      sync_input_indices_e_ =
        std::make_shared<message_filters::Synchronizer<message_filters::sync_policies::ExactTime<
          sensor_msgs::msg::PointCloud2, pcl_msgs::msg::PointIndices>>>(max_queue_size_);
      sync_input_indices_e_->connectInput(sub_input_filter_, sub_indices_filter_);
      sync_input_indices_e_->registerCallback(std::bind(
        &GroundFilterComponent::faster_input_indices_callback, this, std::placeholders::_1,
        std::placeholders::_2));
    }
  } else {
    std::function<void(const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg)> cb = std::bind(
      &GroundFilterComponent::faster_input_indices_callback, this, std::placeholders::_1,
      pcl_msgs::msg::PointIndices::ConstSharedPtr());
    sub_input_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      "input", rclcpp::SensorDataQoS().keep_last(max_queue_size_), cb);
  }
}

bool GroundFilterComponent::calculate_transform_matrix(
  const std::string & target_frame, const sensor_msgs::msg::PointCloud2 & from,
  TransformInfo & transform_info /*output*/)
{
  transform_info.need_transform = false;

  if (target_frame.empty() || from.header.frame_id == target_frame) return true;

  RCLCPP_DEBUG(
    this->get_logger(), "[get_transform_matrix] Transforming input dataset from %s to %s.",
    from.header.frame_id.c_str(), target_frame.c_str());

  auto tf_ptr = transform_listener_->get_transform(
    target_frame, from.header.frame_id, from.header.stamp, rclcpp::Duration::from_seconds(1.0));

  if (!tf_ptr) {
    return false;
  }

  auto eigen_tf = tf2::transformToEigen(*tf_ptr);
  transform_info.eigen_transform = eigen_tf.matrix().cast<float>();
  transform_info.need_transform = true;
  return true;
}

bool GroundFilterComponent::convert_output_costly(
  std::unique_ptr<sensor_msgs::msg::PointCloud2> & output)
{
  // In terms of performance, we should avoid using pcl_ros library function,
  // but this code path isn't reached in the main use case of Autoware, so it's left as is for now.
  if (!tf_output_frame_.empty() && output->header.frame_id != tf_output_frame_) {
    RCLCPP_DEBUG(
      this->get_logger(), "[convert_output_costly] Transforming output dataset from %s to %s.",
      output->header.frame_id.c_str(), tf_output_frame_.c_str());

    // Convert the cloud into the different frame
    auto cloud_transformed = std::make_unique<sensor_msgs::msg::PointCloud2>();

    auto tf_ptr = transform_listener_->get_transform(
      tf_output_frame_, output->header.frame_id, output->header.stamp,
      rclcpp::Duration::from_seconds(1.0));
    if (!tf_ptr) {
      RCLCPP_ERROR(
        this->get_logger(),
        "[convert_output_costly] Error converting output dataset from %s to %s.",
        output->header.frame_id.c_str(), tf_output_frame_.c_str());
      return false;
    }

    auto eigen_tf = tf2::transformToEigen(*tf_ptr);
    pcl_ros::transformPointCloud(eigen_tf.matrix().cast<float>(), *output, *cloud_transformed);
    output = std::move(cloud_transformed);
  }

  // Same as the comment above
  if (tf_output_frame_.empty() && output->header.frame_id != tf_input_orig_frame_) {
    // No tf_output_frame given, transform the dataset to its original frame
    RCLCPP_DEBUG(
      this->get_logger(), "[convert_output_costly] Transforming output dataset from %s back to %s.",
      output->header.frame_id.c_str(), tf_input_orig_frame_.c_str());

    auto cloud_transformed = std::make_unique<sensor_msgs::msg::PointCloud2>();

    auto tf_ptr = transform_listener_->get_transform(
      tf_input_orig_frame_, output->header.frame_id, output->header.stamp,
      rclcpp::Duration::from_seconds(1.0));
    if (!tf_ptr) {
      return false;
    }

    auto eigen_tf = tf2::transformToEigen(*tf_ptr);
    pcl_ros::transformPointCloud(eigen_tf.matrix().cast<float>(), *output, *cloud_transformed);
    output = std::move(cloud_transformed);
  }

  return true;
}

void GroundFilterComponent::faster_input_indices_callback(
  const sensor_msgs::msg::PointCloud2::ConstSharedPtr cloud,
  const pcl_msgs::msg::PointIndices::ConstSharedPtr indices)
{
  if (
    !is_data_layout_compatible_with_point_xyzircaedt(*cloud) &&
    !is_data_layout_compatible_with_point_xyzirc(*cloud)) {
    RCLCPP_ERROR(
      get_logger(),
      "The pointcloud layout is not compatible with PointXYZIRCAEDT or PointXYZIRC. Aborting");

    if (is_data_layout_compatible_with_point_xyziradrt(*cloud)) {
      RCLCPP_ERROR(
        get_logger(),
        "The pointcloud layout is compatible with PointXYZIRADRT. You may be using legacy "
        "code/data");
    }

    if (is_data_layout_compatible_with_point_xyzi(*cloud)) {
      RCLCPP_ERROR(
        get_logger(),
        "The pointcloud layout is compatible with PointXYZI. You may be using legacy "
        "code/data");
    }

    return;
  }

  if (!isValid(cloud)) {
    RCLCPP_ERROR(this->get_logger(), "[input_indices_callback] Invalid input!");
    return;
  }

  if (!isValid(indices)) {
    RCLCPP_ERROR(this->get_logger(), "[input_indices_callback] Invalid indices!");
    return;
  }

  if (indices) {
    RCLCPP_DEBUG(
      this->get_logger(),
      "[input_indices_callback]\n"
      "   - PointCloud with %d data points (%s), stamp %f, and frame %s on input topic received.\n"
      "   - PointIndices with %zu values, stamp %f, and frame %s on indices topic received.",
      cloud->width * cloud->height, pcl::getFieldsList(*cloud).c_str(),
      rclcpp::Time(cloud->header.stamp).seconds(), cloud->header.frame_id.c_str(),
      indices->indices.size(), rclcpp::Time(indices->header.stamp).seconds(),
      indices->header.frame_id.c_str());
  } else {
    RCLCPP_DEBUG(
      this->get_logger(),
      "[input_indices_callback] PointCloud with %d data points and frame %s on input topic "
      "received.",
      cloud->width * cloud->height, cloud->header.frame_id.c_str());
  }

  tf_input_orig_frame_ = cloud->header.frame_id;

  // For performance reason, defer the transform computation.
  // Do not use pcl_ros::transformPointCloud(). It's too slow due to the unnecessary copy.
  TransformInfo transform_info;
  if (!calculate_transform_matrix(tf_input_frame_, *cloud, transform_info)) return;

  // Need setInputCloud() here because we have to extract x/y/z
  pcl::IndicesPtr vindices;
  if (indices) {
    vindices.reset(new std::vector<int>(indices->indices));
  }

  auto output = std::make_unique<PointCloud2>();

  // TODO(sykwer): Change to `filter()` call after when the filter nodes conform to new API.
  faster_filter(cloud, vindices, *output, transform_info);

  if (!convert_output_costly(output)) return;

  output->header.stamp = cloud->header.stamp;
  pub_output_->publish(std::move(output));
  published_time_publisher_->publish_if_subscribed(pub_output_, cloud->header.stamp);
}

void GroundFilterComponent::convertPointcloud(
  const sensor_msgs::msg::PointCloud2::ConstSharedPtr & in_cloud,
  std::vector<PointCloudVector> & out_radial_ordered_points) const
{
  std::unique_ptr<autoware_utils_debug::ScopedTimeTrack> st_ptr;
  if (time_keeper_)
    st_ptr = std::make_unique<autoware_utils_debug::ScopedTimeTrack>(__func__, *time_keeper_);

  out_radial_ordered_points.resize(radial_dividers_num_);
  PointData current_point;

  const auto inv_radial_divider_angle_rad = 1.0f / radial_divider_angle_rad_;

  const size_t in_cloud_data_size = in_cloud->data.size();
  const size_t in_cloud_point_step = in_cloud->point_step;

  {  // grouping pointcloud by its azimuth angle
    std::unique_ptr<autoware_utils_debug::ScopedTimeTrack> inner_st_ptr;
    if (time_keeper_)
      inner_st_ptr = std::make_unique<autoware_utils_debug::ScopedTimeTrack>(
        "azimuth_angle_grouping", *time_keeper_);

    pcl::PointXYZ input_point;
    for (size_t data_index = 0; data_index + in_cloud_point_step <= in_cloud_data_size;
         data_index += in_cloud_point_step) {
      // Get Point
      data_accessor_.getPoint(in_cloud, data_index, input_point);

      // determine the azimuth angle group
      auto radius{static_cast<float>(std::hypot(input_point.x, input_point.y))};
      auto theta{normalize_radian(std::atan2(input_point.x, input_point.y), 0.0)};
      auto radial_div{static_cast<size_t>(std::floor(theta * inv_radial_divider_angle_rad))};

      current_point.radius = radius;
      current_point.point_state = PointLabel::INIT;
      current_point.data_index = data_index;

      // store the point in the corresponding radial division
      out_radial_ordered_points[radial_div].emplace_back(current_point);
    }
  }

  {  // sorting pointcloud by distance, on each azimuth angle group
    std::unique_ptr<autoware_utils_debug::ScopedTimeTrack> inner_st_ptr;
    if (time_keeper_)
      inner_st_ptr = std::make_unique<autoware_utils_debug::ScopedTimeTrack>("sort", *time_keeper_);

    for (size_t i = 0; i < radial_dividers_num_; ++i) {
      std::sort(
        out_radial_ordered_points[i].begin(), out_radial_ordered_points[i].end(),
        [](const PointData & a, const PointData & b) { return a.radius < b.radius; });
    }
  }
}

void GroundFilterComponent::calcVirtualGroundOrigin(pcl::PointXYZ & point) const
{
  point.x = vehicle_info_.wheel_base_m;
  point.y = 0;
  point.z = 0;
}

void GroundFilterComponent::classifyPointCloud(
  const sensor_msgs::msg::PointCloud2::ConstSharedPtr & in_cloud,
  const std::vector<PointCloudVector> & in_radial_ordered_clouds,
  pcl::PointIndices & out_no_ground_indices) const
{
  std::unique_ptr<autoware_utils_debug::ScopedTimeTrack> st_ptr;
  if (time_keeper_)
    st_ptr = std::make_unique<autoware_utils_debug::ScopedTimeTrack>(__func__, *time_keeper_);

  out_no_ground_indices.indices.clear();

  const pcl::PointXYZ init_ground_point(0, 0, 0);
  pcl::PointXYZ virtual_ground_point(0, 0, 0);
  calcVirtualGroundOrigin(virtual_ground_point);

  // run the classification algorithm for each ray (azimuth division)
  for (size_t i = 0; i < in_radial_ordered_clouds.size(); ++i) {
    float prev_gnd_radius = 0.0f;
    float prev_gnd_slope = 0.0f;
    PointsCentroid ground_cluster, non_ground_cluster;
    PointLabel point_label_curr = PointLabel::INIT;

    pcl::PointXYZ prev_gnd_point(0, 0, 0), point_curr, point_prev;

    // iterate over the points in the ray
    for (size_t j = 0; j < in_radial_ordered_clouds[i].size(); ++j) {
      float points_distance = 0.0f;
      const float local_slope_max_angle = local_slope_max_angle_rad_;

      // set the previous point
      point_prev = point_curr;
      PointLabel point_label_prev = point_label_curr;

      // set the current point
      const PointData & pd = in_radial_ordered_clouds[i][j];
      point_label_curr = pd.point_state;

      data_accessor_.getPoint(in_cloud, pd.data_index, point_curr);
      if (j == 0) {
        bool is_front_side = (point_curr.x > virtual_ground_point.x);
        if (use_virtual_ground_point_ && is_front_side) {
          prev_gnd_point = virtual_ground_point;
        } else {
          prev_gnd_point = init_ground_point;
        }
        prev_gnd_radius = std::hypot(prev_gnd_point.x, prev_gnd_point.y);
        prev_gnd_slope = 0.0f;
        ground_cluster.initialize();
        non_ground_cluster.initialize();
        points_distance = calc_distance3d(point_curr, prev_gnd_point);
      } else {
        points_distance = calc_distance3d(point_curr, point_prev);
      }

      float radius_distance_from_gnd = pd.radius - prev_gnd_radius;
      float height_from_gnd = point_curr.z - prev_gnd_point.z;
      float height_from_obj = point_curr.z - non_ground_cluster.getAverageHeight();
      bool calculate_slope = true;
      bool is_point_close_to_prev =
        (points_distance <
         (pd.radius * radial_divider_angle_rad_ + split_points_distance_tolerance_));

      float global_slope_ratio = point_curr.z / pd.radius;
      // check points which is far enough from previous point
      if (global_slope_ratio > global_slope_max_ratio_) {
        point_label_curr = PointLabel::NON_GROUND;
        calculate_slope = false;
      } else if (
        (point_label_prev == PointLabel::NON_GROUND) &&
        (std::abs(height_from_obj) >= split_height_distance_)) {
        calculate_slope = true;
      } else if (is_point_close_to_prev && std::abs(height_from_gnd) < split_height_distance_) {
        // close to the previous point, set point follow label
        point_label_curr = PointLabel::POINT_FOLLOW;
        calculate_slope = false;
      }
      if (is_point_close_to_prev) {
        height_from_gnd = point_curr.z - ground_cluster.getAverageHeight();
        radius_distance_from_gnd = pd.radius - ground_cluster.getAverageRadius();
      }
      if (calculate_slope) {
        // far from the previous point
        auto local_slope = std::atan2(height_from_gnd, radius_distance_from_gnd);
        if (local_slope - prev_gnd_slope > local_slope_max_angle) {
          // the point is outside of the local slope threshold
          point_label_curr = PointLabel::NON_GROUND;
        } else {
          point_label_curr = PointLabel::GROUND;
        }
      }

      if (point_label_curr == PointLabel::GROUND) {
        ground_cluster.initialize();
        non_ground_cluster.initialize();
      }
      if (point_label_curr == PointLabel::NON_GROUND) {
        out_no_ground_indices.indices.push_back(pd.data_index);
      } else if (  // NOLINT
        (point_label_prev == PointLabel::NON_GROUND) &&
        (point_label_curr == PointLabel::POINT_FOLLOW)) {
        point_label_curr = PointLabel::NON_GROUND;
        out_no_ground_indices.indices.push_back(pd.data_index);
      } else if (  // NOLINT
        (point_label_prev == PointLabel::GROUND) &&
        (point_label_curr == PointLabel::POINT_FOLLOW)) {
        point_label_curr = PointLabel::GROUND;
      } else {
      }

      // update the ground state
      if (point_label_curr == PointLabel::GROUND) {
        prev_gnd_radius = pd.radius;
        prev_gnd_point = pcl::PointXYZ(point_curr.x, point_curr.y, point_curr.z);
        ground_cluster.addPoint(pd.radius, point_curr.z);
        prev_gnd_slope = ground_cluster.getAverageSlope();
      }
      // update the non ground state
      if (point_label_curr == PointLabel::NON_GROUND) {
        non_ground_cluster.addPoint(pd.radius, point_curr.z);
      }
    }
  }
}

void GroundFilterComponent::extractObjectPoints(
  const sensor_msgs::msg::PointCloud2::ConstSharedPtr & in_cloud_ptr,
  const pcl::PointIndices & in_indices, sensor_msgs::msg::PointCloud2 & out_object_cloud) const
{
  std::unique_ptr<autoware_utils_debug::ScopedTimeTrack> st_ptr;
  if (time_keeper_)
    st_ptr = std::make_unique<autoware_utils_debug::ScopedTimeTrack>(__func__, *time_keeper_);

  size_t output_data_size = 0;

  for (const auto & idx : in_indices.indices) {
    std::memcpy(
      &out_object_cloud.data[output_data_size], &in_cloud_ptr->data[idx],
      in_cloud_ptr->point_step * sizeof(uint8_t));
    output_data_size += in_cloud_ptr->point_step;
  }
}

void GroundFilterComponent::faster_filter(
  const sensor_msgs::msg::PointCloud2::ConstSharedPtr & input,
  [[maybe_unused]] const pcl::IndicesPtr & indices, sensor_msgs::msg::PointCloud2 & output,
  [[maybe_unused]] const TransformInfo & transform_info)
{
  std::unique_ptr<autoware_utils_debug::ScopedTimeTrack> st_ptr;
  if (time_keeper_)
    st_ptr = std::make_unique<autoware_utils_debug::ScopedTimeTrack>(__func__, *time_keeper_);

  std::scoped_lock lock(mutex_);

  if (stop_watch_ptr_) stop_watch_ptr_->toc("processing_time", true);

  if (!data_accessor_.isInitialized()) {
    data_accessor_.setField(input);
    ground_filter_ptr_->setDataAccessor(input);
  }

  pcl::PointIndices no_ground_indices;

  if (elevation_grid_mode_) {
    ground_filter_ptr_->process(input, no_ground_indices);
  } else {
    std::vector<PointCloudVector> radial_ordered_points;
    convertPointcloud(input, radial_ordered_points);
    classifyPointCloud(input, radial_ordered_points, no_ground_indices);
  }
  output.row_step = no_ground_indices.indices.size() * input->point_step;
  output.data.resize(output.row_step);
  output.width = no_ground_indices.indices.size();
  output.fields = input->fields;
  output.is_dense = true;
  output.height = input->height;
  output.is_bigendian = input->is_bigendian;
  output.point_step = input->point_step;
  output.header = input->header;

  extractObjectPoints(input, no_ground_indices, output);
  if (debug_publisher_ptr_ && stop_watch_ptr_) {
    const double cyclic_time_ms = stop_watch_ptr_->toc("cyclic_time", true);
    const double processing_time_ms = stop_watch_ptr_->toc("processing_time", true);
    debug_publisher_ptr_->publish<autoware_internal_debug_msgs::msg::Float64Stamped>(
      "debug/cyclic_time_ms", cyclic_time_ms);
    debug_publisher_ptr_->publish<autoware_internal_debug_msgs::msg::Float64Stamped>(
      "debug/processing_time_ms", processing_time_ms);
  }
}

// TODO(taisa1): Temporary Implementation: Delete this function definition when all the filter
// nodes conform to new API.
void GroundFilterComponent::filter(
  const sensor_msgs::msg::PointCloud2::ConstSharedPtr & input,
  [[maybe_unused]] const pcl::IndicesPtr & indices,
  const sensor_msgs::msg::PointCloud2 & output) const
{
  (void)input;
  (void)indices;
  (void)output;
}

rcl_interfaces::msg::SetParametersResult GroundFilterComponent::onParameter(
  const std::vector<rclcpp::Parameter> & param)
{
  if (get_param(param, "grid_size_m", grid_size_m_)) {
    // grid_ptr_->initialize(grid_size_m_, radial_divider_angle_rad_, grid_mode_switch_radius_);
  }
  if (get_param(param, "grid_mode_switch_radius", grid_mode_switch_radius_)) {
    // grid_ptr_->initialize(grid_size_m_, radial_divider_angle_rad_, grid_mode_switch_radius_);
  }
  double global_slope_max_angle_deg{get_parameter("global_slope_max_angle_deg").as_double()};
  if (get_param(param, "global_slope_max_angle_deg", global_slope_max_angle_deg)) {
    global_slope_max_angle_rad_ = deg2rad(global_slope_max_angle_deg);
    global_slope_max_ratio_ = std::tan(global_slope_max_angle_rad_);
    RCLCPP_DEBUG(
      this->get_logger(), "Setting global_slope_max_angle_rad to: %f.",
      global_slope_max_angle_rad_);
    RCLCPP_DEBUG(
      this->get_logger(), "Setting global_slope_max_ratio to: %f.", global_slope_max_ratio_);
  }
  double local_slope_max_angle_deg{get_parameter("local_slope_max_angle_deg").as_double()};
  if (get_param(param, "local_slope_max_angle_deg", local_slope_max_angle_deg)) {
    local_slope_max_angle_rad_ = deg2rad(local_slope_max_angle_deg);
    local_slope_max_ratio_ = std::tan(local_slope_max_angle_rad_);
    RCLCPP_DEBUG(
      this->get_logger(), "Setting local_slope_max_angle_rad to: %f.", local_slope_max_angle_rad_);
    RCLCPP_DEBUG(
      this->get_logger(), "Setting local_slope_max_ratio to: %f.", local_slope_max_ratio_);
  }
  double radial_divider_angle_deg{get_parameter("radial_divider_angle_deg").as_double()};
  if (get_param(param, "radial_divider_angle_deg", radial_divider_angle_deg)) {
    radial_divider_angle_rad_ = deg2rad(radial_divider_angle_deg);
    radial_dividers_num_ = std::ceil(2.0 * M_PI / radial_divider_angle_rad_);
    // grid_ptr_->initialize(grid_size_m_, radial_divider_angle_rad_, grid_mode_switch_radius_);
    RCLCPP_DEBUG(
      this->get_logger(), "Setting radial_divider_angle_rad to: %f.", radial_divider_angle_rad_);
    RCLCPP_DEBUG(this->get_logger(), "Setting radial_dividers_num to: %zu.", radial_dividers_num_);
  }
  if (get_param(param, "split_points_distance_tolerance", split_points_distance_tolerance_)) {
    RCLCPP_DEBUG(
      this->get_logger(), "Setting split_points_distance_tolerance to: %f.",
      split_points_distance_tolerance_);
  }
  if (get_param(param, "split_height_distance", split_height_distance_)) {
    RCLCPP_DEBUG(
      this->get_logger(), "Setting split_height_distance to: %f.", split_height_distance_);
  }
  if (get_param(param, "use_virtual_ground_point", use_virtual_ground_point_)) {
    RCLCPP_DEBUG_STREAM(
      this->get_logger(),
      "Setting use_virtual_ground_point to: " << std::boolalpha << use_virtual_ground_point_);
  }
  if (get_param(param, "use_recheck_ground_cluster", use_recheck_ground_cluster_)) {
    RCLCPP_DEBUG_STREAM(
      this->get_logger(),
      "Setting use_recheck_ground_cluster to: " << std::boolalpha << use_recheck_ground_cluster_);
  }

  // For pointcloud
  std::scoped_lock lock(mutex_);

  if (get_param(param, "input_frame", tf_input_frame_)) {
    RCLCPP_DEBUG(this->get_logger(), "Setting the input TF frame to: %s.", tf_input_frame_.c_str());
  }
  if (get_param(param, "output_frame", tf_output_frame_)) {
    RCLCPP_DEBUG(
      this->get_logger(), "Setting the output TF frame to: %s.", tf_output_frame_.c_str());
  }

  // Finally return the result
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  result.reason = "success";

  return result;
}
}  // namespace autoware::ground_filter

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(autoware::ground_filter::GroundFilterComponent)
