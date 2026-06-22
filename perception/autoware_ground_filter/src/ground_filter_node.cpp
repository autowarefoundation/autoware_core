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

#include "ground_filter_node.hpp"

#include "ground_filter.hpp"
#include "sanity_check.hpp"

#include <autoware_utils_geometry/geometry.hpp>
#include <autoware_utils_math/normalization.hpp>
#include <autoware_utils_math/unit_conversion.hpp>
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
using autoware_utils_math::deg2rad;

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
    virtual_lidar_z_ = static_cast<float>(vehicle_info_.vehicle_height_m);

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
      param.virtual_lidar_x =
        static_cast<float>(vehicle_info_.wheel_base_m) / 2.0f + center_pcl_shift_;
      param.virtual_lidar_y = 0.0f;
      param.virtual_lidar_z = virtual_lidar_z_;

      param.elevation_grid_mode = elevation_grid_mode_;
      param.radial_dividers_num = std::ceil(2.0 * M_PI / param.radial_divider_angle_rad);
      param.split_points_distance_tolerance = split_points_distance_tolerance_;
      param.split_height_distance = split_height_distance_;
      param.use_virtual_ground_point = use_virtual_ground_point_;
      param.wheel_base_m = static_cast<float>(vehicle_info_.wheel_base_m);

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

  published_time_publisher_ = std::make_unique<autoware_utils_debug::PublishedTimePublisher>(this);
  RCLCPP_DEBUG(this->get_logger(), "[Filter Constructor] successfully created.");
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
      sync_input_indices_a_->registerCallback(
        std::bind(
          &GroundFilterComponent::faster_input_indices_callback, this, std::placeholders::_1,
          std::placeholders::_2));
    } else {
      sync_input_indices_e_ =
        std::make_shared<message_filters::Synchronizer<message_filters::sync_policies::ExactTime<
          sensor_msgs::msg::PointCloud2, pcl_msgs::msg::PointIndices>>>(max_queue_size_);
      sync_input_indices_e_->connectInput(sub_input_filter_, sub_indices_filter_);
      sync_input_indices_e_->registerCallback(
        std::bind(
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

  // Need setInputCloud() here because we have to extract x/y/z
  pcl::IndicesPtr vindices;
  if (indices) {
    vindices.reset(new std::vector<int>(indices->indices));
  }

  auto output = std::make_unique<PointCloud2>();

  // TODO(sykwer): Change to `filter()` call after when the filter nodes conform to new API.
  faster_filter(cloud, vindices, *output);

  output->header.stamp = cloud->header.stamp;
  pub_output_->publish(std::move(output));
  published_time_publisher_->publish_if_subscribed(pub_output_, cloud->header.stamp);
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
  [[maybe_unused]] const pcl::IndicesPtr & indices, sensor_msgs::msg::PointCloud2 & output)
{
  std::unique_ptr<autoware_utils_debug::ScopedTimeTrack> st_ptr;
  if (time_keeper_)
    st_ptr = std::make_unique<autoware_utils_debug::ScopedTimeTrack>(__func__, *time_keeper_);

  std::scoped_lock lock(mutex_);

  if (stop_watch_ptr_) stop_watch_ptr_->toc("processing_time", true);

  ground_filter_ptr_->setDataAccessor(input);

  pcl::PointIndices no_ground_indices;
  ground_filter_ptr_->process(input, no_ground_indices);

  // Package the output
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
  [[maybe_unused]] const pcl::IndicesPtr & indices, const sensor_msgs::msg::PointCloud2 & output)
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
    global_slope_max_angle_rad_ = static_cast<float>(deg2rad(global_slope_max_angle_deg));
    global_slope_max_ratio_ = std::tan(global_slope_max_angle_rad_);
    RCLCPP_DEBUG(
      this->get_logger(), "Setting global_slope_max_angle_rad to: %f.",
      global_slope_max_angle_rad_);
    RCLCPP_DEBUG(
      this->get_logger(), "Setting global_slope_max_ratio to: %f.", global_slope_max_ratio_);
  }
  double local_slope_max_angle_deg{get_parameter("local_slope_max_angle_deg").as_double()};
  if (get_param(param, "local_slope_max_angle_deg", local_slope_max_angle_deg)) {
    local_slope_max_angle_rad_ = static_cast<float>(deg2rad(local_slope_max_angle_deg));
    local_slope_max_ratio_ = std::tan(local_slope_max_angle_rad_);
    RCLCPP_DEBUG(
      this->get_logger(), "Setting local_slope_max_angle_rad to: %f.", local_slope_max_angle_rad_);
    RCLCPP_DEBUG(
      this->get_logger(), "Setting local_slope_max_ratio to: %f.", local_slope_max_ratio_);
  }
  double radial_divider_angle_deg{get_parameter("radial_divider_angle_deg").as_double()};
  if (get_param(param, "radial_divider_angle_deg", radial_divider_angle_deg)) {
    radial_divider_angle_rad_ = static_cast<float>(deg2rad(radial_divider_angle_deg));
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

  // Finally return the result
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  result.reason = "success";

  return result;
}
}  // namespace autoware::ground_filter

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(autoware::ground_filter::GroundFilterComponent)
