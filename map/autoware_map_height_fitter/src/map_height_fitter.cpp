// Copyright 2022 The Autoware Contributors
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

#include "autoware/map_height_fitter/map_height_fitter.hpp"

#include "map_height_fitter_kernel.hpp"

#include <autoware/agnocast_wrapper/autoware_agnocast_wrapper.hpp>
#include <autoware/agnocast_wrapper/node.hpp>
#include <autoware/agnocast_wrapper/parameter_client.hpp>
#include <autoware/agnocast_wrapper/tf2.hpp>
#include <autoware/lanelet2_utils/conversion.hpp>
#include <autoware/qos_utils/qos_compatibility.hpp>
#include <tf2_ros/transform_listener.hpp>

#include <autoware_map_msgs/msg/lanelet_map_bin.hpp>
#include <autoware_map_msgs/srv/get_partial_point_cloud_map.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <lanelet2_core/Forward.h>
#include <lanelet2_core/LaneletMap.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <cstddef>
#include <memory>
#include <string>

namespace autoware::map_height_fitter
{

struct MapHeightFitter::Impl
{
  static constexpr char enable_partial_load[] = "enable_partial_load";

  explicit Impl(autoware::agnocast_wrapper::Node * node);
  void on_pcd_map(const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg);
  void on_vector_map(const autoware_map_msgs::msg::LaneletMapBin::ConstSharedPtr msg);
  bool get_partial_point_cloud_map(const Point & point);
  double get_ground_height(const Point & point) const;
  std::optional<Point> fit(const Point & position, const std::string & frame);

  tf2::BufferCore tf2_buffer_;
  // The wrapper's listener, not tf2_ros', because an AgnocastOnly executor does not spin a
  // plain tf2_ros::TransformListener -- the agnocast backend has to own the /tf subscription.
  autoware::agnocast_wrapper::TransformListener tf2_listener_;
  std::string map_frame_;
  // Only the logger is kept, not the node: the endpoints this class needs are all created in the
  // constructor (or by the closures below), so holding the node would mean templating everything
  // that touches it.
  rclcpp::Logger logger_;

  std::string fit_target_;

  // for fitting by pointcloud_map_loader
  rclcpp::CallbackGroup::SharedPtr group_;
  pcl::PointCloud<pcl::PointXYZ>::Ptr map_cloud_;
  pcl::KdTreeFLANN<pcl::PointXYZ> map_cloud_kdtree_;
  AUTOWARE_CLIENT_PTR(autoware_map_msgs::srv::GetPartialPointCloudMap) cli_pcd_map_;
  AUTOWARE_SUBSCRIPTION_PTR(sensor_msgs::msg::PointCloud2) sub_pcd_map_;
  autoware::agnocast_wrapper::AsyncParametersClient::SharedPtr params_pcd_map_loader_;
  // Whether partial loading is enabled is only known once the map loader answers, so both ways of
  // setting up the pointcloud path are bound here, where the node type is still known.
  std::function<void()> setup_partial_pcd_map_;
  std::function<void()> setup_whole_pcd_map_;

  // for fitting by vector_map_loader
  lanelet::LaneletMapPtr vector_map_;
  AUTOWARE_SUBSCRIPTION_PTR(autoware_map_msgs::msg::LaneletMapBin) sub_vector_map_;
};

MapHeightFitter::Impl::Impl(autoware::agnocast_wrapper::Node * node)
: tf2_listener_(tf2_buffer_, *node), logger_(node->get_logger())
{
  fit_target_ = node->declare_parameter<std::string>("map_height_fitter.target");
  if (fit_target_ == "pointcloud_map") {
    setup_partial_pcd_map_ = [this, node]() {
      group_ = node->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
      cli_pcd_map_ = node->create_client<autoware_map_msgs::srv::GetPartialPointCloudMap>(
        "~/partial_map_load", AUTOWARE_DEFAULT_SERVICES_QOS_PROFILE(), group_);
    };
    setup_whole_pcd_map_ = [this, node]() {
      const auto durable_qos = rclcpp::QoS(1).transient_local();
      sub_pcd_map_ = node->create_subscription<sensor_msgs::msg::PointCloud2>(
        "~/pointcloud_map", durable_qos,
        std::bind(&MapHeightFitter::Impl::on_pcd_map, this, std::placeholders::_1));
    };

    const auto callback =
      [this](const std::shared_future<std::vector<rclcpp::Parameter>> & future) {
        bool partial_load = false;
        for (const auto & param : future.get()) {
          if (param.get_name() == enable_partial_load) {
            partial_load = param.as_bool();
          }
        }

        if (partial_load) {
          setup_partial_pcd_map_();
        } else {
          setup_whole_pcd_map_();
        }
      };

    const auto map_loader_name =
      node->declare_parameter<std::string>("map_height_fitter.map_loader_name");
    params_pcd_map_loader_ =
      autoware::agnocast_wrapper::create_async_parameters_client(node, map_loader_name);
    params_pcd_map_loader_->wait_for_service();
    params_pcd_map_loader_->get_parameters({enable_partial_load}, callback);

  } else if (fit_target_ == "vector_map") {
    const auto durable_qos = rclcpp::QoS(1).transient_local();
    sub_vector_map_ = node->create_subscription<autoware_map_msgs::msg::LaneletMapBin>(
      "~/vector_map", durable_qos,
      std::bind(&MapHeightFitter::Impl::on_vector_map, this, std::placeholders::_1));

  } else {
    throw std::runtime_error("invalid fit_target");
  }
}

void MapHeightFitter::Impl::on_pcd_map(const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg)
{
  map_frame_ = msg->header.frame_id;
  map_cloud_ = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  pcl::fromROSMsg(*msg, *map_cloud_);
  map_cloud_kdtree_ = build_pointcloud_xy_kdtree(*map_cloud_);
}

bool MapHeightFitter::Impl::get_partial_point_cloud_map(const Point & point)
{
  const auto & logger = logger_;

  if (!cli_pcd_map_) {
    RCLCPP_WARN_STREAM(logger, "Partial map loading in pointcloud_map_loader is not enabled");
    return false;
  }
  if (!cli_pcd_map_->service_is_ready()) {
    RCLCPP_WARN_STREAM(logger, "Partial map loading in pointcloud_map_loader is not ready");
    return false;
  }

  const auto req = std::make_shared<autoware_map_msgs::srv::GetPartialPointCloudMap::Request>();
  req->area.center_x = static_cast<float>(point.x);
  req->area.center_y = static_cast<float>(point.y);
  req->area.radius = 50;

  RCLCPP_DEBUG(logger, "Send request to map_loader");
  auto future = cli_pcd_map_->async_send_request(req);
  auto status = future.wait_for(std::chrono::seconds(1));
  while (status != std::future_status::ready) {
    RCLCPP_DEBUG(logger, "waiting response");
    if (!rclcpp::ok()) {
      return false;
    }
    status = future.wait_for(std::chrono::seconds(1));
  }

  const auto res = future.get();
  RCLCPP_DEBUG(
    logger, "Loaded partial pcd map from map_loader (grid size: %lu)",
    res->new_pointcloud_with_ids.size());

  sensor_msgs::msg::PointCloud2 pcd_msg;
  std::size_t total_data_size = 0;
  for (const auto & pcd_with_id : res->new_pointcloud_with_ids) {
    total_data_size += pcd_with_id.pointcloud.data.size();
  }
  pcd_msg.data.reserve(total_data_size);
  for (const auto & pcd_with_id : res->new_pointcloud_with_ids) {
    if (pcd_msg.width == 0) {
      pcd_msg = pcd_with_id.pointcloud;
    } else {
      pcd_msg.width += pcd_with_id.pointcloud.width;
      pcd_msg.row_step += pcd_with_id.pointcloud.row_step;
      pcd_msg.data.insert(
        pcd_msg.data.end(), pcd_with_id.pointcloud.data.begin(), pcd_with_id.pointcloud.data.end());
    }
  }
  map_frame_ = res->header.frame_id;
  map_cloud_ = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  pcl::fromROSMsg(pcd_msg, *map_cloud_);
  map_cloud_kdtree_ = build_pointcloud_xy_kdtree(*map_cloud_);
  return true;
}

void MapHeightFitter::Impl::on_vector_map(
  const autoware_map_msgs::msg::LaneletMapBin::ConstSharedPtr msg)
{
  vector_map_ = autoware::experimental::lanelet2_utils::remove_const(
    autoware::experimental::lanelet2_utils::from_autoware_map_msgs(*msg));
  map_frame_ = msg->header.frame_id;
}

double MapHeightFitter::Impl::get_ground_height(const Point & point) const
{
  const auto & logger = logger_;

  const double x = point.x;
  const double y = point.y;

  if (fit_target_ == "pointcloud_map") {
    return get_ground_height_from_pointcloud(*map_cloud_, map_cloud_kdtree_, x, y, point.z);
  }
  if (fit_target_ == "vector_map") {
    // The kernel runs the nearest-point search once; std::nullopt means no closest lanelet, in
    // which case we warn and fall back to the original point.z (matching the original behavior).
    const auto height = get_ground_height_from_vector_map(*vector_map_, x, y, point.z);
    if (!height) {
      RCLCPP_WARN_STREAM(logger, "failed to get closest lanelet");
      return point.z;
    }
    return *height;
  }

  return point.z;  // unreachable: fit() validates fit_target_ before calling get_ground_height
}

std::optional<Point> MapHeightFitter::Impl::fit(const Point & position, const std::string & frame)
{
  const auto & logger = logger_;
  RCLCPP_INFO_STREAM(logger, "fit_target: " << fit_target_ << ", frame: " << frame);

  Point point;
  point.x = position.x;
  point.y = position.y;
  point.z = position.z;

  RCLCPP_DEBUG(logger, "original point: %.3f %.3f %.3f", point.x, point.y, point.z);

  // prepare data
  if (fit_target_ == "pointcloud_map") {
    if (cli_pcd_map_) {  // if cli_pcd_map_ is available, prepare pointcloud map by partial loading
      if (!get_partial_point_cloud_map(position)) {
        RCLCPP_WARN_STREAM(logger, "failed to get partial point cloud map");
        return std::nullopt;
      }
    }  // otherwise, pointcloud map should be already prepared by on_pcd_map
    if (!map_cloud_) {
      RCLCPP_WARN_STREAM(logger, "point cloud map is not ready");
      return std::nullopt;
    }
  } else if (fit_target_ == "vector_map") {
    // vector_map_ should be already prepared by on_vector_map
    if (!vector_map_) {
      RCLCPP_WARN_STREAM(logger, "vector map is not ready");
      return std::nullopt;
    }
  } else {
    throw std::runtime_error("invalid fit_target");
  }

  // transform frame to map_frame_
  try {
    const auto stamped = tf2_buffer_.lookupTransform(map_frame_, frame, tf2::TimePointZero);
    tf2::doTransform(point, point, stamped);
  } catch (tf2::TransformException & exception) {
    RCLCPP_WARN_STREAM(logger, "failed to lookup transform: " << exception.what());
    return std::nullopt;
  }

  // fit height on map_frame_
  point.z = get_ground_height(point);

  // transform map_frame_ to frame
  try {
    const auto stamped = tf2_buffer_.lookupTransform(frame, map_frame_, tf2::TimePointZero);
    tf2::doTransform(point, point, stamped);
  } catch (tf2::TransformException & exception) {
    RCLCPP_WARN_STREAM(logger, "failed to lookup transform: " << exception.what());
    return std::nullopt;
  }

  RCLCPP_DEBUG(logger, "modified point: %.3f %.3f %.3f", point.x, point.y, point.z);

  return point;
}

MapHeightFitter::MapHeightFitter(autoware::agnocast_wrapper::Node * node)
{
  impl_ = std::make_unique<Impl>(node);
}

MapHeightFitter::~MapHeightFitter() = default;

std::optional<Point> MapHeightFitter::fit(const Point & position, const std::string & frame)
{
  return impl_->fit(position, frame);
}

}  // namespace autoware::map_height_fitter
