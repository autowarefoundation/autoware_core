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

#include <autoware/ndt_scan_matcher/ndt_scan_matcher_core.hpp>

#include <autoware_internal_debug_msgs/msg/float32_stamped.hpp>

#include <cstdint>
#include <memory>
#include <sstream>
#include <string>

namespace autoware::ndt_scan_matcher
{
namespace
{
DiagnosticsInterface * as_diag(void * d)
{
  return static_cast<DiagnosticsInterface *>(d);
}

std::string as_str(const std::uint8_t * p, std::size_t len)
{
  return std::string(reinterpret_cast<const char *>(p), len);
}

void diag_clear(void * d)
{
  as_diag(d)->clear();
}

void diag_add_key_value_bool(void * d, const std::uint8_t * key, std::size_t key_len, bool v)
{
  as_diag(d)->add_key_value(as_str(key, key_len), v);
}

void diag_add_key_value_i64(void * d, const std::uint8_t * key, std::size_t key_len, std::int64_t v)
{
  as_diag(d)->add_key_value(as_str(key, key_len), v);
}

void diag_add_key_value_f64(void * d, const std::uint8_t * key, std::size_t key_len, double v)
{
  as_diag(d)->add_key_value(as_str(key, key_len), v);
}

void diag_add_key_value_str(
  void * d, const std::uint8_t * key, std::size_t key_len, const std::uint8_t * val,
  std::size_t val_len)
{
  as_diag(d)->add_key_value(as_str(key, key_len), as_str(val, val_len));
}

void diag_update_level_and_message(
  void * d, std::int8_t level, const std::uint8_t * msg, std::size_t msg_len)
{
  as_diag(d)->update_level_and_message(level, as_str(msg, msg_len));
}

void diag_publish(void * d, std::int64_t stamp_ns)
{
  as_diag(d)->publish(rclcpp::Time(stamp_ns));
}

AwDiagnostics make_diagnostics(DiagnosticsInterface * d)
{
  return AwDiagnostics{
    d,
    diag_clear,
    diag_add_key_value_bool,
    diag_add_key_value_i64,
    diag_add_key_value_f64,
    diag_add_key_value_str,
    diag_update_level_and_message,
    diag_publish};
}
}  // namespace

autoware_internal_debug_msgs::msg::Float32Stamped make_float32_stamped(
  const builtin_interfaces::msg::Time & stamp, float data);

bool NDTScanMatcher::callback_sensor_points_main(
  sensor_msgs::msg::PointCloud2::ConstSharedPtr sensor_points_msg_in_sensor_frame)
{
  const auto exe_start_time = std::chrono::system_clock::now();

  const rclcpp::Time sensor_ros_time = sensor_points_msg_in_sensor_frame->header.stamp;


  return ndt_ptr_.with([&](const auto & ndt_ptr) {
    const AwHost host = make_host();
    const AwDiagnostics diag = make_diagnostics(diagnostics_scan_points_.get());
    const AwPointCloud2View view = make_pointcloud2_view(*sensor_points_msg_in_sensor_frame);
    const AwSensorPointsMatchParams match_params{
      param_.dynamic_map_loading.lidar_radius,
      param_.dynamic_map_loading.map_radius,
      param_.validation.initial_to_result_distance_tolerance_m,
      param_.ndt.regularization_scale_factor,
      param_.score_estimation.no_ground_points.enable,
      param_.score_estimation.no_ground_points.z_margin_for_ground_removal};

    bool is_converged = false;
    const int32_t status = autoware_ndt_scan_matcher_rs_node_on_sensor_points(
      rs_.raw(), ndt_ptr->raw_handle(), &host, &diag, &match_params, &view, &is_converged);
    if (status != 0) {
      return false;
    }

    const auto exe_end_time = std::chrono::system_clock::now();
    const auto duration_micro_sec =
      std::chrono::duration_cast<std::chrono::microseconds>(exe_end_time - exe_start_time).count();
    const auto exe_time = static_cast<float>(duration_micro_sec) / 1000.0f;
    diagnostics_scan_points_->add_key_value("execution_time", exe_time);
    if (exe_time > param_.validation.critical_upper_bound_exe_time_ms) {
      std::stringstream message;
      message << "NDT exe time is too long (took " << exe_time << " [ms]).";
      diagnostics_scan_points_->update_level_and_message(
        diagnostic_msgs::msg::DiagnosticStatus::WARN, message.str());
    }
    exe_time_pub_->publish(make_float32_stamped(sensor_ros_time, exe_time));
    return is_converged;
  });

}

}  // namespace autoware::ndt_scan_matcher
