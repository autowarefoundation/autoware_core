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

#include <array>
#include <cstdint>
#include <optional>
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

void NDTScanMatcher::callback_timer()
{
  const rclcpp::Time ros_time_now = this->now();

  diagnostics_map_update_->clear();

  diagnostics_map_update_->add_key_value("timer_callback_time_stamp", ros_time_now.nanoseconds());

  // Activation + latest-EKF-position are Rust-owned (Phase 1 slice B); read them over the FFI.
  const bool node_is_activated = autoware_ndt_scan_matcher_rs_is_activated(rs_.raw());
  std::optional<geometry_msgs::msg::Point> latest_ekf_position;
  std::array<double, 3> ekf_xyz{};
  if (autoware_ndt_scan_matcher_rs_latest_ekf_position(rs_.raw(), ekf_xyz.data())) {
    geometry_msgs::msg::Point point;
    point.x = ekf_xyz[0];
    point.y = ekf_xyz[1];
    point.z = ekf_xyz[2];
    latest_ekf_position = point;
  }
  map_update_module_->callback_timer(node_is_activated, latest_ekf_position, diagnostics_map_update_);

  diagnostics_map_update_->publish(ros_time_now);
}

void NDTScanMatcher::callback_initial_pose(
  const geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr initial_pose_msg_ptr)
{
  // Callback-level: the whole body — diagnostics + activation/frame gates + buffer push +
  // latest-EKF-position — runs in Rust, driving the Rust-owned state on the handle (Phase 1 slice B).
  // The C++ shell only builds the diagnostics vtable + the pose view.
  const AwDiagnostics diag = make_diagnostics(diagnostics_initial_pose_.get());
  const AwPoseWithCovarianceStampedView view = make_pose_with_cov_view(*initial_pose_msg_ptr);
  autoware_ndt_scan_matcher_rs_node_on_initial_pose(rs_.raw(), &diag, &view);
}

void NDTScanMatcher::callback_regularization_pose(
  geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr pose_conv_msg_ptr)
{
  // Callback-level: the whole body (diagnostics + buffer push) runs in Rust. The pose now crosses as
  // a value view and is pushed into the Rust-owned regularization buffer on the handle (Phase 1
  // slice A) — no host vtable needed here.
  const AwDiagnostics diag = make_diagnostics(diagnostics_regularization_pose_.get());
  const AwPoseWithCovarianceStampedView view = make_pose_with_cov_view(*pose_conv_msg_ptr);
  autoware_ndt_scan_matcher_rs_node_on_regularization_pose(rs_.raw(), &diag, &view);
}

void NDTScanMatcher::service_trigger_node(
  const std_srvs::srv::SetBool::Request::SharedPtr req,
  std_srvs::srv::SetBool::Response::SharedPtr res)
{
  const rclcpp::Time ros_time_now = this->now();

  // Callback-level: the whole body — diagnostics + activation + buffer clear — runs in Rust, driving
  // the handle's Rust-owned state (Phase 1 slice B). The C++ shell only builds the diagnostics vtable
  // and assigns res->success.
  const AwDiagnostics diag = make_diagnostics(diagnostics_trigger_node_.get());
  res->success = autoware_ndt_scan_matcher_rs_node_on_trigger(
    rs_.raw(), &diag, req->data, ros_time_now.nanoseconds());
}

}  // namespace autoware::ndt_scan_matcher
