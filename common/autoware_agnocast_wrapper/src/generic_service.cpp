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

#include "autoware/agnocast_wrapper/generic_service.hpp"

#include <rclcpp/exceptions/exceptions.hpp>
#include <rclcpp/logging.hpp>
#include <rosidl_runtime_cpp/message_initialization.hpp>

#include <stdexcept>
#include <utility>

namespace autoware::agnocast_wrapper
{

void ROS2GenericService::init_service_handle(const rclcpp::QoS & qos)
{
  service_handle_ = std::shared_ptr<rcl_service_t>(
    new rcl_service_t, [handle = node_handle_, name = service_name_](rcl_service_t * service) {
      if (rcl_service_fini(service, handle.get()) != RCL_RET_OK) {
        RCLCPP_ERROR(
          rclcpp::get_node_logger(handle.get()),
          "Error in destruction of rcl service handle '%s': %s", name.c_str(),
          rcl_get_error_string().str);
        rcl_reset_error();
      }
      delete service;
    });
  *service_handle_ = rcl_get_zero_initialized_service();

  rcl_service_options_t options = rcl_service_get_default_options();
  options.qos = qos.get_rmw_qos_profile();

  const rcl_ret_t ret = rcl_service_init(
    service_handle_.get(), node_handle_.get(), ts_bundle_.service_ts, service_name_.c_str(),
    &options);
  if (ret != RCL_RET_OK) {
    rclcpp::exceptions::throw_from_rcl_error(ret, "Failed to create generic service");
  }
}

std::shared_ptr<void> ROS2GenericService::create_request()
{
  const auto * members = ts_bundle_.request_members;
  auto request = std::shared_ptr<void>(new uint8_t[members->size_of_], [members](void * ptr) {
    members->fini_function(ptr);
    delete[] static_cast<uint8_t *>(ptr);
  });
  members->init_function(request.get(), rosidl_runtime_cpp::MessageInitialization::ZERO);
  return request;
}

std::shared_ptr<rmw_request_id_t> ROS2GenericService::create_request_header()
{
  return std::make_shared<rmw_request_id_t>();
}

void ROS2GenericService::handle_request(
  std::shared_ptr<rmw_request_id_t> request_header, std::shared_ptr<void> request)
{
  if (dispatcher_.is_deferred()) {
    {
      std::lock_guard<std::mutex> lock(pending_mutex_);
      pending_[request.get()] = std::move(request_header);
    }
    dispatcher_.invoke_deferred(shared_from_this(), request);
    return;
  }

  auto response = allocate_response();
  dispatcher_.invoke_basic(request, response);
  send_response_impl(*request_header, response);
}

ROS2GenericService::SharedResponse ROS2GenericService::allocate_response()
{
  const auto * members = ts_bundle_.response_members;
  auto response = std::shared_ptr<void>(new uint8_t[members->size_of_], [members](void * ptr) {
    members->fini_function(ptr);
    delete[] static_cast<uint8_t *>(ptr);
  });
  members->init_function(response.get(), rosidl_runtime_cpp::MessageInitialization::ZERO);
  return response;
}

ROS2GenericService::SharedResponse ROS2GenericService::create_response(
  const SharedRequest & request)
{
  {
    std::lock_guard<std::mutex> lock(pending_mutex_);
    if (pending_.find(request.get()) == pending_.end()) {
      throw std::runtime_error(
        "ROS2GenericService::create_response(): request is not a pending deferred call on this "
        "service (already answered, or not obtained from this service's own callback)");
    }
  }
  return allocate_response();
}

void ROS2GenericService::send_response(const SharedRequest & request, SharedResponse response)
{
  std::shared_ptr<rmw_request_id_t> header;
  {
    std::lock_guard<std::mutex> lock(pending_mutex_);
    auto it = pending_.find(request.get());
    if (it == pending_.end()) {
      throw std::runtime_error(
        "ROS2GenericService::send_response(): request is not a pending deferred call on this "
        "service (already answered, or not obtained from this service's own callback)");
    }
    header = std::move(it->second);
    pending_.erase(it);
  }
  send_response_impl(*header, response);
}

void ROS2GenericService::cancel_response(const SharedRequest & request, SharedResponse /*response*/)
{
  // Tolerant of an unknown/already-resolved request, matching AgnocastGenericService's
  // cancel_response(): meant to be callable from error-handling code without its own try/catch.
  // response's own deleter (fini_function + delete[]) frees it normally as it goes out of scope;
  // there is nothing Agnocast-specific to release on this path.
  std::lock_guard<std::mutex> lock(pending_mutex_);
  pending_.erase(request.get());
}

void ROS2GenericService::send_response_impl(
  rmw_request_id_t & request_header, const SharedResponse & response)
{
  const rcl_ret_t ret =
    rcl_send_response(get_service_handle().get(), &request_header, response.get());
  if (ret != RCL_RET_OK) {
    rclcpp::exceptions::throw_from_rcl_error(ret, "Failed to send generic service response");
  }
}

}  // namespace autoware::agnocast_wrapper
