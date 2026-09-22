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

#include "autoware/agnocast_wrapper/generic_client.hpp"

#include <rclcpp/exceptions/exceptions.hpp>
#include <rosidl_runtime_cpp/message_initialization.hpp>

#include <stdexcept>
#include <utility>

namespace autoware::agnocast_wrapper
{

ROS2GenericClient::ROS2GenericClient(
  rclcpp::Node * node, const std::string & service_name, const std::string & service_type,
  const rclcpp::QoS & qos)
: rclcpp::ClientBase(node->get_node_base_interface().get(), node->get_node_graph_interface()),
  ts_bundle_(detail::load_service_typesupport(service_type))
{
  rcl_client_options_t options = rcl_client_get_default_options();
  options.qos = qos.get_rmw_qos_profile();

  const rcl_ret_t ret = rcl_client_init(
    get_client_handle().get(), node->get_node_base_interface()->get_rcl_node_handle(),
    ts_bundle_.service_ts, service_name.c_str(), &options);
  if (ret != RCL_RET_OK) {
    rclcpp::exceptions::throw_from_rcl_error(ret, "Failed to create generic client");
  }
}

std::shared_ptr<void> ROS2GenericClient::create_response()
{
  const auto * members = ts_bundle_.response_members;
  auto response = std::shared_ptr<void>(new uint8_t[members->size_of_], [members](void * ptr) {
    members->fini_function(ptr);
    delete[] static_cast<uint8_t *>(ptr);
  });
  members->init_function(response.get(), rosidl_runtime_cpp::MessageInitialization::ZERO);
  return response;
}

std::shared_ptr<rmw_request_id_t> ROS2GenericClient::create_request_header()
{
  return std::make_shared<rmw_request_id_t>();
}

void ROS2GenericClient::handle_response(
  std::shared_ptr<rmw_request_id_t> request_header, std::shared_ptr<void> response)
{
  PendingResponse pending;
  {
    std::lock_guard<std::mutex> lock(pending_mutex_);
    auto it = pending_.find(request_header->sequence_number);
    if (it == pending_.end()) {
      return;
    }
    pending = std::move(it->second);
    pending_.erase(it);
  }
  pending.promise.set_value(response);
  if (pending.callback) {
    pending.callback(pending.future);
  }
}

ROS2GenericClient::SharedRequest ROS2GenericClient::create_request()
{
  const auto * members = ts_bundle_.request_members;
  auto request = std::shared_ptr<void>(new uint8_t[members->size_of_], [members](void * ptr) {
    members->fini_function(ptr);
    delete[] static_cast<uint8_t *>(ptr);
  });
  members->init_function(request.get(), rosidl_runtime_cpp::MessageInitialization::ZERO);
  return request;
}

ROS2GenericClient::SharedFuture ROS2GenericClient::async_send_request(
  SharedRequest request, ResponseCallback callback)
{
  // The lock spans rcl_send_request() itself, not just the pending_ insertion below:
  // handle_response() takes this same mutex to look up pending_ by sequence number, so without
  // this the response could arrive and be dispatched (on another executor thread) in the window
  // between the send succeeding and the entry being registered, find nothing in pending_, and be
  // silently dropped -- permanently, since rcl/rmw does not redeliver it. This mirrors
  // rclcpp::Client::async_send_request_impl(), which holds its own pending-requests mutex across
  // the send for the same reason, and also serializes concurrent async_send_request() calls on
  // this same client instance the same way upstream does.
  std::lock_guard<std::mutex> lock(pending_mutex_);

  int64_t sequence_number = 0;
  const rcl_ret_t ret =
    rcl_send_request(get_client_handle().get(), request.get(), &sequence_number);
  if (ret != RCL_RET_OK) {
    rclcpp::exceptions::throw_from_rcl_error(ret, "Failed to send generic request");
  }

  PendingResponse pending;
  pending.callback = std::move(callback);
  pending.future = pending.promise.get_future().share();
  SharedFuture future = pending.future;
  pending_[sequence_number] = std::move(pending);
  return future;
}

ROS2GenericClient::SharedFuture ROS2GenericClient::async_send_request(SharedRequest request)
{
  return async_send_request(std::move(request), nullptr);
}

void ROS2GenericClient::cancel_request(SharedRequest /*request*/)
{
  // A true no-op: request's own deleter frees the heap buffer normally when it goes out of scope.
}

}  // namespace autoware::agnocast_wrapper
