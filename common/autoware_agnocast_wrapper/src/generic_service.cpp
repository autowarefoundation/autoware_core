// Copyright 2026 TIER IV, Inc.
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

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/typesupport_helpers.hpp>
#include <rcpputils/shared_library.hpp>
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <rosidl_typesupport_introspection_cpp/message_introspection.hpp>
#include <rosidl_typesupport_introspection_cpp/service_introspection.hpp>

#include <rcl/client.h>
#include <rcl/service.h>
#include <rclcpp/version.h>
#include <rmw/error_handling.h>
#include <rmw/rmw.h>
#include <rosidl_runtime_c/service_type_support_struct.h>

#include <cstdint>
#include <map>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <utility>

#ifdef USE_AGNOCAST_ENABLED
#include <agnocast/agnocast.hpp>
#endif

namespace autoware::agnocast_wrapper
{
namespace
{

using rosidl_typesupport_introspection_cpp::MessageMembers;
using rosidl_typesupport_introspection_cpp::ServiceMembers;

// "package/srv/Type" -> {"package", "srv", "Type"}
struct TypeParts
{
  std::string package;
  std::string middle;
  std::string type_name;
};

TypeParts split_service_type(const std::string & service_type)
{
  const auto sep1 = service_type.find('/');
  const auto sep2 = sep1 == std::string::npos ? sep1 : service_type.find('/', sep1 + 1);
  if (sep2 == std::string::npos) {
    throw std::runtime_error(
      "Invalid service type '" + service_type + "': expected 'package/srv/TypeName'");
  }
  return {
    service_type.substr(0, sep1), service_type.substr(sep1 + 1, sep2 - sep1 - 1),
    service_type.substr(sep2 + 1)};
}

const rosidl_service_type_support_t * get_service_typesupport_handle(
  const std::string & service_type, const std::string & typesupport_identifier,
  rcpputils::SharedLibrary & library)
{
  const auto parts = split_service_type(service_type);
  const std::string symbol_name = typesupport_identifier + "__get_service_type_support_handle__" +
                                  parts.package + "__" + parts.middle + "__" + parts.type_name;
  if (!library.has_symbol(symbol_name)) {
    throw std::runtime_error(
      "Failed to find symbol '" + symbol_name + "' for service type '" + service_type + "'");
  }
  using HandleFunc = const rosidl_service_type_support_t * (*)();
  const auto * handle = reinterpret_cast<HandleFunc>(library.get_symbol(symbol_name))();
  if (!handle) {
    throw std::runtime_error("Service typesupport handle is null for '" + service_type + "'");
  }
  return handle;
}

/// Typesupport of a service type, loaded from its runtime type string. The handles stay valid
/// only while the libraries are loaded, so they are kept together.
struct ServiceTypeSupport
{
  std::shared_ptr<rcpputils::SharedLibrary> ts_lib;
  std::shared_ptr<rcpputils::SharedLibrary> introspection_lib;
  const rosidl_service_type_support_t * service_ts{nullptr};
  const MessageMembers * request_members{nullptr};
  const MessageMembers * response_members{nullptr};
};

ServiceTypeSupport load_service_typesupport(const std::string & service_type)
{
  const std::string ts_identifier = "rosidl_typesupport_cpp";
  const std::string introspection_identifier = "rosidl_typesupport_introspection_cpp";

  ServiceTypeSupport ts;
  ts.ts_lib = rclcpp::get_typesupport_library(service_type, ts_identifier);
  ts.service_ts = get_service_typesupport_handle(service_type, ts_identifier, *ts.ts_lib);

  ts.introspection_lib = rclcpp::get_typesupport_library(service_type, introspection_identifier);
  const auto * introspection_ts =
    get_service_typesupport_handle(service_type, introspection_identifier, *ts.introspection_lib);
  const auto * members = static_cast<const ServiceMembers *>(introspection_ts->data);
  if (!members) {
    throw std::runtime_error("Failed to get ServiceMembers for '" + service_type + "'");
  }
  ts.request_members = members->request_members_;
  ts.response_members = members->response_members_;
  return ts;
}

std::shared_ptr<void> allocate_message(const MessageMembers * members)
{
  auto message = std::shared_ptr<void>(new uint8_t[members->size_of_], [members](void * ptr) {
    members->fini_function(ptr);
    delete[] static_cast<uint8_t *>(ptr);
  });
  members->init_function(message.get(), rosidl_runtime_cpp::MessageInitialization::ZERO);
  return message;
}

rcl_service_options_t make_service_options(const rclcpp::QoS & qos)
{
  rcl_service_options_t options = rcl_service_get_default_options();
  options.qos = qos.get_rmw_qos_profile();
  return options;
}

rcl_client_options_t make_client_options(const rclcpp::QoS & qos)
{
  rcl_client_options_t options = rcl_client_get_default_options();
  options.qos = qos.get_rmw_qos_profile();
  return options;
}

// ===== rclcpp backend =====

// TODO(Kilted): Use rclcpp::GenericService instead of this once Jazzy is dropped.
/// rclcpp::ServiceBase for a runtime service type. Humble's ServiceBase does not allocate the rcl
/// handle itself, so this does.
class RclcppGenericServiceImpl : public rclcpp::ServiceBase
{
public:
  using RequestCallback =
    std::function<void(std::shared_ptr<rmw_request_id_t>, std::shared_ptr<void>)>;

  RclcppGenericServiceImpl(
    std::shared_ptr<rcl_node_t> node_handle, const std::string & service_name,
    const std::string & service_type, RequestCallback callback,
    const rcl_service_options_t & service_options)
  : rclcpp::ServiceBase(node_handle),
    ts_(load_service_typesupport(service_type)),
    callback_(std::move(callback))
  {
    service_handle_ = std::shared_ptr<rcl_service_t>(
      new rcl_service_t, [handle = node_handle](rcl_service_t * service) {
        if (rcl_service_fini(service, handle.get()) != RCL_RET_OK) {
          RCLCPP_ERROR(
            rclcpp::get_node_logger(handle.get()), "Error in destruction of rcl service handle: %s",
            rcl_get_error_string().str);
          rcl_reset_error();
        }
        delete service;
      });
    *service_handle_ = rcl_get_zero_initialized_service();

    const rcl_ret_t ret = rcl_service_init(
      service_handle_.get(), node_handle.get(), ts_.service_ts, service_name.c_str(),
      &service_options);
    if (ret != RCL_RET_OK) {
      rclcpp::exceptions::throw_from_rcl_error(ret, "Failed to create generic service");
    }
  }

  std::shared_ptr<void> create_request() override { return allocate_message(ts_.request_members); }

  std::shared_ptr<rmw_request_id_t> create_request_header() override
  {
    return std::make_shared<rmw_request_id_t>();
  }

  void handle_request(
    std::shared_ptr<rmw_request_id_t> request_header, std::shared_ptr<void> request) override
  {
    callback_(std::move(request_header), std::move(request));
  }

  void send_response(rmw_request_id_t & request_header, const void * response)
  {
    const rcl_ret_t ret =
      rcl_send_response(get_service_handle().get(), &request_header, const_cast<void *>(response));
    if (ret != RCL_RET_OK) {
      rclcpp::exceptions::throw_from_rcl_error(ret, "Failed to send generic service response");
    }
  }

private:
  ServiceTypeSupport ts_;
  RequestCallback callback_;
};

class ROS2GenericService : public GenericService
{
  class ROS2Request : public Request
  {
  public:
    ROS2Request(std::shared_ptr<rmw_request_id_t> header, std::shared_ptr<void> payload)
    : header_(std::move(header)), payload_(std::move(payload))
    {
    }
    const void * get() const override { return payload_.get(); }

    std::shared_ptr<rmw_request_id_t> header_;
    std::shared_ptr<void> payload_;
    bool answered_{false};
  };

public:
  ROS2GenericService(
    rclcpp::Node * node, const std::string & service_name, const std::string & service_type,
    Callback callback, const rclcpp::QoS & qos, const rclcpp::CallbackGroup::SharedPtr & group)
  : callback_(std::move(callback)),
    service_(std::make_shared<RclcppGenericServiceImpl>(
      node->get_node_base_interface()->get_shared_rcl_node_handle(), service_name, service_type,
      [this](std::shared_ptr<rmw_request_id_t> header, std::shared_ptr<void> payload) {
        callback_(
          shared_from_this(), std::make_shared<ROS2Request>(std::move(header), std::move(payload)));
      },
      make_service_options(qos)))
  {
    node->get_node_services_interface()->add_service(
      std::dynamic_pointer_cast<rclcpp::ServiceBase>(service_), group);
  }

  const char * get_service_name() const override { return service_->get_service_name(); }

  void send_response(const RequestPtr & request, const void * response) override
  {
    auto * ros2_request = dynamic_cast<ROS2Request *>(request.get());
    if (!ros2_request) {
      throw std::runtime_error("send_response(): the request does not belong to this service");
    }
    std::lock_guard<std::mutex> lock(mutex_);
    if (ros2_request->answered_) {
      throw std::runtime_error("send_response(): the request has already been answered");
    }
    service_->send_response(*ros2_request->header_, response);
    ros2_request->answered_ = true;
  }

private:
  Callback callback_;
  std::shared_ptr<RclcppGenericServiceImpl> service_;
  std::mutex mutex_;
};

// TODO(Jazzy): Use rclcpp::GenericClient instead of this once Humble is dropped.
/// rclcpp::ClientBase for a runtime service type.
class RclcppGenericClientImpl : public rclcpp::ClientBase
{
public:
  using SharedFuture = GenericClient::SharedFuture;
  using Callback = GenericClient::Callback;

  RclcppGenericClientImpl(
    rclcpp::node_interfaces::NodeBaseInterface * node_base,
    rclcpp::node_interfaces::NodeGraphInterface::SharedPtr node_graph,
    const std::string & service_name, const std::string & service_type,
    const rcl_client_options_t & client_options)
  : rclcpp::ClientBase(node_base, std::move(node_graph)),
    ts_(load_service_typesupport(service_type))
  {
    const rcl_ret_t ret = rcl_client_init(
      get_client_handle().get(), node_base->get_rcl_node_handle(), ts_.service_ts,
      service_name.c_str(), &client_options);
    if (ret != RCL_RET_OK) {
      rclcpp::exceptions::throw_from_rcl_error(ret, "Failed to create generic client");
    }
  }

  std::shared_ptr<void> create_response() override
  {
    return allocate_message(ts_.response_members);
  }

  std::shared_ptr<rmw_request_id_t> create_request_header() override
  {
    return std::make_shared<rmw_request_id_t>();
  }

  void handle_response(
    std::shared_ptr<rmw_request_id_t> request_header, std::shared_ptr<void> response) override
  {
    Callback callback;
    SharedFuture future;
    {
      std::lock_guard<std::mutex> lock(pending_requests_mutex_);
      const auto it = pending_requests_.find(request_header->sequence_number);
      if (it == pending_requests_.end()) {
        return;
      }
      callback = std::move(it->second.callback);
      future = it->second.future;
      it->second.promise.set_value(std::move(response));
      pending_requests_.erase(it);
    }
    if (callback) {
      callback(future);
    }
  }

  SharedFuture async_send_request(const void * request, Callback callback)
  {
    std::lock_guard<std::mutex> lock(pending_requests_mutex_);

    int64_t sequence_number = 0;
    const rcl_ret_t ret = rcl_send_request(get_client_handle().get(), request, &sequence_number);
    if (ret != RCL_RET_OK) {
      rclcpp::exceptions::throw_from_rcl_error(ret, "Failed to send generic service request");
    }

    PendingRequest pending;
    pending.callback = std::move(callback);
    pending.future = pending.promise.get_future().share();
    auto future = pending.future;
    pending_requests_[sequence_number] = std::move(pending);
    return future;
  }

private:
  struct PendingRequest
  {
    std::promise<GenericClient::SharedResponse> promise;
    SharedFuture future;
    Callback callback;
  };

  ServiceTypeSupport ts_;
  std::mutex pending_requests_mutex_;
  std::map<int64_t, PendingRequest> pending_requests_;
};

class ROS2GenericClient : public GenericClient
{
public:
  ROS2GenericClient(
    rclcpp::Node * node, const std::string & service_name, const std::string & service_type,
    const rclcpp::QoS & qos, const rclcpp::CallbackGroup::SharedPtr & group)
  : client_(std::make_shared<RclcppGenericClientImpl>(
      node->get_node_base_interface().get(), node->get_node_graph_interface(), service_name,
      service_type, make_client_options(qos)))
  {
    node->get_node_services_interface()->add_client(
      std::dynamic_pointer_cast<rclcpp::ClientBase>(client_), group);
  }

  const char * get_service_name() const override { return client_->get_service_name(); }

  bool service_is_ready() const override { return client_->service_is_ready(); }

  SharedFuture async_send_request(const void * request, Callback callback) override
  {
    return client_->async_send_request(request, std::move(callback));
  }

private:
  std::shared_ptr<RclcppGenericClientImpl> client_;
};

#ifdef USE_AGNOCAST_ENABLED

// ===== Agnocast backend =====

/// Copies a message of a runtime type into a buffer already initialized for that type, by
/// serializing and deserializing it, as Agnocast's own service bridge does.
class MessageCopier
{
public:
  explicit MessageCopier(const std::string & message_type)
  : lib_(rclcpp::get_typesupport_library(message_type, "rosidl_typesupport_cpp")),
#if RCLCPP_VERSION_MAJOR >= 28
    ts_(rclcpp::get_message_typesupport_handle(message_type, "rosidl_typesupport_cpp", *lib_))
#else
    ts_(rclcpp::get_typesupport_handle(message_type, "rosidl_typesupport_cpp", *lib_))
#endif
  {
  }

  bool copy_message(const void * src, void * dst) const
  {
    rclcpp::SerializedMessage serialized;
    if (rmw_serialize(src, ts_, &serialized.get_rcl_serialized_message()) != RMW_RET_OK) {
      rmw_reset_error();
      return false;
    }
    if (rmw_deserialize(&serialized.get_rcl_serialized_message(), ts_, dst) != RMW_RET_OK) {
      rmw_reset_error();
      return false;
    }
    return true;
  }

private:
  std::shared_ptr<rcpputils::SharedLibrary> lib_;
  const rosidl_message_type_support_t * ts_;
};

class AgnocastGenericService : public GenericService
{
  class AgnocastRequest : public Request
  {
  public:
    explicit AgnocastRequest(agnocast::ipc_shared_ptr<void> && payload)
    : payload_(std::move(payload))
    {
    }
    const void * get() const override { return payload_.get(); }

    agnocast::ipc_shared_ptr<void> payload_;
    bool answered_{false};
  };

public:
  AgnocastGenericService(
    agnocast::Node * node, const std::string & service_name, const std::string & service_type,
    Callback callback, const rclcpp::QoS & qos, const rclcpp::CallbackGroup::SharedPtr & group)
  : callback_(std::move(callback)), response_copier_(service_type + "_Response")
  {
    service_ = std::make_shared<agnocast::GenericService>(
      node, service_name, service_type,
      [this](
        const std::shared_ptr<agnocast::GenericService> &,
        agnocast::ipc_shared_ptr<void> && payload) {
        callback_(shared_from_this(), std::make_shared<AgnocastRequest>(std::move(payload)));
      },
      qos, group);
  }

  const char * get_service_name() const override { return service_->get_service_name(); }

  void send_response(const RequestPtr & request, const void * response) override
  {
    auto * agnocast_request = dynamic_cast<AgnocastRequest *>(request.get());
    if (!agnocast_request) {
      throw std::runtime_error("send_response(): the request does not belong to this service");
    }
    std::lock_guard<std::mutex> lock(mutex_);
    if (agnocast_request->answered_) {
      throw std::runtime_error("send_response(): the request has already been answered");
    }
    agnocast_request->answered_ = true;

    auto agnocast_response = service_->borrow_loaned_response(agnocast_request->payload_);
    if (!response_copier_.copy_message(response, agnocast_response.get())) {
      service_->cancel_response(
        std::move(agnocast_request->payload_), std::move(agnocast_response));
      throw std::runtime_error("send_response(): failed to copy the response");
    }
    service_->send_response(std::move(agnocast_request->payload_), std::move(agnocast_response));
  }

private:
  Callback callback_;
  MessageCopier response_copier_;
  std::shared_ptr<agnocast::GenericService> service_;
  std::mutex mutex_;
};

class AgnocastGenericClient : public GenericClient
{
public:
  AgnocastGenericClient(
    agnocast::Node * node, const std::string & service_name, const std::string & service_type,
    const rclcpp::QoS & qos, const rclcpp::CallbackGroup::SharedPtr & group)
  : request_copier_(service_type + "_Request"),
    client_(std::make_shared<agnocast::GenericClient>(node, service_name, service_type, qos, group))
  {
  }

  const char * get_service_name() const override { return client_->get_service_name(); }

  bool service_is_ready() const override { return client_->service_is_ready(); }

  SharedFuture async_send_request(const void * request, Callback callback) override
  {
    auto agnocast_request = client_->borrow_loaned_request();
    if (!request_copier_.copy_message(request, agnocast_request.get())) {
      client_->cancel_request(std::move(agnocast_request));
      throw std::runtime_error("async_send_request(): failed to copy the request");
    }

    auto promise = std::make_shared<std::promise<SharedResponse>>();
    SharedFuture future = promise->get_future().share();
    client_->async_send_request(
      std::move(agnocast_request),
      [promise, future, callback = std::move(callback)](
        const agnocast::GenericClient::SharedFuture & agnocast_future) {
        try {
          auto holder = std::make_shared<agnocast::ipc_shared_ptr<void>>(agnocast_future.get());
          promise->set_value(SharedResponse(holder, holder->get()));
        } catch (...) {
          promise->set_exception(std::current_exception());
          return;
        }
        if (callback) {
          callback(future);
        }
      });
    return future;
  }

private:
  MessageCopier request_copier_;
  std::shared_ptr<agnocast::GenericClient> client_;
};

#endif  // USE_AGNOCAST_ENABLED

}  // namespace

namespace detail
{

GenericService::SharedPtr create_ros2_generic_service(
  rclcpp::Node * node, const std::string & service_name, const std::string & service_type,
  GenericService::Callback callback, const rclcpp::QoS & qos,
  const rclcpp::CallbackGroup::SharedPtr & group)
{
  return std::make_shared<ROS2GenericService>(
    node, service_name, service_type, std::move(callback), qos, group);
}

GenericClient::SharedPtr create_ros2_generic_client(
  rclcpp::Node * node, const std::string & service_name, const std::string & service_type,
  const rclcpp::QoS & qos, const rclcpp::CallbackGroup::SharedPtr & group)
{
  return std::make_shared<ROS2GenericClient>(node, service_name, service_type, qos, group);
}

#ifdef USE_AGNOCAST_ENABLED
GenericService::SharedPtr create_agnocast_generic_service(
  agnocast::Node * node, const std::string & service_name, const std::string & service_type,
  GenericService::Callback callback, const rclcpp::QoS & qos,
  const rclcpp::CallbackGroup::SharedPtr & group)
{
  return std::make_shared<AgnocastGenericService>(
    node, service_name, service_type, std::move(callback), qos, group);
}

GenericClient::SharedPtr create_agnocast_generic_client(
  agnocast::Node * node, const std::string & service_name, const std::string & service_type,
  const rclcpp::QoS & qos, const rclcpp::CallbackGroup::SharedPtr & group)
{
  return std::make_shared<AgnocastGenericClient>(node, service_name, service_type, qos, group);
}
#endif

}  // namespace detail

}  // namespace autoware::agnocast_wrapper
