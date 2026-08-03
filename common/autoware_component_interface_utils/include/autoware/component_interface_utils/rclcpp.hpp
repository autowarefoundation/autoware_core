// Copyright 2022 TIER IV, Inc.
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

#ifndef AUTOWARE__COMPONENT_INTERFACE_UTILS__RCLCPP_HPP_
#define AUTOWARE__COMPONENT_INTERFACE_UTILS__RCLCPP_HPP_

#include <autoware/component_interface_utils/rclcpp/create_interface.hpp>
#include <autoware/component_interface_utils/rclcpp/interface.hpp>
#include <autoware/component_interface_utils/rclcpp/service_client.hpp>
#include <autoware/component_interface_utils/rclcpp/service_server.hpp>
#include <autoware/component_interface_utils/rclcpp/topic_publisher.hpp>
#include <autoware/component_interface_utils/rclcpp/topic_subscription.hpp>

#include <functional>
#include <memory>
#include <optional>
#include <utility>

namespace autoware::component_interface_utils
{

template <class NodeT = rclcpp::Node>
class NodeAdaptor
{
private:
  using CallbackGroup = rclcpp::CallbackGroup::SharedPtr;

  // Member-function callback shapes, keyed on the spec. The returning create_X<Spec>()
  // forms name their spec explicitly, so these are the primary definitions; the
  // SharedPtrT-keyed aliases below project the spec out of the wrapper's smart pointer
  // for the out-parameter forms, so both spellings stay provably the same shape.
  template <class SpecT, class InstanceT>
  using SpecMessagePtrCallback = void (InstanceT::*)(const typename SpecT::Message::ConstSharedPtr);
  template <class SpecT, class InstanceT>
  using SpecMessageRefCallback = void (InstanceT::*)(const typename SpecT::Message &);

  template <class SpecT, class InstanceT>
  using SpecServiceCallback = void (InstanceT::*)(
    const typename SpecT::Service::Request::SharedPtr,
    const typename SpecT::Service::Response::SharedPtr);

  template <class SharedPtrT, class InstanceT>
  using MessagePtrCallback =
    SpecMessagePtrCallback<typename SharedPtrT::element_type::SpecType, InstanceT>;
  template <class SharedPtrT, class InstanceT>
  using MessageRefCallback =
    SpecMessageRefCallback<typename SharedPtrT::element_type::SpecType, InstanceT>;

  template <class SharedPtrT, class InstanceT>
  using ServiceCallback =
    SpecServiceCallback<typename SharedPtrT::element_type::SpecType, InstanceT>;

public:
  /// Constructor. D is deduced separately from NodeT so that `NodeAdaptor(this)` on a derived
  /// node keeps NodeT at its default instead of deducing the derived type.
  template <class D>
  explicit NodeAdaptor(D * node)
  {
    interface_ = std::make_shared<NodeInterface<NodeT>>(node);
  }

  /// Create a client wrapper for logging.
  template <class SharedPtrT>
  void init_cli(SharedPtrT & cli, CallbackGroup group = nullptr) const
  {
    using SpecT = typename SharedPtrT::element_type::SpecType;
    cli = create_client_impl<SpecT>(interface_, group);
  }

  /// Create a service wrapper for logging.
  template <class SharedPtrT, class CallbackT>
  void init_srv(SharedPtrT & srv, CallbackT && callback, CallbackGroup group = nullptr) const
  {
    using SpecT = typename SharedPtrT::element_type::SpecType;
    srv = create_service_impl<SpecT>(interface_, std::forward<CallbackT>(callback), group);
  }

  /// Create a publisher using traits like services.
  template <class SharedPtrT>
  void init_pub(SharedPtrT & pub) const
  {
    using SpecT = typename SharedPtrT::element_type::SpecType;
    pub = create_publisher_impl<SpecT>(interface_->node);
  }

  /// Create a subscription using traits like services.
  template <class SharedPtrT, class CallbackT>
  void init_sub(SharedPtrT & sub, CallbackT && callback) const
  {
    using SpecT = typename SharedPtrT::element_type::SpecType;
    sub = create_subscription_impl<SpecT>(interface_->node, std::forward<CallbackT>(callback));
  }

  /// Relay message. Goes straight to the creation functions rather than through
  /// init_pub/init_sub, so this helper does not depend on the out-parameter forms.
  template <class P, class S>
  void relay_message(P & pub, S & sub) const
  {
    using PubSpecT = typename P::element_type::SpecType;
    using SubSpecT = typename S::element_type::SpecType;
    using MsgT = typename PubSpecT::Message::ConstSharedPtr;
    pub = create_publisher_impl<PubSpecT>(interface_->node);
    sub =
      create_subscription_impl<SubSpecT>(interface_->node, [pub](MsgT msg) { pub->publish(*msg); });
  }

  /// Relay service. Goes straight to the creation functions rather than through
  /// init_cli/init_srv, so this helper does not depend on the out-parameter forms.
  /// The client is deliberately left out of `group`: only the service joins the
  /// caller's (typically MutuallyExclusive) callback group, so the client's
  /// response can still be taken while the service callback that issued the call
  /// is blocked in Client::call.
  template <class C, class S>
  void relay_service(
    C & cli, S & srv, CallbackGroup group, std::optional<double> timeout = std::nullopt) const
  {
    using CliSpecT = typename C::element_type::SpecType;
    using SrvSpecT = typename S::element_type::SpecType;
    cli = create_client_impl<CliSpecT>(interface_, nullptr);
    srv = create_service_impl<SrvSpecT>(
      interface_, [cli, timeout](auto req, auto res) { *res = *cli->call(req, timeout); }, group);
  }

  /// Create a subscription wrapper for pointer callback.
  template <class SharedPtrT, class InstanceT>
  void init_sub(
    SharedPtrT & sub, InstanceT * instance,
    MessagePtrCallback<SharedPtrT, InstanceT> && callback) const
  {
    using SpecT = typename SharedPtrT::element_type::SpecType;
    using std::placeholders::_1;
    sub = create_subscription_impl<SpecT>(interface_->node, std::bind(callback, instance, _1));
  }

  /// Create a subscription wrapper for reference callback.
  template <class SharedPtrT, class InstanceT>
  void init_sub(
    SharedPtrT & sub, InstanceT * instance,
    MessageRefCallback<SharedPtrT, InstanceT> && callback) const
  {
    using SpecT = typename SharedPtrT::element_type::SpecType;
    using std::placeholders::_1;
    sub = create_subscription_impl<SpecT>(interface_->node, std::bind(callback, instance, _1));
  }

  /// Create a service wrapper for logging.
  template <class SharedPtrT, class InstanceT>
  void init_srv(
    SharedPtrT & srv, InstanceT * instance, ServiceCallback<SharedPtrT, InstanceT> && callback,
    CallbackGroup group = nullptr) const
  {
    using SpecT = typename SharedPtrT::element_type::SpecType;
    using std::placeholders::_1;
    using std::placeholders::_2;
    srv = create_service_impl<SpecT>(interface_, std::bind(callback, instance, _1, _2), group);
  }

  /// Create a publisher, returning it instead of assigning to an out-parameter.
  template <class SpecT>
  typename Publisher<SpecT, NodeT>::SharedPtr create_publisher()
  {
    return create_publisher_impl<SpecT>(interface_->node);
  }

  /// Create a subscription, returning it instead of assigning to an out-parameter.
  template <class SpecT, class CallbackT>
  typename Subscription<SpecT, NodeT>::SharedPtr create_subscription(CallbackT && callback)
  {
    return create_subscription_impl<SpecT>(interface_->node, std::forward<CallbackT>(callback));
  }

  /// Create a subscription bound to a member function taking the message pointer.
  template <class SpecT, class InstanceT>
  typename Subscription<SpecT, NodeT>::SharedPtr create_subscription(
    InstanceT * instance, SpecMessagePtrCallback<SpecT, InstanceT> && callback)
  {
    using std::placeholders::_1;
    return create_subscription<SpecT>(std::bind(callback, instance, _1));
  }

  /// Create a subscription bound to a member function taking the message by reference.
  template <class SpecT, class InstanceT>
  typename Subscription<SpecT, NodeT>::SharedPtr create_subscription(
    InstanceT * instance, SpecMessageRefCallback<SpecT, InstanceT> && callback)
  {
    using std::placeholders::_1;
    return create_subscription<SpecT>(std::bind(callback, instance, _1));
  }

  /// Create a service server, returning it instead of assigning to an out-parameter.
  template <class SpecT, class CallbackT>
  typename Service<SpecT, NodeT>::SharedPtr create_service(
    CallbackT && callback, rclcpp::CallbackGroup::SharedPtr group = nullptr)
  {
    return create_service_impl<SpecT>(interface_, std::forward<CallbackT>(callback), group);
  }

  /// Create a service server bound to a member function.
  template <class SpecT, class InstanceT>
  typename Service<SpecT, NodeT>::SharedPtr create_service(
    InstanceT * instance, SpecServiceCallback<SpecT, InstanceT> && callback,
    rclcpp::CallbackGroup::SharedPtr group = nullptr)
  {
    using std::placeholders::_1;
    using std::placeholders::_2;
    return create_service<SpecT>(std::bind(callback, instance, _1, _2), group);
  }

  /// Create a service client, returning it instead of assigning to an out-parameter.
  template <class SpecT>
  typename Client<SpecT, NodeT>::SharedPtr create_client(
    rclcpp::CallbackGroup::SharedPtr group = nullptr)
  {
    return create_client_impl<SpecT>(interface_, group);
  }

private:
  // Use a node pointer because shared_from_this cannot be used in constructor.
  typename NodeInterface<NodeT>::SharedPtr interface_;
};

}  // namespace autoware::component_interface_utils

#endif  // AUTOWARE__COMPONENT_INTERFACE_UTILS__RCLCPP_HPP_
