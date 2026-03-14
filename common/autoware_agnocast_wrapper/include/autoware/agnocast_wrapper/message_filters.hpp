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

#pragma once

#include "autoware/agnocast_wrapper/autoware_agnocast_wrapper.hpp"

#ifdef USE_AGNOCAST_ENABLED

#include <agnocast/message_filters/subscriber.hpp>
#include <agnocast/message_filters/sync_policies/approximate_time.hpp>
#include <agnocast/message_filters/synchronizer.hpp>

#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>

#include <functional>
#include <memory>
#include <string>
#include <utility>

namespace autoware::agnocast_wrapper
{
namespace message_filters
{

/// @brief Wrapper message_filters Subscriber that switches between
///        rclcpp and agnocast message_filters at runtime.
template <class M>
class Subscriber
{
public:
  Subscriber() = default;

  Subscriber(
    rclcpp::Node * node, const std::string & topic,
    const rmw_qos_profile_t qos = rmw_qos_profile_default)
  {
    subscribe(node, topic, qos);
  }

  void subscribe(
    rclcpp::Node * node, const std::string & topic,
    const rmw_qos_profile_t qos = rmw_qos_profile_default)
  {
    if (use_agnocast()) {
      agnocast_sub_.subscribe(node, topic, qos);
    } else {
      rclcpp_sub_.subscribe(node, topic, qos);
    }
  }

  ::message_filters::Subscriber<M> & rclcpp_subscriber() { return rclcpp_sub_; }
  agnocast::message_filters::Subscriber<M> & agnocast_subscriber() { return agnocast_sub_; }

private:
  ::message_filters::Subscriber<M> rclcpp_sub_;
  agnocast::message_filters::Subscriber<M> agnocast_sub_;
};

/// @brief Wrapper ApproximateTime Synchronizer that switches between
///        rclcpp and agnocast message_filters at runtime.
///
/// The callback receives `(AUTOWARE_MESSAGE_SHARED_PTR(const M0)&&,
///                         AUTOWARE_MESSAGE_SHARED_PTR(const M1)&&)`.
/// In agnocast mode, non-owning shared_ptrs are created from the ipc_shared_ptrs,
/// preserving zero-copy semantics during the callback lifetime.
template <typename M0, typename M1>
class ApproximateTimeSynchronizer
{
public:
  using Callback = std::function<void(
    AUTOWARE_MESSAGE_SHARED_PTR(const M0) &&, AUTOWARE_MESSAGE_SHARED_PTR(const M1) &&)>;

  ApproximateTimeSynchronizer(uint32_t queue_size, Subscriber<M0> & sub0, Subscriber<M1> & sub1)
  {
    if (use_agnocast()) {
      agnocast_sync_ = std::make_shared<AgnocastSync>(
        AgnocastPolicy(queue_size), sub0.agnocast_subscriber(), sub1.agnocast_subscriber());
    } else {
      rclcpp_sync_ = std::make_shared<RclcppSync>(
        RclcppPolicy(queue_size), sub0.rclcpp_subscriber(), sub1.rclcpp_subscriber());
    }
  }

  void registerCallback(Callback callback)
  {
    stored_callback_ = std::move(callback);
    if (use_agnocast()) {
      agnocast_sync_->registerCallback(&ApproximateTimeSynchronizer::agnocastCallbackAdapter, this);
    } else {
      rclcpp_sync_->registerCallback(&ApproximateTimeSynchronizer::rclcppCallbackAdapter, this);
    }
  }

private:
  Callback stored_callback_;

  using M0Event = agnocast::message_filters::MessageEvent<const M0>;
  using M1Event = agnocast::message_filters::MessageEvent<const M1>;

  void agnocastCallbackAdapter(const M0Event & e0, const M1Event & e1)
  {
    // Wrap ipc_shared_ptr in message_ptr (copies ipc_shared_ptr refcount, not data)
    auto p0 =
      AUTOWARE_MESSAGE_SHARED_PTR(const M0)(agnocast::ipc_shared_ptr<const M0>(e0.getMessage()));
    auto p1 =
      AUTOWARE_MESSAGE_SHARED_PTR(const M1)(agnocast::ipc_shared_ptr<const M1>(e1.getMessage()));
    stored_callback_(std::move(p0), std::move(p1));
  }

  void rclcppCallbackAdapter(
    const typename M0::ConstSharedPtr & m0, const typename M1::ConstSharedPtr & m1)
  {
    auto p0 = AUTOWARE_MESSAGE_SHARED_PTR(const M0)(std::shared_ptr<const M0>(m0));
    auto p1 = AUTOWARE_MESSAGE_SHARED_PTR(const M1)(std::shared_ptr<const M1>(m1));
    stored_callback_(std::move(p0), std::move(p1));
  }

  using RclcppPolicy = ::message_filters::sync_policies::ApproximateTime<M0, M1>;
  using RclcppSync = ::message_filters::Synchronizer<RclcppPolicy>;
  std::shared_ptr<RclcppSync> rclcpp_sync_;

  using AgnocastPolicy = agnocast::message_filters::sync_policies::ApproximateTime<M0, M1>;
  using AgnocastSync = agnocast::message_filters::Synchronizer<AgnocastPolicy>;
  std::shared_ptr<AgnocastSync> agnocast_sync_;
};

/// @brief Policy and Synchronizer types that mirror the rclcpp message_filters API.
///        Allows node code to use the same pattern as rclcpp:
///          using SyncPolicy = sync_policies::ApproximateTime<M0, M1>;
///          using Sync = Synchronizer<SyncPolicy>;
///          sync = std::make_shared<Sync>(SyncPolicy(10), sub0, sub1);
namespace sync_policies
{
template <typename M0, typename M1>
struct ApproximateTime
{
  uint32_t queue_size;
  explicit ApproximateTime(uint32_t qs) : queue_size(qs) {}
};
}  // namespace sync_policies

template <typename Policy>
class Synchronizer;

template <typename M0, typename M1>
class Synchronizer<sync_policies::ApproximateTime<M0, M1>>
: public ApproximateTimeSynchronizer<M0, M1>
{
public:
  Synchronizer(
    sync_policies::ApproximateTime<M0, M1> policy, Subscriber<M0> & sub0, Subscriber<M1> & sub1)
  : ApproximateTimeSynchronizer<M0, M1>(policy.queue_size, sub0, sub1)
  {
  }
};

}  // namespace message_filters
}  // namespace autoware::agnocast_wrapper

#else

#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>

namespace autoware::agnocast_wrapper
{
namespace message_filters
{

template <class M>
using Subscriber = ::message_filters::Subscriber<M>;

namespace sync_policies
{
template <typename M0, typename M1>
using ApproximateTime = ::message_filters::sync_policies::ApproximateTime<M0, M1>;
}  // namespace sync_policies

template <typename Policy>
using Synchronizer = ::message_filters::Synchronizer<Policy>;

}  // namespace message_filters
}  // namespace autoware::agnocast_wrapper

#endif
