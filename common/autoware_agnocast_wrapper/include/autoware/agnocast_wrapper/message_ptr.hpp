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

// Ownership-parameterized message handle used by the Agnocast build.

#ifdef USE_AGNOCAST_ENABLED

#include <agnocast/agnocast.hpp>

#include <memory>
#include <utility>

namespace autoware::agnocast_wrapper
{

enum class OwnershipType { Unique, Shared };

template <typename MessageT, OwnershipType Ownership>
class message_interface;

template <typename MessageT>
class message_interface<MessageT, OwnershipType::Unique>
{
public:
  message_interface() = default;

  virtual ~message_interface() = default;

  message_interface(const message_interface & r) = delete;
  message_interface & operator=(const message_interface & r) = delete;

  message_interface(message_interface && r) = default;
  message_interface & operator=(message_interface && r) = default;

  virtual MessageT & as_ref() const noexcept = 0;
  virtual MessageT * as_ptr() const noexcept = 0;

  virtual agnocast::ipc_shared_ptr<MessageT> move_agnocast_ptr() && noexcept = 0;
  virtual std::unique_ptr<MessageT> move_ros2_ptr() && noexcept = 0;
};

template <typename MessageT>
class message_interface<MessageT, OwnershipType::Shared>
{
public:
  virtual ~message_interface() = default;

  virtual MessageT & as_ref() const noexcept = 0;
  virtual MessageT * as_ptr() const noexcept = 0;

  virtual agnocast::ipc_shared_ptr<MessageT> move_agnocast_ptr() && noexcept = 0;
  virtual std::shared_ptr<MessageT> move_ros2_ptr() && noexcept = 0;

  virtual std::unique_ptr<message_interface<MessageT, OwnershipType::Shared>> clone() const = 0;
};

template <typename MessageT, OwnershipType Ownership>
class agnocast_message;

template <typename MessageT>
class agnocast_message<MessageT, OwnershipType::Unique>
: public message_interface<MessageT, OwnershipType::Unique>
{
  agnocast::ipc_shared_ptr<MessageT> ptr_;

public:
  explicit agnocast_message(agnocast::ipc_shared_ptr<MessageT> && ptr) : ptr_(std::move(ptr)) {}

  MessageT & as_ref() const noexcept override { return *ptr_; }
  MessageT * as_ptr() const noexcept override { return ptr_.get(); }

  agnocast::ipc_shared_ptr<MessageT> move_agnocast_ptr() && noexcept override
  {
    return std::move(ptr_);
  }

  // The following member function should never be called at runtime. They are implemented just for
  // inheriting `message_interface`.
  std::unique_ptr<MessageT> move_ros2_ptr() && noexcept override
  {
    return std::unique_ptr<MessageT>{};
  }
};

template <typename MessageT>
class agnocast_message<MessageT, OwnershipType::Shared>
: public message_interface<MessageT, OwnershipType::Shared>
{
  agnocast::ipc_shared_ptr<MessageT> ptr_;

public:
  explicit agnocast_message(agnocast::ipc_shared_ptr<MessageT> && ptr) : ptr_(std::move(ptr)) {}

  MessageT & as_ref() const noexcept override { return *ptr_; }
  MessageT * as_ptr() const noexcept override { return ptr_.get(); }

  agnocast::ipc_shared_ptr<MessageT> move_agnocast_ptr() && noexcept override
  {
    return std::move(ptr_);
  }

  // The following member function should never be called at runtime. They are implemented just for
  // inheriting `message_interface`.
  std::shared_ptr<MessageT> move_ros2_ptr() && noexcept override
  {
    return std::shared_ptr<MessageT>{};
  }

  std::unique_ptr<message_interface<MessageT, OwnershipType::Shared>> clone() const override
  {
    return std::make_unique<agnocast_message<MessageT, OwnershipType::Shared>>(*this);
  }
};

template <typename MessageT, OwnershipType Ownership>
class ros2_message;

template <typename MessageT>
class ros2_message<MessageT, OwnershipType::Unique>
: public message_interface<MessageT, OwnershipType::Unique>
{
  std::unique_ptr<MessageT> ptr_;

public:
  explicit ros2_message(std::unique_ptr<MessageT> && ptr) : ptr_(std::move(ptr)) {}

  MessageT & as_ref() const noexcept override { return *ptr_; }
  MessageT * as_ptr() const noexcept override { return ptr_.get(); }

  std::unique_ptr<MessageT> move_ros2_ptr() && noexcept override { return std::move(ptr_); }

  // The following member function should never be called at runtime. They are implemented just for
  // inheriting `message_interface`.
  agnocast::ipc_shared_ptr<MessageT> move_agnocast_ptr() && noexcept override
  {
    return agnocast::ipc_shared_ptr<MessageT>{};
  }
};

template <typename MessageT>
class ros2_message<MessageT, OwnershipType::Shared>
: public message_interface<MessageT, OwnershipType::Shared>
{
  std::shared_ptr<MessageT> ptr_;

public:
  explicit ros2_message(std::shared_ptr<MessageT> && ptr) : ptr_(std::move(ptr)) {}

  MessageT & as_ref() const noexcept override { return *ptr_; }
  MessageT * as_ptr() const noexcept override { return ptr_.get(); }

  std::shared_ptr<MessageT> move_ros2_ptr() && noexcept override { return std::move(ptr_); }

  // The following member function should never be called at runtime. They are implemented just for
  // inheriting `message_interface`.
  agnocast::ipc_shared_ptr<MessageT> move_agnocast_ptr() && noexcept override
  {
    return agnocast::ipc_shared_ptr<MessageT>{};
  }

  std::unique_ptr<message_interface<MessageT, OwnershipType::Shared>> clone() const override
  {
    return std::make_unique<ros2_message<MessageT, OwnershipType::Shared>>(*this);
  }
};

template <typename MessageT, OwnershipType Ownership>
class message_ptr;

template <typename MessageT>
class message_ptr<MessageT, OwnershipType::Unique>
{
  using ros2_ptr_t = std::unique_ptr<MessageT>;

  std::unique_ptr<message_interface<MessageT, OwnershipType::Unique>> ptr_;

  template <typename U>
  friend class AgnocastPublisher;
  template <typename U>
  friend class ROS2Publisher;

private:
  agnocast::ipc_shared_ptr<MessageT> move_agnocast_ptr() && noexcept
  {
    return std::move(*(std::move(ptr_))).move_agnocast_ptr();
  }

  auto move_ros2_ptr() && noexcept { return std::move(*(std::move(ptr_))).move_ros2_ptr(); }

public:
  message_ptr() : ptr_(nullptr) {}

  explicit message_ptr(agnocast::ipc_shared_ptr<MessageT> && ptr)
  : ptr_(std::make_unique<agnocast_message<MessageT, OwnershipType::Unique>>(std::move(ptr)))
  {
  }

  explicit message_ptr(ros2_ptr_t && ptr)
  : ptr_(std::make_unique<ros2_message<MessageT, OwnershipType::Unique>>(std::move(ptr)))
  {
  }

  message_ptr(const message_ptr & r) = delete;
  message_ptr & operator=(const message_ptr & r) = delete;

  message_ptr(message_ptr && r) noexcept = default;
  message_ptr & operator=(message_ptr && r) noexcept = default;

  MessageT & operator*() const noexcept { return ptr_->as_ref(); }

  MessageT * operator->() const noexcept { return ptr_->as_ptr(); }

  explicit operator bool() const noexcept { return ptr_ && static_cast<bool>(ptr_->as_ptr()); }

  MessageT * get() const noexcept { return ptr_ ? ptr_->as_ptr() : nullptr; }
};

template <typename MessageT>
class message_ptr<MessageT, OwnershipType::Shared>
{
  using ros2_ptr_t = std::shared_ptr<MessageT>;

  std::unique_ptr<message_interface<MessageT, OwnershipType::Shared>> ptr_;

  template <typename U>
  friend class AgnocastPublisher;
  template <typename U>
  friend class ROS2Publisher;
  template <typename U>
  friend class ROS2Client;
  template <typename U>
  friend class AgnocastClient;

private:
  agnocast::ipc_shared_ptr<MessageT> move_agnocast_ptr() && noexcept
  {
    return std::move(*(std::move(ptr_))).move_agnocast_ptr();
  }

  auto move_ros2_ptr() && noexcept { return std::move(*(std::move(ptr_))).move_ros2_ptr(); }

public:
  message_ptr() : ptr_(nullptr) {}

  explicit message_ptr(agnocast::ipc_shared_ptr<MessageT> && ptr)
  : ptr_(std::make_unique<agnocast_message<MessageT, OwnershipType::Shared>>(std::move(ptr)))
  {
  }

  explicit message_ptr(ros2_ptr_t && ptr)
  : ptr_(std::make_unique<ros2_message<MessageT, OwnershipType::Shared>>(std::move(ptr)))
  {
  }

  message_ptr(const message_ptr & r)
  {
    if (r.ptr_ != nullptr) {
      ptr_ = r.ptr_->clone();
    }
  }
  message_ptr & operator=(const message_ptr & r)
  {
    if (this != &r) {
      if (r.ptr_ != nullptr) {
        ptr_ = r.ptr_->clone();
      } else {
        ptr_ = nullptr;
      }
    }
    return *this;
  }

  message_ptr(message_ptr && r) noexcept = default;
  message_ptr & operator=(message_ptr && r) noexcept = default;

  MessageT & operator*() const noexcept { return ptr_->as_ref(); }

  MessageT * operator->() const noexcept { return ptr_->as_ptr(); }

  explicit operator bool() const noexcept { return ptr_ && static_cast<bool>(ptr_->as_ptr()); }

  MessageT * get() const noexcept { return ptr_ ? ptr_->as_ptr() : nullptr; }
};

namespace detail
{

/// Adapt a subscription-received message to an interface that insists on std::shared_ptr. Every
/// live copy pins one agnocast shared-memory entry, and none of them may outlive the subscription
/// that delivered it: its destruction drops the kernel-side reference, so a later publish can
/// recycle the entry the copies still point at. Subscriber-side handles only: the address is read
/// once here, so a publisher-side handle would keep handing it out after publish() invalidates it.
template <typename MessageT>
std::shared_ptr<const MessageT> to_std_shared_ptr(agnocast::ipc_shared_ptr<const MessageT> && ptr)
{
  if (!ptr) {
    return nullptr;
  }
  auto holder = std::make_shared<agnocast::ipc_shared_ptr<const MessageT>>(std::move(ptr));
  return std::shared_ptr<const MessageT>(holder, holder->get());
}

}  // namespace detail

}  // namespace autoware::agnocast_wrapper

#endif
