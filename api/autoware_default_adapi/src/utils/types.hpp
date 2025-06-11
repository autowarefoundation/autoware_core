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

#ifndef UTILS__TYPES_HPP_
#define UTILS__TYPES_HPP_

#include <rclcpp/rclcpp.hpp>

namespace autoware::default_adapi
{

template <class T>
using Pub = typename rclcpp::Publisher<T>::SharedPtr;
template <class T>
using Sub = typename rclcpp::Subscription<T>::SharedPtr;
template <class T>
using Cli = typename rclcpp::Client<T>::SharedPtr;
template <class T>
using Srv = typename rclcpp::Service<T>::SharedPtr;

}  // namespace autoware::default_adapi

#endif  // UTILS__TYPES_HPP_
