// Copyright 2026 The Autoware Contributors
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

#ifndef AUTOWARE__COMPONENT_INTERFACE_SPECS__CONCEPTS_HPP_
#define AUTOWARE__COMPONENT_INTERFACE_SPECS__CONCEPTS_HPP_

#include "autoware/component_interface_specs/version.hpp"

#if __cplusplus >= 202002L

#include <rmw/qos_profiles.h>

#include <concepts>
#include <cstddef>
#include <tuple>
#include <utility>

namespace autoware::component_interface_specs
{

/// A topic interface spec: message type + name + QoS triple.
template <class T>
concept InterfaceSpec = requires {
  typename T::Message;
  { T::name } -> std::convertible_to<const char *>;
  { T::depth } -> std::convertible_to<std::size_t>;
  { T::reliability } -> std::convertible_to<rmw_qos_reliability_policy_t>;
  { T::durability } -> std::convertible_to<rmw_qos_durability_policy_t>;
};

/// A service interface spec: service type + name.
template <class T>
concept ServiceSpec = requires {
  typename T::Service;
  { T::name } -> std::convertible_to<const char *>;
};

template <class T>
concept AnySpec = InterfaceSpec<T> || ServiceSpec<T>;

template <class Tuple, std::size_t... Is>
constexpr bool all_specs_valid_impl(std::index_sequence<Is...>)
{
  return (AnySpec<std::tuple_element_t<Is, Tuple>> && ...);
}

template <class Tuple>
constexpr bool all_specs_valid()
{
  return all_specs_valid_impl<Tuple>(std::make_index_sequence<std::tuple_size_v<Tuple>>{});
}

}  // namespace autoware::component_interface_specs

#endif  // __cplusplus >= 202002L
#endif  // AUTOWARE__COMPONENT_INTERFACE_SPECS__CONCEPTS_HPP_
