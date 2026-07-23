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

/// 1 when this header declares the concepts, 0 when it expands to nothing.
///
/// The concepts need C++20: the standard library only exposes `<concepts>` -- and the
/// `std::convertible_to` these build on -- in C++20 mode. `autoware_package()` leaves
/// every target at `CMAKE_CXX_STANDARD 17`, so a target that wants the concepts opts in
/// with
///
///     target_compile_features(<target> PRIVATE cxx_std_20)
///
/// CMake raises that one target to `-std=c++20` and leaves every other target -- and
/// every C++17 consumer of the domain headers -- on `-std=c++17`. This is what the
/// package's own `generate_interface_manifest` and gtest targets do, on Humble/gcc-11
/// (Ubuntu 22.04) as well as Jazzy.
///
/// A target left at C++17 gets an empty header rather than a hard error, so including it
/// is always safe. Test this macro before naming anything the header declares.
#if __cplusplus >= 202002L
#define AUTOWARE_COMPONENT_INTERFACE_SPECS_HAS_CONCEPTS 1
#else
#define AUTOWARE_COMPONENT_INTERFACE_SPECS_HAS_CONCEPTS 0
#endif

#if AUTOWARE_COMPONENT_INTERFACE_SPECS_HAS_CONCEPTS

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

/// A service interface spec: service type + name. Services carry no QoS of their own
/// because there is exactly one service profile and no call site varies it; see
/// `service_qos` in utils.hpp.
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

#endif  // AUTOWARE_COMPONENT_INTERFACE_SPECS_HAS_CONCEPTS
#endif  // AUTOWARE__COMPONENT_INTERFACE_SPECS__CONCEPTS_HPP_
