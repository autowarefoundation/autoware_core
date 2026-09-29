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

#ifndef AUTOWARE__COMPONENT_INTERFACE_UTILS__RCLCPP__REGISTRATION_HPP_
#define AUTOWARE__COMPONENT_INTERFACE_UTILS__RCLCPP__REGISTRATION_HPP_

#include <rmw/types.h>

#include <cstddef>
#include <cstdint>
#include <string>
#include <type_traits>
#include <utility>

namespace autoware::component_interface_utils
{

/// One registered endpoint of this node: which spec, in which role, on which
/// remap-resolved name, with which actual QoS. Accumulated by the create/init
/// implementation layer -- every adaptor entry point registers, so a node's
/// manifest() is complete by the end of construction.
struct InterfaceRecord
{
  enum class Kind { Topic, Service };
  enum class Role { Provide, Require };

  Kind kind{Kind::Topic};
  Role role{Role::Provide};
  std::string interface_name;  // the spec-declared name (Spec::name)
  std::string resolved_name;   // after remapping
  std::string type_name;       // rosidl type identifier
  bool has_version{false};     // false for specs without a domain version
  std::uint16_t major{0};
  std::uint16_t minor{0};
  std::uint16_t patch{0};
  rmw_qos_reliability_policy_t reliability{RMW_QOS_POLICY_RELIABILITY_SYSTEM_DEFAULT};
  rmw_qos_durability_policy_t durability{RMW_QOS_POLICY_DURABILITY_SYSTEM_DEFAULT};
  std::size_t depth{0};
};

/// Detects whether ADL finds a usable resolve_domain_version(Spec) overload --
/// the hook AUTOWARE_COMPONENT_INTERFACE_SPECS_DEFINE_DOMAIN emits. False for
/// specs outside any versioned domain AND for specs a domain excluded with a
/// deleted overload (selecting a deleted function is a substitution failure).
template <class SpecT, class = void>
struct HasDomainVersion : std::false_type
{
};
template <class SpecT>
struct HasDomainVersion<
  SpecT, std::void_t<decltype(resolve_domain_version(std::declval<const SpecT &>()))>>
: std::true_type
{
};

/// Build the record for one endpoint. The version is resolved via ADL into the
/// spec's own namespace where declared, and omitted otherwise.
template <class SpecT>
InterfaceRecord make_record(
  InterfaceRecord::Kind kind, InterfaceRecord::Role role, std::string resolved_name,
  std::string type_name, const rmw_qos_profile_t & qos)
{
  InterfaceRecord record;
  record.kind = kind;
  record.role = role;
  record.interface_name = SpecT::name;
  record.resolved_name = std::move(resolved_name);
  record.type_name = std::move(type_name);
  if constexpr (HasDomainVersion<SpecT>::value) {
    const auto version = resolve_domain_version(SpecT{});
    record.has_version = true;
    record.major = version.major;
    record.minor = version.minor;
    record.patch = version.patch;
  }
  record.reliability = qos.reliability;
  record.durability = qos.durability;
  record.depth = qos.depth;
  return record;
}

}  // namespace autoware::component_interface_utils

#endif  // AUTOWARE__COMPONENT_INTERFACE_UTILS__RCLCPP__REGISTRATION_HPP_
