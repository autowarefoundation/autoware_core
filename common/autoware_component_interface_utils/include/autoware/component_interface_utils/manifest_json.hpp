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

#ifndef AUTOWARE__COMPONENT_INTERFACE_UTILS__MANIFEST_JSON_HPP_
#define AUTOWARE__COMPONENT_INTERFACE_UTILS__MANIFEST_JSON_HPP_

// This header is the only place in the package that depends on nlohmann::json.
// rclcpp.hpp itself stays free of it, so a plain NodeAdaptor consumer never
// pays for the JSON dependency; only a caller that opts in by including this
// header does.
#include "autoware/component_interface_utils/rclcpp.hpp"
#include "autoware/component_interface_utils/rclcpp/registration.hpp"

#include <nlohmann/json.hpp>

#include <stdexcept>
#include <string>
#include <vector>

namespace autoware::component_interface_utils
{

namespace detail
{

/// Map a QoS reliability policy to the admission schema's vocabulary. Throws
/// for any value outside it (notably InterfaceRecord's own SYSTEM_DEFAULT
/// default) rather than emitting a placeholder.
inline std::string to_reliability_string(rmw_qos_reliability_policy_t reliability)
{
  switch (reliability) {
    case RMW_QOS_POLICY_RELIABILITY_RELIABLE:
      return "reliable";
    case RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT:
      return "best_effort";
    default:
      throw std::invalid_argument(
        "InterfaceRecord reliability policy has no admission schema representation");
  }
}

/// Map a QoS durability policy to the admission schema's vocabulary. Throws
/// for any value outside it (notably InterfaceRecord's own SYSTEM_DEFAULT
/// default) rather than emitting a placeholder.
inline std::string to_durability_string(rmw_qos_durability_policy_t durability)
{
  switch (durability) {
    case RMW_QOS_POLICY_DURABILITY_VOLATILE:
      return "volatile";
    case RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL:
      return "transient_local";
    default:
      throw std::invalid_argument(
        "InterfaceRecord durability policy has no admission schema representation");
  }
}

inline nlohmann::json to_qos_json(const InterfaceRecord & record)
{
  return {
    {"reliability", to_reliability_string(record.reliability)},
    {"durability", to_durability_string(record.durability)},
    {"depth", record.depth},
  };
}

/// Build one `provided[]` entry. major/minor/patch appear iff the record is
/// versioned; they are omitted entirely otherwise (never null, never zero).
inline nlohmann::json to_provided_json(const InterfaceRecord & record)
{
  nlohmann::json entry = {
    {"ns", ""},
    {"interface_name", record.interface_name},
    {"resolved_name", record.resolved_name},
    {"type_name", record.type_name},
  };
  if (record.has_version) {
    entry["major"] = record.major;
    entry["minor"] = record.minor;
    entry["patch"] = record.patch;
  }
  entry["qos"] = to_qos_json(record);
  return entry;
}

/// Build one `required[]` entry. A registration-derived consumer accepts
/// exactly the MAJOR it compiled against and does not over-constrain MINOR,
/// so accept_major_min == accept_major_max == major and min_minor == 0; these
/// keys appear iff the record is versioned, and are omitted otherwise.
inline nlohmann::json to_required_json(const InterfaceRecord & record)
{
  nlohmann::json entry = {
    {"ns", ""},
    {"interface_name", record.interface_name},
    {"resolved_name", record.resolved_name},
    {"type_name", record.type_name},
  };
  if (record.has_version) {
    entry["accept_major_min"] = record.major;
    entry["accept_major_max"] = record.major;
    entry["min_minor"] = 0;
  }
  entry["qos"] = to_qos_json(record);
  return entry;
}

}  // namespace detail

/// Serialize one node's registered interfaces into an admission schema v2
/// manifest document. Preserves the relative registration order of `records`
/// within each of `provided[]` and `required[]`.
inline nlohmann::json to_manifest_json(
  const std::string & node_name, const std::vector<InterfaceRecord> & records)
{
  auto provided = nlohmann::json::array();
  auto required = nlohmann::json::array();
  for (const auto & record : records) {
    if (record.role == InterfaceRecord::Role::Provide) {
      provided.push_back(detail::to_provided_json(record));
    } else {
      required.push_back(detail::to_required_json(record));
    }
  }
  return {
    {"owner", ""},
    {"node_name", node_name},
    {"provided", provided},
    {"required", required},
  };
}

/// Convenience overload for a live adaptor. `node_name` is the caller-supplied
/// fully-qualified node name (rclcpp::Node::get_fully_qualified_name()); kept
/// as a free function rather than a NodeAdaptor member so that rclcpp.hpp
/// itself never has to include nlohmann::json.
template <class NodeT>
nlohmann::json to_manifest_json(const NodeAdaptor<NodeT> & adaptor, const std::string & node_name)
{
  return to_manifest_json(node_name, adaptor.manifest());
}

}  // namespace autoware::component_interface_utils

#endif  // AUTOWARE__COMPONENT_INTERFACE_UTILS__MANIFEST_JSON_HPP_
