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

#include "autoware/component_interface_utils/manifest_json.hpp"
#include "autoware/component_interface_utils/rclcpp/registration.hpp"
#include "gtest/gtest.h"

#include <nlohmann/json.hpp>

#include <vector>

namespace ciu = autoware::component_interface_utils;

TEST(manifest_json, renders_the_admission_document_schema)
{
  ciu::InterfaceRecord provided;
  provided.kind = ciu::InterfaceRecord::Kind::Topic;
  provided.role = ciu::InterfaceRecord::Role::Provide;
  provided.interface_name = "/t";
  provided.resolved_name = "/t";
  provided.type_name = "pkg/msg/T";
  provided.has_version = true;
  provided.major = 0;
  provided.minor = 1;
  provided.patch = 0;
  provided.reliability = RMW_QOS_POLICY_RELIABILITY_RELIABLE;
  provided.durability = RMW_QOS_POLICY_DURABILITY_VOLATILE;
  provided.depth = 1;

  ciu::InterfaceRecord required;  // unversioned service client
  required.kind = ciu::InterfaceRecord::Kind::Service;
  required.role = ciu::InterfaceRecord::Role::Require;
  required.interface_name = "/s";
  required.resolved_name = "/s";
  required.type_name = "pkg/srv/S";
  required.reliability = RMW_QOS_POLICY_RELIABILITY_RELIABLE;
  required.durability = RMW_QOS_POLICY_DURABILITY_VOLATILE;
  required.depth = 10;

  const auto actual = ciu::to_manifest_json("/test/node", {provided, required});
  const auto expected = nlohmann::json::parse(R"({
    "owner": "",
    "node_name": "/test/node",
    "provided": [
      {"ns": "", "interface_name": "/t", "resolved_name": "/t", "type_name": "pkg/msg/T",
       "major": 0, "minor": 1, "patch": 0,
       "qos": {"reliability": "reliable", "durability": "volatile", "depth": 1}}
    ],
    "required": [
      {"ns": "", "interface_name": "/s", "resolved_name": "/s", "type_name": "pkg/srv/S",
       "qos": {"reliability": "reliable", "durability": "volatile", "depth": 10}}
    ]
  })");
  EXPECT_EQ(actual, expected) << actual.dump(2);
}

// A versioned Require record must emit the accept range (accept_major_min ==
// accept_major_max == major, min_minor == 0), never the Provide-shaped
// major/minor/patch triple. major/minor/patch are distinct non-zero values so
// a swap (e.g. accept_major_max fed from minor instead of major) cannot
// coincide with the correct answer.
TEST(manifest_json, versioned_require_emits_the_accept_range_not_a_single_version)
{
  ciu::InterfaceRecord record;
  record.role = ciu::InterfaceRecord::Role::Require;
  record.interface_name = "/s";
  record.resolved_name = "/s";
  record.type_name = "pkg/srv/S";
  record.has_version = true;
  record.major = 2;
  record.minor = 3;
  record.patch = 4;
  record.reliability = RMW_QOS_POLICY_RELIABILITY_RELIABLE;
  record.durability = RMW_QOS_POLICY_DURABILITY_VOLATILE;
  record.depth = 10;

  const auto actual = ciu::to_manifest_json("/n", {record});
  const auto expected = nlohmann::json::parse(R"({
    "owner": "",
    "node_name": "/n",
    "provided": [],
    "required": [
      {"ns": "", "interface_name": "/s", "resolved_name": "/s", "type_name": "pkg/srv/S",
       "accept_major_min": 2, "accept_major_max": 2, "min_minor": 0,
       "qos": {"reliability": "reliable", "durability": "volatile", "depth": 10}}
    ]
  })");
  EXPECT_EQ(actual, expected) << actual.dump(2);
}

// An unversioned Provide record must omit major/minor/patch entirely rather
// than emitting the struct's own {0, 0, 0} defaults. Uses best_effort /
// transient_local (untested by the primary schema test) so this also
// exercises the other half of the QoS vocabulary.
TEST(manifest_json, unversioned_provide_omits_version_keys)
{
  ciu::InterfaceRecord record;
  record.role = ciu::InterfaceRecord::Role::Provide;
  record.interface_name = "/t";
  record.resolved_name = "/t";
  record.type_name = "pkg/msg/T";
  record.reliability = RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT;
  record.durability = RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL;
  record.depth = 5;

  const auto actual = ciu::to_manifest_json("/n", {record});
  const auto expected = nlohmann::json::parse(R"({
    "owner": "",
    "node_name": "/n",
    "provided": [
      {"ns": "", "interface_name": "/t", "resolved_name": "/t", "type_name": "pkg/msg/T",
       "qos": {"reliability": "best_effort", "durability": "transient_local", "depth": 5}}
    ],
    "required": []
  })");
  EXPECT_EQ(actual, expected) << actual.dump(2);
}

TEST(manifest_json, unnameable_policy_throws)
{
  ciu::InterfaceRecord record;
  record.interface_name = "/t";
  record.resolved_name = "/t";
  record.reliability = RMW_QOS_POLICY_RELIABILITY_SYSTEM_DEFAULT;  // not in the vocabulary
  record.durability = RMW_QOS_POLICY_DURABILITY_VOLATILE;
  EXPECT_THROW(ciu::to_manifest_json("/n", {record}), std::invalid_argument);
}

// to_qos_json() checks reliability before durability, so a record with an
// unnameable reliability never reaches the durability check. Drive
// reliability valid and durability invalid so that branch is actually
// exercised.
TEST(manifest_json, unnameable_durability_throws)
{
  ciu::InterfaceRecord record;
  record.interface_name = "/t";
  record.resolved_name = "/t";
  record.reliability = RMW_QOS_POLICY_RELIABILITY_RELIABLE;
  record.durability = RMW_QOS_POLICY_DURABILITY_SYSTEM_DEFAULT;  // not in the vocabulary
  EXPECT_THROW(ciu::to_manifest_json("/n", {record}), std::invalid_argument);
}
