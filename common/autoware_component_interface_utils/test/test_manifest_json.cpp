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

TEST(manifest_json, unnameable_policy_throws)
{
  ciu::InterfaceRecord record;
  record.interface_name = "/t";
  record.resolved_name = "/t";
  record.reliability = RMW_QOS_POLICY_RELIABILITY_SYSTEM_DEFAULT;  // not in the vocabulary
  record.durability = RMW_QOS_POLICY_DURABILITY_VOLATILE;
  EXPECT_THROW(ciu::to_manifest_json("/n", {record}), std::invalid_argument);
}
