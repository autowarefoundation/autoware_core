// Copyright 2023 The Autoware Contributors
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

#include "autoware/component_interface_specs/concepts.hpp"
#include "autoware/component_interface_specs/perception.hpp"
#include "gtest/gtest.h"
#include "spec_test_utils.hpp"

#include <tuple>

namespace specs = autoware::component_interface_specs;
namespace tu = autoware::component_interface_specs::test_utils;

TEST(perception, concept_and_registration)
{
  using specs::perception::ObjectRecognition;
  using specs::perception::Specs;
  using specs::perception::TrackedObjects;
  using specs::perception::TrafficSignals;

  static_assert(specs::InterfaceSpec<ObjectRecognition>);
  static_assert(specs::InterfaceSpec<TrafficSignals>);
  static_assert(specs::InterfaceSpec<TrackedObjects>);

  static_assert(tu::has_type<ObjectRecognition, Specs>::value);
  static_assert(tu::has_type<TrafficSignals, Specs>::value);
  static_assert(tu::has_type<TrackedObjects, Specs>::value);
  static_assert(std::tuple_size_v<Specs> == 3);
  SUCCEED();
}

TEST(perception, interface)
{
  using specs::perception::ObjectRecognition;
  tu::expect_topic_qos<ObjectRecognition>(
    "/perception/object_recognition/objects", 1, RMW_QOS_POLICY_RELIABILITY_RELIABLE,
    RMW_QOS_POLICY_DURABILITY_VOLATILE);
}
