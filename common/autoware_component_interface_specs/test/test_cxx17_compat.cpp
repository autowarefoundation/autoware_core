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

// The domain headers are the surface every consumer compiles against, and every consumer gets
// the C++17 that autoware_package() sets. Only this package's own producer-side targets opt up
// to C++20. This target is pinned to C++17 so that a C++20 construct leaking into a domain
// header, or into version.hpp or utils.hpp, fails here rather than in a downstream package --
// on Humble / gcc-11 as much as on Jazzy.

#include "autoware/component_interface_specs/concepts.hpp"
#include "autoware/component_interface_specs/control.hpp"
#include "autoware/component_interface_specs/localization.hpp"
#include "autoware/component_interface_specs/map.hpp"
#include "autoware/component_interface_specs/perception.hpp"
#include "autoware/component_interface_specs/planning.hpp"
#include "autoware/component_interface_specs/system.hpp"
#include "autoware/component_interface_specs/utils.hpp"
#include "autoware/component_interface_specs/vehicle.hpp"
#include "autoware/component_interface_specs/version.hpp"
#include "gtest/gtest.h"

// Including concepts.hpp from a C++17 translation unit has to stay safe: it must expand to
// nothing rather than leak C++20 syntax into the parse.
#if AUTOWARE_COMPONENT_INTERFACE_SPECS_HAS_CONCEPTS
#error "this target must be compiled as C++17; see set_target_properties() in CMakeLists.txt"
#endif

namespace cis = autoware::component_interface_specs;

// Every domain resolves its version through the ADL hook the domain macro emits, with no
// C++20 machinery involved.
TEST(cxx17_compat, every_domain_resolves_its_version)
{
  constexpr cis::Version v0_1_0{0, 1, 0};

  static_assert(cis::spec_version<cis::control::ControlCommand>() == v0_1_0);
  static_assert(cis::spec_version<cis::localization::Initialize>() == v0_1_0);
  static_assert(cis::spec_version<cis::map::VectorMap>() == v0_1_0);
  static_assert(cis::spec_version<cis::perception::ObjectRecognition>() == v0_1_0);
  static_assert(cis::spec_version<cis::planning::Trajectory>() == v0_1_0);
  static_assert(cis::spec_version<cis::system::OperationModeState>() == v0_1_0);
  static_assert(cis::spec_version<cis::vehicle::GearStatus>() == v0_1_0);

  EXPECT_EQ(cis::spec_version<cis::control::ControlCommand>(), v0_1_0);
  EXPECT_EQ(std::tuple_size_v<cis::planning::Specs>, 6u);
}

TEST(cxx17_compat, qos_helpers_are_available)
{
  const auto topic = cis::get_qos<cis::vehicle::GearStatus>();
  EXPECT_EQ(topic.depth(), cis::vehicle::GearStatus::depth);
  EXPECT_EQ(topic.reliability(), rclcpp::ReliabilityPolicy::Reliable);

  const auto service = cis::get_service_qos();
  EXPECT_EQ(service.depth(), cis::service_qos::depth);
  EXPECT_EQ(service.reliability(), rclcpp::ReliabilityPolicy::Reliable);
}
