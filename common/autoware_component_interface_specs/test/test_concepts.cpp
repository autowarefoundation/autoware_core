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

#include "autoware/component_interface_specs/concepts.hpp"
#include "autoware/component_interface_specs/localization.hpp"
#include "autoware/component_interface_specs/planning.hpp"
#include "gtest/gtest.h"

// concepts.hpp is an empty header below C++20. Without this, dropping the target's
// `target_compile_features(... cxx_std_20)` would not fail here as a missing standard but as a
// wall of "no member named InterfaceSpec" further down.
#if !AUTOWARE_COMPONENT_INTERFACE_SPECS_HAS_CONCEPTS
#error "this target must be compiled as C++20; see target_compile_features() in CMakeLists.txt"
#endif

namespace cis = autoware::component_interface_specs;

// A topic spec satisfies InterfaceSpec, a service spec satisfies ServiceSpec.
static_assert(cis::InterfaceSpec<cis::localization::KinematicState>);
static_assert(cis::ServiceSpec<cis::localization::Initialize>);
static_assert(!cis::InterfaceSpec<cis::localization::Initialize>);    // no Message/QoS
static_assert(!cis::ServiceSpec<cis::localization::KinematicState>);  // no Service

// Every member of each domain's Specs tuple is a well-formed spec.
static_assert(cis::all_specs_valid<cis::localization::Specs>());
static_assert(cis::all_specs_valid<cis::planning::Specs>());

TEST(concepts, registered_tuples_are_valid)
{
  EXPECT_TRUE(cis::all_specs_valid<cis::localization::Specs>());
  EXPECT_TRUE(cis::all_specs_valid<cis::planning::Specs>());
}
