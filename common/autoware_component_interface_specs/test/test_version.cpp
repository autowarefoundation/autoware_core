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

#include "autoware/component_interface_specs/version.hpp"
#include "gtest/gtest.h"

namespace cis = autoware::component_interface_specs;

TEST(version, value_and_equality)
{
  constexpr cis::Version v{0, 1, 0};
  EXPECT_EQ(v.major, 0u);
  EXPECT_EQ(v.minor, 1u);
  EXPECT_EQ(v.patch, 0u);
  EXPECT_EQ(v, (cis::Version{0, 1, 0}));
  EXPECT_NE(v, (cis::Version{1, 0, 0}));
}

TEST(version, owner_is_autowarefoundation)
{
  EXPECT_STREQ(cis::owner, "autowarefoundation");
}

TEST(version, is_compatible_checks_major_only)
{
  // 0.x is unstable, but admission is gated on MAJOR per the shared contract.
  static_assert(cis::is_compatible(cis::Version{0, 1, 0}, 0));
  static_assert(cis::is_compatible(cis::Version{0, 7, 3}, 0));
  static_assert(!cis::is_compatible(cis::Version{1, 0, 0}, 0));
  EXPECT_TRUE(cis::is_compatible(cis::Version{2, 4, 1}, 2));
  EXPECT_FALSE(cis::is_compatible(cis::Version{2, 4, 1}, 3));
}

TEST(version, accept_major_range)
{
  constexpr cis::accept_major range{0, 1};  // migration window: accept MAJOR 0 or 1
  EXPECT_TRUE(cis::is_compatible(cis::Version{0, 9, 0}, range));
  EXPECT_TRUE(cis::is_compatible(cis::Version{1, 0, 0}, range));
  EXPECT_FALSE(cis::is_compatible(cis::Version{2, 0, 0}, range));

  // A window with a non-zero lo exercises the below-lo side of the check.
  constexpr cis::accept_major shifted{1, 2};                         // accept MAJOR 1 or 2
  EXPECT_FALSE(cis::is_compatible(cis::Version{0, 9, 0}, shifted));  // below lo
  EXPECT_TRUE(cis::is_compatible(cis::Version{2, 0, 0}, shifted));   // at hi
  EXPECT_FALSE(cis::is_compatible(cis::Version{3, 0, 0}, shifted));  // above hi
}
