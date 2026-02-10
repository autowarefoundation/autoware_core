// Copyright 2026 Autoware Foundation
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

#include <autoware/ndt_scan_matcher/map_update_module.hpp>

#include <gtest/gtest.h>

#include <memory>
#include <string>
#include <vector>

namespace
{
struct FakeNdt
{
  using CloudPtr = std::shared_ptr<int>;

  void addTarget(const CloudPtr &, const std::string & id) { added_ids.push_back(id); }
  void removeTarget(const std::string & id) { removed_ids.push_back(id); }
  void createVoxelKdtree() { kdtree_built = true; }

  std::vector<std::string> added_ids;
  std::vector<std::string> removed_ids;
  bool kdtree_built{false};
};
}  // namespace

using Diff = autoware::ndt_scan_matcher::MapUpdateDiffTemplate<FakeNdt::CloudPtr>;

TEST(MapUpdateModuleTest, AppliesAddsAndRemovals)
{
  FakeNdt ndt;
  Diff diff;
  diff.additions.push_back({std::make_shared<int>(1), "cell_a"});
  diff.removals.push_back("cell_b");

  const auto result = autoware::ndt_scan_matcher::MapUpdateModule::apply_map_update(ndt, diff);

  EXPECT_TRUE(result.updated);
  EXPECT_EQ(1U, result.added);
  EXPECT_EQ(1U, result.removed);
  EXPECT_TRUE(ndt.kdtree_built);
  ASSERT_EQ(1U, ndt.added_ids.size());
  EXPECT_EQ("cell_a", ndt.added_ids.front());
  ASSERT_EQ(1U, ndt.removed_ids.size());
  EXPECT_EQ("cell_b", ndt.removed_ids.front());
}

TEST(MapUpdateModuleTest, NoChangesSkipsKdtree)
{
  FakeNdt ndt;
  Diff diff;

  const auto result = autoware::ndt_scan_matcher::MapUpdateModule::apply_map_update(ndt, diff);

  EXPECT_FALSE(result.updated);
  EXPECT_EQ(0U, result.added);
  EXPECT_EQ(0U, result.removed);
  EXPECT_FALSE(ndt.kdtree_built);
}

TEST(MapUpdateModuleTest, AddsOnlyBuildsKdtree)
{
  FakeNdt ndt;
  Diff diff;
  diff.additions.push_back({std::make_shared<int>(42), "cell_x"});

  const auto result = autoware::ndt_scan_matcher::MapUpdateModule::apply_map_update(ndt, diff);

  EXPECT_TRUE(result.updated);
  EXPECT_EQ(1U, result.added);
  EXPECT_EQ(0U, result.removed);
  EXPECT_TRUE(ndt.kdtree_built);
}
