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

#ifndef AUTOWARE__COMPONENT_INTERFACE_UTILS__TESTING__MANIFEST_DRIFT_HPP_
#define AUTOWARE__COMPONENT_INTERFACE_UTILS__TESTING__MANIFEST_DRIFT_HPP_

// This header calls gtest assertion macros (ASSERT_TRUE/EXPECT_EQ), so any
// target that includes it must link gtest itself; this package adds no new
// dependency on its account, the same way its own tests already bring gtest
// in via ament_add_gtest()/ament_add_ros_isolated_gtest().
#include "autoware/component_interface_utils/manifest_json.hpp"
#include "autoware/component_interface_utils/rclcpp.hpp"

#include <nlohmann/json.hpp>

#include <gtest/gtest.h>

#include <fstream>
#include <sstream>
#include <string>

namespace autoware::component_interface_utils::testing
{

/// Pin one node's registered manifest against a fragment file committed to a
/// package's source tree, comparing as parsed JSON values so formatting and
/// key order in the fragment do not matter. On mismatch, the actual document
/// is dumped so the diff is visible in the test log.
///
/// The caller must keep both `adaptor` and the node it wraps alive across
/// this call. Every in-tree node builds its NodeAdaptor as a local inside the
/// node's own constructor, so `manifest()` is unreachable once construction
/// returns; this helper only works because a unit test constructs the node
/// and the adaptor itself and holds them until the assertion runs, not
/// because it can reach into an already-running node.
template <class NodeT>
void expect_manifest_matches(
  const NodeAdaptor<NodeT> & adaptor, const std::string & node_name,
  const std::string & fragment_path)
{
  std::ifstream is(fragment_path);
  ASSERT_TRUE(is.good()) << "cannot read fragment " << fragment_path;
  std::stringstream ss;
  ss << is.rdbuf();
  const auto expected = nlohmann::json::parse(ss.str());
  const auto actual = to_manifest_json(adaptor, node_name);
  EXPECT_EQ(expected, actual) << "manifest does not match fragment " << fragment_path << ":\n"
                              << actual.dump(2);
}

}  // namespace autoware::component_interface_utils::testing

#endif  // AUTOWARE__COMPONENT_INTERFACE_UTILS__TESTING__MANIFEST_DRIFT_HPP_
