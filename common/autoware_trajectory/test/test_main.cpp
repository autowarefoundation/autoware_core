// Copyright 2026 TIER IV, Inc.
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

#include <gtest/gtest.h>
#include <pybind11/embed.h>

#include <cstdlib>
#include <string>

#ifdef PLOT
#include "test_plot.hpp"

#include <optional>
#endif
namespace autoware::test_utils
{

bool plot_enabled()
{
  const char * env = std::getenv("ENABLE_TEST_PLOT");
  return env && std::string(env) == "1";
}

}  // namespace autoware::test_utils

int main(int argc, char ** argv)
{
#ifdef PLOT
  std::optional<pybind11::scoped_interpreter> guard;
  if (autoware::test_utils::plot_enabled()) {
    guard.emplace();
  }
#endif
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
