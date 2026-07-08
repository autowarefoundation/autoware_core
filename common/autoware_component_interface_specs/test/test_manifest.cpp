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

#include "gtest/gtest.h"

#include <cstdlib>
#include <fstream>
#include <sstream>
#include <string>

// GENERATE_TOOL is passed by CMake as the path to the built generator.
TEST(manifest, generates_known_boundaries)
{
  const char * tmpdir = std::getenv("TEST_TMPDIR");
  const std::string out = std::string(tmpdir ? tmpdir : "/tmp") + "/manifest_under_test.json";
  const std::string cmd = std::string(GENERATE_TOOL) + " " + out;
  ASSERT_EQ(std::system(cmd.c_str()), 0);

  std::ifstream is(out);
  std::stringstream ss;
  ss << is.rdbuf();
  const std::string json = ss.str();

  EXPECT_NE(json.find("\"owner\": \"autowarefoundation\""), std::string::npos);
  EXPECT_NE(json.find("/localization/kinematic_state"), std::string::npos);
  EXPECT_NE(json.find("nav_msgs/msg/Odometry"), std::string::npos);
  EXPECT_NE(json.find("/planning/set_lanelet_route"), std::string::npos);
  EXPECT_NE(json.find("\"version\": \"0.1.0\""), std::string::npos);
}
