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

namespace
{

std::string read_file(const std::string & path)
{
  std::ifstream is(path);
  EXPECT_TRUE(is.good()) << "cannot read " << path;
  std::stringstream ss;
  ss << is.rdbuf();
  return ss.str();
}

// GENERATE_TOOL is passed by CMake as the path to the built generator.
std::string regenerate()
{
  const char * tmpdir = std::getenv("TEST_TMPDIR");
  const std::string out = std::string(tmpdir ? tmpdir : "/tmp") + "/manifest_under_test.json";
  const std::string cmd = std::string(GENERATE_TOOL) + " " + out;
  EXPECT_EQ(std::system(cmd.c_str()), 0);
  return read_file(out);
}

}  // namespace

TEST(manifest, generates_known_boundaries)
{
  const std::string json = regenerate();

  EXPECT_NE(json.find("\"owner\": \"autowarefoundation\""), std::string::npos);
  EXPECT_NE(json.find("/localization/kinematic_state"), std::string::npos);
  EXPECT_NE(json.find("nav_msgs/msg/Odometry"), std::string::npos);
  EXPECT_NE(json.find("/planning/set_lanelet_route"), std::string::npos);
  EXPECT_NE(json.find("\"version\": \"0.1.0\""), std::string::npos);
}

// QoS is half of a topic's contract: a RELIABLE subscription never hears a BEST_EFFORT
// publisher, and a TRANSIENT_LOCAL one never gets the latched message a VOLATILE publisher
// dropped. A deploy-time gate reading this manifest cannot check that unless the manifest
// records it -- for services, which all share one profile, as well as for topics.
TEST(manifest, records_the_qos_of_topics_and_services)
{
  const std::string json = regenerate();

  EXPECT_NE(json.find(R"("history": "keep_last")"), std::string::npos);
  EXPECT_NE(json.find(R"("reliability": "reliable")"), std::string::npos);
  EXPECT_NE(json.find(R"("durability": "volatile")"), std::string::npos);
  EXPECT_NE(json.find(R"("durability": "transient_local")"), std::string::npos);
  EXPECT_NE(json.find(R"("depth": 1)"), std::string::npos);
  EXPECT_NE(json.find(R"("depth": 10)"), std::string::npos);  // the shared service profile
}

// The committed manifest is what consumers and the deploy-time admission gate read; the
// generator is only how it is produced. Anything that changes a spec, a QoS policy or a
// domain version has to land the regenerated file in the same commit.
TEST(manifest, committed_file_is_up_to_date)
{
  EXPECT_EQ(regenerate(), read_file(COMMITTED_MANIFEST))
    << "interface_manifest.json is stale -- regenerate it, see README.md";
}
