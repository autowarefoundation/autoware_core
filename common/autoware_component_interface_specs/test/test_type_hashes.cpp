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

// The RIHS01 type-hash API (<rosidl_runtime_c/type_hash.h>) is Iron+. On distros without it
// (Humble) generate_type_hashes is a no-op, so this whole suite degrades to a single skip and
// the Humble CI leg stays green.
#if __has_include(<rosidl_runtime_c/type_hash.h>)

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

// GENERATE_HASHES_TOOL is passed by CMake as the path to the built generator.
std::string regenerate()
{
  const char * tmpdir = std::getenv("TEST_TMPDIR");
  const std::string out = std::string(tmpdir ? tmpdir : "/tmp") + "/type_hashes_under_test.lock";
  const std::string cmd = std::string(GENERATE_HASHES_TOOL) + " " + out;
  EXPECT_EQ(std::system(cmd.c_str()), 0);
  return read_file(out);
}

}  // namespace

// A RIHS01 hash is a pure function of the type description, so two runs against the same
// installed messages must be byte-identical. Safe on every job -- it never reads the committed
// file, so dependency skew cannot fail it.
TEST(type_hashes, generator_is_deterministic)
{
  EXPECT_EQ(regenerate(), regenerate());
}

#else  // type-hash API unavailable (e.g. Humble)

TEST(type_hashes, skipped_without_type_hash_api)
{
  GTEST_SKIP() << "RIHS01 type-hash API (<rosidl_runtime_c/type_hash.h>) is unavailable here";
}

#endif
