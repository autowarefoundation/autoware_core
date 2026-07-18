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
#include <set>
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

// Every quoted value following a "type": key in interface_manifest.json.
std::set<std::string> types_in_manifest(const std::string & json)
{
  std::set<std::string> out;
  const std::string key = "\"type\": \"";
  for (std::size_t pos = json.find(key); pos != std::string::npos; pos = json.find(key, pos)) {
    pos += key.size();
    const std::size_t end = json.find('"', pos);
    out.insert(json.substr(pos, end - pos));
    pos = end;
  }
  return out;
}

// The second whitespace-separated field of every non-comment lockfile line.
std::set<std::string> types_in_lockfile(const std::string & lock)
{
  std::set<std::string> out;
  std::istringstream is(lock);
  std::string line;
  while (std::getline(is, line)) {
    if (line.empty() || line[0] == '#') {
      continue;
    }
    std::istringstream ls(line);
    std::string hash;
    std::string type;
    ls >> hash >> type;
    out.insert(type);
  }
  return out;
}

}  // namespace

// A RIHS01 hash is a pure function of the type description, so two runs against the same
// installed messages must be byte-identical. Safe on every job -- it never reads the committed
// file, so dependency skew cannot fail it.
TEST(type_hashes, generator_is_deterministic)
{
  EXPECT_EQ(regenerate(), regenerate());
}

// The lockfile must cover exactly the types the manifest registers. Type *names* never change
// on a rebuild, so this is safe to assert on every job (it cannot fail on dependency skew) and
// it is the tripwire for a domain added to one generator's list but not the other's.
TEST(type_hashes, covers_every_manifest_type)
{
  const std::set<std::string> manifest_types = types_in_manifest(read_file(COMMITTED_MANIFEST));
  const std::set<std::string> lock_types = types_in_lockfile(regenerate());
  EXPECT_EQ(manifest_types, lock_types)
    << "interface_type_hashes.jazzy.lock and interface_manifest.json cover different type sets -- "
       "a domain was likely added to one generator but not the other";
}

// The committed lockfile must match the hashes the generator produces against the installed
// messages. OPT-IN via AUTOWARE_CIS_CHECK_TYPE_HASHES: only this repo's own GitHub Actions sets
// it. Buildfarm devel jobs build BUILD_TESTING but never set it, so they cannot redden when an
// installed dependency's definition differs from the lockfile-generation environment -- a
// failure no PR to this repo could fix. See README.md.
TEST(type_hashes, committed_lockfile_is_up_to_date)
{
  if (std::getenv("AUTOWARE_CIS_CHECK_TYPE_HASHES") == nullptr) {
    GTEST_SKIP() << "opt-in gate; set AUTOWARE_CIS_CHECK_TYPE_HASHES to enable";
  }
  EXPECT_EQ(regenerate(), read_file(COMMITTED_HASHES))
    << "interface_type_hashes.jazzy.lock is stale -- regenerate it, see README.md";
}

#else  // type-hash API unavailable (e.g. Humble)

TEST(type_hashes, skipped_without_type_hash_api)
{
  GTEST_SKIP() << "RIHS01 type-hash API (<rosidl_runtime_c/type_hash.h>) is unavailable here";
}

#endif
