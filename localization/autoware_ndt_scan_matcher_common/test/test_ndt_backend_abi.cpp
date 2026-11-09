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

// The inline namespace of the interface must keep a backend with another Eigen alignment or
// another heap allocator of Eigen from loading. Without the inline namespace, the test of each
// library with another configuration fails.

#include <autoware/ndt_scan_matcher_common/ndt_backend.hpp>
#include <class_loader/class_loader.hpp>

#include <dlfcn.h>
#include <gtest/gtest.h>

#include <string>
#include <type_traits>

namespace
{

using autoware::ndt_scan_matcher::NdtBackend;

// A copy assignment cannot slice a backend.
static_assert(!std::is_copy_assignable_v<NdtBackend>);

// The Eigen configuration of this test, as fake_ndt_backend.cpp computes it for a library.
constexpr int static_alignment = EIGEN_MAX_STATIC_ALIGN_BYTES;
constexpr int std_malloc =
  ((EIGEN_DEFAULT_ALIGN_BYTES == 0) || EIGEN_MALLOC_ALREADY_ALIGNED) ? 1 : 0;

std::string library_path(const std::string & variant)
{
  return std::string(FAKE_BACKEND_DIRECTORY) + "/libfake_ndt_backend_" + variant + ".so";
}

// Calls a function of a library that is already loaded.
int call(const std::string & library, const char * function)
{
  void * handle = dlopen(library.c_str(), RTLD_NOW | RTLD_NOLOAD);
  EXPECT_NE(handle, nullptr) << library;
  if (handle == nullptr) {
    return -1;
  }
  const auto pointer = reinterpret_cast<int (*)()>(dlsym(handle, function));  // NOLINT
  EXPECT_NE(pointer, nullptr) << function;
  const int value = (pointer != nullptr) ? pointer() : -1;
  dlclose(handle);
  return value;
}

// The number of libraries that the tests expected not to load.
int refused_count = 0;  // NOLINT

class FakeBackendLibrary : public testing::TestWithParam<const char *>
{
};

// A library loads only if its Eigen configuration is the configuration of this test.
TEST_P(FakeBackendLibrary, LoadsOnlyWithTheSameConfiguration)
{
  const std::string library = library_path(GetParam());
  class_loader::ClassLoader loader(library);
  const bool same_configuration =
    call(library, "fake_ndt_backend_static_alignment") == static_alignment &&
    call(library, "fake_ndt_backend_std_malloc") == std_malloc;

  if (same_configuration) {
    const auto backend = loader.createUniqueInstance<NdtBackend>("fake::FakeBackend");
    ASSERT_NE(backend, nullptr);
    EXPECT_EQ(backend->name(), "fake");
  } else {
    ++refused_count;
    EXPECT_THROW(
      loader.createUniqueInstance<NdtBackend>("fake::FakeBackend"),
      class_loader::CreateClassException);
  }
}

// "same" has no definition of its own. The other libraries set a static alignment of 0, 16, 32
// (AVX) or 64 bytes (AVX-512), or the allocator: the one of Eigen or std::malloc(). Which of them
// have the configuration of the test depends on the compiler options of the build.
INSTANTIATE_TEST_SUITE_P(
  NdtBackendAbi, FakeBackendLibrary,
  testing::Values(
    "same", "align_0", "align_16", "align_32", "align_64", "allocator_eigen", "allocator_std"));

// Runs after all tests. At least the alignments that differ from the test were refused, so the
// definitions of the libraries had an effect.
class RefusedCount : public testing::Environment
{
public:
  void TearDown() override { EXPECT_GE(refused_count, 3); }
};

const testing::Environment * const refused_count_environment =  // NOLINT
  testing::AddGlobalTestEnvironment(new RefusedCount);          // NOLINT

}  // namespace
