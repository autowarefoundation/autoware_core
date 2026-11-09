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

// Loads the backends with pluginlib, as the node does. This test must not link the backend
// library: pluginlib loads it.

#include <autoware/ndt_scan_matcher/ndt_backend_loader.hpp>

#include <gtest/gtest.h>

#include <stdexcept>
#include <string>

namespace autoware::ndt_scan_matcher
{

TEST(NdtBackendLoader, LoadsTheOmpBackend)
{
  NdtBackendLoader loader;
  const NdtBackend::SharedPtr backend = loader.create("omp");
  ASSERT_NE(backend, nullptr);
  EXPECT_EQ(backend->name(), "omp");

  // A clone comes from the same library, so it also needs the loader.
  const NdtBackend::SharedPtr clone = backend->clone();
  EXPECT_EQ(clone->name(), "omp");
}

TEST(NdtBackendLoader, UnknownBackendThrowsAndListsTheAvailableOnes)
{
  NdtBackendLoader loader;
  try {
    loader.create("no_such_backend");
    FAIL() << "create() did not throw";
  } catch (const std::runtime_error & e) {
    const std::string message = e.what();
    EXPECT_NE(message.find("no_such_backend"), std::string::npos) << message;
    EXPECT_NE(message.find("omp"), std::string::npos) << message;
  }
}

}  // namespace autoware::ndt_scan_matcher
