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

// Loads the backends with pluginlib, as the node does. pluginlib loads the backend "ndt_omp" from
// the library autoware_ndt_scan_matcher_omp_backend, and this test does not link it.

#include <autoware/ndt_scan_matcher/ndt_backend_loader.hpp>

#include <console_bridge/console.h>
#include <dlfcn.h>
#include <gtest/gtest.h>

#include <stdexcept>
#include <string>

namespace autoware::ndt_scan_matcher
{

TEST(LoadNdtBackend, LoadsTheOmpBackend)
{
  const NdtBackend::SharedPtr backend = load_ndt_backend("ndt_omp");
  ASSERT_NE(backend, nullptr);
  EXPECT_EQ(backend->name(), "ndt_omp");

  // The backend makes its clones without pluginlib.
  const NdtBackend::SharedPtr clone = backend->clone();
  ASSERT_NE(clone, nullptr);
  EXPECT_EQ(clone->name(), "ndt_omp");
}

namespace
{
// Keeps the log messages of class_loader while it exists.
class LogSpy : public console_bridge::OutputHandler
{
public:
  LogSpy() { console_bridge::useOutputHandler(this); }
  ~LogSpy() override { console_bridge::restorePreviousOutputHandler(); }
  LogSpy(const LogSpy &) = delete;
  LogSpy & operator=(const LogSpy &) = delete;

  void log(const std::string & text, console_bridge::LogLevel, const char *, int) override
  {
    text_ += text + "\n";
  }
  const std::string & text() const { return text_; }

private:
  std::string text_;
};
}  // namespace

TEST(LoadNdtBackend, TheBackendLibraryStaysLoaded)
{
  // The clones of a backend come from the backend library, not from pluginlib. So the library must
  // stay loaded after the instance from pluginlib is gone.
  //
  // A loader that is destroyed while the instance from pluginlib exists cannot unload the library,
  // and class_loader then gives a warning. The test expects no such warning. The test cannot find
  // a loader that is destroyed while only a clone exists: in this process, the library stays
  // loaded in that case too.
  NdtBackend::SharedPtr clone;
  {
    const LogSpy spy;
    clone = load_ndt_backend("ndt_omp")->clone();
    EXPECT_EQ(spy.text().find("SEVERE WARNING"), std::string::npos) << spy.text();
  }

  void * handle = dlopen("libautoware_ndt_scan_matcher_omp_backend.so", RTLD_NOW | RTLD_NOLOAD);
  ASSERT_NE(handle, nullptr) << "the backend library was unloaded";
  dlclose(handle);
  EXPECT_FALSE(clone->clone_empty()->has_target());
}

TEST(LoadNdtBackend, UnknownBackendThrowsAndListsTheAvailableOnes)
{
  try {
    load_ndt_backend("no_such_backend");
    FAIL() << "load_ndt_backend() did not throw";
  } catch (const std::runtime_error & e) {
    const std::string message = e.what();
    EXPECT_NE(message.find("no_such_backend"), std::string::npos) << message;
    EXPECT_NE(message.find("ndt_omp"), std::string::npos) << message;
  }
}

// An exception from the constructor of a backend gets the name of the backend.
TEST(LoadNdtBackend, BackendWhoseConstructorThrowsGetsItsNameInTheMessage)
{
  try {
    load_ndt_backend("test_constructor_throws");
    FAIL() << "load_ndt_backend() did not throw";
  } catch (const std::runtime_error & e) {
    const std::string message = e.what();
    EXPECT_NE(
      message.find("Failed to create the NDT backend 'test_constructor_throws'"), std::string::npos)
      << message;
    EXPECT_NE(message.find("the test backend cannot be made"), std::string::npos) << message;
  }
}

// The test backend "test_other_abi" was built with EIGEN_MALLOC_ALREADY_ALIGNED=0. In a normal
// build, it then has another heap allocator of Eigen than this test, so its library has no factory
// for the interface of this test.
TEST(LoadNdtBackend, BackendWithAnotherAbiThrowsWithTheCause)
{
#if (EIGEN_DEFAULT_ALIGN_BYTES == 0) || !EIGEN_MALLOC_ALREADY_ALIGNED
  GTEST_SKIP() << "This build already has the Eigen allocator of the test backend.";
#endif
  try {
    load_ndt_backend("test_other_abi");
    FAIL() << "load_ndt_backend() did not throw";
  } catch (const std::runtime_error & e) {
    const std::string message = e.what();
    EXPECT_NE(message.find("Failed to create the NDT backend 'test_other_abi'"), std::string::npos)
      << message;
    EXPECT_NE(
      message.find("another version of autoware_ndt_scan_matcher_common"), std::string::npos)
      << message;
  }
}

}  // namespace autoware::ndt_scan_matcher
