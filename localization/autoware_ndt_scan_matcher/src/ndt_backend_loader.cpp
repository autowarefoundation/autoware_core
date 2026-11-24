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

#include "autoware/ndt_scan_matcher/ndt_backend_loader.hpp"

#include <pluginlib/class_loader.hpp>

#include <mutex>
#include <stdexcept>
#include <string>

namespace autoware::ndt_scan_matcher
{
namespace
{

// pluginlib unloads the libraries of a class loader when the loader is destroyed. This loader is
// never destroyed, so a backend library stays loaded until the process ends.
pluginlib::ClassLoader<NdtBackend> & class_loader()
{
  static auto & loader = *new pluginlib::ClassLoader<NdtBackend>(  // NOLINT
    "autoware_ndt_scan_matcher_common", "autoware::ndt_scan_matcher::NdtBackend");
  return loader;
}

// pluginlib::ClassLoader is not thread safe, and several nodes can load backends at the same time
// in one process.
std::mutex & class_loader_mutex()
{
  static std::mutex mutex;
  return mutex;
}

}  // namespace

NdtBackend::SharedPtr load_ndt_backend(const std::string & backend_name)
{
  const std::lock_guard<std::mutex> lock(class_loader_mutex());
  try {
    return class_loader().createSharedInstance(backend_name);
  } catch (const pluginlib::CreateClassException & e) {
    // The library is loaded, but it has no factory for this class and this version of the
    // interface. See the inline namespace in ndt_types.hpp.
    throw std::runtime_error(
      "Failed to create the NDT backend '" + backend_name + "'. " + std::string(e.what()) +
      ". The backend was possibly built against another version of autoware_ndt_scan_matcher_common"
      ", or with SIMD options that change the alignment of Eigen (for example AVX), or the class "
      "type in its plugin description differs from the class that it exports.");
  } catch (const pluginlib::PluginlibException & e) {
    // The message of pluginlib lists the declared backends.
    throw std::runtime_error(
      "Failed to load the NDT backend '" + backend_name + "'. " + std::string(e.what()));
  } catch (const std::exception & e) {
    // From the constructor of the backend.
    throw std::runtime_error(
      "Failed to create the NDT backend '" + backend_name + "': " + std::string(e.what()));
  }
}

}  // namespace autoware::ndt_scan_matcher
