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

#ifndef AUTOWARE__NDT_SCAN_MATCHER__NDT_BACKEND_LOADER_HPP_
#define AUTOWARE__NDT_SCAN_MATCHER__NDT_BACKEND_LOADER_HPP_

#include "ndt_backend.hpp"

#include <memory>
#include <string>

namespace autoware::ndt_scan_matcher
{

/// Loads NDT backends with pluginlib. The pluginlib headers stay in the source file, because they
/// make every file that includes them compile slower.
///
/// pluginlib unloads a backend library when its loader is destroyed. So a loader must live longer
/// than every backend that it made, including the clones.
class NdtBackendLoader
{
public:
  NdtBackendLoader();
  ~NdtBackendLoader();
  NdtBackendLoader(const NdtBackendLoader &) = delete;
  NdtBackendLoader & operator=(const NdtBackendLoader &) = delete;

  /// Makes the backend with this pluginlib name. Throws std::runtime_error with the names of the
  /// available backends if the backend cannot be loaded.
  NdtBackend::SharedPtr create(const std::string & backend_name);

private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace autoware::ndt_scan_matcher

#endif  // AUTOWARE__NDT_SCAN_MATCHER__NDT_BACKEND_LOADER_HPP_
