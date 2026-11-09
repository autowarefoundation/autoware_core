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

#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>

namespace autoware::ndt_scan_matcher
{

struct NdtBackendLoader::Impl
{
  pluginlib::ClassLoader<NdtBackend> loader{
    "autoware_ndt_scan_matcher", "autoware::ndt_scan_matcher::NdtBackend"};
};

NdtBackendLoader::NdtBackendLoader() : impl_(std::make_unique<Impl>())
{
}

NdtBackendLoader::~NdtBackendLoader() = default;

NdtBackend::SharedPtr NdtBackendLoader::create(const std::string & backend_name)
{
  try {
    return impl_->loader.createSharedInstance(backend_name);
  } catch (const pluginlib::PluginlibException & e) {
    std::stringstream message;
    message << "Failed to load the NDT backend '" << backend_name << "': " << e.what()
            << " Available backends:";
    for (const auto & name : impl_->loader.getDeclaredClasses()) {
      message << " " << name;
    }
    throw std::runtime_error(message.str());
  }
}

}  // namespace autoware::ndt_scan_matcher
