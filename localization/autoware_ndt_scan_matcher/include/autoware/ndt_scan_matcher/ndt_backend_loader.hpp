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

#include "autoware/ndt_scan_matcher_common/ndt_backend.hpp"

#include <string>

namespace autoware::ndt_scan_matcher
{

/// Makes the NDT backend with this pluginlib lookup name. Throws std::runtime_error if the backend
/// cannot be made. For an unknown name, the message lists the declared backends.
///
/// A backend library stays loaded until the process ends, so instances that a backend makes
/// without pluginlib (for example, in clone()) stay valid.
NdtBackend::SharedPtr load_ndt_backend(const std::string & backend_name);

}  // namespace autoware::ndt_scan_matcher

#endif  // AUTOWARE__NDT_SCAN_MATCHER__NDT_BACKEND_LOADER_HPP_
