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

#ifndef AUTOWARE__COMPONENT_INTERFACE_ADMISSION__MANIFEST_JSON_HPP_
#define AUTOWARE__COMPONENT_INTERFACE_ADMISSION__MANIFEST_JSON_HPP_

#include "autoware/component_interface_admission/records.hpp"

#include <string>

namespace autoware::component_interface_admission
{

// Serialize an InterfaceManifest to the JSON payload carried in the OCI image label
// (org.autoware.interface_manifest) and the fixed-path /opt/autoware/manifest.json. The key names
// are documented in the package README as the label payload schema and are the single source of
// truth shared with from_json().
std::string to_json(const InterfaceManifest & manifest);

// Parse one InterfaceManifest from its JSON payload. DEFENSIVE: any malformed input (JSON syntax
// error, wrong root type, a missing required key, or a value of the wrong type) is reported by
// throwing std::runtime_error. It never triggers undefined behaviour or crashes on bad input.
//
// Required keys per entry: interface_name, and the numeric version / range fields (major / minor /
// patch for a provided entry; accept_major_min / accept_major_max / min_minor for a required one).
// Optional keys default: ns / type_name / owner / node_name to "", resolved_name to interface_name
// (equal to it when not remapped), and the provided / required arrays to empty when absent.
InterfaceManifest from_json(const std::string & doc);

}  // namespace autoware::component_interface_admission

#endif  // AUTOWARE__COMPONENT_INTERFACE_ADMISSION__MANIFEST_JSON_HPP_
