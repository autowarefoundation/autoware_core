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

#include <map>
#include <string>
#include <vector>

namespace autoware::component_interface_admission
{

// Serialize an InterfaceManifest to the JSON payload carried in the OCI image label
// (org.autoware.interface_manifest) or in an installed interface_manifest_fragment.json file. The
// key names are documented in the package README as the label payload schema and are the single
// source of truth shared with from_json().
std::string to_json(const InterfaceManifest & manifest);

// Parse one InterfaceManifest from its JSON payload. DEFENSIVE: any malformed input (JSON syntax
// error, wrong root type, a missing required key, or a value of the wrong type) is reported by
// throwing std::runtime_error. It never triggers undefined behaviour or crashes on bad input.
//
// Required keys per entry: interface_name. The version fields (major / minor / patch for a
// provided entry; accept_major_min / accept_major_max / min_minor for a required one) are a group:
// all three present parses as versioned (has_version = true); all three absent parses as
// unversioned (has_version = false, the fields default to 0); any other combination -- a partial
// declaration -- throws. `qos` (reliability / durability / depth) is optional; when present it
// populates has_qos = true and qos, and an out-of-vocabulary reliability/durability string throws.
// Optional keys default: ns / type_name / owner / node_name to "", resolved_name to interface_name
// (equal to it when not remapped), and the provided / required arrays to empty when absent.
InterfaceManifest from_json(const std::string & doc);

// Parse a manifest document set from one JSON payload: an object root yields exactly one manifest
// (equivalent to from_json()); an array root yields one manifest per element; any other root type
// throws std::runtime_error.
std::vector<InterfaceManifest> manifests_from_json(const std::string & doc);

// Parse the spec manifest (autoware_component_interface_specs' interface_manifest.json) into the
// QoS each interface declares. Returns a map from `interfaces[].interface` to the QoS at
// `interfaces[].qos`. DEFENSIVE: throws std::runtime_error unless the root is an object carrying an
// `interfaces` array in which every entry is an object with both `interface` and `qos` -- this
// package must never silently treat an unrelated document as a spec manifest and end up with an
// empty QoS table, which would disable the spec-conformance verdict without saying so.
std::map<std::string, QosRecord> spec_qos_from_json(const std::string & doc);

}  // namespace autoware::component_interface_admission

#endif  // AUTOWARE__COMPONENT_INTERFACE_ADMISSION__MANIFEST_JSON_HPP_
