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

#ifndef AUTOWARE__COMPONENT_INTERFACE_ADMISSION__RECORDS_HPP_
#define AUTOWARE__COMPONENT_INTERFACE_ADMISSION__RECORDS_HPP_

#include <cstdint>
#include <string>
#include <vector>

namespace autoware::component_interface_admission
{

// Plain C++ records that mirror the handshake message set field-for-field. Whether these become
// rosidl messages, and the runtime broadcast that would use them, is not decided or implemented by
// this package, so these structs stand in for the future rosidl messages; the binding to real
// messages, once decided, is a mechanical field-by-field mapping.

// One interface a component provides (it is a publisher / service server / action server for it).
struct ProvidedInterface
{
  std::string ns;
  // Spec-declared interface name (Spec::name = the topic / service / action name). This is the
  // remap-invariant contract identity and the admission matching key.
  std::string interface_name;
  // Remap-resolved fully-qualified name (get_topic_name() / get_service_name()), captured at
  // create-time. Equal to interface_name when the interface is not remapped.
  std::string resolved_name;
  std::string type_name;
  std::uint16_t major{0};
  std::uint16_t minor{0};
  std::uint16_t patch{0};
};

// One interface a component requires (it is a subscription / service client / action client of it).
struct RequiredInterface
{
  std::string ns;
  // Spec-declared interface name (Spec::name); the admission matching key. See ProvidedInterface.
  std::string interface_name;
  // Remap-resolved fully-qualified name; equal to interface_name when not remapped.
  std::string resolved_name;
  std::string type_name;
  // The consumer declares an acceptance range, not a single version: it admits a provider whose
  // MAJOR lies in [accept_major_min, accept_major_max]. min_minor (0 = unconstrained) is an
  // optional, inclusive lower bound on the provider's MINOR. Per semver, MINOR resets to 0 on every
  // MAJOR bump, so min_minor binds ONLY at the MAJOR it was declared against (accept_major_min); at
  // any higher accepted MAJOR the bound is already satisfied.
  std::uint16_t accept_major_min{0};
  std::uint16_t accept_major_max{0};
  std::uint16_t min_minor{0};
};

// A component's interface manifest: the interfaces it provides and requires.
struct InterfaceManifest
{
  std::string owner;      // GitHub org that owns the spec set (e.g. "autowarefoundation").
  std::string node_name;  // The declaring component / node.
  std::vector<ProvidedInterface> provided;
  std::vector<RequiredInterface> required;
};

}  // namespace autoware::component_interface_admission

#endif  // AUTOWARE__COMPONENT_INTERFACE_ADMISSION__RECORDS_HPP_
