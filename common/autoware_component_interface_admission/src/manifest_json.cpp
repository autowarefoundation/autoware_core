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

#include "autoware/component_interface_admission/manifest_json.hpp"

#include "autoware/component_interface_admission/records.hpp"

#include <nlohmann/json.hpp>

#include <cstdint>
#include <stdexcept>
#include <string>
#include <utility>

namespace autoware::component_interface_admission
{

namespace
{

// Defensive accessors: each reports a bad payload by throwing std::runtime_error (never a bare
// nlohmann exception, and never undefined behaviour), so callers can rely on a single exception
// type. Keys confirmed present and correctly typed here, so the .get<> calls below never throw.

std::string require_string(const nlohmann::json & j, const char * key)
{
  if (!j.is_object() || !j.contains(key)) {
    throw std::runtime_error(std::string("interface manifest: missing required key '") + key + "'");
  }
  const auto & value = j.at(key);
  if (!value.is_string()) {
    throw std::runtime_error(std::string("interface manifest: key '") + key + "' is not a string");
  }
  return value.get<std::string>();
}

std::string optional_string(
  const nlohmann::json & j, const char * key, const std::string & fallback)
{
  if (!j.is_object() || !j.contains(key)) {
    return fallback;
  }
  const auto & value = j.at(key);
  if (!value.is_string()) {
    throw std::runtime_error(std::string("interface manifest: key '") + key + "' is not a string");
  }
  return value.get<std::string>();
}

std::uint16_t require_uint16(const nlohmann::json & j, const char * key)
{
  if (!j.is_object() || !j.contains(key)) {
    throw std::runtime_error(std::string("interface manifest: missing required key '") + key + "'");
  }
  const auto & value = j.at(key);
  if (value.is_number_unsigned()) {
    const auto n = value.get<std::uint64_t>();
    if (n > 65535U) {
      throw std::runtime_error(std::string("interface manifest: key '") + key + "' exceeds uint16");
    }
    return static_cast<std::uint16_t>(n);
  }
  if (value.is_number_integer()) {
    const auto n = value.get<std::int64_t>();
    if (n < 0 || n > 65535) {
      throw std::runtime_error(
        std::string("interface manifest: key '") + key + "' out of uint16 range");
    }
    return static_cast<std::uint16_t>(n);
  }
  throw std::runtime_error(std::string("interface manifest: key '") + key + "' is not an integer");
}

}  // namespace

std::string to_json(const InterfaceManifest & manifest)
{
  nlohmann::json j;
  j["owner"] = manifest.owner;
  j["node_name"] = manifest.node_name;
  j["provided"] = nlohmann::json::array();
  for (const auto & p : manifest.provided) {
    nlohmann::json pj;
    pj["ns"] = p.ns;
    pj["interface_name"] = p.interface_name;
    pj["resolved_name"] = p.resolved_name;
    pj["type_name"] = p.type_name;
    pj["major"] = p.major;
    pj["minor"] = p.minor;
    pj["patch"] = p.patch;
    j["provided"].push_back(std::move(pj));
  }
  j["required"] = nlohmann::json::array();
  for (const auto & r : manifest.required) {
    nlohmann::json rj;
    rj["ns"] = r.ns;
    rj["interface_name"] = r.interface_name;
    rj["resolved_name"] = r.resolved_name;
    rj["type_name"] = r.type_name;
    rj["accept_major_min"] = r.accept_major_min;
    rj["accept_major_max"] = r.accept_major_max;
    rj["min_minor"] = r.min_minor;
    j["required"].push_back(std::move(rj));
  }
  return j.dump();
}

InterfaceManifest from_json(const std::string & doc)
{
  nlohmann::json j;
  try {
    j = nlohmann::json::parse(doc);
  } catch (const nlohmann::json::exception & e) {
    // Normalise the library's parse_error to the documented std::runtime_error contract.
    throw std::runtime_error(std::string("interface manifest: JSON parse error: ") + e.what());
  }
  if (!j.is_object()) {
    throw std::runtime_error("interface manifest: JSON root is not an object");
  }

  InterfaceManifest m;
  m.owner = optional_string(j, "owner", "");
  m.node_name = optional_string(j, "node_name", "");

  if (j.contains("provided")) {
    const auto & arr = j.at("provided");
    if (!arr.is_array()) {
      throw std::runtime_error("interface manifest: 'provided' is not an array");
    }
    for (const auto & p : arr) {
      if (!p.is_object()) {
        throw std::runtime_error("interface manifest: a 'provided' entry is not an object");
      }
      ProvidedInterface pi;
      pi.ns = optional_string(p, "ns", "");
      pi.interface_name = require_string(p, "interface_name");
      pi.resolved_name = optional_string(p, "resolved_name", pi.interface_name);
      pi.type_name = optional_string(p, "type_name", "");
      pi.major = require_uint16(p, "major");
      pi.minor = require_uint16(p, "minor");
      pi.patch = require_uint16(p, "patch");
      m.provided.push_back(std::move(pi));
    }
  }

  if (j.contains("required")) {
    const auto & arr = j.at("required");
    if (!arr.is_array()) {
      throw std::runtime_error("interface manifest: 'required' is not an array");
    }
    for (const auto & r : arr) {
      if (!r.is_object()) {
        throw std::runtime_error("interface manifest: a 'required' entry is not an object");
      }
      RequiredInterface ri;
      ri.ns = optional_string(r, "ns", "");
      ri.interface_name = require_string(r, "interface_name");
      ri.resolved_name = optional_string(r, "resolved_name", ri.interface_name);
      ri.type_name = optional_string(r, "type_name", "");
      ri.accept_major_min = require_uint16(r, "accept_major_min");
      ri.accept_major_max = require_uint16(r, "accept_major_max");
      ri.min_minor = require_uint16(r, "min_minor");
      m.required.push_back(std::move(ri));
    }
  }

  return m;
}

}  // namespace autoware::component_interface_admission
