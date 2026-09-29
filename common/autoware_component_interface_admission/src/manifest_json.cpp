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
#include <initializer_list>
#include <map>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

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

// How many of `keys` are present as keys of the (assumed object) `j`. Used to enforce the v2
// version-key group rule: major/minor/patch (or accept_major_min/accept_major_max/min_minor) must
// be all present or all absent -- a partial declaration is malformed and must throw.
int count_present(const nlohmann::json & j, std::initializer_list<const char *> keys)
{
  int n = 0;
  for (const auto * key : keys) {
    if (j.is_object() && j.contains(key)) {
      ++n;
    }
  }
  return n;
}

// QoS policy vocabularies. Any other string is out of vocabulary and from_json() must reject it
// (fail closed at the parse boundary, rather than silently accepting a string admission_rule.hpp's
// rank helpers would later treat as incomparable).
bool is_known_reliability(const std::string & policy)
{
  return policy == "reliable" || policy == "best_effort";
}

bool is_known_durability(const std::string & policy)
{
  return policy == "volatile" || policy == "transient_local";
}

QosRecord parse_qos_record(const nlohmann::json & j)
{
  QosRecord qos;
  qos.reliability = require_string(j, "reliability");
  if (!is_known_reliability(qos.reliability)) {
    throw std::runtime_error(
      "interface manifest: qos 'reliability' must be 'reliable' or 'best_effort'");
  }
  qos.durability = require_string(j, "durability");
  if (!is_known_durability(qos.durability)) {
    throw std::runtime_error(
      "interface manifest: qos 'durability' must be 'volatile' or 'transient_local'");
  }
  qos.depth = static_cast<int>(require_uint16(j, "depth"));
  return qos;
}

// Parse one entry's version-key group ("major"/"minor"/"patch" for provided, or
// "accept_major_min"/"accept_major_max"/"min_minor" for required) under the v1-always-present /
// v2-may-omit-as-a-group rule, writing has_version and the three uint16 fields via the supplied
// setters. `keys` and `setters` must be given in the same order.
void parse_version_group(
  const nlohmann::json & entry, std::initializer_list<const char *> keys, bool & has_version,
  std::uint16_t & a, std::uint16_t & b, std::uint16_t & c)
{
  const int present = count_present(entry, keys);
  if (present == 0) {
    has_version = false;
    return;
  }
  if (present != static_cast<int>(keys.size())) {
    throw std::runtime_error(
      "interface manifest: entry has a partial version key group (all of " +
      std::string(*keys.begin()) + "/... must be present, or all absent)");
  }
  has_version = true;
  auto it = keys.begin();
  a = require_uint16(entry, *it++);
  b = require_uint16(entry, *it++);
  c = require_uint16(entry, *it++);
}

InterfaceManifest parse_manifest_object(const nlohmann::json & j)
{
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
      parse_version_group(
        p, {"major", "minor", "patch"}, pi.has_version, pi.major, pi.minor, pi.patch);
      pi.has_qos = p.contains("qos");
      if (pi.has_qos) {
        pi.qos = parse_qos_record(p.at("qos"));
      }
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
      parse_version_group(
        r, {"accept_major_min", "accept_major_max", "min_minor"}, ri.has_version,
        ri.accept_major_min, ri.accept_major_max, ri.min_minor);
      ri.has_qos = r.contains("qos");
      if (ri.has_qos) {
        ri.qos = parse_qos_record(r.at("qos"));
      }
      m.required.push_back(std::move(ri));
    }
  }

  return m;
}

}  // namespace

namespace
{
nlohmann::json qos_to_json(const QosRecord & qos)
{
  nlohmann::json qj;
  qj["reliability"] = qos.reliability;
  qj["durability"] = qos.durability;
  qj["depth"] = qos.depth;
  return qj;
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
    if (p.has_version) {
      pj["major"] = p.major;
      pj["minor"] = p.minor;
      pj["patch"] = p.patch;
    }
    if (p.has_qos) {
      pj["qos"] = qos_to_json(p.qos);
    }
    j["provided"].push_back(std::move(pj));
  }
  j["required"] = nlohmann::json::array();
  for (const auto & r : manifest.required) {
    nlohmann::json rj;
    rj["ns"] = r.ns;
    rj["interface_name"] = r.interface_name;
    rj["resolved_name"] = r.resolved_name;
    rj["type_name"] = r.type_name;
    if (r.has_version) {
      rj["accept_major_min"] = r.accept_major_min;
      rj["accept_major_max"] = r.accept_major_max;
      rj["min_minor"] = r.min_minor;
    }
    if (r.has_qos) {
      rj["qos"] = qos_to_json(r.qos);
    }
    j["required"].push_back(std::move(rj));
  }
  return j.dump();
}

namespace
{
nlohmann::json parse_json_or_throw(const std::string & doc, const char * context)
{
  try {
    return nlohmann::json::parse(doc);
  } catch (const nlohmann::json::exception & e) {
    // Normalise the library's parse_error to the documented std::runtime_error contract.
    throw std::runtime_error(std::string(context) + ": JSON parse error: " + e.what());
  }
}
}  // namespace

InterfaceManifest from_json(const std::string & doc)
{
  return parse_manifest_object(parse_json_or_throw(doc, "interface manifest"));
}

std::vector<InterfaceManifest> manifests_from_json(const std::string & doc)
{
  const auto j = parse_json_or_throw(doc, "interface manifest");

  if (j.is_object()) {
    return {parse_manifest_object(j)};
  }
  if (j.is_array()) {
    std::vector<InterfaceManifest> manifests;
    manifests.reserve(j.size());
    for (const auto & element : j) {
      manifests.push_back(parse_manifest_object(element));
    }
    return manifests;
  }
  throw std::runtime_error("interface manifest: JSON root is neither an object nor an array");
}

std::map<std::string, QosRecord> spec_qos_from_json(const std::string & doc)
{
  const auto j = parse_json_or_throw(doc, "spec manifest");
  if (!j.is_object()) {
    throw std::runtime_error("spec manifest: JSON root is not an object");
  }

  // `interfaces` is REQUIRED, not optional: a document without it is not a spec manifest, and
  // accepting one would hand back an empty QoS table that silently admits every endpoint.
  if (!j.contains("interfaces")) {
    throw std::runtime_error("spec manifest: missing top-level 'interfaces'");
  }
  const auto & arr = j.at("interfaces");
  if (!arr.is_array()) {
    throw std::runtime_error("spec manifest: 'interfaces' is not an array");
  }

  std::map<std::string, QosRecord> spec_qos;
  for (const auto & entry : arr) {
    if (!entry.is_object()) {
      throw std::runtime_error("spec manifest: an 'interfaces' entry is not an object");
    }
    const auto interface_name = require_string(entry, "interface");
    if (!entry.contains("qos")) {
      throw std::runtime_error(
        std::string("spec manifest: entry '") + interface_name + "' is missing 'qos'");
    }
    spec_qos[interface_name] = parse_qos_record(entry.at("qos"));
  }
  return spec_qos;
}

}  // namespace autoware::component_interface_admission
