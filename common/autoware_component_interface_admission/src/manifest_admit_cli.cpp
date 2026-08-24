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

#include "autoware/component_interface_admission/manifest_admit_cli.hpp"

#include "autoware/component_interface_admission/admission_rule.hpp"
#include "autoware/component_interface_admission/manifest_json.hpp"
#include "autoware/component_interface_admission/records.hpp"

#include <cstddef>
#include <exception>
#include <fstream>
#include <iterator>
#include <map>
#include <ostream>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

namespace autoware::component_interface_admission
{

namespace
{

// Reads the whole file at `path` and returns its contents via `out`. On failure, writes a
// diagnostic naming `what` (e.g. "manifest" or "spec manifest") to `err` and returns false.
bool read_file(const std::string & path, const char * what, std::string & out, std::ostream & err)
{
  std::ifstream f(path);
  if (!f) {
    err << "manifest_admit: cannot open " << what << " " << path << "\n";
    return false;
  }
  std::ostringstream ss;
  ss << f.rdbuf();
  out = ss.str();
  return true;
}

}  // namespace

int run_manifest_admit(
  const std::vector<std::string> & args, std::ostream & out, std::ostream & err)
{
  std::string spec_manifest_path;
  bool has_spec_manifest = false;

  std::size_t i = 0;
  while (i < args.size() && args[i] == "--spec-manifest") {
    if (i + 1 >= args.size()) {
      err << "manifest_admit: --spec-manifest requires a path argument\n";
      return 2;
    }
    spec_manifest_path = args[i + 1];  // repeatable: the last occurrence wins
    has_spec_manifest = true;
    i += 2;
  }

  const std::vector<std::string> manifest_paths(
    args.begin() + static_cast<std::ptrdiff_t>(i), args.end());
  if (manifest_paths.empty()) {
    err << "usage: manifest_admit [--spec-manifest <interface_manifest.json>] "
           "<manifest.json> [<manifest.json> ...]\n";
    return 2;
  }

  std::map<std::string, QosRecord> spec_qos;
  if (has_spec_manifest) {
    std::string doc;
    if (!read_file(spec_manifest_path, "spec manifest", doc, err)) {
      return 2;
    }
    try {
      spec_qos = spec_qos_from_json(doc);
    } catch (const std::exception & e) {
      err << "manifest_admit: failed to parse spec manifest " << spec_manifest_path << ": "
          << e.what() << "\n";
      return 2;
    }
  } else {
    // The per-endpoint spec-conformance check (QOS_SPEC_MISMATCH) never runs without a spec QoS
    // table, so a deploy whose endpoints deviate from their specs' declared QoS can exit 0 here.
    // That must never be a silent no-op for a safety-adjacent gate.
    err << "warning: manifest_admit: no --spec-manifest supplied; the spec QoS conformance "
           "verdict (QOS_SPEC_MISMATCH) is disabled for this run\n";
  }

  std::vector<InterfaceManifest> manifests;
  manifests.reserve(manifest_paths.size());
  for (const auto & path : manifest_paths) {
    std::string doc;
    if (!read_file(path, "manifest", doc, err)) {
      return 2;
    }
    try {
      // Each positional file may hold either a single manifest document (an object root) or a
      // fragment for a multi-node package (an array root, one manifest per node) -- see the
      // package README's fragment-discovery note. manifests_from_json() throws on anything else,
      // and on a malformed element inside an array, so a bad fragment still fails closed here
      // exactly as a bad single-document one always has.
      auto parsed = manifests_from_json(doc);
      manifests.insert(
        manifests.end(), std::make_move_iterator(parsed.begin()),
        std::make_move_iterator(parsed.end()));
    } catch (const std::exception & e) {
      err << "manifest_admit: failed to parse " << path << ": " << e.what() << "\n";
      return 2;
    }
  }

  const auto results = evaluate_deploy(manifests, spec_qos);
  for (const auto & r : results) {
    // QOS_SPEC_MISMATCH is a per-endpoint row: one of the two node fields is left empty (see
    // AdmissionResult), so it is substituted with a placeholder for readability.
    const std::string consumer = r.consumer_node.empty() ? "(no consumer)" : r.consumer_node;
    const std::string provider = r.provider_node.empty() ? "(no provider)" : r.provider_node;
    out << consumer << " <- " << provider << " [" << r.interface_name
        << "]: " << verdict_text(r.code) << " (code=" << r.code << ")\n";
  }
  if (results.empty()) {
    out << "manifest_admit: no consumer/provider pairings to evaluate\n";
  }

  // Best-effort diagnostic: for every ACCEPTED pairing, warn (do not reject) if the QoS gate could
  // not run because the matched provider or the required entry itself does not carry qos.
  std::map<std::pair<std::string, std::string>, const RequiredInterface *> required_by_node_and_if;
  std::map<std::pair<std::string, std::string>, const ProvidedInterface *> provided_by_node_and_if;
  for (const auto & m : manifests) {
    for (const auto & r : m.required) {
      required_by_node_and_if[{m.node_name, r.interface_name}] = &r;
    }
    for (const auto & p : m.provided) {
      provided_by_node_and_if[{m.node_name, p.interface_name}] = &p;
    }
  }
  for (const auto & res : results) {
    if (res.code != ACCEPTED) {
      continue;
    }
    const auto rit = required_by_node_and_if.find({res.consumer_node, res.interface_name});
    const auto pit = provided_by_node_and_if.find({res.provider_node, res.interface_name});
    const bool consumer_has_qos = rit != required_by_node_and_if.end() && rit->second->has_qos;
    const bool provider_has_qos = pit != provided_by_node_and_if.end() && pit->second->has_qos;
    if (!consumer_has_qos || !provider_has_qos) {
      err << "warning: " << res.consumer_node << " <- " << res.provider_node << " ["
          << res.interface_name << "]: no QoS on one or both sides; QoS compatibility was not "
          << "evaluated for this pairing\n";
    }
  }

  return any_rejected(results) ? 1 : 0;
}

}  // namespace autoware::component_interface_admission
