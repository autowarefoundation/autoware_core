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

#ifndef AUTOWARE__COMPONENT_INTERFACE_ADMISSION__MANIFEST_ADMIT_CLI_HPP_
#define AUTOWARE__COMPONENT_INTERFACE_ADMISSION__MANIFEST_ADMIT_CLI_HPP_

#include <ostream>
#include <string>
#include <vector>

namespace autoware::component_interface_admission
{

// The whole of what the manifest_admit executable's main() does, factored out so its argument
// parsing and exit-code contract can be exercised directly by tests without spawning a process.
// `args` is argv[1..] (argv[0], the program name, excluded); verdict lines are written to `out`,
// diagnostics and warnings to `err`.
//
// Accepted arguments: an optional `--spec-manifest <path>` option (may repeat; the last one wins),
// which must appear before the positional manifest file arguments, followed by one or more
// `<manifest.json>` positionals. `--spec-manifest`'s file is the spec manifest
// (autoware_component_interface_specs' interface_manifest.json), parsed with spec_qos_from_json()
// and threaded into evaluate_deploy() so the spec-conformance verdict (QOS_SPEC_MISMATCH) is
// reported. Without it, a "warning: ..." line is written to `err` (that check is silently disabled
// otherwise, which would be unsafe), and an interface the spec set declares no QoS for falls back
// to evaluate_deploy()'s direct offered-vs-requested QOS_PAIR_INCOMPATIBLE check.
//
// Each positional file is parsed with manifests_from_json(): its root may be a single manifest
// document (an object) or a fragment for a multi-node package (an array, one manifest per node);
// either shape is spliced into the same manifest set. A malformed element inside an array fails
// closed exactly like a malformed single-document file.
//
// For every ACCEPTED pairing where the provider or the required entry does not carry `qos` at all,
// a "warning: ..." line naming the pairing is written to `err` — the QoS gate could not evaluate
// it, but (per the version-only verdict) this is not itself a rejection.
//
// Exit code: 0 = every pairing ACCEPTED, 1 = at least one rejection, 2 = operational/parse error
// (bad usage, unreadable file, malformed manifest, or malformed spec manifest).
int run_manifest_admit(
  const std::vector<std::string> & args, std::ostream & out, std::ostream & err);

}  // namespace autoware::component_interface_admission

#endif  // AUTOWARE__COMPONENT_INTERFACE_ADMISSION__MANIFEST_ADMIT_CLI_HPP_
