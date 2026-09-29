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

// Deploy-time admission gate. Reads N per-component interface manifest JSON files (one per
// component image, extracted from image metadata before boot), runs the shared rule's deploy-time
// trigger via evaluate_deploy() (stage 1: version + interface_name only, plus a spec QoS
// conformance check when a --spec-manifest is given), prints one verdict line per pairing, and
// exits:
//   0 = every pairing ACCEPTED
//   1 = at least one rejection (MAJOR / MINOR mismatch, NO_PROVIDER, or a QOS_* verdict)
//   2 = operational / parse error (bad usage, unreadable file, malformed manifest)
// This is the entry point the meta-repo deploy_check.sh gate invokes before `docker compose up`.
// See manifest_admit_cli.hpp for the full argument and exit-code contract; this file is a thin
// argv/stdout/stderr wrapper around run_manifest_admit(), which is what is actually unit tested.

#include "autoware/component_interface_admission/manifest_admit_cli.hpp"

#include <iostream>
#include <string>
#include <vector>

int main(int argc, char ** argv)
{
  const std::vector<std::string> args(argv + 1, argv + argc);
  return autoware::component_interface_admission::run_manifest_admit(args, std::cout, std::cerr);
}
