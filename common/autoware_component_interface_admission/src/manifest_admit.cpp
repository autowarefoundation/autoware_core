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
// trigger via evaluate_deploy() (stage 1: version + interface_name only), prints one verdict line
// per pairing, and exits:
//   0 = every pairing ACCEPTED
//   1 = at least one rejection (MAJOR / MINOR mismatch or NO_PROVIDER)
//   2 = operational / parse error (bad usage, unreadable file, malformed manifest)
// This is the entry point the meta-repo deploy_check.sh gate invokes before `docker compose up`.

#include "autoware/component_interface_admission/admission_rule.hpp"
#include "autoware/component_interface_admission/manifest_json.hpp"
#include "autoware/component_interface_admission/records.hpp"

#include <exception>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

int main(int argc, char ** argv)
{
  namespace adm = autoware::component_interface_admission;

  if (argc < 2) {
    std::cerr << "usage: manifest_admit <manifest.json> [<manifest.json> ...]\n";
    return 2;
  }

  std::vector<adm::InterfaceManifest> manifests;
  manifests.reserve(static_cast<std::size_t>(argc - 1));
  for (int i = 1; i < argc; ++i) {
    std::ifstream f(argv[i]);
    if (!f) {
      std::cerr << "manifest_admit: cannot open " << argv[i] << "\n";
      return 2;
    }
    std::ostringstream ss;
    ss << f.rdbuf();
    try {
      manifests.push_back(adm::from_json(ss.str()));
    } catch (const std::exception & e) {
      std::cerr << "manifest_admit: failed to parse " << argv[i] << ": " << e.what() << "\n";
      return 2;
    }
  }

  const auto results = adm::evaluate_deploy(manifests);
  for (const auto & r : results) {
    std::cout << r.consumer_node << " <- " << r.provider_node << " [" << r.interface_name
              << "]: " << adm::verdict_text(r.code) << " (code=" << r.code << ")\n";
  }
  if (results.empty()) {
    std::cout << "manifest_admit: no consumer/provider pairings to evaluate\n";
  }
  return adm::any_rejected(results) ? 1 : 0;
}
