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

// Library-level coverage of the deploy-time gate the manifest_admit executable drives: parse N
// manifest JSON payloads, run evaluate_deploy() over the complete set, and derive the exit-code
// verdict via any_rejected(). The executable itself is a thin argv / file / stdout wrapper around
// exactly these calls, so its exit-code contract (0 = all accepted, 1 = any rejection) is what is
// asserted here; there is no separate process-level CLI test.

#include "autoware/component_interface_admission/admission_rule.hpp"
#include "autoware/component_interface_admission/manifest_json.hpp"
#include "autoware/component_interface_admission/records.hpp"

#include <gtest/gtest.h>

#include <cstdint>
#include <string>
#include <vector>

namespace adm = autoware::component_interface_admission;

namespace
{
constexpr char kIf[] = "/perception/object_recognition/objects";

std::string provider_json(std::uint16_t major, std::uint16_t minor)
{
  adm::InterfaceManifest m;
  m.owner = "autowarefoundation";
  m.node_name = "/provider";
  adm::ProvidedInterface p;
  p.ns = "perception";
  p.interface_name = kIf;
  p.resolved_name = kIf;
  p.type_name = "autoware_perception_msgs/msg/PredictedObjects";
  p.major = major;
  p.minor = minor;
  m.provided.push_back(p);
  return adm::to_json(m);
}

std::string consumer_json(std::uint16_t accept_major)
{
  adm::InterfaceManifest m;
  m.owner = "autowarefoundation";
  m.node_name = "/consumer";
  adm::RequiredInterface r;
  r.ns = "perception";
  r.interface_name = kIf;
  r.resolved_name = kIf;
  r.type_name = "autoware_perception_msgs/msg/PredictedObjects";
  r.accept_major_min = accept_major;
  r.accept_major_max = accept_major;
  m.required.push_back(r);
  return adm::to_json(m);
}

std::vector<adm::InterfaceManifest> parse_all(const std::vector<std::string> & docs)
{
  std::vector<adm::InterfaceManifest> manifests;
  manifests.reserve(docs.size());
  for (const auto & d : docs) {
    manifests.push_back(adm::from_json(d));
  }
  return manifests;
}
}  // namespace

TEST(ManifestAdmit, accepts_compatible_image_set)
{
  const auto results = adm::evaluate_deploy(parse_all({provider_json(2, 1), consumer_json(2)}));
  ASSERT_EQ(results.size(), 1u);
  EXPECT_EQ(results[0].code, adm::ACCEPTED);
  EXPECT_FALSE(adm::any_rejected(results));
}

TEST(ManifestAdmit, rejects_incompatible_image_set)
{
  // Provider 2.1.0, consumer built against MAJOR 3: the same MAJOR-mismatch condition as
  // AdmissionRule.rejects_higher_required_major in test_admission_rule.cpp, now driven through
  // JSON manifests and evaluate_deploy() instead of evaluate().
  const auto results = adm::evaluate_deploy(parse_all({provider_json(2, 1), consumer_json(3)}));
  ASSERT_EQ(results.size(), 1u);
  EXPECT_EQ(results[0].code, adm::MAJOR_MISMATCH);
  EXPECT_TRUE(adm::any_rejected(results));
}

TEST(ManifestAdmit, rejects_required_with_no_provider)
{
  // A deploy set where the consumer requires the interface but NO image provides it. The runtime
  // observe-mode evaluate() skips this (a provider may not have started); the deploy-time gate
  // must reject it because the whole set is known up front.
  const auto results = adm::evaluate_deploy(parse_all({consumer_json(2)}));
  ASSERT_EQ(results.size(), 1u);
  EXPECT_EQ(results[0].code, adm::NO_PROVIDER);
  EXPECT_TRUE(adm::any_rejected(results));
}
