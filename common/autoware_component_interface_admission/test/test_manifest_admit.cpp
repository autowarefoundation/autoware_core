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

// Two layers of coverage for the deploy-time gate the manifest_admit executable drives:
//  - Library-level: parse N manifest JSON payloads, run evaluate_deploy() over the complete set,
//    and derive the exit-code verdict via any_rejected() directly (the ManifestAdmit test suite
//    below).
//  - CLI-level: drive run_manifest_admit() (manifest_admit's whole main(), factored out — see
//    manifest_admit_cli.hpp) with argv-style arguments and on-disk files, asserting its exit code
//    and what it writes to stdout/stderr (the ManifestAdmitCli test suite below). This is still a
//    single in-process function call, not a spawned process.

#include "autoware/component_interface_admission/admission_rule.hpp"
#include "autoware/component_interface_admission/manifest_admit_cli.hpp"
#include "autoware/component_interface_admission/manifest_json.hpp"
#include "autoware/component_interface_admission/records.hpp"

#include <gtest/gtest.h>

#include <cstdint>
#include <fstream>
#include <sstream>
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

// --- CLI-level (run_manifest_admit) fixtures ---

// run_manifest_admit() reads files by path, exactly like main() does, so these tests write their
// fixtures to gtest's per-run temp directory rather than passing JSON in memory.
std::string write_temp_file(const std::string & name, const std::string & content)
{
  const std::string path = ::testing::TempDir() + name;
  std::ofstream f(path);
  f << content;
  return path;
}

std::string provided_qos_manifest_json(
  const std::string & node, const std::string & interface_name, const std::string & reliability,
  const std::string & durability)
{
  adm::InterfaceManifest m;
  m.node_name = node;
  adm::ProvidedInterface p;
  p.interface_name = interface_name;
  p.resolved_name = interface_name;
  p.has_qos = true;
  p.qos = {reliability, durability, 1};
  m.provided.push_back(p);
  return adm::to_json(m);
}

std::string required_qos_manifest_json(
  const std::string & node, const std::string & interface_name, const std::string & reliability,
  const std::string & durability)
{
  adm::InterfaceManifest m;
  m.node_name = node;
  adm::RequiredInterface r;
  r.interface_name = interface_name;
  r.resolved_name = interface_name;
  r.has_qos = true;
  r.qos = {reliability, durability, 1};
  m.required.push_back(r);
  return adm::to_json(m);
}
}  // namespace

TEST(ManifestAdmitCli, spec_manifest_flag_accepted_before_positional_manifests)
{
  const std::string spec_path = write_temp_file("cli_spec_ok.json", R"({
    "interfaces": [{"interface": "/t",
                    "qos": {"reliability": "reliable", "durability": "volatile", "depth": 1}}]
  })");
  const std::string prov_path = write_temp_file(
    "cli_provider_ok.json", provided_qos_manifest_json("/n1", "/t", "reliable", "volatile"));
  const std::string cons_path = write_temp_file(
    "cli_consumer_ok.json", required_qos_manifest_json("/n2", "/t", "reliable", "volatile"));

  std::ostringstream out;
  std::ostringstream err;
  const int code =
    adm::run_manifest_admit({"--spec-manifest", spec_path, prov_path, cons_path}, out, err);
  EXPECT_EQ(code, 0);
  EXPECT_NE(out.str().find("accepted"), std::string::npos);
}

TEST(ManifestAdmitCli, qos_violation_pair_exits_1_with_a_qos_verdict_line)
{
  const std::string spec_path = write_temp_file("cli_spec_reliable.json", R"({
    "interfaces": [{"interface": "/t",
                    "qos": {"reliability": "reliable", "durability": "volatile", "depth": 1}}]
  })");
  // Both endpoints use best_effort where the spec declares reliable -> QOS_SPEC_MISMATCH on each.
  const std::string prov_path = write_temp_file(
    "cli_provider_off_spec.json",
    provided_qos_manifest_json("/n1", "/t", "best_effort", "volatile"));
  const std::string cons_path = write_temp_file(
    "cli_consumer_off_spec.json",
    required_qos_manifest_json("/n2", "/t", "best_effort", "volatile"));

  std::ostringstream out;
  std::ostringstream err;
  const int code =
    adm::run_manifest_admit({"--spec-manifest", spec_path, prov_path, cons_path}, out, err);
  EXPECT_EQ(code, 1);
  EXPECT_NE(out.str().find("QOS"), std::string::npos);
}

TEST(ManifestAdmitCli, missing_qos_manifests_exit_0_with_a_warning_on_stderr)
{
  // Plain v1-style manifests (no qos at all): the version verdict still ACCEPTs, but the QoS gate
  // could not run, so stderr must carry a warning even though the exit code stays 0.
  const std::string prov_path = write_temp_file("cli_provider_no_qos.json", provider_json(2, 1));
  const std::string cons_path = write_temp_file("cli_consumer_no_qos.json", consumer_json(2));

  std::ostringstream out;
  std::ostringstream err;
  const int code = adm::run_manifest_admit({prov_path, cons_path}, out, err);
  EXPECT_EQ(code, 0);
  EXPECT_NE(err.str().find("warning:"), std::string::npos);
}

TEST(ManifestAdmitCli, spec_manifest_flag_with_no_value_exits_2)
{
  std::ostringstream out;
  std::ostringstream err;
  const int code = adm::run_manifest_admit({"--spec-manifest"}, out, err);
  EXPECT_EQ(code, 2);
}

TEST(ManifestAdmitCli, spec_manifest_flag_given_twice_the_last_one_wins)
{
  // The first --spec-manifest points at a spec manifest that would reject this pairing (it
  // declares reliable, and both endpoints use best_effort); the second, later one is a no-op empty
  // interface list, so the pairing must pass -- proving the second occurrence overrides the first
  // rather than merging with or being ignored in favor of it.
  const std::string strict_spec_path = write_temp_file("cli_spec_twice_strict.json", R"({
    "interfaces": [{"interface": "/t",
                    "qos": {"reliability": "reliable", "durability": "volatile", "depth": 1}}]
  })");
  const std::string lenient_spec_path =
    write_temp_file("cli_spec_twice_lenient.json", R"({"interfaces": []})");
  const std::string prov_path = write_temp_file(
    "cli_provider_twice.json", provided_qos_manifest_json("/n1", "/t", "best_effort", "volatile"));
  const std::string cons_path = write_temp_file(
    "cli_consumer_twice.json", required_qos_manifest_json("/n2", "/t", "best_effort", "volatile"));

  std::ostringstream out;
  std::ostringstream err;
  const int code = adm::run_manifest_admit(
    {"--spec-manifest", strict_spec_path, "--spec-manifest", lenient_spec_path, prov_path,
     cons_path},
    out, err);
  EXPECT_EQ(code, 0);
}

TEST(ManifestAdmitCli, malformed_spec_manifest_exits_2)
{
  // No top-level `interfaces` at all: not a spec manifest, so it must fail closed rather than
  // parse to an empty QoS table that silently admits every endpoint.
  const std::string spec_path = write_temp_file("cli_spec_malformed.json", R"({"owner": "x"})");
  const std::string prov_path = write_temp_file("cli_provider_malformed.json", provider_json(2, 1));
  const std::string cons_path = write_temp_file("cli_consumer_malformed.json", consumer_json(2));

  std::ostringstream out;
  std::ostringstream err;
  const int code =
    adm::run_manifest_admit({"--spec-manifest", spec_path, prov_path, cons_path}, out, err);
  EXPECT_EQ(code, 2);
  EXPECT_FALSE(err.str().empty());
}

TEST(ManifestAdmitCli, array_root_manifest_file_admits_all_its_nodes)
{
  // A single positional file whose root is a JSON array is the on-disk shape of a multi-node
  // package's fragment (see the README's fragment-discovery note): one manifest per array element,
  // spliced together exactly as if each had been its own positional file.
  const std::string array_path = write_temp_file(
    "cli_array_root.json", "[" + provider_json(2, 1) + "," + consumer_json(2) + "]");

  std::ostringstream out;
  std::ostringstream err;
  const int code = adm::run_manifest_admit({array_path}, out, err);
  EXPECT_EQ(code, 0);
  EXPECT_NE(out.str().find("/consumer <- /provider"), std::string::npos);
}

TEST(ManifestAdmitCli, malformed_element_inside_array_root_manifest_exits_2)
{
  // The single element in this array is missing the required 'interface_name' key. A bad element
  // inside an array-root fragment must fail closed exactly like a bad single-document file.
  const std::string array_path = write_temp_file(
    "cli_array_root_malformed.json",
    R"([{"node_name":"/n1","provided":[{"major":1,"minor":0,"patch":0}]}])");

  std::ostringstream out;
  std::ostringstream err;
  const int code = adm::run_manifest_admit({array_path}, out, err);
  EXPECT_EQ(code, 2);
  EXPECT_FALSE(err.str().empty());
}

TEST(ManifestAdmitCli, non_object_non_array_root_manifest_exits_2)
{
  const std::string scalar_path = write_temp_file("cli_scalar_root.json", "42");

  std::ostringstream out;
  std::ostringstream err;
  const int code = adm::run_manifest_admit({scalar_path}, out, err);
  EXPECT_EQ(code, 2);
  EXPECT_FALSE(err.str().empty());
}

TEST(ManifestAdmitCli, no_spec_manifest_emits_a_qos_check_disabled_warning)
{
  // Without --spec-manifest, the per-endpoint spec-conformance check cannot run for any interface.
  // That must never be a silent no-op, so manifest_admit itself warns on stderr even though the
  // version-only verdict here still accepts (and exits 0).
  const std::string prov_path = write_temp_file("cli_provider_no_spec.json", provider_json(2, 1));
  const std::string cons_path = write_temp_file("cli_consumer_no_spec.json", consumer_json(2));

  std::ostringstream out;
  std::ostringstream err;
  const int code = adm::run_manifest_admit({prov_path, cons_path}, out, err);
  EXPECT_EQ(code, 0);
  EXPECT_NE(err.str().find("no --spec-manifest"), std::string::npos);
}

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
