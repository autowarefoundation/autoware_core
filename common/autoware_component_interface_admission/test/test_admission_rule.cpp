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

#include "autoware/component_interface_admission/admission_rule.hpp"
#include "autoware/component_interface_admission/records.hpp"

#include <gtest/gtest.h>

#include <cstdint>
#include <string>

namespace adm = autoware::component_interface_admission;

namespace
{
constexpr char kIf[] = "/perception/object_recognition/objects";

adm::InterfaceManifest provider(
  std::uint16_t major, std::uint16_t minor, const std::string & node = "/provider",
  const std::string & resolved = kIf)
{
  adm::InterfaceManifest m;
  m.node_name = node;
  adm::ProvidedInterface p;
  p.interface_name = kIf;
  p.resolved_name = resolved;
  p.major = major;
  p.minor = minor;
  m.provided.push_back(p);
  return m;
}

adm::InterfaceManifest consumer(
  std::uint16_t lo, std::uint16_t hi, std::uint16_t min_minor = 0,
  const std::string & resolved = kIf)
{
  adm::InterfaceManifest m;
  m.node_name = "/consumer";
  adm::RequiredInterface r;
  r.interface_name = kIf;
  r.resolved_name = resolved;
  r.accept_major_min = lo;
  r.accept_major_max = hi;
  r.min_minor = min_minor;
  m.required.push_back(r);
  return m;
}
}  // namespace

TEST(AdmissionRule, accepts_same_major)
{
  const auto results = adm::evaluate({provider(2, 1), consumer(2, 2)});
  ASSERT_EQ(results.size(), 1u);
  EXPECT_EQ(results[0].code, adm::ACCEPTED);
  EXPECT_EQ(results[0].provider_node, "/provider");
  EXPECT_EQ(results[0].consumer_node, "/consumer");
}

// A required MAJOR ahead of what the provider offers: provider 2.1.0, consumer built against
// MAJOR 3 -> reject.
TEST(AdmissionRule, rejects_higher_required_major)
{
  const auto results = adm::evaluate({provider(2, 1), consumer(3, 3)});
  ASSERT_EQ(results.size(), 1u);
  EXPECT_EQ(results[0].code, adm::MAJOR_MISMATCH);
}

// A migration window [2, 3] accepts both a MAJOR-2 and a MAJOR-3 provider.
TEST(AdmissionRule, migration_window_accepts_both_majors)
{
  const auto lo = adm::evaluate({provider(2, 5), consumer(2, 3)});
  ASSERT_EQ(lo.size(), 1u);
  EXPECT_EQ(lo[0].code, adm::ACCEPTED);

  const auto hi = adm::evaluate({provider(3, 0), consumer(2, 3)});
  ASSERT_EQ(hi.size(), 1u);
  EXPECT_EQ(hi[0].code, adm::ACCEPTED);
}

TEST(AdmissionRule, rejects_when_min_minor_unmet)
{
  const auto results = adm::evaluate({provider(2, 1), consumer(2, 2, /*min_minor=*/5)});
  ASSERT_EQ(results.size(), 1u);
  EXPECT_EQ(results[0].code, adm::MINOR_MISMATCH);
}

TEST(AdmissionRule, accepts_at_min_minor_boundary)
{
  // provider MINOR == the required min_minor: the lower bound is inclusive.
  const auto results = adm::evaluate({provider(2, 5), consumer(2, 2, /*min_minor=*/5)});
  ASSERT_EQ(results.size(), 1u);
  EXPECT_EQ(results[0].code, adm::ACCEPTED);
}

TEST(AdmissionRule, min_minor_binds_only_to_its_declared_major)
{
  // min_minor is a lower bound on the provider's MINOR declared against the LOWER accepted MAJOR
  // (accept_major_min). Per semver, MINOR resets to 0 on every MAJOR bump, so the bound must NOT
  // carry into a higher accepted MAJOR within the same acceptance window.
  const auto cons = consumer(2, 3, /*min_minor=*/5);  // accept [2, 3], min_minor 5

  // Higher accepted MAJOR (3): min_minor does not apply, so provider 3.0 is ACCEPTED.
  const auto higher_major = adm::evaluate({provider(3, 0), cons});
  ASSERT_EQ(higher_major.size(), 1u);
  EXPECT_EQ(higher_major[0].code, adm::ACCEPTED);

  // Declared MAJOR (2), below the bound: MINOR_MISMATCH.
  const auto below = adm::evaluate({provider(2, 4), cons});
  ASSERT_EQ(below.size(), 1u);
  EXPECT_EQ(below[0].code, adm::MINOR_MISMATCH);

  // Declared MAJOR (2), at the bound: ACCEPTED (inclusive lower bound).
  const auto at_bound = adm::evaluate({provider(2, 5), cons});
  ASSERT_EQ(at_bound.size(), 1u);
  EXPECT_EQ(at_bound[0].code, adm::ACCEPTED);
}

TEST(AdmissionRule, rejects_remap_topic_mismatch)
{
  // Same logical IF + compatible MAJOR, but the provider's wire topic was remapped away.
  // Logical-name-only admission (matching on interface_name alone) would false-accept; the
  // resolved_name comparison catches the disjoint wiring.
  const auto results = adm::evaluate(
    {provider(2, 1, "/provider", "/perception/object_recognition/objects_remapped"),
     consumer(2, 2)});
  ASSERT_EQ(results.size(), 1u);
  EXPECT_EQ(results[0].code, adm::TOPIC_MISMATCH);
}

TEST(AdmissionRule, picks_wired_provider_among_several)
{
  // Two version-compatible providers of the same interface: one remapped onto a disjoint wire
  // topic, one whose resolved_name coincides with the consumer's. Admission must pick the wired
  // one (stage-2 resolved_name match), not the first version-compatible entry.
  const auto disjoint =
    provider(2, 1, "/provider_remapped", "/perception/object_recognition/objects_remapped");
  const auto wired = provider(2, 1, "/provider_wired", kIf);
  const auto results = adm::evaluate({disjoint, wired, consumer(2, 2)});
  ASSERT_EQ(results.size(), 1u);
  EXPECT_EQ(results[0].code, adm::ACCEPTED);
  EXPECT_EQ(results[0].provider_node, "/provider_wired");
}

TEST(AdmissionRule, runtime_skips_missing_provider_but_deploy_reports_it)
{
  // Runtime observe mode: a required interface with no provider is SKIPPED (the provider may not
  // have started yet).
  const auto runtime = adm::evaluate({consumer(2, 2)});
  EXPECT_TRUE(runtime.empty());

  // Deploy time: the set is complete, so the same missing provider is a hard NO_PROVIDER.
  const auto deploy = adm::evaluate_deploy({consumer(2, 2)});
  ASSERT_EQ(deploy.size(), 1u);
  EXPECT_EQ(deploy[0].code, adm::NO_PROVIDER);
  EXPECT_TRUE(adm::any_rejected(deploy));
}

TEST(AdmissionRule, deploy_ignores_remap_resolved_name_but_runtime_catches_it)
{
  // Two manifests, same interface_name, version-compatible, but the provider's remap left it on a
  // DIVERGENT resolved_name. The deploy trigger reads static image metadata, where remaps (which
  // live in the launch / compose layer) are not visible, so it matches on interface_name + version
  // ONLY (stage 1) and must ACCEPT — it must never emit TOPIC_MISMATCH.
  const auto prov = provider(2, 1, "/provider", "/perception/object_recognition/objects_remapped");
  const auto cons = consumer(2, 2);  // resolved_name = kIf, divergent from the provider's

  const auto deploy = adm::evaluate_deploy({prov, cons});
  ASSERT_EQ(deploy.size(), 1u);
  EXPECT_EQ(deploy[0].code, adm::ACCEPTED);
  EXPECT_EQ(deploy[0].provider_node, "/provider");
  EXPECT_FALSE(adm::any_rejected(deploy));

  // Companion assertion: the SAME pair under the runtime trigger still catches the disjoint wiring
  // as a TOPIC_MISMATCH — stage 2 (resolved_name) stays a runtime-only backstop.
  const auto runtime = adm::evaluate({prov, cons});
  ASSERT_EQ(runtime.size(), 1u);
  EXPECT_EQ(runtime[0].code, adm::TOPIC_MISMATCH);
}

TEST(AdmissionRule, empty_input_yields_no_results)
{
  EXPECT_TRUE(adm::evaluate({}).empty());
  EXPECT_TRUE(adm::evaluate_deploy({}).empty());
  EXPECT_FALSE(adm::any_rejected({}));
}

TEST(AdmissionRule, verdict_text_covers_all_codes)
{
  // The reason string is presentational and derived off-wire; the wire contract is the verdict
  // CODE, not its wording. So assert only the load-bearing behavior: every known code yields a
  // non-empty reason (each switch branch is exercised), and an unrecognized code hits the safe
  // "unknown" fallback via the default branch instead of returning an empty string. The exact
  // prose is free to change without any behavior change, so it is not asserted here.
  for (const std::uint16_t code :
       {adm::ACCEPTED, adm::MAJOR_MISMATCH, adm::MINOR_MISMATCH, adm::TOPIC_MISMATCH,
        adm::NO_PROVIDER}) {
    EXPECT_STRNE(adm::verdict_text(code), "");
  }
  EXPECT_STREQ(adm::verdict_text(999), "unknown");
}

TEST(AdmissionRule, rejection_names_the_wire_relevant_provider)
{
  // Two providers, neither version-compatible. One sits on a disjoint wire topic, the other on
  // the consumer's own resolved_name. The rejection diagnostic must name the provider the
  // consumer is actually wired to, not whichever happens to be registered first.
  const auto off_wire = provider(9, 0, "/p1_bar", "/perception/object_recognition/objects_bar");
  const auto on_wire = provider(5, 0, "/p2_foo", kIf);
  const auto results = adm::evaluate({off_wire, on_wire, consumer(1, 1)});
  ASSERT_EQ(results.size(), 1u);
  EXPECT_EQ(results[0].code, adm::MAJOR_MISMATCH);
  EXPECT_EQ(results[0].provider_node, "/p2_foo");
}

TEST(AdmissionRule, rejection_prefers_the_actionable_minor_mismatch)
{
  // Neither provider satisfies the bounds, but one is only a MINOR behind (MAJOR in range).
  // That is the actionable upgrade, so it must be the reported verdict and provider.
  const auto major_out = provider(9, 0, "/p_major_out");
  const auto minor_short = provider(2, 1, "/p_minor_short");
  const auto results = adm::evaluate({major_out, minor_short, consumer(2, 2, 5)});
  ASSERT_EQ(results.size(), 1u);
  EXPECT_EQ(results[0].code, adm::MINOR_MISMATCH);
  EXPECT_EQ(results[0].provider_node, "/p_minor_short");
}

TEST(AdmissionRule, rejection_verdict_is_independent_of_manifest_order)
{
  // manifest_admit receives its manifests in argv order; the verdict must not depend on it.
  const auto a = provider(9, 0, "/p_a", "/perception/object_recognition/objects_a");
  const auto b = provider(7, 0, "/p_b", "/perception/object_recognition/objects_b");
  const auto c = consumer(1, 1);

  const auto forward = adm::evaluate({a, b, c});
  const auto reversed = adm::evaluate({b, a, c});
  ASSERT_EQ(forward.size(), 1u);
  ASSERT_EQ(reversed.size(), 1u);
  EXPECT_EQ(forward[0].code, reversed[0].code);
  EXPECT_EQ(forward[0].provider_node, reversed[0].provider_node);
}

TEST(AdmissionRule, topic_mismatch_provider_is_independent_of_manifest_order)
{
  // Two version-compatible providers, both left on disjoint wire topics by a remap.
  const auto a = provider(2, 1, "/p_a", "/perception/object_recognition/objects_a");
  const auto b = provider(2, 1, "/p_b", "/perception/object_recognition/objects_b");
  const auto c = consumer(2, 2);

  const auto forward = adm::evaluate({a, b, c});
  const auto reversed = adm::evaluate({b, a, c});
  ASSERT_EQ(forward.size(), 1u);
  ASSERT_EQ(reversed.size(), 1u);
  EXPECT_EQ(forward[0].code, adm::TOPIC_MISMATCH);
  EXPECT_EQ(forward[0].provider_node, reversed[0].provider_node);
}
