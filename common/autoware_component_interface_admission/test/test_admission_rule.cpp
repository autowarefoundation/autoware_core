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
#include <map>
#include <string>
#include <utility>
#include <vector>

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

// --- QoS verdict fixtures ---

adm::InterfaceManifest make_manifest_with_provided(
  const std::string & node, const std::string & interface_name, adm::QosRecord qos)
{
  adm::InterfaceManifest m;
  m.node_name = node;
  adm::ProvidedInterface p;
  p.interface_name = interface_name;
  p.resolved_name = interface_name;
  p.has_qos = true;
  p.qos = std::move(qos);
  m.provided.push_back(std::move(p));
  return m;
}

adm::InterfaceManifest make_manifest_with_provided_no_qos(
  const std::string & node, const std::string & interface_name)
{
  adm::InterfaceManifest m;
  m.node_name = node;
  adm::ProvidedInterface p;
  p.interface_name = interface_name;
  p.resolved_name = interface_name;
  m.provided.push_back(std::move(p));
  return m;
}

adm::InterfaceManifest make_manifest_with_required(
  const std::string & node, const std::string & interface_name, adm::QosRecord qos)
{
  adm::InterfaceManifest m;
  m.node_name = node;
  adm::RequiredInterface r;
  r.interface_name = interface_name;
  r.resolved_name = interface_name;
  r.has_qos = true;
  r.qos = std::move(qos);
  m.required.push_back(std::move(r));
  return m;
}

adm::InterfaceManifest make_manifest_with_required_no_qos(
  const std::string & node, const std::string & interface_name)
{
  adm::InterfaceManifest m;
  m.node_name = node;
  adm::RequiredInterface r;
  r.interface_name = interface_name;
  r.resolved_name = interface_name;
  m.required.push_back(std::move(r));
  return m;
}

bool has_verdict(const std::vector<adm::AdmissionResult> & results, std::uint16_t code)
{
  for (const auto & r : results) {
    if (r.code == code) {
      return true;
    }
  }
  return false;
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

TEST(AdmissionRule, verdict_text_covers_the_qos_codes_too)
{
  for (const std::uint16_t code : {adm::QOS_SPEC_MISMATCH, adm::QOS_PAIR_INCOMPATIBLE}) {
    EXPECT_STRNE(adm::verdict_text(code), "");
  }
}

// --- Spec QoS conformance verdicts (evaluate_deploy(manifests, spec_qos)) ---

TEST(admission_rule, provider_weaker_than_the_spec_qos_is_rejected)
{
  auto provider = make_manifest_with_provided("/n1", "/t", /*qos*/ {"best_effort", "volatile", 1});
  auto consumer = make_manifest_with_required("/n2", "/t", /*qos*/ {"reliable", "volatile", 1});
  const std::map<std::string, adm::QosRecord> spec_qos{{"/t", {"reliable", "volatile", 1}}};
  const auto verdicts = adm::evaluate_deploy({provider, consumer}, spec_qos);
  EXPECT_TRUE(has_verdict(verdicts, adm::QOS_SPEC_MISMATCH));
}

TEST(admission_rule, provider_stronger_than_the_spec_qos_is_also_rejected)
{
  // The declared QoS is an exact requirement, not a lower bound: a provider offering
  // transient_local where the spec says volatile is as non-conforming as one offering less.
  // Deviating in the "safer" direction is still deviating from what every consumer written against
  // the specification was told to expect.
  auto provider = make_manifest_with_provided("/n1", "/t", {"reliable", "transient_local", 1});
  auto consumer = make_manifest_with_required("/n2", "/t", {"reliable", "volatile", 1});
  const std::map<std::string, adm::QosRecord> spec_qos{{"/t", {"reliable", "volatile", 1}}};
  const auto verdicts = adm::evaluate_deploy({provider, consumer}, spec_qos);
  EXPECT_TRUE(has_verdict(verdicts, adm::QOS_SPEC_MISMATCH));
}

TEST(admission_rule, consumer_weaker_than_the_spec_qos_is_rejected)
{
  // The mistake interface specifications exist to prevent: the spec declares RELIABLE, meaning the
  // interface is carried without drops or reordering, and a subscription quietly asks for
  // BEST_EFFORT instead. DDS's request-vs-offered rule would connect that pair happily; the gate
  // must not, because the delivery guarantee the specification promised is gone.
  auto provider = make_manifest_with_provided("/n1", "/t", {"reliable", "volatile", 1});
  auto consumer = make_manifest_with_required("/n2", "/t", {"best_effort", "volatile", 1});
  const std::map<std::string, adm::QosRecord> spec_qos{{"/t", {"reliable", "volatile", 1}}};
  const auto verdicts = adm::evaluate_deploy({provider, consumer}, spec_qos);
  EXPECT_TRUE(has_verdict(verdicts, adm::QOS_SPEC_MISMATCH));
}

TEST(admission_rule, consumer_stronger_than_the_spec_qos_is_also_rejected)
{
  auto provider = make_manifest_with_provided("/n1", "/t", {"reliable", "volatile", 1});
  auto consumer = make_manifest_with_required("/n2", "/t", {"reliable", "transient_local", 1});
  const std::map<std::string, adm::QosRecord> spec_qos{{"/t", {"reliable", "volatile", 1}}};
  const auto verdicts = adm::evaluate_deploy({provider, consumer}, spec_qos);
  EXPECT_TRUE(has_verdict(verdicts, adm::QOS_SPEC_MISMATCH));
}

TEST(admission_rule, endpoints_matching_the_spec_qos_exactly_are_accepted)
{
  auto provider = make_manifest_with_provided("/n1", "/t", {"reliable", "volatile", 1});
  auto consumer = make_manifest_with_required("/n2", "/t", {"reliable", "volatile", 1});
  const std::map<std::string, adm::QosRecord> spec_qos{{"/t", {"reliable", "volatile", 1}}};
  const auto verdicts = adm::evaluate_deploy({provider, consumer}, spec_qos);
  EXPECT_FALSE(has_verdict(verdicts, adm::QOS_SPEC_MISMATCH));
  EXPECT_TRUE(has_verdict(verdicts, adm::ACCEPTED));
}

TEST(admission_rule, an_out_of_vocabulary_policy_string_never_matches_the_spec_qos)
{
  // Fail closed: a policy string this package cannot name is not "close enough" to the declared
  // one, so it is a mismatch rather than something that slips through unevaluated.
  auto provider = make_manifest_with_provided("/n1", "/t", {"garbage", "volatile", 1});
  const std::map<std::string, adm::QosRecord> spec_qos{{"/t", {"reliable", "volatile", 1}}};
  const auto verdicts = adm::evaluate_deploy({provider}, spec_qos);
  EXPECT_TRUE(has_verdict(verdicts, adm::QOS_SPEC_MISMATCH));
}

TEST(admission_rule, no_spec_qos_row_incompatible_pair_is_rejected)
{
  // A vendor / out-of-tree interface the spec set declares nothing for: with no declared QoS to
  // hold either side to, fall back to the direct offered-vs-requested pairing check. offered
  // best_effort against requested reliable cannot connect at all -> QOS_PAIR_INCOMPATIBLE.
  auto provider = make_manifest_with_provided("/n1", "/vendor_if", {"best_effort", "volatile", 1});
  auto consumer = make_manifest_with_required("/n2", "/vendor_if", {"reliable", "volatile", 1});
  const auto verdicts = adm::evaluate_deploy({provider, consumer}, {});
  EXPECT_TRUE(has_verdict(verdicts, adm::QOS_PAIR_INCOMPATIBLE));
}

TEST(admission_rule, no_spec_qos_row_compatible_pair_is_accepted)
{
  auto provider = make_manifest_with_provided("/n1", "/vendor_if", {"reliable", "volatile", 1});
  auto consumer = make_manifest_with_required("/n2", "/vendor_if", {"best_effort", "volatile", 1});
  const auto verdicts = adm::evaluate_deploy({provider, consumer}, {});
  EXPECT_FALSE(has_verdict(verdicts, adm::QOS_PAIR_INCOMPATIBLE));
  EXPECT_TRUE(has_verdict(verdicts, adm::ACCEPTED));
}

TEST(admission_rule, provider_without_qos_produces_no_qos_verdict)
{
  // The warning for a missing qos is a CLI-level concern (manifest_admit), not a verdict here.
  auto provider = make_manifest_with_provided_no_qos("/n1", "/t");
  auto consumer = make_manifest_with_required("/n2", "/t", {"reliable", "volatile", 1});
  const std::map<std::string, adm::QosRecord> spec_qos{{"/t", {"reliable", "volatile", 1}}};
  const auto verdicts = adm::evaluate_deploy({provider, consumer}, spec_qos);
  EXPECT_FALSE(has_verdict(verdicts, adm::QOS_SPEC_MISMATCH));
  EXPECT_FALSE(has_verdict(verdicts, adm::QOS_PAIR_INCOMPATIBLE));
  EXPECT_TRUE(has_verdict(verdicts, adm::ACCEPTED));
}

TEST(admission_rule, depth_mismatch_never_produces_a_verdict)
{
  // depth is presentational only; a 1-vs-10 mismatch must never surface as a verdict, whether the
  // spec set declares a QoS for the interface or not.
  auto provider = make_manifest_with_provided("/n1", "/t", {"reliable", "volatile", 1});
  auto consumer = make_manifest_with_required("/n2", "/t", {"reliable", "volatile", 10});

  const std::map<std::string, adm::QosRecord> spec_qos{{"/t", {"reliable", "volatile", 5}}};
  const auto with_spec_qos = adm::evaluate_deploy({provider, consumer}, spec_qos);
  EXPECT_FALSE(has_verdict(with_spec_qos, adm::QOS_SPEC_MISMATCH));
  EXPECT_TRUE(has_verdict(with_spec_qos, adm::ACCEPTED));

  const auto without_spec_qos = adm::evaluate_deploy({provider, consumer});
  EXPECT_FALSE(has_verdict(without_spec_qos, adm::QOS_PAIR_INCOMPATIBLE));
  EXPECT_TRUE(has_verdict(without_spec_qos, adm::ACCEPTED));
}

TEST(admission_rule, unversioned_required_entry_never_yields_a_version_verdict)
{
  // has_version == false must suppress MAJOR_MISMATCH / MINOR_MISMATCH / NO_PROVIDER entirely --
  // version bounds simply do not apply -- even though the provider's MAJOR (9) would mismatch under
  // a versioned check.
  adm::InterfaceManifest prov_m;
  prov_m.node_name = "/n1";
  adm::ProvidedInterface p;
  p.interface_name = "/t";
  p.resolved_name = "/t";
  p.major = 9;
  p.has_qos = true;
  p.qos = {"reliable", "volatile", 1};
  prov_m.provided.push_back(p);

  adm::InterfaceManifest cons_m;
  cons_m.node_name = "/n2";
  adm::RequiredInterface r;
  r.interface_name = "/t";
  r.resolved_name = "/t";
  r.has_version = false;
  r.has_qos = true;
  r.qos = {"reliable", "volatile", 1};
  cons_m.required.push_back(r);

  const auto verdicts = adm::evaluate_deploy({prov_m, cons_m});
  ASSERT_FALSE(verdicts.empty());
  EXPECT_FALSE(has_verdict(verdicts, adm::MAJOR_MISMATCH));
  EXPECT_FALSE(has_verdict(verdicts, adm::MINOR_MISMATCH));
  EXPECT_FALSE(has_verdict(verdicts, adm::NO_PROVIDER));
  EXPECT_TRUE(has_verdict(verdicts, adm::ACCEPTED));
}

TEST(admission_rule, unversioned_required_entry_with_no_provider_is_no_provider)
{
  // NO_PROVIDER is a completeness verdict, not a version verdict: the deploy image set is known
  // complete up front, so a required entry with no provider anywhere must be reported regardless
  // of whether it makes a version claim at all. This mirrors the pre-v2 behavior, where every
  // required entry got this check.
  adm::InterfaceManifest cons_m;
  cons_m.node_name = "/n2";
  adm::RequiredInterface r;
  r.interface_name = "/nowhere";
  r.resolved_name = "/nowhere";
  r.has_version = false;
  cons_m.required.push_back(r);

  const auto verdicts = adm::evaluate_deploy({cons_m});
  ASSERT_EQ(verdicts.size(), 1u);
  EXPECT_EQ(verdicts[0].code, adm::NO_PROVIDER);
}

TEST(admission_rule, old_signature_delegates_to_an_empty_spec_qos_map)
{
  auto provider = make_manifest_with_provided("/n1", "/vendor_if", {"reliable", "volatile", 1});
  auto consumer = make_manifest_with_required("/n2", "/vendor_if", {"best_effort", "volatile", 1});
  const auto one_arg = adm::evaluate_deploy({provider, consumer});
  const auto two_arg = adm::evaluate_deploy({provider, consumer}, {});
  ASSERT_EQ(one_arg.size(), two_arg.size());
  for (std::size_t i = 0; i < one_arg.size(); ++i) {
    EXPECT_EQ(one_arg[i].code, two_arg[i].code);
  }
}

// --- Spec QoS verdicts are per-ENDPOINT, checked independently of stage-1 pairing ---

TEST(admission_rule, spec_qos_provider_violation_is_reported_even_without_a_qos_carrying_consumer)
{
  // A provider deviating from its spec's declared QoS must be flagged even though the paired
  // consumer's required entry does not carry qos at all -- conformance is a property of the single
  // endpoint and its spec, not of a counterparty's own qos.
  auto prov = make_manifest_with_provided("/n1", "/t", {"best_effort", "volatile", 1});
  auto cons = make_manifest_with_required_no_qos("/n2", "/t");
  const std::map<std::string, adm::QosRecord> spec_qos{{"/t", {"reliable", "volatile", 1}}};
  const auto verdicts = adm::evaluate_deploy({prov, cons}, spec_qos);
  EXPECT_TRUE(has_verdict(verdicts, adm::QOS_SPEC_MISMATCH));
}

TEST(admission_rule, spec_qos_provider_violation_is_reported_even_with_no_consumer_at_all)
{
  // A publisher-only image with no consumer anywhere in the deploy set is exactly as checkable
  // against its spec as a matched pair, and it is the common deployment shape, since fragments are
  // per package.
  auto prov = make_manifest_with_provided("/n1", "/t", {"best_effort", "volatile", 1});
  const std::map<std::string, adm::QosRecord> spec_qos{{"/t", {"reliable", "volatile", 1}}};
  const auto verdicts = adm::evaluate_deploy({prov}, spec_qos);
  EXPECT_TRUE(has_verdict(verdicts, adm::QOS_SPEC_MISMATCH));
}

TEST(admission_rule, spec_qos_consumer_violation_is_reported_even_without_a_qos_carrying_provider)
{
  // A consumer deviating from its spec's declared QoS must be flagged even though the paired
  // provider's provided entry does not carry qos at all.
  auto prov = make_manifest_with_provided_no_qos("/n1", "/t");
  auto cons = make_manifest_with_required("/n2", "/t", {"reliable", "transient_local", 1});
  const std::map<std::string, adm::QosRecord> spec_qos{{"/t", {"reliable", "volatile", 1}}};
  const auto verdicts = adm::evaluate_deploy({prov, cons}, spec_qos);
  EXPECT_TRUE(has_verdict(verdicts, adm::QOS_SPEC_MISMATCH));
}

TEST(admission_rule, every_provider_is_checked_against_the_spec_qos_independently)
{
  // Two providers of the same interface: stage-1 pairing matches the consumer to only one of them
  // (the lowest node name), but the per-endpoint check must not stop there -- the second,
  // unmatched provider must still be flagged, so a verdict does not depend on node-name ordering.
  auto ok = make_manifest_with_provided("/a_ok", "/t", {"reliable", "volatile", 1});
  auto bad = make_manifest_with_provided("/z_bad", "/t", {"best_effort", "volatile", 1});
  auto cons = make_manifest_with_required("/n2", "/t", {"reliable", "volatile", 1});
  const std::map<std::string, adm::QosRecord> spec_qos{{"/t", {"reliable", "volatile", 1}}};
  const auto verdicts = adm::evaluate_deploy({ok, bad, cons}, spec_qos);
  bool z_bad_flagged = false;
  for (const auto & v : verdicts) {
    if (v.code == adm::QOS_SPEC_MISMATCH && v.provider_node == "/z_bad") {
      z_bad_flagged = true;
    }
  }
  EXPECT_TRUE(z_bad_flagged);
}

// --- has_version must be honored on the provider side too, at both triggers ---

TEST(admission_rule, unversioned_provider_is_no_provider_against_a_narrow_accept_window)
{
  // An unversioned provider makes no version claim; a consumer requiring MAJOR in [1, 2] must
  // not blame it with MAJOR_MISMATCH -- that would blame an entry that made no version claim.
  // Treated like "no version-checkable provider exists", it is NO_PROVIDER instead.
  adm::InterfaceManifest prov_m;
  prov_m.node_name = "/n1";
  adm::ProvidedInterface p;
  p.interface_name = "/t";
  p.resolved_name = "/t";
  p.has_version = false;
  prov_m.provided.push_back(p);

  adm::InterfaceManifest cons_m;
  cons_m.node_name = "/n2";
  adm::RequiredInterface r;
  r.interface_name = "/t";
  r.resolved_name = "/t";
  r.accept_major_min = 1;
  r.accept_major_max = 2;
  cons_m.required.push_back(r);

  const auto verdicts = adm::evaluate_deploy({prov_m, cons_m});
  ASSERT_EQ(verdicts.size(), 1u);
  EXPECT_EQ(verdicts[0].code, adm::NO_PROVIDER);
}

TEST(admission_rule, unversioned_provider_does_not_fail_open_against_an_all_zero_accept_window)
{
  // The same unversioned provider against a consumer whose accept window is [0, 0]: the
  // provider's absent version defaults to major=0, which would satisfy [0, 0] if that default
  // were compared directly. It must not -- that is exactly the fail-open the parse-layer
  // partial-key rule exists to prevent, reintroduced at the verdict layer.
  adm::InterfaceManifest prov_m;
  prov_m.node_name = "/n1";
  adm::ProvidedInterface p;
  p.interface_name = "/t";
  p.resolved_name = "/t";
  p.has_version = false;
  prov_m.provided.push_back(p);

  adm::InterfaceManifest cons_m;
  cons_m.node_name = "/n2";
  adm::RequiredInterface r;
  r.interface_name = "/t";
  r.resolved_name = "/t";
  r.accept_major_min = 0;
  r.accept_major_max = 0;
  cons_m.required.push_back(r);

  const auto verdicts = adm::evaluate_deploy({prov_m, cons_m});
  ASSERT_EQ(verdicts.size(), 1u);
  EXPECT_EQ(verdicts[0].code, adm::NO_PROVIDER);
}

TEST(
  admission_rule, runtime_evaluate_never_yields_a_version_verdict_for_an_unversioned_required_entry)
{
  // The same has_version contract must hold at the runtime trigger too ("one rule, two
  // triggers"): an unversioned required entry against a MAJOR-9 provider must be ACCEPTED, not
  // MAJOR_MISMATCH.
  adm::InterfaceManifest prov_m;
  prov_m.node_name = "/n1";
  adm::ProvidedInterface p;
  p.interface_name = "/t";
  p.resolved_name = "/t";
  p.major = 9;
  prov_m.provided.push_back(p);

  adm::InterfaceManifest cons_m;
  cons_m.node_name = "/n2";
  adm::RequiredInterface r;
  r.interface_name = "/t";
  r.resolved_name = "/t";
  r.has_version = false;
  cons_m.required.push_back(r);

  const auto verdicts = adm::evaluate({prov_m, cons_m});
  ASSERT_EQ(verdicts.size(), 1u);
  EXPECT_EQ(verdicts[0].code, adm::ACCEPTED);
}

TEST(admission_rule, runtime_evaluate_unversioned_required_entry_still_catches_a_remap_false_accept)
{
  // has_version == false suppresses a VERSION verdict, but TOPIC_MISMATCH is a wiring verdict, not
  // a version one -- it must still fire when the only provider's wire topic is disjoint from the
  // one this consumer is actually wired to, exactly as it would for a versioned required entry.
  adm::InterfaceManifest prov_m;
  prov_m.node_name = "/n1";
  adm::ProvidedInterface p;
  p.interface_name = "/t";
  p.resolved_name = "/t/remapped_elsewhere";
  prov_m.provided.push_back(p);

  adm::InterfaceManifest cons_m;
  cons_m.node_name = "/n2";
  adm::RequiredInterface r;
  r.interface_name = "/t";
  r.resolved_name = "/t";
  r.has_version = false;
  cons_m.required.push_back(r);

  const auto verdicts = adm::evaluate({prov_m, cons_m});
  ASSERT_EQ(verdicts.size(), 1u);
  EXPECT_EQ(verdicts[0].code, adm::TOPIC_MISMATCH);
  EXPECT_EQ(verdicts[0].provider_node, "/n1");
}

TEST(admission_rule, runtime_evaluate_unversioned_required_entry_names_the_correctly_wired_provider)
{
  // Two providers of the same interface: /a_wrong is remapped away, /z_wired is the one actually
  // wired to this consumer. Node-name ordering must not decide the verdict -- the wired provider
  // must be the one named ACCEPTED even though it does not sort first alphabetically.
  adm::InterfaceManifest wrong_m;
  wrong_m.node_name = "/a_wrong";
  adm::ProvidedInterface wrong_p;
  wrong_p.interface_name = "/t";
  wrong_p.resolved_name = "/t/remapped_elsewhere";
  wrong_m.provided.push_back(wrong_p);

  adm::InterfaceManifest wired_m;
  wired_m.node_name = "/z_wired";
  adm::ProvidedInterface wired_p;
  wired_p.interface_name = "/t";
  wired_p.resolved_name = "/t";
  wired_m.provided.push_back(wired_p);

  adm::InterfaceManifest cons_m;
  cons_m.node_name = "/n2";
  adm::RequiredInterface r;
  r.interface_name = "/t";
  r.resolved_name = "/t";
  r.has_version = false;
  cons_m.required.push_back(r);

  const auto verdicts = adm::evaluate({wrong_m, wired_m, cons_m});
  ASSERT_EQ(verdicts.size(), 1u);
  EXPECT_EQ(verdicts[0].code, adm::ACCEPTED);
  EXPECT_EQ(verdicts[0].provider_node, "/z_wired");
}

// --- Fail-closed rank behavior (already correct; this is coverage only) ---

TEST(admission_rule, reliability_rank_is_fail_closed_for_an_out_of_vocabulary_string)
{
  EXPECT_EQ(adm::reliability_rank("garbage"), -1);
}

TEST(admission_rule, durability_rank_is_fail_closed_for_an_out_of_vocabulary_string)
{
  EXPECT_EQ(adm::durability_rank("garbage"), -1);
}

TEST(admission_rule, qos_is_at_least_rejects_an_out_of_vocabulary_policy_string)
{
  // A garbage policy ranks -1 on both sides of the comparison, so it must never compare as "at
  // least" anything, even against an equally-garbage counterpart.
  EXPECT_FALSE(adm::qos_is_at_least({"garbage", "volatile", 1}, {"best_effort", "volatile", 1}));
  EXPECT_FALSE(adm::qos_is_at_least({"reliable", "volatile", 1}, {"garbage", "volatile", 1}));
}
