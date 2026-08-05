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

#include <gtest/gtest.h>

#include <stdexcept>
#include <string>

namespace adm = autoware::component_interface_admission;

namespace
{
adm::InterfaceManifest sample_manifest()
{
  adm::InterfaceManifest m;
  m.owner = "autowarefoundation";
  m.node_name = "/perception/detection";

  adm::ProvidedInterface p0;
  p0.ns = "perception";
  p0.interface_name = "/perception/object_recognition/objects";
  p0.resolved_name = "/perception/object_recognition/objects";
  p0.type_name = "autoware_perception_msgs/msg/PredictedObjects";
  p0.major = 2;
  p0.minor = 1;
  p0.patch = 3;
  m.provided.push_back(p0);

  adm::ProvidedInterface p1;
  p1.ns = "perception";
  p1.interface_name = "/perception/traffic_light/state";
  p1.resolved_name = "/remapped/traffic_light";
  p1.type_name = "autoware_perception_msgs/msg/TrafficLightGroupArray";
  p1.major = 1;
  p1.minor = 0;
  p1.patch = 0;
  m.provided.push_back(p1);

  adm::RequiredInterface r0;
  r0.ns = "map";
  r0.interface_name = "/map/vector_map";
  r0.resolved_name = "/map/vector_map";
  r0.type_name = "autoware_map_msgs/msg/LaneletMapBin";
  r0.accept_major_min = 1;
  r0.accept_major_max = 2;
  r0.min_minor = 4;
  m.required.push_back(r0);

  return m;
}
}  // namespace

TEST(ManifestJson, round_trip_preserves_all_fields)
{
  const auto original = sample_manifest();
  const auto parsed = adm::from_json(adm::to_json(original));

  EXPECT_EQ(parsed.owner, "autowarefoundation");
  EXPECT_EQ(parsed.node_name, "/perception/detection");

  ASSERT_EQ(parsed.provided.size(), 2u);
  EXPECT_EQ(parsed.provided[0].ns, "perception");
  EXPECT_EQ(parsed.provided[0].interface_name, "/perception/object_recognition/objects");
  EXPECT_EQ(parsed.provided[0].resolved_name, "/perception/object_recognition/objects");
  EXPECT_EQ(parsed.provided[0].type_name, "autoware_perception_msgs/msg/PredictedObjects");
  EXPECT_EQ(parsed.provided[0].major, 2u);
  EXPECT_EQ(parsed.provided[0].minor, 1u);
  EXPECT_EQ(parsed.provided[0].patch, 3u);
  EXPECT_EQ(parsed.provided[1].interface_name, "/perception/traffic_light/state");
  EXPECT_EQ(parsed.provided[1].resolved_name, "/remapped/traffic_light");
  EXPECT_EQ(parsed.provided[1].major, 1u);

  ASSERT_EQ(parsed.required.size(), 1u);
  EXPECT_EQ(parsed.required[0].ns, "map");
  EXPECT_EQ(parsed.required[0].interface_name, "/map/vector_map");
  EXPECT_EQ(parsed.required[0].type_name, "autoware_map_msgs/msg/LaneletMapBin");
  EXPECT_EQ(parsed.required[0].accept_major_min, 1u);
  EXPECT_EQ(parsed.required[0].accept_major_max, 2u);
  EXPECT_EQ(parsed.required[0].min_minor, 4u);
}

TEST(ManifestJson, to_json_emits_documented_keys)
{
  const auto json = adm::to_json(sample_manifest());
  EXPECT_NE(json.find("\"owner\":\"autowarefoundation\""), std::string::npos);
  EXPECT_NE(
    json.find("\"interface_name\":\"/perception/object_recognition/objects\""), std::string::npos);
  EXPECT_NE(json.find("\"major\":2"), std::string::npos);
  EXPECT_NE(json.find("\"accept_major_max\":2"), std::string::npos);
}

TEST(ManifestJson, resolved_name_defaults_to_interface_name_when_absent)
{
  const auto m = adm::from_json(
    R"({"node_name":"/n","provided":[{"interface_name":"/a","major":1,"minor":0,"patch":0}]})");
  ASSERT_EQ(m.provided.size(), 1u);
  EXPECT_EQ(m.provided[0].resolved_name, "/a");
}

TEST(ManifestJson, absent_arrays_parse_as_empty)
{
  const auto m = adm::from_json(R"({"owner":"autowarefoundation","node_name":"/n"})");
  EXPECT_TRUE(m.provided.empty());
  EXPECT_TRUE(m.required.empty());
  EXPECT_EQ(m.owner, "autowarefoundation");
}

TEST(ManifestJson, malformed_json_throws_runtime_error)
{
  EXPECT_THROW(adm::from_json("{ this is not valid json"), std::runtime_error);
  EXPECT_THROW(adm::from_json(""), std::runtime_error);
  EXPECT_THROW(adm::from_json("[1, 2, 3]"), std::runtime_error);  // root not an object
}

TEST(ManifestJson, missing_required_key_throws_runtime_error)
{
  // A provided entry missing interface_name (the matching key).
  EXPECT_THROW(
    adm::from_json(R"({"node_name":"/n","provided":[{"major":1,"minor":0,"patch":0}]})"),
    std::runtime_error);
  // A provided entry missing a version field.
  EXPECT_THROW(
    adm::from_json(R"({"node_name":"/n","provided":[{"interface_name":"/a","minor":0}]})"),
    std::runtime_error);
}

TEST(ManifestJson, wrong_value_type_throws_runtime_error)
{
  // major must be an integer, not a string.
  EXPECT_THROW(
    adm::from_json(
      R"({"node_name":"/n","provided":[{"interface_name":"/a","major":"x","minor":0,"patch":0}]})"),
    std::runtime_error);
  // provided must be an array.
  EXPECT_THROW(adm::from_json(R"({"node_name":"/n","provided":{}})"), std::runtime_error);
}

// --- v2 schema: QoS-carrying manifests and conditional version keys ---

TEST(manifest_json, v2_qos_and_omitted_version_parse)
{
  const auto m = adm::from_json(R"({
    "node_name": "/n",
    "provided": [{"interface_name": "/t",
                  "qos": {"reliability": "reliable", "durability": "volatile", "depth": 1}}]
  })");
  ASSERT_EQ(m.provided.size(), 1u);
  EXPECT_FALSE(m.provided[0].has_version);
  ASSERT_TRUE(m.provided[0].has_qos);
  EXPECT_EQ(m.provided[0].qos.reliability, "reliable");
  EXPECT_EQ(m.provided[0].qos.durability, "volatile");
  EXPECT_EQ(m.provided[0].qos.depth, 1);
}

TEST(manifest_json, v1_doc_parses_with_has_qos_false)
{
  const auto m = adm::from_json(
    R"({"node_name":"/n","provided":[{"interface_name":"/a","major":1,"minor":0,"patch":0}]})");
  ASSERT_EQ(m.provided.size(), 1u);
  EXPECT_TRUE(m.provided[0].has_version);
  EXPECT_FALSE(m.provided[0].has_qos);
}

TEST(manifest_json, required_entry_version_keys_omitted_parses_unversioned)
{
  const auto m = adm::from_json(R"({
    "node_name": "/n",
    "required": [{"interface_name": "/s",
                  "qos": {"reliability": "reliable", "durability": "volatile", "depth": 10}}]
  })");
  ASSERT_EQ(m.required.size(), 1u);
  EXPECT_FALSE(m.required[0].has_version);
  ASSERT_TRUE(m.required[0].has_qos);
  EXPECT_EQ(m.required[0].qos.depth, 10);
}

TEST(manifest_json, partial_version_keys_throws)
{
  // Only "major" present out of the major/minor/patch group -- a malformed partial declaration.
  EXPECT_THROW(
    adm::from_json(R"({"node_name":"/n","provided":[{"interface_name":"/a","major":1}]})"),
    std::runtime_error);
  EXPECT_THROW(
    adm::from_json(
      R"({"node_name":"/n",
          "required":[{"interface_name":"/s","accept_major_min":1,"accept_major_max":1}]})"),
    std::runtime_error);
}

TEST(manifest_json, unknown_policy_string_throws)
{
  EXPECT_THROW(
    adm::from_json(R"({
      "node_name": "/n",
      "provided": [{"interface_name": "/t",
                    "qos": {"reliability": "best-effort", "durability": "volatile", "depth": 1}}]
    })"),
    std::runtime_error);
  EXPECT_THROW(
    adm::from_json(R"({
      "node_name": "/n",
      "provided": [{"interface_name": "/t",
                    "qos": {"reliability": "reliable", "durability": "persistent", "depth": 1}}]
    })"),
    std::runtime_error);
}

TEST(manifest_json, manifests_from_json_object_root_yields_one)
{
  const auto manifests = adm::manifests_from_json(R"({"node_name":"/n"})");
  ASSERT_EQ(manifests.size(), 1u);
  EXPECT_EQ(manifests[0].node_name, "/n");
}

TEST(manifest_json, manifests_from_json_array_root_yields_n)
{
  const auto manifests = adm::manifests_from_json(R"([{"node_name":"/n1"}, {"node_name":"/n2"}])");
  ASSERT_EQ(manifests.size(), 2u);
  EXPECT_EQ(manifests[0].node_name, "/n1");
  EXPECT_EQ(manifests[1].node_name, "/n2");
}

TEST(manifest_json, manifests_from_json_other_root_throws)
{
  EXPECT_THROW(adm::manifests_from_json("42"), std::runtime_error);
  EXPECT_THROW(adm::manifests_from_json(R"("a string")"), std::runtime_error);
}

TEST(manifest_json, spec_qos_is_parsed_from_the_spec_manifest)
{
  // The shape autoware_component_interface_specs actually commits as interface_manifest.json.
  const char * spec = R"({
    "owner": "autowarefoundation",
    "interfaces": [{"domain": "control", "interface": "/c", "type": "pkg/msg/C",
                    "kind": "topic", "version": "0.1.0",
                    "qos": {"history": "keep_last", "depth": 1,
                            "reliability": "reliable", "durability": "volatile"}}]
  })";
  const auto spec_qos = adm::spec_qos_from_json(spec);
  ASSERT_EQ(spec_qos.count("/c"), 1u);
  EXPECT_EQ(spec_qos.at("/c").reliability, "reliable");
  EXPECT_EQ(spec_qos.at("/c").durability, "volatile");
  EXPECT_EQ(spec_qos.at("/c").depth, 1);
}

TEST(manifest_json, spec_qos_rejects_a_document_that_is_not_a_spec_manifest)
{
  // Accepting a document with no `interfaces` at all would hand back an empty QoS table, which
  // silently disables the spec-conformance verdict for every endpoint. Fail closed instead.
  EXPECT_THROW(adm::spec_qos_from_json(R"({"owner": "x"})"), std::runtime_error);
  EXPECT_THROW(adm::spec_qos_from_json(R"({"interfaces": {}})"), std::runtime_error);
  EXPECT_THROW(adm::spec_qos_from_json(R"([])"), std::runtime_error);
  EXPECT_THROW(adm::spec_qos_from_json("not json"), std::runtime_error);
}

TEST(manifest_json, spec_qos_rejects_an_entry_without_qos)
{
  const char * missing_qos = R"({"interfaces": [{"interface": "/c"}]})";
  EXPECT_THROW(adm::spec_qos_from_json(missing_qos), std::runtime_error);
}

TEST(manifest_json, spec_qos_accepts_an_empty_interface_list)
{
  // An empty list is a well-formed spec manifest that simply declares nothing; unlike a missing
  // `interfaces` key, it is a positive statement rather than a wrong document.
  EXPECT_TRUE(adm::spec_qos_from_json(R"({"interfaces": []})").empty());
}
