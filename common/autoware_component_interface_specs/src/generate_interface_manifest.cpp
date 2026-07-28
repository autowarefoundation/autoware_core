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

// Deterministic, hand-emitted JSON walker over the per-domain `Specs` tuples.
// The emitted layout is byte-identical to the committed `interface_manifest.json`
// (and to what Prettier produces for it), so a rebuild plus diff is the freshness
// check consumed by later CI work.

#include "autoware/component_interface_specs/concepts.hpp"
#include "autoware/component_interface_specs/control.hpp"
#include "autoware/component_interface_specs/localization.hpp"
#include "autoware/component_interface_specs/map.hpp"
#include "autoware/component_interface_specs/perception.hpp"
#include "autoware/component_interface_specs/planning.hpp"
#include "autoware/component_interface_specs/system.hpp"
#include "autoware/component_interface_specs/utils.hpp"
#include "autoware/component_interface_specs/vehicle.hpp"

#include <rosidl_runtime_cpp/traits.hpp>

#include <rmw/qos_profiles.h>

#include <cstddef>
#include <fstream>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <tuple>
#include <type_traits>
#include <utility>
#include <vector>

namespace cis = autoware::component_interface_specs;

namespace
{

template <class T>
concept HasMessage = requires { typename T::Message; };

/// The QoS an endpoint of this interface is created with: what `get_qos<Spec>()` returns
/// for a topic spec, and what `get_service_qos()` returns for a service spec. Reliability
/// and durability are the two axes ROS 2 checks for endpoint compatibility, so they are
/// what an admission gate reads back out of the manifest.
struct Qos
{
  std::size_t depth;
  rmw_qos_reliability_policy_t reliability;
  rmw_qos_durability_policy_t durability;
};

struct Entry
{
  std::string domain;
  std::string interface;
  std::string type;
  std::string kind;  // "topic" | "service"
  cis::Version version;
  Qos qos;
};

template <class Spec>
Entry make_entry(const std::string & domain, cis::Version version)
{
  Entry e;
  e.domain = domain;
  e.interface = Spec::name;
  e.version = version;
  if constexpr (HasMessage<Spec>) {
    e.kind = "topic";
    e.type = rosidl_generator_traits::name<typename Spec::Message>();
    e.qos = Qos{Spec::depth, Spec::reliability, Spec::durability};
  } else {
    e.kind = "service";
    e.type = rosidl_generator_traits::name<typename Spec::Service>();
    e.qos =
      Qos{cis::service_qos::depth, cis::service_qos::reliability, cis::service_qos::durability};
  }
  return e;
}

/// Unmapped policies throw rather than emit a placeholder: a spec that reaches for a policy
/// the manifest cannot name must be an explicit decision, not a silently degraded entry.
const char * to_string(rmw_qos_reliability_policy_t policy)
{
  switch (policy) {
    case RMW_QOS_POLICY_RELIABILITY_RELIABLE:
      return "reliable";
    case RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT:
      return "best_effort";
    default:
      throw std::invalid_argument("spec declares a reliability policy the manifest cannot name");
  }
}

const char * to_string(rmw_qos_durability_policy_t policy)
{
  switch (policy) {
    case RMW_QOS_POLICY_DURABILITY_VOLATILE:
      return "volatile";
    case RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL:
      return "transient_local";
    default:
      throw std::invalid_argument("spec declares a durability policy the manifest cannot name");
  }
}

template <class Tuple, class F, std::size_t... Is>
void for_each_type(F && f, std::index_sequence<Is...>)
{
  (f(std::type_identity<std::tuple_element_t<Is, Tuple>>{}), ...);
}

template <class Specs>
void collect(const std::string & domain, cis::Version version, std::vector<Entry> * out)
{
  for_each_type<Specs>(
    [&](auto tag) {
      using Spec = typename decltype(tag)::type;
      out->push_back(make_entry<Spec>(domain, version));
    },
    std::make_index_sequence<std::tuple_size_v<Specs>>{});
}

std::string ver_str(cis::Version v)
{
  return std::to_string(v.major) + "." + std::to_string(v.minor) + "." + std::to_string(v.patch);
}

}  // namespace

int main(int argc, char ** argv)
{
  if (argc < 2) {
    std::cerr << "usage: generate_interface_manifest <out.json>\n";
    return 1;
  }

  std::vector<Entry> entries;
  collect<cis::control::Specs>("control", cis::control::version, &entries);
  collect<cis::localization::Specs>("localization", cis::localization::version, &entries);
  collect<cis::map::Specs>("map", cis::map::version, &entries);
  collect<cis::perception::Specs>("perception", cis::perception::version, &entries);
  collect<cis::planning::Specs>("planning", cis::planning::version, &entries);
  collect<cis::system::Specs>("system", cis::system::version, &entries);
  collect<cis::vehicle::Specs>("vehicle", cis::vehicle::version, &entries);

  // Rendered whole before the output file is opened, so a spec the emitter cannot name
  // leaves the committed manifest untouched instead of truncated.
  std::ostringstream os;
  try {
    os << "{\n";
    os << "  \"owner\": \"" << cis::owner << "\",\n";
    os << "  \"interfaces\": [\n";
    for (std::size_t i = 0; i < entries.size(); ++i) {
      const auto & e = entries[i];
      os << "    {\n";
      os << "      \"domain\": \"" << e.domain << "\",\n";
      os << "      \"interface\": \"" << e.interface << "\",\n";
      os << "      \"type\": \"" << e.type << "\",\n";
      os << "      \"kind\": \"" << e.kind << "\",\n";
      os << "      \"version\": \"" << ver_str(e.version) << "\",\n";
      os << "      \"qos\": {\n";
      // Both get_qos<Spec>() and get_service_qos() build an rclcpp::QoS from a depth alone,
      // which is KEEP_LAST. No spec can currently ask for KEEP_ALL.
      os << "        \"history\": \"keep_last\",\n";
      os << "        \"depth\": " << e.qos.depth << ",\n";
      os << "        \"reliability\": \"" << to_string(e.qos.reliability) << "\",\n";
      os << "        \"durability\": \"" << to_string(e.qos.durability) << "\"\n";
      os << "      }\n";
      os << "    }" << (i + 1 < entries.size() ? "," : "") << "\n";
    }
    os << "  ]\n";
    os << "}\n";
  } catch (const std::exception & error) {
    std::cerr << "generate_interface_manifest: " << error.what() << "\n";
    return 1;
  }

  std::ofstream out(argv[1]);
  if (!out) {
    std::cerr << "generate_interface_manifest: cannot write " << argv[1] << "\n";
    return 1;
  }
  out << os.str();
  return 0;
}
