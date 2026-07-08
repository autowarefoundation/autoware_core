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
#include "autoware/component_interface_specs/vehicle.hpp"

#include <rosidl_runtime_cpp/traits.hpp>

#include <cstddef>
#include <fstream>
#include <iostream>
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

struct Entry
{
  std::string domain;
  std::string interface;
  std::string type;
  std::string kind;  // "topic" | "service"
  cis::Version version;
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
  } else {
    e.kind = "service";
    e.type = rosidl_generator_traits::name<typename Spec::Service>();
  }
  return e;
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

  std::ofstream os(argv[1]);
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
    os << "      \"version\": \"" << ver_str(e.version) << "\"\n";
    os << "    }" << (i + 1 < entries.size() ? "," : "") << "\n";
  }
  os << "  ]\n";
  os << "}\n";
  return 0;
}
