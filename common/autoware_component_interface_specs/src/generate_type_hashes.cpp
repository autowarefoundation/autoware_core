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

// Emits interface_type_hashes.jazzy.lock: the committed RIHS01 type hash of every registered
// message and service type. The hash is a deterministic pure function of the type description
// (transitively including nested .msg includes), so a change to a type definition in any repo
// shows up as a changed hash. This complements the same-PR version-bump gate: it catches the
// case a version bump was forgotten, and it is the only mechanism for the types owned by other
// repositories. Producer-side / CI only; the freshness test drives it.

// The type-hash API is Iron+. Guard on its header so the tool still compiles as a no-op on
// Humble (gcc-11, C++20-capable, header absent), where the Jazzy-only freshness test skips.
#if __has_include(<rosidl_runtime_c/type_hash.h>)

#include "autoware/component_interface_specs/control.hpp"
#include "autoware/component_interface_specs/localization.hpp"
#include "autoware/component_interface_specs/map.hpp"
#include "autoware/component_interface_specs/perception.hpp"
#include "autoware/component_interface_specs/planning.hpp"
#include "autoware/component_interface_specs/system.hpp"
#include "autoware/component_interface_specs/vehicle.hpp"

#include <rosidl_runtime_cpp/traits.hpp>
#include <rosidl_typesupport_cpp/message_type_support.hpp>
#include <rosidl_typesupport_cpp/service_type_support.hpp>

#include <rcutils/allocator.h>
#include <rcutils/types/rcutils_ret.h>
#include <rosidl_runtime_c/type_hash.h>

#include <algorithm>
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

// cpplint's include-what-you-use tracker resets its per-branch include list on the #else
// below (it does not evaluate preprocessor conditions), so it cannot see the STL headers
// included above and misreports them as missing for the usages in this #if branch.
// NOLINTBEGIN(build/include_what_you_use)

namespace cis = autoware::component_interface_specs;

namespace
{

template <class T>
concept HasMessage = requires { typename T::Message; };

std::string stringify(const rosidl_type_hash_t * hash)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  char * out = nullptr;
  if (rosidl_stringify_type_hash(hash, allocator, &out) != RCUTILS_RET_OK || out == nullptr) {
    throw std::runtime_error("rosidl_stringify_type_hash failed");
  }
  std::string result(out);
  allocator.deallocate(out, allocator.state);
  return result;
}

template <class Message>
std::string message_hash()
{
  const rosidl_message_type_support_t * ts =
    rosidl_typesupport_cpp::get_message_type_support_handle<Message>();
  return stringify(ts->get_type_hash_func(ts));
}

template <class Service>
std::string service_hash()
{
  const rosidl_service_type_support_t * ts =
    rosidl_typesupport_cpp::get_service_type_support_handle<Service>();
  return stringify(ts->get_type_hash_func(ts));
}

struct Row
{
  std::string type;
  std::string kind;  // "topic" | "service"
  std::string hash;
};

template <class Spec>
Row make_row()
{
  if constexpr (HasMessage<Spec>) {
    using Message = typename Spec::Message;
    return Row{rosidl_generator_traits::name<Message>(), "topic", message_hash<Message>()};
  } else {
    using Service = typename Spec::Service;
    return Row{rosidl_generator_traits::name<Service>(), "service", service_hash<Service>()};
  }
}

template <class Tuple, class F, std::size_t... Is>
void for_each_type(F && f, std::index_sequence<Is...>)
{
  (f(std::type_identity<std::tuple_element_t<Is, Tuple>>{}), ...);
}

template <class Specs>
void collect(std::vector<Row> * out)
{
  for_each_type<Specs>(
    [&](auto tag) {
      using Spec = typename decltype(tag)::type;
      out->push_back(make_row<Spec>());
    },
    std::make_index_sequence<std::tuple_size_v<Specs>>{});
}

}  // namespace

int main(int argc, char ** argv)
{
  if (argc < 2) {
    std::cerr << "usage: generate_type_hashes <out.lock>\n";
    return 1;
  }

  std::vector<Row> rows;
  try {
    // The domain list mirrors generate_interface_manifest.cpp. test_type_hashes.cpp's
    // covers_every_manifest_type test fails loudly if the two ever diverge, so a domain
    // cannot be silently dropped from the lockfile.
    collect<cis::control::Specs>(&rows);
    collect<cis::localization::Specs>(&rows);
    collect<cis::map::Specs>(&rows);
    collect<cis::perception::Specs>(&rows);
    collect<cis::planning::Specs>(&rows);
    collect<cis::system::Specs>(&rows);
    collect<cis::vehicle::Specs>(&rows);
  } catch (const std::exception & error) {
    std::cerr << "generate_type_hashes: " << error.what() << "\n";
    return 1;
  }

  // The hash is a property of the type, so the same type used by two interfaces yields one row.
  // Sort by (type, kind) for a stable, merge-friendly, registration-order-independent file.
  std::sort(rows.begin(), rows.end(), [](const Row & a, const Row & b) {
    return std::tie(a.type, a.kind) < std::tie(b.type, b.kind);
  });
  rows.erase(
    std::unique(
      rows.begin(), rows.end(),
      [](const Row & a, const Row & b) { return a.type == b.type && a.kind == b.kind; }),
    rows.end());

  std::ostringstream os;
  os << "# RIHS01 type-hash lockfile for autoware_component_interface_specs (Jazzy only).\n";
  os << "# Generated by generate_type_hashes; do not edit by hand. See README.md.\n";
  os << "# <rihs01-hash>  <type>  <kind>\n";
  for (const auto & row : rows) {
    os << row.hash << "  " << row.type << "  " << row.kind << "\n";
  }

  std::ofstream out(argv[1]);
  if (!out) {
    std::cerr << "generate_type_hashes: cannot write " << argv[1] << "\n";
    return 1;
  }
  out << os.str();
  return 0;
}

// NOLINTEND

#else  // type-hash API unavailable (e.g. Humble)

#include <iostream>

// The RIHS01 type-hash API is Iron+. On older distros this tool is intentionally a no-op so the
// BUILD_TESTING compile still succeeds; the Jazzy-only freshness test skips there, so nothing
// consumes an output file. Writing nothing (not even an empty file) is fine.
int main(int argc, char ** argv)
{
  (void)argc;
  (void)argv;
  std::cerr << "generate_type_hashes: RIHS01 type-hash API unavailable on this distro; no-op\n";
  return 0;
}

#endif
