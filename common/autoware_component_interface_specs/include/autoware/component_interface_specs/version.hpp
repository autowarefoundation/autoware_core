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

#ifndef AUTOWARE__COMPONENT_INTERFACE_SPECS__VERSION_HPP_
#define AUTOWARE__COMPONENT_INTERFACE_SPECS__VERSION_HPP_

#include <cstdint>
#include <tuple>

namespace autoware::component_interface_specs
{

/// Per-domain semantic version. 0.x is unstable while the standard is stabilizing.
struct Version
{
  std::uint16_t major;
  std::uint16_t minor;
  std::uint16_t patch;
};

constexpr bool operator==(const Version & a, const Version & b)
{
  return a.major == b.major && a.minor == b.minor && a.patch == b.patch;
}

constexpr bool operator!=(const Version & a, const Version & b)
{
  return !(a == b);
}

/// Inclusive MAJOR acceptance window declared by a required (consumer) side.
struct accept_major
{
  std::uint16_t lo;
  std::uint16_t hi;
};

/// The reference encoding of the MAJOR compatibility axis: a provided interface
/// satisfies a required MAJOR iff equal. This is the definition consumers compile
/// against; it is not itself the deploy-time gate. The gate lives in
/// autoware_component_interface_admission, a no-dependency leaf package that cannot
/// include this header and therefore restates the same relation. Change one and the
/// other must follow.
constexpr bool is_compatible(const Version & provided, std::uint16_t required_major)
{
  return provided.major == required_major;
}

/// The same reference encoding against a migration window [lo, hi].
constexpr bool is_compatible(const Version & provided, const accept_major & range)
{
  return range.lo <= provided.major && provided.major <= range.hi;
}

/// Owner of every spec defined in this package (the OSS standard partition).
static constexpr char owner[] = "autowarefoundation";

/// Resolve a spec type's domain version via ADL into the spec's own namespace.
/// Each domain header declares a `resolve_domain_version(const Spec &)` overload
/// that returns that namespace's `version`. A compile error if a spec's domain
/// forgot to declare one is exactly the version-consistency discipline we want.
template <class Spec>
constexpr Version spec_version()
{
  return resolve_domain_version(Spec{});
}

}  // namespace autoware::component_interface_specs

/// Declares the three things every domain namespace owes the versioning contract: its
/// `version`, the `Specs` tuple registering the interfaces it owns, and the ADL hook
/// `spec_version<Spec>()` resolves through. Emitting them from one macro keeps them from
/// drifting apart -- a domain cannot bump its version but forget to register a new spec,
/// or register a spec whose version nothing can resolve.
///
/// Invoke once per domain header, inside the domain namespace, after the spec structs:
///
///     namespace autoware::component_interface_specs::control
///     {
///     struct ControlCommand { ... };
///     AUTOWARE_COMPONENT_INTERFACE_SPECS_DEFINE_DOMAIN(0, 1, 0, ControlCommand)
///     }
///
/// A domain may still add a `resolve_domain_version(const Spec &) = delete;` overload
/// afterwards to exclude an individual spec from version resolution.
#define AUTOWARE_COMPONENT_INTERFACE_SPECS_DEFINE_DOMAIN(MAJOR, MINOR, PATCH, ...)              \
  static constexpr ::autoware::component_interface_specs::Version version{MAJOR, MINOR, PATCH}; \
  using Specs = ::std::tuple<__VA_ARGS__>;                                                      \
                                                                                                \
  template <class Spec>                                                                         \
  constexpr ::autoware::component_interface_specs::Version resolve_domain_version(              \
    const Spec &) noexcept                                                                      \
  {                                                                                             \
    return version;                                                                             \
  }

#endif  // AUTOWARE__COMPONENT_INTERFACE_SPECS__VERSION_HPP_
