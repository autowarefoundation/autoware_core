// Copyright 2025 TIER IV, Inc.
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

#pragma once

// Runtime (string-typed) service typesupport lookup for ROS2GenericService/ROS2GenericClient (the
// hand-rolled rcl-based implementations in generic_service.hpp/generic_client.hpp).
// AgnocastGenericService/AgnocastGenericClient need no equivalent: they delegate directly to
// agnocast::GenericService/GenericClient, which resolve service_type's typesupport themselves.
// This file is load-bearing in both builds regardless (unlike most of this package, it has no
// Agnocast dependency at all, since ROS2GenericService/Client are declared unconditionally), so it
// is compiled unconditionally rather than guarded by USE_AGNOCAST_ENABLED. Mirrors
// autoware_generic_service_divider's own service_typesupport_helpers, which this was adapted from.

#include <rcpputils/shared_library.hpp>
#include <rosidl_typesupport_introspection_cpp/service_introspection.hpp>

#include <rosidl_runtime_c/service_type_support_struct.h>

#include <memory>
#include <string>

namespace autoware::agnocast_wrapper::detail
{

/// A generic service endpoint needs two independent typesupports (rosidl_typesupport_cpp, for
/// rcl_service_init()/rcl_client_init(); rosidl_typesupport_introspection_cpp, for the request and
/// response MessageMembers -- size, init, fini), each valid only while its shared library stays
/// loaded, hence the two shared_ptr<SharedLibrary> members below.
struct ServiceTsBundle
{
  std::shared_ptr<rcpputils::SharedLibrary> ts_lib;
  std::shared_ptr<rcpputils::SharedLibrary> introspection_ts_lib;
  const rosidl_service_type_support_t * service_ts{nullptr};
  const rosidl_typesupport_introspection_cpp::MessageMembers * request_members{nullptr};
  const rosidl_typesupport_introspection_cpp::MessageMembers * response_members{nullptr};
};

/// Loads both typesupport libraries for @p service_type (e.g. "std_srvs/srv/SetBool") and resolves
/// their handles.
/// @throws std::runtime_error if @p service_type is malformed or either typesupport library or
///         symbol cannot be loaded.
ServiceTsBundle load_service_typesupport(const std::string & service_type);

}  // namespace autoware::agnocast_wrapper::detail
