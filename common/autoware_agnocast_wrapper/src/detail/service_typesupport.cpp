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

#include "autoware/agnocast_wrapper/detail/service_typesupport.hpp"

#include <ament_index_cpp/get_package_prefix.hpp>
#include <rcpputils/shared_library.hpp>

#include <stdexcept>
#include <string>
#include <tuple>

namespace
{

struct TypeParts
{
  std::string package;
  std::string middle;
  std::string type_name;
};

TypeParts extract_type_parts(const std::string & type)
{
  const auto sep1 = type.find('/');
  if (sep1 == std::string::npos) {
    throw std::runtime_error(
      "Invalid service type '" + type + "': expected format 'package/subfolder/TypeName'");
  }
  const auto sep2 = type.find('/', sep1 + 1);
  if (sep2 == std::string::npos) {
    throw std::runtime_error(
      "Invalid service type '" + type + "': expected format 'package/subfolder/TypeName'");
  }

  return {type.substr(0, sep1), type.substr(sep1 + 1, sep2 - sep1 - 1), type.substr(sep2 + 1)};
}

std::string get_typesupport_library_path(
  const std::string & package_name, const std::string & typesupport_identifier)
{
#ifdef _WIN32
  const char * dynamic_library_folder = "/bin/";
#else
  const char * dynamic_library_folder = "/lib/";
#endif

  std::string package_prefix;
  try {
    package_prefix = ament_index_cpp::get_package_prefix(package_name);
  } catch (const ament_index_cpp::PackageNotFoundError & e) {
    throw std::runtime_error(e.what());
  }

  return package_prefix + dynamic_library_folder +
         rcpputils::get_platform_library_name(package_name + "__" + typesupport_identifier);
}

std::shared_ptr<rcpputils::SharedLibrary> get_service_typesupport_library(
  const std::string & type, const std::string & typesupport_identifier)
{
  const auto parts = extract_type_parts(type);
  const auto library_path = get_typesupport_library_path(parts.package, typesupport_identifier);

  try {
    return std::make_shared<rcpputils::SharedLibrary>(library_path);
  } catch (const std::runtime_error & e) {
    throw std::runtime_error(
      "Failed to load typesupport library for service type '" + type + "': " + e.what());
  }
}

const rosidl_service_type_support_t * get_service_typesupport_handle(
  const std::string & type, const std::string & typesupport_identifier,
  const std::shared_ptr<rcpputils::SharedLibrary> & library)
{
  const auto parts = extract_type_parts(type);

  const std::string symbol_name = typesupport_identifier + "__get_service_type_support_handle__" +
                                  parts.package + "__" + parts.middle + "__" + parts.type_name;

  if (!library->has_symbol(symbol_name)) {
    throw std::runtime_error("Failed to find symbol '" + symbol_name + "' in typesupport library");
  }

  using ServiceTypesupportHandleFunc = const rosidl_service_type_support_t * (*)();
  auto func = reinterpret_cast<ServiceTypesupportHandleFunc>(library->get_symbol(symbol_name));
  auto * handle = func();
  if (!handle) {
    throw std::runtime_error("Service typesupport handle is null for type '" + type + "'");
  }
  return handle;
}

}  // namespace

namespace autoware::agnocast_wrapper::detail
{

ServiceTsBundle load_service_typesupport(const std::string & service_type)
{
  ServiceTsBundle bundle;

  bundle.ts_lib = get_service_typesupport_library(service_type, "rosidl_typesupport_cpp");
  bundle.service_ts =
    get_service_typesupport_handle(service_type, "rosidl_typesupport_cpp", bundle.ts_lib);

  const std::string introspection_identifier = "rosidl_typesupport_introspection_cpp";
  bundle.introspection_ts_lib =
    get_service_typesupport_library(service_type, introspection_identifier);
  const auto * introspection_handle = get_service_typesupport_handle(
    service_type, introspection_identifier, bundle.introspection_ts_lib);

  const auto * members = static_cast<const rosidl_typesupport_introspection_cpp::ServiceMembers *>(
    introspection_handle->data);
  if (!members) {
    throw std::runtime_error("Failed to get ServiceMembers for type '" + service_type + "'");
  }

  bundle.request_members = members->request_members_;
  bundle.response_members = members->response_members_;
  return bundle;
}

}  // namespace autoware::agnocast_wrapper::detail
