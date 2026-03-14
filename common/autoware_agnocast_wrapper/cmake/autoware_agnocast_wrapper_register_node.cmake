# Copyright 2025 TIER IV, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

# Register an autoware node component with runtime switching support
# between rclcpp::Node and agnocast::Node.
#
# This macro creates:
# 1. A standard rclcpp_components registration for the component
# 2. A standalone executable that can switch between rclcpp::Node and agnocast::Node
#    at runtime based on the ENABLE_AGNOCAST environment variable
#
# When ENABLE_AGNOCAST is not set or set to 0, this macro falls back to
# standard rclcpp_components_register_node behavior.
#
# usage: autoware_agnocast_wrapper_register_node(
#        <target> PLUGIN <component> EXECUTABLE <node>
#        [ROS2_EXECUTOR <executor>] [AGNOCAST_EXECUTOR <executor>])
#
# :param target: the shared library target
# :type target: string
# :param PLUGIN: the plugin name (fully qualified class name)
# :type PLUGIN: string
# :param EXECUTABLE: the node's executable name
# :type EXECUTABLE: string
# :param ROS2_EXECUTOR: the executor class name to use when ENABLE_AGNOCAST=0
#   (default: SingleThreadedExecutor)
#   Valid values: SingleThreadedExecutor, MultiThreadedExecutor
# :type ROS2_EXECUTOR: string
# :param AGNOCAST_EXECUTOR: the executor class name to use when ENABLE_AGNOCAST=1
#   (default: SingleThreadedAgnocastExecutor)
#   Valid values: SingleThreadedAgnocastExecutor, MultiThreadedAgnocastExecutor,
#                 CallbackIsolatedAgnocastExecutor, AgnocastOnlySingleThreadedExecutor,
#                 AgnocastOnlyMultiThreadedExecutor, AgnocastOnlyCallbackIsolatedExecutor
# :type AGNOCAST_EXECUTOR: string
#
# Node class requirements:
#   The PLUGIN's required base class depends on the AGNOCAST_EXECUTOR type:
#   - AgnocastOnly* executors  -> PLUGIN must inherit from autoware::agnocast_wrapper::Node
#   - Other agnocast executors -> PLUGIN can be any rclcpp::Node-compatible class
#   The generated template enforces this via if constexpr at compile time.
#
# Generated targets:
#   When ENABLE_AGNOCAST=1 (agnocast mode), two targets are created:
#     - <EXECUTABLE>            : standalone executable with runtime rclcpp/agnocast switching
#     - <EXECUTABLE>_component  : standard rclcpp_components executable for component containers
#   When ENABLE_AGNOCAST is not set (standard mode), only one target is created:
#     - <EXECUTABLE>            : standard rclcpp_components executable (delegates to
#                                 rclcpp_components_register_node as-is)
#   Launch files should always reference <EXECUTABLE> for consistent behavior across both modes.
#
# Example:
#   autoware_agnocast_wrapper_register_node(my_node_component
#     PLUGIN "my_package::MyNode"
#     EXECUTABLE my_node
#     ROS2_EXECUTOR MultiThreadedExecutor
#     AGNOCAST_EXECUTOR MultiThreadedAgnocastExecutor
#   )
#
# NOTE: This is intentionally a macro (not a function) because it calls
# rclcpp_components_register_node, which is also a macro and requires
# access to the caller's variable scope for ament resource index registration.
macro(autoware_agnocast_wrapper_register_node target)
  if(NOT TARGET ${target})
    message(FATAL_ERROR
      "autoware_agnocast_wrapper_register_node: target '${target}' does not exist")
  endif()

  # cspell: ignore ARGN
  cmake_parse_arguments(ARGS "" "PLUGIN;EXECUTABLE;ROS2_EXECUTOR;AGNOCAST_EXECUTOR" "" ${ARGN})

  if(ARGS_UNPARSED_ARGUMENTS)
    message(FATAL_ERROR "autoware_agnocast_wrapper_register_node() called with unused "
      "arguments: ${ARGS_UNPARSED_ARGUMENTS}")
  endif()

  if("${ARGS_PLUGIN}" STREQUAL "")
    message(FATAL_ERROR
      "autoware_agnocast_wrapper_register_node macro requires a PLUGIN argument for target ${target}")
  endif()

  if("${ARGS_EXECUTABLE}" STREQUAL "")
    message(FATAL_ERROR
      "autoware_agnocast_wrapper_register_node macro requires an EXECUTABLE argument for target ${target}")
  endif()

  # Set defaults
  if("${ARGS_ROS2_EXECUTOR}" STREQUAL "")
    set(ARGS_ROS2_EXECUTOR "SingleThreadedExecutor")
  endif()
  if("${ARGS_AGNOCAST_EXECUTOR}" STREQUAL "")
    set(ARGS_AGNOCAST_EXECUTOR "SingleThreadedAgnocastExecutor")
  endif()

  # --- Validate executor threading model consistency ---
  # Warn if ROS2_EXECUTOR and AGNOCAST_EXECUTOR have different threading models,
  # as this silently changes behavior depending on the runtime ENABLE_AGNOCAST value.
  if("${ARGS_AGNOCAST_EXECUTOR}" MATCHES "SingleThreaded")
    set(_AWR_agnocast_threading "single")
  else()
    set(_AWR_agnocast_threading "multi")
  endif()
  if("${ARGS_ROS2_EXECUTOR}" STREQUAL "SingleThreadedExecutor")
    set(_AWR_ros2_threading "single")
  else()
    set(_AWR_ros2_threading "multi")
  endif()
  if(NOT "${_AWR_ros2_threading}" STREQUAL "${_AWR_agnocast_threading}")
    message(WARNING
      "autoware_agnocast_wrapper_register_node: ROS2_EXECUTOR '${ARGS_ROS2_EXECUTOR}' and "
      "AGNOCAST_EXECUTOR '${ARGS_AGNOCAST_EXECUTOR}' have different threading models. "
      "This means behavior will differ depending on the runtime ENABLE_AGNOCAST value.")
  endif()
  unset(_AWR_agnocast_threading)
  unset(_AWR_ros2_threading)

  # --- Map ROS2_EXECUTOR to actual type ---
  # Variables prefixed with _AWR_ (Agnocast Wrapper Register) are template substitution
  # variables used by configure_file for @VAR@ replacement in node_main_switchable.cpp.in.
  # The prefix prevents name collisions since CMake macros share the caller's variable scope.
  # This mapping is performed before the ENABLE_AGNOCAST check because it is used in both
  # the agnocast branch (for the template) and the non-agnocast branch (for the executor).
  if("${ARGS_ROS2_EXECUTOR}" STREQUAL "SingleThreadedExecutor")
    set(_AWR_ros2_executor_type "rclcpp::executors::SingleThreadedExecutor")
  elseif("${ARGS_ROS2_EXECUTOR}" STREQUAL "MultiThreadedExecutor")
    set(_AWR_ros2_executor_type "rclcpp::executors::MultiThreadedExecutor")
  else()
    message(FATAL_ERROR
      "autoware_agnocast_wrapper_register_node: invalid ROS2_EXECUTOR '${ARGS_ROS2_EXECUTOR}'. "
      "Valid values: SingleThreadedExecutor, MultiThreadedExecutor")
  endif()

  if(DEFINED ENV{ENABLE_AGNOCAST} AND "$ENV{ENABLE_AGNOCAST}" STREQUAL "1")
    # ===== Agnocast mode: create component + switchable executable =====

    find_package(agnocastlib REQUIRED)

    # --- Map AGNOCAST_EXECUTOR to actual type, include, and add_node expression ---
    if("${ARGS_AGNOCAST_EXECUTOR}" STREQUAL "SingleThreadedAgnocastExecutor")
      set(_AWR_agnocast_executor_include "agnocast/agnocast_single_threaded_executor.hpp")
      set(_AWR_agnocast_executor_type "agnocast::SingleThreadedAgnocastExecutor")
      set(_AWR_agnocast_add_node_expr "get_node_base_interface()")
      set(_AWR_agnocast_only "false")
    elseif("${ARGS_AGNOCAST_EXECUTOR}" STREQUAL "MultiThreadedAgnocastExecutor")
      set(_AWR_agnocast_executor_include "agnocast/agnocast_multi_threaded_executor.hpp")
      set(_AWR_agnocast_executor_type "agnocast::MultiThreadedAgnocastExecutor")
      set(_AWR_agnocast_add_node_expr "get_node_base_interface()")
      set(_AWR_agnocast_only "false")
    elseif("${ARGS_AGNOCAST_EXECUTOR}" STREQUAL "CallbackIsolatedAgnocastExecutor")
      set(_AWR_agnocast_executor_include "agnocast/agnocast_callback_isolated_executor.hpp")
      set(_AWR_agnocast_executor_type "agnocast::CallbackIsolatedAgnocastExecutor")
      set(_AWR_agnocast_add_node_expr "get_node_base_interface()")
      set(_AWR_agnocast_only "false")
    elseif("${ARGS_AGNOCAST_EXECUTOR}" STREQUAL "AgnocastOnlySingleThreadedExecutor")
      set(_AWR_agnocast_executor_include "agnocast/node/agnocast_only_single_threaded_executor.hpp")
      set(_AWR_agnocast_executor_type "agnocast::AgnocastOnlySingleThreadedExecutor")
      set(_AWR_agnocast_add_node_expr "get_agnocast_node()")
      set(_AWR_agnocast_only "true")
    elseif("${ARGS_AGNOCAST_EXECUTOR}" STREQUAL "AgnocastOnlyMultiThreadedExecutor")
      set(_AWR_agnocast_executor_include "agnocast/node/agnocast_only_multi_threaded_executor.hpp")
      set(_AWR_agnocast_executor_type "agnocast::AgnocastOnlyMultiThreadedExecutor")
      set(_AWR_agnocast_add_node_expr "get_agnocast_node()")
      set(_AWR_agnocast_only "true")
    elseif("${ARGS_AGNOCAST_EXECUTOR}" STREQUAL "AgnocastOnlyCallbackIsolatedExecutor")
      set(_AWR_agnocast_executor_include "agnocast/node/agnocast_only_callback_isolated_executor.hpp")
      set(_AWR_agnocast_executor_type "agnocast::AgnocastOnlyCallbackIsolatedExecutor")
      set(_AWR_agnocast_add_node_expr "get_agnocast_node()")
      set(_AWR_agnocast_only "true")
    else()
      message(FATAL_ERROR
        "autoware_agnocast_wrapper_register_node: invalid AGNOCAST_EXECUTOR '${ARGS_AGNOCAST_EXECUTOR}'. "
        "Valid values: SingleThreadedAgnocastExecutor, MultiThreadedAgnocastExecutor, "
        "CallbackIsolatedAgnocastExecutor, AgnocastOnlySingleThreadedExecutor, "
        "AgnocastOnlyMultiThreadedExecutor, AgnocastOnlyCallbackIsolatedExecutor")
    endif()

    # Save the values with unique prefix to avoid collision with rclcpp_components_register_node
    set(_AGNOCAST_WRAPPER_COMPONENT ${ARGS_PLUGIN})
    set(_AGNOCAST_WRAPPER_NODE ${ARGS_EXECUTABLE})

    # Register with rclcpp_components for standard component container support
    # Note: This call will overwrite 'node', 'component', 'library_name' variables
    rclcpp_components_register_node(${target}
      PLUGIN ${_AGNOCAST_WRAPPER_COMPONENT}
      EXECUTABLE ${_AGNOCAST_WRAPPER_NODE}_component
      EXECUTOR ${ARGS_ROS2_EXECUTOR})

    # Set template substitution variables (_AWR_ prefix) after rclcpp_components_register_node call
    set(_AWR_node ${_AGNOCAST_WRAPPER_NODE})
    set(_AWR_component ${_AGNOCAST_WRAPPER_COMPONENT})
    set(_AWR_library_name "$<TARGET_FILE_NAME:${target}>")

    # Two-pass template generation:
    # Pass 1: configure_file replaces @VAR@ placeholders with CMake variables
    # Pass 2: file(GENERATE) resolves generator expressions like $<TARGET_FILE_NAME:...>
    #          that are only available at generation time
    file(MAKE_DIRECTORY "${PROJECT_BINARY_DIR}/autoware_agnocast_wrapper")
    configure_file(
      ${autoware_agnocast_wrapper_NODE_TEMPLATE}
      ${PROJECT_BINARY_DIR}/autoware_agnocast_wrapper/node_main_configured_${_AGNOCAST_WRAPPER_NODE}.cpp.in)
    file(GENERATE
      OUTPUT ${PROJECT_BINARY_DIR}/autoware_agnocast_wrapper/node_main_${_AGNOCAST_WRAPPER_NODE}.cpp
      INPUT ${PROJECT_BINARY_DIR}/autoware_agnocast_wrapper/node_main_configured_${_AGNOCAST_WRAPPER_NODE}.cpp.in)

    # Create runtime-switchable executable
    add_executable(${_AGNOCAST_WRAPPER_NODE}
      ${PROJECT_BINARY_DIR}/autoware_agnocast_wrapper/node_main_${_AGNOCAST_WRAPPER_NODE}.cpp)

    target_link_libraries(${_AGNOCAST_WRAPPER_NODE}
      ${target})

    ament_target_dependencies(${_AGNOCAST_WRAPPER_NODE}
      rclcpp
      rclcpp_components
      class_loader
      agnocastlib
      autoware_agnocast_wrapper)

    # Apply agnocast wrapper setup (adds USE_AGNOCAST_ENABLED if ENABLE_AGNOCAST=1)
    # Both the component library and the executable must have USE_AGNOCAST_ENABLED defined
    # to ensure ABI consistency (ament_target_dependencies does not propagate PUBLIC definitions)
    autoware_agnocast_wrapper_setup(${target})
    autoware_agnocast_wrapper_setup(${_AGNOCAST_WRAPPER_NODE})

    # Install executable
    install(TARGETS ${_AGNOCAST_WRAPPER_NODE}
      DESTINATION lib/${PROJECT_NAME})

  else()
    # ===== Standard rclcpp mode: fall back to rclcpp_components_register_node =====
    rclcpp_components_register_node(${target}
      PLUGIN ${ARGS_PLUGIN}
      EXECUTABLE ${ARGS_EXECUTABLE}
      EXECUTOR ${ARGS_ROS2_EXECUTOR})
  endif()

  # Cleanup temporary variables to prevent scope leakage across multiple macro invocations
  unset(_AWR_ros2_executor_type)
  unset(_AWR_agnocast_executor_type)
  unset(_AWR_agnocast_executor_include)
  unset(_AWR_agnocast_add_node_expr)
  unset(_AWR_agnocast_only)
  unset(_AWR_node)
  unset(_AWR_component)
  unset(_AWR_library_name)
  unset(_AGNOCAST_WRAPPER_COMPONENT)
  unset(_AGNOCAST_WRAPPER_NODE)
endmacro()
