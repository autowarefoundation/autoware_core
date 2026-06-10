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

"""Python equivalent of agnocast_env.launch.xml.

Configuration:
- use_agnocast arg overrides the ENABLE_AGNOCAST environment variable per include
  (set to "1" to enable). When not passed by the including launch file, it defaults to the
  ENABLE_AGNOCAST environment variable (which itself defaults to "0").
- Heaphook path is configurable via the agnocast_heaphook_path arg
  (default: /opt/ros/$ROS_DISTRO/lib/libagnocast_heaphook.so, falls back to humble)

Provides the following launch configurations:
- ld_preload_value: LD_PRELOAD value with heaphook prepended when Agnocast is enabled
- container_package: resolved component container package name
- container_executable: resolved component container executable name

Side effect (when ENABLE_AGNOCAST=1): emits exactly one
``agnocast_discovery_agent`` per ``ros2 launch`` invocation. The actual spawn
(and its tree-wide deduplication) lives in ``discovery_agent.launch.py``, which
this file includes.
"""

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.actions import OpaqueFunction
from launch.actions import SetLaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def _resolve_agnocast_env(context):
    use_agnocast = context.perform_substitution(LaunchConfiguration("use_agnocast"))
    use_multithread = context.perform_substitution(LaunchConfiguration("use_multithread"))
    heaphook_path = context.perform_substitution(LaunchConfiguration("agnocast_heaphook_path"))
    existing_ld_preload = context.perform_substitution(
        EnvironmentVariable("LD_PRELOAD", default_value="")
    )

    if use_agnocast == "1":
        ld_preload_value = f"{heaphook_path}:{existing_ld_preload}"
        container_package = "agnocast_components"
        if use_multithread == "true":
            container_executable = "agnocast_component_container_cie"
        else:
            container_executable = "agnocast_component_container"
    else:
        ld_preload_value = existing_ld_preload
        container_package = "rclcpp_components"
        if use_multithread == "true":
            container_executable = "component_container_mt"
        else:
            container_executable = "component_container"

    return [
        SetLaunchConfiguration("ld_preload_value", ld_preload_value),
        SetLaunchConfiguration("container_package", container_package),
        SetLaunchConfiguration("container_executable", container_executable),
    ]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "agnocast_heaphook_path",
                default_value=f"/opt/ros/{os.environ.get('ROS_DISTRO', 'humble')}/lib/libagnocast_heaphook.so",
            ),
            DeclareLaunchArgument(
                "use_multithread",
                default_value="false",
            ),
            DeclareLaunchArgument(
                "use_agnocast",
                default_value=EnvironmentVariable("ENABLE_AGNOCAST", default_value="0"),
            ),
            OpaqueFunction(function=_resolve_agnocast_env),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution(
                        [
                            FindPackageShare("autoware_agnocast_wrapper"),
                            "launch",
                            "discovery_agent.launch.py",
                        ]
                    )
                )
            ),
        ]
    )
