# Copyright 2022 TIER IV, Inc.
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

import pathlib

import launch
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterFile
from launch_ros.substitutions import FindPackageShare


def create_standalone_api_node(node_name, executable):
    """Launch a node that derives from autoware::agnocast_wrapper::Node.

    Those nodes need an AgnocastOnly executor when ENABLE_AGNOCAST=1, which a shared component
    container cannot provide, so they run as their own process instead of being composed. All
    three keep a multi-threaded executor, matching the component_container_mt they used to share.
    """
    fullname = pathlib.Path("adapi/node") / node_name
    return Node(
        namespace=str(fullname.parent),
        name=str(fullname.name),
        package="autoware_default_adapi",
        executable=executable,
        parameters=[ParameterFile(LaunchConfiguration("config"))],
    )


def get_default_config():
    path = FindPackageShare("autoware_default_adapi")
    path = PathJoinSubstitution([path, "config/default_adapi.param.yaml"])
    return path


def generate_launch_description():
    nodes = [
        create_standalone_api_node("interface", "interface_node"),
        create_standalone_api_node("localization", "localization_node"),
        create_standalone_api_node("routing", "routing_node"),
    ]
    argument = DeclareLaunchArgument("config", default_value=get_default_config())
    return launch.LaunchDescription([argument, *nodes])
