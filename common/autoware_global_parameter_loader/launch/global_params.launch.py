# Copyright 2021 Tier IV, Inc. All rights reserved.
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

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.actions import OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import SetParameter
from launch_ros.substitutions import FindPackageShare
import yaml

PARAM_FILE_SEPARATOR = ","


def split_param_files(param_files):
    """Split a separated list of parameter file paths, ignoring surrounding whitespace."""
    paths = (path.strip() for path in param_files.split(PARAM_FILE_SEPARATOR))
    return [path for path in paths if path]


def load_param_file(param_file):
    """Read the global parameters from a single parameter file."""
    try:
        with open(param_file, "r") as f:
            params = yaml.safe_load(f)
    except OSError as e:
        raise RuntimeError(f"failed to read the parameter file '{param_file}': {e}") from e
    except yaml.YAMLError as e:
        raise RuntimeError(f"failed to parse the parameter file '{param_file}': {e}") from e

    try:
        return params["/**"]["ros__parameters"]
    except (TypeError, KeyError) as e:
        raise RuntimeError(
            f"the parameter file '{param_file}' is not in the '/**: ros__parameters:' format"
        ) from e


def create_set_parameter_actions(param_files):
    """
    Create SetParameter actions for the given parameter files.

    Files that do not exist are silently skipped, so that optional parameter files can be
    listed unconditionally. When the same parameter appears in multiple files, the value of
    the last file wins.
    """
    params = {}
    for param_file in param_files:
        if os.path.isfile(param_file):
            params.update(load_param_file(param_file))
    return [SetParameter(name=k, value=v) for k, v in params.items()]


def launch_setup(context, *args, **kwargs):
    # use_sim_time
    set_use_sim_time = SetParameter(name="use_sim_time", value=LaunchConfiguration("use_sim_time"))

    # vehicle_info
    vehicle_description_pkg = FindPackageShare(
        [LaunchConfiguration("vehicle_model"), "_description"]
    ).perform(context)

    load_vehicle_info = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [FindPackageShare("autoware_vehicle_info_utils"), "/launch/vehicle_info.launch.py"]
        ),
        launch_arguments={
            "vehicle_info_param_file": [vehicle_description_pkg, "/config/vehicle_info.param.yaml"]
        }.items(),
    )

    # additional global parameters given by the caller
    param_files = split_param_files(
        LaunchConfiguration("global_parameter_loader_param_files").perform(context)
    )

    return [set_use_sim_time, load_vehicle_info, *create_set_parameter_actions(param_files)]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            DeclareLaunchArgument(
                "global_parameter_loader_param_files",
                default_value="",
                description="parameter files to load as global parameters, separated by '"
                + PARAM_FILE_SEPARATOR
                + "'; files that do not exist are skipped",
            ),
            OpaqueFunction(function=launch_setup),
        ]
    )
