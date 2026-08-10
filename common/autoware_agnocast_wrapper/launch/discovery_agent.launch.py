# Copyright 2026 TIER IV, Inc.
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

"""Spawn exactly one ``agnocast_discovery_agent`` per ``ros2 launch`` invocation.

``agnocast_env.launch.{xml,py}`` is included once per agnocast-using node, but the
discovery agent is a per-IPC-namespace singleton — one per launch tree is enough.
Both wrappers include this file; the guard marker lives on the ``LaunchContext``
*globals* so deduplication works across the whole launch tree, including the
XML -> Python include boundary.

Spawns only when ``ENABLE_AGNOCAST=1`` (read from the environment directly so it
does not depend on the including scope). The agent's own ``flock(2)`` remains the
cross-invocation safety net for separate ``ros2 launch`` runs against the same
IPC namespace.
"""

import os

from launch import LaunchDescription
from launch.actions import OpaqueFunction
from launch_ros.actions import Node

# Marker key on the LaunchContext globals (tree-wide, not scope-local).
_DISCOVERY_AGENT_SPAWNED_FLAG = "_agnocast_discovery_agent_spawned"


def _spawn_once(context):
    if os.environ.get("ENABLE_AGNOCAST", "0") != "1":
        return []
    if context.get_locals_as_dict().get(_DISCOVERY_AGENT_SPAWNED_FLAG):
        return []
    context.extend_globals({_DISCOVERY_AGENT_SPAWNED_FLAG: True})
    return [
        Node(
            package="ros2agnocast_discovery_agent",
            executable="discovery_agent",
            name="agnocast_discovery_agent",
            # Pin to root: which include wins the deduplication race is arbitrary,
            # so without this the singleton would inherit some node's namespace.
            namespace="/",
            output="screen",
        ),
    ]


def generate_launch_description():
    return LaunchDescription([OpaqueFunction(function=_spawn_once)])
