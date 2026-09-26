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

"""The ``autoware_node`` action."""

import os
from typing import List
from typing import Optional

from ament_index_python.resources import get_resource
from ament_index_python.resources import has_resource
from launch import Action
from launch.frontend import Entity
from launch.frontend import Parser
from launch.frontend import expose_action
from launch.launch_context import LaunchContext
from launch.logging import get_logger
from launch.utilities import normalize_to_list_of_substitutions
from launch.utilities import perform_substitutions
from launch_ros.actions import LoadComposableNodes
from launch_ros.actions import Node
from launch_ros.descriptions import ComposableNode

MODES = ("auto", "rclcpp")

#: Registered by autoware_agnocast_wrapper_register_node(), one resource per executable named
#: ``<package>__<executable>``, holding ``<component class>;<ENABLE_AGNOCAST at build: 0|1>``.
PLUGIN_RESOURCE = "autoware_node_plugins"


def _default_heaphook_path() -> str:
    distro = os.environ.get("ROS_DISTRO", "humble")
    return f"/opt/ros/{distro}/lib/libagnocast_heaphook.so"


@expose_action("autoware_node")
class AutowareNode(Action):
    """Launch a node the way it is written, or on Agnocast when the workspace runs on it.

    ``target`` decides the form, as it would anywhere else: given, the node is loaded into that
    container; omitted, it is a process of its own.

    ``mode`` only says whether to use Agnocast, and is read only when the node was built with
    Agnocast support and the launch runs with ``ENABLE_AGNOCAST=1``:

    - ``auto`` (the default): run on Agnocast where it is available, on rclcpp where it is not.
    - ``rclcpp``: stay on rclcpp, in the form ``target`` asks for.

    ``parameters`` and ``remappings`` apply to both forms. ``extra_arguments`` are the container's
    ``rclcpp::NodeOptions`` arguments, so they apply to the container form only. Everything else
    ``Node`` takes — ``output``, ``arguments``, ``respawn``, ``additional_env`` and the rest — is
    about a process, so it applies to the standalone form only.
    """

    def __init__(
        self,
        *,
        package,
        executable,
        name=None,
        namespace=None,
        mode=None,
        target=None,
        parameters=None,
        remappings=None,
        extra_arguments=None,
        condition=None,
        **process_kwargs,
    ):
        """Create an AutowareNode action.

        Anything ``Node`` takes beyond the arguments named here is kept in ``process_kwargs``.
        """
        super().__init__(condition=condition)
        self._package = package
        self._executable = executable
        self._name = name
        self._namespace = namespace
        self._mode = mode
        self._target = target
        self._parameters = parameters
        self._remappings = remappings
        self._extra_arguments = extra_arguments
        self._process_kwargs = process_kwargs

    @classmethod
    def parse(cls, entity: Entity, parser: Parser):
        """Parse an autoware_node entity."""
        # Take what only this action knows about first; what is left is then exactly a <node>,
        # which <node>'s own parser reads, so <autoware_node> accepts everything it does.
        mode = entity.get_attr("mode", optional=True)
        target = entity.get_attr("target", optional=True)
        extra_arguments = entity.get_attr("extra_arg", data_type=List[Entity], optional=True)

        _, kwargs = Node.parse(entity, parser)

        if "package" not in kwargs:
            raise ValueError("autoware_node requires a 'pkg' attribute")
        if mode is not None:
            kwargs["mode"] = parser.parse_substitution(mode)
        if target is not None:
            kwargs["target"] = parser.parse_substitution(target)
        if extra_arguments is not None:
            kwargs["extra_arguments"] = [
                {
                    tuple(parser.parse_substitution(extra_arg.get_attr("name"))): (
                        parser.parse_substitution(extra_arg.get_attr("value"))
                    )
                }
                for extra_arg in extra_arguments
            ]
            for extra_arg in extra_arguments:
                extra_arg.assert_entity_completely_parsed()

        entity.assert_entity_completely_parsed()
        return cls, kwargs

    def _perform(self, context: LaunchContext, value) -> Optional[str]:
        return None if value is None else perform_substitutions(context, value)

    @staticmethod
    def _registration(package: str, executable: str):
        """Return the component class and whether it was built with ENABLE_AGNOCAST=1."""
        resource = f"{package}__{executable}"
        if not has_resource(PLUGIN_RESOURCE, resource):
            return None, False
        content, _ = get_resource(PLUGIN_RESOURCE, resource)
        plugin, _, built_with_agnocast = content.strip().partition(";")
        return plugin, built_with_agnocast == "1"

    @staticmethod
    def _heaphook_path(context: LaunchContext, given: Optional[str]) -> str:
        """Return the heaphook to preload.

        One in the LD_PRELOAD the launch file ``given`` for this node wins, so that one node can
        run another build of it; otherwise it is the ``agnocast_heaphook_path`` config.
        """
        heaphook = context.launch_configurations.get(
            "agnocast_heaphook_path", _default_heaphook_path()
        )
        if given is not None:
            heaphook = next(
                (
                    entry
                    for entry in given.split(":")
                    if os.path.basename(entry) == os.path.basename(heaphook)
                ),
                heaphook,
            )
        # A bare file name is the dynamic linker's to resolve; a path is ours to check.
        if os.sep in heaphook and not os.path.exists(heaphook):
            raise RuntimeError(
                f"the Agnocast heaphook '{heaphook}' does not exist; point <env> LD_PRELOAD or "
                "the 'agnocast_heaphook_path' launch configuration at the one built against this "
                "workspace's agnocastlib"
            )
        return heaphook

    @staticmethod
    def _ld_preload(heaphook: str, base: str, label: str) -> str:
        """Prepend the heaphook to ``base``, keeping exactly one of it.

        A ``base`` already carrying the heaphook would otherwise hand the node a second copy of
        it, which Agnocast does not survive.
        """
        kept = []
        replaced = []
        for entry in base.split(":"):
            if not entry:
                continue
            if os.path.basename(entry) != os.path.basename(heaphook):
                kept.append(entry)
            elif entry != heaphook:
                replaced.append(entry)

        if replaced:
            get_logger(__name__).warning(
                f"the LD_PRELOAD for '{label}' already carries {', '.join(replaced)}; preloading "
                f"'{heaphook}' instead, since a second copy of the heaphook in one process does "
                "not work"
            )

        return ":".join([heaphook, *kept])

    def _mode_of(self, context: LaunchContext) -> str:
        mode = self._perform(context, self._mode) or "auto"
        if mode not in MODES:
            raise RuntimeError(f"unknown mode '{mode}', expected one of {', '.join(MODES)}")
        return mode

    def execute(self, context: LaunchContext):
        """Emit the node in the form ``target`` and ``mode`` ask for."""
        package = self._perform(context, self._package)
        executable = self._perform(context, self._executable)
        name = self._perform(context, self._name)
        namespace = self._perform(context, self._namespace) or None
        target = self._perform(context, self._target)
        label = name or executable
        plugin, built_with_agnocast = self._registration(package, executable)

        # Read unconditionally, so an unknown mode is rejected in every build and run.
        mode = self._mode_of(context)

        on_agnocast = built_with_agnocast and os.environ.get("ENABLE_AGNOCAST", "0") == "1"
        use_agnocast = on_agnocast and mode != "rclcpp"

        if use_agnocast and target:
            get_logger(__name__).info(
                f"'{label}' runs as a process of its own rather than in '{target}', "
                "since it runs on Agnocast"
            )
            target = None

        if target:
            if plugin is None:
                raise RuntimeError(
                    f"'{executable}' of {package} registers no {PLUGIN_RESOURCE} resource, so the "
                    f"component to load as '{label}' is unknown"
                )
            if on_agnocast:
                # Only a warning: the container is a name to the action, not something it can
                # look at, and the launch may well have started it correctly.
                get_logger(__name__).warning(
                    f"loading '{label}' into '{target}' while ENABLE_AGNOCAST=1: the container "
                    "has to be a plain rclcpp one started with ENABLE_AGNOCAST=0, since under "
                    "ENABLE_AGNOCAST=1 the node is an agnocast::Node, which no component "
                    "container can run"
                )
            if self._process_kwargs:
                # Not a warning: one node is written for both forms, so these being here and
                # sitting this form out is the normal case.
                get_logger(__name__).debug(
                    f"'{label}' is a component in '{target}', which has its own process, so "
                    f"{', '.join(sorted(self._process_kwargs))} do not apply"
                )
            return [
                LoadComposableNodes(
                    target_container=target,
                    composable_node_descriptions=[
                        ComposableNode(
                            package=package,
                            plugin=plugin,
                            name=name,
                            namespace=namespace,
                            parameters=self._parameters,
                            remappings=self._remappings,
                            extra_arguments=self._extra_arguments,
                        )
                    ],
                )
            ]

        process_kwargs = dict(self._process_kwargs)
        env = {
            perform_substitutions(context, normalize_to_list_of_substitutions(key)): value
            for key, value in (process_kwargs.pop("additional_env", None) or {}).items()
        }

        if use_agnocast:
            # An LD_PRELOAD of the launch file's own replaces the inherited one, as it would on
            # <node>, and the heaphook goes in front of whichever it is.
            given = env.pop("LD_PRELOAD", None)
            if given is not None:
                given = perform_substitutions(context, normalize_to_list_of_substitutions(given))
            heaphook = self._heaphook_path(context, given)
            base = os.environ.get("LD_PRELOAD", "") if given is None else given
            transport_env = {
                "ENABLE_AGNOCAST": "1",
                "LD_PRELOAD": self._ld_preload(heaphook, base, label),
            }
        else:
            # Only worth overriding when the node would otherwise have picked Agnocast up.
            transport_env = {"ENABLE_AGNOCAST": "0"} if on_agnocast else {}

        # The launch file's own <env> comes first, so the transport the action settled on wins.
        env.update(transport_env)

        return [
            Node(
                package=package,
                executable=executable,
                name=name,
                namespace=namespace,
                parameters=self._parameters,
                remappings=self._remappings,
                additional_env=env,
                **process_kwargs,
            )
        ]
