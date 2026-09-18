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

"""What <autoware_node> emits for each build, run, mode and target."""

import io
import logging

from autoware_agnocast_wrapper_launch.actions import AutowareNode
from launch import LaunchContext
from launch.frontend import Parser
from launch.logging import get_logger
from launch.substitutions import TextSubstitution
from launch_ros.actions import LoadComposableNodes
from launch_ros.actions import Node
import pytest

PLUGIN = "a_package::AComponent"
CONTAINER = "a_container"


def subs(text):
    """Stand in for what the frontend hands the action."""
    return None if text is None else [TextSubstitution(text=text)]


@pytest.fixture
def heaphook(tmp_path):
    """Give the action a heaphook that exists, since it refuses to preload one that does not."""
    path = tmp_path / "libagnocast_heaphook.so"
    path.touch()
    return str(path)


@pytest.fixture
def context(heaphook):
    ctx = LaunchContext()
    ctx.launch_configurations["agnocast_heaphook_path"] = heaphook
    return ctx


@pytest.fixture
def warnings():
    """Collect what the action logs; launch's loggers do not propagate to caplog."""
    logger = get_logger("autoware_agnocast_wrapper_launch.actions.autoware_node")
    records = []

    class Collect(logging.Handler):
        def emit(self, record):
            if record.levelno >= logging.WARNING:
                records.append(record)

    handler = Collect()
    logger.addHandler(handler)
    yield records
    logger.removeHandler(handler)


@pytest.fixture
def build(monkeypatch):
    """Answer the resource lookup, which otherwise needs a built workspace."""

    def _build(transport, plugin=PLUGIN):
        monkeypatch.setattr(
            AutowareNode, "_registration", staticmethod(lambda pkg, exe: (plugin, transport))
        )

    return _build


@pytest.fixture
def run(monkeypatch):
    def _run(enable_agnocast, ld_preload=""):
        monkeypatch.setenv("ENABLE_AGNOCAST", enable_agnocast)
        monkeypatch.setenv("LD_PRELOAD", ld_preload)

    return _run


def action(mode=None, target=None, name="a_node", **process_kwargs):
    return AutowareNode(
        package=subs("a_package"),
        executable=subs("an_executable"),
        name=subs(name),
        mode=subs(mode),
        target=subs(target),
        **process_kwargs,
    )


def emit(act, context):
    (emitted,) = act.execute(context)
    return emitted


def env_of(node):
    return {
        "".join(t.text for t in key): "".join(t.text for t in value)
        for key, value in node.additional_env
    }


def component_of(load):
    # LoadComposableNodes keeps its descriptions private; there is no accessor to go through.
    (description,) = load._LoadComposableNodes__composable_node_descriptions
    return description


@pytest.mark.parametrize("transport", ["agnocast", "rclcpp"], ids=["build=1", "build=0"])
@pytest.mark.parametrize("enable_agnocast", ["1", "0"], ids=["run=1", "run=0"])
@pytest.mark.parametrize("mode", [None, "auto", "rclcpp"])
@pytest.mark.parametrize("target", [CONTAINER, None], ids=["target", "no target"])
def test_form_and_transport(
    build, run, context, heaphook, transport, enable_agnocast, mode, target
):
    """The decision table: Agnocast only where the build and the run offer it and mode allows."""
    build(transport)
    run(enable_agnocast)

    emitted = emit(action(mode=mode, target=target), context)

    on_agnocast = transport == "agnocast" and enable_agnocast == "1"
    if on_agnocast and mode != "rclcpp":
        # Agnocast always takes a process of its own, so target is dropped.
        assert isinstance(emitted, Node)
        assert env_of(emitted)["ENABLE_AGNOCAST"] == "1"
        assert env_of(emitted)["LD_PRELOAD"] == heaphook
    elif target is not None:
        assert isinstance(emitted, LoadComposableNodes)
        assert "".join(t.text for t in component_of(emitted).node_plugin) == PLUGIN
    else:
        assert isinstance(emitted, Node)
        assert env_of(emitted).get("ENABLE_AGNOCAST") == ("0" if on_agnocast else None)


def test_an_empty_target_is_no_container(build, run, context):
    build("rclcpp")
    run("0")
    assert isinstance(emit(action(target=""), context), Node)


@pytest.mark.parametrize("transport", ["agnocast", "rclcpp"])
@pytest.mark.parametrize("enable_agnocast", ["1", "0"])
def test_an_unknown_mode_is_rejected_everywhere(build, run, context, transport, enable_agnocast):
    """Including where Agnocast is unavailable, where mode changes nothing."""
    build(transport)
    run(enable_agnocast)
    with pytest.raises(RuntimeError, match="unknown mode"):
        emit(action(mode="agnocast"), context)


def test_a_container_needs_a_registered_component(build, run, context):
    build("rclcpp", plugin=None)
    run("0")
    with pytest.raises(RuntimeError, match="registers no"):
        emit(action(target=CONTAINER), context)


def test_a_heaphook_that_is_not_there_stops_the_launch(build, run, context):
    build("agnocast")
    run("1")
    context.launch_configurations["agnocast_heaphook_path"] = "/nowhere/libagnocast_heaphook.so"
    with pytest.raises(RuntimeError, match="does not exist"):
        emit(action(), context)


@pytest.mark.parametrize(
    "inherited,expected_tail",
    [
        ("", []),
        ("{heaphook}", []),
        ("/opt/ros/humble/lib/libagnocast_heaphook.so", []),
        ("/lib/one.so:/lib/two.so", ["/lib/one.so", "/lib/two.so"]),
        ("/lib/one.so:{heaphook}", ["/lib/one.so"]),
    ],
    ids=["empty", "the same one", "another one", "unrelated", "mixed"],
)
def test_the_heaphook_is_preloaded_exactly_once(
    build, run, context, heaphook, inherited, expected_tail
):
    """A second copy of it in one process does not work, whatever the launch was started with."""
    build("agnocast")
    run("1", ld_preload=inherited.format(heaphook=heaphook))

    preload = env_of(emit(action(), context))["LD_PRELOAD"].split(":")

    assert preload == [heaphook, *expected_tail]


def test_the_standalone_form_takes_the_process_arguments(build, run, context):
    build("rclcpp")
    run("0")

    emitted = emit(
        action(
            output=subs("both"),
            arguments=[subs("--foo")],
            additional_env={("A",): subs("B")},
        ),
        context,
    )

    assert "".join(t.text for t in emitted.output) == "both"
    assert env_of(emitted)["A"] == "B"


def test_the_transport_wins_over_the_launch_files_own_env(build, run, context):
    """ENABLE_AGNOCAST is the action's to decide; mode is how a launch file has its say."""
    build("agnocast")
    run("1")

    emitted = emit(action(additional_env={("ENABLE_AGNOCAST",): subs("0")}), context)

    assert env_of(emitted)["ENABLE_AGNOCAST"] == "1"


def test_the_container_form_drops_the_process_arguments(build, run, context):
    build("rclcpp")
    run("0")

    emitted = emit(action(target=CONTAINER, output=subs("both")), context)

    assert isinstance(emitted, LoadComposableNodes)


def test_extra_arguments_reach_the_component(build, run, context):
    build("rclcpp")
    run("0")

    extra = [{("use_intra_process_comms",): subs("true")}]
    emitted = emit(action(target=CONTAINER, extra_arguments=extra), context)

    # ComposableNode normalizes the keys, so compare what they say rather than how they are held.
    assert [
        {"".join(t.text for t in key): "".join(t.text for t in value) for key, value in arg.items()}
        for arg in component_of(emitted).extra_arguments
    ] == [{"use_intra_process_comms": "true"}]


@pytest.mark.parametrize(
    "attributes",
    [
        'output="both"',
        'args="--foo"',
        'ros_args="--log-level debug"',
        'respawn="true" respawn_delay="1.0"',
        'exec_name="an_alias"',
        'launch-prefix="nice"',
        'namespace="a_namespace"',
        'if="true"',
    ],
)
def test_it_accepts_what_node_accepts(attributes):
    """<autoware_node> is parsed by <node>'s own code, so it takes the same attributes."""
    xml = f'<launch><autoware_node pkg="p" exec="e" name="n" {attributes}/></launch>'
    root, parser = Parser.load(io.StringIO(xml))
    parser.parse_description(root)


def test_it_still_rejects_what_node_rejects():
    xml = '<launch><autoware_node pkg="p" exec="e" not_an_attribute="1"/></launch>'
    root, parser = Parser.load(io.StringIO(xml))
    with pytest.raises(ValueError, match="Unexpected attribute"):
        parser.parse_description(root)


def test_it_needs_a_package():
    xml = '<launch><autoware_node exec="e"/></launch>'
    root, parser = Parser.load(io.StringIO(xml))
    with pytest.raises(ValueError, match="requires a 'pkg'"):
        parser.parse_description(root)


@pytest.mark.parametrize(
    "transport,enable_agnocast,mode,target,warned",
    [
        # The one case where something outside <autoware_node> settles the transport.
        ("agnocast", "1", "rclcpp", CONTAINER, True),
        # Agnocast takes a process of its own, so no container is involved.
        ("agnocast", "1", "auto", CONTAINER, False),
        ("agnocast", "1", "rclcpp", None, False),
        # Agnocast is not in play, so the container's environment does not matter.
        ("agnocast", "0", "rclcpp", CONTAINER, False),
        ("rclcpp", "1", "rclcpp", CONTAINER, False),
        ("rclcpp", "0", "auto", CONTAINER, False),
    ],
)
def test_it_warns_where_the_container_decides_the_transport(
    build, run, context, warnings, transport, enable_agnocast, mode, target, warned
):
    """The action cannot see the container it hands the node to, so all it can do is say so."""
    build(transport)
    run(enable_agnocast)

    emit(action(mode=mode, target=target), context)

    assert bool(warnings) == warned
    if warned:
        assert "ENABLE_AGNOCAST=0" in warnings[0].getMessage()
