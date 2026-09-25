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

from autoware_agnocast_wrapper.actions import AutowareNode
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
    logger = get_logger("autoware_agnocast_wrapper.actions.autoware_node")
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

    def _build(built_with_agnocast, plugin=PLUGIN):
        monkeypatch.setattr(
            AutowareNode,
            "_registration",
            staticmethod(lambda pkg, exe: (plugin, built_with_agnocast)),
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


def text(substitutions):
    return "".join(t.text for t in substitutions)


def env_of(node):
    return {text(key): text(value) for key, value in node.additional_env}


def component_of(load):
    # LoadComposableNodes keeps its descriptions private; there is no accessor to go through.
    (description,) = load._LoadComposableNodes__composable_node_descriptions
    return description


@pytest.mark.parametrize("built_with_agnocast", [True, False], ids=["build=1", "build=0"])
@pytest.mark.parametrize("enable_agnocast", ["1", "0"], ids=["run=1", "run=0"])
@pytest.mark.parametrize("mode", [None, "auto", "rclcpp"])
@pytest.mark.parametrize("target", [CONTAINER, None], ids=["target", "no target"])
def test_form_and_transport(
    build, run, context, heaphook, built_with_agnocast, enable_agnocast, mode, target
):
    """The decision table: Agnocast only where the build and the run offer it and mode allows."""
    build(built_with_agnocast)
    run(enable_agnocast)

    emitted = emit(action(mode=mode, target=target), context)

    on_agnocast = built_with_agnocast and enable_agnocast == "1"
    if on_agnocast and mode != "rclcpp":
        # Agnocast always takes a process of its own, so target is dropped.
        assert isinstance(emitted, Node)
        assert env_of(emitted)["ENABLE_AGNOCAST"] == "1"
        assert env_of(emitted)["LD_PRELOAD"] == heaphook
    elif target is not None:
        assert isinstance(emitted, LoadComposableNodes)
        assert text(component_of(emitted).node_plugin) == PLUGIN
    else:
        assert isinstance(emitted, Node)
        assert env_of(emitted).get("ENABLE_AGNOCAST") == ("0" if on_agnocast else None)


def test_an_empty_target_is_no_container(build, run, context):
    build(False)
    run("0")
    assert isinstance(emit(action(target=""), context), Node)


@pytest.mark.parametrize("built_with_agnocast", [True, False])
@pytest.mark.parametrize("enable_agnocast", ["1", "0"])
def test_an_unknown_mode_is_rejected_everywhere(
    build, run, context, built_with_agnocast, enable_agnocast
):
    """Including where Agnocast is unavailable, where mode changes nothing."""
    build(built_with_agnocast)
    run(enable_agnocast)
    with pytest.raises(RuntimeError, match="unknown mode"):
        emit(action(mode="agnocast"), context)


def test_a_container_needs_a_registered_component(build, run, context):
    build(False, plugin=None)
    run("0")
    with pytest.raises(RuntimeError, match="registers no"):
        emit(action(target=CONTAINER), context)


def test_the_registration_is_read_from_the_ament_index(monkeypatch, tmp_path):
    """In the format autoware_agnocast_wrapper_register_node() writes it."""
    index = tmp_path / "share" / "ament_index" / "resource_index" / "autoware_node_plugins"
    index.mkdir(parents=True)
    (index / "a_package__an_executable").write_text(f"{PLUGIN};1")
    monkeypatch.setenv("AMENT_PREFIX_PATH", str(tmp_path))

    assert AutowareNode._registration("a_package", "an_executable") == (PLUGIN, True)
    assert AutowareNode._registration("a_package", "another_executable") == (None, False)


@pytest.mark.parametrize("source", ["config", "env"])
def test_a_heaphook_that_is_not_there_stops_the_launch(build, run, context, source):
    build(True)
    run("1")
    missing = "/nowhere/libagnocast_heaphook.so"
    if source == "config":
        context.launch_configurations["agnocast_heaphook_path"] = missing
        act = action()
    else:
        act = action(additional_env={("LD_PRELOAD",): subs(missing)})
    with pytest.raises(RuntimeError, match="does not exist"):
        emit(act, context)


@pytest.mark.parametrize("source", ["launch", "env"], ids=["from the launch", "from <env>"])
@pytest.mark.parametrize(
    "given,expected_tail",
    [
        ("{heaphook}", []),
        ("/lib/one.so:/lib/two.so", ["/lib/one.so", "/lib/two.so"]),
        ("/lib/one.so:{heaphook}", ["/lib/one.so"]),
    ],
    ids=["the same one", "unrelated", "mixed"],
)
def test_the_heaphook_is_preloaded_exactly_once(
    build, run, context, heaphook, source, given, expected_tail
):
    """A second copy of it in one process does not work, wherever the LD_PRELOAD came from."""
    build(True)
    given = given.format(heaphook=heaphook)
    if source == "launch":
        run("1", ld_preload=given)
        act = action()
    else:
        run("1", ld_preload="/lib/inherited.so")
        act = action(additional_env={("LD_PRELOAD",): subs(given)})

    preload = env_of(emit(act, context))["LD_PRELOAD"].split(":")

    assert preload == [heaphook, *expected_tail]


def test_an_inherited_heaphook_elsewhere_is_replaced_and_reported(
    build, run, context, heaphook, warnings
):
    build(True)
    run("1", ld_preload="/elsewhere/libagnocast_heaphook.so")

    preload = env_of(emit(action(name="a_node"), context))["LD_PRELOAD"].split(":")

    assert preload == [heaphook]
    (warning,) = warnings
    assert "'a_node'" in warning.getMessage()
    assert "/elsewhere/libagnocast_heaphook.so" in warning.getMessage()


def test_a_heaphook_given_through_env_wins(build, run, context, tmp_path, warnings):
    """So that one node can run another build of it."""
    build(True)
    run("1")
    own = tmp_path / "debug" / "libagnocast_heaphook.so"
    own.parent.mkdir()
    own.touch()

    emitted = emit(action(additional_env={("LD_PRELOAD",): subs(f"/lib/one.so:{own}")}), context)

    assert env_of(emitted)["LD_PRELOAD"].split(":") == [str(own), "/lib/one.so"]
    assert not warnings


def test_the_standalone_form_takes_the_process_arguments(build, run, context):
    build(False)
    run("0")

    emitted = emit(action(output=subs("both"), additional_env={("A",): subs("B")}), context)

    assert text(emitted.output) == "both"
    assert env_of(emitted)["A"] == "B"


@pytest.mark.parametrize("mode,decided", [("auto", "1"), ("rclcpp", "0")])
def test_enable_agnocast_in_env_is_overridden(build, run, context, mode, decided):
    """ENABLE_AGNOCAST is the action's to decide; mode is how a launch file has its say."""
    build(True)
    run("1")
    given = "0" if decided == "1" else "1"

    emitted = emit(action(mode=mode, additional_env={("ENABLE_AGNOCAST",): subs(given)}), context)

    assert env_of(emitted)["ENABLE_AGNOCAST"] == decided


def test_the_container_form_drops_the_process_arguments(build, run, context):
    """Handing any of them to ComposableNode, which takes none, would raise."""
    build(False)
    run("0")

    emitted = emit(action(target=CONTAINER, output=subs("both")), context)

    assert isinstance(emitted, LoadComposableNodes)


def parsed(attributes):
    """Parse one <autoware_node> the way the XML frontend does."""
    xml = f'<launch><autoware_node pkg="p" exec="e" name="n" {attributes}/></launch>'
    root, parser = Parser.load(io.StringIO(xml))
    (act,) = parser.parse_description(root).entities
    return act


@pytest.mark.parametrize("target", [CONTAINER, None], ids=["container", "standalone"])
def test_the_namespace_reaches_both_forms(build, run, context, target):
    build(False)
    run("0")
    target_attr = f'target="{target}"' if target else ""

    emitted = emit(parsed(f'namespace="a_namespace" {target_attr}'), context)

    if target:
        assert text(component_of(emitted).node_namespace) == "a_namespace"
    else:
        # Node keeps the namespace it was given private; there is no accessor to go through.
        assert emitted._Node__node_namespace == "a_namespace"


@pytest.mark.parametrize("target", [CONTAINER, None], ids=["container", "standalone"])
def test_if_false_emits_nothing(build, run, context, target):
    build(False)
    run("0")
    target_attr = f'target="{target}"' if target else ""

    assert parsed(f'if="false" {target_attr}').visit(context) is None


def test_the_child_elements_reach_the_component(build, run, context):
    build(False)
    run("0")
    xml = f"""<launch>
      <autoware_node pkg="p" exec="e" name="n" target="{CONTAINER}">
        <param name="a_param" value="1"/>
        <remap from="input" to="an_input"/>
        <extra_arg name="use_intra_process_comms" value="true"/>
      </autoware_node>
    </launch>"""
    root, parser = Parser.load(io.StringIO(xml))
    (act,) = parser.parse_description(root).entities

    component = component_of(emit(act, context))

    ((name, value),) = component.parameters[0].items()
    assert (text(name), value.evaluate(context)) == ("a_param", 1)
    assert [(text(src), text(dst)) for src, dst in component.remappings] == [("input", "an_input")]
    ((name, value),) = component.extra_arguments[0].items()
    assert (text(name), text(value)) == ("use_intra_process_comms", "true")


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
    "built_with_agnocast,enable_agnocast,mode,target,warned",
    [
        # The one case where something outside <autoware_node> settles the transport.
        (True, "1", "rclcpp", CONTAINER, True),
        # Agnocast takes a process of its own, so no container is involved.
        (True, "1", "auto", CONTAINER, False),
        (True, "1", "rclcpp", None, False),
        # Agnocast is not in play, so the container's environment does not matter.
        (True, "0", "rclcpp", CONTAINER, False),
        (False, "1", "rclcpp", CONTAINER, False),
    ],
)
def test_it_warns_where_the_container_decides_the_transport(
    build, run, context, warnings, built_with_agnocast, enable_agnocast, mode, target, warned
):
    """The action cannot see the container it hands the node to, so all it can do is say so."""
    build(built_with_agnocast)
    run(enable_agnocast)

    emit(action(mode=mode, target=target), context)

    assert bool(warnings) == warned
    if warned:
        assert "ENABLE_AGNOCAST=0" in warnings[0].getMessage()
