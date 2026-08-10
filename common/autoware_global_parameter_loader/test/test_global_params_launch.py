# Copyright 2026 Tier IV, Inc. All rights reserved.
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

"""Test the optional parameter files of global_params.launch.py, called from an XML launch file."""

import os
from pathlib import Path
import re
from xml.sax.saxutils import quoteattr

from ament_index_python.packages import get_package_share_directory
from ament_index_python.packages import PackageNotFoundError
from launch import LaunchContext
from launch.frontend import Parser
from launch.utilities import visit_all_entities_and_collect_futures
import pytest
import yaml

GLOBAL_PARAMS_LAUNCH = Path(__file__).resolve().parent.parent / "launch" / "global_params.launch.py"

VEHICLE_MODEL = "test_global_params"
# vehicle_info is not under test here, but the launch file always loads it, so a minimal
# parameter file has to exist. Its parameters are excluded from the assertions below.
REQUIRED_VEHICLE_INFO = {"wheel_radius": 0.39}
ALWAYS_LOADED_PARAMS = {"use_sim_time", *REQUIRED_VEHICLE_INFO}

FIRST_PARAMS = {"first": 1, "shared": "from_first"}
SECOND_PARAMS = {"second": 2, "shared": "from_second"}


def write_param_file(path, params):
    """Write the parameters in the format expected by the launch file, and return the path."""
    path.write_text(yaml.safe_dump({"/**": {"ros__parameters": params}}))
    return path


def register_package(prefix, package):
    """Make a package findable through the ament index of the prefix, and return its share dir."""
    marker = prefix / "share" / "ament_index" / "resource_index" / "packages" / package
    marker.parent.mkdir(parents=True, exist_ok=True)
    marker.write_text("")

    share = prefix / "share" / package
    share.mkdir(parents=True, exist_ok=True)
    return share


def register_vehicle_info_utils_from_source_tree(prefix):
    """
    Register autoware_vehicle_info_utils from the source tree if it is not installed.

    The launch file under test always includes vehicle_info.launch.py of that package. It is
    installed when the tests run as part of a build, so this fallback only exists to keep
    the tests runnable with pytest alone, in a workspace that has not been built yet.
    """
    package = "autoware_vehicle_info_utils"
    try:
        get_package_share_directory(package)
        return
    except PackageNotFoundError:
        pass

    source = Path(__file__).resolve().parents[2] / package
    if not (source / "launch" / "vehicle_info.launch.py").is_file():
        pytest.skip(f"{package} is neither installed nor available in the source tree")

    (register_package(prefix, package) / "launch").symlink_to(source / "launch")


@pytest.fixture(scope="module")
def vehicle_description_package(tmp_path_factory):
    """Register the <vehicle_model>_description package that the launch file always needs."""
    prefix = tmp_path_factory.mktemp("prefix")

    config = register_package(prefix, f"{VEHICLE_MODEL}_description") / "config"
    config.mkdir()
    write_param_file(config / "vehicle_info.param.yaml", REQUIRED_VEHICLE_INFO)

    original = os.environ.get("AMENT_PREFIX_PATH", "")
    os.environ["AMENT_PREFIX_PATH"] = os.pathsep.join([str(prefix), original])
    register_vehicle_info_utils_from_source_tree(prefix)
    yield
    os.environ["AMENT_PREFIX_PATH"] = original


@pytest.fixture()
def first_param_file(tmp_path):
    return write_param_file(tmp_path / "first.param.yaml", FIRST_PARAMS)


@pytest.fixture()
def second_param_file(tmp_path):
    return write_param_file(tmp_path / "second.param.yaml", SECOND_PARAMS)


@pytest.fixture()
def launch_from_xml(tmp_path, vehicle_description_package):
    """Return a function that runs an XML launch file and returns the global parameters."""

    def run(param_files, scoped="false"):
        xml = tmp_path / "test.launch.xml"
        xml.write_text(
            "<launch>\n"
            f'  <group scoped="{scoped}">\n'
            f'    <include file="{GLOBAL_PARAMS_LAUNCH}">\n'
            f'      <arg name="vehicle_model" value="{VEHICLE_MODEL}"/>\n'
            '      <arg name="global_parameter_loader_param_files"'
            f" value={quoteattr(str(param_files))}/>\n"
            "    </include>\n"
            "  </group>\n"
            "</launch>\n"
        )

        root_entity, parser = Parser.load(str(xml))
        context = LaunchContext()
        visit_all_entities_and_collect_futures(parser.parse_description(root_entity), context)
        return dict(context.launch_configurations.get("global_params", []))

    return run


def optional_params(params):
    """Drop the parameters that are loaded regardless of the optional parameter files."""
    return {k: v for k, v in params.items() if k not in ALWAYS_LOADED_PARAMS}


@pytest.mark.parametrize(
    "param_files",
    ["", "   ", ",", " , ,, "],
    ids=["empty", "spaces only", "separator only", "separators and spaces"],
)
def test_no_param_file(launch_from_xml, param_files):
    assert optional_params(launch_from_xml(param_files)) == {}


def test_single_param_file(launch_from_xml, first_param_file):
    params = launch_from_xml(first_param_file)
    assert params["first"] == 1
    assert params["shared"] == "from_first"


def test_multiple_param_files(launch_from_xml, first_param_file, second_param_file):
    params = launch_from_xml(f"{first_param_file},{second_param_file}")
    assert params["first"] == 1
    assert params["second"] == 2
    assert params["shared"] == "from_second", "the value of the last file must win"


@pytest.mark.parametrize(
    "template",
    [
        "{first}, {second}",
        "{first} ,{second}",
        "{first} , {second}",
        "\t{first} ,\n {second}\n",
        "{first},{second},",
        ",{first},,{second},",
    ],
    ids=[
        "space after separator",
        "space before separator",
        "spaces around separator",
        "tabs and newlines",
        "trailing separator",
        "empty entries",
    ],
)
def test_separator_tolerance(launch_from_xml, first_param_file, second_param_file, template):
    params = launch_from_xml(template.format(first=first_param_file, second=second_param_file))
    assert params["first"] == 1
    assert params["second"] == 2
    assert params["shared"] == "from_second", "the value of the last file must win"


def test_same_param_file_twice(launch_from_xml, first_param_file):
    params = launch_from_xml(f"{first_param_file},{first_param_file}")
    assert params["first"] == 1
    assert params["shared"] == "from_first"


def test_only_existing_param_files_are_loaded(launch_from_xml, first_param_file, tmp_path):
    params = launch_from_xml(f"{tmp_path / 'does_not_exist.param.yaml'},{first_param_file}")
    assert params["first"] == 1


def test_symbolic_link_is_followed(launch_from_xml, first_param_file, tmp_path):
    link = tmp_path / "link.param.yaml"
    link.symlink_to(first_param_file)
    assert launch_from_xml(link)["first"] == 1


def test_path_containing_a_space(launch_from_xml, tmp_path):
    spaced = write_param_file(tmp_path / "with space.param.yaml", FIRST_PARAMS)
    assert launch_from_xml(spaced)["first"] == 1


def test_relative_path(launch_from_xml, first_param_file, monkeypatch):
    """A relative path works, but it depends on the working directory, so it is discouraged."""
    monkeypatch.chdir(first_param_file.parent)
    assert launch_from_xml(first_param_file.name)["first"] == 1


def broken_symbolic_link(tmp_path):
    link = tmp_path / "broken.param.yaml"
    link.symlink_to(tmp_path / "does_not_exist.param.yaml")
    return link


@pytest.mark.parametrize(
    "make_param_files",
    [
        lambda tmp_path: tmp_path / "does_not_exist.param.yaml",
        lambda tmp_path: tmp_path,
        broken_symbolic_link,
        lambda tmp_path: write_param_file(tmp_path / "with,separator.param.yaml", FIRST_PARAMS),
        lambda tmp_path: "~/first.param.yaml",
        lambda tmp_path: "$PARAM_DIR/first.param.yaml",
        lambda tmp_path: f'"{write_param_file(tmp_path / "quoted.param.yaml", FIRST_PARAMS)}"',
    ],
    ids=[
        "non-existent file",
        "directory",
        "broken symbolic link",
        # the remaining paths point at an existing file, but cannot be resolved as given
        "path containing the separator",
        "home directory, which is not expanded",
        "environment variable, which is not expanded",
        "quoted path",
    ],
)
def test_unusable_path_is_skipped(launch_from_xml, tmp_path, monkeypatch, make_param_files):
    """A path that cannot be resolved to a file is skipped instead of aborting the launch."""
    write_param_file(tmp_path / "first.param.yaml", FIRST_PARAMS)
    monkeypatch.setenv("PARAM_DIR", str(tmp_path))

    assert optional_params(launch_from_xml(make_param_files(tmp_path))) == {}


@pytest.mark.parametrize(
    "content",
    [
        "",
        "some_node:\n  ros__parameters:\n    x: 1\n",
        "/**:\n  other: 1\n",
        "just a string\n",
        "/**:\n  ros__parameters:\n   - [unclosed\n",
    ],
    ids=["empty", "no /** root", "no ros__parameters", "scalar", "broken syntax"],
)
def test_invalid_param_file_reports_its_path(launch_from_xml, tmp_path, content):
    invalid = tmp_path / "invalid.param.yaml"
    invalid.write_text(content)

    with pytest.raises(RuntimeError, match=re.escape(str(invalid))):
        launch_from_xml(invalid)


def test_unreadable_param_file_reports_its_path(launch_from_xml, tmp_path):
    unreadable = write_param_file(tmp_path / "unreadable.param.yaml", FIRST_PARAMS)
    unreadable.chmod(0)
    if os.access(unreadable, os.R_OK):
        pytest.skip("a privileged user can read a file without the read permission")

    with pytest.raises(RuntimeError, match=re.escape(str(unreadable))):
        launch_from_xml(unreadable)


def test_parameters_are_not_set_in_a_scoped_group(launch_from_xml, first_param_file):
    """The parameters must reach the enclosing scope, so the launch file must not scope them."""
    assert launch_from_xml(first_param_file, scoped="true") == {}
