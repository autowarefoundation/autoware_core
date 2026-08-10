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
from xml.sax.saxutils import quoteattr

from ament_index_python.packages import get_package_share_directory
from ament_index_python.packages import PackageNotFoundError
from launch import LaunchContext
from launch.frontend import Parser
from launch.utilities import visit_all_entities_and_collect_futures
import pytest

VEHICLE_MODEL = "test_global_params"
# vehicle_info is not under test here, but the launch file always loads it, so a minimal
# parameter file has to exist. Its parameters are excluded from the assertions below.
REQUIRED_VEHICLE_INFO = "/**:\n  ros__parameters:\n    wheel_radius: 0.39\n"
ALWAYS_LOADED_PARAMS = {"use_sim_time", "wheel_radius"}
FIRST_PARAMS = "/**:\n  ros__parameters:\n    first: 1\n    shared: from_first\n"
SECOND_PARAMS = "/**:\n  ros__parameters:\n    second: 2\n    shared: from_second\n"


def find_global_params_launch():
    """Return the path of the launch file under test, from the source tree or the install space."""
    in_source_tree = Path(__file__).resolve().parent.parent / "launch" / "global_params.launch.py"
    if in_source_tree.is_file():
        return in_source_tree
    share = get_package_share_directory("autoware_global_parameter_loader")
    return Path(share) / "launch" / "global_params.launch.py"


def register_package(prefix, package, share_dir):
    """Make a package findable through the ament index of the given prefix."""
    marker = prefix / "share" / "ament_index" / "resource_index" / "packages" / package
    marker.parent.mkdir(parents=True, exist_ok=True)
    marker.write_text("")
    share_dir.mkdir(parents=True, exist_ok=True)


def register_vehicle_info_utils_from_source_tree(prefix):
    """Fall back to the source tree when the workspace is not built and sourced."""
    package = "autoware_vehicle_info_utils"
    try:
        get_package_share_directory(package)
        return
    except PackageNotFoundError:
        pass

    source = Path(__file__).resolve().parents[2] / package
    if not (source / "launch" / "vehicle_info.launch.py").is_file():
        pytest.skip(f"{package} is neither installed nor available in the source tree")

    share = prefix / "share" / package
    register_package(prefix, package, share)
    (share / "launch").symlink_to(source / "launch")


@pytest.fixture(scope="module", autouse=True)
def vehicle_description_package(tmp_path_factory):
    """Register the <vehicle_model>_description package that the launch file always needs."""
    prefix = tmp_path_factory.mktemp("prefix")
    package = f"{VEHICLE_MODEL}_description"

    config = prefix / "share" / package / "config"
    register_package(prefix, package, config)
    (config / "vehicle_info.param.yaml").write_text(REQUIRED_VEHICLE_INFO)

    original = os.environ.get("AMENT_PREFIX_PATH", "")
    os.environ["AMENT_PREFIX_PATH"] = os.pathsep.join([str(prefix), original])
    register_vehicle_info_utils_from_source_tree(prefix)
    yield
    os.environ["AMENT_PREFIX_PATH"] = original


@pytest.fixture()
def first_param_file(tmp_path):
    path = tmp_path / "first.param.yaml"
    path.write_text(FIRST_PARAMS)
    return path


@pytest.fixture()
def second_param_file(tmp_path):
    path = tmp_path / "second.param.yaml"
    path.write_text(SECOND_PARAMS)
    return path


@pytest.fixture()
def launch_from_xml(tmp_path):
    """Return a function that runs an XML launch file and returns the global parameters."""

    def run(param_files, scoped="false"):
        xml = tmp_path / "test.launch.xml"
        xml.write_text(
            "<launch>\n"
            f'  <group scoped="{scoped}">\n'
            f'    <include file="{find_global_params_launch()}">\n'
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


def test_no_param_file(launch_from_xml):
    assert optional_params(launch_from_xml("")) == {}


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
        "{first},{second}",
        "{first}, {second}",
        "{first} ,{second}",
        "{first} , {second}",
        "\t{first} ,\n {second}\n",
        "{first},{second},",
        ",{first},,{second},",
    ],
    ids=[
        "plain",
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


def test_symbolic_link_is_followed(launch_from_xml, first_param_file, tmp_path):
    link = tmp_path / "link.param.yaml"
    link.symlink_to(first_param_file)
    assert launch_from_xml(link)["first"] == 1


def test_path_containing_a_space(launch_from_xml, tmp_path):
    spaced = tmp_path / "with space.param.yaml"
    spaced.write_text(FIRST_PARAMS)
    assert launch_from_xml(spaced)["first"] == 1


def test_relative_path(launch_from_xml, first_param_file, monkeypatch):
    """A relative path is resolved against the working directory of the launch process."""
    monkeypatch.chdir(first_param_file.parent)
    assert launch_from_xml(first_param_file.name)["first"] == 1


@pytest.mark.parametrize(
    "param_files",
    ["", "   ", ",", " , ,, "],
    ids=["empty", "spaces only", "separator only", "separators and spaces"],
)
def test_no_param_file_variants(launch_from_xml, param_files):
    assert optional_params(launch_from_xml(param_files)) == {}


def test_non_existent_param_file_is_skipped(launch_from_xml, tmp_path):
    assert optional_params(launch_from_xml(tmp_path / "does_not_exist.param.yaml")) == {}


def test_directory_is_skipped(launch_from_xml, tmp_path):
    assert optional_params(launch_from_xml(tmp_path)) == {}


def test_broken_symbolic_link_is_skipped(launch_from_xml, tmp_path):
    link = tmp_path / "broken.param.yaml"
    link.symlink_to(tmp_path / "does_not_exist.param.yaml")
    assert optional_params(launch_from_xml(link)) == {}


def test_path_containing_the_separator_is_not_supported(launch_from_xml, tmp_path):
    """A path with a separator in it is split, so it cannot be found. This is a known limit."""
    with_separator = tmp_path / "with,separator.param.yaml"
    with_separator.write_text(FIRST_PARAMS)
    assert optional_params(launch_from_xml(with_separator)) == {}


@pytest.mark.parametrize(
    "param_files",
    ["~/first.param.yaml", "$PARAM_DIR/first.param.yaml", '"{first}"'],
    ids=["home directory", "environment variable", "quoted path"],
)
def test_paths_are_taken_literally(launch_from_xml, first_param_file, monkeypatch, param_files):
    """Neither the shell nor launch expands the given paths, so they are simply not found."""
    monkeypatch.setenv("PARAM_DIR", str(first_param_file.parent))
    assert optional_params(launch_from_xml(param_files.format(first=first_param_file))) == {}


def test_only_existing_param_files_are_loaded(launch_from_xml, first_param_file, tmp_path):
    params = launch_from_xml(f"{tmp_path / 'does_not_exist.param.yaml'},{first_param_file}")
    assert params["first"] == 1


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

    with pytest.raises(RuntimeError, match=str(invalid)):
        launch_from_xml(invalid)


@pytest.mark.skipif(os.geteuid() == 0, reason="the root user can read a file without permission")
def test_unreadable_param_file_reports_its_path(launch_from_xml, first_param_file):
    first_param_file.chmod(0)

    with pytest.raises(RuntimeError, match=str(first_param_file)):
        launch_from_xml(first_param_file)


def test_scoped_group_hides_the_parameters(launch_from_xml, first_param_file):
    """Confirm that the caller has to use <group scoped="false"> to get global parameters."""
    assert launch_from_xml(first_param_file, scoped="true") == {}
