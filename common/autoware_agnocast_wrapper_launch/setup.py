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

from setuptools import setup

package_name = "autoware_agnocast_wrapper_launch"

setup(
    name=package_name,
    version="1.9.0",
    packages=[package_name, f"{package_name}.actions"],
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Koichi Imai",
    maintainer_email="koichi.imai.2@tier4.jp",
    description="Launch actions for nodes that can run standalone or inside a component container",
    license="Apache License 2.0",
    tests_require=["pytest"],
    entry_points={
        "launch.frontend.launch_extension": [
            f"{package_name} = {package_name}",
        ],
    },
)
