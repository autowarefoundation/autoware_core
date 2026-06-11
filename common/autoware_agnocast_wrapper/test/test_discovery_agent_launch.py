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

"""Tests for discovery_agent.launch.py's tree-wide deduplication.

These exercise the spawn helper against a real ``LaunchContext`` — no agent
executable, kmod or ``ros2 launch`` is needed.
"""

# cspell:ignore delenv

import importlib.util
import os

from launch import LaunchContext
from launch_ros.actions import Node
import pytest


def _load():
    path = os.path.join(os.path.dirname(__file__), "..", "launch", "discovery_agent.launch.py")
    spec = importlib.util.spec_from_file_location("discovery_agent_launch", path)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


@pytest.fixture
def mod():
    return _load()


def test_spawns_one_agent_per_launch_tree(mod, monkeypatch):
    """Within one launch tree (shared context) only the first include spawns."""
    monkeypatch.setenv("ENABLE_AGNOCAST", "1")
    ctx = LaunchContext()
    first = mod._spawn_once(ctx)
    second = mod._spawn_once(ctx)
    third = mod._spawn_once(ctx)
    assert len(first) == 1
    assert isinstance(first[0], Node)
    assert second == []
    assert third == []


def test_separate_launch_trees_each_spawn(mod, monkeypatch):
    """A fresh context (separate ros2 launch invocation) spawns again."""
    monkeypatch.setenv("ENABLE_AGNOCAST", "1")
    assert len(mod._spawn_once(LaunchContext())) == 1
    assert len(mod._spawn_once(LaunchContext())) == 1


def test_no_spawn_when_agnocast_disabled(mod, monkeypatch):
    """Nothing is spawned unless ENABLE_AGNOCAST is exactly "1"."""
    monkeypatch.setenv("ENABLE_AGNOCAST", "0")
    assert mod._spawn_once(LaunchContext()) == []
    monkeypatch.delenv("ENABLE_AGNOCAST", raising=False)
    assert mod._spawn_once(LaunchContext()) == []
