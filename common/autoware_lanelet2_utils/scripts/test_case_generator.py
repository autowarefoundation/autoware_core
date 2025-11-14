#!/usr/bin/env python3

# Copyright 2025 TIER IV, Inc.
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

from argparse import ArgumentParser
import copy
from dataclasses import dataclass
from dataclasses import field
import math
from pathlib import Path
from tkinter import filedialog
from typing import List
from typing import Optional
from typing import Tuple

from ament_index_python.packages import get_package_share_directory
from autoware_lanelet2_extension_python.projection import MGRSProjector
from geometry_msgs.msg import Point
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Quaternion
import lanelet2
from matplotlib.patches import Polygon
import matplotlib.pyplot as plt
from matplotlib.widgets import Button
import numpy as np
from rclpy_message_converter import message_converter
from rosidl_runtime_py import message_to_yaml
from tf_transformations import euler_from_quaternion
from tf_transformations import quaternion_from_euler
import yaml


def draw_centerline_arrow(ax, centerline):
    size = len(centerline)
    mid = int(size / 2)
    start = max(0, mid - 1)
    end = max(size - 1, mid + 1)
    start_pt = centerline[start].basicPoint()
    end_pt = centerline[end].basicPoint()
    mid_point = (start_pt + end_pt) * 0.5
    diff = end_pt - start_pt
    theta = math.atan2(diff.y, diff.x)
    ax.quiver(
        [mid_point.x],
        [mid_point.y],
        [math.cos(theta)],
        [math.sin(theta)],
        angles="xy",
        scale_units="xy",
        scale=1,
    )


def load_properly_projected_map(map_path, projector):
    dry_run_map = lanelet2.io.load(map_path, projector)
    for point in dry_run_map.pointLayer:
        projector.reverse(lanelet2.core.BasicPoint3d(point.x, point.y, point.z))
        break
    projector.setMGRSCode(projector.getProjectedMGRSGrid())
    return lanelet2.io.load(map_path, projector)


class LaneletVisualizationHandler:
    def __init__(self, fig, ax, map_path) -> None:
        projector = MGRSProjector()
        lanelet_map = load_properly_projected_map(map_path, projector)

        self.fig, self.ax = fig, ax
        self.ax.set_aspect("equal")
        self.ax.grid(True, alpha=0.3)

        self.fig.patch.set_facecolor("white")
        self.ax.set_facecolor("white")
        self.fig.subplots_adjust(left=0, right=1, top=1, bottom=0, wspace=0, hspace=0)

        self.lanelet_polygon_patches = []
        self.lanelet_annot = self.ax.annotate(
            "",
            xy=(0, 0),
            xytext=(10, 10),  # cSpell:ignore xytext
            textcoords="offset points",  # cSpell:ignore textcoords
            bbox={"boxstyle": "round", "fc": "w"},  # cSpell:ignore boxstyle
            arrowprops={"arrowstyle": "->"},  # cSpell:ignore arrowstyle, arrowprops
        )
        self.lanelet_annot.set_visible(False)
        self.fig.canvas.mpl_connect("motion_notify_event", self.hover_on_lanelet)
        for lanelet in lanelet_map.laneletLayer:
            self.draw_lanelet_as_polygon(lanelet)
        self.ax.relim()  # cSpell:ignore relim
        self.ax.autoscale_view()

    def draw_lanelet_as_polygon(self, lanelet):
        colormap = {
            "road": "lightgray",
            "road_shoulder": "darkgray",
            "bicycle_lane": "steelblue",
            "crosswalk": "darkgray",
        }
        subtype = lanelet.attributes["subtype"]
        left_boundary = lanelet.leftBound
        right_boundary = lanelet.rightBound

        left_points = [(pt.x, pt.y) for pt in left_boundary]
        right_points = [(pt.x, pt.y) for pt in reversed(right_boundary)]
        left_points.extend(right_points)
        p = (
            Polygon(
                left_points,
                closed=True,
                facecolor=colormap[subtype],
                edgecolor="k",
                alpha=0.8,
                hatch="//",
                linewidth=0.5,
            )
            if subtype == "crosswalk"
            else Polygon(
                left_points,
                closed=True,
                facecolor=colormap[subtype],
                edgecolor="black",
                alpha=0.8,
                linewidth=0.5,
            )
        )
        self.ax.add_patch(p)
        self.lanelet_polygon_patches.append((p, lanelet.id))

        centerline = lanelet.centerline
        center_x = [pt.x for pt in centerline]
        center_y = [pt.y for pt in centerline]
        self.ax.plot(center_x, center_y, "k--", linewidth=1, alpha=0.5)
        draw_centerline_arrow(self.ax, centerline)

    def hover_on_lanelet(self, event):
        if event.inaxes == self.ax:
            for patch, lane_id in self.lanelet_polygon_patches:
                if patch.contains_point((event.x, event.y)):
                    self.lanelet_annot.xy = (event.xdata, event.ydata)
                    self.lanelet_annot.set_text(f"id = {lane_id}")
                    self.lanelet_annot.set_visible(True)
                    self.fig.canvas.draw_idle()
                    return
        self.lanelet_annot.set_visible(False)
        self.fig.canvas.draw_idle()


@dataclass
class PoseArrowState:
    arrow_mode: bool = False
    _start_point: Optional[Tuple[float, float]] = None
    _added_arrow_points: List[Pose] = field(default_factory=list)
    _added_arrow_artists: List[plt.Artist] = field(default_factory=list)
    _drawing_arrow_artist: Optional[plt.Artist] = None

    def __del__(self):
        for artist in self._added_arrow_artists:
            artist.remove()

    def get_added_points(self):
        return copy.deepcopy(self._added_arrow_points)

    def set_start_point(self, x, y):
        self._start_point = (x, y)

    def update_drawing_arrow(self, ax, x, y):
        if not self._start_point:
            return
        if self._drawing_arrow_artist:
            self._drawing_arrow_artist.remove()
        x0 = self._start_point[0]
        y0 = self._start_point[1]
        dx = x - x0
        dy = y - y0
        self._drawing_arrow_artist = ax.arrow(
            x0,
            y0,
            dx,
            dy,
            head_width=0.2,
            head_length=0.3,
            fc="r",
            ec="r",
        )

    def commit_drawing_arrow(self, ax, x, y):
        if not self._start_point:
            return
        # clear drawing arrow
        if self._drawing_arrow_artist:
            self._drawing_arrow_artist.remove()
        self._drawing_arrow_artist = None
        # add new arrow
        x0 = self._start_point[0]
        y0 = self._start_point[1]
        dx = x - x0
        dy = y - y0
        self._added_arrow_artists.append(
            ax.arrow(x0, y0, dx, dy, head_width=1.0, head_length=1.0, fc="r", ec="r")
        )
        self._added_arrow_artists.append(
            ax.text(x0, y0, f"P{len(self._added_arrow_points)}", color="red", fontsize=10)
        )
        yaw = np.arctan2(dy, dx)
        (qx, qy, qz, qw) = quaternion_from_euler(0.0, 0.0, yaw)
        self._added_arrow_points.append(
            Pose(
                position=Point(x=x0, y=y0, z=100.0),
                orientation=Quaternion(x=qx, y=qy, z=qz, w=qw),
            )
        )
        self._start_point = None


@dataclass
class SaveContext:
    map_rel_path: Optional[Path] = None
    manual_poses: List[Pose] = field(default_factory=list)

    def dump_v1(self, map_rel_path, dump_path):
        data = {}
        data["version"] = 1
        data["map_rel_path"] = str(map_rel_path)
        data["manual_poses"] = [yaml.safe_load(message_to_yaml(pose)) for pose in self.manual_poses]
        with open(dump_path, "w") as f:
            f.write(
                """# Auto-generated by running `scripts/test_case_generator.py --generate`
#
# Run `scripts/test_case_generator.py --view <this file name>` to view the test context
#
# Run `scripts/test_case_generator.py --edit <this file name>` to modify the test context
#
# format1
#
# version: 1 <int>
# manual_poses: <array of geometry_msgs>
#
# format2(TBD)

"""
            )
            yaml.dump(data, f, encoding="utf-8", allow_unicode=True)

    @staticmethod
    def load(path: str) -> "SaveContext":
        with open(path, "r") as f:
            data = yaml.safe_load(f)
            if data["version"] == 1:
                return SaveContext.load_v1(data, path)
        raise RuntimeError(f"""version {data["version"]} is not supported""")

    @staticmethod
    def load_v1(data, path: str) -> "SaveContext":
        map_rel_path = data["map_rel_path"]

        manual_poses = []
        for pose_yml in data["manual_poses"]:
            pose = message_converter.convert_dictionary_to_ros_message(
                "geometry_msgs/msg/Pose", pose_yml
            )
            manual_poses.append(pose)

        return SaveContext(map_rel_path, manual_poses)


class ContextSaveHandler:
    def __init__(self, fig, ax, button_ax, map_rel_path) -> None:
        self.fig, self.ax = fig, ax
        self.map_rel_path = map_rel_path

        self.button = Button(button_ax, "Save context")
        self.button.on_clicked(self.on_save)

    def on_save(self, event):
        filename = filedialog.asksaveasfilename()
        global shared_ctx
        shared_ctx.dump_v1(self.map_rel_path, filename)


class UserProvidedPointsVisualizer:
    def __init__(self, fig, ax) -> None:
        self.fig, self.ax = fig, ax

        global shared_ctx
        for i, pose in enumerate(shared_ctx.manual_poses):
            x0, y0 = pose.position.x, pose.position.y
            quat = pose.orientation
            yaw = euler_from_quaternion([quat.x, quat.y, quat.z, quat.w])[2]
            length = 2.0
            dx, dy = length * math.cos(yaw), length * math.sin(yaw)
            self.ax.arrow(x0, y0, dx, dy, head_width=1.0, head_length=1.0, fc="r", ec="r")
            self.ax.text(x0, y0, f"P{i}", color="red", fontsize=10)


class UserProvidedPointsHandler:
    def __init__(self, fig, ax, button_ax) -> None:
        self.fig, self.ax = fig, ax

        self.drawing_state = PoseArrowState()

        self.button = Button(button_ax, "Manual pose mode")
        self.deactivate_button()
        self.button.on_clicked(self.toggle_mode)

        self.added_points = []

        self.fig.canvas.mpl_connect("button_press_event", self.on_press_position)
        self.fig.canvas.mpl_connect("motion_notify_event", self.during_motion)
        self.fig.canvas.mpl_connect("button_release_event", self.on_release_motion)

    def deactivate_button(self) -> None:
        self.button.label.set_text("Manual pose mode")
        self.button.hovercolor = self.button.color = "gray"  # cSpell:ignore hovercolor
        self.drawing_state = PoseArrowState()
        global shared_ctx
        shared_ctx.manual_poses = []

    def toggle_mode(self, event: object) -> None:
        self.drawing_state.arrow_mode = not self.drawing_state.arrow_mode
        if self.drawing_state.arrow_mode:
            self.button.label.set_text("Drag to get pose / Press to clear")
            self.button.hovercolor = self.button.color = "red"  # cSpell:ignore hovercolor
        else:
            self.deactivate_button()
        self.fig.canvas.draw()

    def on_press_position(self, event) -> None:
        if not self.drawing_state.arrow_mode or event.inaxes != self.ax:
            return
        self.drawing_state.set_start_point(event.xdata, event.ydata)

    def during_motion(self, event) -> None:
        if not self.drawing_state.arrow_mode or event.inaxes != self.ax:
            return
        self.drawing_state.update_drawing_arrow(self.ax, event.xdata, event.ydata)
        self.fig.canvas.draw_idle()

    def on_release_motion(self, event) -> None:
        if not self.drawing_state.arrow_mode or event.inaxes != self.ax:
            return
        self.drawing_state.commit_drawing_arrow(self.ax, event.xdata, event.ydata)
        global shared_ctx
        shared_ctx.manual_poses = self.drawing_state.get_added_points()


shared_ctx = SaveContext(None, [])


def main():
    parser = ArgumentParser()
    group = parser.add_mutually_exclusive_group()
    group.add_argument("--generate", action="store_true")
    group.add_argument("--view", help="path to test data file")
    group.add_argument("--view-map", help="relative path to lanelet2 map")
    group.add_argument("--edit", help="path to test data file")
    args = parser.parse_args()
    if args.generate:
        pkg_dir = Path(get_package_share_directory("autoware_lanelet2_utils"))
        sample_map_dir = pkg_dir / "sample_map"
        map_path = filedialog.askopenfilenames(initialdir=sample_map_dir)[
            0
        ]  # cSpell:ignore initialdir
        map_rel_path = Path(map_path).relative_to(sample_map_dir)

        fig, ax = plt.subplots()
        plt.subplots_adjust(right=0.8)

        lanelet_visualizer = LaneletVisualizationHandler(fig, ax, map_path)  # noqa

        pose_button = plt.axes([0.82, 0.7, 0.15, 0.08])
        point_handler = UserProvidedPointsHandler(fig, ax, pose_button)  # noqa

        save_button = plt.axes([0.82, 0.6, 0.15, 0.08])
        save_handler = ContextSaveHandler(fig, ax, save_button, map_rel_path)  # noqa

        plt.show()

    if args.view:
        data_path = args.view
        ctx = SaveContext.load(data_path)
        if ctx.map_rel_path is None:
            return

        global shared_ctx
        shared_ctx.manual_poses = ctx.manual_poses

        fig, ax = plt.subplots()
        plt.subplots_adjust(right=0.8)

        lanelet_visualizer = LaneletVisualizationHandler(  # noqa
            fig,
            ax,
            str(
                Path(get_package_share_directory("autoware_lanelet2_utils"))
                / "sample_map"
                / ctx.map_rel_path
            ),
        )

        point_visualizer = UserProvidedPointsVisualizer(fig, ax)  # noqa

        plt.show()

    if args.view_map:
        fig, ax = plt.subplots()
        plt.subplots_adjust(right=0.8)
        map_path = args.view_map
        lanelet_visualizer = LaneletVisualizationHandler(fig, ax, args.view_map)  # noqa
        plt.show()

    if args.edit:
        print("--edit option is not currently supported")
        return


if __name__ == "__main__":
    main()
