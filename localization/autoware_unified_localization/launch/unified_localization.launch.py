# Copyright 2025 Autoware Foundation
# Licensed under the Apache License, Version 2.0.

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    default_param = PathJoinSubstitution([
        FindPackageShare("autoware_unified_localization"),
        "config",
        "unified_localization.param.yaml",
    ])

    declare_param_file = DeclareLaunchArgument(
        "param_file",
        default_value=default_param,
        description="Path to the unified localization parameter file.",
    )
    declare_input_pose = DeclareLaunchArgument(
        "input_pose_with_covariance",
        default_value="in_pose_with_covariance",
        description="Input pose with covariance topic (e.g. from NDT).",
    )
    declare_input_twist = DeclareLaunchArgument(
        "input_twist_with_covariance",
        default_value="in_twist_with_covariance",
        description="Input twist with covariance topic (e.g. from gyro_odometer).",
    )
    declare_output_kinematic = DeclareLaunchArgument(
        "output_kinematic_state",
        default_value="kinematic_state",
        description="Output kinematic state (Odometry) topic.",
    )
    declare_output_accel = DeclareLaunchArgument(
        "output_acceleration",
        default_value="acceleration",
        description="Output acceleration topic.",
    )

    localization_node = Node(
        package="autoware_unified_localization",
        executable="localization_node",
        name="localization_node",
        output="both",
        parameters=[LaunchConfiguration("param_file")],
        remappings=[
            ("in_pose_with_covariance", LaunchConfiguration("input_pose_with_covariance")),
            ("in_twist_with_covariance", LaunchConfiguration("input_twist_with_covariance")),
            ("kinematic_state", LaunchConfiguration("output_kinematic_state")),
            ("acceleration", LaunchConfiguration("output_acceleration")),
        ],
    )

    return LaunchDescription([
        declare_param_file,
        declare_input_pose,
        declare_input_twist,
        declare_output_kinematic,
        declare_output_accel,
        localization_node,
    ])
