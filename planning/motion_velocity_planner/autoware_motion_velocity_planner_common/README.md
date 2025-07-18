# Motion Velocity Planner Common

This package provides common utilities and data structures for the motion velocity planner in the Autoware system. It contains tools for geometric calculations, trajectory processing, and velocity planning results.

## Overview

Package motion velocity planner is responsible for generating velocity profiles for autonomous vehicles based on the current trajectory and environment. This package `autoware_motion_velocity_planner_common` contains essential utilities and structures that support the planning process, including:

- Geometric operations for polygons and trajectories.
- General utilities for trajectory processing and visualization.
- Data structures for storing velocity planning results.

## Design

### Key Components

1. **Polygon Utilities (`polygon_utils.hpp`)**
   This component provides functions for handling geometric polygons related to motion planning. It includes:
   - Collision detection between trajectories and obstacles.
   - Creation of polygons representing the vehicle's trajectory at different time steps.
   - Geometric calculations using Boost Geometry.

2. **General Utilities (`utils.hpp`)**
   This component provides various utility functions, including:
   - Conversion between point representations (e.g., `pcl::PointXYZ` to `geometry_msgs::msg::Point`).
   - Distance calculations between trajectory points and obstacles.
   - Functions for concatenating vectors and processing trajectories.
   - Visualization tools for creating markers.

3. **Velocity Planning Results (`velocity_planning_result.hpp`)**
   This component defines data structures for storing the results of velocity planning, including:
   - `SlowdownInterval`: Represents a segment where the vehicle should slow down, with specified start and end points and velocity.
   - `VelocityPlanningResult`: Contains a collection of stop points, slowdown intervals, and optional velocity limits and clear commands.

## Usage

### Including the Package

To use this package in your project, you need to include it in your CMakeLists.txt:

```cmake
find_package(autoware_motion_velocity_planner_common REQUIRED)
```

### Example Usage

Here's a simple example demonstrating the use of the utility functions:

```cpp
#include "autoware/motion_velocity_planner_common/utils.hpp"

const auto decimated_traj_points = autoware::motion_velocity_planner::utils::decimate_trajectory_points_from_ego(
    traj_points, current_pose, ego_nearest_dist_threshold, ego_nearest_yaw_threshold,
    p.decimate_trajectory_step_length, 0.0);
```

this example is from autoware_universe/planning/motion_velocity_planner/autoware_motion_velocity_obstacle_cruise_module/src/obstacle_cruise_module.cpp
