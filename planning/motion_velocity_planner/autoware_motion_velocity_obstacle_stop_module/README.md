# autoware_motion_velocity_obstacle_stop_module

## Overview

The `obstacle_stop` module does the stop planning when there is a static obstacle near the trajectory.

## Design

### Obstacle Filtering

The obstacles meeting the following condition are determined as obstacles for stopping.

- The object type is for stopping according to `obstacle_filtering.object_type.*`.
- The lateral distance from the object to the ego's trajectory is smaller than `obstacle_filtering.max_lat_margin`.
  - `obstacle_filtering.max_lat_margin_against_predicted_object_unknown` is applied to the predicted object of unknown.
  - For the 'outside' objects, objects' future poses until 'obstacle_filtering.outside_obstacle.estimation_time_horizon' is also considered.
- The object
  - does not cross the ego's trajectory (\*1)
  - and its collision time margin is large enough (\*2)
  - when `obstacle_filtering.ignore_crossing_obstacle` is false.

#### NOTE

##### \*1: Crossing obstacles

Crossing obstacle is the object whose orientation's yaw angle against the ego's trajectory is smaller than `obstacle_filtering.crossing_obstacle.obstacle_traj_angle_threshold`.

##### \*2: Enough collision time margin

We predict the collision area and its time by the ego with a constant velocity motion and the obstacle with its predicted path.
Then, we calculate a collision time margin which is the difference of the time when the ego will be inside the collision area and the obstacle will be inside the collision area.
When this time margin is smaller than `obstacle_filtering.crossing_obstacle.collision_time_margin`, the margin is not enough.

### Stop Planning

The role of the stop planning is keeping a safe distance with static vehicle objects or dynamic/static non vehicle objects.

The stop planning just inserts the stop point in the trajectory to keep a distance with obstacles.
The safe distance is parameterized as `stop_planning.stop_margin`.
When it stops at the end of the trajectory, and obstacle is on the same point, the safe distance becomes `stop_planning.terminal_stop_margin`.

When inserting the stop point, the required acceleration for the ego to stop in front of the stop point is calculated.
If the acceleration is less than `common.min_strong_accel`, the stop planning will be cancelled since this package does not assume a strong sudden brake for emergency.

### Leading Vehicle Following by RSS Stop Position Determination

This function enables distance adjustment with moving leading vehicles in the obstacle_stop module as well.
While leading vehicle following is primarily handled by the obstacle_cruise_module, this function provides a supplementary role.

The operating principle is to assume a constant deceleration for the leading vehicle and predict the stopping point when the leading vehicle stops with a certain deceleration. Then, the logic embeds a stop point in the trajectory with the usual stop margin against that leading vehicle's stopping point.
Leading vehicle following is realized through the cooperation of this function and the velocity_smoother.
Please note that the ego vehicle's behavior is influenced not only by the obstacle_stop settings but also by the velocity_smoother settings.

For objects moving slower than `obstacle_velocity_threshold_enter_fixed_stop`, the braking distance is not calculated. Instead, only the object's current position is used for stop position calculation.
Additionally, if this feature is disabled by `use_rss_stop`, stop planning is performed only for objects moving slower than `obstacle_velocity_threshold_enter_fixed_stop`.

### Minor functions

#### Prioritization of behavior module's stop point

When stopping for a pedestrian walking on the crosswalk, the behavior module inserts the zero velocity in the trajectory in front of the crosswalk.
Also `obstacle_cruise_module`'s stop planning also works, and the ego may not reach the behavior module's stop point since the safe distance defined in `obstacle_cruise_module` may be longer than the behavior module's safe distance.
To resolve this non-alignment of the stop point between the behavior module and this module, `stop_planning.min_behavior_stop_margin` is defined.
In the case of the crosswalk described above, this module inserts the stop point with a distance `stop_planning.min_behavior_stop_margin` at minimum between the ego and obstacle.

#### Holding the closest stop obstacle in target obstacles

In order to keep the closest stop obstacle in the target obstacles against the perception's detection instability, we check whether it is disappeared or not from the target obstacles in the `check_consistency` function.
If the previous closest stop obstacle is removed from the lists, we keep it in the lists for `obstacle_filtering.stop_obstacle_hold_time_threshold` seconds.
Note that if a new stop obstacle appears and the previous closest obstacle removes from the lists, we do not add it to the target obstacles again.

#### Holding the previous stop point if necessary

When the ego stops for the front obstacle or pointcloud, even though the obstacle does not move, the detected position of the obstacle may change due to the perception's detection noise. In this case, the ego will start driving a little bit.
In order to avoid the ego to restart, we keep the previous stop point if the ego's velocity is less than `stop_planning.hold_stop_velocity_threshold` and the distance between the current and previous stop points is less than `stop_planning.hold_stop_distance_threshold`.

#### Stop point adjustment on a curve

When the ego stops on a curve road, since the stop margin is a bit long by default for the curve road, as shown in the following figure, the object B will come between the ego and object A, which may be dangerous.
To avoid this cut-in, the feature to use the shorter stop margin than usual on a curve can be used by enabling `stop_planning.stop_on_curve.enable_approaching`.

The following figure shows the logic.

- First, calculate the blue point where the straight driving footprint of the each future ego's pose collides with the front object (object A).
- Then, calculate the stop point `stop_planning.stop_on_curve.additional_stop_margin` behind the above blue ego's pose.
  - `stop_planning.stop_on_curve.min_stop_margin` will be kept at minimum between the ego and object.

![stop_on_curve](./docs/stop_on_curve.drawio.svg)

#### Sudden stop suppression

- Enabling `obstacle_filtering.suppress_sudden_stop` will make the deceleration for the stop higher than `limit_min_acc` by force to suppress the sudden obstacle stop.

## Visualization

### Detection area

Green polygons which is a detection area is visualized by `detection_polygons` in the `~/debug/marker` topic.

![detection_area](./docs/detection_area.png)

### Collision points

Red points which are collision points with obstacle are visualized by `*_collision_points` for each behavior in the `~/debug/marker` topic.

![collision_point](./docs/collision_point.png)

### Obstacle for stop

Red sphere which is an obstacle for stop is visualized by `obstacles_to_stop` in the `~/debug/marker` topic.

Red wall which means a safe distance to stop if the ego's front meets the wall is visualized in the `~/virtual_wall` topic.

![stop_visualization](./docs/stop_visualization.png)

## Usage

This module is activated if the launch parameter `launch_obstacle_stop_module` is set to true.
e.g. launcher/autoware_launch/autoware_launch/config/planning/preset/default_preset.yaml

```yaml
# motion velocity planner modules
- arg:
    name: launch_obstacle_stop_module
    default: "true"
```

That will trigger autoware_universe/launch/tier4_planning_launch/launch/scenario_planning/lane_driving/motion_planning/motion_planning.launch.xml to assemble launch config for motion_velocity_planner.

Finally motion_velocity_planner will load `obstacle_stop_module` as plugin.

```xml
<!-- assemble launch config for motion velocity planner -->
  <arg name="motion_velocity_planner_launch_modules" default="["/>
  <let
    name="motion_velocity_planner_launch_modules"
    value="$(eval &quot;'$(var motion_velocity_planner_launch_modules)' + 'autoware::motion_velocity_planner::ObstacleStopModule, '&quot;)"
    if="$(var launch_obstacle_stop_module)"
  />
```
