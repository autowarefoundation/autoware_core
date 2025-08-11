# Path Generator

The `path_generator` node receives a route from `mission_planner` and converts the center line into a path.
If the route has waypoints set, it generates a path passing through them.

This package is a simple alternative of `behavior_path_generator`.

## Path generation

When input data is ready, it first searches for the lanelet closest to the vehicle.
If found, it gets the lanelets within a distance of `path_length.backward` behind and `path_length.forward` in front.
Their center lines are concatenated to generate a path.

If waypoints exist in the route, it replaces the overlapped segment of the center line with them.
The waypoints are grouped as shown in the following figure.

![waypoint_grouping](./media/waypoint_grouping.drawio.svg)

## Path cut

If there is a self-intersection on either of the path bounds, the path is cut off a specified distance before the first intersection, as shown in the following figure (path: green, bound: blue).

![path_cut_self_intersection](./media/path_cut_self_intersection.drawio.svg)

Depending on the crossing angle, the return path's bound may be closer to the centerline than the outward's one, depicted in the diagram below. To deal with this, intersections of the left and right bounds (mutual intersection) are taken into account as well, and the path is cut at the nearest intersecting point.

![path_cut_mutual_intersection](./media/path_cut_mutual_intersection.drawio.svg)

Furthermore, in the case of the following figure, the return path goes inside even if mutual intersection is considered. Therefore, path cut is also made when the start edge of the path and the path bounds intersect.

![path_cut_start_edge_intersection](./media/path_cut_start_edge_intersection.drawio.svg)

## Goal connection

The path is connected to the goal smoothly in the following way:

(a) The path has not reached the lane where the goal is placed, thus goal connection is not performed.

(b) The path has reached the goal lane and the end is inside the connection section. However, the longitudinal position of the path end is still in front of the goal, so it does not connect the path with the goal.

(c) The path has passed the goal (in terms of longitudinal position). In this case, the original path is cropped up to the connection section, and it is connected to the pre-goal and goal, sequentially. The pre-goal is inserted before the goal with the given offset, which helps align with the goal pose.

(d) If the start of the path is already inside the connection section, it just connects the path start, pre-goal, and goal.

![goal_connection](./media/goal_connection.drawio.svg)

## Turn signal

Turn signal is determined based on the rules defined for behavior_path_planner. (See [here](https://autowarefoundation.github.io/autoware_universe/main/planning/behavior_path_planner/autoware_behavior_path_planner_common/docs/behavior_path_planner_turn_signal_design/) for details)

As a general rule, the turn signal is turned on at a specified distance before the lanelet in which a turn is designated, and turned off when the driving direction has changed to the specified degree.

![turn_signal_sections](./media/turn_signal_sections.drawio.svg)

If consecutive turns are required, the turn signal corresponding to the required section or the last section takes precedence.

![turn_signal_conflict](./media/turn_signal_conflict.drawio.svg)

## Hazard signal

This node always publishes a hazard signal of `autoware_vehicle_msgs::msg::HazardLightsCommand::NO_COMMAND`.

## Flowchart

```plantuml
@startuml
title run
start

:take_data;
:set_planner_data;
if (is_data_ready) then (yes)
else (no)
  stop
endif

group plan_path
  group generate_path
    :update_current_lanelet;
    :get lanelets within route;
    if (path bounds have intersections?) then (yes)
      :align path end with intersection point;
    endif
    while (path end is outside range of lanelets?)
      :extend lanelets forward;
    endwhile
    while (path start is outside range of lanelets?)
      :extend lanelets backward;
    endwhile
    if (any waypoint interval starts behind lanelets?) then (yes)
      :extend lanelets backward;
    endif
    while (for each center line point)
      if (overlapped by waypoint group?) then (yes)
        if (previously overlapped?) then
        else (no)
          :add waypoints to path;
        endif
      else (no)
        :add point to path;
      endif
    endwhile
    :crop path;
    if (goal is in search range for smooth goal connection?) then (yes)
      :modify_path_for_smooth_goal_connection;
    endif
    :set path bounds;
  end group
end group

group get_turn_signal
  while (for each path point)
    if (turn direction is set to corresponding lanelet?) then (yes)
      if (ego is in front of lanelet?) then (yes)
        if (distance to lanelet < turn_signal_distance?) then (yes)
          :return turn signal;
        endif
      else (no)
        if (ego passed required section?) then (yes)
          :return turn signal;
        endif
      endif
    endif
  endwhile
end group

:publish turn signal;

:publish hazard signal;

:publish path;

stop
@enduml
```

## Input topics

| Name                 | Type                                        | Description                      |
| :------------------- | :------------------------------------------ | :------------------------------- |
| `~/input/odometry`   | `nav_msgs::msg::Odometry`                   | ego pose                         |
| `~/input/vector_map` | `autoware_map_msgs::msg::LaneletMapBin`     | vector map information           |
| `~/input/route`      | `autoware_planning_msgs::msg::LaneletRoute` | current route from start to goal |

## Output topics

| Name                           | Type                                                   | Description    | QoS Durability |
| :----------------------------- | :----------------------------------------------------- | :------------- | :------------- |
| `~/output/path`                | `autoware_internal_planning_msgs::msg::PathWithLaneId` | generated path | `volatile`     |
| `~/output/turn_indicators_cmd` | `autoware_vehicle_msgs::msg::TurnIndicatorsCommand`    | turn signal    | `volatile`     |
| `~/output/hazard_lights_cmd`   | `autoware_vehicle_msgs::msg::HazardLightsCommand`      | hazard signal  | `volatile`     |

## Parameters

{{ json_to_markdown("planning/autoware_path_generator/schema/path_generator.schema.json") }}

In addition, the following parameters should be provided to the node:

- [nearest search parameters](https://github.com/autowarefoundation/autoware_launch/blob/main/autoware_launch/config/planning/scenario_planning/common/nearest_search.param.yaml)
- [vehicle info parameters](https://github.com/autowarefoundation/sample_vehicle_launch/blob/main/sample_vehicle_description/config/vehicle_info.param.yaml)
