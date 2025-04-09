# Path Generator

The `path_generator` node receives a route from `mission_planner` and converts the center line into a path.
If the route has waypoints set, it generates a path passing through them.

This package is a simple alternative of `behavior_path_generator`.

## Path generation

When input data is ready, it first searches for the lanelet closest to the vehicle.
If found, it gets the lanelets within a distance of `path_length.backward` behind and `path_length.forward` in front.
Their center lines are concatenated to generate a path.

If waypoints exist in the route, it replaces the overlapped segment of the center line with them.
The overlap interval is determined as shown in the following figure.

![waypoint_group_overlap_interval_determination](./media/waypoint_group_overlap_interval_determination.drawio.svg)

## Path cut

If there is a self-intersection on either of the path bounds, the path is cut off a specified distance before the first intersection, as shown in the following figure (path: green, bound: blue).

![path_cut_self_intersection](./media/path_cut_self_intersection.drawio.svg)

Depending on the crossing angle, the return path's bound may be closer to the centerline than the outward's one, depicted in the diagram below. To deal with this, intersections of the left and right bounds (mutual intersection) are taken into account as well, and the path is cut at the nearest intersecting point.

![path_cut_mutual_intersection](./media/path_cut_mutual_intersection.drawio.svg)

Furthermore, in the case of the following figure, the return path goes inside even if mutual intersection is considered. Therefore, path cut is also made when the start edge of the path and the path bounds intersect.

![path_cut_start_edge_intersection](./media/path_cut_start_edge_intersection.drawio.svg)

## Turn signal

Turn signal is determined based on the rules defined for behavior_path_planner. (See [here](https://autowarefoundation.github.io/autoware_universe/main/planning/behavior_path_planner/autoware_behavior_path_planner_common/docs/behavior_path_planner_turn_signal_design/) for details)

As a general rule, the turn signal is turned on at a specified distance before the lanelet in which a turn is designated, and turned off when the driving direction has changed to the specified degree.

![turn_signal_sections](./media/turn_signal_sections.drawio.svg)

If consecutive turns are required, the turn signal corresponding to the required section or the last section takes precedence.

![turn_signal_conflict](./media/turn_signal_conflict.drawio.svg)

## Hazard signal

This node always publishes a hazard signal of `autoware_vehicle_msgs::msg::HazardLightsCommand::NO_COMMAND`.

## Flowchart

![Flowchart](https://www.plantuml.com/plantuml/png/dLHHajem3FttAVJ9NW4witkJZ8YLnBD6JcKPIa_VHWunsytEjEM3CRBUqzEJp5jM52sNw3He9932paBE7HNVgVEeU7B7JDhD4Pb9rb0Ou12o_UY4qDzVG2TYEDmflpYAcU30wSK1P4sp8_PXS6wKL6POcBePTN80DJ8Iaw3I8mfmBBDV0dqH8TOk8bCaFTNBaHJMI8PRq2amI2fAzNuHjp31EHNs6IQy4WHMaaozXiJvzODS06F564O60n9xu_s0WZa5NYLOGtQwJI4I73POo92Avk0917aaICEczNMhHJ-KsBScXYGt5B-H6-idVflDVrOuO_--NyByGhx33U_MOktwYj4ABbYQGkFQTU_hUlj-3Kc0i9-WfwKOnC1K3QwCLLMwaaISP_9mRlhgrkofM6SlT0sfv7Y7Hh2cRubFExfzZkZzLYM39jk6sa7hOWVVq0PzHZRS9z-FlQIv9PhsCM5S9XmOCg7qqphZnQzyIKadi9G-CTUrt9t69VamtEqzTR-XNOLs2xHhq3zdipnqU7ev9sUtrM_NJkxnRBNJ8jpbC396ztcunjMcQXqk2F11geIbrqpcM9z4AC-9VU3niKWxtHaBZMcrQP34-lG6ljuB7x8YzmIQ7idm3Puww1uP-nm0Ho6JRryLSuA_Me-cUiQSoOFGznA4FEHgnB-KtAzevtPgo_OykUDSpZ7awIDV2q_u4yNlNDGrTVPt_aRiooN-0W00)

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
