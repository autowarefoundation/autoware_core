# autoware_ground_filter

## Purpose

The `autoware_ground_filter` is a node that remove the ground points from the input pointcloud.

## Inner-workings / Algorithms

Detail description of each ground segmentation algorithm is in the following links.

| Filter Name   | Description                                                                                                | Detail                        |
| ------------- | ---------------------------------------------------------------------------------------------------------- | ----------------------------- |
| ground_filter | A method of removing the ground based on the geometrical relationship between points lined up on radiation | [link](docs/ground-filter.md) |

## Inputs / Outputs

The interfaces below are generated from the node design file
[`design/GroundFilter.node.yaml`](design/GroundFilter.node.yaml).

{{ node_interfaces_to_markdown("perception/autoware_ground_filter/design/GroundFilter.node.yaml") }}

## Parameters

### Node Parameters

The node launch parameters below are generated from the node design file
[`design/GroundFilter.node.yaml`](design/GroundFilter.node.yaml).

{{ node_param_values_to_markdown("perception/autoware_ground_filter/design/GroundFilter.node.yaml") }}

### Parameter files

{{ node_param_files_to_markdown("perception/autoware_ground_filter/design/GroundFilter.node.yaml") }}

The ground segmentation parameters are described in [ground-filter.md](docs/ground-filter.md).

## Assumptions / Known limits

Implemented based on pcl_perception [1] because of [this issue](https://github.com/ros-perception/perception_pcl/issues/9).

## References/External links

[1] <https://github.com/ros-perception/perception_pcl/blob/ros2/pcl_ros/src/pcl_ros/filters/filter.cpp>
