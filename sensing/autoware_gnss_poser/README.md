# gnss_poser

## Overview

The `gnss_poser` is a node that subscribes gnss sensing messages and calculates vehicle pose with covariance.

## Design

This node subscribes to NavSatFix to publish the pose of **base_link**. The data in NavSatFix represents the antenna's position. Therefore, it performs a coordinate transformation using the tf from `base_link` to the antenna's position. The frame_id of the antenna's position refers to NavSatFix's `header.frame_id`.
(**Note that `header.frame_id` in NavSatFix indicates the antenna's frame_id, not the Earth or reference ellipsoid.** [See also NavSatFix definition.](https://docs.ros.org/en/noetic/api/sensor_msgs/html/msg/NavSatFix.html))

The antenna is rigidly mounted on the vehicle, so this transformation is expected to be static; an antenna that moves relative to `base_link` is out of scope. Publishing it on `/tf_static` is recommended, but a setup that publishes the same fixed relation on `/tf` is supported as well: the node looks up the latest transform rather than the transform at the fix's `header.stamp`, which `/tf` cannot always provide.

If the transformation from `base_link` to the antenna cannot be obtained, the fix is skipped: `gnss_fixed` still reports the receiver's status, but no pose is published. Depending on the startup order, the transformation may arrive after the first fixes, so the fixes are skipped silently for the first 5 seconds, counted by the `header.stamp` of the fixes from the first fix (whatever its status). After that, a fix that cannot get the transformation makes the node log an error once, naming both frames, so that a `header.frame_id` that never gets a transformation is visible instead of silently yielding no pose. When TF has the transformations but the lookup still fails, for example because `header.frame_id` is not a frame name TF accepts (empty, or starting with `/`), waiting does not help, so it is logged as an error at once. Each of the two problems is logged once, and again only after the transformation has been obtained in between. A rosbag replayed in a loop gets the 5 seconds only on its first pass.

## Inputs / Outputs

### Input

| Name                           | Type                                                    | Description                                                                                                                    |
| ------------------------------ | ------------------------------------------------------- | ------------------------------------------------------------------------------------------------------------------------------ |
| `/map/map_projector_info`      | `autoware_map_msgs::msg::MapProjectorInfo`              | map projection info                                                                                                            |
| `~/input/fix`                  | `sensor_msgs::msg::NavSatFix`                           | gnss status message                                                                                                            |
| `~/input/autoware_orientation` | `autoware_sensing_msgs::msg::GnssInsOrientationStamped` | orientation [click here for more details](https://github.com/autowarefoundation/autoware_msgs/tree/main/autoware_sensing_msgs) |

### Output

| Name                     | Type                                             | Description                                                    |
| ------------------------ | ------------------------------------------------ | -------------------------------------------------------------- |
| `~/output/pose`          | `geometry_msgs::msg::PoseStamped`                | vehicle pose calculated from gnss sensing data                 |
| `~/output/gnss_pose_cov` | `geometry_msgs::msg::PoseWithCovarianceStamped`  | vehicle pose with covariance calculated from gnss sensing data |
| `~/output/gnss_fixed`    | `autoware_internal_debug_msgs::msg::BoolStamped` | gnss fix status                                                |

## Parameters

Parameters in below table

| Name                       | Type      | Default          | Description                                                                                                                          |
| -------------------------- | --------- | ---------------- | ------------------------------------------------------------------------------------------------------------------------------------ |
| `base_frame`               | `string`  | `base_link`      | frame id for base_frame                                                                                                              |
| `gnss_base_frame`          | `string`  | `gnss_base_link` | frame id for gnss_base_frame                                                                                                         |
| `map_frame`                | `string`  | `map`            | frame id for map_frame                                                                                                               |
| `use_gnss_ins_orientation` | `boolean` | `true`           | use Gnss-Ins orientation                                                                                                             |
| `gnss_pose_pub_method`     | `integer` | `0`              | 0: Instant Value 1: Average Value 2: Median Value. Any other value is rejected at startup.                                           |
| `buff_epoch`               | `integer` | `1`              | Number of positions the average / median is taken over (ignored for method 0). Range: 1~inf; smaller values are rejected at startup. |

All above parameters can be changed in config file [gnss_poser.param.yaml](./config/gnss_poser.param.yaml "Click here to open config file") .
