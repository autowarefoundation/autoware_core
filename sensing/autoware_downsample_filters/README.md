# autoware_downsample_filters

## Overview

The `autoware_downsample_filters` is a package that that reduces the number of points of the pointcloud. 

## Design

The `autoware_downsample_filters` is implemented for reducing the computational load and improving the performance of the system.

## Inner-workings / Algorithms

### Approximate Downsample Filter

`pcl::VoxelGridNearestCentroid` is used. The algorithm is described in [autoware_pcl_extensions](../../autoware_pcl_extensions/README.md)

### Random Downsample Filter

`pcl::RandomSample` is used, which points are sampled with uniform probability.

### Voxel Grid Downsample Filter

`pcl::VoxelGrid` is used, which points in each voxel are approximated with their centroid.

### Pickup Based Voxel Grid Downsample Filter

This algorithm samples a single actual point existing within the voxel, not the centroid. The computation cost is low compared to Centroid Based Voxel Grid Filter.

## Inputs / Outputs

### Input

| Name             | Type                            | Description       |
| ---------------- | ------------------------------- | ----------------  |
| `input`          | `sensor_msgs::msg::PointCloud2` | reference points  |
| `indices`        | `pcl_msgs::msg::PointIndices`   | reference indices |

### Output

| Name             | Type                                 | Description          |
| ---------------- | ------------------------------------ | -------------------- |
| `output`         | `sensor_msgs::msg::PointCloud2`      | downsampled points   |

## Parameters

### Launch file Parameters

| Name                   | Type   | Default Value | Description                                                 |
| ---------------------- | ------ | ------------- | ----------------------------------------------------------- |
| `input_frame`          | string | " "           | the frame id in which filtering is performed                |
| `output_frame`         | string | " "           | output frame id of the filtered points                      |
| `max_queue_size`       | size_t | 5             | max buffer size of input/output topics                      |
| `use_indices`          | bool   | false         | whether specifying the subset of the point cloud by indices |
| `approximate_sync`     | bool   | false         | schema used for synchronizing topic `input` and `indices`   |

### Node Parameters

#### approximate_downsample_filter_node

| Name              | Type   | Default Value | Description             |
| ----------------- | ------ | ------------- | ----------------------- |
| `voxel_size_x`    | double | 0.3           | x value of the voxel    |
| `voxel_size_y`    | double | 0.3           | y value of the voxel    |
| `voxel_size_z`    | double | 0.1           | z value of the voxel    |

#### pickup_based_voxel_grid_downsample_filter_node

| Name              | Type   | Default Value | Description             |
| ----------------- | ------ | ------------- | ----------------------- |
| `voxel_size_x`    | double | 1.0           | x value of the voxel    |
| `voxel_size_y`    | double | 1.0           | y value of the voxel    |
| `voxel_size_z`    | double | 1.0           | z value of the voxel    |

#### random_downsample_filter_node

| Name              | Type   | Default Value | Description             |
| ----------------- | ------ | ------------- | ----------------------- |
| `sample_num`      | size_t | 1500          | random sample number    |

#### voxel_grid_downsample_filter_node

| Name              | Type   | Default Value | Description             |
| ----------------- | ------ | ------------- | ----------------------- |
| `voxel_size_x`    | double | 0.3           | x value of the voxel    |
| `voxel_size_y`    | double | 0.3           | y value of the voxel    |
| `voxel_size_z`    | double | 0.1           | z value of the voxel    |

## Usage

### 1.publish static tf from input pointcloud to target frame that is used for filtering, e.g.

```cpp
ros2 run tf2_ros static_transform_publisher 2.0 3.2 1.3 0 0 0 1  velodyne_top_base_link  base_link
```

### 2.launch one of the downsample node

```cpp
ros2 launch autoware_downsample_filters random_downsample_filter_node.launch.xml
```

### 3. launch rviz2 and AWSIM to check the downsample result


## Assumptions / Known limits

<!-- cspell: ignore martinus -->

This implementation uses the `robin_hood.h` hashing library by martinus, available under the MIT License at [martinus/robin-hood-hashing](https://github.com/martinus/robin-hood-hashing) on GitHub. Special thanks to martinus for this contribution.

## (Optional) Error detection and handling

## (Optional) Performance characterization

## (Optional) References/External links

## (Optional) Future extensions / Unimplemented parts
