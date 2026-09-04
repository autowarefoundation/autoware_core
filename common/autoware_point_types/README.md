# Autoware Point Types

## Overview

This package provides a variety of structures to represent different types of point cloud data, mainly used for point cloud processing and analysis.

## Design

### Point cloud data type definition

`autoware_point_types` defines multiple structures (such as PointXYZI, PointXYZIRC, PointXYZIRCT, PointXYZIRADRT, PointXYZIRCAEDT, PointXYZCPE), each structure contains different attributes to adapt to different application scenarios.

- `autoware::point_types::PointXYZI`: Point type with intensity information.
- `autoware::point_types::PointXYZIRC`: Extended PointXYZI, adds return_type and channel information.
- `autoware::point_types::PointXYZIRCT`: Extended PointXYZIRC, adds a per-point time_stamp (`std::uint32_t`, nanoseconds relative to the point cloud's header stamp).
- `autoware::point_types::PointXYZIRADRT`: Extended PointXYZI, adds ring, azimuth, distance, return_type and time_stamp information.
- `autoware::point_types::PointXYZIRCAEDT`: Similar to PointXYZIRADRT, but adds elevation information and uses `std::uint32_t` as the data type for time_stamp.
- `autoware::point_types::PointXYZCPE`: Point type for segmented points, adds class_id, probability, and entropy information.
  - `PointXYZCPE::class_id` is initialized to `PointCloudClassification::INVALID=255` to represent an invalid or unset class_id value.
  - `PointXYZCPE::entropy` is initialized to `NaN` to represent an invalid or unset entropy value, then `PointXYZCPE` equality treats two `NaN` entropy values as equal, while a `NaN` entropy and a finite entropy are not equal.

### Operator overload

Each structure overloads the `==` operator, allowing users to easily compare whether two points are equal, which is very useful for deduplication and matching of point cloud data.

### Field generators

The field generator is implemented using macro definitions and std::tuple, which simplifies the serialization and deserialization process of point cloud messages and improves the reusability and readability of the code.

### Registration mechanism

Register custom point cloud structures into the PCL library through the macro `POINT_CLOUD_REGISTER_POINT_STRUCT`, so that these structures can be directly integrated with other functions of the PCL library.

## Usage

- Create a point cloud object of PointXYZIRC type

```cpp
#include "autoware/point_types/types.hpp"

int main(){
    pcl::PointCloud<autoware::point_types::PointXYZIRC>::Ptr cloud(new pcl::PointCloud<autoware::point_types::PointXYZIRC>());

    for (int i = 0; i < 5; ++i) {
        autoware::point_types::PointXYZIRC point;
        point.x = static_cast<float>(i * 0.1);
        point.y = static_cast<float>(i * 0.2);
        point.z = static_cast<float>(i * 0.3);
        point.intensity = static_cast<std::uint8_t>(i * 10);
        point.return_type = autoware::point_types::ReturnType::SINGLE_STRONGEST;
        point.channel = static_cast<std::uint16_t>(i);

        cloud->points.push_back(point);
    }
    cloud->width = cloud->points.size();
    cloud->height = 1;

    return 0;
}
```

- Convert ROS message to point cloud of PointXYZIRC type

```cpp
ExampleNode::points_callback(const PointCloud2::ConstSharedPtr & points_msg_ptr)
{
    pcl::PointCloud<autoware::point_types::PointXYZIRC>::Ptr points_ptr(
    new pcl::PointCloud<autoware::point_types::PointXYZIRC>);

    pcl::fromROSMsg(*points_msg_ptr, *points_ptr);
}
```
