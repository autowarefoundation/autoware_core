# autoware_perception_objects_converter

## Purpose

This package provides nodes for converting between different perception object message types in Autoware.

## Nodes

### detected_to_predicted_objects_converter_node

This node converts `DetectedObjects` messages to `PredictedObjects` messages.

#### Input

| Name                     | Type                                           | Description            |
| :----------------------- | :--------------------------------------------- | :--------------------- |
| `input/detected_objects` | autoware_perception_msgs::msg::DetectedObjects | Input detected objects |

#### Output

| Name                       | Type                                            | Description              |
| :------------------------- | :---------------------------------------------- | :----------------------- |
| `output/predicted_objects` | autoware_perception_msgs::msg::PredictedObjects | Output predicted objects |

#### Parameters

None

## Usage

```bash
ros2 launch autoware_perception_objects_converter detected_to_predicted_objects.launch.xml
```
