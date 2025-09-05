// Copyright 2025 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "types.hpp"

#include <gtest/gtest.h>

namespace autoware::motion_velocity_planner
{

TEST(StopObstacleClassificationTest, InitializesFromPerceptionObjectClassification)
{
  ObjectClassification perception_obj_classification;

  perception_obj_classification.label = ObjectClassification::CAR;
  ASSERT_EQ(
    StopObstacleClassification(perception_obj_classification).label,
    StopObstacleClassification::Type::CAR);

  perception_obj_classification.label = ObjectClassification::UNKNOWN;
  ASSERT_EQ(
    StopObstacleClassification(perception_obj_classification).label,
    StopObstacleClassification::Type::UNKNOWN);

  perception_obj_classification.label = ObjectClassification::PEDESTRIAN;
  ASSERT_EQ(
    StopObstacleClassification(perception_obj_classification).label,
    StopObstacleClassification::Type::PEDESTRIAN);
}

TEST(StopObstacleClassificationTest, InitializesFromPredictedObjectClassification)
{
  PredictedObject predicted_obj;

  // test by multiple classification elements
  for (size_t i = 1; i <= 2; ++i) {
    predicted_obj.classification.resize(i);

    predicted_obj.classification.at(0).label = ObjectClassification::CAR;
    ASSERT_EQ(
      StopObstacleClassification(predicted_obj.classification).label,
      StopObstacleClassification::Type::CAR);

    predicted_obj.classification.at(0).label = ObjectClassification::UNKNOWN;
    ASSERT_EQ(
      StopObstacleClassification(predicted_obj.classification).label,
      StopObstacleClassification::Type::UNKNOWN);

    predicted_obj.classification.at(0).label = ObjectClassification::PEDESTRIAN;
    ASSERT_EQ(
      StopObstacleClassification(predicted_obj.classification).label,
      StopObstacleClassification::Type::PEDESTRIAN);
  }
}

TEST(StopObstacleClassificationTest, InitializesFromStopObstacleClassification)
{
  ASSERT_EQ(
    StopObstacleClassification{StopObstacleClassification::Type::UNKNOWN}.label,
    StopObstacleClassification::Type::UNKNOWN);
  ASSERT_EQ(
    StopObstacleClassification{StopObstacleClassification::Type::CAR}.label,
    StopObstacleClassification::Type::CAR);
  ASSERT_EQ(
    StopObstacleClassification{StopObstacleClassification::Type::PEDESTRIAN}.label,
    StopObstacleClassification::Type::PEDESTRIAN);
  ASSERT_EQ(
    StopObstacleClassification{StopObstacleClassification::Type::POINTCLOUD}.label,
    StopObstacleClassification::Type::POINTCLOUD);
}

}  // namespace autoware::motion_velocity_planner
