// Copyright 2025 Autoware Foundation
// Licensed under the Apache License, Version 2.0.
// Row-major 6x6 covariance: order x, y, z, roll, pitch, yaw (index = row*6 + col).

#ifndef AUTOWARE__UNIFIED_LOCALIZATION_CORE__COVARIANCE_INDEX_HPP_
#define AUTOWARE__UNIFIED_LOCALIZATION_CORE__COVARIANCE_INDEX_HPP_

namespace autoware::unified_localization_core
{

namespace CovarianceIndex
{
enum Index : size_t {
  X_X = 0,
  X_Y = 1,
  X_Z = 2,
  X_ROLL = 3,
  X_PITCH = 4,
  X_YAW = 5,
  Y_X = 6,
  Y_Y = 7,
  Y_Z = 8,
  Y_ROLL = 9,
  Y_PITCH = 10,
  Y_YAW = 11,
  Z_X = 12,
  Z_Y = 13,
  Z_Z = 14,
  Z_ROLL = 15,
  Z_PITCH = 16,
  Z_YAW = 17,
  ROLL_X = 18,
  ROLL_Y = 19,
  ROLL_Z = 20,
  ROLL_ROLL = 21,
  ROLL_PITCH = 22,
  ROLL_YAW = 23,
  PITCH_X = 24,
  PITCH_Y = 25,
  PITCH_Z = 26,
  PITCH_ROLL = 27,
  PITCH_PITCH = 28,
  PITCH_YAW = 29,
  YAW_X = 30,
  YAW_Y = 31,
  YAW_Z = 32,
  YAW_ROLL = 33,
  YAW_PITCH = 34,
  YAW_YAW = 35,
};
}  // namespace CovarianceIndex

}  // namespace autoware::unified_localization_core

#endif  // AUTOWARE__UNIFIED_LOCALIZATION_CORE__COVARIANCE_INDEX_HPP_
