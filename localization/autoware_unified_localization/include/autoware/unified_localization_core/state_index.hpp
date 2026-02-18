// Copyright 2025 Autoware Foundation
// Licensed under the Apache License, Version 2.0.

#ifndef AUTOWARE__UNIFIED_LOCALIZATION_CORE__STATE_INDEX_HPP_
#define AUTOWARE__UNIFIED_LOCALIZATION_CORE__STATE_INDEX_HPP_

namespace autoware::unified_localization_core
{

enum class StateIndex : int {
  X = 0,
  Y = 1,
  YAW = 2,
  YAWB = 3,
  VX = 4,
  WZ = 5,
};

constexpr int STATE_DIM = 6;

}  // namespace autoware::unified_localization_core

#endif  // AUTOWARE__UNIFIED_LOCALIZATION_CORE__STATE_INDEX_HPP_
