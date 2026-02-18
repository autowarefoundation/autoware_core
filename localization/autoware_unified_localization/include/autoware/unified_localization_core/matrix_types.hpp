// Copyright 2025 Autoware Foundation
// Licensed under the Apache License, Version 2.0.

#ifndef AUTOWARE__UNIFIED_LOCALIZATION_CORE__MATRIX_TYPES_HPP_
#define AUTOWARE__UNIFIED_LOCALIZATION_CORE__MATRIX_TYPES_HPP_

#include <Eigen/Core>

namespace autoware::unified_localization_core
{

using Vector6d = Eigen::Matrix<double, 6, 1>;
using Matrix6d = Eigen::Matrix<double, 6, 6>;

}  // namespace autoware::unified_localization_core

#endif  // AUTOWARE__UNIFIED_LOCALIZATION_CORE__MATRIX_TYPES_HPP_
