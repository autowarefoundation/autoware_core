// Copyright 2025 Autoware Foundation
// Licensed under the Apache License, Version 2.0.

#ifndef AUTOWARE__UNIFIED_LOCALIZATION_CORE__STATE_TRANSITION_CORE_HPP_
#define AUTOWARE__UNIFIED_LOCALIZATION_CORE__STATE_TRANSITION_CORE_HPP_

#include "autoware/unified_localization_core/matrix_types.hpp"

namespace autoware::unified_localization_core
{

double normalize_yaw(double yaw);
Vector6d predict_next_state(const Vector6d & x_curr, double dt);
Matrix6d create_state_transition_matrix(const Vector6d & x_curr, double dt);
Matrix6d process_noise_covariance(
  double proc_cov_yaw_d, double proc_cov_vx_d, double proc_cov_wz_d);

}  // namespace autoware::unified_localization_core

#endif  // AUTOWARE__UNIFIED_LOCALIZATION_CORE__STATE_TRANSITION_CORE_HPP_
