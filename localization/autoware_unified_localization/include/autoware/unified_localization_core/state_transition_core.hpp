// Copyright 2025 Autoware Foundation
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
