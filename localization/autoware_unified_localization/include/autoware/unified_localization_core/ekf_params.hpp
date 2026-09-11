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

#ifndef AUTOWARE__UNIFIED_LOCALIZATION_CORE__EKF_PARAMS_HPP_
#define AUTOWARE__UNIFIED_LOCALIZATION_CORE__EKF_PARAMS_HPP_

#include <cstddef>

namespace autoware::unified_localization_core
{

struct EKFParams
{
  double predict_frequency{50.0};
  double ekf_dt{0.02};  // 1.0 / predict_frequency
  bool enable_yaw_bias_estimation{true};
  size_t extend_state_step{50};

  double pose_additional_delay{0.0};
  double pose_gate_dist{49.5};
  size_t pose_smoothing_steps{5};
  size_t max_pose_queue_size{5};

  double twist_additional_delay{0.0};
  double twist_gate_dist{46.1};
  size_t twist_smoothing_steps{2};
  size_t max_twist_queue_size{2};

  double proc_stddev_vx_c{10.0};
  double proc_stddev_wz_c{5.0};
  double proc_stddev_yaw_c{0.005};

  double z_filter_proc_dev{5.0};
  double roll_filter_proc_dev{0.1};
  double pitch_filter_proc_dev{0.1};
};

}  // namespace autoware::unified_localization_core

#endif  // AUTOWARE__UNIFIED_LOCALIZATION_CORE__EKF_PARAMS_HPP_
