// Copyright 2024 Tier IV, Inc.
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

#include "../src/simple_pure_pursuit_core_logics.hpp"

#include <autoware_test_utils/autoware_test_utils.hpp>

#include <gtest/gtest.h>

#include <cmath>
#include <memory>

namespace autoware::control::simple_pure_pursuit
{
using autoware_planning_msgs::msg::Trajectory;
using nav_msgs::msg::Odometry;

// Floating point tolerance at EXCEPT_NEAR checks
constexpr float near_tol = 1e-4F;

Odometry makeOdometry(const double x, const double y, const double yaw)
{
  Odometry odom;
  odom.pose.pose.position.x = x;
  odom.pose.pose.position.y = y;
  odom.pose.pose.orientation.z = std::sin(yaw / 2.0);
  odom.pose.pose.orientation.w = std::cos(yaw / 2.0);
  return odom;
}

class SimplePurePursuitCoreLogicsTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    params_.lookahead_gain = 1.0;
    params_.lookahead_min_distance = 1.0;
    params_.speed_proportional_gain = 1.0;
    params_.use_external_target_vel = false;
    params_.external_target_vel = 1.0;
    params_.wheel_base_m = 2.79;
    core_logics_ = std::make_unique<SimplePurePursuitCoreLogics>(params_);
  }

  [[nodiscard]] autoware_control_msgs::msg::Control create_control_command(
    const Odometry & odom, const Trajectory & traj) const
  {
    return core_logics_->create_control_command(odom, traj);
  }

  void set_external_target_velocity(const double external_target_vel)
  {
    params_.use_external_target_vel = true;
    params_.external_target_vel = external_target_vel;
    core_logics_->set_params(params_);
  }

  [[nodiscard]] double speed_proportional_gain() const { return params_.speed_proportional_gain; }

  SimplePurePursuitParameters params_;
  std::unique_ptr<SimplePurePursuitCoreLogics> core_logics_;
};

// ================== TESTING AREA HERE ==================



}  // namespace autoware::control::simple_pure_pursuit
