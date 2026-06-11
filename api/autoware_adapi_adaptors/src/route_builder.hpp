// Copyright 2025 The Autoware Contributors
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

#ifndef ROUTE_BUILDER_HPP_
#define ROUTE_BUILDER_HPP_

namespace autoware::adapi_adaptors
{

/// Set the route request to a single goal, clearing any pending waypoints.
///
/// Pure route-building rule shared by the fixed-goal and rough-goal callbacks. The
/// @p allow_goal_modification flag is the only behavioral difference between the two.
/// Templated on the request and pose types so it can be unit-tested with lightweight
/// stubs without a ROS context.
template <typename RequestT, typename PoseStampedT>
void set_goal(RequestT & route, const PoseStampedT & pose, bool allow_goal_modification)
{
  route.header = pose.header;
  route.goal = pose.pose;
  route.waypoints.clear();
  route.option.allow_goal_modification = allow_goal_modification;
}

/// Append a waypoint to the route request, rejecting frame-id mismatches.
///
/// Returns true if the waypoint was appended, false if its frame_id does not match the
/// goal frame (in which case the route is left unchanged). This isolates the
/// frame-mismatch guard in on_waypoint so it can be unit-tested directly.
template <typename RequestT, typename PoseStampedT>
bool append_waypoint(RequestT & route, const PoseStampedT & pose)
{
  if (route.header.frame_id != pose.header.frame_id) {
    return false;
  }
  route.waypoints.push_back(pose.pose);
  return true;
}

}  // namespace autoware::adapi_adaptors

#endif  // ROUTE_BUILDER_HPP_
