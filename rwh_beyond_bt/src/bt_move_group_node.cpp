// Copyright 2026 rwh_beyond contributors
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

#include "rwh_beyond_bt/bt_move_group_node.hpp"

namespace rwh_beyond_bt
{

MoveGroupNode::MoveGroupNode(
  const std::string & name, const std::string & action_name,
  const BT::NodeConfiguration & config)
: DiagnosticActionNode(name, action_name, config)
{}

void MoveGroupNode::set_common_request()
{
  goal_.request.group_name = "arm";
  goal_.request.pipeline_id = "ompl";
  goal_.request.planner_id = "RRTConnectkConfigDefault";
  goal_.request.allowed_planning_time = 10.0;
  goal_.request.max_velocity_scaling_factor = 0.1;
  goal_.request.max_acceleration_scaling_factor = 0.1;
  goal_.request.start_state.is_diff = true;
  goal_.planning_options.plan_only = false;
}

}  // namespace rwh_beyond_bt
