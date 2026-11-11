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

#include "rwh_beyond_bt/bt_gripper_command.hpp"

#include <utility>


namespace rwh_beyond_bt
{

GripperCommand::GripperCommand(
  const std::string & name, const std::string & action_name,
  const BT::NodeConfiguration & config, double aperture, std::string stage)
: DiagnosticActionNode(name, action_name, config), aperture_(aperture), stage_(std::move(stage))
{}

void GripperCommand::on_tick()
{
  set_stage(stage_);
  goal_.trajectory.joint_names = {"gripper"};
  trajectory_msgs::msg::JointTrajectoryPoint point;
  point.positions = {aperture_};
  point.time_from_start.sec = 3;
  goal_.trajectory.points = {point};
}

}  // namespace rwh_beyond_bt
