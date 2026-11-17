// Copyright 2026 rwh_beyond contributors
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at http://www.apache.org/licenses/LICENSE-2.0
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
#include "rwh_beyond_bt/plugins/action/check_grasp_ready_action.hpp"
#include "rwh_beyond_bt/plugins/action/select_feasible_grasp_action.hpp"
#include "rwh_beyond_bt/plugins/action/move_to_pregrasp_action.hpp"
#include "rwh_beyond_bt/plugins/action/approach_grasp_action.hpp"
#include "rwh_beyond_bt/plugins/action/attach_object_action.hpp"
#include "rwh_beyond_bt/plugins/action/lift_object_action.hpp"
#include "rwh_beyond_bt/plugins/action/move_home_action.hpp"
#include "rwh_beyond_bt/plugins/action/move_observation_action.hpp"
#include "rwh_beyond_bt/plugins/action/move_place_action.hpp"
#include "rwh_beyond_bt/plugins/action/open_gripper_action.hpp"
#include "rwh_beyond_bt/plugins/action/release_object_action.hpp"

BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<rwh_beyond_bt::CheckGraspReadyAction>("CheckGraspReady");
  factory.registerNodeType<rwh_beyond_bt::SelectFeasibleGraspAction>("SelectFeasibleGrasp");
  factory.registerNodeType<rwh_beyond_bt::MoveToPregraspAction>("MoveToPregrasp");
  factory.registerNodeType<rwh_beyond_bt::ApproachGraspAction>("ApproachGrasp");
  factory.registerNodeType<rwh_beyond_bt::AttachObjectAction>("AttachObject");
  factory.registerNodeType<rwh_beyond_bt::LiftObjectAction>("LiftObject");
  factory.registerNodeType<rwh_beyond_bt::MoveHomeAction>("MoveHome");
  factory.registerNodeType<rwh_beyond_bt::MoveObservationAction>("MoveObservation");
  factory.registerNodeType<rwh_beyond_bt::MovePlaceAction>("MovePlace");
  factory.registerNodeType<rwh_beyond_bt::OpenGripperAction>("OpenGripper");
  factory.registerNodeType<rwh_beyond_bt::ReleaseObjectAction>("ReleaseObject");
}
