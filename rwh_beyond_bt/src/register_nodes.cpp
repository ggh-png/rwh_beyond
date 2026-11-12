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

#include "behaviortree_cpp/bt_factory.h"
#include "behaviortree_cpp/utils/shared_library.h"

// Compatibility loader for configurations that use rwh_beyond_bt_nodes.
// New configurations may list the individual action plugin libraries instead.
BT_REGISTER_NODES(factory)
{
  factory.registerFromPlugin(BT::SharedLibrary::getOSName(
    "rwh_beyond_clear_octomap_action_bt_node"));
  factory.registerFromPlugin(BT::SharedLibrary::getOSName("rwh_beyond_move_home_action_bt_node"));
  factory.registerFromPlugin(BT::SharedLibrary::getOSName(
    "rwh_beyond_move_observation_action_bt_node"));
  factory.registerFromPlugin(BT::SharedLibrary::getOSName("rwh_beyond_move_tcp_up_action_bt_node"));
  factory.registerFromPlugin(BT::SharedLibrary::getOSName(
    "rwh_beyond_close_gripper_action_bt_node"));
  factory.registerFromPlugin(BT::SharedLibrary::getOSName(
    "rwh_beyond_open_gripper_action_bt_node"));
  factory.registerFromPlugin(BT::SharedLibrary::getOSName("rwh_beyond_set_gripper_action_bt_node"));
  factory.registerFromPlugin(BT::SharedLibrary::getOSName("rwh_beyond_move_tcp_action_bt_node"));
  factory.registerFromPlugin(BT::SharedLibrary::getOSName(
    "rwh_beyond_make_tcp_pose_action_bt_node"));
  factory.registerFromPlugin(BT::SharedLibrary::getOSName(
    "rwh_beyond_check_grasp_ready_action_bt_node"));
  factory.registerFromPlugin(BT::SharedLibrary::getOSName(
    "rwh_beyond_segment_object_action_bt_node"));
  factory.registerFromPlugin(BT::SharedLibrary::getOSName(
    "rwh_beyond_build_object_cloud_action_bt_node"));
  factory.registerFromPlugin(BT::SharedLibrary::getOSName(
    "rwh_beyond_infer_grasps_action_bt_node"));
  factory.registerFromPlugin(BT::SharedLibrary::getOSName(
    "rwh_beyond_select_feasible_grasp_action_bt_node"));
  factory.registerFromPlugin(BT::SharedLibrary::getOSName(
    "rwh_beyond_move_to_pregrasp_action_bt_node"));
  factory.registerFromPlugin(BT::SharedLibrary::getOSName(
    "rwh_beyond_approach_grasp_action_bt_node"));
  factory.registerFromPlugin(BT::SharedLibrary::getOSName(
    "rwh_beyond_attach_object_action_bt_node"));
  factory.registerFromPlugin(BT::SharedLibrary::getOSName("rwh_beyond_lift_object_action_bt_node"));
}
