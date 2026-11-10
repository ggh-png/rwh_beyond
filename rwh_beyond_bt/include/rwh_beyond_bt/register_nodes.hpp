#pragma once

#include "behaviortree_cpp/bt_factory.h"

namespace rwh_beyond_bt
{
void register_clear_octomap(BT::BehaviorTreeFactory & factory);
void register_move_home(BT::BehaviorTreeFactory & factory);
void register_move_tcp_up(BT::BehaviorTreeFactory & factory);
void register_close_gripper(BT::BehaviorTreeFactory & factory);
void register_open_gripper(BT::BehaviorTreeFactory & factory);
void register_set_gripper(BT::BehaviorTreeFactory & factory);
void register_move_tcp(BT::BehaviorTreeFactory & factory);
void register_make_tcp_pose(BT::BehaviorTreeFactory & factory);
}  // namespace rwh_beyond_bt
