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

#ifndef RWH_BEYOND_BT__BT_MOVE_GROUP_NODE_HPP_
#define RWH_BEYOND_BT__BT_MOVE_GROUP_NODE_HPP_

#include <string>

#include "moveit_msgs/action/move_group.hpp"
#include "rwh_beyond_bt/bt_diagnostic_action_node.hpp"

namespace rwh_beyond_bt
{

using MoveGroup = moveit_msgs::action::MoveGroup;

class MoveGroupNode : public DiagnosticActionNode<MoveGroup>
{
public:
  MoveGroupNode(
    const std::string & name, const std::string & action_name,
    const BT::NodeConfiguration & config);

protected:
  void set_common_request();
};

}  // namespace rwh_beyond_bt

#endif  // RWH_BEYOND_BT__BT_MOVE_GROUP_NODE_HPP_
