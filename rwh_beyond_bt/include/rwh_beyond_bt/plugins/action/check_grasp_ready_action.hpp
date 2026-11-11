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

#ifndef RWH_BEYOND_BT__PLUGINS__ACTION__CHECK_GRASP_READY_ACTION_HPP_
#define RWH_BEYOND_BT__PLUGINS__ACTION__CHECK_GRASP_READY_ACTION_HPP_

#include <string>

#include "rwh_beyond_bt/bt_grasp_action_node.hpp"

namespace rwh_beyond_bt
{

/** @brief CheckGraspReady action with typed Blackboard ports. */
class CheckGraspReadyAction : public GraspActionNode
{
public:
  CheckGraspReadyAction(
    const std::string & xml_tag_name,
    const std::string & action_name,
    const BT::NodeConfiguration & conf);

  static BT::PortsList providedPorts();
};

}  // namespace rwh_beyond_bt

#endif  // RWH_BEYOND_BT__PLUGINS__ACTION__CHECK_GRASP_READY_ACTION_HPP_
