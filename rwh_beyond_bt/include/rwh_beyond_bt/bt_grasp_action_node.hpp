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

#ifndef RWH_BEYOND_BT__BT_GRASP_ACTION_NODE_HPP_
#define RWH_BEYOND_BT__BT_GRASP_ACTION_NODE_HPP_

#include <memory>
#include <string>
#include <type_traits>
#include <vector>

#include "nav2_behavior_tree/bt_action_node.hpp"
#include "rwh_beyond_bt/tcp_pose.hpp"
#include "rwh_beyond_msgs/action/run_grasp_stage.hpp"

namespace rwh_beyond_bt
{

/** @brief Shared ROS action transport and diagnostics for the grasp stage nodes. */
class GraspActionNode
  : public nav2_behavior_tree::BtActionNode<rwh_beyond_msgs::action::RunGraspStage>
{
public:
  using Action = rwh_beyond_msgs::action::RunGraspStage;
  using ObjectMask = sensor_msgs::msg::Image;
  using ObjectCloud = sensor_msgs::msg::PointCloud2;
  using GraspCandidates = std::vector<rwh_beyond_msgs::msg::GraspCandidate>;

  GraspActionNode(
    const std::string & xml_tag_name,
    const std::string & action_name,
    const BT::NodeConfiguration & conf);

  BT::NodeStatus tick() override;
  void on_tick() override;
  void on_wait_for_result(std::shared_ptr<const Action::Feedback> feedback) override;
  BT::NodeStatus on_success() override;
  BT::NodeStatus on_aborted() override;
  BT::NodeStatus on_cancelled() override;

protected:
  /// Read a required typed port; report invalid data through the common tick handler.
  template<typename T>
  T required_input(const std::string & port)
  {
    auto value = getInput<T>(port);
    if (!value) {
      throw BT::RuntimeError("Missing or invalid ", port, ": ", value.error());
    }
    if constexpr (std::is_same_v<T, std::string>) {
      if (value.value().empty()) {
        throw BT::RuntimeError("Empty ", port);
      }
    }
    return value.value();
  }
};

}  // namespace rwh_beyond_bt

#endif  // RWH_BEYOND_BT__BT_GRASP_ACTION_NODE_HPP_
