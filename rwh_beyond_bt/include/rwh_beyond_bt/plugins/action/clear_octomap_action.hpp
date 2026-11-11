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

#ifndef RWH_BEYOND_BT__PLUGINS__ACTION__CLEAR_OCTOMAP_ACTION_HPP_
#define RWH_BEYOND_BT__PLUGINS__ACTION__CLEAR_OCTOMAP_ACTION_HPP_

#include <chrono>
#include <cstdint>
#include <future>
#include <memory>
#include <set>
#include <string>

#include "behaviortree_cpp/action_node.h"
#include "moveit_msgs/srv/get_planning_scene.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "std_srvs/srv/empty.hpp"

namespace rwh_beyond_bt
{

/** @brief ClearOctomap behavior tree node. */
class ClearOctomapAction : public BT::StatefulActionNode
{
public:
  ClearOctomapAction(const std::string & xml_tag_name, const BT::NodeConfiguration & conf);

  static BT::PortsList providedPorts();

  BT::NodeStatus onStart() override;

  BT::NodeStatus onRunning() override;

  void onHalted() override;

private:
  using Clear = std_srvs::srv::Empty;
  using Scene = moveit_msgs::srv::GetPlanningScene;

  std::string phase_description() const;

  BT::NodeStatus failure(int32_t code, const std::string & error, const std::string & message);

  void cleanup();
  enum class Phase {IDLE, SERVICES, CLEAR, CLOUDS, SCENE};

  template<class Future>
  static bool ready(const Future & future)
  {
    return future.valid() && future.wait_for(std::chrono::seconds(0)) == std::future_status::ready;
  }

  rclcpp::Node::SharedPtr node_;
  rclcpp::CallbackGroup::SharedPtr group_;
  rclcpp::executors::SingleThreadedExecutor executor_;
  rclcpp::Client<Clear>::SharedPtr clear_;
  rclcpp::Client<Scene>::SharedPtr scene_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_;
  rclcpp::Client<Clear>::SharedFuture clear_future_;
  rclcpp::Client<Scene>::SharedFuture scene_future_;
  std::chrono::steady_clock::time_point deadline_;
  std::set<int64_t> stamps_;
  int64_t cleared_at_{0};
  int64_t last_query_stamp_{0};
  int required_{2};
  bool initialized_{false};
  Phase phase_{Phase::IDLE};
};

}  // namespace rwh_beyond_bt

#endif  // RWH_BEYOND_BT__PLUGINS__ACTION__CLEAR_OCTOMAP_ACTION_HPP_
