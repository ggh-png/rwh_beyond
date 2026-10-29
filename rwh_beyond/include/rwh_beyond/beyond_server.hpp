// Copyright 2026 RWH contributors. SPDX-License-Identifier: Apache-2.0
#pragma once

#include <memory>
#include "nav2_util/lifecycle_node.hpp"
#include "rwh_beyond_msgs/srv/get_status.hpp"

namespace rwh_beyond
{
// Extend this lifecycle component with application services. It has no motion API.
class BeyondServer : public nav2_util::LifecycleNode
{
public:
  explicit BeyondServer(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

protected:
  nav2_util::CallbackReturn on_configure(const rclcpp_lifecycle::State &) override;
  nav2_util::CallbackReturn on_activate(const rclcpp_lifecycle::State &) override;
  nav2_util::CallbackReturn on_deactivate(const rclcpp_lifecycle::State &) override;
  nav2_util::CallbackReturn on_cleanup(const rclcpp_lifecycle::State &) override;
  nav2_util::CallbackReturn on_shutdown(const rclcpp_lifecycle::State &) override;

private:
  rclcpp::Service<rwh_beyond_msgs::srv::GetStatus>::SharedPtr status_service_;
};
}  // namespace rwh_beyond
