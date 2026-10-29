// Copyright 2026 RWH contributors. SPDX-License-Identifier: Apache-2.0
#include "rwh_beyond/beyond_server.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include "rclcpp_components/register_node_macro.hpp"

namespace rwh_beyond
{
BeyondServer::BeyondServer(const rclcpp::NodeOptions & options)
: nav2_util::LifecycleNode("beyond_server", "", options)
{
  using GetStatus = rwh_beyond_msgs::srv::GetStatus;
  // State inspection remains available while unconfigured or inactive.
  status_service_ = create_service<GetStatus>("~/get_status",
    [this](GetStatus::Request::SharedPtr, GetStatus::Response::SharedPtr response) {
      const auto & state = get_current_state();
      response->state_id = state.id();
      response->state_label = state.label();
      response->active = state.id() == lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE;
      response->node_namespace = get_namespace();
    });
}

nav2_util::CallbackReturn BeyondServer::on_configure(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_logger(), "RWH service server configured");
  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn BeyondServer::on_activate(const rclcpp_lifecycle::State &)
{
  createBond();
  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn BeyondServer::on_deactivate(const rclcpp_lifecycle::State &)
{
  destroyBond();
  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn BeyondServer::on_cleanup(const rclcpp_lifecycle::State &)
{
  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn BeyondServer::on_shutdown(const rclcpp_lifecycle::State &)
{
  destroyBond();
  return nav2_util::CallbackReturn::SUCCESS;
}
}  // namespace rwh_beyond

RCLCPP_COMPONENTS_REGISTER_NODE(rwh_beyond::BeyondServer)
