// Copyright 2026 RWH contributors. SPDX-License-Identifier: Apache-2.0
#include "rwh_beyond/beyond_server.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto server = std::make_shared<rwh_beyond::BeyondServer>();
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(server->get_node_base_interface());
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
