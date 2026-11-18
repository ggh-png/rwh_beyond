// Copyright 2026 rwh_beyond contributors
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at http://www.apache.org/licenses/LICENSE-2.0
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
#include <algorithm>
#include <chrono>
#include <cmath>
#include <iostream>
#include <map>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rwh_beyond_bt/joint_pose_store.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  int code = 0;
  try {
    const auto args = rclcpp::remove_ros_arguments(argc, argv);
    std::string name, path = rwh_beyond_bt::default_joint_poses_file();
    bool overwrite = false, list = false;
    for (size_t i = 1; i < args.size(); ++i) {
      if (args[i] == "--overwrite") {overwrite = true;} else if (args[i] == "--list") {
        list = true;
      } else if (args[i] == "--file" && i + 1 < args.size()) {
        path = args[++i];
      } else if (!args[i].empty() && args[i][0] != '-' && name.empty()) {name = args[i];} else {
        throw std::invalid_argument(
          "Usage: capture_joint_pose NAME [--file PATH] [--overwrite] | --list [--file PATH]");
      }
    }
    if (list) {
      if (!name.empty() || overwrite) {
        throw std::invalid_argument("--list cannot capture or overwrite a pose");
      }
      for (const auto & pose : rwh_beyond_bt::read_joint_poses(path)) {
        std::cout << pose.first << ":";
        for (double q : pose.second) {
          std::cout << ' ' << q;
        }
        std::cout << '\n';
      }
    } else {
      if (name.empty()) {throw std::invalid_argument("Specify pose name: front, left or right");}
      auto node = std::make_shared<rclcpp::Node>("capture_joint_pose");
      sensor_msgs::msg::JointState::ConstSharedPtr sample;
      auto subscription = node->create_subscription<sensor_msgs::msg::JointState>(
        "feedback/joint_states", rclcpp::SensorDataQoS(),
        [&sample](sensor_msgs::msg::JointState::ConstSharedPtr message) {sample = message;});
      const auto until = std::chrono::steady_clock::now() + std::chrono::seconds(5);
      std::vector<double> positions;
      while (rclcpp::ok() && std::chrono::steady_clock::now() < until) {
        rclcpp::spin_some(node);
        if (sample && sample->name.size() == sample->position.size()) {
          std::map<std::string, double> values;
          bool valid = true;
          for (size_t i = 0; i < sample->name.size(); ++i) {
            valid = values.emplace(sample->name[i], sample->position[i]).second && valid;
          }
          positions.clear();
          for (int i = 1; i <= 6; ++i) {
            const auto joint = values.find("joint" + std::to_string(i));
            if (joint == values.end() || !std::isfinite(joint->second)) {valid = false; break;}
            positions.push_back(joint->second);
          }
          if (valid) {break;}
          positions.clear();
        }
        rclcpp::sleep_for(std::chrono::milliseconds(10));
      }
      if (positions.size() != 6) {
        throw std::runtime_error(
          "Fresh joint1..joint6 feedback unavailable; "
          "check ROS domain/RMW and feedback/joint_states");
      }
      rwh_beyond_bt::save_joint_pose(path, name, positions, overwrite);
      std::cout << "Saved '" << name << "' to " << path <<
        " (joint1..joint6, radians; no motion sent)\n";
    }
  } catch (const std::exception & error) {
    std::cerr << error.what() << '\n'; code = 1;
  }
  rclcpp::shutdown();
  return code;
}
