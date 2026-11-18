// Copyright 2026 rwh_beyond contributors
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at http://www.apache.org/licenses/LICENSE-2.0
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
#ifndef RWH_BEYOND_BT__JOINT_POSE_STORE_HPP_
#define RWH_BEYOND_BT__JOINT_POSE_STORE_HPP_

#include <map>
#include <string>
#include <vector>

namespace rwh_beyond_bt
{
using JointPoses = std::map<std::string, std::vector<double>>;
std::string default_joint_poses_file();
JointPoses read_joint_poses(const std::string & path);
std::vector<double> load_joint_pose(const std::string & path, const std::string & name);
void save_joint_pose(
  const std::string & path, const std::string & name,
  const std::vector<double> & positions, bool overwrite = false);
}  // namespace rwh_beyond_bt
#endif  // RWH_BEYOND_BT__JOINT_POSE_STORE_HPP_
