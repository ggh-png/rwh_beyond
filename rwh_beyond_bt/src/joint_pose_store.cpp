// Copyright 2026 rwh_beyond contributors
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at http://www.apache.org/licenses/LICENSE-2.0
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
#include "rwh_beyond_bt/joint_pose_store.hpp"

#include <sys/file.h>
#include <unistd.h>
#include <fcntl.h>

#include <cmath>
#include <filesystem>
#include <fstream>
#include <regex>
#include <stdexcept>
#include <utility>

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "yaml-cpp/yaml.h"

namespace rwh_beyond_bt
{
namespace
{
void validate(const std::string & name, const std::vector<double> & positions)
{
  if (!std::regex_match(name, std::regex("[A-Za-z][A-Za-z0-9_-]{0,63}"))) {
    throw std::invalid_argument(
          "Pose name must start with a letter and contain only letters/digits/_/-");
  }
  if (positions.size() != 6) {
    throw std::invalid_argument("Pose requires joint1..joint6 in radians");
  }
  for (double q : positions) {
    if (!std::isfinite(q)) {
      throw std::invalid_argument("Joint pose must contain six finite radians");
    }
  }
}
struct Lock
{
  explicit Lock(const std::string & path)
  {
    descriptor = open(path.c_str(), O_CREAT | O_RDWR | O_CLOEXEC, 0600);
    if (descriptor < 0) {throw std::runtime_error("Cannot open joint pose lock: " + path);}
    if (flock(descriptor, LOCK_EX | LOCK_NB) != 0) {
      close(descriptor); descriptor = -1;
      throw std::runtime_error("Joint pose file is being saved by another process; retry");
    }
  }
  ~Lock() {if (descriptor >= 0) {close(descriptor);}}
  int descriptor{-1};
};
}  // namespace

std::string default_joint_poses_file()
{
  return ament_index_cpp::get_package_share_directory("rwh_beyond_bt") +
         "/config/joint_poses.yaml";
}

std::string source_joint_poses_file()
{
  return RWH_JOINT_POSES_SOURCE;
}
JointPoses read_joint_poses(const std::string & path)
{
  if (path.empty() || !std::filesystem::is_regular_file(path)) {
    throw std::runtime_error("Joint pose file missing: " + path +
      "; register front/left/right with ros2 run rwh_beyond_bt capture_joint_pose NAME");
  }
  if (std::filesystem::file_size(path) > 1024 * 1024) {
    throw std::invalid_argument("Joint pose file exceeds 1 MiB");
  }
  const auto root = YAML::LoadFile(path);
  if (!root.IsMap() || !root["version"] || root["version"].as<int>() != 1 ||
    !root["units"] || root["units"].as<std::string>() != "radians" ||
    !root["joint_names"].IsSequence() || root["joint_names"].size() != 6 ||
    !root["poses"].IsMap())
  {throw std::invalid_argument("Invalid joint pose file schema (version=1, units=radians)");}
  for (int i = 0; i < 6; ++i) {
    if (root["joint_names"][i].as<std::string>() != "joint" + std::to_string(i + 1)) {
      throw std::invalid_argument("Joint pose joint_names must be ordered joint1..joint6");
    }
  }
  JointPoses result;
  for (const auto & entry : root["poses"]) {
    const auto name = entry.first.as<std::string>();
    const auto positions = entry.second.as<std::vector<double>>();
    validate(name, positions);
    if (!result.emplace(name, positions).second) {
      throw std::invalid_argument("Duplicate joint pose: " + name);
    }
  }
  return result;
}
std::vector<double> load_joint_pose(const std::string & path, const std::string & name)
{
  const auto poses = read_joint_poses(path);
  const auto entry = poses.find(name);
  if (entry == poses.end()) {
    throw std::runtime_error("Joint pose '" + name +
        "' is not registered; capture it before running BT");
  }
  return entry->second;
}
void save_joint_pose(
  const std::string & path, const std::string & name,
  const std::vector<double> & positions, bool overwrite)
{
  validate(name, positions);
  if (path.empty()) {throw std::invalid_argument("Joint pose path is empty");}
  const std::filesystem::path target(path);
  if (!target.parent_path().empty()) {std::filesystem::create_directories(target.parent_path());}
  Lock lock(path + ".lock");
  auto poses = std::filesystem::exists(target) ? read_joint_poses(path) : JointPoses();
  if (poses.count(name) && !overwrite) {
    throw std::runtime_error("Pose already registered: " + name +
        "; pass --overwrite to replace it");
  }
  if (poses.size() >= 256 && !poses.count(name)) {
    throw std::invalid_argument("Maximum 256 joint poses");
  }
  poses[name] = positions;
  YAML::Emitter output;
  output.SetDoublePrecision(17);
  output << YAML::BeginMap << YAML::Key << "version" << YAML::Value << 1;
  output << YAML::Key << "units" << YAML::Value << "radians";
  output << YAML::Key << "joint_names" << YAML::Value << YAML::Flow << YAML::BeginSeq;
  for (int i = 1; i <= 6; ++i) {
    output << "joint" + std::to_string(i);
  }
  output << YAML::EndSeq << YAML::Key << "poses" << YAML::Value << YAML::BeginMap;
  for (const auto & pose : poses) {
    output << YAML::Key << pose.first << YAML::Value << YAML::Flow << pose.second;
  }
  output << YAML::EndMap << YAML::EndMap;
  std::string temporary = path + ".tmp.XXXXXX";
  const int fd = mkstemp(temporary.data());
  if (fd < 0) {throw std::runtime_error("Cannot create temporary pose file");}
  close(fd);
  try {
    std::ofstream file(temporary);
    file.exceptions(std::ios::badbit | std::ios::failbit);
    file << output.c_str() << '\n'; file.close();
    std::filesystem::rename(temporary, target);
  } catch (...) {
    std::filesystem::remove(temporary); throw;
  }
}
}  // namespace rwh_beyond_bt
