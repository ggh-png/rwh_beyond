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

#ifndef RWH_BEYOND_BT__TCP_POSE_HPP_
#define RWH_BEYOND_BT__TCP_POSE_HPP_


#include "behaviortree_cpp/basic_types.h"
#include "rwh_beyond_msgs/msg/tcp_pose.hpp"

namespace rwh_beyond_bt
{
using TcpPose = rwh_beyond_msgs::msg::TcpPose;

// Validate a copy so normalization never mutates the shared blackboard target.
TcpPose validate_tcp_pose(TcpPose target);
}  // namespace rwh_beyond_bt

namespace BT
{
template<>
inline rwh_beyond_msgs::msg::TcpPose convertFromString(StringView)
{
  throw RuntimeError("TcpPose requires a typed blackboard object; use MakeTcpPose");
}
}  // namespace BT

#endif  // RWH_BEYOND_BT__TCP_POSE_HPP_
