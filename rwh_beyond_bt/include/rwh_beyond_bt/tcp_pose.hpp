#pragma once

#include <cmath>
#include <stdexcept>

#include "behaviortree_cpp/basic_types.h"
#include "rwh_beyond_msgs/msg/tcp_pose.hpp"

namespace rwh_beyond_bt
{
using TcpPose = rwh_beyond_msgs::msg::TcpPose;

// Validate a copy so normalization never mutates the shared blackboard target.
inline TcpPose validateTcpPose(TcpPose target)
{
  if (target.frame_id.empty()) {
    throw std::invalid_argument("TcpPose.frame_id must not be empty");
  }
  const auto & p = target.pose.position;
  if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z)) {
    throw std::invalid_argument("TcpPose position must be finite");
  }
  if (!target.keep_current_orientation) {
    auto & q = target.pose.orientation;
    if (!std::isfinite(q.x) || !std::isfinite(q.y) ||
      !std::isfinite(q.z) || !std::isfinite(q.w))
    {
      throw std::invalid_argument("TcpPose quaternion components must be finite");
    }
    const double norm = std::hypot(std::hypot(q.x, q.y), std::hypot(q.z, q.w));
    if (!std::isfinite(norm) || norm < 1e-12) {
      throw std::invalid_argument("TcpPose quaternion norm must be finite and at least 1e-12");
    }
    q.x /= norm;
    q.y /= norm;
    q.z /= norm;
    q.w /= norm;
  }
  return target;
}
}  // namespace rwh_beyond_bt

namespace BT
{
template<>
inline rwh_beyond_msgs::msg::TcpPose convertFromString(StringView)
{
  throw RuntimeError("TcpPose requires a typed blackboard object; use MakeTcpPose");
}
}  // namespace BT
