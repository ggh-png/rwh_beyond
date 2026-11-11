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

#include "rwh_beyond_bt/tcp_pose.hpp"

#include <cmath>
#include <stdexcept>


namespace rwh_beyond_bt
{

TcpPose validate_tcp_pose(TcpPose target)
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
