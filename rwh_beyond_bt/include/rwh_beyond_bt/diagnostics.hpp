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

#ifndef RWH_BEYOND_BT__DIAGNOSTICS_HPP_
#define RWH_BEYOND_BT__DIAGNOSTICS_HPP_

#include <cstdint>
#include <string>

#include "behaviortree_cpp/blackboard.h"
#include "rwh_beyond_msgs/action/run_demo.hpp"

namespace rwh_beyond_bt
{
using DemoFeedback = rwh_beyond_msgs::action::RunDemo::Feedback;

DemoFeedback demo_feedback(const BT::Blackboard::Ptr & board);

void begin_stage(
  const BT::Blackboard::Ptr & board, const std::string & stage, const std::string & node);

void report_failure(
  const BT::Blackboard::Ptr & board, int32_t code, const std::string & name,
  const std::string & source, const std::string & message);
}  // namespace rwh_beyond_bt

#endif  // RWH_BEYOND_BT__DIAGNOSTICS_HPP_
