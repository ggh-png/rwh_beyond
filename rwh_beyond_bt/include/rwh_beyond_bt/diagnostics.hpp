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
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>

#include "behaviortree_cpp/blackboard.h"
#include "rwh_beyond_msgs/msg/error_code.hpp"

namespace rwh_beyond_bt
{
using ErrorCode = rwh_beyond_msgs::msg::ErrorCode;

// BT-owned state; ROS Feedback/Result objects are constructed only at the boundary.
struct ExecutionDiagnostic
{
  std::string current_stage{"starting"}, current_node, backend_stage, moveit_state;
  int32_t error_code{0};
  std::string error_domain, error_name, error_source, error_msg;
};
struct ExecutionState
{
  std::mutex mutex;
  ExecutionDiagnostic diagnostic;
};
class ActionFailure : public std::runtime_error
{
public:
  ActionFailure(
    int32_t code, const std::string & domain, const std::string & name,
    const std::string & source, const std::string & message)
  : std::runtime_error(message), code(code), domain(domain), name(name), source(source) {}
  int32_t code;
  std::string domain, name, source;
};
ExecutionDiagnostic execution_diagnostic(const BT::Blackboard::Ptr & board);
void reset_diagnostics(const BT::Blackboard::Ptr & board);
void begin_stage(
  const BT::Blackboard::Ptr & board, const std::string & stage, const std::string & node);
void backend_stage(
  const BT::Blackboard::Ptr & board, const std::string & stage,
  const std::string & moveit_state = "");
void report_failure(
  const BT::Blackboard::Ptr & board, int32_t code, const std::string & name,
  const std::string & source, const std::string & message, const std::string & domain = "bt");
void report_exception(
  const BT::Blackboard::Ptr & board, const std::exception & error,
  const std::string & source);
}  // namespace rwh_beyond_bt
#endif  // RWH_BEYOND_BT__DIAGNOSTICS_HPP_
