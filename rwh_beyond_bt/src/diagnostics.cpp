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

#include "rwh_beyond_bt/diagnostics.hpp"

namespace rwh_beyond_bt
{
namespace
{
std::shared_ptr<ExecutionState> state_for(const BT::Blackboard::Ptr & board)
{
  static std::mutex creation_mutex;
  std::lock_guard<std::mutex> lock(creation_mutex);
  auto root = board->rootBlackboard();
  std::shared_ptr<ExecutionState> state;
  if (!root->get("execution_state", state)) {
    state = std::make_shared<ExecutionState>();
    root->set("execution_state", state);
  }
  return state;
}
}  // namespace
ExecutionDiagnostic execution_diagnostic(const BT::Blackboard::Ptr & board)
{
  auto state = state_for(board);
  std::lock_guard<std::mutex> lock(state->mutex);
  return state->diagnostic;
}
void reset_diagnostics(const BT::Blackboard::Ptr & board)
{
  auto state = state_for(board);
  std::lock_guard<std::mutex> lock(state->mutex);
  state->diagnostic = ExecutionDiagnostic();
}
void begin_stage(
  const BT::Blackboard::Ptr & board, const std::string & stage, const std::string & node)
{
  auto state = state_for(board);
  std::lock_guard<std::mutex> lock(state->mutex);
  state->diagnostic = ExecutionDiagnostic();
  state->diagnostic.current_stage = stage;
  state->diagnostic.current_node = node;
}
void backend_stage(
  const BT::Blackboard::Ptr & board, const std::string & stage, const std::string & moveit_state)
{
  auto state = state_for(board);
  std::lock_guard<std::mutex> lock(state->mutex);
  // Cleanup must not hide the operation that failed.
  if (state->diagnostic.error_code != 0) {return;}
  state->diagnostic.backend_stage = stage;
  if (!moveit_state.empty()) {state->diagnostic.moveit_state = moveit_state;}
}
void report_failure(
  const BT::Blackboard::Ptr & board, int32_t code, const std::string & name,
  const std::string & source, const std::string & message, const std::string & domain)
{
  auto state = state_for(board);
  std::lock_guard<std::mutex> lock(state->mutex);
  auto & diagnostic = state->diagnostic;
  if (diagnostic.error_code != 0) {return;}
  diagnostic.error_code = code;
  diagnostic.error_domain = domain;
  diagnostic.error_name = name;
  diagnostic.error_source = source;
  diagnostic.error_msg = message;
}
void report_exception(
  const BT::Blackboard::Ptr & board, const std::exception & error, const std::string & source)
{
  if (const auto * failure = dynamic_cast<const ActionFailure *>(&error)) {
    report_failure(board, failure->code, failure->name, failure->source, error.what(),
      failure->domain);
  } else {
    const bool invalid = dynamic_cast<const std::invalid_argument *>(&error) != nullptr;
    report_failure(board, invalid ? ErrorCode::INVALID_INPUT : ErrorCode::INTERNAL_ERROR,
      invalid ? "INVALID_INPUT" : "INTERNAL_ERROR", source, error.what());
  }
}
}  // namespace rwh_beyond_bt
