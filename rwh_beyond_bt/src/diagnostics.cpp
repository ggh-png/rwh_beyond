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

DemoFeedback demo_feedback(const BT::Blackboard::Ptr & board)
{
  DemoFeedback feedback;
  if (!board->rootBlackboard()->get("demo_feedback", feedback)) {return DemoFeedback{};}
  return feedback;
}

void begin_stage(
  const BT::Blackboard::Ptr & board, const std::string & stage, const std::string & node)
{
  DemoFeedback feedback;
  feedback.current_stage = stage;
  feedback.current_node = node;
  board->rootBlackboard()->set("demo_stage", stage);
  board->rootBlackboard()->set("demo_feedback", feedback);
}

void report_failure(
  const BT::Blackboard::Ptr & board, int32_t code, const std::string & name,
  const std::string & source, const std::string & message)
{
  auto feedback = demo_feedback(board);
  feedback.error_code = code;
  feedback.error_name = name;
  feedback.error_source = source;
  feedback.error_msg = message;
  board->rootBlackboard()->set("demo_feedback", feedback);
}

}  // namespace rwh_beyond_bt
