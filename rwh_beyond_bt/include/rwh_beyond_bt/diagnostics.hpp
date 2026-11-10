#pragma once

#include <string>
#include "behaviortree_cpp/blackboard.h"
#include "rwh_beyond_msgs/action/run_demo.hpp"

namespace rwh_beyond_bt
{
using DemoFeedback = rwh_beyond_msgs::action::RunDemo::Feedback;

inline DemoFeedback demoFeedback(const BT::Blackboard::Ptr & board)
{
  DemoFeedback feedback;
  if (!board->rootBlackboard()->get("demo_feedback", feedback)) {return DemoFeedback{};}
  return feedback;
}

inline void beginStage(
  const BT::Blackboard::Ptr & board, const std::string & stage, const std::string & node)
{
  DemoFeedback feedback;
  feedback.current_stage = stage;
  feedback.current_node = node;
  board->rootBlackboard()->set("demo_stage", stage);
  board->rootBlackboard()->set("demo_feedback", feedback);
}

inline void reportFailure(
  const BT::Blackboard::Ptr & board, int32_t code, const std::string & name,
  const std::string & source, const std::string & message)
{
  auto feedback = demoFeedback(board);
  feedback.error_code = code;
  feedback.error_name = name;
  feedback.error_source = source;
  feedback.error_msg = message;
  board->rootBlackboard()->set("demo_feedback", feedback);
}
}  // namespace rwh_beyond_bt
