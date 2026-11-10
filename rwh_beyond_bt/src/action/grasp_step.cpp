#include "nav2_behavior_tree/bt_action_node.hpp"
#include "rwh_beyond_msgs/action/run_grasp_step.hpp"
#include "rwh_beyond_bt/register_nodes.hpp"
#include "rwh_beyond_bt/tcp_pose.hpp"
#include "rwh_beyond_bt/diagnostics.hpp"

namespace rwh_beyond_bt
{
class GraspStep : public nav2_behavior_tree::BtActionNode<rwh_beyond_msgs::action::RunGraspStep>
{
  using Action = rwh_beyond_msgs::action::RunGraspStep;
public:
  GraspStep(const std::string & name, const BT::NodeConfiguration & config)
  : BtActionNode<Action>(name, "grasp_step", config)
  {cancel_timeout_ = std::chrono::milliseconds(2000);}

  static BT::PortsList providedPorts()
  {
    return providedBasicPorts({BT::InputPort<std::string>("command", "Simulation grasp stage"),
      BT::OutputPort<TcpPose>("pregrasp", "Collision-validated pregrasp TCP")});
  }

  BT::NodeStatus tick() override
  {
    try {
      const auto status = BtActionNode<Action>::tick();
      if (status == BT::NodeStatus::FAILURE && demoFeedback(config().blackboard).error_code == 0) {
        reportFailure(config().blackboard, -1202, "GRASP_ACTION_REQUEST_FAILED", "grasp_step",
          "Grasp stage was rejected or its acknowledgement timed out");
      }
      return status;
    } catch (const std::exception & error) {
      reportFailure(config().blackboard, -1203, "GRASP_BT_EXCEPTION", "grasp_step", error.what());
      return BT::NodeStatus::FAILURE;
    }
  }

  void on_tick() override
  {
    auto command = getInput<std::string>("command");
    beginStage(config().blackboard, command ? command.value() : "grasp_step", name());
    if (!command || command.value().empty()) {
      reportFailure(config().blackboard, -1200, "INVALID_GRASP_STAGE", "grasp_step", "command is required");
      should_send_goal_ = false;
      return;
    }
    goal_.command = command.value();
  }

  void on_wait_for_result(std::shared_ptr<const Action::Feedback> feedback) override
  {
    if (feedback) {
      auto diagnostic = demoFeedback(config().blackboard);
      diagnostic.current_stage = feedback->stage;
      config().blackboard->rootBlackboard()->set("demo_feedback", diagnostic);
    }
  }

  BT::NodeStatus on_success() override
  {
    if (!result_.result || !result_.result->success) {return on_aborted();}
    if (goal_.command == "prepare") {setOutput("pregrasp", result_.result->pregrasp);}
    return BT::NodeStatus::SUCCESS;
  }

  BT::NodeStatus on_aborted() override
  {
    reportFailure(config().blackboard, -1201, "GRASP_STAGE_FAILED", "grasp_step/" + goal_.command,
      result_.result ? result_.result->message : "Grasp action returned no result");
    return BT::NodeStatus::FAILURE;
  }

  BT::NodeStatus on_cancelled() override {return BT::NodeStatus::FAILURE;}
};

void register_grasp_step(BT::BehaviorTreeFactory & factory)
{
  factory.registerNodeType<GraspStep>("GraspStep");
}
}  // namespace rwh_beyond_bt
