#include "nav2_behavior_tree/bt_action_node.hpp"
#include "rwh_beyond_msgs/action/run_grasp_stage.hpp"
#include "rwh_beyond_bt/register_nodes.hpp"
#include "rwh_beyond_bt/diagnostics.hpp"
#include "rwh_beyond_bt/tcp_pose.hpp"

namespace rwh_beyond_bt
{
using StageAction = rwh_beyond_msgs::action::RunGraspStage;
using Mask = sensor_msgs::msg::Image;
using Cloud = sensor_msgs::msg::PointCloud2;
using Candidates = std::vector<rwh_beyond_msgs::msg::GraspCandidate>;
enum class Stage {Ready, Segment, Cloud, Infer, Select, Move, Approach, Attach, Lift};

// Shared ROS transport only; each specialization has fixed semantics and typed ports.
template<Stage S>
class GraspStage : public nav2_behavior_tree::BtActionNode<StageAction>
{
  using Base = nav2_behavior_tree::BtActionNode<StageAction>;
public:
  GraspStage(const std::string & name, const BT::NodeConfiguration & config)
  : Base(name, endpoint(), config) {this->cancel_timeout_ = std::chrono::milliseconds(2000);}

  static std::string endpoint()
  {
    if constexpr (S == Stage::Ready) {return "grasp/check_ready";}
    if constexpr (S == Stage::Segment) {return "grasp/segment_object";}
    if constexpr (S == Stage::Cloud) {return "grasp/build_object_cloud";}
    if constexpr (S == Stage::Infer) {return "grasp/infer_grasps";}
    if constexpr (S == Stage::Select) {return "grasp/select_feasible_grasp";}
    if constexpr (S == Stage::Move) {return "grasp/move_pregrasp";}
    if constexpr (S == Stage::Approach) {return "grasp/approach";}
    if constexpr (S == Stage::Attach) {return "grasp/attach";}
    return "grasp/lift";
  }

  static BT::PortsList providedPorts()
  {
    BT::PortsList ports;
    if constexpr (S == Stage::Segment) {
      ports.insert(BT::OutputPort<std::string>("observation_id"));
      ports.insert(BT::OutputPort<Mask>("mask"));
    }
    if constexpr (S == Stage::Cloud || S == Stage::Infer || S == Stage::Select) {
      ports.insert(BT::InputPort<std::string>("observation_id"));
    }
    if constexpr (S == Stage::Cloud) {
      ports.insert(BT::InputPort<Mask>("mask"));
      ports.insert(BT::OutputPort<Cloud>("cloud"));
    }
    if constexpr (S == Stage::Infer || S == Stage::Select) {
      ports.insert(BT::InputPort<Cloud>("cloud"));
    }
    if constexpr (S == Stage::Infer) {ports.insert(BT::OutputPort<Candidates>("candidates"));}
    if constexpr (S == Stage::Select) {
      ports.insert(BT::InputPort<Candidates>("candidates"));
      ports.insert(BT::OutputPort<std::string>("plan_id"));
      ports.insert(BT::OutputPort<TcpPose>("pregrasp"));
    }
    if constexpr (S == Stage::Move || S == Stage::Approach || S == Stage::Attach || S == Stage::Lift) {
      ports.insert(BT::InputPort<std::string>("plan_id"));
    }
    return Base::providedBasicPorts(ports);
  }

  template<typename T> T required(const char * port)
  {
    auto value = this->template getInput<T>(port);
    if (!value) {throw BT::RuntimeError("Missing or invalid ", port, ": ", value.error());}
    return value.value();
  }

  BT::NodeStatus tick() override
  {
    try {
      auto status = Base::tick();
      if (status == BT::NodeStatus::FAILURE && demoFeedback(this->config().blackboard).error_code == 0) {
        reportFailure(this->config().blackboard, -1202, "GRASP_ACTION_REQUEST_FAILED", endpoint(),
          "Stage rejected or acknowledgement timed out");
      }
      return status;
    } catch (const std::exception & error) {
      reportFailure(this->config().blackboard, -1203, "GRASP_BT_EXCEPTION", endpoint(), error.what());
      return BT::NodeStatus::FAILURE;
    }
  }

  void on_tick() override
  {
    beginStage(this->config().blackboard, endpoint(), this->name());
    this->goal_ = StageAction::Goal();
    if constexpr (S == Stage::Cloud || S == Stage::Infer || S == Stage::Select) {
      this->goal_.observation_id = required<std::string>("observation_id");
      if (this->goal_.observation_id.empty()) {throw BT::RuntimeError("Empty observation_id");}
    }
    if constexpr (S == Stage::Cloud) {this->goal_.object_mask = required<Mask>("mask");}
    if constexpr (S == Stage::Infer || S == Stage::Select) {this->goal_.object_cloud = required<Cloud>("cloud");}
    if constexpr (S == Stage::Select) {this->goal_.candidates = required<Candidates>("candidates");}
    if constexpr (S == Stage::Move || S == Stage::Approach || S == Stage::Attach || S == Stage::Lift) {
      this->goal_.plan_id = required<std::string>("plan_id");
      if (this->goal_.plan_id.empty()) {throw BT::RuntimeError("Empty plan_id");}
    }
  }

  void on_wait_for_result(std::shared_ptr<const StageAction::Feedback> feedback) override
  {
    if (feedback) {
      auto diagnostic = demoFeedback(this->config().blackboard);
      diagnostic.current_stage = feedback->stage;
      this->config().blackboard->rootBlackboard()->set("demo_feedback", diagnostic);
    }
  }

  BT::NodeStatus on_success() override
  {
    if (!this->result_.result || !this->result_.result->success) {return on_aborted();}
    const auto & result = *this->result_.result;
    if constexpr (S == Stage::Segment) {
      this->setOutput("observation_id", result.observation_id);
      this->setOutput("mask", result.object_mask);
    }
    if constexpr (S == Stage::Cloud) {this->setOutput("cloud", result.object_cloud);}
    if constexpr (S == Stage::Infer) {this->setOutput("candidates", result.candidates);}
    if constexpr (S == Stage::Select) {
      this->setOutput("plan_id", result.plan_id);
      this->setOutput("pregrasp", result.pregrasp);
    }
    return BT::NodeStatus::SUCCESS;
  }

  BT::NodeStatus on_aborted() override
  {
    reportFailure(this->config().blackboard, -1201, "GRASP_STAGE_FAILED", endpoint(),
      this->result_.result ? this->result_.result->message : "Stage returned no result");
    return BT::NodeStatus::FAILURE;
  }
  BT::NodeStatus on_cancelled() override {return BT::NodeStatus::FAILURE;}
};

void register_grasp_stages(BT::BehaviorTreeFactory & factory)
{
  factory.registerNodeType<GraspStage<Stage::Ready>>("CheckGraspReady");
  factory.registerNodeType<GraspStage<Stage::Segment>>("SegmentObject");
  factory.registerNodeType<GraspStage<Stage::Cloud>>("BuildObjectCloud");
  factory.registerNodeType<GraspStage<Stage::Infer>>("InferGrasps");
  factory.registerNodeType<GraspStage<Stage::Select>>("SelectFeasibleGrasp");
  factory.registerNodeType<GraspStage<Stage::Move>>("MoveToPregrasp");
  factory.registerNodeType<GraspStage<Stage::Approach>>("ApproachGrasp");
  factory.registerNodeType<GraspStage<Stage::Attach>>("AttachObject");
  factory.registerNodeType<GraspStage<Stage::Lift>>("LiftObject");
}
}  // namespace rwh_beyond_bt
