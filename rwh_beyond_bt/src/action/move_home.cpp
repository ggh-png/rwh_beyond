#include "rwh_beyond_bt/action/move_group_node.hpp"
#include "rwh_beyond_bt/register_nodes.hpp"

namespace rwh_beyond_bt
{

class MoveHome : public MoveGroupNode
{
public:
  MoveHome(const std::string & name, const BT::NodeConfiguration & config)
  : MoveGroupNode(name, "move_action", config) {}

  void on_tick() override
  {
    setStage("move_home");
    setCommonRequest();
    moveit_msgs::msg::Constraints constraints;
    for (const auto * joint : {"joint1", "joint2", "joint3", "joint4", "joint5", "joint6"}) {
      moveit_msgs::msg::JointConstraint constraint;
      constraint.joint_name = joint;
      constraint.position = 0.0;
      constraint.tolerance_above = 0.001;
      constraint.tolerance_below = 0.001;
      constraint.weight = 1.0;
      constraints.joint_constraints.push_back(constraint);
    }
    goal_.request.goal_constraints = {constraints};
  }
};


void register_move_home(BT::BehaviorTreeFactory & factory)
{
  factory.registerNodeType<MoveHome>("MoveHome");
}

}  // namespace rwh_beyond_bt
