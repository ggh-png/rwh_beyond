#include "rwh_beyond_bt/action/move_group_node.hpp"
#include "rwh_beyond_bt/register_nodes.hpp"
#include <array>

namespace rwh_beyond_bt
{
class MoveObservation : public MoveGroupNode
{
public:
  MoveObservation(const std::string & name, const BT::NodeConfiguration & config)
  : MoveGroupNode(name, "move_action", config) {}
  void on_tick() override
  {
    setStage("move_observation");setCommonRequest();
    // Collision-free simulation observation pose from moma config/robots/piper.yaml.
    constexpr std::array<double, 6> positions{0.22229381216392086, 1.0279353924406198,
      -0.22003795712607355, -1.0347526435042003, 0.041778217520923366, -1.0109176820764199};
    moveit_msgs::msg::Constraints constraints;
    for (size_t i=0; i<positions.size(); ++i) {
      moveit_msgs::msg::JointConstraint joint;
      joint.joint_name="joint"+std::to_string(i+1);joint.position=positions[i];
      joint.tolerance_above=0.001;joint.tolerance_below=0.001;joint.weight=1.0;
      constraints.joint_constraints.push_back(joint);
    }
    goal_.request.goal_constraints={constraints};
  }
};
void register_move_observation(BT::BehaviorTreeFactory & factory)
{
  factory.registerNodeType<MoveObservation>("MoveObservation");
}
}  // namespace rwh_beyond_bt
