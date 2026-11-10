#include "rwh_beyond_bt/action/gripper_command.hpp"
#include "rwh_beyond_bt/register_nodes.hpp"

namespace rwh_beyond_bt
{

class CloseGripper : public GripperCommand
{
public:
  CloseGripper(const std::string & name, const BT::NodeConfiguration & config)
  : GripperCommand(name, "gripper_controller/follow_joint_trajectory", config, 0.0, "close_gripper") {}
};


void register_close_gripper(BT::BehaviorTreeFactory & factory)
{
  factory.registerNodeType<CloseGripper>("CloseGripper");
}

}  // namespace rwh_beyond_bt
