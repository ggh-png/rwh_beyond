#include "rwh_beyond_bt/action/gripper_command.hpp"
#include "rwh_beyond_bt/register_nodes.hpp"

namespace rwh_beyond_bt
{

class OpenGripper : public GripperCommand
{
public:
  OpenGripper(const std::string & name, const BT::NodeConfiguration & config)
  : GripperCommand(name, "gripper_controller/follow_joint_trajectory", config, 0.1, "open_gripper") {}
};


void register_open_gripper(BT::BehaviorTreeFactory & factory)
{
  factory.registerNodeType<OpenGripper>("OpenGripper");
}

}  // namespace rwh_beyond_bt
