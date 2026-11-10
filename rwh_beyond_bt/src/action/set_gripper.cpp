#include <charconv>
#include <cmath>
#include <stdexcept>

#include "rwh_beyond_bt/action/gripper_command.hpp"
#include "rwh_beyond_bt/register_nodes.hpp"

namespace rwh_beyond_bt
{

class SetGripper : public nav2_behavior_tree::BtActionNode<FollowJointTrajectory>
{
public:
  SetGripper(const std::string & name, const BT::NodeConfiguration & config)
  : BtActionNode(name, "gripper_controller/follow_joint_trajectory", config) {}

  static BT::PortsList providedPorts()
  {
    // The default BT integer parser accepts a numeric prefix (e.g. "50.5" as 50).
    // Require the entire value to be an integer, including when loading XML.
    BT::PortInfo opening_port(BT::PortDirection::INPUT, typeid(int), [](BT::StringView text) {
      int value = 0;
      const auto parsed = std::from_chars(text.data(), text.data() + text.size(), value);
      if (parsed.ec != std::errc() || parsed.ptr != text.data() + text.size()) {
        throw std::invalid_argument("opening must contain a complete integer");
      }
      return BT::Any(value);
    });
    opening_port.setDescription("Required total opening width in mm (0..100); percent optional via unit");
    return providedBasicPorts({
      {"opening", opening_port},
      BT::InputPort<std::string>("unit", "mm", "Opening unit: mm (default) or percent"),
      BT::InputPort<double>("duration", 3.0, "Trajectory duration in seconds (0, 3600]"),
    });
  }

  void on_tick() override
  {
    try {
      const auto opening = getInput<int>("opening");
      const auto unit = getInput<std::string>("unit");
      const auto duration = getInput<double>("duration");
      if (!opening || !unit || !duration) {
        throw std::invalid_argument("opening, unit and duration must have valid values");
      }
      if (opening.value() < 0 || opening.value() > 100) {
        throw std::invalid_argument("opening must be an integer in [0, 100]");
      }
      if (unit.value() != "percent" && unit.value() != "mm") {
        throw std::invalid_argument("unit must be percent or mm");
      }
      if (!std::isfinite(duration.value()) || duration.value() <= 0.0 ||
        duration.value() > 3600.0)
      {
        throw std::invalid_argument("duration must be finite and in (0, 3600] seconds");
      }
      // This model uses total aperture in metres, with a maximum width of 0.1 m.
      const double aperture = unit.value() == "percent" ?
        0.1 * opening.value() / 100.0 : opening.value() / 1000.0;
      goal_.trajectory.joint_names = {"gripper"};
      trajectory_msgs::msg::JointTrajectoryPoint point;
      point.positions = {aperture};
      point.time_from_start = rclcpp::Duration::from_seconds(duration.value());
      goal_.trajectory.points = {point};
      config().blackboard->set("demo_stage", std::string("set_gripper"));
    } catch (const std::exception & error) {
      RCLCPP_ERROR(node_->get_logger(), "SetGripper: %s", error.what());
      should_send_goal_ = false;
    }
  }

  BT::NodeStatus on_success() override
  {
    return result_.result && result_.result->error_code == FollowJointTrajectory::Result::SUCCESSFUL ?
           BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
  }

  BT::NodeStatus on_cancelled() override {return BT::NodeStatus::FAILURE;}
};

void register_set_gripper(BT::BehaviorTreeFactory & factory)
{
  factory.registerNodeType<SetGripper>("SetGripper");
}

}  // namespace rwh_beyond_bt
