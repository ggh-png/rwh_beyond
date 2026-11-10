#include "behaviortree_cpp/action_node.h"
#include "rclcpp/rclcpp.hpp"
#include "rwh_beyond_bt/register_nodes.hpp"
#include "rwh_beyond_bt/tcp_pose.hpp"
#include "rwh_beyond_bt/diagnostics.hpp"

namespace rwh_beyond_bt
{

class MakeTcpPose : public BT::SyncActionNode
{
public:
  MakeTcpPose(const std::string & name, const BT::NodeConfiguration & config)
  : BT::SyncActionNode(name, config) {}

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<double>("x", "Required x in metres"),
      BT::InputPort<double>("y", "Required y in metres"),
      BT::InputPort<double>("z", "Required z in metres"),
      BT::InputPort<double>("qx", "Optional quaternion x; supply all four components"),
      BT::InputPort<double>("qy", "Optional quaternion y; supply all four components"),
      BT::InputPort<double>("qz", "Optional quaternion z; supply all four components"),
      BT::InputPort<double>("qw", "Optional quaternion w; supply all four components"),
      BT::InputPort<std::string>("frame_id", "base_link", "Reference frame for position and orientation"),
      BT::InputPort<bool>("relative", false, "Interpret only position as an offset along frame axes"),
      BT::OutputPort<TcpPose>("pose", "Constructed and validated TcpPose object"),
    };
  }

  BT::NodeStatus tick() override
  {
    beginStage(config().blackboard, "make_tcp_pose", name());
    try {
      if (config().output_ports.count("pose") == 0) {
        throw std::invalid_argument("pose output must be mapped to a blackboard key");
      }
      const auto x = getInput<double>("x");
      const auto y = getInput<double>("y");
      const auto z = getInput<double>("z");
      const auto frame = getInput<std::string>("frame_id");
      const auto relative = getInput<bool>("relative");
      if (!x || !y || !z || !frame || !relative) {
        throw std::invalid_argument("x, y, z, frame_id and relative must have valid values");
      }
      TcpPose target;
      target.frame_id = frame.value();
      target.pose.position.x = x.value();
      target.pose.position.y = y.value();
      target.pose.position.z = z.value();
      target.relative = relative.value();
      size_t quaternion_inputs = 0;
      for (const auto * port : {"qx", "qy", "qz", "qw"}) {
        quaternion_inputs += config().input_ports.count(port);
      }
      if (quaternion_inputs != 0 && quaternion_inputs != 4) {
        throw std::invalid_argument("supply all of qx, qy, qz, qw or omit all four");
      }
      target.keep_current_orientation = quaternion_inputs == 0;
      if (!target.keep_current_orientation) {
        const auto qx = getInput<double>("qx");
        const auto qy = getInput<double>("qy");
        const auto qz = getInput<double>("qz");
        const auto qw = getInput<double>("qw");
        if (!qx || !qy || !qz || !qw) {
          throw std::invalid_argument("qx, qy, qz and qw must have valid values");
        }
        target.pose.orientation.x = qx.value();
        target.pose.orientation.y = qy.value();
        target.pose.orientation.z = qz.value();
        target.pose.orientation.w = qw.value();
      }
      const auto output = setOutput("pose", validateTcpPose(target));
      if (!output) {throw std::runtime_error(output.error());}
      return BT::NodeStatus::SUCCESS;
    } catch (const std::exception & error) {
      RCLCPP_ERROR(rclcpp::get_logger("make_tcp_pose"), "%s", error.what());
      reportFailure(config().blackboard, -1000, "INVALID_INPUT", "MakeTcpPose", error.what());
      return BT::NodeStatus::FAILURE;
    }
  }
};

void register_make_tcp_pose(BT::BehaviorTreeFactory & factory)
{
  factory.registerNodeType<MakeTcpPose>("MakeTcpPose");
}
}  // namespace rwh_beyond_bt
