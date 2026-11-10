#include <cmath>
#include <stdexcept>

#include "rwh_beyond_bt/action/move_group_node.hpp"
#include "rwh_beyond_bt/register_nodes.hpp"
#include "rwh_beyond_bt/tcp_pose.hpp"

namespace rwh_beyond_bt
{

class MoveTcp : public MoveGroupNode
{
public:
  MoveTcp(const std::string & name, const BT::NodeConfiguration & config)
  : MoveGroupNode(name, "move_action", config) {}

  static BT::PortsList providedPorts()
  {
    return providedBasicPorts({
      BT::InputPort<TcpPose>("pose", "Required TcpPose object from the blackboard"),
      BT::InputPort<std::string>("tcp_link", "tcp_link", "Controlled TCP link"),
    });
  }

  void on_tick() override
  {
    try {
      const auto input = getInput<TcpPose>("pose");
      const auto link = getInput<std::string>("tcp_link");
      if (!input || !link || link.value().empty()) {
        throw std::invalid_argument("a valid TcpPose input and nonempty tcp_link are required");
      }
      const auto command = validateTcpPose(input.value());
      auto target = command.pose;
      if (command.relative || command.keep_current_orientation) {
        auto tf_buffer = config().blackboard->get<std::shared_ptr<tf2_ros::Buffer>>("tf_buffer");
        if (!tf_buffer) {throw std::runtime_error("tf_buffer is unavailable");}
        const auto transform = tf_buffer->lookupTransform(
          command.frame_id, link.value(), tf2::TimePointZero);
        if (command.relative) {
          target.position.x += transform.transform.translation.x;
          target.position.y += transform.transform.translation.y;
          target.position.z += transform.transform.translation.z;
        }
        if (command.keep_current_orientation) {
          target.orientation = transform.transform.rotation;
        }
      }
      if (!std::isfinite(target.position.x) || !std::isfinite(target.position.y) ||
        !std::isfinite(target.position.z))
      {
        throw std::invalid_argument("resolved TCP position must be finite");
      }
      setCommonRequest();
      moveit_msgs::msg::Constraints constraints;
      moveit_msgs::msg::PositionConstraint position;
      position.header.frame_id = command.frame_id;
      position.link_name = link.value();
      position.weight = 1.0;
      position.constraint_region.primitives.resize(1);
      position.constraint_region.primitives[0].type = shape_msgs::msg::SolidPrimitive::SPHERE;
      position.constraint_region.primitives[0].dimensions = {0.005};
      position.constraint_region.primitive_poses = {target};
      constraints.position_constraints = {position};
      moveit_msgs::msg::OrientationConstraint orientation;
      orientation.header.frame_id = command.frame_id;
      orientation.link_name = link.value();
      orientation.orientation = target.orientation;
      orientation.absolute_x_axis_tolerance = 0.01;
      orientation.absolute_y_axis_tolerance = 0.01;
      orientation.absolute_z_axis_tolerance = 0.01;
      orientation.weight = 1.0;
      constraints.orientation_constraints = {orientation};
      goal_.request.goal_constraints = {constraints};
      setStage("move_tcp");
    } catch (const std::exception & error) {
      RCLCPP_ERROR(node_->get_logger(), "MoveTcp: %s", error.what());
      should_send_goal_ = false;
    }
  }

  BT::NodeStatus on_success() override
  {
    return result_.result &&
           result_.result->error_code.val == moveit_msgs::msg::MoveItErrorCodes::SUCCESS ?
           BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
  }

  BT::NodeStatus on_cancelled() override {return BT::NodeStatus::FAILURE;}
};

void register_move_tcp(BT::BehaviorTreeFactory & factory)
{
  factory.registerNodeType<MoveTcp>("MoveTcp");
}

}  // namespace rwh_beyond_bt
