#include "rwh_beyond_bt/action/move_group_node.hpp"
#include "rwh_beyond_bt/register_nodes.hpp"

namespace rwh_beyond_bt
{

class MoveTcpUp : public MoveGroupNode
{
public:
  MoveTcpUp(const std::string & name, const BT::NodeConfiguration & config)
  : MoveGroupNode(name, "move_action", config) {}

  void on_tick() override
  {
    setStage("move_tcp_up");
    auto tf_buffer = config().blackboard->get<std::shared_ptr<tf2_ros::Buffer>>("tf_buffer");
    const auto transform = tf_buffer->lookupTransform(
      "base_link", "tcp_link", tf2::TimePointZero, tf2::durationFromSec(1.0));

    setCommonRequest();
    moveit_msgs::msg::Constraints constraints;
    moveit_msgs::msg::PositionConstraint position;
    position.header.frame_id = "base_link";
    position.link_name = "tcp_link";
    position.weight = 1.0;
    position.constraint_region.primitives.resize(1);
    position.constraint_region.primitives[0].type = shape_msgs::msg::SolidPrimitive::SPHERE;
    position.constraint_region.primitives[0].dimensions = {0.005};
    geometry_msgs::msg::Pose target;
    target.position.x = transform.transform.translation.x;
    target.position.y = transform.transform.translation.y;
    target.position.z = transform.transform.translation.z + 0.30;
    target.orientation = transform.transform.rotation;
    position.constraint_region.primitive_poses = {target};
    constraints.position_constraints = {position};

    moveit_msgs::msg::OrientationConstraint orientation;
    orientation.header.frame_id = "base_link";
    orientation.link_name = "tcp_link";
    orientation.orientation = target.orientation;
    orientation.absolute_x_axis_tolerance = 0.01;
    orientation.absolute_y_axis_tolerance = 0.01;
    orientation.absolute_z_axis_tolerance = 0.01;
    orientation.weight = 1.0;
    constraints.orientation_constraints = {orientation};
    goal_.request.goal_constraints = {constraints};
  }
};


void register_move_tcp_up(BT::BehaviorTreeFactory & factory)
{
  factory.registerNodeType<MoveTcpUp>("MoveTcpUp");
}

}  // namespace rwh_beyond_bt
