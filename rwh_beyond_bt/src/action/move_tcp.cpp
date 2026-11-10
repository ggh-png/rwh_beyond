#include <cmath>
#include <stdexcept>

#include "rwh_beyond_bt/action/move_group_node.hpp"
#include "rwh_beyond_bt/register_nodes.hpp"

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
      BT::InputPort<double>("x", "Required target x in metres"),
      BT::InputPort<double>("y", "Required target y in metres"),
      BT::InputPort<double>("z", "Required target z in metres"),
      BT::InputPort<double>("qx", "Optional target quaternion x; supply all four components"),
      BT::InputPort<double>("qy", "Optional target quaternion y; supply all four components"),
      BT::InputPort<double>("qz", "Optional target quaternion z; supply all four components"),
      BT::InputPort<double>("qw", "Optional target quaternion w; supply all four components"),
      BT::InputPort<std::string>("frame_id", "base_link", "Target reference frame"),
      BT::InputPort<std::string>("tcp_link", "tcp_link", "Controlled TCP link"),
      BT::InputPort<bool>("relative", false, "Interpret xyz as offsets along reference-frame axes"),
    });
  }

  void on_tick() override
  {
    try {
      const auto x = getInput<double>("x");
      const auto y = getInput<double>("y");
      const auto z = getInput<double>("z");
      const auto frame = getInput<std::string>("frame_id");
      const auto link = getInput<std::string>("tcp_link");
      const auto relative = getInput<bool>("relative");
      if (!x || !y || !z || !frame || !link || !relative) {
        throw std::invalid_argument("x, y, z, frame_id, tcp_link and relative must have valid values");
      }
      if (!std::isfinite(x.value()) || !std::isfinite(y.value()) || !std::isfinite(z.value())) {
        throw std::invalid_argument("x, y and z must be finite");
      }
      if (frame.value().empty() || link.value().empty()) {
        throw std::invalid_argument("frame_id and tcp_link must not be empty");
      }
      geometry_msgs::msg::Pose target;
      size_t quaternion_inputs = 0;
      for (const auto * port : {"qx", "qy", "qz", "qw"}) {
        quaternion_inputs += config().input_ports.count(port);
      }
      if (quaternion_inputs != 0 && quaternion_inputs != 4) {
        throw std::invalid_argument("supply all of qx, qy, qz, qw or omit all four");
      }
      if (quaternion_inputs == 4) {
        const auto qx = getInput<double>("qx");
        const auto qy = getInput<double>("qy");
        const auto qz = getInput<double>("qz");
        const auto qw = getInput<double>("qw");
        if (!qx || !qy || !qz || !qw) {
          throw std::invalid_argument("qx, qy, qz and qw must have valid values");
        }
        if (!std::isfinite(qx.value()) || !std::isfinite(qy.value()) ||
          !std::isfinite(qz.value()) || !std::isfinite(qw.value()))
        {
          throw std::invalid_argument("quaternion components must be finite");
        }
        const double norm = std::hypot(
          std::hypot(qx.value(), qy.value()), std::hypot(qz.value(), qw.value()));
        if (!std::isfinite(norm) || norm < 1e-12) {
          throw std::invalid_argument("quaternion norm must be finite and at least 1e-12");
        }
        target.orientation.x = qx.value() / norm;
        target.orientation.y = qy.value() / norm;
        target.orientation.z = qz.value() / norm;
        target.orientation.w = qw.value() / norm;
      }

      geometry_msgs::msg::TransformStamped transform;
      // Absolute full-pose goals need no current-TCP transform. Relative xyz or
      // omitted orientation still need the measured TCP pose in frame_id.
      if (relative.value() || quaternion_inputs == 0) {
        auto tf_buffer = config().blackboard->get<std::shared_ptr<tf2_ros::Buffer>>("tf_buffer");
        if (!tf_buffer) {throw std::runtime_error("tf_buffer is unavailable");}
        transform = tf_buffer->lookupTransform(frame.value(), link.value(), tf2::TimePointZero);
        if (quaternion_inputs == 0) {
          target.orientation = transform.transform.rotation;
        }
      }
      target.position.x = x.value() + (relative.value() ? transform.transform.translation.x : 0.0);
      target.position.y = y.value() + (relative.value() ? transform.transform.translation.y : 0.0);
      target.position.z = z.value() + (relative.value() ? transform.transform.translation.z : 0.0);
      setCommonRequest();
      moveit_msgs::msg::Constraints constraints;
      moveit_msgs::msg::PositionConstraint position;
      position.header.frame_id = frame.value();
      position.link_name = link.value();
      position.weight = 1.0;
      position.constraint_region.primitives.resize(1);
      position.constraint_region.primitives[0].type = shape_msgs::msg::SolidPrimitive::SPHERE;
      position.constraint_region.primitives[0].dimensions = {0.005};
      position.constraint_region.primitive_poses = {target};
      constraints.position_constraints = {position};
      moveit_msgs::msg::OrientationConstraint orientation;
      orientation.header.frame_id = frame.value();
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
