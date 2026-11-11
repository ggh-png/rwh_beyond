"""MoveIt execution, contact attachment and physical lift verification."""
import copy, time
from geometry_msgs.msg import Pose, PoseStamped
from std_msgs.msg import Header
from rclpy.time import Time
from rclpy.duration import Duration
from moveit_msgs.msg import RobotState, PlanningScene, AttachedCollisionObject
from moveit_msgs.srv import GetPositionIK, GetCartesianPath
from moveit_msgs.action import ExecuteTrajectory
from .stage_constants import FINGERS

class MotionStages:
    def ik(self,pose,state):
        request=GetPositionIK.Request();r=request.ik_request
        r.group_name='arm';r.ik_link_name=self.tcp;r.robot_state=state;r.avoid_collisions=True
        r.pose_stamped=PoseStamped(header=Header(frame_id=self.base),pose=pose)
        r.timeout=Duration(seconds=.1).to_msg()
        result=self.call('ik',request,5)
        return result.solution if result.error_code.val==1 else None

    def cartesian(self,pose,state):
        request=GetCartesianPath.Request(header=Header(frame_id=self.base),start_state=state,
            group_name='arm',link_name=self.tcp,waypoints=[pose],max_step=.004,
            avoid_collisions=True,revolute_jump_threshold=.35,
            max_velocity_scaling_factor=.1,max_acceleration_scaling_factor=.1)
        return self.call('cartesian',request,15)

    def execute_cartesian(self,pose):
        path=self.cartesian(pose,self.state())
        if path.error_code.val!=1 or path.fraction<.999:raise RuntimeError(f'Cartesian path rejected: fraction={path.fraction}, code={path.error_code.val}')
        self.execute_trajectory(path.solution)

    def execute_trajectory(self,trajectory):
        self.physics()  # Require the simulation verifier immediately before every motion.
        self.wait(self.execute_client.server_is_ready,10)
        sent=self.execute_client.send_goal_async(ExecuteTrajectory.Goal(trajectory=trajectory))
        self.wait(sent.done,10);self.motion_handle=sent.result()
        if not self.motion_handle.accepted:raise RuntimeError('ExecuteTrajectory rejected')
        future=self.motion_handle.get_result_async()
        try:self.wait(future.done,90)
        except Exception:
            self.motion_handle.cancel_goal_async();raise
        finally:self.motion_handle=None
        result=future.result()
        if result.status!=4 or result.result.error_code.val!=1:raise RuntimeError('MoveIt execution failed: '+str(result.result.error_code))

    def attach(self):
        if not self.prepared:raise RuntimeError('No prepared target')
        self.stage('verify_bilateral_object_contact')
        response,data=self.physics()
        if not response.success:raise RuntimeError('No bilateral object contact: '+response.message)
        self.baseline_height=data['object_position'][2]
        attached=AttachedCollisionObject(link_name=self.tcp,object=copy.deepcopy(self.object),touch_links=list(FINGERS)+[self.tcp])
        # MoveIt transfers an attached object's ID out of world automatically.
        scene=PlanningScene()
        scene.robot_state.attached_collision_objects=[attached]
        self.apply(scene);self.attached=True
        self.stage('object_attached_in_planning_scene')

    def lift(self):
        if not self.attached:raise RuntimeError('Object is not attached/verified')
        transform=self.buffer.lookup_transform(self.base,self.tcp,Time())
        pose=Pose();pose.position.x=transform.transform.translation.x;pose.position.y=transform.transform.translation.y;pose.position.z=transform.transform.translation.z+self.get_parameter('lift_distance').value;pose.orientation=transform.transform.rotation
        self.execute_cartesian(pose)
        self.stage('verify_physical_lift')
        end=time.monotonic()+5;stable_since=None
        while time.monotonic()<end:
            response,data=self.physics()
            delta=data['object_position'][2]-self.baseline_height
            if response.success and delta>.06:
                if stable_since is None:stable_since=time.monotonic()
            else:stable_since=None
            if stable_since is not None and time.monotonic()-stable_since>=2.0:
                self.stage(f'physical_lift_verified delta_z={delta:.4f} forces={data["finger_forces"]}')
                return
            time.sleep(.1)
        raise RuntimeError('Lift was not physically verified: '+str(data))
