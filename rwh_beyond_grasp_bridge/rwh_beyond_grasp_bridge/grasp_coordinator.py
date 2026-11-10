"""Simulation-only RGBD/GraspGenX/MoveIt coordinator. BT owns stage sequencing."""
import copy
import json
import math
import threading
import time
from collections import Counter
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation
import rclpy
from rclpy.action import ActionServer, ActionClient, GoalResponse, CancelResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor, ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from rclpy.duration import Duration
from sensor_msgs.msg import Image, CameraInfo, JointState, PointCloud2
from sensor_msgs_py.point_cloud2 import create_cloud_xyz32
from std_msgs.msg import Header, ColorRGBA
from geometry_msgs.msg import Pose, PoseStamped
from std_srvs.srv import Trigger, SetBool, Empty
from moveit_msgs.msg import (PlanningScene, PlanningSceneWorld, PlanningSceneComponents as PC, CollisionObject,
    AttachedCollisionObject, RobotState, Constraints, JointConstraint, ObjectColor)
from moveit_msgs.srv import (GetPlanningScene, ApplyPlanningScene, GetPositionIK,
    GetCartesianPath, GetPositionFK, GetMotionPlan)
from moveit_msgs.action import ExecuteTrajectory
from shape_msgs.msg import SolidPrimitive
from tf2_ros import Buffer, TransformListener
from rwh_beyond_msgs.action import InferGrasps, RunGraspStep
from rwh_beyond_msgs.msg import TcpPose
from .depth_converter import depth_xyz
from .red_perception import red_mask, pose_matrix, matrix_pose, set_pair
from .zmq_client import request as zmq_request

TARGET='red_grasp_target'
TABLE='observed_support_table'
FINGERS=('gripper_link1','gripper_link2')

class Canceled(Exception):pass

class GraspCoordinator(Node):
    def __init__(self):
        super().__init__('grasp_coordinator')
        defaults={'endpoint':'tcp://127.0.0.1:5566','base_frame':'base_link','tcp_link':'tcp_link',
            'pregrasp_distance':0.08,'lift_distance':0.08,'object_dimensions':[0.04,0.04,0.05],
            # Model gripper +X is closing; PiPER physical closing is +/-Y.
            # Model fingertip is +Z 0.130 m (official piper_hand config).
            'model_to_tcp_xyz':[0.0,0.0,0.13],'model_to_tcp_rpy':[0.0,0.0,-math.pi/2],
            'num_grasps':100,'top_k':100,'simulation_only':True,'max_candidates':100}
        for k,v in defaults.items():self.declare_parameter(k,v)
        for name in ('object_dimensions','model_to_tcp_xyz','model_to_tcp_rpy'):
            value=np.asarray(self.get_parameter(name).value,dtype=float)
            if value.shape!=(3,) or not np.isfinite(value).all():raise ValueError(name+' must have three finite values')
            if name=='object_dimensions' and np.any(value<=0):raise ValueError('object dimensions must be positive')
        if not .02<=self.get_parameter('pregrasp_distance').value<=.2 or not .065<=self.get_parameter('lift_distance').value<=.2:
            raise ValueError('Invalid pregrasp/lift distance')
        self.base=self.get_parameter('base_frame').value
        self.tcp=self.get_parameter('tcp_link').value
        self.group=ReentrantCallbackGroup()
        self.lock=threading.Lock();self.scene_lock=threading.RLock();self.active_handle=None;self.motion_handle=None
        self.frames={};self.joints=None;self.cloud_stamp=0;self.cloud_seq=0
        self.prepared=False;self.attached=False;self.scene_active=False;self.original_pairs={}
        self.cleanup_requested=False;self.baseline_height=None
        self.buffer=Buffer(cache_time=Duration(seconds=30));self.listener=TransformListener(self.buffer,self)
        self.subs=[]
        for key,typ,topic in [('rgb',Image,'camera/color/image_raw'),('depth',Image,'camera/depth/image_rect_raw'),
                              ('info',CameraInfo,'camera/depth/camera_info')]:
            self.subs.append(self.create_subscription(typ,topic,lambda m,k=key:self.capture(k,m),qos_profile_sensor_data))
        self.subs.append(self.create_subscription(JointState,'feedback/joint_states',self.joint_callback,10))
        self.subs.append(self.create_subscription(PointCloud2,'camera/depth/filtered_points',self.cloud_callback,qos_profile_sensor_data))
        self.mask_pub=self.create_publisher(Image,'grasp/red_mask',qos_profile_sensor_data)
        self.points_pub=self.create_publisher(PointCloud2,'grasp/object_cloud',qos_profile_sensor_data)
        self.pose_pub=self.create_publisher(PoseStamped,'grasp/selected_tcp',10)
        self.stage_clients={}
        for key,typ,name in [('scene',GetPlanningScene,'get_planning_scene'),('apply',ApplyPlanningScene,'apply_planning_scene'),
            ('ik',GetPositionIK,'compute_ik'),('cartesian',GetCartesianPath,'compute_cartesian_path'),
            ('fk',GetPositionFK,'compute_fk'),('plan',GetMotionPlan,'plan_kinematic_path'),
            ('exclude',SetBool,'simulation/exclude_red_target'),('physics',Trigger,'simulation/grasp_state'),
            ('clear',Empty,'clear_octomap')]:
            self.stage_clients[key]=self.create_client(typ,name,callback_group=self.group)
        self.infer=ActionClient(self,InferGrasps,'infer_grasps',callback_group=self.group)
        self.execute_client=ActionClient(self,ExecuteTrajectory,'execute_trajectory',callback_group=self.group)
        self.server=ActionServer(self,RunGraspStep,'grasp_step',self.execute,
            callback_group=self.group,goal_callback=self.goal,cancel_callback=self.cancel)
        self.cleanup_service=self.create_service(Trigger,'grasp/cleanup',self.cleanup_service_cb,callback_group=self.group)

    def capture(self,key,msg):
        stamp=(msg.header.stamp.sec,msg.header.stamp.nanosec)
        self.frames.setdefault(stamp,{})[key]=msg
        for old in sorted(self.frames)[:-8]:self.frames.pop(old,None)

    def joint_callback(self,msg):self.joints=msg
    def cloud_callback(self,msg):
        if msg.width and msg.height:
            self.cloud_stamp=msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9;self.cloud_seq+=1

    def goal(self,goal):
        return GoalResponse.ACCEPT if self.lock.acquire(False) else GoalResponse.REJECT

    def cancel(self,handle):
        if self.motion_handle:self.motion_handle.cancel_goal_async()
        self.cleanup_requested=True
        return CancelResponse.ACCEPT

    def stage(self,text):
        self.get_logger().info(text)
        if self.active_handle:self.active_handle.publish_feedback(RunGraspStep.Feedback(stage=text))

    def wait(self,predicate,timeout=15,check_cancel=True):
        end=time.monotonic()+timeout
        while rclpy.ok() and not predicate():
            if check_cancel and self.active_handle and self.active_handle.is_cancel_requested:raise Canceled('stage canceled')
            if time.monotonic()>end:raise TimeoutError('Timed out waiting for stage dependency/result')
            time.sleep(.01)
        if not rclpy.ok():raise RuntimeError('ROS shutdown')
        if check_cancel and self.active_handle and self.active_handle.is_cancel_requested:raise Canceled('stage canceled')

    def call(self,key,value,timeout=15,check_cancel=True):
        client=self.stage_clients[key]
        self.wait(client.service_is_ready,timeout,check_cancel)
        future=client.call_async(value);self.wait(future.done,timeout,check_cancel)
        return future.result()

    def state(self):
        self.wait(lambda:self.joints is not None,5)
        return RobotState(joint_state=copy.deepcopy(self.joints),is_diff=True)

    def apply(self,scene,check_cancel=True):
        scene.is_diff=True;scene.robot_state.is_diff=True
        if not self.call('apply',ApplyPlanningScene.Request(scene=scene),check_cancel=check_cancel).success:
            raise RuntimeError('Planning Scene update rejected')

    def physics(self,check_cancel=True):
        response=self.call('physics',Trigger.Request(),check_cancel=check_cancel)
        data=json.loads(response.message)
        if data.get('simulation')!='mujoco':raise RuntimeError('Simulation-only coordinator requires MuJoCo verification service')
        return response,data

    def execute(self,handle):
        self.scene_lock.acquire()
        self.active_handle=handle
        result=RunGraspStep.Result()
        try:
            command=handle.request.command
            self.stage(command)
            if command!='cleanup':
                if not self.get_parameter('simulation_only').value:raise RuntimeError('Hardware execution is not supported')
                self.physics()
            if command=='check_ready':
                if not self.get_parameter('simulation_only').value:raise RuntimeError('Hardware execution is not supported')
                _,physics_state=self.physics()
                attached_query=GetPlanningScene.Request()
                attached_query.components.components=PC.ROBOT_STATE_ATTACHED_OBJECTS
                existing=self.call('scene',attached_query).scene.robot_state.attached_collision_objects
                if TARGET in {o.object.id for o in existing}:
                    self.attached=True;self.scene_active=True
                if physics_state.get('bilateral_contact') and not self.attached:
                    raise RuntimeError('Target already in finger contact; reset/release before a new pick')
                if self.attached:raise RuntimeError('Already holding a target; place/release before another pick')
                health=zmq_request(self.get_parameter('endpoint').value,{'action':'health'},3)
                if health.get('status')!='ok' or health.get('mock'):raise RuntimeError('A real GraspGenX server is required')
            elif command=='prepare':result.pregrasp=self.prepare()
            elif command=='move_pregrasp':
                if not self.prepared:raise RuntimeError('No prepared grasp')
                self.execute_trajectory(self.pre_trajectory)
            elif command=='approach':
                if not self.prepared:raise RuntimeError('No prepared grasp')
                self.execute_cartesian(self.grasp_pose)
            elif command=='attach':self.attach()
            elif command=='lift':self.lift()
            elif command=='cleanup':self.cleanup()
            else:raise ValueError('Unknown grasp command: '+command)
            if handle.is_cancel_requested:raise Canceled('stage canceled')
            result.success=True;result.message='Completed '+command;handle.succeed()
        except Canceled as error:
            result.message=str(error);handle.canceled()
        except Exception as error:
            result.message=f'{type(error).__name__}: {error}'
            self.get_logger().error(result.message);handle.abort()
        finally:
            if self.cleanup_requested or (not result.success and handle.request.command!='check_ready'):
                try:self.cleanup()
                except Exception as error:self.get_logger().error('Scene cleanup failed: '+str(error))
            self.cleanup_requested=False;self.active_handle=None;self.scene_lock.release();self.lock.release()
        return result

    def snapshot(self):
        after=self.get_clock().now().nanoseconds
        def ready():
            return any(set(v)=={'rgb','depth','info'} and k[0]*10**9+k[1]>after for k,v in list(self.frames.items()))
        self.wait(ready,10)
        keys=[k for k,v in list(self.frames.items()) if set(v)=={'rgb','depth','info'} and k[0]*10**9+k[1]>after]
        return copy.deepcopy(self.frames[max(keys)])

    def prepare(self):
        if self.attached:raise RuntimeError('Already holding object')
        self.cleanup()
        self.stage('acquire_red_rgbd')
        frame=self.snapshot();rgb,depth,info=frame['rgb'],frame['depth'],frame['info']
        if rgb.header.frame_id!=depth.header.frame_id:raise ValueError('RGB and depth must be registered')
        labels=red_mask(rgb)
        mask=Image(header=depth.header,height=depth.height,width=depth.width,encoding='mono8',
                   step=depth.width,data=labels.astype(np.uint8).tobytes())
        xyz=depth_xyz(depth,info,mask)
        self.mask_pub.publish(mask);self.points_pub.publish(create_cloud_xyz32(depth.header,xyz.tolist()))
        self.stage(f'infer_red_grasps points={len(xyz)}')
        self.wait(self.infer.server_is_ready,10)
        goal=InferGrasps.Goal(input_mode=InferGrasps.Goal.DEPTH_IMAGE,depth_image=depth,
            camera_info=info,object_mask=mask,gripper_name='piper_hand',
            num_grasps=self.get_parameter('num_grasps').value,top_k=self.get_parameter('top_k').value)
        sent=self.infer.send_goal_async(goal);self.wait(sent.done,10);inference=sent.result()
        if not inference.accepted:raise RuntimeError('Grasp inference goal rejected')
        response=inference.get_result_async()
        try:self.wait(response.done,90)
        except Exception:
            inference.cancel_goal_async();raise
        result=response.result().result
        if not result.success or not result.message.startswith('OK:'):raise RuntimeError('Real GraspGenX inference failed: '+result.message)
        transform=self.buffer.lookup_transform(self.base,depth.header.frame_id,Time.from_msg(depth.header.stamp))
        camera=Pose();camera.position.x=transform.transform.translation.x;camera.position.y=transform.transform.translation.y;camera.position.z=transform.transform.translation.z;camera.orientation=transform.transform.rotation
        base_T_camera=pose_matrix(camera)
        points=xyz@base_T_camera[:3,:3].T+base_T_camera[:3,3]
        lower,upper=np.quantile(points,[.02,.98],axis=0)
        dimensions=np.array(self.get_parameter('object_dimensions').value)
        center=(lower+upper)/2;center[2]=upper[2]-dimensions[2]/2
        self.stage(f'update_target_scene center={center.tolist()}')
        self.object_pose=Pose();self.object_pose.position.x,self.object_pose.position.y,self.object_pose.position.z=map(float,center);self.object_pose.orientation.w=1.
        self.object=CollisionObject(header=Header(frame_id=self.base),id=TARGET,
            primitives=[SolidPrimitive(type=SolidPrimitive.BOX,dimensions=dimensions.tolist())],primitive_poses=[self.object_pose],operation=CollisionObject.ADD)
        query=GetPlanningScene.Request();query.components.components=PC.ALLOWED_COLLISION_MATRIX
        matrix=self.call('scene',query).scene.allowed_collision_matrix
        self.original_pairs={}
        for finger in FINGERS:
            old=False
            if TARGET in matrix.entry_names and finger in matrix.entry_names:
                old=matrix.entry_values[matrix.entry_names.index(TARGET)].enabled[matrix.entry_names.index(finger)]
            self.original_pairs[finger]=old
            set_pair(matrix,finger,TARGET,True)
        # Known cube height, observed top surface -> support plane estimate, no simulator pose.
        support_z=float(upper[2]-dimensions[2])
        table_pose=Pose();table_pose.orientation.w=1.;table_pose.position.z=support_z-.012
        table=CollisionObject(header=Header(frame_id=self.base),id=TABLE,
            primitives=[SolidPrimitive(type=SolidPrimitive.BOX,dimensions=[1.5,1.5,.02])],primitive_poses=[table_pose],operation=CollisionObject.ADD)
        self.scene_active=True
        self.apply(PlanningScene(allowed_collision_matrix=matrix,world=PlanningSceneWorld(collision_objects=[self.object,table]),
            object_colors=[ObjectColor(id=TARGET,color=ColorRGBA(r=1.,g=.1,b=.05,a=1.))]))
        if not self.call('exclude',SetBool.Request(data=True)).success:raise RuntimeError('Target depth exclusion failed')
        self.call('clear',Empty.Request())
        cleared=self.get_clock().now().nanoseconds*1e-9;sequence=self.cloud_seq
        self.wait(lambda:self.cloud_seq>=sequence+2 and self.cloud_stamp>cleared,20)
        query.components.components=PC.OCTOMAP
        def map_ready():return bool(self.call('scene',query).scene.world.octomap.octomap.data)
        self.wait(map_ready,15)
        Path('/tmp/rwh-red-grasp-candidates.json').write_text(json.dumps(dict(
            observed_center=center.tolist(), base_T_camera=base_T_camera.tolist(),
            candidates=[dict(score=c.score, model_pose=(base_T_camera@pose_matrix(c.pose.pose)).tolist()) for c in result.candidates])))
        self.stage('select_collision_free_grasp')
        model_T_tcp=np.eye(4)
        model_T_tcp[:3,:3]=Rotation.from_euler('xyz',self.get_parameter('model_to_tcp_rpy').value).as_matrix()
        model_T_tcp[:3,3]=self.get_parameter('model_to_tcp_xyz').value
        reasons=Counter();self.selection_diagnostics=[]
        for index,candidate in enumerate(result.candidates[:self.get_parameter('max_candidates').value]):
            grasp=base_T_camera@pose_matrix(candidate.pose.pose)
            if grasp[2,2]>-math.cos(math.radians(35)):
                reasons['not_top_down']+=1;continue
            contact=(grasp@np.array([0,0,.105,1]))[:3]
            if np.linalg.norm(contact-center)>.055:
                reasons['off_target']+=1;continue
            for flipped in (False,True):
                g=grasp.copy()
                if flipped:g[:3,:3]=g[:3,:3]@Rotation.from_euler('z',math.pi).as_matrix()
                tcp=g@model_T_tcp
                if tcp[2,3]<support_z+.004:
                    reasons['table_clearance']+=1;continue
                pre=tcp.copy();pre[:3,3]-=g[:3,2]*self.get_parameter('pregrasp_distance').value
                pre_pose,grasp_pose=matrix_pose(pre),matrix_pose(tcp)
                state=self.ik(pre_pose,self.state())
                if state is None:reasons['pre_ik']+=1;continue
                cart=self.cartesian(grasp_pose,state)
                if cart.fraction<.999 or cart.error_code.val!=1:
                    reasons[f'approach_fraction_{cart.fraction:.2f}']+=1;continue
                grasp_state=copy.deepcopy(state)
                end=cart.solution.joint_trajectory.points[-1]
                solved=dict(zip(cart.solution.joint_trajectory.joint_names,end.positions))
                grasp_state.joint_state.position=[solved.get(n,q) for n,q in zip(grasp_state.joint_state.name,grasp_state.joint_state.position)]
                lift_pose=copy.deepcopy(grasp_pose)
                lift_pose.position.z+=self.get_parameter('lift_distance').value
                lift_path=self.cartesian(lift_pose,grasp_state)
                if lift_path.fraction<.999 or lift_path.error_code.val!=1:
                    reasons['lift_path']+=1;continue
                plan=GetMotionPlan.Request();m=plan.motion_plan_request
                m.group_name='arm';m.pipeline_id='ompl';m.planner_id='RRTConnectkConfigDefault';m.allowed_planning_time=2.
                m.max_velocity_scaling_factor=.1;m.max_acceleration_scaling_factor=.1;m.start_state=self.state()
                positions=dict(zip(state.joint_state.name,state.joint_state.position))
                m.goal_constraints=[Constraints(joint_constraints=[JointConstraint(joint_name=f'joint{i}',position=positions[f'joint{i}'],tolerance_above=.001,tolerance_below=.001,weight=1.) for i in range(1,7)])]
                planned=self.call('plan',plan,10).motion_plan_response
                if planned.error_code.val!=1:
                    reasons['pre_plan']+=1;continue
                self.grasp_pose=grasp_pose;self.pre_trajectory=planned.trajectory;self.prepared=True
                self.pose_pub.publish(PoseStamped(header=Header(frame_id=self.base),pose=grasp_pose))
                self.stage(f'selected candidate={index} score={candidate.score:.3f} tcp={tcp[:3,3].tolist()} rejected={dict(reasons)}')
                return TcpPose(frame_id=self.base,pose=pre_pose)
        raise RuntimeError('No feasible model grasp; rejection counts: '+str(dict(reasons)))

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

    def cleanup_service_cb(self,request,response):
        if not self.scene_lock.acquire(False):
            self.cleanup_requested=True
            response.success=True;response.message='Cleanup deferred to active stage'
            return response
        try:
            self.cleanup();response.success=True;response.message='Contact rules restored; verified held object retained if attached'
        except Exception as error:response.success=False;response.message=str(error)
        finally:self.scene_lock.release()
        return response

    def cleanup(self):
        if not self.scene_active:return
        query=GetPlanningScene.Request();query.components.components=PC.ALLOWED_COLLISION_MATRIX|PC.ROBOT_STATE_ATTACHED_OBJECTS|PC.WORLD_OBJECT_NAMES
        current=self.call('scene',query,check_cancel=False).scene
        matrix=current.allowed_collision_matrix
        if TARGET in {o.object.id for o in current.robot_state.attached_collision_objects}:
            self.attached=True
        # Keep a verified carried object in the collision model; never silently drop it.
        if not self.attached:
            for finger,allowed in self.original_pairs.items():set_pair(matrix,finger,TARGET,allowed)
            scene=PlanningScene(allowed_collision_matrix=matrix)
            if TARGET in {o.id for o in current.world.collision_objects}:
                scene.world.collision_objects=[CollisionObject(id=TARGET,operation=CollisionObject.REMOVE)]
            self.apply(scene,check_cancel=False)
            self.call('exclude',SetBool.Request(data=False),check_cancel=False)
            self.scene_active=False;self.prepared=False
        else:
            self.get_logger().warning('Object remains attached and gripper remains closed; no automatic release')


def main(args=None):
    rclpy.init(args=args);node=GraspCoordinator();executor=MultiThreadedExecutor(num_threads=6);executor.add_node(node)
    try:executor.spin()
    except (KeyboardInterrupt,ExternalShutdownException):pass
    finally:
        executor.shutdown();node.destroy_node();rclpy.try_shutdown()
