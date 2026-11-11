"""SelectionStages operations, independent of BT command dispatch."""
import copy, json, math
from collections import Counter
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation
from geometry_msgs.msg import Pose, Point
from std_msgs.msg import Header, ColorRGBA
from std_srvs.srv import SetBool, Empty
from moveit_msgs.msg import (CollisionObject, PlanningScene, PlanningSceneWorld,
    PlanningSceneComponents as PC, ObjectColor, Constraints, JointConstraint)
from moveit_msgs.srv import GetPlanningScene, GetMotionPlan
from shape_msgs.msg import SolidPrimitive
from rwh_beyond_msgs.msg import TcpPose
from rwh_beyond_msgs.srv import SetGraspRegion
from .observed_geometry import observed_bounds
from .red_perception import pose_matrix, matrix_pose, set_pair
from .stage_constants import TARGET, TABLE, FINGERS

class SelectionStages:
    def select_feasible_grasp(self,context,candidates):
        if self.attached:raise RuntimeError('Already holding object')
        if self.scene_active:raise RuntimeError('Target scene already active; cleanup and reobserve before reselection')
        self.prepared=False;self.plan_id=""
        frame=context['frame'];depth,info=frame['depth'],frame['info']
        xyz,mask,base_T_camera,backend=context['xyz'],context['mask'],context['base_T_camera'],context['backend']
        if backend=='sam3':
            center,dimensions,support_z=observed_bounds(xyz,depth,info,mask,base_T_camera)
        else:
            points=xyz@base_T_camera[:3,:3].T+base_T_camera[:3,3]
            lower,upper=np.quantile(points,[.02,.98],axis=0)
            dimensions=np.array(self.get_parameter('object_dimensions').value)
            center=(lower+upper)/2;center[2]=upper[2]-dimensions[2]/2
            support_z=float(upper[2]-dimensions[2])
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
        # SAM mode uses an observed horizontal support plane; color_demo retains its known-box prior.
        table_pose=Pose();table_pose.orientation.w=1.;table_pose.position.z=support_z-.012
        table=CollisionObject(header=Header(frame_id=self.base),id=TABLE,
            primitives=[SolidPrimitive(type=SolidPrimitive.BOX,dimensions=[1.5,1.5,.02])],primitive_poses=[table_pose],operation=CollisionObject.ADD)
        self.scene_active=True
        self.apply(PlanningScene(allowed_collision_matrix=matrix,world=PlanningSceneWorld(collision_objects=[self.object,table]),
            object_colors=[ObjectColor(id=TARGET,color=ColorRGBA(r=1.,g=.1,b=.05,a=1.))]))
        if backend=='sam3':
            lower=center-dimensions/2-.003;upper=center+dimensions/2+.003
            # Preserve the support plane in Octomap; only exclude above-table target volume.
            lower[2]=support_z+.003
            region=SetGraspRegion.Request(enabled=True,frame_id=self.base,
                lower=Point(x=float(lower[0]),y=float(lower[1]),z=float(lower[2])),
                upper=Point(x=float(upper[0]),y=float(upper[1]),z=float(upper[2])))
            if not self.call('region',region).success:raise RuntimeError('Selected target region exclusion failed')
        elif not self.call('exclude',SetBool.Request(data=True)).success:raise RuntimeError('Target depth exclusion failed')
        self.call('clear',Empty.Request())
        cleared=self.get_clock().now().nanoseconds*1e-9;sequence=self.cloud_seq
        self.wait(lambda:self.cloud_seq>=sequence+2 and self.cloud_stamp>cleared,20)
        query.components.components=PC.OCTOMAP
        def map_ready():return bool(self.call('scene',query).scene.world.octomap.octomap.data)
        self.wait(map_ready,15)
        Path('/tmp/rwh-red-grasp-candidates.json').write_text(json.dumps(dict(
            observed_center=center.tolist(), base_T_camera=base_T_camera.tolist(),
            candidates=[dict(score=c.score, model_pose=(base_T_camera@pose_matrix(c.pose.pose)).tolist()) for c in candidates])))
        self.stage('select_collision_free_grasp')
        model_T_tcp=np.eye(4)
        model_T_tcp[:3,:3]=Rotation.from_euler('xyz',self.get_parameter('model_to_tcp_rpy').value).as_matrix()
        model_T_tcp[:3,3]=self.get_parameter('model_to_tcp_xyz').value
        reasons=Counter();self.selection_diagnostics=[]
        for index,candidate in enumerate(candidates[:self.get_parameter('max_candidates').value]):
            grasp=base_T_camera@pose_matrix(candidate.pose.pose)
            if grasp[2,2]>-math.cos(math.radians(35)):
                reasons['not_top_down']+=1;continue
            contact=(grasp@np.array([0,0,.105,1]))[:3]
            if np.any(np.abs(contact-center)>dimensions/2+.03):
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
                self.visualization.publish_selection(pre_pose,grasp_pose,depth.header,candidate.score,index,flipped)
                self.stage(f'selected candidate={index} score={candidate.score:.3f} tcp={tcp[:3,3].tolist()} rejected={dict(reasons)}')
                return TcpPose(frame_id=self.base,pose=pre_pose)
        raise RuntimeError('No feasible model grasp; rejection counts: '+str(dict(reasons)))
