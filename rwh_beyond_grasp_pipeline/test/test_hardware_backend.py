"""Physical contracts without starting nodes, CAN, cameras, or actions."""
from types import SimpleNamespace as NS
from unittest.mock import Mock
import numpy as np
import pytest
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
from rwh_beyond_msgs.srv import SetGraspRegion
from rwh_beyond_grasp_pipeline.hardware_coordinator import HardwareGraspCoordinator, feedback_values
from rwh_beyond_grasp_pipeline.physical_trajectory import trajectory_points
from rwh_beyond_grasp_pipeline.target_cloud_filter import validate_region


def measured():
    return JointState(name=[f'joint{i}' for i in range(1,7)]+['gripper'],
                      position=[.1]*6+[.04], effort=[0.]*6+[.5])


def test_missing_stale_or_nonfinite_feedback_cannot_drive_motion():
    for joints, received in [(None, 1), (measured(), 0)]:
        with pytest.raises(RuntimeError): feedback_values(joints, received, 2)
    bad = measured(); bad.position[2] = float('nan')
    with pytest.raises(RuntimeError): feedback_values(bad, 2, 2)
    assert feedback_values(measured(),2,2)['gripper']==.04


def test_trajectory_starts_at_measured_state_and_rejects_invalid_inputs():
    current = {f'joint{i}':.1 for i in range(1,7)}
    names = list(current)
    goal = JointTrajectory(joint_names=names, points=[JointTrajectoryPoint(
        positions=[.2]*6,time_from_start=Duration(sec=3))])
    joints, times, points = trajectory_points(goal,current,names)
    np.testing.assert_allclose(points[0],[.1]*6)
    assert list(times)==[0.,3.]
    for invalid in [JointTrajectory(), JointTrajectory(joint_names=['wrong'],points=goal.points),
                    JointTrajectory(joint_names=names,points=[JointTrajectoryPoint(positions=[float('nan')]*6,time_from_start=Duration(sec=3))]),
                    JointTrajectory(joint_names=names,points=[JointTrajectoryPoint(positions=[.3]*6)])]:
        with pytest.raises(ValueError):trajectory_points(invalid,current,names)


def test_physical_readiness_requires_transport_marker_and_never_calls_physics():
    state=NS(get_parameter=lambda name:NS(value=False),feedback=Mock(),
             call=Mock(return_value=NS(success=True,message='mock_transport')),physics=Mock())
    with pytest.raises(RuntimeError):HardwareGraspCoordinator.ensure_backend(state)
    state.physics.assert_not_called()
    state.call.return_value.message='physical_feedback_transport'
    HardwareGraspCoordinator.ensure_backend(state)


def test_region_cannot_remove_arbitrary_world_geometry():
    request=SetGraspRegion.Request(enabled=True,frame_id='base_link')
    request.upper.x=request.upper.y=request.upper.z=.1
    lower,upper=validate_region(request)
    np.testing.assert_allclose(upper,[.1]*3)
    for bad_frame,bad_size in [('world',.1),('base_link',2.),('base_link',float('nan'))]:
        request.frame_id=bad_frame;request.upper.x=bad_size
        with pytest.raises(ValueError):validate_region(request)


def test_feedback_transport_completes_from_measured_positions_and_closes_gate(monkeypatch):
    import threading
    import rwh_beyond_grasp_pipeline.physical_trajectory as module
    clock=NS(now=10.)
    monkeypatch.setattr(module,'time',NS(monotonic=lambda:clock.now,
        sleep=lambda delta:setattr(clock,'now',clock.now+delta)))
    monkeypatch.setattr(module.rclpy,'ok',lambda:True)
    values={f'joint{i}':.1 for i in range(1,7)}
    gate=[];published=[]
    def publish(msg):
        published.append(list(msg.position))
        values.update(zip(msg.name,msg.position))
    state=NS(lock=threading.Lock(),values=lambda:values.copy(),
        publisher=NS(publish=publish),set_gate=gate.append,
        get_parameter=lambda name:NS(value=.05),
        get_logger=lambda:NS(error=lambda msg:None))
    state.lock.acquire()
    handle=NS(request=NS(trajectory=JointTrajectory(joint_names=list(values),points=[
        JointTrajectoryPoint(positions=[.2]*6,time_from_start=Duration(sec=1))])),
        is_cancel_requested=False,succeed=Mock(),abort=Mock(),canceled=Mock())
    result=module.PhysicalTrajectory.execute(state,handle,list(values))
    assert result.error_code==0 and gate==[True,False]
    np.testing.assert_allclose(published[0],[.1]*6)
    np.testing.assert_allclose(published[-1],[.2]*6)
    handle.succeed.assert_called_once();handle.abort.assert_not_called()
    assert not state.lock.locked()


def test_feedback_loss_aborts_and_closes_gate(monkeypatch):
    import threading
    import rwh_beyond_grasp_pipeline.physical_trajectory as module
    monkeypatch.setattr(module.rclpy,'ok',lambda:True)
    current={f'joint{i}':.1 for i in range(1,7)}
    gate=[]
    state=NS(lock=threading.Lock(),values=Mock(side_effect=[current,RuntimeError('stale'),RuntimeError('stale')]),
        publisher=NS(publish=Mock()),set_gate=gate.append,
        get_parameter=lambda name:NS(value=.05),get_logger=lambda:NS(error=lambda msg:None))
    state.lock.acquire()
    handle=NS(request=NS(trajectory=JointTrajectory(joint_names=list(current),points=[
        JointTrajectoryPoint(positions=[.2]*6,time_from_start=Duration(sec=1))])),
        is_cancel_requested=False,succeed=Mock(),abort=Mock(),canceled=Mock())
    result=module.PhysicalTrajectory.execute(state,handle,list(current))
    assert result.error_code!=0 and gate==[True,False]
    handle.abort.assert_called_once();handle.succeed.assert_not_called()
    assert not state.lock.locked()


def test_filter_preserves_fields_and_withholds_cloud_on_missing_tf():
    from geometry_msgs.msg import TransformStamped
    from sensor_msgs.msg import PointCloud2, PointField
    from std_msgs.msg import Header
    from rwh_beyond_grasp_pipeline.target_cloud_filter import TargetCloudFilter
    import threading
    records=np.array([(0.05,0.05,0.05,123),(.5,.5,.5,456)],dtype=[
        ('x','<f4'),('y','<f4'),('z','<f4'),('rgba','<u4')])
    msg=PointCloud2(header=Header(frame_id='camera'),height=1,width=2,
        fields=[PointField(name=name,offset=4*i,count=1,datatype=(PointField.UINT32 if name=='rgba' else PointField.FLOAT32))
                for i,name in enumerate(records.dtype.names)],point_step=16,row_step=32,data=records.tobytes())
    transform=TransformStamped();transform.transform.rotation.w=1.
    state=NS(lock=threading.Lock(),region=(np.zeros(3),np.ones(3)*.1),
        publisher=NS(publish=Mock()),buffer=NS(lookup_transform=Mock(return_value=transform)),
        get_logger=lambda:NS(warning=Mock()))
    TargetCloudFilter.cloud(state,msg)
    output=state.publisher.publish.call_args.args[0]
    assert output.width==1 and bytes(output.data)==records[1:].tobytes()
    assert output.fields==msg.fields and output.header==msg.header
    state.publisher.publish.reset_mock();state.buffer.lookup_transform.side_effect=RuntimeError('no TF')
    TargetCloudFilter.cloud(state,msg)
    state.publisher.publish.assert_not_called()


def test_repeated_old_feedback_does_not_refresh_watchdog():
    from rwh_beyond_grasp_pipeline.physical_trajectory import PhysicalTrajectory
    old=measured();old.header.stamp.sec=1
    state=NS(joints=old,received_at=0.)
    PhysicalTrajectory.feedback_cb(state,old)
    assert state.received_at==0.
    new=measured();new.header.stamp.sec=2
    PhysicalTrajectory.feedback_cb(state,new)
    assert state.received_at>0.


def test_depth_xyz_undistorts_realsense_plumb_bob_calibration():
    from sensor_msgs.msg import CameraInfo, Image
    from std_msgs.msg import Header
    from rwh_beyond_model_utils.depth_converter import depth_xyz
    header = Header(frame_id='d405_color_optical_frame')
    depth = Image(header=header, height=12, width=12, encoding='32FC1', step=48,
                  data=np.ones((12, 12), dtype='<f4').tobytes())
    mask = Image(header=header, height=12, width=12, encoding='mono8', step=12,
                 data=bytes([1]) * 144)
    info = CameraInfo(header=header, height=12, width=12,
                      distortion_model='plumb_bob', d=[.1, 0., 0., 0., 0.],
                      k=[100., 0., 5., 0., 100., 5., 0., 0., 1.])
    cloud = depth_xyz(depth, info, mask)
    point = cloud[np.where((cloud[:, 0] > .029) & (cloud[:, 1] > .019))[0][0]]
    assert point[0] == pytest.approx(.0299961, abs=1e-6)
    assert point[1] == pytest.approx(.0199974, abs=1e-6)
    assert point[2] == pytest.approx(1.)


def test_attach_and_lift_do_not_judge_grasp_or_require_force():
    from geometry_msgs.msg import TransformStamped
    from moveit_msgs.msg import CollisionObject
    transform=TransformStamped();transform.transform.rotation.w=1.
    state=NS(prepared=True,attached=False,tcp='tcp_link',base='base_link',
        object=CollisionObject(id='test_target'),
        get_parameter=lambda name:NS(value={'gripper_close_width':.04,'lift_distance':.08}[name]),
        command_gripper=Mock(),apply=Mock(),stage=Mock(),
        holding=Mock(side_effect=AssertionError('Grasp must not be checked')),
        wait=Mock(side_effect=AssertionError('No wait for contact/force')),
        buffer=NS(lookup_transform=Mock(return_value=transform)),execute_cartesian=Mock())
    HardwareGraspCoordinator.attach(state)
    state.command_gripper.assert_called_once_with(.04)
    assert state.attached
    assert state.apply.call_args.args[0].robot_state.attached_collision_objects[0].object.id=='test_target'
    HardwareGraspCoordinator.lift(state)
    assert state.execute_cartesian.call_args.args[0].position.z==pytest.approx(.08)
    state.holding.assert_not_called()
    state.stage.assert_called_with('lift_motion_completed_grasp_not_checked')


def test_fully_closed_gripper_completion_uses_position_without_force(monkeypatch):
    import threading
    import rwh_beyond_grasp_pipeline.physical_trajectory as module
    clock=NS(now=10.)
    monkeypatch.setattr(module,'time',NS(monotonic=lambda:clock.now,
        sleep=lambda delta:setattr(clock,'now',clock.now+delta)))
    monkeypatch.setattr(module.rclpy,'ok',lambda:True)
    current={'gripper':.04};gate=[]
    def publish(msg):current.update(zip(msg.name,msg.position))
    state=NS(lock=threading.Lock(),values=lambda:current.copy(),
        publisher=NS(publish=publish),set_gate=gate.append,
        get_parameter=lambda name:NS(value=.05),get_logger=lambda:NS(error=Mock()))
    state.lock.acquire()
    handle=NS(request=NS(trajectory=JointTrajectory(joint_names=['gripper'],points=[
        JointTrajectoryPoint(positions=[0.],time_from_start=Duration(sec=1))])),
        is_cancel_requested=False,succeed=Mock(),abort=Mock(),canceled=Mock())
    result=module.PhysicalTrajectory.execute(state,handle,['gripper'])
    assert result.error_code==0
    handle.succeed.assert_called_once()
    assert gate==[True,False]


@pytest.mark.parametrize('mode', ['stalled_close', 'cancel', 'stale_feedback', 'stalled_open'])
def test_timed_close_uses_four_seconds_and_keeps_cancel_and_feedback_checks(monkeypatch, mode):
    import threading
    import rwh_beyond_grasp_pipeline.physical_trajectory as module
    clock=NS(now=0.);published=[];gate=[]
    monkeypatch.setattr(module,'time',NS(monotonic=lambda:clock.now,
        sleep=lambda delta:setattr(clock,'now',clock.now+delta)))
    monkeypatch.setattr(module.rclpy,'ok',lambda:True)
    target=.1 if mode=='stalled_open' else .04
    goal=JointTrajectory(joint_names=['gripper'],points=[
        JointTrajectoryPoint(positions=[target],time_from_start=Duration(sec=4))])
    class Handle:
        request=NS(trajectory=goal)
        succeed=Mock();abort=Mock();canceled=Mock()
        @property
        def is_cancel_requested(self):return mode=='cancel' and clock.now>=1.
    handle=Handle()
    def feedback():
        if mode=='stale_feedback' and clock.now>=1.:raise RuntimeError('stale physical feedback')
        return {'gripper':.08}  # Stays far from target, and no force reading exists.
    params={'initial_position_tolerance':.05,'gripper_close_width':.04,'gripper_close_duration':4.}
    state=NS(lock=threading.Lock(),values=feedback,
        publisher=NS(publish=lambda msg:published.append((clock.now,msg.position[0]))),
        set_gate=gate.append,get_parameter=lambda name:NS(value=params[name]),
        get_logger=lambda:NS(error=Mock()))
    state.lock.acquire()
    result=module.PhysicalTrajectory.execute(state,handle,['gripper'])
    assert gate==[True,False]
    assert not state.lock.locked()
    if mode=='stalled_close':
        assert result.error_code==0
        handle.succeed.assert_called_once()
        assert 4. <= clock.now < 4.03
        assert all(value==.04 for _,value in published[1:])
    elif mode=='cancel':
        handle.canceled.assert_called_once()
        handle.succeed.assert_not_called()
        assert clock.now<4.
    else:
        handle.abort.assert_called_once()
        handle.succeed.assert_not_called()
        assert result.error_code!=0


def test_coordinator_close_goal_carries_four_second_duration():
    result=NS(status=4,result=NS(error_code=0))
    handle=NS(accepted=True,get_result_async=lambda:NS(done=lambda:True,result=lambda:result))
    send=Mock(return_value=NS(done=lambda:True,result=lambda:handle))
    params={'gripper_close_width':.04,'gripper_close_duration':4.}
    state=NS(ensure_backend=Mock(),wait=lambda predicate,timeout:predicate(),motion_handle=None,
        get_parameter=lambda name:NS(value=params[name]),
        gripper_client=NS(server_is_ready=lambda:True,send_goal_async=send))
    HardwareGraspCoordinator.command_gripper(state,.04)
    goal=send.call_args.args[0]
    assert list(goal.trajectory.points[0].positions)==[.04]
    assert goal.trajectory.points[0].time_from_start.sec==4
