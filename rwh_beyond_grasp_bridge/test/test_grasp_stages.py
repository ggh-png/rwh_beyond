"""Stage boundaries reject stale data, wrong ordering and canceled work before motion."""
import copy
import threading
from types import SimpleNamespace, MethodType
import pytest
from sensor_msgs.msg import Image, PointCloud2
from rwh_beyond_msgs.action import RunGraspStage
from rwh_beyond_grasp_bridge.grasp_coordinator import GraspCoordinator
from rwh_beyond_grasp_bridge.stage_errors import Canceled


@pytest.fixture
def coordinator():
    context={'mask':Image(width=3), 'cloud':PointCloud2(width=120)}
    obj=SimpleNamespace(observation_id='capture-a',plan_id='plan-a',plan_phase='selected',
        prepared=True,attached=False,scene_lock=threading.RLock(),lock=threading.Lock(),
        cleanup_requested=False,active_handle=None,pre_trajectory='trajectory',grasp_pose='pose',
        cleanups=0,motions=[])
    obj.perception_context=lambda:context
    obj.require_observation=MethodType(GraspCoordinator.require_observation,obj)
    obj.stage=lambda text:None
    obj.get_parameter=lambda name:SimpleNamespace(value=True)
    obj.physics=lambda:None
    obj.ensure_backend=lambda:None
    obj.get_logger=lambda:SimpleNamespace(error=lambda message:None)
    def cleanup():
        obj.cleanups+=1
        if not obj.attached:obj.plan_id='';obj.prepared=False
    obj.cleanup=cleanup
    obj.execute_trajectory=lambda path:obj.motions.append(path)
    obj.execute_cartesian=lambda pose:obj.motions.append(pose)
    return obj


class Handle:
    def __init__(self,**kwargs):
        self.request=RunGraspStage.Goal(**kwargs)
        self.is_cancel_requested=False
        self.outcome=None
    def succeed(self):self.outcome='success'
    def abort(self):self.outcome='abort'
    def canceled(self):self.outcome='canceled'


def execute(obj,handle,stage):
    assert obj.lock.acquire(False)
    return GraspCoordinator.execute_stage(obj,handle,stage)


def test_reject_observation_from_other_capture(coordinator):
    with pytest.raises(ValueError,match='ID mismatch'):
        coordinator.require_observation(RunGraspStage.Goal(observation_id='old'))


@pytest.mark.parametrize('artifact', ['object_mask','object_cloud'])
def test_reject_modified_artifact(coordinator,artifact):
    request=RunGraspStage.Goal(observation_id='capture-a')
    with pytest.raises(ValueError,match='does not match'):
        coordinator.require_observation(request,artifact)


def test_accept_original_artifacts(coordinator):
    context=coordinator.perception_context()
    request=RunGraspStage.Goal(observation_id='capture-a',object_mask=copy.deepcopy(context['mask']),object_cloud=copy.deepcopy(context['cloud']))
    assert coordinator.require_observation(request,'object_mask') is context
    assert coordinator.require_observation(request,'object_cloud') is context


def test_wrong_plan_never_moves_and_cleans_up(coordinator):
    handle=Handle(plan_id='old-plan')
    result=execute(coordinator,handle,'move_pregrasp')
    assert not result.success and handle.outcome=='abort'
    assert coordinator.motions==[] and coordinator.cleanups==1
    assert not coordinator.lock.locked()


def test_cannot_skip_pregrasp(coordinator):
    result=execute(coordinator,Handle(plan_id='plan-a'),'approach')
    assert not result.success and 'out of order' in result.message
    assert not coordinator.motions


def test_motion_plan_cannot_be_replayed(coordinator):
    assert execute(coordinator,Handle(plan_id='plan-a'),'move_pregrasp').success
    assert coordinator.plan_phase=='pregrasp'
    result=execute(coordinator,Handle(plan_id='plan-a'),'move_pregrasp')
    assert not result.success
    assert coordinator.motions==['trajectory']


def test_expired_observation_prevents_motion(coordinator):
    def expired():raise RuntimeError('Perception snapshot expired')
    coordinator.perception_context=expired
    result=execute(coordinator,Handle(plan_id='plan-a'),'move_pregrasp')
    assert not result.success and not coordinator.motions
    assert coordinator.cleanups==1


def test_cancel_restores_scene_and_releases_lock(coordinator):
    def canceled(path):raise Canceled('stage canceled')
    coordinator.execute_trajectory=canceled
    handle=Handle(plan_id='plan-a')
    result=execute(coordinator,handle,'move_pregrasp')
    assert not result.success and handle.outcome=='canceled'
    assert coordinator.cleanups==1 and not coordinator.lock.locked()
