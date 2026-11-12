"""Per-request target text reaches SAM without changing the configured default."""
from types import SimpleNamespace as NS
from unittest.mock import Mock
import pytest
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import Image
from rwh_beyond_grasp_pipeline.perception_stages import PerceptionStages


@pytest.mark.parametrize('requested,expected', [
    ('blue cup', 'blue cup'), (' yellow block ', 'yellow block'),
    ('', 'default object'), (None, 'default object'), ('  ', 'default object'),
])
def test_target_is_per_request_with_default_fallback(monkeypatch, requested, expected):
    import rwh_beyond_grasp_pipeline.perception_stages as module
    params = dict(target_text='default object', segmentation_backend='sam3',
                  target_box=[0.]*4, selection_point=[-1,-1], selection_policy='unique')
    image = Image()
    transform = TransformStamped(); transform.transform.rotation.w = 1.
    segment = Mock(return_value=(image, .9, 1, None))
    monkeypatch.setattr(module, 'segment_via_action', segment)
    state = NS(get_parameter=lambda name: NS(value=params[name]), attached=False,
        cleanup=Mock(), stage=Mock(), snapshot=lambda: dict(rgb=image, depth=image, info=None),
        base='base_link', buffer=NS(lookup_transform=lambda *a, **k: transform),
        joints=None, segment=object(), active_handle=None,
        visualization=NS(publish_segmentation=Mock()))
    PerceptionStages.segment_target(state, requested)
    assert segment.call_args.args[3] == expected
    assert state.perception['target_text'] == expected
    assert params['target_text'] == 'default object'


def test_explicit_text_cannot_silently_use_fixed_color_demo():
    state = NS(get_parameter=lambda name: NS(value='color_demo'))
    with pytest.raises(ValueError, match='requires segmentation_backend=sam3'):
        PerceptionStages.segment_target(state, 'blue cup')
