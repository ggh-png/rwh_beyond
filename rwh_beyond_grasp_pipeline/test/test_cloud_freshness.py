"""Empty processed depth frames must satisfy the same freshness gate as nonempty ones."""
from types import SimpleNamespace

from sensor_msgs.msg import PointCloud2
from rwh_beyond_grasp_pipeline.grasp_coordinator import GraspCoordinator


def test_empty_processed_cloud_advances_freshness_after_known_objects_are_masked():
    state = SimpleNamespace(cloud_seq=4, cloud_stamp=1.)
    message = PointCloud2(height=1, width=0)
    message.header.frame_id = 'd405_color_optical_frame'
    message.header.stamp.sec = 2
    GraspCoordinator.cloud_callback(state, message)
    assert state.cloud_seq == 5 and state.cloud_stamp == 2.
    message.header.stamp.sec = 3
    GraspCoordinator.cloud_callback(state, message)
    assert state.cloud_seq >= 6 and state.cloud_stamp > 1.


def test_default_message_does_not_prove_a_processed_frame():
    state = SimpleNamespace(cloud_seq=4, cloud_stamp=1.)
    GraspCoordinator.cloud_callback(state, PointCloud2())
    assert state.cloud_seq == 4 and state.cloud_stamp == 1.
