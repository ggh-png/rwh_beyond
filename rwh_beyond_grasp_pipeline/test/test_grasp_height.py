import numpy as np
import pytest
from rwh_beyond_grasp_pipeline.grasp_height import validate_height_parameters,observed_top_z,in_height_band


@pytest.mark.parametrize('settings', [
 {'grasp_height_filter_enabled':'true'},
 {'grasp_height_above_object_min_m':-.01},
 {'grasp_height_above_object_max_m':.01},
 {'grasp_height_above_object_min_m':float('nan')},
 {'grasp_height_above_object_max_m':float('inf')},
])
def test_invalid_height_settings_are_rejected(settings):
    with pytest.raises(ValueError):validate_height_parameters(settings)


def test_disabled_filter_has_no_band_and_enabled_uses_configured_range():
    assert validate_height_parameters({}) is None
    assert validate_height_parameters({'grasp_height_filter_enabled':True,
      'grasp_height_above_object_min_m':.06,'grasp_height_above_object_max_m':.09})==(.06,.09)


def test_top_is_measured_in_base_z_and_ignores_isolated_depth_outlier():
    points=np.zeros((1000,3));points[:,0]=.2;points[-1,0]=5.
    transform=np.eye(4);transform[:3,:3]=[[0,0,-1],[0,1,0],[1,0,0]];transform[2,3]=.3
    assert observed_top_z(points,transform)==pytest.approx(.5)
    assert in_height_band(.575,observed_top_z(points,transform),(.05,.10))


def test_missing_geometry_does_not_substitute_a_guessed_object_top():
    with pytest.raises(ValueError):observed_top_z(np.zeros((5,3)),np.eye(4))
