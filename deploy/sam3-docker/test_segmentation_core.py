import numpy as np
import pytest
from segmentation_core import select_instance, validate_request

def test_requires_explicit_instance_selection_when_ambiguous():
    masks=np.zeros((2,20,20),bool);masks[0,:8,:8]=True;masks[1,12:,12:]=True
    with pytest.raises(ValueError,match='AMBIGUOUS_OBJECT'):select_instance(masks,np.array([.9,.8]),[],'unique')
    mask,score,count=select_instance(masks,np.array([.9,.8]),[15,15],'unique')
    assert score==.8 and count==2 and mask[15,15] and not mask[0,0]

def test_no_object_and_explicit_score_policy():
    with pytest.raises(ValueError,match='NO_OBJECT'):select_instance(np.zeros((0,20,20)),np.array([]),[],'unique')
    masks=np.ones((2,20,20),bool)
    assert select_instance(masks,np.array([.8,.9]),[],'highest_score')[1]==.9

def test_pixel_box_and_selection_bounds():
    payload=dict(rgb=np.zeros((20,30,3),np.uint8),text='box',box=[1,2,20,18],selection_point=[5,6])
    assert validate_request(payload)[0].shape==(20,30,3)
    for key,value in [('box',[0,0,31,20]),('selection_point',[-1,0]),('rgb',np.zeros((3,20,30))),('selection_policy','first')]:
        with pytest.raises(ValueError):validate_request(dict(payload,**{key:value}))
