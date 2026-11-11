"""SAM RPC adapter: no torch, credentials, color fallback, or ROS motion APIs."""
import numpy as np
from scipy.ndimage import binary_erosion
from sensor_msgs.msg import Image
from rwh_beyond_model_utils.depth_converter import stamp_seconds
from rwh_beyond_model_utils.zmq_client import request, ProtocolError

from rwh_beyond_model_utils.rgb_converter import rgb_array

def segment_image(endpoint, rgb, depth=None, text='', box=(), point=(), policy='unique', timeout=60., canceled=lambda:False, allow_mock=False):
    if depth is None:
        depth = rgb  # RGB-only SAM Action retains the original image header.
    array=rgb_array(rgb,depth)
    result=request(endpoint,dict(action='segment',rgb=array,text=text,box=list(box),selection_point=list(point),selection_policy=policy),timeout,canceled)
    if result.get('backend')!='sam3' or result.get('mock') is not False:
        if not (allow_mock and result.get('backend')=='mock' and result.get('mock') is True):
            raise ProtocolError('Real SAM 3 segmentation required; no automatic fallback')
    mask=np.asarray(result.get('mask'))
    score=result.get('score');count=result.get('instance_count')
    if mask.dtype!=np.uint8 or mask.shape!=array.shape[:2] or not np.isin(mask,[0,1]).all():
        raise ProtocolError('invalid binary segmentation mask')
    if not isinstance(score,(int,float)) or not np.isfinite(score) or not 0<=score<=1 or type(count) is not int or count<1:
        raise ProtocolError('invalid segmentation score/count')
    # Avoid depth edge contamination; the raw mask is retained for diagnostics.
    labels=binary_erosion(mask,structure=np.ones((3,3)),iterations=1).astype(np.uint8)
    if not labels.any():raise ValueError('segmentation mask empty after depth edge erosion')
    message=Image(header=depth.header,height=depth.height,width=depth.width,encoding='mono8',step=depth.width,data=labels.tobytes())
    return message,float(score),count,bool(result['mock'])
