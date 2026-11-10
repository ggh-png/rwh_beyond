"""SAM RPC adapter: no torch, credentials, color fallback, or ROS motion APIs."""
import numpy as np
from scipy.ndimage import binary_erosion
from sensor_msgs.msg import Image
from .depth_converter import stamp_seconds
from .zmq_client import request, ProtocolError

def rgb_array(image, depth, sync_tolerance=0.1):
    if image.encoding not in ('rgb8','bgr8') or (image.width,image.height)!=(depth.width,depth.height):
        raise ValueError('registered rgb8/bgr8 image must match depth dimensions')
    if image.header.frame_id!=depth.header.frame_id or abs(stamp_seconds(image.header)-stamp_seconds(depth.header))>sync_tolerance:
        raise ValueError('RGB/depth frame or timestamp mismatch')
    if not image.header.frame_id or not 0<image.width*image.height<=2073600 or image.step<image.width*3 or len(image.data)!=image.step*image.height:
        raise ValueError('invalid RGB image size/stride/data')
    rgb=np.ndarray((image.height,image.width,3),np.uint8,buffer=bytes(image.data),strides=(image.step,3,1))
    if image.encoding=='bgr8':rgb=rgb[:,:,::-1]
    return np.ascontiguousarray(rgb)

def segment_image(endpoint, rgb, depth, text='', box=(), point=(), policy='unique', timeout=60., canceled=lambda:False, allow_mock=False):
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
