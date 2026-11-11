"""Validate registered RGB metadata and expose an RGB uint8 array."""
import numpy as np
from .depth_converter import stamp_seconds

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
