"""Tabletop geometry from measured depth; no completed hidden surfaces or simulator poses."""
import numpy as np
from .depth_converter import depth_xyz
from sensor_msgs.msg import Image

def observed_bounds(xyz, depth, info, mask, base_T_camera, padding=0.003):
    # Use non-target depth for a horizontal support plane in base coordinates.
    background=(np.frombuffer(bytes(mask.data),np.uint8).reshape(mask.height,mask.width)==0).astype(np.uint8)
    other=Image(header=depth.header,height=depth.height,width=depth.width,encoding='mono8',step=depth.width,data=background.tobytes())
    env=depth_xyz(depth,info,other,min_points=100,max_points=20000)
    env=env@base_T_camera[:3,:3].T+base_T_camera[:3,3]
    points=xyz@base_T_camera[:3,:3].T+base_T_camera[:3,3]
    if len(points)<100 or not np.isfinite(points).all():raise ValueError('insufficient finite object points')
    lower,upper=np.quantile(points,[.01,.99],axis=0)
    # A nearby horizontal table is the supported initial scene assumption.
    near=(np.linalg.norm(env[:,:2]-(lower[:2]+upper[:2])/2,axis=1)<0.35)&(env[:,2]<upper[2]-.005)
    z=env[near,2]
    if len(z)<100:raise ValueError('No observed nearby support plane; reobserve')
    bins=np.floor(z/.005).astype(int)
    values,counts=np.unique(bins,return_counts=True)
    dominant=values[np.argmax(counts)]
    plane=z[np.abs(z-(dominant+.5)*.005)<.006]
    if len(plane)<100 or len(plane)<.15*len(z):raise ValueError('Uncertain support plane')
    support=float(np.median(plane))
    lower[2]=support
    dimensions=upper-lower+2*padding
    if np.any(dimensions<.008) or np.any(dimensions>.3) or not .01<upper[2]-support<.3:
        raise ValueError('Observed object bounds outside supported tabletop range')
    center=(lower+upper)/2
    return center,dimensions,support
