import numpy as np
from sensor_msgs_py import point_cloud2

def cloud_xyz(cloud, min_points=100, max_points=200000):
    if not cloud.header.frame_id.strip():
        raise ValueError('object_cloud.header.frame_id is required')
    if cloud.width * cloud.height > max_points or len(cloud.data) > 32 * 1024 * 1024:
        raise ValueError('cloud exceeds input size limit; segment/downsample upstream')
    if len(cloud.data) != cloud.row_step * cloud.height:
        raise ValueError('PointCloud2 data/row_step/height mismatch')
    # Jazzy read_points handles field offsets/endian; handle padded rows explicitly.
    fields = {f.name: f for f in cloud.fields}
    if not all(name in fields and fields[name].count == 1 and fields[name].datatype in (7,8)
               for name in ('x','y','z')):
        raise ValueError('scalar FLOAT32/FLOAT64 x,y,z fields required')
    if cloud.point_step <= 0 or cloud.row_step < cloud.width * cloud.point_step:
        raise ValueError('invalid PointCloud2 strides')
    dtype = point_cloud2.dtype_from_fields(cloud.fields, point_step=cloud.point_step)
    if any(fields[n].offset < 0 or fields[n].offset + (4 if fields[n].datatype == 7 else 8) > cloud.point_step for n in ('x','y','z')):
        raise ValueError('XYZ offset exceeds point_step')
    dtype = dtype.newbyteorder('>' if cloud.is_bigendian else '<')
    structured = np.ndarray((cloud.height, cloud.width), dtype=dtype,
                           buffer=bytes(cloud.data), strides=(cloud.row_step, cloud.point_step))
    xyz = np.stack([structured[n].reshape(-1) for n in ('x','y','z')], axis=1).astype(np.float32)
    xyz = xyz[np.isfinite(xyz).all(axis=1)]
    if len(xyz) < min_points:
        raise ValueError(f'at least {min_points} finite object points required')
    return np.ascontiguousarray(xyz)
