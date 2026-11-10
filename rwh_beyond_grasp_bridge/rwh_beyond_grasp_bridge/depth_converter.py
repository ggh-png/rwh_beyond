"""Pinhole depth + registered target mask -> segmented metric XYZ (no TF)."""
import numpy as np


def image_array(image, encodings, max_pixels):
    if image.encoding not in encodings:
        raise ValueError(f'unsupported image encoding: {image.encoding}')
    if not image.width or not image.height or image.width * image.height > max_pixels:
        raise ValueError('image dimensions are empty or exceed max_depth_pixels')
    dtype = np.dtype(encodings[image.encoding]).newbyteorder('>' if image.is_bigendian else '<')
    if image.step < image.width * dtype.itemsize or len(image.data) != image.step * image.height:
        raise ValueError('invalid Image step/data length')
    if len(image.data) > 32 * 1024 * 1024:
        raise ValueError('image exceeds byte limit')
    return np.ndarray((image.height, image.width), dtype=dtype,
                      buffer=bytes(image.data), strides=(image.step, dtype.itemsize))


def stamp_seconds(header):
    return header.stamp.sec + header.stamp.nanosec * 1e-9


def depth_xyz(depth, info, mask, min_points=100, max_points=200000,
              max_pixels=2073600, sync_tolerance=0.1):
    if not depth.header.frame_id.strip():
        raise ValueError('depth optical frame_id required')
    for label, image_header in [('camera_info', info.header), ('object_mask', mask.header)]:
        if image_header.frame_id != depth.header.frame_id:
            raise ValueError(f'{label} must be registered in the depth optical frame')
        # CameraInfo stamp zero denotes static calibration, not a synchronized image.
        if label == 'camera_info' and stamp_seconds(image_header) == 0:
            continue
        if abs(stamp_seconds(image_header) - stamp_seconds(depth.header)) > sync_tolerance:
            raise ValueError(f'{label} timestamp differs from depth beyond tolerance')
    if (info.width, info.height) != (depth.width, depth.height):
        raise ValueError('CameraInfo dimensions must match depth')
    if info.binning_x > 1 or info.binning_y > 1 or info.roi.x_offset or info.roi.y_offset:
        raise ValueError('provide full-size calibrated depth without binning/ROI offsets')
    if not np.isfinite(info.d).all() or np.any(np.abs(info.d) > 1e-8):
        raise ValueError('nonzero distortion: provide undistorted depth with its matching pinhole calibration')
    k = np.asarray(info.k, dtype=float).reshape(3,3)
    if (not np.isfinite(k).all() or k[0,0] <= 0 or k[1,1] <= 0 or
        not np.allclose(k[2], [0,0,1]) or abs(k[1,0]) > 1e-8):
        raise ValueError('invalid pinhole camera matrix K')
    values = image_array(depth, {'16UC1':'u2', '32FC1':'f4'}, max_pixels)
    labels = image_array(mask, {'mono8':'u1', '8UC1':'u1'}, max_pixels)
    if labels.shape != values.shape:
        raise ValueError('object_mask dimensions must match depth')
    z = values.astype(np.float32)
    if depth.encoding == '16UC1':
        z *= 0.001
    rows, cols = np.nonzero((labels != 0) & np.isfinite(z) & (z > 0))
    if len(rows) < min_points:
        raise ValueError(f'at least {min_points} finite positive target depth pixels required')
    if len(rows) > max_points:
        index = np.linspace(0, len(rows)-1, max_points, dtype=int)
        rows, cols = rows[index], cols[index]
    pixels = np.stack([cols, rows, np.ones_like(rows)], axis=1)
    rays = pixels @ np.linalg.inv(k).T
    xyz = rays * z[rows, cols, None]
    with np.errstate(over='ignore', invalid='ignore'):
        xyz = np.ascontiguousarray(xyz, dtype=np.float32)
    if not np.isfinite(xyz).all():
        raise ValueError('depth/calibration produced nonfinite XYZ')
    return xyz
