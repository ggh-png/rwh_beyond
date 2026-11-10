"""Numeric-only RPC validation and explicit single-instance selection (no torch)."""
import numpy as np

MAX_PIXELS = 1920 * 1080

def validate_request(payload):
    image = np.asarray(payload.get('rgb'))
    if image.dtype != np.uint8 or image.ndim != 3 or image.shape[2] != 3 or not 0 < image.shape[0]*image.shape[1] <= MAX_PIXELS:
        raise ValueError('rgb must be uint8 HxWx3, at most 2073600 pixels')
    text = payload.get('text', '')
    if not isinstance(text, str) or len(text) > 256:
        raise ValueError('text must be a string of at most 256 characters')
    h, w = image.shape[:2]
    box = np.asarray(payload.get('box', []), dtype=float)
    if box.size:
        if box.shape != (4,) or not np.isfinite(box).all():
            raise ValueError('box must be finite pixel xyxy')
        x0,y0,x1,y1 = box
        if not 0 <= x0 < x1 <= w or not 0 <= y0 < y1 <= h:
            raise ValueError('box outside image')
    if not text.strip() and not box.size:
        raise ValueError('text or a box prompt is required')
    point = np.asarray(payload.get('selection_point', []), dtype=float)
    if point.size and (point.shape != (2,) or not np.isfinite(point).all() or
                       not 0 <= point[0] < w or not 0 <= point[1] < h):
        raise ValueError('selection point outside image')
    policy = payload.get('selection_policy', 'unique')
    if policy not in ('unique', 'highest_score'):
        raise ValueError('selection_policy must be unique or highest_score')
    return image, text, box, point, policy

def select_instance(masks, scores, point, policy):
    masks = np.asarray(masks)
    scores = np.asarray(scores)
    if masks.ndim != 3 or scores.shape != (len(masks),) or not np.isfinite(scores).all() or np.any((scores<0)|(scores>1)):
        raise ValueError('invalid model masks/scores')
    masks = masks.astype(bool)
    valid = np.flatnonzero(masks.any(axis=(1,2)))
    count = len(valid)
    if len(point):
        x,y = map(int, point)
        valid = np.array([i for i in valid if masks[i,y,x]], dtype=int)
    if len(valid) == 0:
        raise ValueError('NO_OBJECT: no matching object at target')
    if len(valid) > 1 and policy == 'unique':
        raise ValueError('AMBIGUOUS_OBJECT: supply a tighter box/selection point or explicitly choose highest_score')
    index = int(valid[np.argmax(scores[valid])])
    return masks[index].astype(np.uint8), float(scores[index]), count
