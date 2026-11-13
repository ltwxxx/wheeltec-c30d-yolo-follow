"""Laser geometry in camera optical coordinates (x right, y down, z forward)."""
import numpy as np
from follow_settings import SETTINGS


def summarize_points(points, beam_indices, fx, fy, cx, cy):
    """Keep adjacent beams together; split spatial discontinuities, including leg gaps."""
    if not len(points):
        return [], None, None, []
    points = np.asarray(points, dtype=float)
    offset = SETTINGS.camera_to_front_m or 0.0
    corridor = (points[:, 2] > 0) & (np.abs(points[:, 0]) <= SETTINGS.body_half_width_m + SETTINGS.corridor_margin_m)
    nearest = float(points[corridor, 2].min() - offset) if corridor.any() else None
    splits = np.flatnonzero((np.diff(beam_indices) > 1) |
                           (np.linalg.norm(np.diff(points, axis=0), axis=1) > .16)) + 1
    clusters = []
    confirmed_front = []
    for group in np.split(points, splits):
        front = group[(group[:, 2] > 0) & (np.abs(group[:, 0]) <= SETTINGS.body_half_width_m + SETTINGS.corridor_margin_m)]
        if len(front) >= 2:
            confirmed_front.append(float(front[:, 2].min() - offset))
        visible = group[group[:, 2] > .03]
        if len(visible) < 3:
            continue
        x, y, z = visible.T
        u, v = fx * x / z + cx, fy * y / z + cy
        if not np.any((u >= 0) & (u <= 1) & (v >= 0) & (v <= 1)):
            continue
        clusters.append({"u": float(np.median(u)), "v": float(np.median(v)),
                         "u_min": float(u.min()), "u_max": float(u.max()),
                         "distance_m": float(np.percentile(z, 25)),
                         "width_m": float(np.ptp(x)), "count": len(visible),
                         "spread_m": float(np.ptp(z))})
    front = min(confirmed_front, default=None)
    visible = points[points[:, 2] > .03]
    x, y, z = visible.T
    u, v = fx * x / z + cx, fy * y / z + cy
    keep = (u >= 0) & (u <= 1) & (v >= 0) & (v <= 1)
    projected = np.round(np.column_stack((u[keep], v[keep], z[keep])), 4).tolist()
    return projected, front, nearest, clusters
