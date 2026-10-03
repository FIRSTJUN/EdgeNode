"""LiDAR geometry in sensor x-forward/y-left/z-up coordinates."""
import math
import numpy as np


def pointcloud_xyz(msg):
    fields = {field.name: field for field in msg.fields}
    if not {'x', 'y', 'z'}.issubset(fields) or msg.width <= 0 or msg.height <= 0:
        return None
    if msg.point_step <= 0 or msg.row_step < msg.width*msg.point_step:
        return None
    if len(msg.data) < msg.row_step*msg.height:
        return None
    endian = '>' if msg.is_bigendian else '<'
    columns = []
    try:
        for name in ('x', 'y', 'z'):
            field = fields[name]
            dtype = {7: 'f4', 8: 'f8'}.get(field.datatype)
            if dtype is None or field.count != 1:
                return None
            dt = np.dtype(endian+dtype)
            if field.offset < 0 or field.offset+dt.itemsize > msg.point_step:
                return None
            column = np.ndarray((msg.height, msg.width), dtype=dt, buffer=msg.data,
                                offset=field.offset, strides=(msg.row_step, msg.point_step))
            columns.append(column.reshape(-1).astype(np.float32, copy=False))
    except (ValueError, TypeError, BufferError):
        return None
    return np.column_stack(columns)


def corridor_points(points, curvature, half_width=1.05, x_min=.4,
                    x_max=15., z_min=-.35, z_max=1.5):
    p = points[np.isfinite(points).all(axis=1)]
    p = p[(p[:, 0] >= x_min) & (p[:, 0] <= x_max) &
          (p[:, 2] >= z_min) & (p[:, 2] <= z_max)]
    if len(p) == 0:
        return p
    # Distance to a sampled constant-curvature swept vehicle centerline.
    # Unlike a centroid lateral ROI, this follows bends and keeps wall surfaces.
    arc = np.linspace(0, x_max+3., 70)
    k = float(np.clip(curvature, -.45, .45)) if math.isfinite(curvature) else 0.
    if abs(k) < 1e-5:
        path = np.column_stack((arc, np.zeros_like(arc)))
    else:
        path = np.column_stack((np.sin(k*arc)/k, (1-np.cos(k*arc))/k))
    keep = np.zeros(len(p), dtype=bool)
    for begin in range(0, len(p), 2000):
        xy = p[begin:begin+2000, :2]
        distances = np.sum((xy[:, None, :]-path[None, :, :])**2, axis=2)
        keep[begin:begin+len(xy)] = distances.min(axis=1) <= half_width**2
    return p[keep]
