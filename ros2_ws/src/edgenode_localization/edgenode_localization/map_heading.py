"""True ENU heading into the projected map basis, without site-specific yaw."""
import math


def grid_convergence(projection,longitude,latitude):
    if not all(math.isfinite(v) for v in (longitude,latitude)):
        raise ValueError('finite GPS coordinates required')
    value=projection.get_factors(longitude,latitude).meridian_convergence
    if not math.isfinite(value):raise ValueError('finite map convergence required')
    return math.radians(value)


def grid_yaw(enu_yaw,convergence):
    if not all(math.isfinite(v) for v in (enu_yaw,convergence)):
        raise ValueError('finite heading basis required')
    return math.atan2(math.sin(enu_yaw+convergence),math.cos(enu_yaw+convergence))
