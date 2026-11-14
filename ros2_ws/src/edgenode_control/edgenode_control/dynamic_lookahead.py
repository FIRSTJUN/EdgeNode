"""ROS-independent preview distance from measured vehicle speed."""

import math


def calculate_lookahead_distance(measured_speed_kmh, lookahead_min_m=2.5,
                                 lookahead_max_m=10.0,
                                 lookahead_preview_time_sec=0.45):
    """Return the clamped arc lookahead in metres, or None for invalid input.

    Speed is a nonnegative magnitude in km/h, matching EgoVehicleStatus handling
    in ControlNode. Invalid speed/configuration must use the existing safe stop.
    """
    values = (measured_speed_kmh, lookahead_min_m, lookahead_max_m,
              lookahead_preview_time_sec)
    for value in values:
        if isinstance(value, bool) or not isinstance(value, (int, float)):
            return None
        try:
            if not math.isfinite(value):
                return None
        except OverflowError:
            return None
    if (measured_speed_kmh < 0 or not 0 < lookahead_min_m <= lookahead_max_m
            or lookahead_preview_time_sec <= 0):
        return None

    v_mps = float(measured_speed_kmh) / 3.6
    preview_distance_m = v_mps * float(lookahead_preview_time_sec)
    return max(float(lookahead_min_m),
               min(float(lookahead_max_m), preview_distance_m))


def path_lookahead_distance(speed_kmh,points,pose,lookahead_min_m=2.5,
                           lookahead_max_m=10.,lookahead_preview_time_sec=.45,
                           heading_budget_rad=.4):
    """Use the speed preview on straights; bound accumulated turning on curves.

    Shared by the actual PP controller and S4's PP rollout. Low-speed minimum
    lookahead remains identical. This changes the target distance, never geometry.
    """
    distance=calculate_lookahead_distance(speed_kmh,lookahead_min_m,
        lookahead_max_m,lookahead_preview_time_sec)
    if distance is None or not math.isfinite(heading_budget_rad) or heading_budget_rad<=0:return None
    if distance<=lookahead_min_m:return distance
    if len(points)<2 or len(pose)!=3:return None
    best=None;segments=[]
    try:
        for i,(a,b) in enumerate(zip(points,points[1:])):
            dx,dy=b[0]-a[0],b[1]-a[1];length=math.hypot(dx,dy)
            if length<=1e-9:continue
            along=max(0.,min(length,((pose[0]-a[0])*dx+(pose[1]-a[1])*dy)/length))
            error=math.hypot(pose[0]-a[0]-along*dx/length,pose[1]-a[1]-along*dy/length)
            segments.append((i,length,math.atan2(dy,dx)))
            if best is None or error<best[0]:best=(error,i,along)
        if best is None:return None
        station=0.;turn=0.;previous=None
        for index,length,heading in segments:
            if index<best[1]:continue
            if previous is not None:
                turn+=abs((heading-previous+math.pi)%(2*math.pi)-math.pi)
                if turn>heading_budget_rad:return max(lookahead_min_m,min(distance,station))
            station+=length-(best[2] if index==best[1] else 0.)
            if station>=distance:break
            previous=heading
        return distance
    except (TypeError,IndexError,ValueError,OverflowError):return None
