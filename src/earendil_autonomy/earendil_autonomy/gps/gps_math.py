import math

def normalize_heading_deg(angle: float) -> float:
    while angle >= 360.0:
        angle -= 360.0
    while angle < 0.0:
        angle += 360.0
    return angle

def angle_error_deg(target_deg: float, current_deg: float) -> float:
    """
    Result is between -180 and +180.
    Positive error: turn right
    Negative error: turn left
    """
    error = target_deg - current_deg

    while error > 180.0:
        error -= 360.0
    while error < -180.0:
        error += 360.0

    return error

def angle_error_rad(target_rad: float, current_rad: float) -> float:
    """
    Result is between -PI and +PI.
    """
    error = target_rad - current_rad
    return (error + math.pi) % (2 * math.pi) - math.pi

def bearing_between_gps_deg(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
    """
    Calculates the target bearing (in degrees) between two GPS points.
    """
    lat1_rad = math.radians(lat1)
    lat2_rad = math.radians(lat2)
    dlon_rad = math.radians(lon2 - lon1)

    y = math.sin(dlon_rad) * math.cos(lat2_rad)
    x = (
        math.cos(lat1_rad) * math.sin(lat2_rad)
        - math.sin(lat1_rad) * math.cos(lat2_rad) * math.cos(dlon_rad)
    )

    bearing = math.degrees(math.atan2(y, x))
    return normalize_heading_deg(bearing)

def bearing_between_gps_rad(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
    """
    Calculates the target bearing (in radians) between two GPS points.
    """
    return math.radians(bearing_between_gps_deg(lat1, lon1, lat2, lon2))

def haversine(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
    """
    Calculates the distance in meters between two GPS points.
    """
    R = 6371000.0  # Earth radius in meters
    p1, p2 = math.radians(lat1), math.radians(lat2)
    dp = math.radians(lat2 - lat1)
    dl = math.radians(lon2 - lon1)
    a = math.sin(dp/2)**2 + math.cos(p1) * math.cos(p2) * math.sin(dl/2)**2
    return R * 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))
