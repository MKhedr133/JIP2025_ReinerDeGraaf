# Small helpers shared across nodes.

import math
from geometry_msgs.msg import Quaternion

def yaw_to_quat(yaw_rad: float) -> Quaternion:
    """Convert a planar yaw angle (radians) into a Quaternion (z,w fields)."""
    q = Quaternion()
    q.z = math.sin(yaw_rad / 2.0)
    q.w = math.cos(yaw_rad / 2.0)
    return q
