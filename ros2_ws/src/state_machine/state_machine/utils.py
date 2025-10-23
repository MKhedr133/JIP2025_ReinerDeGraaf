import math
from geometry_msgs.msg import Quaternion

def yaw_to_quat(yaw_rad: float) -> Quaternion:
    q = Quaternion()
    q.z = math.sin(yaw_rad / 2.0)
    q.w = math.cos(yaw_rad / 2.0)
    return q

def clamp(x, lo, hi):
    return max(lo, min(hi, x))
