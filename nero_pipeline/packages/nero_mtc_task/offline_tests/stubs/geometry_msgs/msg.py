class _N:
    def __init__(self, **kw): self.__dict__.update(kw)
class Point(_N):
    def __init__(self, x=0.0, y=0.0, z=0.0): self.x, self.y, self.z = x, y, z
class Quaternion(_N):
    def __init__(self, x=0.0, y=0.0, z=0.0, w=1.0): self.x, self.y, self.z, self.w = x, y, z, w
class Vector3(Point): pass
class Header(_N):
    def __init__(self, frame_id=""): self.frame_id = frame_id
class Pose(_N):
    def __init__(self, position=None, orientation=None): self.position = position or Point(); self.orientation = orientation or Quaternion()
class PoseStamped(_N):
    def __init__(self, header=None, pose=None): self.header = header or Header(); self.pose = pose or Pose()
class PoseArray(_N):
    def __init__(self, header=None, poses=None): self.header = header or Header(); self.poses = poses or []
class Twist(_N):
    def __init__(self): self.linear = Vector3(); self.angular = Vector3()
class TwistStamped(_N):
    def __init__(self): self.header = Header(); self.twist = Twist()
