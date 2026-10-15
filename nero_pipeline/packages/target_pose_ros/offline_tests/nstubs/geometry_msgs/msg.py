class _N: pass
class _Pos: x = y = z = 0.0
class _Ori: x = y = z = 0.0; w = 1.0
class _Pose:
    def __init__(self): self.position = _Pos(); self.orientation = _Ori()
class _Hdr:
    frame_id = ""; stamp = None
class PoseStamped:
    def __init__(self): self.header = _Hdr(); self.pose = _Pose()
