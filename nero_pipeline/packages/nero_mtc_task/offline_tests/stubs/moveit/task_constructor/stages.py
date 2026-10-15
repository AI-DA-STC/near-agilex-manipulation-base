from registry import Rec
class PoseDimension:
    ROLL = "ROLL"; PITCH = "PITCH"; YAW = "YAW"
def _mk(kind):
    def f(*a, **k): return Rec(kind, *a, **k)
    return f
for _n in ("CurrentState", "Connect", "GeneratePose", "ComputeIK", "MoveTo", "MoveRelative", "ModifyPlanningScene", "GenerateRandomPose"):
    globals()[_n] = _mk(_n)
