from registry import Rec
class Stage:
    class PropertyInitializerSource:
        INTERFACE = "INTERFACE"
def _mk(kind):
    def f(*a, **k): return Rec(kind, *a, **k)
    return f
Task = _mk("Task"); Alternatives = _mk("Alternatives"); Fallbacks = _mk("Fallbacks")
PipelinePlanner = _mk("PipelinePlanner"); CartesianPath = _mk("CartesianPath"); JointInterpolationPlanner = _mk("JointInterpolationPlanner")
