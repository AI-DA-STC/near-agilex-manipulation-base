import os, numpy as np
_CALLS = [0]
class GraspEstimator:
    def __init__(self, cfg): self.model = object(); self.cfg = cfg
    def extract_point_clouds(self, depth, K, segmap=None, rgb=None, skip_border_objects=False, z_range=None):
        ys, xs = np.nonzero(segmap > 0)
        if len(xs) == 0: return np.zeros((10, 3)), {}, None
        z = depth[ys, xs]
        pts = np.stack([(xs - K[0, 2]) * z / K[0, 0], (ys - K[1, 2]) * z / K[1, 1], z], 1)
        return np.zeros((10, 3)), {1: pts}, None
    def predict_scene_grasps(self, pc_full, pc_segments=None, local_regions=True, filter_grasps=True, forward_passes=1):
        _CALLS[0] += 1
        flag = os.environ.get("STUB_CRASH_FLAG")
        if flag and os.path.exists(flag): os._exit(3)                       # the process dies mid-request
        if _CALLS[0] <= int(os.environ.get("STUB_EMPTY_FIRST", "0")): return {}, {}, {}, {}   # empty result, like a bad random draw
        if not pc_segments: return {}, {}, {}, {}
        c = pc_segments[1].mean(0); poses = []
        for i in range(3):
            a = 0.3 * i; T = np.eye(4); T[:3, :3] = [[np.cos(a), -np.sin(a), 0], [np.sin(a), np.cos(a), 0], [0, 0, 1]]; T[:3, 3] = c + [0.01 * i, 0, -0.02]; poses.append(T)
        return {1: np.array(poses)}, {1: np.array([0.9, 0.8, 0.7])}, {1: np.zeros((3, 3))}, {1: np.array([0.06, 0.05, 0.04])}
